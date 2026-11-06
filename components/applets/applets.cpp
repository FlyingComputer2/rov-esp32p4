#include "applets.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <thread>
#include <vector>

#include "apps/bwtest/Bwtest.h"
#include "endpoint/UpstreamQueue.h"
#include "esp_log.h"

namespace applets
{
namespace
{

const char* TAG = "applets";

// At most this many datagrams per socket or queue per reactor pass, so one
// busy source cannot hold the reactor (it comes back for the rest).
constexpr size_t k_max_batch = 16;

using queue_registry = bfc::cv_reactor_circular_lossy_queue_registry<bfc::shared_sized_buffer>;
using handler_t = std::function<std::string(const std::string& line)>;

bool find_reactor(const std::string& name, std::shared_ptr<reactor_t>* out, const char* who)
{
    *out = bfc::default_reactor_registry::get_instance().get(name);
    if (!*out)
    {
        ESP_LOGE(TAG, "%s: no reactor %s", who, name.c_str());
    }
    return bool(*out);
}

bool find_queues(const upstream_names& names, std::shared_ptr<queue_t>* to, std::shared_ptr<queue_t>* from)
{
    *to = queue_registry::get_instance().get(names.to_l3);
    *from = queue_registry::get_instance().get(names.from_l3);
    if (!*to || !*from)
    {
        ESP_LOGE(TAG, "no queues %s / %s", names.to_l3.c_str(), names.from_l3.c_str());
        return false;
    }
    return true;
}

// Empty names = no m-plane; otherwise its queues must exist.
bool find_mplane(const upstream_names& names, std::shared_ptr<queue_t>* to, std::shared_ptr<queue_t>* from)
{
    return names.to_l3.empty() || find_queues(names, to, from);
}

void push(queue_t& q, const void* data, size_t len)
{
    q.push(bfc::shared_sized_buffer::copy_from(static_cast<const uint8_t*>(data), len));
}

bool pop(queue_t& q, std::vector<uint8_t>* out)
{
    auto packet = q.pop();
    if (!packet)
    {
        return false;
    }
    const uint8_t* p = reinterpret_cast<const uint8_t*>(packet->data());
    out->assign(p, p + packet->size());
    return true;
}

krono::Transport transport(std::shared_ptr<queue_t> to, std::shared_ptr<queue_t> from)
{
    return krono::Transport {[to](const uint8_t* data, size_t len)
                             {
                                 push(*to, data, len);
                                 return true;
                             },
                             [from](std::vector<uint8_t>* out) { return pop(*from, out); }};
}

// The applets' UDP console: one command per datagram, the reply to its
// sender, up to k_max_batch per pass.
bool serve_udp(reactor_t& r, const std::string& host_port, handler_t handle)
{
    const size_t colon = host_port.rfind(':');
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(std::atoi(host_port.c_str() + colon + 1)));
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (colon == std::string::npos || addr.sin_port == 0 ||
        inet_pton(AF_INET, host_port.substr(0, colon).c_str(), &addr.sin_addr) != 1 || fd < 0 ||
        bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0)
    {
        ESP_LOGE(TAG, "cannot bind console %s", host_port.c_str());
        if (fd >= 0)
        {
            close(fd);
        }
        return false;
    }
    auto h = std::make_shared<handler_t>(std::move(handle));
    r.add_read_rdy(fd,
                   [fd, h]
                   {
                       for (size_t i = 0; i < k_max_batch; ++i)
                       {
                           char buf[512];
                           sockaddr_in from {};
                           socklen_t from_len = sizeof(from);
                           const ssize_t n = recvfrom(fd, buf, sizeof(buf) - 1, MSG_DONTWAIT,
                                                      reinterpret_cast<sockaddr*>(&from), &from_len);
                           if (n <= 0)
                           {
                               return;
                           }
                           buf[n] = '\0';
                           const std::string reply = (*h)(buf);
                           if (!reply.empty())
                           {
                               sendto(fd, reply.data(), reply.size(), 0, reinterpret_cast<const sockaddr*>(&from),
                                      from_len);
                           }
                       }
                   });
    return true;
}

// The applets' m-plane on an upstream: each request datagram handled, its
// reply pushed back, up to k_max_batch per pass.
void serve_queue(const std::shared_ptr<reactor_t>& r, std::shared_ptr<queue_t> to, std::shared_ptr<queue_t> from,
                 handler_t handle)
{
    auto h = std::make_shared<handler_t>(std::move(handle));
    r->add_read_rdy(*from,
                    [to, from, h]
                    {
                        std::vector<uint8_t> req;
                        for (size_t i = 0; i < k_max_batch && pop(*from, &req); ++i)
                        {
                            const std::string reply = (*h)(std::string(req.begin(), req.end()));
                            if (!reply.empty())
                            {
                                push(*to, reply.data(), reply.size());
                            }
                        }
                    });
    from->set_reactor(r);
}

// esp_pthread_set_cfg() for this scope (threads created meanwhile get cfg),
// then back to the previous one.
class scoped_pthread_cfg
{
public:
    explicit scoped_pthread_cfg(const esp_pthread_cfg_t& cfg) : had_prev_(esp_pthread_get_cfg(&prev_) == ESP_OK)
    {
        esp_pthread_set_cfg(&cfg);
    }
    ~scoped_pthread_cfg()
    {
        const esp_pthread_cfg_t restore = had_prev_ ? prev_ : esp_pthread_get_default_config();
        esp_pthread_set_cfg(&restore);
    }

private:
    esp_pthread_cfg_t prev_ {};
    bool had_prev_;
};

}  // namespace

bool BwtestApplet::configure(const bwtest_config& cfg)
{
    cfg_ = cfg;
    return find_reactor(cfg.reactor, &reactor_, "bwtest") && find_queues(cfg.data, &to_l3_, &from_l3_) &&
           find_mplane(cfg.mplane, &mp_to_l3_, &mp_from_l3_);
}

bool BwtestApplet::start()
{
    if (!reactor_ || !from_l3_)
    {
        return false;
    }
    /* Lives for the program: the callbacks hold it. app_bwtest takes an
     * UpstreamQueue: built on the named queues (ingress = to L3). */
    auto up = std::make_shared<winject::UpstreamQueue>(to_l3_, from_l3_);
    auto bw = std::make_shared<app_bwtest::Bwtest>(up, cfg_.payload_bytes);
    /* test_send creates its sender from the handler: set its thread settings
     * there (the reactor's do not pass on). */
    handler_t handle = [bw, sender = cfg_.sender](const std::string& line)
    {
        const scoped_pthread_cfg scope(sender);
        return bw->handle_command(line);
    };
    if (!serve_udp(*reactor_, cfg_.console, handle))
    {
        return false;
    }
    if (mp_from_l3_)
    {
        serve_queue(reactor_, mp_to_l3_, mp_from_l3_, handle);
    }
    /* Received test traffic, as it arrives (poll_receive takes all queued). */
    reactor_->add_read_rdy(*from_l3_, [bw] { bw->poll_receive(); });
    from_l3_->set_reactor(reactor_);
    printf("bwtest: console %s%s, payload %u (reactor %s)\n", cfg_.console.c_str(),
           mp_from_l3_ ? " and m-plane upstream" : "", (unsigned)cfg_.payload_bytes, cfg_.reactor.c_str());
    return true;
}

bool SynchClientApplet::configure(const synch_client_config& cfg)
{
    cfg_ = cfg;
    return find_reactor(cfg.reactor, &reactor_, "synch") && find_queues(cfg.data, &to_l3_, &from_l3_) &&
           find_mplane(cfg.mplane, &mp_to_l3_, &mp_from_l3_);
}

bool SynchClientApplet::start()
{
    if (!reactor_ || !from_l3_)
    {
        return false;
    }
    /* Lives for the program: the worker and the callbacks hold it. */
    auto client = std::make_shared<krono::SynchClient>(transport(to_l3_, from_l3_), cfg_.options);
    handler_t handle = [client](const std::string& line) { return client->handle_command(line); };
    if (!serve_udp(*reactor_, cfg_.console, handle))
    {
        return false;
    }
    if (mp_from_l3_)
    {
        serve_queue(reactor_, mp_to_l3_, mp_from_l3_, handle);
    }
    const uint32_t period_ms = cfg_.period_ms;
    {
        const scoped_pthread_cfg scope(cfg_.worker);
        std::thread(
            [client, period_ms]
            {
                auto next = std::chrono::steady_clock::now();
                for (;;)
                {
                    client->step_once();
                    next += std::chrono::milliseconds(period_ms);
                    std::this_thread::sleep_until(next);
                }
            })
            .detach();
    }
    printf("synch: client console %s%s, mode %s, period %u ms (reactor %s)\n", cfg_.console.c_str(),
           mp_from_l3_ ? " and m-plane upstream" : "", krono::mode_name(cfg_.options.mode), (unsigned)period_ms,
           cfg_.reactor.c_str());
    return true;
}

bool SynchServerApplet::configure(const synch_server_config& cfg)
{
    cfg_ = cfg;
    return find_reactor(cfg.reactor, &reactor_, "synch") && find_queues(cfg.data, &to_l3_, &from_l3_);
}

bool SynchServerApplet::start()
{
    if (!reactor_ || !from_l3_)
    {
        return false;
    }
    /* Lives for the program: the callbacks hold it. */
    auto server = std::make_shared<krono::SynchServer>(transport(to_l3_, from_l3_), cfg_.edges);
    if (!serve_udp(*reactor_, cfg_.console, [server](const std::string& line) { return server->handle_command(line); }))
    {
        return false;
    }
    reactor_->add_read_rdy(*from_l3_, [server] { server->poll(); });
    from_l3_->set_reactor(reactor_);
    printf("synch: server console %s, answers on arrival (reactor %s)\n", cfg_.console.c_str(),
           cfg_.reactor.c_str());
    return true;
}

}  // namespace applets
