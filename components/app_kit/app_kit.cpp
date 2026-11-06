#include "app_kit.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace app_kit
{
namespace
{

const char* TAG = "app_kit";

void arm(reactor_t& r, uint32_t period_ms, const std::shared_ptr<std::function<void()>>& fn)
{
    reactor_t* rp = &r;
    r.get_timer().wait_ms(period_ms,
                          [rp, period_ms, fn]
                          {
                              (*fn)();
                              arm(*rp, period_ms, fn);
                          });
}

}  // namespace

esp_pthread_cfg_t pthread_cfg(const thread_spec& spec)
{
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.thread_name = spec.name;
    cfg.stack_size = spec.stack_bytes;
    cfg.prio = spec.prio;
    cfg.pin_to_core = spec.core < 0 ? tskNO_AFFINITY : spec.core;
    cfg.inherit_cfg = spec.inherit;
    cfg.stack_alloc_caps = spec.stack == stack_in::psram ? MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
                                                         : MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    return cfg;
}

scoped_pthread_cfg::scoped_pthread_cfg(const esp_pthread_cfg_t& cfg)
    : had_prev(esp_pthread_get_cfg(&prev) == ESP_OK)
{
    esp_pthread_set_cfg(&cfg);
}

scoped_pthread_cfg::~scoped_pthread_cfg()
{
    if (had_prev)
    {
        esp_pthread_set_cfg(&prev);
    }
    else
    {
        const esp_pthread_cfg_t def = esp_pthread_get_default_config();
        esp_pthread_set_cfg(&def);
    }
}

std::thread spawn(const thread_spec& spec, std::function<void()> fn)
{
    const scoped_pthread_cfg scope(spec);
    return std::thread(std::move(fn));
}

bool start_reactor(const std::string& name, const thread_spec& thread)
{
    auto r = std::make_shared<reactor_t>();
    if (!reactor_registry::get_instance().add(name, r))
    {
        ESP_LOGE(TAG, "reactor %s already registered", name.c_str());
        return false;
    }
    spawn(thread, [r] { r->run(); }).detach();
    return true;
}

std::shared_ptr<reactor_t> find_reactor(const std::string& name)
{
    return reactor_registry::get_instance().get(name);
}

void post(reactor_t& r, std::function<void()> fn)
{
    auto boxed = std::make_shared<std::function<void()>>(std::move(fn));
    r.wake_up([boxed] { (*boxed)(); });
}

void every(reactor_t& r, uint32_t period_ms, std::function<void()> fn)
{
    auto boxed = std::make_shared<std::function<void()>>(std::move(fn));
    reactor_t* rp = &r;
    /* Armed on the reactor thread, so its wait picks the new deadline up. */
    r.wake_up([rp, period_ms, boxed] { arm(*rp, period_ms, boxed); });
}

bool parse_host_port(const std::string& s, void* sockaddr_in_out)
{
    auto* out = static_cast<sockaddr_in*>(sockaddr_in_out);
    const size_t colon = s.rfind(':');
    if (colon == std::string::npos)
    {
        return false;
    }
    char* end = nullptr;
    const long port = std::strtol(s.c_str() + colon + 1, &end, 10);
    if (*end != '\0' || port <= 0 || port > 65535)
    {
        return false;
    }
    std::memset(out, 0, sizeof(*out));
    out->sin_family = AF_INET;
    out->sin_port = htons(static_cast<uint16_t>(port));
    return inet_pton(AF_INET, s.substr(0, colon).c_str(), &out->sin_addr) == 1;
}

void watch(reactor_t& r, int fd, std::function<void()> fn)
{
    auto boxed = std::make_shared<std::function<void()>>(std::move(fn));
    reactor_t* rp = &r;
    post(r, [rp, fd, boxed] { rp->add_read_rdy(fd, [boxed] { (*boxed)(); }); });
}

bool queue_console(reactor_t& r, datagrams link, std::function<std::string(const std::string& line)> handle,
                   uint32_t poll_ms, size_t max_batch)
{
    if (!link.send || !link.receive || !handle)
    {
        return false;
    }
    every(r, poll_ms,
                 [link = std::move(link), handle = std::move(handle), max_batch]
                 {
                     std::vector<uint8_t> req;
                     for (size_t i = 0; i < max_batch && link.receive(&req); ++i)
                     {
                         const std::string reply = handle(std::string(req.begin(), req.end()));
                         if (!reply.empty())
                         {
                             link.send(reinterpret_cast<const uint8_t*>(reply.data()), reply.size());
                         }
                     }
                 });
    return true;
}

bool udp_console(reactor_t& r, const std::string& host_port,
                 std::function<std::string(const std::string& line)> handle)
{
    sockaddr_in addr {};
    if (!parse_host_port(host_port, &addr))
    {
        ESP_LOGE(TAG, "bad console address %s", host_port.c_str());
        return false;
    }
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        return false;
    }
    if (bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0)
    {
        ESP_LOGE(TAG, "cannot bind console %s", host_port.c_str());
        close(fd);
        return false;
    }
    watch(r, fd,
                 [fd, handle = std::move(handle)]
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
                         const std::string reply = handle(buf);
                         if (!reply.empty())
                         {
                             sendto(fd, reply.data(), reply.size(), 0, reinterpret_cast<const sockaddr*>(&from),
                                    from_len);
                         }
                     }
                 });
    return true;
}

}  // namespace app_kit
