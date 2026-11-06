#include "applets.hpp"

#include <chrono>
#include <thread>

#include "apps/bwtest/Bwtest.h"
#include "esp_log.h"

namespace applets
{
namespace
{

const char* TAG = "applets";

bool find(const std::string& reactor, std::shared_ptr<app_kit::reactor_t>* r, const char* who)
{
    *r = app_kit::find_reactor(reactor);
    if (!*r)
    {
        ESP_LOGE(TAG, "%s: no reactor %s", who, reactor.c_str());
    }
    return bool(*r);
}

/* Empty names = no m-plane; otherwise the queues must exist. */
bool find_mplane(const app_kit::upstream_names& names, app_kit::upstream_queues* q, bool* has)
{
    *has = !names.to_l3.empty();
    return !*has || app_kit::find_upstream_queues(names, q);
}

krono::Transport transport(const app_kit::upstream_queues& q)
{
    app_kit::datagrams d = app_kit::link(q);
    return krono::Transport {std::move(d.send), std::move(d.receive)};
}

}  // namespace

bool BwtestApplet::configure(const bwtest_config& cfg)
{
    cfg_ = cfg;
    return find(cfg.reactor, &reactor_, "bwtest") && app_kit::find_upstream_queues(cfg.data, &data_) &&
           find_mplane(cfg.mplane, &mplane_, &has_mplane_);
}

bool BwtestApplet::start()
{
    if (!reactor_ || !data_.from_l3)
    {
        return false;
    }
    /* Lives for the program: the callbacks hold it. app_bwtest takes an
     * UpstreamQueue: built on the named queues (ingress = to L3). */
    auto up = std::make_shared<winject::UpstreamQueue>(data_.to_l3, data_.from_l3);
    auto bw = std::make_shared<app_bwtest::Bwtest>(up, cfg_.payload_bytes);
    /* test_send creates its sender from the handler: set its thread settings
     * there (the reactor's do not pass on). */
    auto handle = [bw, sender = cfg_.sender](const std::string& line)
    {
        const app_kit::scoped_pthread_cfg scope(sender);
        return bw->handle_command(line);
    };
    if (!app_kit::udp_console(*reactor_, cfg_.console, handle))
    {
        return false;
    }
    if (has_mplane_)
    {
        app_kit::queue_console(*reactor_, mplane_, handle);
    }
    app_kit::on_receive(*reactor_, data_, [bw] { bw->poll_receive(); });
    printf("bwtest: console %s%s, payload %u (reactor %s)\n", cfg_.console.c_str(),
           has_mplane_ ? " and m-plane upstream" : "", (unsigned)cfg_.payload_bytes, cfg_.reactor.c_str());
    return true;
}

bool SynchClientApplet::configure(const synch_client_config& cfg)
{
    cfg_ = cfg;
    return find(cfg.reactor, &reactor_, "synch") && app_kit::find_upstream_queues(cfg.data, &data_) &&
           find_mplane(cfg.mplane, &mplane_, &has_mplane_);
}

bool SynchClientApplet::start()
{
    if (!reactor_ || !data_.from_l3)
    {
        return false;
    }
    /* Lives for the program: the worker and the callbacks hold it. */
    auto client = std::make_shared<krono::SynchClient>(transport(data_), cfg_.options);
    auto handle = [client](const std::string& line) { return client->handle_command(line); };
    if (!app_kit::udp_console(*reactor_, cfg_.console, handle))
    {
        return false;
    }
    if (has_mplane_)
    {
        app_kit::queue_console(*reactor_, mplane_, handle);
    }
    const uint32_t period_ms = cfg_.period_ms;
    app_kit::spawn(cfg_.worker,
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
    printf("synch: client console %s%s, mode %s, period %u ms (reactor %s)\n", cfg_.console.c_str(),
           has_mplane_ ? " and m-plane upstream" : "", krono::mode_name(cfg_.options.mode), (unsigned)period_ms,
           cfg_.reactor.c_str());
    return true;
}

bool SynchServerApplet::configure(const synch_server_config& cfg)
{
    cfg_ = cfg;
    return find(cfg.reactor, &reactor_, "synch") && app_kit::find_upstream_queues(cfg.data, &data_);
}

bool SynchServerApplet::start()
{
    if (!reactor_ || !data_.from_l3)
    {
        return false;
    }
    /* Lives for the program: the callbacks hold it. */
    auto server = std::make_shared<krono::SynchServer>(transport(data_), cfg_.edges);
    if (!app_kit::udp_console(*reactor_, cfg_.console,
                              [server](const std::string& line) { return server->handle_command(line); }))
    {
        return false;
    }
    app_kit::on_receive(*reactor_, data_, [server] { server->poll(); });
    printf("synch: server console %s, answers on arrival (reactor %s)\n", cfg_.console.c_str(),
           cfg_.reactor.c_str());
    return true;
}

}  // namespace applets
