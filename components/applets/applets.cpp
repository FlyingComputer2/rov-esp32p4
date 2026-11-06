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

}  // namespace

bool BwtestApplet::configure(std::shared_ptr<winject::UpstreamQueue> up, const bwtest_config& cfg)
{
    cfg_ = cfg;
    up_ = std::move(up);
    reactor_ = app_kit::find_reactor(cfg.reactor);
    if (!up_ || !reactor_)
    {
        ESP_LOGE(TAG, "bwtest: no reactor %s or no queue", cfg.reactor.c_str());
        return false;
    }
    return true;
}

void BwtestApplet::set_mplane(app_kit::datagrams link)
{
    mplane_ = std::move(link);
    if (handle_ && app_kit::queue_console(*reactor_, mplane_, handle_, cfg_.poll_ms))
    {
        printf("bwtest: m-plane on its upstream\n");
    }
}

bool BwtestApplet::start()
{
    if (!reactor_)
    {
        return false;
    }
    /* Lives for the program: the callbacks hold it. */
    auto bw = std::make_shared<app_bwtest::Bwtest>(up_, cfg_.payload_bytes);
    /* test_send creates its sender from the handler: set its thread settings
     * there (the reactor's do not pass on). */
    handle_ = [bw, sender = cfg_.sender](const std::string& line)
    {
        const app_kit::scoped_pthread_cfg scope(sender);
        return bw->handle_command(line);
    };
    if (!app_kit::udp_console(*reactor_, cfg_.console, handle_))
    {
        handle_ = {};
        return false;
    }
    const bool mplane = app_kit::queue_console(*reactor_, mplane_, handle_, cfg_.poll_ms);
    app_kit::every(*reactor_, cfg_.poll_ms, [bw] { bw->poll_receive(); });
    printf("bwtest: console %s%s, payload %u (reactor %s)\n", cfg_.console.c_str(),
           mplane ? " and its upstream" : "", (unsigned)cfg_.payload_bytes, cfg_.reactor.c_str());
    return true;
}

bool SynchClientApplet::configure(krono::Transport transport, const synch_client_config& cfg)
{
    cfg_ = cfg;
    transport_ = std::move(transport);
    reactor_ = app_kit::find_reactor(cfg.reactor);
    if (!reactor_)
    {
        ESP_LOGE(TAG, "synch: no reactor %s", cfg.reactor.c_str());
        return false;
    }
    return true;
}

void SynchClientApplet::set_mplane(app_kit::datagrams link)
{
    mplane_ = std::move(link);
    if (handle_ && app_kit::queue_console(*reactor_, mplane_, handle_))
    {
        printf("synch: m-plane on its upstream\n");
    }
}

bool SynchClientApplet::start()
{
    if (!reactor_)
    {
        return false;
    }
    /* Lives for the program: the worker and the callbacks hold it. */
    auto client = std::make_shared<krono::SynchClient>(transport_, cfg_.options);
    handle_ = [client](const std::string& line) { return client->handle_command(line); };
    if (!app_kit::udp_console(*reactor_, cfg_.console, handle_))
    {
        handle_ = {};
        return false;
    }
    const bool mplane = app_kit::queue_console(*reactor_, mplane_, handle_);
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
           mplane ? " and its upstream" : "", krono::mode_name(cfg_.options.mode), (unsigned)period_ms,
           cfg_.reactor.c_str());
    return true;
}

bool SynchServerApplet::configure(krono::Transport transport, const synch_server_config& cfg)
{
    cfg_ = cfg;
    transport_ = std::move(transport);
    reactor_ = app_kit::find_reactor(cfg.reactor);
    if (!reactor_)
    {
        ESP_LOGE(TAG, "synch: no reactor %s", cfg.reactor.c_str());
        return false;
    }
    return true;
}

bool SynchServerApplet::start()
{
    if (!reactor_)
    {
        return false;
    }
    /* Lives for the program: the callbacks hold it. */
    auto server = std::make_shared<krono::SynchServer>(transport_, cfg_.edges);
    if (!app_kit::udp_console(*reactor_, cfg_.console,
                              [server](const std::string& line) { return server->handle_command(line); }))
    {
        return false;
    }
    app_kit::every(*reactor_, cfg_.poll_ms, [server] { server->poll(); });
    printf("synch: server console %s, polled every %u ms (reactor %s)\n", cfg_.console.c_str(),
           (unsigned)cfg_.poll_ms, cfg_.reactor.c_str());
    return true;
}

}  // namespace applets
