#include "mplane_relay_applet.hpp"

#include <atomic>
#include <memory>

#include "esp_log.h"
#include "esp_timer.h"

namespace mplane_relay
{
struct Applet::Impl
{
    struct State
    {
        std::atomic<bool> running {false};
        std::unique_ptr<Client> client;
        app_kit::cancel_fn poll;
        app_kit::cancel_fn expiry;
    };
    AppletConfig cfg;
    Transport transport;
    std::shared_ptr<app_kit::reactor_t> reactor;
    std::shared_ptr<State> state = std::make_shared<State>();
    bool configured = false;
};

Applet::Applet() : impl_(std::make_unique<Impl>()) {}
Applet::~Applet() { stop(); }

bool Applet::configure(const AppletConfig& cfg, Transport transport)
{
    stop();
    impl_->cfg = cfg;
    impl_->transport = std::move(transport);
    impl_->reactor = app_kit::find_reactor(cfg.reactor);
    impl_->configured = impl_->reactor && impl_->transport.send && impl_->transport.receive;
    if (!impl_->configured)
        ESP_LOGE("mplane_relay", "%s: no reactor %s or no transport", cfg.name.c_str(), cfg.reactor.c_str());
    return impl_->configured;
}

bool Applet::start()
{
    if (!impl_->configured || impl_->state->running.exchange(true)) return false;
    auto r = impl_->reactor;
    auto cfg = impl_->cfg;
    auto transport = impl_->transport;
    auto state = impl_->state;
    app_kit::post(*r, [r, cfg, transport, state]
    {
        if (!state->running.load()) return;
        app_kit::reactor_t* rp = r.get();
        Hooks hooks;
        hooks.send_air = transport.send;
        hooks.now_ms = [] { return esp_timer_get_time() / 1000; };
        hooks.watch = [rp, state](int fd)
        {
            rp->add_read_rdy(fd, [state, fd]
            {
                if (state->running.load() && state->client) state->client->on_readable(fd);
            });
        };
        hooks.unwatch = [rp](int fd) { rp->rem_read_rdy(fd); };
        state->client = std::make_unique<Client>(cfg.relay, hooks);
        if (!state->client->ok())
        {
            ESP_LOGE("mplane_relay", "%s: bad target %s", cfg.name.c_str(), cfg.relay.target.c_str());
            state->client.reset();
            state->running.store(false);
            return;
        }
        state->poll = app_kit::every(*rp, cfg.poll_ms, [state, receive = transport.receive]
        {
            std::vector<uint8_t> air;
            while (state->running.load() && receive(&air)) state->client->on_air(air.data(), air.size());
        });
        state->expiry = app_kit::every(*rp, 1000, [state]
        {
            if (state->running.load()) state->client->tick();
        });
    });
    return true;
}

void Applet::stop()
{
    auto state = impl_->state;
    if (!state->running.exchange(false) || !impl_->reactor) return;
    app_kit::post(*impl_->reactor, [state]
    {
        if (state->poll) state->poll();
        if (state->expiry) state->expiry();
        state->poll = {};
        state->expiry = {};
        state->client.reset();
    });
}

}  // namespace mplane_relay
