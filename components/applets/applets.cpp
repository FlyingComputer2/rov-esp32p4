#include "applets.hpp"

#include <atomic>
#include <chrono>
#include <thread>

#include "apps/bwtest/Bwtest.h"
#include "esp_log.h"

namespace applets
{
namespace { const char* TAG = "applets"; }

struct BwtestApplet::Impl
{
    bwtest_config cfg;
    std::shared_ptr<winject::UpstreamQueue> up;
    std::shared_ptr<app_kit::reactor_t> reactor;
    std::shared_ptr<app_bwtest::Bwtest> bw;
    app_kit::cancel_fn console;
    app_kit::cancel_fn poll;
    bool configured = false;
    bool running = false;
};

BwtestApplet::BwtestApplet() : impl_(std::make_unique<Impl>()) {}
BwtestApplet::~BwtestApplet() { stop(); }

bool BwtestApplet::configure(std::shared_ptr<winject::UpstreamQueue> up, const bwtest_config& cfg)
{
    stop();
    impl_->cfg = cfg;
    impl_->up = std::move(up);
    impl_->reactor = app_kit::find_reactor(cfg.reactor);
    impl_->configured = impl_->up && impl_->reactor;
    if (!impl_->configured) ESP_LOGE(TAG, "bwtest: no reactor %s or no queue", cfg.reactor.c_str());
    return impl_->configured;
}

bool BwtestApplet::start()
{
    if (!impl_->configured || impl_->running) return false;
    impl_->bw = std::make_shared<app_bwtest::Bwtest>(impl_->up, impl_->cfg.payload_bytes);
    const auto sender = impl_->cfg.sender;
    impl_->console = app_kit::udp_console(*impl_->reactor, impl_->cfg.console,
        [bw = impl_->bw, sender](const std::string& line)
        {
            const app_kit::scoped_pthread_cfg scope(sender);
            return bw->handle_command(line);
        });
    if (!impl_->console)
    {
        impl_->bw.reset();
        return false;
    }
    impl_->poll = app_kit::every(*impl_->reactor, impl_->cfg.poll_ms, [bw = impl_->bw]
    {
        while (bw->poll_receive() > 0) {}
    });
    impl_->running = true;
    printf("bwtest: console %s payload %u (reactor %s)\n", impl_->cfg.console.c_str(),
           (unsigned)impl_->cfg.payload_bytes, impl_->cfg.reactor.c_str());
    return true;
}

void BwtestApplet::stop()
{
    if (impl_->poll) impl_->poll();
    if (impl_->console) impl_->console();
    impl_->poll = {};
    impl_->console = {};
    impl_->bw.reset();
    impl_->running = false;
}

struct SynchClientApplet::Impl
{
    synch_client_config cfg;
    krono::Transport transport;
    std::shared_ptr<app_kit::reactor_t> reactor;
    std::shared_ptr<krono::SynchClient> client;
    app_kit::cancel_fn console;
    std::thread worker;
    std::atomic<bool> running {false};
    bool configured = false;
};

SynchClientApplet::SynchClientApplet() : impl_(std::make_unique<Impl>()) {}
SynchClientApplet::~SynchClientApplet() { stop(); }

bool SynchClientApplet::configure(krono::Transport transport, const synch_client_config& cfg)
{
    stop();
    impl_->cfg = cfg;
    impl_->transport = std::move(transport);
    impl_->reactor = app_kit::find_reactor(cfg.reactor);
    impl_->configured = bool(impl_->reactor);
    if (!impl_->configured) ESP_LOGE(TAG, "synch: no reactor %s", cfg.reactor.c_str());
    return impl_->configured;
}

bool SynchClientApplet::start()
{
    if (!impl_->configured || impl_->running.exchange(true)) return false;
    impl_->client = std::make_shared<krono::SynchClient>(impl_->transport, impl_->cfg.options);
    impl_->console = app_kit::udp_console(*impl_->reactor, impl_->cfg.console,
        [client = impl_->client](const std::string& line) { return client->handle_command(line); });
    if (!impl_->console)
    {
        impl_->client.reset();
        impl_->running.store(false);
        return false;
    }
    Impl* state = impl_.get();
    impl_->worker = app_kit::spawn(impl_->cfg.worker, [state]
    {
        auto next = std::chrono::steady_clock::now();
        while (state->running.load())
        {
            state->client->step_once();
            next += std::chrono::milliseconds(state->cfg.period_ms);
            while (state->running.load() && std::chrono::steady_clock::now() < next)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    printf("synch: client console %s, mode %s, period %u ms (reactor %s)\n",
           impl_->cfg.console.c_str(), krono::mode_name(impl_->cfg.options.mode),
           (unsigned)impl_->cfg.period_ms, impl_->cfg.reactor.c_str());
    return true;
}

void SynchClientApplet::stop()
{
    impl_->running.store(false);
    if (impl_->console) impl_->console();
    impl_->console = {};
    if (impl_->worker.joinable()) impl_->worker.join();
    impl_->client.reset();
}

}  // namespace applets
