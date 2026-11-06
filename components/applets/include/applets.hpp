// Owned applets on a named reactor. Each applet has only the
// init/start/stop lifecycle; blocking work may use a worker, but an applet
// never owns a main loop.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "app_kit.hpp"
#include "endpoint/UpstreamQueue.h"
#include "synch/Synch.h"

namespace applets
{

struct bwtest_config
{
    std::string reactor = app_kit::k_apps_reactor;
    // UDP console, one command per datagram, reply to the sender.
    std::string console = "0.0.0.0:5100";
    size_t payload_bytes = 1400;
    // How often received test datagrams are counted (the latency it reports
    // includes up to this).
    uint32_t poll_ms = 2;
    // The sender test_send starts: low priority so video wins, PSRAM stack.
    app_kit::thread_spec sender {"bwtest", 6 * 1024, 5, -1, app_kit::stack_in::psram, false};
};

// app_bwtest on its L3-owned queue: console and receive side on the reactor;
// test_send's sender is a thread for the length of the test. False if there
// is no such reactor or the console cannot be bound.
class BwtestApplet
{
public:
    BwtestApplet();
    ~BwtestApplet();
    BwtestApplet(const BwtestApplet&) = delete;
    BwtestApplet& operator=(const BwtestApplet&) = delete;
    bool configure(std::shared_ptr<winject::UpstreamQueue> up, const bwtest_config& cfg);
    bool start();
    void stop();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

struct synch_client_config
{
    std::string reactor = app_kit::k_apps_reactor;
    std::string console = "0.0.0.0:5110";
    krono::ClientOptions options;
    uint32_t period_ms = 1000;
    // An exchange blocks for up to options.timeout_ms (300 ms), so it keeps a
    // thread; priority 12 so its timestamps are taken on time. No flash I/O.
    app_kit::thread_spec worker {"synch", 6 * 1024, 12, -1, app_kit::stack_in::psram, false};
};

// krono's SynchClient on transport: its exchanges on the worker thread, its
// console on the reactor. False if there is no such reactor or the console
// cannot be bound.
class SynchClientApplet
{
public:
    SynchClientApplet();
    ~SynchClientApplet();
    SynchClientApplet(const SynchClientApplet&) = delete;
    SynchClientApplet& operator=(const SynchClientApplet&) = delete;
    bool configure(krono::Transport transport, const synch_client_config& cfg);
    bool start();
    void stop();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace applets
