// The bandwidth tester and the clock-sync client / server as applets on a
// named app reactor (app_kit). Shared by rov-esp32p4 and gnd-esp32p4 (keep the
// copies identical). configure() takes the settings and the reactor's name;
// start() registers the callbacks on that reactor. They live for the life of
// the program (no stop). Only what blocks keeps a thread: test_send's sender
// and the clock-sync exchanges.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
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
    // includes up to this) and m-plane requests taken.
    uint32_t poll_ms = 2;
    // The sender test_send starts: low priority so video wins, PSRAM stack.
    app_kit::thread_spec sender {"bwtest", 6 * 1024, 5, -1, app_kit::stack_in::psram, false};
};

// app_bwtest on its L3-owned queue: console, m-plane and receive side on the
// reactor; test_send's sender is a thread for the length of the test.
class BwtestApplet
{
public:
    // False if there is no such reactor or no queue.
    bool configure(std::shared_ptr<winject::UpstreamQueue> up, const bwtest_config& cfg);
    // Its m-plane over the air: the console's commands served directly on
    // link (an L3 upstream: request datagram in, reply out). Before or after
    // start() (L3 hands the upstreams out in index order).
    void set_mplane(app_kit::datagrams link);
    // Registers its callbacks. False if not configured or the console cannot
    // be bound.
    bool start();

private:
    bwtest_config cfg_;
    std::shared_ptr<winject::UpstreamQueue> up_;
    std::shared_ptr<app_kit::reactor_t> reactor_;
    app_kit::datagrams mplane_;
    // The console handler, once started.
    std::function<std::string(const std::string&)> handle_;
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
// console and m-plane on the reactor.
class SynchClientApplet
{
public:
    // False if there is no such reactor.
    bool configure(krono::Transport transport, const synch_client_config& cfg);
    // As BwtestApplet::set_mplane.
    void set_mplane(app_kit::datagrams link);
    // Registers its callbacks and starts the worker. False if not configured
    // or the console cannot be bound.
    bool start();

private:
    synch_client_config cfg_;
    krono::Transport transport_;
    std::shared_ptr<app_kit::reactor_t> reactor_;
    app_kit::datagrams mplane_;
    std::function<std::string(const std::string&)> handle_;
};

struct synch_server_config
{
    std::string reactor = app_kit::k_apps_reactor;
    std::string console = "0.0.0.0:5110";
    krono::EdgeSource edges;
    // Requests are answered (and the reference time read) when polled: every
    // poll_ms, on the reactor, up to everything queued.
    uint32_t poll_ms = 1;
};

// krono's SynchServer on transport: polled and its console served on the
// reactor (non-blocking, so no thread of its own).
class SynchServerApplet
{
public:
    // False if there is no such reactor.
    bool configure(krono::Transport transport, const synch_server_config& cfg);
    // Registers its callbacks. False if not configured or the console cannot
    // be bound.
    bool start();

private:
    synch_server_config cfg_;
    krono::Transport transport_;
    std::shared_ptr<app_kit::reactor_t> reactor_;
};

}  // namespace applets
