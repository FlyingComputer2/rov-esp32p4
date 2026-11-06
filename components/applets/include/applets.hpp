// The bandwidth tester and the clock-sync client / server as applets on a
// named app reactor (app_kit), each on named L3 upstream queues. Shared by
// rov-esp32p4 and gnd-esp32p4 (keep the copies identical).
//
// configure() takes the settings and the names (reactor, queues) and finds
// them; start() registers the callbacks. They live for the life of the
// program (no stop). Received datagrams wake the reactor (no polling). Only
// what blocks keeps a thread: test_send's sender and the clock-sync client's
// exchanges.
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
    // The test traffic's upstream.
    app_kit::upstream_names data;
    // Its m-plane over the air (the console's commands, request in, reply
    // out); empty names = none.
    app_kit::upstream_names mplane;
    // UDP console, one command per datagram, reply to the sender.
    std::string console = "0.0.0.0:5100";
    size_t payload_bytes = 1400;
    // The sender test_send starts: low priority so video wins, PSRAM stack.
    app_kit::thread_spec sender {"bwtest", 6 * 1024, 5, -1, app_kit::stack_in::psram, false};
};

// app_bwtest: console, m-plane and receive side on the reactor; test_send's
// sender is a thread for the length of the test.
class BwtestApplet
{
public:
    // False if the reactor or a queue is missing.
    bool configure(const bwtest_config& cfg);
    // Registers its callbacks. False if not configured or the console cannot
    // be bound.
    bool start();

private:
    bwtest_config cfg_;
    std::shared_ptr<app_kit::reactor_t> reactor_;
    app_kit::upstream_queues data_, mplane_;
    bool has_mplane_ = false;
};

struct synch_client_config
{
    std::string reactor = app_kit::k_apps_reactor;
    app_kit::upstream_names data;
    app_kit::upstream_names mplane;  // empty = none
    std::string console = "0.0.0.0:5110";
    krono::ClientOptions options;
    uint32_t period_ms = 1000;
    // An exchange blocks for up to options.timeout_ms (300 ms), so it keeps a
    // thread; priority 12 so its timestamps are taken on time. No flash I/O.
    app_kit::thread_spec worker {"synch", 6 * 1024, 12, -1, app_kit::stack_in::psram, false};
};

// krono's SynchClient: its exchanges on the worker thread (they poll the
// data queue themselves), its console and m-plane on the reactor.
class SynchClientApplet
{
public:
    bool configure(const synch_client_config& cfg);
    bool start();

private:
    synch_client_config cfg_;
    std::shared_ptr<app_kit::reactor_t> reactor_;
    app_kit::upstream_queues data_, mplane_;
    bool has_mplane_ = false;
};

struct synch_server_config
{
    std::string reactor = app_kit::k_apps_reactor;
    app_kit::upstream_names data;
    std::string console = "0.0.0.0:5110";
    krono::EdgeSource edges;
};

// krono's SynchServer: answers each request as it arrives (the reference
// time is read then), on the reactor; console there too.
class SynchServerApplet
{
public:
    bool configure(const synch_server_config& cfg);
    bool start();

private:
    synch_server_config cfg_;
    std::shared_ptr<app_kit::reactor_t> reactor_;
    app_kit::upstream_queues data_;
};

}  // namespace applets
