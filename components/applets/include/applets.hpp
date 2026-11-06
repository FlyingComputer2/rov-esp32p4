// The bandwidth tester and the clock-sync client / server as applets on a
// named reactor (bfc::default_reactor<> in bfc::default_reactor_registry),
// each on named L3 upstream queues (bfc's queue registry). Shared by
// rov-esp32p4 and gnd-esp32p4 (keep the copies identical).
//
// configure() takes the settings and the names and finds them; start()
// registers the callbacks on the reactor: call it before the reactor thread
// runs (registration is not thread-safe). They live for the life of the
// program (no stop). A push from L3 wakes the reactor (queue.set_reactor);
// only what blocks keeps a thread: test_send's sender and the clock-sync
// client's exchanges.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <bfc/default_reactor.hpp>
#include <bfc/event_queue.hpp>
#include <bfc/shared_sized_buffer.hpp>

#include "esp_pthread.h"
#include "synch/Synch.h"

namespace applets
{

using reactor_t = bfc::default_reactor<>;
using queue_t = bfc::reactive_circular_lossy_queue<bfc::shared_sized_buffer>;

// An L3 QUEUE_FORWARDING upstream's two named queues.
struct upstream_names
{
    std::string to_l3;    // the upstream's `rx`: the applet pushes, L3 consumes
    std::string from_l3;  // its `tx`: L3 pushes, the applet consumes
};

struct bwtest_config
{
    std::string reactor;
    // The test traffic's upstream.
    upstream_names data;
    // Its m-plane over the air (the console's commands, request in, reply
    // out); empty names = none.
    upstream_names mplane;
    // UDP console, one command per datagram, reply to the sender.
    std::string console = "0.0.0.0:5100";
    size_t payload_bytes = 1400;
    // The thread test_send starts (the host's policy: low priority so video
    // wins, PSRAM stack).
    esp_pthread_cfg_t sender = esp_pthread_get_default_config();
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
    std::shared_ptr<reactor_t> reactor_;
    std::shared_ptr<queue_t> to_l3_, from_l3_, mp_to_l3_, mp_from_l3_;
};

struct synch_client_config
{
    std::string reactor;
    upstream_names data;
    upstream_names mplane;  // empty = none
    std::string console = "0.0.0.0:5110";
    krono::ClientOptions options;
    uint32_t period_ms = 1000;
    // The exchanges' thread (an exchange blocks for up to options.timeout_ms).
    esp_pthread_cfg_t worker = esp_pthread_get_default_config();
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
    std::shared_ptr<reactor_t> reactor_;
    std::shared_ptr<queue_t> to_l3_, from_l3_, mp_to_l3_, mp_from_l3_;
};

struct synch_server_config
{
    std::string reactor;
    upstream_names data;
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
    std::shared_ptr<reactor_t> reactor_;
    std::shared_ptr<queue_t> to_l3_, from_l3_;
};

}  // namespace applets
