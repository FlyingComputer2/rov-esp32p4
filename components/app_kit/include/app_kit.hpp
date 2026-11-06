// app_kit: named app reactors and the common thread default.
//
// Low-rate applets (UDP consoles, m-plane relays, the tester's receive side,
// rover_sample's RC link) share one reactor thread instead of a thread each.
// The host starts a reactor under a name (start_reactor); an applet gets only
// that name in its config and attaches itself (find_reactor, then
// udp_console / every / post). Sockets wake the reactor when readable;
// reactor timers drain the L3-owned queues.
//
// The reactor is bfc::default_reactor (poll_reactor on ESP-IDF). It uses
// std::function callbacks because poll_reactor's removal path captures a
// callback inside another callback, which cannot fit in bfc's fixed-size
// light_function. The helpers here also box callers' larger callables.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <bfc/default_reactor.hpp>
#include <bfc/event_queue.hpp>
#include <bfc/shared_sized_buffer.hpp>

#include "esp_pthread.h"

namespace app_kit
{

using reactor_t = bfc::default_reactor<std::function<void()>>;
using reactor_tag = bfc::object_tag<reactor_t>;
using reactor_registry = bfc::object_registry<reactor_tag>;

// The board's app reactor (main.cpp starts it).
constexpr const char* k_apps_reactor = "apps";

// Where a thread's stack lives. PSRAM is the default: internal RAM is what
// live TX runs short of (~10-20 KB left once the 1080p encoder holds its
// reference frame). internal only for a thread that writes flash (a PSRAM
// stack is unusable while the cache is off) or whose timing would suffer;
// say why at the call site.
enum class stack_in
{
    psram,
    internal,
};

struct thread_spec
{
    const char* name = nullptr;
    size_t stack_bytes = 4 * 1024;
    int prio = 5;
    int core = -1;  // -1 = unpinned
    stack_in stack = stack_in::psram;
    // Threads this one creates get the same settings.
    bool inherit = false;
};

esp_pthread_cfg_t pthread_cfg(const thread_spec& spec);

// esp_pthread_set_cfg() applies to every thread the calling thread creates
// next. Sets cfg for this scope, then restores the previous config, or the
// default when there was none, so it cannot leak into the caller's later
// threads.
class scoped_pthread_cfg
{
public:
    explicit scoped_pthread_cfg(const esp_pthread_cfg_t& cfg);
    explicit scoped_pthread_cfg(const thread_spec& spec) : scoped_pthread_cfg(pthread_cfg(spec)) {}
    ~scoped_pthread_cfg();
    scoped_pthread_cfg(const scoped_pthread_cfg&) = delete;
    scoped_pthread_cfg& operator=(const scoped_pthread_cfg&) = delete;

private:
    esp_pthread_cfg_t prev {};
    bool had_prev;
};

// A thread with spec's settings; the caller detaches or joins it.
std::thread spawn(const thread_spec& spec, std::function<void()> fn);

// Host: creates a reactor, registers it under name and runs it on a thread
// with thread's settings, for the life of the program. False if the name is
// taken.
bool start_reactor(const std::string& name, const thread_spec& thread);

// Applets: the reactor registered under name; nullptr if there is none.
std::shared_ptr<reactor_t> find_reactor(const std::string& name);

// Runs fn on r's thread (from any thread).
void post(reactor_t& r, std::function<void()> fn);

// Calls fn on r's thread every period_ms from now on, for the life of the
// program (from any thread). fn must not block.
void every(reactor_t& r, uint32_t period_ms, std::function<void()> fn);

// At most this many datagrams per callback or tick, so one busy socket or
// queue cannot hold the reactor: the reactor is level-triggered, so what is
// left is read on its next pass, after the other ready work.
constexpr size_t k_max_batch = 16;

// A datagram transport, e.g. an L3-owned upstream queue: send one (false if
// not taken), receive one (false when none, never blocks).
struct datagrams
{
    std::function<bool(const uint8_t* data, size_t len)> send;
    std::function<bool(std::vector<uint8_t>* out)> receive;
};

// ---- Named L3 upstream queues --------------------------------------------
//
// An app upstream is an L3 QUEUE_FORWARDING upstream on two named queues in
// bfc's queue registry: its `rx` (to_l3, the app pushes, L3 consumes on its
// reactor) and its `tx` (from_l3, L3 pushes, the applet consumes). The host
// registers them before winject-l3 starts; an applet finds them by name. A
// push into from_l3 wakes the applet's reactor (no polling).

using queue_t = bfc::reactive_circular_lossy_queue<bfc::shared_sized_buffer>;
using queue_registry = bfc::cv_reactor_circular_lossy_queue_registry<bfc::shared_sized_buffer>;

struct upstream_names
{
    std::string to_l3;    // the upstream's `rx` in the L3 config
    std::string from_l3;  // the upstream's `tx`
};

struct upstream_queues
{
    std::shared_ptr<queue_t> to_l3;
    std::shared_ptr<queue_t> from_l3;
};

// Host: registers both queues with this capacity (datagrams; a full queue
// drops its oldest). False if a name is taken.
bool add_upstream_queues(const upstream_names& names, size_t capacity);

// Applets: the queues by name; false (and logged) if either is missing.
bool find_upstream_queues(const upstream_names& names, upstream_queues* out);

// The queues as a datagram transport.
datagrams link(const upstream_queues& q);

// Calls on_data on r's thread whenever q.from_l3 gets data, for the life of
// the program. on_data takes at most k_max_batch datagrams; while some are
// left it is called again (after the reactor's other ready work). One
// consumer per queue.
void on_receive(reactor_t& r, const upstream_queues& q, std::function<void()> on_data);

// An m-plane served directly on an upstream (no socket): each request
// datagram from q.from_l3 is handled on r's thread and its non-empty reply
// pushed to q.to_l3.
void queue_console(reactor_t& r, const upstream_queues& q,
                   std::function<std::string(const std::string& line)> handle);

// Calls fn on r's thread while fd is readable, for the life of the program
// (from any thread). fn should read at most k_max_batch datagrams per call.
void watch(reactor_t& r, int fd, std::function<void()> fn);

// A UDP console on host:port, served on r for the life of the program: one
// command per datagram, handle's reply to the sender, at most k_max_batch per
// pass. False if the socket cannot be bound.
bool udp_console(reactor_t& r, const std::string& host_port,
                 std::function<std::string(const std::string& line)> handle);

// Parses host:port into a sockaddr_in (as bytes, to keep lwIP out of this
// header); false if it is not one.
bool parse_host_port(const std::string& s, void* sockaddr_in_out);

}  // namespace app_kit
