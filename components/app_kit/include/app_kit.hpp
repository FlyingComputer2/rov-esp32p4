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

// An m-plane served directly on a datagram transport (an L3 upstream, no
// socket): every poll_ms up to max_batch requests are taken from it and each
// one's non-empty reply sent back on it. False if link is incomplete.
bool queue_console(reactor_t& r, datagrams link, std::function<std::string(const std::string& line)> handle,
                   uint32_t poll_ms = 2, size_t max_batch = k_max_batch);

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
