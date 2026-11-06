#pragma once

// mplane_relay as an applet on a named app reactor (app_kit): the client
// sockets are watched there, the L3 queue drained every poll_ms and idle
// sockets closed once a second. ESP-IDF only; the core (mplane_relay.hpp) is
// host-tested.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "app_kit.hpp"
#include "mplane_relay.hpp"

namespace mplane_relay
{

struct AppletConfig
{
    std::string reactor = app_kit::k_apps_reactor;
    ClientConfig relay;
    // How often the L3 queue is drained (requests from the air).
    uint32_t poll_ms = 2;
    // For logs.
    std::string name = "relay";
};

// The L3-owned queue: send one air datagram (false if not taken); receive
// one, never blocking.
struct Transport
{
    std::function<bool(const uint8_t* data, size_t len)> send;
    std::function<bool(std::vector<uint8_t>* out)> receive;
};

class Applet
{
public:
    Applet();
    ~Applet();
    Applet(const Applet&) = delete;
    Applet& operator=(const Applet&) = delete;
    bool configure(const AppletConfig& cfg, Transport transport);
    bool start();
    void stop();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mplane_relay
