#pragma once

// Rover half of a multi-client m-plane relay over an L3-owned queue upstream.
//
// The ground P4 serves a UDP port for operators and tags each request with a
// 1-byte client id (gnd-esp32p4 components/mplane_relay). Air datagrams are
//   [client_id u8][request or reply bytes]
// This half keeps one loopback UDP socket per client id, connected to a local
// request/response console (app_bwtest 5100, krono 5110, udp_ota 3232), and
// sends each request from that client's socket. The console replies to the
// sender, so every reply comes back on the socket of the client that asked,
// and goes back over the air with that client's id.
//
// No thread of its own: the host feeds air datagrams to on_air(), watches the
// sockets it is told about and calls on_readable() for them, and calls tick()
// now and then for idle expiry (the named app_kit reactor).

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace mplane_relay
{

struct Stats
{
    uint64_t requests = 0;   // air datagrams forwarded to the console
    uint64_t replies = 0;    // console datagrams sent back over the air
    uint64_t bad = 0;        // air datagrams with no payload after the id
    uint64_t send_fail = 0;  // console send or air send failures
    uint64_t too_big = 0;    // replies over max_datagram - 1 bytes, dropped
    uint64_t evicted = 0;    // client sockets closed to make room
};

struct ClientConfig
{
    // Local console, host:port.
    std::string target = "127.0.0.1:5100";
    // Client sockets kept at most (lwIP sockets are scarce); the least
    // recently used one is closed for a new id.
    size_t max_clients = 4;
    // A client socket unused this long is closed (tick()).
    int64_t idle_ms = 120000;
    // Largest air datagram (L3 stream payload limit).
    size_t max_datagram = 1445;
};

struct Hooks
{
    // One air datagram out (L3 queue try_send); false if not taken.
    std::function<bool(const uint8_t* data, size_t len)> send_air;
    // Start / stop calling on_readable(fd) when fd is readable. unwatch is
    // called before the socket is closed.
    std::function<void(int fd)> watch;
    std::function<void(int fd)> unwatch;
    // Milliseconds, monotonic.
    std::function<int64_t()> now_ms;
};

class Client
{
public:
    // False from ok() when target is not a valid host:port.
    Client(ClientConfig cfg, Hooks hooks);
    ~Client();
    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;

    bool ok() const { return m_ok; }
    // One datagram from the air: [client id][request].
    void on_air(const uint8_t* data, size_t len);
    // A watched client socket is readable: its replies go over the air.
    void on_readable(int fd);
    // Closes client sockets idle for idle_ms.
    void tick();
    const Stats& stats() const { return m_stats; }

private:
    struct Slot
    {
        int fd = -1;
        uint8_t id = 0;
        int64_t last_ms = 0;
    };
    Slot* socket_for(uint8_t id, int64_t now);
    void close_slot(Slot* s);

    ClientConfig m_cfg;
    Hooks m_hooks;
    bool m_ok = false;
    std::vector<Slot> m_slots;
    std::vector<uint8_t> m_buf;
    Stats m_stats;
    // target, as sockaddr_in bytes (kept opaque here).
    std::vector<uint8_t> m_target;
};

}  // namespace mplane_relay
