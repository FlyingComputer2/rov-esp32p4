#pragma once

// Rover half of a multi-client m-plane relay over an L3-owned queue upstream.
//
// The ground P4 serves a UDP port for operators and tags each request with a
// 1-byte client id (gnd-esp32p4 components/mplane_relay). Air datagrams are
//   [client_id u8][request or reply bytes]
// This half keeps one loopback UDP socket per client id, connected to a local
// request/response console (app_bwtest 5100, krono 5110), and sends each
// request from that client's socket. The console replies to the sender, so
// every reply comes back on the socket of the client that asked, and goes back
// over the air with that client's id. Clients do not see each other's replies.

#include <atomic>
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
    uint64_t send_fail = 0;  // console sendto or air send failures
    uint64_t too_big = 0;    // replies over max_datagram - 1 bytes, dropped
    uint64_t evicted = 0;    // client sockets closed to make room
};

struct ClientConfig
{
    // L3 queue: send one air datagram (false on reject); receive one, non-blocking.
    std::function<bool(const uint8_t* data, size_t len)> send;
    std::function<bool(std::vector<uint8_t>* out)> receive;
    // Local console, host:port.
    std::string target = "127.0.0.1:5100";
    // Client sockets kept at most (lwIP sockets are scarce); the least
    // recently used one is closed for a new id.
    size_t max_clients = 4;
    // A client socket unused this long is closed.
    int idle_ms = 120000;
    // select() timeout between drains of the air queue.
    int poll_ms = 5;
    // Largest air datagram (L3 stream payload limit).
    size_t max_datagram = 1445;
    // Set to stop client_main; nullptr = run forever.
    const std::atomic<bool>* stop = nullptr;
    // Optional: receives counters after every loop pass.
    std::function<void(const Stats&)> on_stats;
};

// Runs the relay until *stop. Returns 0, or 1 when the config is unusable.
int client_main(const ClientConfig& cfg);

}  // namespace mplane_relay
