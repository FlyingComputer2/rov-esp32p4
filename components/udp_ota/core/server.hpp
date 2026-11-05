// udp_ota server state machine, transport and flash agnostic (host-tested).
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "wire.hpp"

namespace udp_ota
{

// The image slot. Calls come from the server's one thread.
struct backend
{
    virtual ~backend() = default;
    // Largest image in bytes.
    virtual size_t capacity() = 0;
    // Starts a new image; drops any unfinished or finished-but-not-booted one.
    virtual bool begin() = 0;
    virtual bool write(uint32_t offset, const uint8_t* data, size_t len) = 0;
    // All bytes written: verify and make it the boot image.
    virtual status finish(uint32_t size) = 0;
    virtual void abort() = 0;
    // One line for status_response, e.g. running slot and image.
    virtual std::string describe() = 0;
};

struct peer
{
    uint32_t addr = 0;
    uint16_t port = 0;
    bool operator==(const peer& o) const { return addr == o.addr && port == o.port; }
};

class server
{
public:
    enum class state
    {
        idle,
        active,  // receiving fragments from owner
        done,    // image complete and set to boot; waits for reboot_request
    };

    // on_active(true) when an OTA starts, (false) when it completes, fails or
    // times out. idle_timeout_ms: an active OTA with no request from its
    // owner for this long is aborted.
    server(backend& flash, uint32_t idle_timeout_ms, std::function<void(bool)> on_active = {});

    // Handles one datagram. Returns the reply length in reply (0 = no
    // reply). reply must hold at least k_reply_max bytes.
    static constexpr size_t k_reply_max = 256;
    size_t handle(const uint8_t* req, size_t len, const peer& from, uint32_t now_ms, uint8_t* reply);

    // Call periodically (and before handle) for the idle timeout.
    void tick(uint32_t now_ms);

    // A reboot_request was answered; restart once the reply is sent.
    bool reboot_requested() const { return m_reboot; }

    state current() const { return m_state; }
    uint16_t received() const { return m_received; }
    uint16_t fragment_count() const { return m_count; }

private:
    size_t on_start(const uint8_t* req, size_t len, const peer& from, uint32_t now_ms, uint8_t* reply);
    size_t on_fragment(const uint8_t* req, size_t len, const peer& from, uint32_t now_ms, uint8_t* reply);
    size_t on_status(uint8_t* reply);
    void end_active(state next);

    backend& m_flash;
    uint32_t m_idle_timeout_ms;
    std::function<void(bool)> m_on_active;
    state m_state = state::idle;
    peer m_owner;
    uint32_t m_last_ms = 0;
    uint16_t m_count = 0;
    uint16_t m_received = 0;
    uint32_t m_last_len = 0;
    std::vector<bool> m_have;
    bool m_reboot = false;
};

}  // namespace udp_ota
