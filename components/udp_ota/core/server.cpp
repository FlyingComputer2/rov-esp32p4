#include "server.hpp"

#include <cstdio>
#include <cstring>

namespace udp_ota
{

const char* status_name(status s)
{
    switch (s)
    {
    case status::ok: return "OK";
    case status::einprogress: return "EINPROGRESS";
    case status::einval: return "EINVAL";
    case status::enoent: return "ENOENT";
    case status::eio: return "EIO";
    case status::ebadmsg: return "EBADMSG";
    case status::efbig: return "EFBIG";
    }
    return "E?";
}

const char* state_name(server::state s)
{
    switch (s)
    {
    case server::state::idle: return "idle";
    case server::state::active: return "active";
    case server::state::done: return "done";
    }
    return "?";
}

server::server(backend& flash, uint32_t idle_timeout_ms, std::function<void(bool)> on_active)
    : m_flash(flash)
    , m_idle_timeout_ms(idle_timeout_ms)
    , m_on_active(std::move(on_active))
{
}

void server::end_active(state next)
{
    m_state = next;
    if (next == state::idle)
    {
        m_have.clear();
        m_count = 0;
        m_received = 0;
    }
    if (m_on_active)
    {
        m_on_active(false);
    }
}

void server::tick(uint32_t now_ms)
{
    if (m_state == state::active && now_ms - m_last_ms >= m_idle_timeout_ms)
    {
        std::printf("UDP_OTA timeout after %u/%u fragments\n", (unsigned)m_received, (unsigned)m_count);
        m_flash.abort();
        end_active(state::idle);
    }
}

size_t server::handle(const uint8_t* req, size_t len, const peer& from, uint32_t now_ms, uint8_t* reply)
{
    tick(now_ms);
    if (len < 1 || (req[0] & k_response))
    {
        return 0;  // empty or a response: never answer one (no reply loops)
    }
    switch (static_cast<msg>(req[0]))
    {
    case msg::ota_start: return on_start(req, len, from, now_ms, reply);
    case msg::ota_fragment: return on_fragment(req, len, from, now_ms, reply);
    case msg::reboot:
        reply[0] = req[0] | k_response;
        if (len != 1)
        {
            reply[1] = static_cast<uint8_t>(status::einval);
            return 2;
        }
        if (m_state == state::active)
        {
            m_flash.abort();
            end_active(state::idle);
        }
        m_reboot = true;
        reply[1] = static_cast<uint8_t>(status::ok);
        return 2;
    case msg::status:
        if (len != 1)
        {
            reply[0] = req[0] | k_response;
            reply[1] = static_cast<uint8_t>(status::einval);
            return 2;
        }
        return on_status(reply);
    }
    reply[0] = req[0] | k_response;
    reply[1] = static_cast<uint8_t>(status::einval);
    return 2;
}

size_t server::on_start(const uint8_t* req, size_t len, const peer& from, uint32_t now_ms, uint8_t* reply)
{
    reply[0] = req[0] | k_response;
    if (len != 3)
    {
        reply[1] = reply[2] = 0;
        reply[3] = static_cast<uint8_t>(status::einval);
        return 4;
    }
    const uint16_t count = get_u16(req + 1);
    put_u16(reply + 1, count);
    status st = status::ok;
    if (m_state == state::active)
    {
        // A repeated start (its reply was lost) is fine; anyone else waits.
        if (!(from == m_owner && count == m_count))
        {
            st = status::einprogress;
        }
        else
        {
            m_last_ms = now_ms;
        }
    }
    else if (count == 0)
    {
        st = status::einval;
    }
    else if (static_cast<size_t>(count) * k_fragment_size > m_flash.capacity())
    {
        st = status::efbig;
    }
    else if (!m_flash.begin())
    {
        st = status::eio;
        m_state = state::idle;
    }
    else
    {
        m_state = state::active;
        m_owner = from;
        m_last_ms = now_ms;
        m_count = count;
        m_received = 0;
        m_last_len = 0;
        m_have.assign(count, false);
        std::printf("UDP_OTA start fragments=%u\n", (unsigned)count);
        if (m_on_active)
        {
            m_on_active(true);
        }
    }
    reply[3] = static_cast<uint8_t>(st);
    return 4;
}

size_t server::on_fragment(const uint8_t* req, size_t len, const peer& from, uint32_t now_ms, uint8_t* reply)
{
    reply[0] = req[0] | k_response;
    if (len < 3)
    {
        reply[1] = reply[2] = 0;
        reply[3] = static_cast<uint8_t>(status::einval);
        return 4;
    }
    const uint16_t frag = get_u16(req + 1);
    const uint8_t* data = req + 3;
    const size_t n = len - 3;
    put_u16(reply + 1, frag);
    auto answer = [&](status st)
    {
        reply[3] = static_cast<uint8_t>(st);
        return size_t {4};
    };

    if (m_state == state::idle)
    {
        return answer(status::enoent);
    }
    if (!(from == m_owner))
    {
        return answer(status::einprogress);
    }
    if (frag >= m_count)
    {
        return answer(status::einval);
    }
    const bool last = frag + 1 == m_count;
    if (n == 0 || n > k_fragment_size || (!last && n != k_fragment_size))
    {
        return answer(status::einval);
    }
    if (m_state == state::done || m_have[frag])
    {
        return answer(status::ok);  // a retransmit whose reply was lost
    }
    m_last_ms = now_ms;
    if (!m_flash.write(static_cast<uint32_t>(frag) * k_fragment_size, data, n))
    {
        std::printf("UDP_OTA write failed at fragment %u\n", (unsigned)frag);
        m_flash.abort();
        end_active(state::idle);
        return answer(status::eio);
    }
    m_have[frag] = true;
    ++m_received;
    if (last)
    {
        m_last_len = static_cast<uint32_t>(n);
    }
    if (m_received < m_count)
    {
        return answer(status::ok);
    }
    // The fragment that completes the image carries the image's verdict.
    const uint32_t size = static_cast<uint32_t>(m_count - 1) * k_fragment_size + m_last_len;
    const status st = m_flash.finish(size);
    std::printf("UDP_OTA complete size=%u %s\n", (unsigned)size, status_name(st));
    end_active(st == status::ok ? state::done : state::idle);
    return answer(st);
}

size_t server::on_status(uint8_t* reply)
{
    reply[0] = static_cast<uint8_t>(msg::status) | k_response;
    reply[1] = static_cast<uint8_t>(status::ok);
    const std::string img = m_flash.describe();
    const int n = std::snprintf(reinterpret_cast<char*>(reply + 2), k_reply_max - 2, "%s ota=%s fragments=%u/%u",
                                img.c_str(), state_name(m_state), (unsigned)m_received, (unsigned)m_count);
    if (n < 0)
    {
        return 2;
    }
    return 2 + (static_cast<size_t>(n) < k_reply_max - 2 ? static_cast<size_t>(n) : k_reply_max - 3);
}

}  // namespace udp_ota
