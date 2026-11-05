#include "server.hpp"

#include <cstdint>
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
    case status::enospc: return "ENOSPC";
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

server::server(backend& flash, uint32_t idle_timeout_ms, std::function<void(bool)> on_active, file_store* files)
    : m_flash(flash)
    , m_idle_timeout_ms(idle_timeout_ms)
    , m_on_active(std::move(on_active))
    , m_files(files)
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

void server::fs_close_idle(uint32_t now_ms)
{
    if (m_fs_open && now_ms - m_fs_last_ms >= m_idle_timeout_ms)
    {
        std::printf("UDP_OTA fs timeout, closing %s\n", m_fs_name.c_str());
        m_files->close();
        m_fs_open = false;
    }
}

void server::tick(uint32_t now_ms)
{
    fs_close_idle(now_ms);
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
    case msg::fs_open:
    case msg::fs_close:
    case msg::fs_read:
    case msg::fs_write:
    case msg::fs_truncate: return on_fs(req, len, from, now_ms, reply);
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

size_t server::on_fs(const uint8_t* req, size_t len, const peer& from, uint32_t now_ms, uint8_t* reply)
{
    const auto type = static_cast<msg>(req[0]);
    reply[0] = req[0] | k_response;
    // Each reply's layout, with st at its status byte (wire.hpp).
    auto answer = [&](status st) -> size_t
    {
        switch (type)
        {
        case msg::fs_open:
            reply[1] = static_cast<uint8_t>(st);
            put_u32(reply + 2, st == status::ok ? m_fs_size : 0);
            return 6;
        case msg::fs_close:
            reply[1] = static_cast<uint8_t>(st);
            return 2;
        case msg::fs_read:
            put_u32(reply + 1, len >= 5 ? get_u32(req + 1) : 0);
            reply[5] = static_cast<uint8_t>(st);
            return 6;
        case msg::fs_write:
            put_u32(reply + 1, len >= 5 ? get_u32(req + 1) : 0);
            put_u16(reply + 5, len >= 7 ? get_u16(req + 5) : 0);
            reply[7] = static_cast<uint8_t>(st);
            put_u32(reply + 8, m_fs_open ? m_fs_size : 0);
            return 12;
        default:  // fs_truncate
            reply[1] = static_cast<uint8_t>(st);
            put_u32(reply + 2, m_fs_open ? m_fs_size : 0);
            return 6;
        }
    };

    if (!m_files)
    {
        return answer(status::enoent);
    }

    if (type == msg::fs_open)
    {
        const char* name = reinterpret_cast<const char*>(req + 1);
        if (!valid_file_name(name, len - 1, k_file_name_max))
        {
            return answer(status::einval);
        }
        const std::string n(name, len - 1);
        if (m_fs_open)
        {
            // The owner repeating its open (lost reply) is fine; anyone else waits.
            if (!(from == m_fs_owner && n == m_fs_name))
            {
                return answer(status::einprogress);
            }
            m_fs_last_ms = now_ms;
            return answer(status::ok);
        }
        const status st = m_files->open(n, &m_fs_size);
        if (st == status::ok)
        {
            m_fs_open = true;
            m_fs_owner = from;
            m_fs_name = n;
            m_fs_last_ms = now_ms;
            std::printf("UDP_OTA fs open %s size=%u\n", n.c_str(), (unsigned)m_fs_size);
        }
        return answer(st);
    }

    if (type == msg::fs_close)
    {
        if (len != 1)
        {
            return answer(status::einval);
        }
        if (!m_fs_open)
        {
            return answer(status::ok);  // nothing open, or a repeated close
        }
        if (!(from == m_fs_owner))
        {
            return answer(status::einprogress);
        }
        m_fs_open = false;
        const status st = m_files->close();
        std::printf("UDP_OTA fs close %s size=%u %s\n", m_fs_name.c_str(), (unsigned)m_fs_size, status_name(st));
        return answer(st);
    }

    // read / write / truncate: on the open file, from its owner.
    if (!m_fs_open)
    {
        return answer(status::enoent);
    }
    if (!(from == m_fs_owner))
    {
        return answer(status::einprogress);
    }
    m_fs_last_ms = now_ms;

    if (type == msg::fs_read)
    {
        if (len != 7)
        {
            return answer(status::einval);
        }
        const uint32_t offset = get_u32(req + 1);
        const uint16_t size = get_u16(req + 5);
        if (size == 0 || size > k_fragment_size || offset > m_fs_size)
        {
            return answer(status::einval);
        }
        size_t n = 0;
        const status st = m_files->read(offset, reply + 6, size, &n);
        return answer(st) + (st == status::ok ? n : 0);
    }

    if (type == msg::fs_write)
    {
        if (len < 7)
        {
            return answer(status::einval);
        }
        const uint32_t offset = get_u32(req + 1);
        const uint16_t size = get_u16(req + 5);
        if (size == 0 || size > k_fragment_size || size != len - 7 || offset > m_fs_size ||
            static_cast<uint64_t>(offset) + size > UINT32_MAX)
        {
            return answer(status::einval);
        }
        const status st = m_files->write(offset, req + 7, size);
        if (st == status::ok && offset + size > m_fs_size)
        {
            m_fs_size = offset + size;
        }
        return answer(st);
    }

    // fs_truncate
    if (len != 5)
    {
        return answer(status::einval);
    }
    const uint32_t size = get_u32(req + 1);
    if (size > m_fs_size)
    {
        return answer(status::einval);
    }
    const status st = m_files->truncate(size);
    if (st == status::ok)
    {
        m_fs_size = size;
    }
    return answer(st);
}

}  // namespace udp_ota
