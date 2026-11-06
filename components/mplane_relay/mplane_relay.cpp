#include "mplane_relay.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>

namespace mplane_relay
{
namespace
{

bool parse_host_port(const std::string& s, sockaddr_in* out)
{
    const size_t colon = s.rfind(':');
    if (colon == std::string::npos)
    {
        return false;
    }
    const std::string host = s.substr(0, colon);
    char* end = nullptr;
    const long port = std::strtol(s.c_str() + colon + 1, &end, 10);
    if (*end != '\0' || port <= 0 || port > 65535)
    {
        return false;
    }
    std::memset(out, 0, sizeof(*out));
    out->sin_family = AF_INET;
    out->sin_port = htons(static_cast<uint16_t>(port));
    return inet_pton(AF_INET, host.c_str(), &out->sin_addr) == 1;
}

}  // namespace

Client::Client(ClientConfig cfg, Hooks hooks)
    : m_cfg(std::move(cfg))
    , m_hooks(std::move(hooks))
{
    sockaddr_in target {};
    m_ok = m_hooks.send_air && m_hooks.watch && m_hooks.unwatch && m_hooks.now_ms && m_cfg.max_clients > 0 &&
           m_cfg.max_datagram >= 2 && parse_host_port(m_cfg.target, &target);
    m_target.assign(reinterpret_cast<const uint8_t*>(&target), reinterpret_cast<const uint8_t*>(&target) + sizeof(target));
    m_slots.resize(m_cfg.max_clients);
    // Room for the id byte in front, and one byte more to detect a reply too
    // long for one air datagram (recv truncates it).
    m_buf.resize(m_cfg.max_datagram + 1);
}

Client::~Client()
{
    for (auto& s : m_slots)
    {
        close_slot(&s);
    }
}

void Client::close_slot(Slot* s)
{
    if (s->fd >= 0)
    {
        m_hooks.unwatch(s->fd);
        close(s->fd);
    }
    s->fd = -1;
}

Client::Slot* Client::socket_for(uint8_t id, int64_t now)
{
    Slot* lru = nullptr;
    for (auto& s : m_slots)
    {
        if (s.fd >= 0 && s.id == id)
        {
            return &s;
        }
        if (!lru || s.fd < 0 || (lru->fd >= 0 && s.last_ms < lru->last_ms))
        {
            lru = &s;
        }
    }
    if (lru->fd >= 0)
    {
        close_slot(lru);
        ++m_stats.evicted;
    }
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        return nullptr;
    }
    // connect(): the bound ephemeral port is this client's identity at the
    // console, and only the console's replies arrive on it.
    if (connect(fd, reinterpret_cast<const sockaddr*>(m_target.data()), sizeof(sockaddr_in)) != 0)
    {
        close(fd);
        return nullptr;
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    lru->fd = fd;
    lru->id = id;
    lru->last_ms = now;
    m_hooks.watch(fd);
    return lru;
}

void Client::on_air(const uint8_t* data, size_t len)
{
    if (!m_ok)
    {
        return;
    }
    if (len < 2)
    {
        ++m_stats.bad;
        return;
    }
    const int64_t now = m_hooks.now_ms();
    Slot* s = socket_for(data[0], now);
    if (!s || send(s->fd, data + 1, len - 1, 0) < 0)
    {
        ++m_stats.send_fail;
        return;
    }
    s->last_ms = now;
    ++m_stats.requests;
}

void Client::on_readable(int fd)
{
    for (auto& s : m_slots)
    {
        if (s.fd != fd)
        {
            continue;
        }
        // Drain: the socket is non-blocking.
        for (;;)
        {
            const ssize_t n = recv(fd, m_buf.data() + 1, m_cfg.max_datagram, 0);
            if (n <= 0)
            {
                return;
            }
            if (static_cast<size_t>(n) > m_cfg.max_datagram - 1)
            {
                ++m_stats.too_big;
                continue;
            }
            m_buf[0] = s.id;
            if (m_hooks.send_air(m_buf.data(), static_cast<size_t>(n) + 1))
            {
                ++m_stats.replies;
            }
            else
            {
                ++m_stats.send_fail;
            }
        }
    }
}

void Client::tick()
{
    const int64_t now = m_hooks.now_ms();
    for (auto& s : m_slots)
    {
        if (s.fd >= 0 && now - s.last_ms > m_cfg.idle_ms)
        {
            close_slot(&s);
        }
    }
}

}  // namespace mplane_relay
