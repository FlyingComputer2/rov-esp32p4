#include "mplane_relay.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <cstring>

namespace mplane_relay
{
namespace
{

int64_t now_ms()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

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

struct Client
{
    int fd = -1;
    uint8_t id = 0;
    int64_t last_ms = 0;
};

void close_client(Client* c)
{
    if (c->fd >= 0)
    {
        close(c->fd);
    }
    c->fd = -1;
}

}  // namespace

int client_main(const ClientConfig& cfg)
{
    sockaddr_in target {};
    if (!cfg.send || !cfg.receive || cfg.max_clients == 0 || cfg.max_datagram < 2 ||
        !parse_host_port(cfg.target, &target))
    {
        return 1;
    }
    std::vector<Client> clients(cfg.max_clients);
    std::vector<uint8_t> air;
    std::vector<uint8_t> buf(cfg.max_datagram);
    Stats st;

    auto socket_for = [&](uint8_t id, int64_t now) -> Client*
    {
        Client* lru = nullptr;
        for (auto& c : clients)
        {
            if (c.fd >= 0 && c.id == id)
            {
                return &c;
            }
            if (!lru || c.fd < 0 || (lru->fd >= 0 && c.last_ms < lru->last_ms))
            {
                lru = &c;
            }
        }
        if (lru->fd >= 0)
        {
            close_client(lru);
            ++st.evicted;
        }
        const int fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0)
        {
            return nullptr;
        }
        // connect(): the bound ephemeral port is this client's identity at the
        // console, and only the console's replies arrive on it.
        if (connect(fd, reinterpret_cast<const sockaddr*>(&target), sizeof(target)) != 0)
        {
            close(fd);
            return nullptr;
        }
        lru->fd = fd;
        lru->id = id;
        lru->last_ms = now;
        return lru;
    };

    while (!cfg.stop || !cfg.stop->load())
    {
        const int64_t now = now_ms();
        while (cfg.receive(&air))
        {
            if (air.size() < 2)
            {
                ++st.bad;
                continue;
            }
            Client* c = socket_for(air[0], now);
            if (!c || send(c->fd, air.data() + 1, air.size() - 1, 0) < 0)
            {
                ++st.send_fail;
                continue;
            }
            c->last_ms = now;
            ++st.requests;
        }

        fd_set rd;
        FD_ZERO(&rd);
        int maxfd = -1;
        for (auto& c : clients)
        {
            if (c.fd >= 0 && now - c.last_ms > cfg.idle_ms)
            {
                close_client(&c);
            }
            if (c.fd >= 0)
            {
                FD_SET(c.fd, &rd);
                maxfd = c.fd > maxfd ? c.fd : maxfd;
            }
        }
        timeval tv {0, cfg.poll_ms * 1000};
        if (maxfd < 0)
        {
            select(0, nullptr, nullptr, nullptr, &tv);
        }
        else if (select(maxfd + 1, &rd, nullptr, nullptr, &tv) > 0)
        {
            for (auto& c : clients)
            {
                if (c.fd < 0 || !FD_ISSET(c.fd, &rd))
                {
                    continue;
                }
                // Room for the id byte in front; a longer reply is truncated
                // by recv, so read one byte more to detect it.
                buf.resize(cfg.max_datagram + 1);
                const ssize_t n = recv(c.fd, buf.data() + 1, cfg.max_datagram, 0);
                if (n <= 0)
                {
                    continue;
                }
                if (static_cast<size_t>(n) > cfg.max_datagram - 1)
                {
                    ++st.too_big;
                    continue;
                }
                buf[0] = c.id;
                if (cfg.send(buf.data(), static_cast<size_t>(n) + 1))
                {
                    ++st.replies;
                }
                else
                {
                    ++st.send_fail;
                }
            }
        }
        if (cfg.on_stats)
        {
            cfg.on_stats(st);
        }
    }
    for (auto& c : clients)
    {
        close_client(&c);
    }
    return 0;
}

}  // namespace mplane_relay
