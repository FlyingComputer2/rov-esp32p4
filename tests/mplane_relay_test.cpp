// Host test for components/mplane_relay: two ground clients share one local
// request/response console, and each gets only its own replies.

#include "mplane_relay.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace
{

// Console: replies "<request>@<sender port>" to the sender, like app_bwtest.
struct Console
{
    int fd = -1;
    uint16_t port = 0;
    std::atomic<bool> stop {false};
    std::thread t;

    Console()
    {
        fd = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in a {};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        assert(bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0);
        socklen_t len = sizeof(a);
        getsockname(fd, reinterpret_cast<sockaddr*>(&a), &len);
        port = ntohs(a.sin_port);
        timeval tv {0, 20000};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        t = std::thread([this] {
            char buf[2048];
            while (!stop)
            {
                sockaddr_in src {};
                socklen_t sl = sizeof(src);
                const ssize_t n =
                    recvfrom(fd, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&src), &sl);
                if (n <= 0)
                {
                    continue;
                }
                std::string reply(buf, static_cast<size_t>(n));
                if (reply == "big")
                {
                    reply.assign(2000, 'x');
                }
                else
                {
                    reply += "@" + std::to_string(ntohs(src.sin_port));
                }
                sendto(fd, reply.data(), reply.size(), 0, reinterpret_cast<sockaddr*>(&src), sl);
            }
        });
    }
    ~Console()
    {
        stop = true;
        t.join();
        close(fd);
    }
};

struct Air
{
    std::mutex mu;
    std::deque<std::vector<uint8_t>> down;  // ground -> rover
    std::deque<std::vector<uint8_t>> up;    // rover -> ground

    void request(uint8_t id, const std::string& s)
    {
        std::vector<uint8_t> d {id};
        d.insert(d.end(), s.begin(), s.end());
        std::lock_guard<std::mutex> l(mu);
        down.push_back(d);
    }
    bool reply(std::vector<uint8_t>* out)
    {
        for (int i = 0; i < 200; ++i)
        {
            {
                std::lock_guard<std::mutex> l(mu);
                if (!up.empty())
                {
                    *out = up.front();
                    up.pop_front();
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return false;
    }
};

std::string body(const std::vector<uint8_t>& d)
{
    return std::string(d.begin() + 1, d.end());
}

std::string port_of(const std::string& reply)
{
    return reply.substr(reply.find('@') + 1);
}

}  // namespace

int main()
{
    Console console;
    Air air;
    std::atomic<bool> stop {false};
    mplane_relay::Stats last;
    std::mutex stats_mu;

    mplane_relay::ClientConfig cfg;
    cfg.target = "127.0.0.1:" + std::to_string(console.port);
    cfg.max_clients = 2;
    cfg.max_datagram = 1445;
    cfg.stop = &stop;
    cfg.send = [&](const uint8_t* d, size_t n) {
        std::lock_guard<std::mutex> l(air.mu);
        air.up.emplace_back(d, d + n);
        return true;
    };
    cfg.receive = [&](std::vector<uint8_t>* out) {
        std::lock_guard<std::mutex> l(air.mu);
        if (air.down.empty())
        {
            return false;
        }
        *out = air.down.front();
        air.down.pop_front();
        return true;
    };
    cfg.on_stats = [&](const mplane_relay::Stats& s) {
        std::lock_guard<std::mutex> l(stats_mu);
        last = s;
    };
    std::thread relay([&] { assert(mplane_relay::client_main(cfg) == 0); });

    std::vector<uint8_t> r;

    // Two clients: each reply carries its client's id and a distinct sender.
    air.request(1, "stats");
    assert(air.reply(&r) && r[0] == 1 && body(r).rfind("stats@", 0) == 0);
    const std::string port1 = port_of(body(r));
    air.request(2, "stats");
    assert(air.reply(&r) && r[0] == 2);
    const std::string port2 = port_of(body(r));
    assert(port1 != port2);

    // Same client keeps its socket.
    air.request(1, "clr");
    assert(air.reply(&r) && r[0] == 1 && port_of(body(r)) == port1);

    // A reply too big for one air datagram is dropped and counted.
    air.request(1, "big");
    assert(!air.reply(&r));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // A third client evicts the least recently used socket (client 2).
    air.request(3, "status");
    assert(air.reply(&r) && r[0] == 3);
    air.request(1, "stats");
    assert(air.reply(&r) && r[0] == 1 && port_of(body(r)) == port1);

    // Id byte only: rejected.
    {
        std::lock_guard<std::mutex> l(air.mu);
        air.down.push_back({7});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    stop = true;
    relay.join();
    std::lock_guard<std::mutex> l(stats_mu);
    assert(last.requests == 6);
    assert(last.replies == 5);
    assert(last.too_big == 1);
    assert(last.evicted == 1);
    assert(last.bad == 1);
    std::printf("mplane_relay_test: ok\n");
    return 0;
}
