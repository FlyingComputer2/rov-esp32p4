// Host test for components/mplane_relay: two ground clients share one local
// request/response console, and each gets only its own replies. The test
// plays the app reactor: it poll()s the sockets the relay asks to watch.

#include "mplane_relay.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <deque>
#include <string>
#include <thread>
#include <vector>

namespace
{

// Console: replies "<request>@<sender port>" to the sender, like app_bwtest;
// "big" gets a reply too long for one air datagram.
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
                const ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&src), &sl);
                if (n <= 0)
                {
                    continue;
                }
                std::string reply(buf, static_cast<size_t>(n));
                reply = reply == "big" ? std::string(2000, 'x') : reply + "@" + std::to_string(ntohs(src.sin_port));
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

int64_t now_ms()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

struct Harness
{
    std::vector<int> watched;
    std::deque<std::vector<uint8_t>> up;  // rover -> ground
    int64_t clock_offset = 0;
    mplane_relay::Client* relay = nullptr;

    mplane_relay::Hooks hooks()
    {
        mplane_relay::Hooks h;
        h.send_air = [this](const uint8_t* d, size_t n) {
            up.emplace_back(d, d + n);
            return true;
        };
        h.watch = [this](int fd) { watched.push_back(fd); };
        h.unwatch = [this](int fd) { watched.erase(std::remove(watched.begin(), watched.end(), fd), watched.end()); };
        h.now_ms = [this] { return now_ms() + clock_offset; };
        return h;
    }

    void request(uint8_t id, const std::string& s)
    {
        std::vector<uint8_t> d {id};
        d.insert(d.end(), s.begin(), s.end());
        relay->on_air(d.data(), d.size());
    }

    // The reactor's job: poll the watched sockets until a reply is out or 1 s.
    bool reply(std::vector<uint8_t>* out, int wait_ms = 1000)
    {
        const int64_t until = now_ms() + wait_ms;
        while (up.empty() && now_ms() < until)
        {
            std::vector<pollfd> p;
            for (int fd : watched)
            {
                p.push_back({fd, POLLIN, 0});
            }
            if (poll(p.data(), p.size(), 10) > 0)
            {
                for (auto& e : p)
                {
                    if (e.revents & POLLIN)
                    {
                        relay->on_readable(e.fd);
                    }
                }
            }
        }
        if (up.empty())
        {
            return false;
        }
        *out = up.front();
        up.pop_front();
        return true;
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
    Harness h;
    mplane_relay::ClientConfig cfg;
    cfg.target = "127.0.0.1:" + std::to_string(console.port);
    cfg.max_clients = 2;
    cfg.idle_ms = 1000;
    mplane_relay::Client relay(cfg, h.hooks());
    assert(relay.ok());
    h.relay = &relay;

    std::vector<uint8_t> r;

    // Two clients: each reply carries its client's id and a distinct sender.
    h.request(1, "stats");
    assert(h.reply(&r) && r[0] == 1 && body(r).rfind("stats@", 0) == 0);
    const std::string port1 = port_of(body(r));
    h.request(2, "stats");
    assert(h.reply(&r) && r[0] == 2);
    const std::string port2 = port_of(body(r));
    assert(port1 != port2);
    assert(h.watched.size() == 2);

    // Same client keeps its socket.
    h.request(1, "clr");
    assert(h.reply(&r) && r[0] == 1 && port_of(body(r)) == port1);

    // A reply too big for one air datagram is dropped and counted.
    h.request(1, "big");
    assert(!h.reply(&r, 200));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // A third client evicts the least recently used socket (client 2), which
    // is unwatched first.
    h.request(3, "status");
    assert(h.reply(&r) && r[0] == 3);
    assert(h.watched.size() == 2);
    h.request(1, "stats");
    assert(h.reply(&r) && r[0] == 1 && port_of(body(r)) == port1);

    // Id byte only: rejected.
    const uint8_t only_id = 7;
    relay.on_air(&only_id, 1);

    // Idle sockets are closed by tick().
    h.clock_offset = 5000;
    relay.tick();
    assert(h.watched.empty());

    const auto& st = relay.stats();
    assert(st.requests == 6);
    assert(st.replies == 5);
    assert(st.too_big == 1);
    assert(st.evicted == 1);
    assert(st.bad == 1);

    // A bad target is refused.
    mplane_relay::ClientConfig bad = cfg;
    bad.target = "nonsense";
    assert(!mplane_relay::Client(bad, h.hooks()).ok());
    std::printf("mplane_relay_test: ok\n");
    return 0;
}
