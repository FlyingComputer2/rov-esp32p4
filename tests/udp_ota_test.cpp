// Host tests for the udp_ota core (no ESP-IDF):
//   cmake -S tests -B build-tests && cmake --build build-tests && build-tests/udp_ota_test
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "server.hpp"

using namespace udp_ota;

namespace
{

int g_failed = 0;

#define CHECK(cond)                                                     \
    do                                                                  \
    {                                                                   \
        if (!(cond))                                                    \
        {                                                               \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++g_failed;                                                 \
        }                                                               \
    } while (0)

struct fake_flash : backend
{
    size_t cap = 8 * k_fragment_size;
    std::vector<uint8_t> img;
    int begins = 0, writes = 0, aborts = 0;
    bool fail_write = false;
    status finish_status = status::ok;
    uint32_t finished_size = 0;

    size_t capacity() override { return cap; }
    bool begin() override
    {
        ++begins;
        img.assign(cap, 0xff);
        return true;
    }
    bool write(uint32_t off, const uint8_t* d, size_t n) override
    {
        ++writes;
        if (fail_write)
        {
            return false;
        }
        std::memcpy(img.data() + off, d, n);
        return true;
    }
    status finish(uint32_t size) override
    {
        finished_size = size;
        return finish_status;
    }
    void abort() override { ++aborts; }
    std::string describe() override { return "running=ota_0"; }
};

const peer k_a {0x0100007f, 2430};
const peer k_b {0x0200000a, 5555};

struct reply
{
    size_t n = 0;
    uint8_t b[server::k_reply_max];
    uint8_t type() const { return b[0]; }
    uint16_t id() const { return get_u16(b + 1); }
    status st() const { return static_cast<status>(b[3]); }
};

reply start(server& s, uint16_t count, const peer& p, uint32_t t = 0)
{
    uint8_t m[3] = {0x01};
    put_u16(m + 1, count);
    reply r;
    r.n = s.handle(m, sizeof(m), p, t, r.b);
    return r;
}

reply frag(server& s, uint16_t i, size_t len, uint8_t fill, const peer& p, uint32_t t = 0)
{
    std::vector<uint8_t> m(3 + len, fill);
    m[0] = 0x02;
    put_u16(m.data() + 1, i);
    reply r;
    r.n = s.handle(m.data(), m.size(), p, t, r.b);
    return r;
}

reply simple(server& s, uint8_t type, const peer& p)
{
    reply r;
    r.n = s.handle(&type, 1, p, 0, r.b);
    return r;
}

void test_happy_path_out_of_order_and_dups()
{
    fake_flash f;
    int active_on = 0, active_off = 0;
    server s(f, 30000, [&](bool on) { (on ? active_on : active_off)++; });
    reply r = start(s, 3, k_a);
    CHECK(r.n == 4 && r.type() == 0x81 && r.id() == 3 && r.st() == status::ok);
    CHECK(active_on == 1);
    CHECK(start(s, 3, k_a).st() == status::ok);  // retransmitted start
    CHECK(f.begins == 1);
    CHECK(frag(s, 2, 100, 0xcc, k_a).st() == status::ok);
    CHECK(frag(s, 0, k_fragment_size, 0xaa, k_a).st() == status::ok);
    CHECK(frag(s, 0, k_fragment_size, 0xaa, k_a).st() == status::ok);  // dup
    CHECK(f.writes == 2);
    CHECK(s.current() == server::state::active);
    r = frag(s, 1, k_fragment_size, 0xbb, k_a);
    CHECK(r.type() == 0x82 && r.id() == 1 && r.st() == status::ok);
    CHECK(s.current() == server::state::done);
    CHECK(f.finished_size == 2 * k_fragment_size + 100);
    CHECK(f.img[0] == 0xaa && f.img[k_fragment_size] == 0xbb && f.img[2 * k_fragment_size + 99] == 0xcc);
    CHECK(active_off == 1);
    // A late retransmit after completion is still OK and writes nothing.
    CHECK(frag(s, 2, 100, 0xcc, k_a).st() == status::ok);
    CHECK(f.writes == 3);
}

void test_multi_client()
{
    fake_flash f;
    server s(f, 30000);
    CHECK(start(s, 2, k_a).st() == status::ok);
    CHECK(start(s, 2, k_b).st() == status::einprogress);
    CHECK(start(s, 3, k_a).st() == status::einprogress);  // owner, other image
    CHECK(frag(s, 0, k_fragment_size, 1, k_b).st() == status::einprogress);
    reply st = simple(s, 0x04, k_b);  // status from anyone
    CHECK(st.n > 2 && st.b[0] == 0x84 && st.b[1] == 0);
    const std::string text(reinterpret_cast<const char*>(st.b + 2), st.n - 2);
    CHECK(text == "running=ota_0 ota=active fragments=0/2");
}

void test_bounds()
{
    fake_flash f;
    server s(f, 30000);
    CHECK(frag(s, 0, 10, 1, k_a).st() == status::enoent);
    CHECK(start(s, 0, k_a).st() == status::einval);
    CHECK(start(s, 9, k_a).st() == status::efbig);  // cap is 8 fragments
    CHECK(f.begins == 0);
    CHECK(start(s, 2, k_a).st() == status::ok);
    CHECK(frag(s, 2, 10, 1, k_a).st() == status::einval);                     // out of range
    CHECK(frag(s, 0, 10, 1, k_a).st() == status::einval);                     // short, not last
    CHECK(frag(s, 1, k_fragment_size + 1, 1, k_a).st() == status::einval);    // too long
    CHECK(frag(s, 1, 0, 1, k_a).st() == status::einval);                      // empty
    uint8_t bad[2] = {0x01, 0};
    reply r;
    r.n = s.handle(bad, sizeof(bad), k_a, 0, r.b);
    CHECK(r.n == 4 && r.st() == status::einval);
    uint8_t resp = 0x81;  // a response is never answered
    CHECK(s.handle(&resp, 1, k_a, 0, r.b) == 0);
    uint8_t unknown = 0x7e;
    r.n = s.handle(&unknown, 1, k_a, 0, r.b);
    CHECK(r.n == 2 && r.b[0] == 0xfe && r.b[1] == static_cast<uint8_t>(status::einval));
}

void test_timeout_and_failures()
{
    fake_flash f;
    int off = 0;
    server s(f, 1000, [&](bool on) { off += !on; });
    CHECK(start(s, 2, k_a, 0).st() == status::ok);
    CHECK(frag(s, 0, k_fragment_size, 1, k_a, 900).st() == status::ok);
    s.tick(1800);
    CHECK(s.current() == server::state::active);  // refreshed at 900
    s.tick(1900);
    CHECK(s.current() == server::state::idle && f.aborts == 1 && off == 1);
    CHECK(start(s, 2, k_b, 2000).st() == status::ok);  // free for another client

    f.fail_write = true;
    CHECK(frag(s, 0, k_fragment_size, 1, k_b, 2000).st() == status::eio);
    CHECK(s.current() == server::state::idle && off == 2);

    f.fail_write = false;
    f.finish_status = status::ebadmsg;
    CHECK(start(s, 1, k_a, 3000).st() == status::ok);
    CHECK(frag(s, 0, 5, 1, k_a, 3000).st() == status::ebadmsg);
    CHECK(s.current() == server::state::idle);
}

void test_reboot()
{
    fake_flash f;
    server s(f, 30000);
    CHECK(start(s, 2, k_a).st() == status::ok);
    reply r = simple(s, 0x03, k_b);
    CHECK(r.n == 2 && r.b[0] == 0x83 && r.b[1] == 0);
    CHECK(s.reboot_requested());
    CHECK(s.current() == server::state::idle && f.aborts == 1);
}

void test_restart_after_done()
{
    fake_flash f;
    server s(f, 30000);
    CHECK(start(s, 1, k_a).st() == status::ok);
    CHECK(frag(s, 0, 7, 1, k_a).st() == status::ok);
    CHECK(s.current() == server::state::done);
    CHECK(start(s, 2, k_b).st() == status::ok);  // a new image replaces the finished one
    CHECK(f.begins == 2 && s.current() == server::state::active);
}

}  // namespace

int main()
{
    test_happy_path_out_of_order_and_dups();
    test_multi_client();
    test_bounds();
    test_timeout_and_failures();
    test_reboot();
    test_restart_after_done();
    if (g_failed)
    {
        std::printf("%d check(s) failed\n", g_failed);
        return 1;
    }
    std::printf("udp_ota_test OK\n");
    return 0;
}
