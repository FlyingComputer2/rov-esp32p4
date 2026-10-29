// Host tests for the rover_sample core (no ESP-IDF):
//   cmake -S tests -B build-tests && cmake --build build-tests && build-tests/rover_sample_test
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "ahrs.hpp"
#include "controller.hpp"
#include "msp.hpp"
#include "params.hpp"
#include "protocol.hpp"
#include "rc_link.hpp"

using namespace rover_sample;

namespace
{

int g_failed = 0;

#define CHECK(cond)                                                       \
    do                                                                    \
    {                                                                     \
        if (!(cond))                                                      \
        {                                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
            ++g_failed;                                                   \
        }                                                                 \
    } while (0)

#define CHECK_NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

constexpr float k_dt = 0.004f;

void test_msp()
{
    const std::vector<uint8_t> payload = {1, 2, 3, 250};
    auto wire = msp::encode(msp::k_to_rover, 0x4201, payload);
    CHECK(wire.size() == msp::k_overhead + payload.size());
    msp::frame f;
    CHECK(msp::decode(wire.data(), wire.size(), &f));
    CHECK(f.dir == '<' && f.func == 0x4201 && f.payload == payload);

    // MSP v2 reference: "$X<" flag 0, func 100 (MSP_API_VERSION... any), no payload.
    const uint8_t api[] = {'$', 'X', '<', 0, 100, 0, 0, 0, 0};
    uint8_t crc = 0;
    for (int i = 3; i < 8; ++i)
    {
        crc = msp::crc8_dvb_s2(crc, api[i]);
    }
    CHECK(crc == 0x8f);  // known value for 00 64 00 00 00

    auto bad = wire;
    bad[9] ^= 1;
    CHECK(!msp::decode(bad.data(), bad.size(), &f));
    auto longer = wire;
    longer.push_back(0);
    CHECK(!msp::decode(longer.data(), longer.size(), &f));
    CHECK(!msp::decode(wire.data(), wire.size() - 1, &f));
}

void test_protocol()
{
    rc_frame rc;
    rc.seq = 65535;
    rc.t_us = 0xdeadbeef;
    rc.src = k_src_udp;
    rc.flags = k_rc_flag_arm;
    rc.mode = 1;
    rc.steer = -1000;
    rc.thr = 999;
    rc.aux = -5;
    auto p = pack(rc);
    CHECK(p.size() == k_rc_bytes);
    auto back = unpack_rc(p);
    CHECK(back && back->seq == 65535 && back->t_us == 0xdeadbeef && back->steer == -1000 &&
          back->thr == 999 && back->aux == -5 && back->mode == 1);
    rc.thr = 1001;
    CHECK(!unpack_rc(pack(rc)));

    telem_frame t;
    t.seq = 7;
    t.roll = -12345;
    t.heading = 35999;
    t.flags = 0x3f;
    auto tp = pack(t);
    CHECK(tp.size() == k_telem_bytes);
    auto tb = unpack_telem(tp);
    CHECK(tb && tb->seq == 7 && tb->roll == -12345 && tb->heading == 35999 && tb->flags == 0x3f);

    uint8_t id;
    float v;
    CHECK(unpack_param(pack_param(12, 0.25f), &id, &v) && id == 12 && v == 0.25f);
    CHECK(sat16(1e9f) == 32767 && sat16(-1e9f) == -32768 && usat16(-3) == 0);
}

void test_params()
{
    params p;
    CHECK(p.get(p_servo_center_us) == 1500);
    CHECK(p.set(p_servo_center_us, 1512.6f) == 1513);
    CHECK(p.set(p_servo_center_us, 9000) == 2500);
    CHECK(p.set(p_kp, NAN) == 0.004f);
    CHECK(p.set(p_imu_yaw_sign, 0.3f) == 1);
    CHECK(find_param("wheelbase_m") && find_param("wheelbase_m")->id == p_wheelbase_m);
    CHECK(!find_param("nope"));
    size_t n;
    const param_info* t = param_table(&n);
    for (size_t i = 0; i < n; ++i)
    {
        CHECK(std::string(t[i].name).size() <= 15);  // NVS key limit
        CHECK(t[i].def >= t[i].min && t[i].def <= t[i].max);
        for (size_t j = i + 1; j < n; ++j)
        {
            CHECK(t[i].id != t[j].id);
        }
    }
}

control_input rc_in(float steer, float thr, bool arm)
{
    control_input in;
    in.dt = k_dt;
    in.rc_valid = in.rc_ever = true;
    in.steer = steer;
    in.thr = thr;
    in.arm = arm;
    in.imu_ok = in.gyro_cal = true;
    return in;
}

control_output run(controller& c, const params& p, control_input in, float seconds)
{
    control_output out;
    for (int i = 0; i < int(seconds / k_dt + 0.5f); ++i)
    {
        out = c.step(in, p);
    }
    return out;
}

void test_arming_failsafe()
{
    params p;
    controller c;
    // Throttle not at neutral: stays disarmed.
    CHECK(c.step(rc_in(0, 0.5f, true), p).st == state::disarmed);
    CHECK(c.step(rc_in(0, 0, true), p).st == state::armed);
    run(c, p, rc_in(0, 0.5f, true), 1.0f);
    // Link lost: brake for 1 s, then coast, servo centred.
    control_input lost = rc_in(0.8f, 0.5f, true);
    lost.rc_valid = false;
    auto out = c.step(lost, p);
    CHECK(out.st == state::failsafe && out.motor == motor_cmd::brake && out.servo == 0);
    out = run(c, p, lost, 1.1f);
    CHECK(out.motor == motor_cmd::coast);
    // Link back with the stick still up: stays in failsafe until neutral.
    CHECK(c.step(rc_in(0, 0.5f, true), p).st == state::failsafe);
    CHECK(c.step(rc_in(0, 0, true), p).st == state::armed);
    // Disarm.
    CHECK(c.step(rc_in(0, 0, false), p).st == state::disarmed);
    // No gyro calibration with an IMU: no arming.
    controller c2;
    auto nocal = rc_in(0, 0, true);
    nocal.gyro_cal = false;
    CHECK(c2.step(nocal, p).st == state::disarmed);
    // Deadman: fault until disarmed.
    auto dm = rc_in(0, 0, true);
    dm.deadman = true;
    CHECK(c.step(rc_in(0, 0, true), p).st == state::armed);
    out = c.step(dm, p);
    CHECK(out.st == state::fault && out.motor == motor_cmd::coast);
    CHECK(c.step(rc_in(0, 0, true), p).st == state::fault);
    CHECK(c.step(rc_in(0, 0, false), p).st == state::disarmed);
}

void test_throttle_esc()
{
    params p;
    controller c;
    c.step(rc_in(0, 0, true), p);
    // Slew: 300 ms to full.
    auto out = c.step(rc_in(0, 1, true), p);
    CHECK(out.motor == motor_cmd::drive);
    CHECK_NEAR(out.duty, k_dt * 1000 / 300.f, 1e-4f);
    out = run(c, p, rc_in(0, 1, true), 2.0f);
    CHECK_NEAR(out.duty, 0.6f, 1e-3f);  // thr_max_fwd 60 %
    CHECK(out.v_est > 3.f);
    // Pull back while moving: brake, never reverse.
    out = c.step(rc_in(0, -1, true), p);
    CHECK(out.motor == motor_cmd::brake && out.duty == 1.f);
    out = run(c, p, rc_in(0, -1, true), 3.0f);
    CHECK(out.motor == motor_cmd::brake);
    // Neutral, then back again (now slow): reverse.
    run(c, p, rc_in(0, 0, true), 0.1f);
    out = run(c, p, rc_in(0, -1, true), 1.0f);
    CHECK(out.motor == motor_cmd::drive && out.duty < -0.39f);
    CHECK(out.v_est < 0);
    // Forward from reverse: dead time through zero first.
    out = c.step(rc_in(0, 1, true), p);
    CHECK(out.motor == motor_cmd::coast && out.duty == 0);
    out = run(c, p, rc_in(0, 1, true), 0.09f);
    CHECK(out.duty == 0);
    out = run(c, p, rc_in(0, 1, true), 0.05f);
    CHECK(out.duty > 0);
    // Deadband.
    controller c3;
    c3.step(rc_in(0, 0, true), p);
    CHECK(c3.step(rc_in(0, 0.02f, true), p).motor == motor_cmd::coast);
}

// Car: bicycle model with servo lag and a steering trim error.
struct car
{
    float L = 0.26f, d_max_deg = 30, v = 0, r = 0, servo = 0, trim = 0;
    float step(float cmd, float dt)
    {
        servo += (cmd - servo) * dt / 0.05f;  // ~50 ms servo
        const float delta = (servo + trim) * d_max_deg / 57.29578f;
        const float target = v * std::tan(delta) / L * 57.29578f;
        r += (target - r) * dt / 0.03f;  // tyre lag
        return r;
    }
};

void test_rate_loop()
{
    params p;
    p.set(p_rev_mode, 1);
    controller c;
    car k;
    c.step(rc_in(0, 0, true), p);
    // Accelerate to a steady speed straight, then step the stick.
    control_input in = rc_in(0, 0.5f, true);
    for (int i = 0; i < 1000; ++i)
    {
        in.yaw_rate_dps = k.step(c.step(in, p).servo, k_dt);
        k.v = c.v_est();
    }
    CHECK(c.v_est() > 2.f);
    in.steer = 0.5f;
    control_output out;
    float r = 0;
    for (int i = 0; i < 250; ++i)  // 1 s
    {
        out = c.step(in, p);
        r = k.step(out.servo, k_dt);
        in.yaw_rate_dps = r;
        k.v = c.v_est();
    }
    CHECK(out.r_cmd > 10);
    CHECK_NEAR(r, out.r_cmd, 0.1f * out.r_cmd);

    // Trim error: centred stick must still drive straight.
    controller c2;
    car k2;
    k2.trim = 0.08f;
    c2.step(rc_in(0, 0, true), p);
    control_input s = rc_in(0, 0.5f, true);
    for (int i = 0; i < 1500; ++i)  // 6 s
    {
        s.yaw_rate_dps = k2.step(c2.step(s, p).servo, k_dt);
        k2.v = c2.v_est();
    }
    CHECK(std::fabs(s.yaw_rate_dps) < 2.f);

    // Standstill: direct steering (stick 1 -> full servo), no windup.
    controller c3;
    c3.step(rc_in(0, 0, true), p);
    out = run(c3, p, rc_in(1, 0, true), 2.0f);
    CHECK_NEAR(out.servo, 1.f, 1e-3f);
    out = c3.step(rc_in(0, 0, true), p);
    CHECK_NEAR(out.servo, 0.f, 1e-3f);

    // IMU gone: direct, flagged.
    auto noimu = rc_in(0.3f, 0, true);
    noimu.imu_ok = false;
    out = c3.step(noimu, p);
    CHECK(out.mode == steer_mode::direct && out.imu_degraded);
}

void test_rc_link()
{
    rc_link l;
    rc_frame f;
    f.src = k_src_udp;
    int64_t now = 0;
    uint32_t rt = 1000000;
    for (int i = 0; i < 50; ++i)
    {
        f.seq = static_cast<uint16_t>(i);
        f.t_us = rt - 20000;  // 20 ms latency
        if (i % 10 != 3)  // 10 % loss
        {
            CHECK(l.accept(f, now, rt, 300));
        }
        now += 20000;
        rt += 20000;
    }
    l.tick(now);
    CHECK(l.loss_permille() >= 90 && l.loss_permille() <= 110);
    CHECK(l.latency_p50_ms() == 20);
    // Another source is ignored while this one is fresh.
    rc_frame other = f;
    other.src = k_src_phone;
    CHECK(!l.accept(other, now, rt, 300));
    CHECK(l.accept(other, now + 400000, rt, 300));
    CHECK(l.source() == k_src_phone);
    // Duplicates rejected.
    CHECK(!l.accept(other, now + 400100, rt, 300));
}

void test_gyro_cal_and_ahrs()
{
    gyro_cal g;
    const float still[3] = {1.5f, -2.0f, 0.7f};
    for (int i = 0; i < 499; ++i)
    {
        CHECK(!g.add(still));
    }
    CHECK(g.add(still));
    CHECK_NEAR(g.bias()[1], -2.0f, 1e-4f);

    gyro_cal moving;
    for (int i = 0; i < 500; ++i)
    {
        const float m[3] = {i % 2 ? 20.f : -20.f, 0, 0};
        moving.add(m);
    }
    CHECK(!moving.done());

    ahrs a;
    const float g0[3] = {0, 0, 0};
    const float acc[3] = {0, 0.5f, 0.866f};  // rolled 30 deg (left side up)
    for (int i = 0; i < 2500; ++i)
    {
        a.update(g0, acc, 0, k_dt);
    }
    CHECK_NEAR(std::fabs(a.roll_deg()), 30.f, 1.f);
    CHECK_NEAR(a.pitch_deg(), 0.f, 1.f);
    // Level, field pointing forward + down: heading 0; on the left: 90.
    ahrs h;
    const float lvl[3] = {0, 0, 1};
    h.update(g0, lvl, 0, k_dt);
    const float north_ahead[3] = {0.2f, 0, -0.4f};
    h.mag(north_ahead, k_dt);
    CHECK(h.heading_valid());
    CHECK_NEAR(h.heading_deg(), 0.f, 0.5f);
    ahrs e;
    e.update(g0, lvl, 0, k_dt);
    const float north_left[3] = {0, 0.2f, -0.4f};
    e.mag(north_left, k_dt);
    CHECK_NEAR(e.heading_deg(), 90.f, 0.5f);
}

}  // namespace

int main()
{
    test_msp();
    test_protocol();
    test_params();
    test_arming_failsafe();
    test_throttle_esc();
    test_rate_loop();
    test_rc_link();
    test_gyro_cal_and_ahrs();
    std::printf(g_failed ? "%d FAILED\n" : "all passed\n", g_failed);
    return g_failed ? 1 : 0;
}
