// The car's control law, one step per control cycle, no I/O:
//  - arming / failsafe / fault state machine
//  - steering: yaw-rate loop (bicycle-model feedforward + PI on the gyro),
//    blended to direct stick steering at low speed, optional heading hold
//  - throttle: ESC-style forward / brake / reverse with slew and current limit
//  - v_est: first-order motor model on the applied duty (no wheel sensor)
#pragma once

#include <cstdint>

#include "params.hpp"
#include "protocol.hpp"

namespace rover_sample
{

enum class steer_mode : uint8_t
{
    direct = 0,
    rate = 1,
    rate_hold = 2,
};

enum class motor_cmd : uint8_t
{
    coast = 0,  // bridge off (EN low)
    drive = 1,  // duty signed: + forward
    brake = 2,  // both low sides on, strength = |duty|
};

struct control_input
{
    float dt = 0.004f;  // s
    bool rc_valid = false;  // a frame within failsafe_ms
    bool rc_ever = false;   // any frame since boot
    float steer = 0;  // -1..1
    float thr = 0;    // -1..1
    bool arm = false;
    uint8_t mode_req = k_rc_mode_none;
    bool imu_ok = false;
    bool gyro_cal = false;
    float yaw_rate_dps = 0;  // bias removed, + = right turn
    float current_a = 0;
    bool bts_fault = false;
    bool deadman = false;
};

struct control_output
{
    state st = state::disarmed;
    steer_mode mode = steer_mode::direct;
    bool imu_degraded = false;
    float servo = 0;  // -1..1, + = right
    motor_cmd motor = motor_cmd::coast;
    float duty = 0;   // -1..1
    float r_cmd = 0;  // dps
    float r_meas = 0; // dps
    float v_est = 0;  // m/s, - in reverse
};

class controller
{
   public:
    control_output step(const control_input& in, const params& p);
    void reset();
    float v_est() const
    {
        return v_;
    }
    state current_state() const
    {
        return st_;
    }

   private:
    state update_state(const control_input& in);
    float steer(const control_input& in, const params& p, steer_mode mode, control_output* out);
    void throttle(const control_input& in, const params& p, control_output* out);
    void update_v(const params& p, float dt, motor_cmd cmd, float duty);

    state st_ = state::disarmed;
    float failsafe_t_ = 0;  // s in failsafe
    // steering
    float integ_ = 0;
    float yaw_deg_ = 0;  // integrated gyro heading for hold
    float hold_ref_ = 0;
    float centred_t_ = 0;
    bool holding_ = false;
    // throttle
    float duty_ = 0;  // applied, signed
    float dir_dead_t_ = 0;
    bool seen_brake_ = false;
    bool rev_ready_ = false;
    bool reversing_ = false;
    float i_scale_ = 1;
    float v_ = 0;
};

// Gyro bias: averages 2 s of still samples; restarts on movement. While
// disarmed and still, track() follows slow drift.
class gyro_cal
{
   public:
    void start();
    // Returns true while calibrated.
    bool add(const float g_dps[3]);
    void track(const float g_dps[3]);
    bool done() const
    {
        return done_;
    }
    const float* bias() const
    {
        return bias_;
    }

   private:
    bool done_ = false;
    int n_ = 0;
    double sum_[3] {};
    double sq_[3] {};
    float bias_[3] {};
};

float expo(float x, float e);
float wrap180(float deg);

}  // namespace rover_sample
