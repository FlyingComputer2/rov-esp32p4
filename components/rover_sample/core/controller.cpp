#include "controller.hpp"

#include <cmath>

namespace rover_sample
{
namespace
{

constexpr float k_pi = 3.14159265f;
constexpr float k_deg = 180.f / k_pi;
// Stick within this to arm, and to leave failsafe.
constexpr float k_arm_thr = 0.05f;
// Failsafe brakes this long, then coasts.
constexpr float k_failsafe_brake_s = 1.0f;
// Floor on speed in the bicycle model, so feedforward stays finite.
constexpr float k_v_min = 0.3f;
// ESC reverse engages only below this speed.
constexpr float k_rev_lock_mps = 0.3f;
// Bridge off between forward and reverse drive.
constexpr float k_dir_dead_s = 0.1f;
// Heading hold engages after the stick sits within k_centre for k_hold_delay_s.
constexpr float k_centre = 0.05f;
constexpr float k_hold_delay_s = 0.2f;

float clamp1(float v)
{
    return std::fmin(std::fmax(v, -1.f), 1.f);
}

float sgn(float v)
{
    return v > 0 ? 1.f : (v < 0 ? -1.f : 0.f);
}

}  // namespace

float expo(float x, float e)
{
    x = clamp1(x);
    return (1.f - e) * x + e * x * x * x;
}

float wrap180(float deg)
{
    deg = std::fmod(deg + 180.f, 360.f);
    if (deg < 0)
    {
        deg += 360.f;
    }
    return deg - 180.f;
}

void controller::reset()
{
    *this = controller();
}

state controller::update_state(const control_input& in)
{
    const bool faulted = in.deadman || in.bts_fault;
    const bool neutral = std::fabs(in.thr) < k_arm_thr;
    if (faulted)
    {
        st_ = state::fault;
        return st_;
    }
    switch (st_)
    {
        case state::disarmed:
            if (in.rc_valid && in.arm && neutral && (in.gyro_cal || !in.imu_ok))
            {
                st_ = state::armed;
            }
            break;
        case state::armed:
            if (!in.rc_valid)
            {
                st_ = state::failsafe;
                failsafe_t_ = 0;
            }
            else if (!in.arm)
            {
                st_ = state::disarmed;
            }
            break;
        case state::failsafe:
            if (in.rc_valid && !in.arm)
            {
                st_ = state::disarmed;
            }
            else if (in.rc_valid && neutral)
            {
                st_ = state::armed;
            }
            break;
        case state::fault:
            // The operator clears a fault by disarming (or losing the link).
            if (!in.arm || !in.rc_valid)
            {
                st_ = state::disarmed;
            }
            break;
    }
    return st_;
}

control_output controller::step(const control_input& in, const params& p)
{
    control_output out;
    out.st = update_state(in);

    steer_mode want = static_cast<steer_mode>(static_cast<int>(p.get(p_mode)));
    if (in.mode_req <= static_cast<uint8_t>(steer_mode::rate_hold))
    {
        want = static_cast<steer_mode>(in.mode_req);
    }
    out.mode = want;
    if (want != steer_mode::direct && (!in.imu_ok || !in.gyro_cal))
    {
        out.mode = steer_mode::direct;
        out.imu_degraded = true;
    }

    out.servo = steer(in, p, out.mode, &out);
    throttle(in, p, &out);
    out.v_est = v_;
    return out;
}

float controller::steer(const control_input& in, const params& p, steer_mode mode,
                        control_output* out)
{
    const float dt = in.dt;
    const float r_meas = in.yaw_rate_dps;
    out->r_meas = r_meas;
    yaw_deg_ = wrap180(yaw_deg_ + r_meas * dt);

    if (st_ == state::failsafe || st_ == state::fault || !in.rc_valid)
    {
        integ_ = 0;
        holding_ = false;
        centred_t_ = 0;
        return 0;
    }
    const float stick = clamp1(in.steer);
    const float se = expo(stick, p.get(p_expo));
    if (st_ == state::disarmed || mode == steer_mode::direct)
    {
        // Disarmed: the servo follows the stick, for setup on the bench.
        integ_ = 0;
        holding_ = false;
        centred_t_ = 0;
        return se;
    }

    const float v = std::fmax(v_, 0.f);
    const float v_div = std::fmax(v, k_v_min);
    const float r_max = std::fmin(p.get(p_rate_max_dps), p.get(p_a_lat_max) / v_div * k_deg);
    float r_cmd = se * r_max;

    if (mode == steer_mode::rate_hold && std::fabs(stick) < k_centre)
    {
        centred_t_ += dt;
        if (centred_t_ >= k_hold_delay_s)
        {
            if (!holding_)
            {
                holding_ = true;
                hold_ref_ = yaw_deg_;
            }
            const float lim = r_max * 0.5f;
            r_cmd = std::fmin(std::fmax(p.get(p_hold_kp) * wrap180(hold_ref_ - yaw_deg_), -lim), lim);
        }
    }
    else
    {
        centred_t_ = 0;
        holding_ = false;
    }
    out->r_cmd = r_cmd;

    const float ff_deg = std::atan(p.get(p_wheelbase_m) * (r_cmd / k_deg) / v_div) * k_deg;
    const float ff = p.get(p_kff) * ff_deg / p.get(p_steer_max_deg);
    const float e = r_cmd - r_meas;

    // Blend: 0 = direct stick, 1 = rate loop. Reverse always steers direct.
    const float v0 = p.get(p_blend_v0), v1 = p.get(p_blend_v1);
    float alpha;
    if (v_ < 0)
    {
        alpha = 0;
    }
    else if (v1 <= v0)
    {
        alpha = v >= v1 ? 1.f : 0.f;
    }
    else
    {
        alpha = std::fmin(std::fmax((v - v0) / (v1 - v0), 0.f), 1.f);
    }

    const float pd = ff + p.get(p_kp) * e;
    if (alpha >= 1.f)
    {
        // Anti-windup: no integration that pushes a saturated output further.
        const float next = integ_ + p.get(p_ki) * e * dt;
        if (std::fabs(pd + next) < 1.f || sgn(e) != sgn(pd + integ_))
        {
            integ_ = std::fmin(std::fmax(next, -1.f), 1.f);
        }
    }
    const float rate_out = clamp1(pd + integ_);
    return clamp1((1.f - alpha) * se + alpha * rate_out);
}

void controller::throttle(const control_input& in, const params& p, control_output* out)
{
    const float dt = in.dt;
    if (st_ != state::armed)
    {
        duty_ = 0;
        dir_dead_t_ = 0;
        seen_brake_ = rev_ready_ = reversing_ = false;
        if (st_ == state::failsafe && failsafe_t_ < k_failsafe_brake_s)
        {
            failsafe_t_ += dt;
            out->motor = motor_cmd::brake;
            out->duty = 1.f;
        }
        else
        {
            out->motor = motor_cmd::coast;
            out->duty = 0;
        }
        update_v(p, dt, out->motor, out->duty);
        return;
    }

    const float db = p.get(p_thr_deadband) / 100.f;
    float t = clamp1(in.thr);
    t = std::fabs(t) <= db ? 0.f : sgn(t) * (std::fabs(t) - db) / (1.f - db);
    const float max_fwd = p.get(p_thr_max_fwd) / 100.f;
    const float max_rev = p.get(p_thr_max_rev) / 100.f;
    const bool rev_direct = p.get(p_rev_mode) >= 1.f;

    motor_cmd cmd = motor_cmd::drive;
    float target = 0;
    float brake = 0;
    if (t > 0)
    {
        seen_brake_ = rev_ready_ = reversing_ = false;
        target = t * max_fwd;
    }
    else if (t < 0)
    {
        if (rev_direct || reversing_)
        {
            target = t * max_rev;
        }
        else if (rev_ready_ && v_ < k_rev_lock_mps)
        {
            reversing_ = true;
            target = t * max_rev;
        }
        else
        {
            seen_brake_ = true;
            cmd = motor_cmd::brake;
            brake = -t * p.get(p_brake_pct) / 100.f;
        }
    }
    else if (seen_brake_)
    {
        rev_ready_ = true;
    }

    if (cmd == motor_cmd::brake)
    {
        duty_ = 0;
        dir_dead_t_ = 0;
        out->motor = motor_cmd::brake;
        out->duty = brake;
        update_v(p, dt, out->motor, brake);
        return;
    }

    if (duty_ != 0 && target != 0 && sgn(target) != sgn(duty_))
    {
        duty_ = 0;
        dir_dead_t_ = k_dir_dead_s;
    }
    if (dir_dead_t_ > 0)
    {
        dir_dead_t_ -= dt;
        target = 0;
    }
    const float slew_ms = p.get(p_thr_slew_ms);
    const float step = slew_ms > 0 ? dt * 1000.f / slew_ms : 1.f;
    if (std::fabs(target) > std::fabs(duty_))
    {
        duty_ += std::fmin(std::fmax(target - duty_, -step), step);
    }
    else
    {
        duty_ = target;
    }

    const float i_max = p.get(p_i_max_a);
    if (in.current_a > i_max)
    {
        i_scale_ -= 5.f * (in.current_a - i_max) / i_max * dt;
    }
    else
    {
        i_scale_ += 0.5f * dt;
    }
    i_scale_ = std::fmin(std::fmax(i_scale_, 0.1f), 1.f);

    out->duty = duty_ * i_scale_;
    out->motor = out->duty == 0 ? motor_cmd::coast : motor_cmd::drive;
    update_v(p, dt, out->motor, out->duty);
}

void controller::update_v(const params& p, float dt, motor_cmd cmd, float duty)
{
    float target = 0;
    float tau = p.get(p_tau_v_s);
    if (cmd == motor_cmd::drive)
    {
        target = p.get(p_top_speed_mps) * duty;
    }
    else if (cmd == motor_cmd::brake)
    {
        tau /= 4.f * std::fmax(duty, 0.25f);
    }
    else
    {
        tau *= 3.f;
    }
    v_ += (target - v_) * std::fmin(dt / tau, 1.f);
}

void gyro_cal::start()
{
    *this = gyro_cal();
}

bool gyro_cal::add(const float g[3])
{
    if (done_)
    {
        return true;
    }
    constexpr int k_samples = 500;  // 2 s at 250 Hz
    constexpr double k_max_std_dps = 0.5;
    constexpr double k_max_bias_dps = 30;
    for (int i = 0; i < 3; ++i)
    {
        sum_[i] += g[i];
        sq_[i] += static_cast<double>(g[i]) * g[i];
    }
    if (++n_ < k_samples)
    {
        return false;
    }
    bool still = true;
    for (int i = 0; i < 3; ++i)
    {
        const double mean = sum_[i] / n_;
        const double var = sq_[i] / n_ - mean * mean;
        still = still && var < k_max_std_dps * k_max_std_dps && std::fabs(mean) < k_max_bias_dps;
        bias_[i] = static_cast<float>(mean);
    }
    if (!still)
    {
        start();
        return false;
    }
    done_ = true;
    return true;
}

void gyro_cal::track(const float g[3])
{
    if (!done_)
    {
        return;
    }
    for (int i = 0; i < 3; ++i)
    {
        if (std::fabs(g[i] - bias_[i]) > 1.5f)
        {
            return;
        }
    }
    for (int i = 0; i < 3; ++i)
    {
        bias_[i] += 0.002f * (g[i] - bias_[i]);
    }
}

}  // namespace rover_sample
