#include "params.hpp"

#include <cmath>

namespace rover_sample
{
namespace
{

constexpr uint8_t I = k_param_integer;
constexpr uint8_t D = k_param_disarmed_only;

/* Defaults are placeholders until the car is measured; wheelbase and
 * steering throw stay parameters, not constants. */
const param_info k_params[] = {
    {p_servo_min_us, "servo_min_us", 1100, 500, 2500, I, "us at full left"},
    {p_servo_center_us, "servo_center_us", 1500, 500, 2500, I, "us straight ahead"},
    {p_servo_max_us, "servo_max_us", 1900, 500, 2500, I, "us at full right"},
    {p_servo_reverse, "servo_reverse", 0, 0, 1, I, "0/1"},
    {p_servo_hz, "servo_hz", 50, 50, 333, I | D, "Hz"},
    {p_steer_max_deg, "steer_max_deg", 30, 5, 60, 0, "wheel deg at full throw"},
    {p_wheelbase_m, "wheelbase_m", 0.26f, 0.05f, 2, 0, "m"},
    {p_mode, "mode", 1, 0, 2, I, "0 direct, 1 rate, 2 rate+hold"},
    {p_rate_max_dps, "rate_max_dps", 360, 30, 1000, 0, "dps at full stick"},
    {p_a_lat_max, "a_lat_max", 8, 1, 30, 0, "m/s2"},
    {p_expo, "expo", 0.3f, 0, 1, 0, ""},
    {p_kp, "kp", 0.004f, 0, 0.1f, 0, "steer per dps"},
    {p_ki, "ki", 0.02f, 0, 1, 0, "steer per deg"},
    {p_kff, "kff", 1, 0, 2, 0, ""},
    {p_blend_v0, "blend_v0", 0.5f, 0, 20, 0, "m/s"},
    {p_blend_v1, "blend_v1", 1.5f, 0, 20, 0, "m/s"},
    {p_gyro_lpf_hz, "gyro_lpf_hz", 42, 5, 188, I | D, "Hz (MPU DLPF step)"},
    {p_imu_yaw_sign, "imu_yaw_sign", -1, -1, 1, I, "+1/-1, right turn = +"},
    {p_hold_kp, "hold_kp", 2, 0, 20, 0, "dps per deg"},
    {p_bts_enabled, "bts_enabled", 0, 0, 1, I | D, "0/1"},
    {p_bts_pwm_hz, "bts_pwm_hz", 20000, 1000, 25000, I | D, "Hz"},
    {p_thr_max_fwd, "thr_max_fwd", 60, 0, 100, 0, "%"},
    {p_thr_max_rev, "thr_max_rev", 40, 0, 100, 0, "%"},
    {p_thr_slew_ms, "thr_slew_ms", 300, 0, 2000, I, "ms 0 -> full"},
    {p_thr_deadband, "thr_deadband", 3, 0, 20, 0, "%"},
    {p_rev_mode, "rev_mode", 0, 0, 1, I, "0 esc, 1 direct"},
    {p_brake_pct, "brake_pct", 100, 0, 100, 0, "%"},
    {p_i_max_a, "i_max_a", 20, 1, 43, 0, "A"},
    {p_top_speed_mps, "top_speed_mps", 8, 0.5f, 40, 0, "m/s at 100%"},
    {p_tau_v_s, "tau_v_s", 0.8f, 0.05f, 5, 0, "s"},
    {p_is_div, "is_div", 2, 1, 10, 0, "IS divider ratio"},
    {p_is_r_ohm, "is_r_ohm", 1000, 100, 10000, 0, "ohm"},
    {p_failsafe_ms, "failsafe_ms", 300, 100, 2000, I, "ms"},
    {p_telem_hz, "telem_hz", 20, 1, 50, I, "Hz"},
    {p_mag_off_x, "mag_off_x", 0, -10, 10, 0, "gauss"},
    {p_mag_off_y, "mag_off_y", 0, -10, 10, 0, "gauss"},
    {p_mag_off_z, "mag_off_z", 0, -10, 10, 0, "gauss"},
};

}  // namespace

const param_info* param_table(size_t* count)
{
    *count = sizeof(k_params) / sizeof(k_params[0]);
    return k_params;
}

const param_info* find_param(param_id id)
{
    for (const auto& p : k_params)
    {
        if (p.id == id)
        {
            return &p;
        }
    }
    return nullptr;
}

const param_info* find_param(const std::string& name)
{
    for (const auto& p : k_params)
    {
        if (name == p.name)
        {
            return &p;
        }
    }
    return nullptr;
}

params::params()
{
    reset();
}

void params::reset()
{
    for (const auto& p : k_params)
    {
        v_[p.id] = p.def;
    }
}

float params::set(param_id id, float value)
{
    const param_info* p = find_param(id);
    if (!p || !std::isfinite(value))
    {
        return p ? v_[id] : 0.f;
    }
    if (p->flags & k_param_integer)
    {
        value = std::round(value);
    }
    value = std::fmin(std::fmax(value, p->min), p->max);
    if (id == p_imu_yaw_sign)
    {
        value = value < 0 ? -1.f : 1.f;
    }
    v_[id] = value;
    return value;
}

}  // namespace rover_sample
