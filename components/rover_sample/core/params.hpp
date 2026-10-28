// Tuning parameters: set at run time on the console (UDP 5120) or over MSP,
// saved to NVS. Ids are part of the MSP wire and never reused.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace rover_sample
{

enum param_id : uint8_t
{
    p_servo_min_us = 1,
    p_servo_center_us = 2,
    p_servo_max_us = 3,
    p_servo_reverse = 4,
    p_servo_hz = 5,
    p_steer_max_deg = 6,
    p_wheelbase_m = 7,
    p_mode = 8,
    p_rate_max_dps = 9,
    p_a_lat_max = 10,
    p_expo = 11,
    p_kp = 12,
    p_ki = 13,
    p_kff = 14,
    p_blend_v0 = 15,
    p_blend_v1 = 16,
    p_gyro_lpf_hz = 17,
    p_imu_yaw_sign = 18,
    p_hold_kp = 19,
    p_bts_enabled = 20,
    p_bts_pwm_hz = 21,
    p_thr_max_fwd = 22,
    p_thr_max_rev = 23,
    p_thr_slew_ms = 24,
    p_thr_deadband = 25,
    p_rev_mode = 26,
    p_brake_pct = 27,
    p_i_max_a = 28,
    p_top_speed_mps = 29,
    p_tau_v_s = 30,
    p_is_div = 31,
    p_is_r_ohm = 32,
    p_failsafe_ms = 40,
    p_telem_hz = 41,
    p_mag_off_x = 50,
    p_mag_off_y = 51,
    p_mag_off_z = 52,
    p_id_end = 53,
};

enum : uint8_t
{
    k_param_integer = 0x01,
    // Only while disarmed: reconfigures a peripheral.
    k_param_disarmed_only = 0x02,
};

struct param_info
{
    param_id id;
    const char* name;
    float def;
    float min;
    float max;
    uint8_t flags;
    const char* unit;
};

const param_info* find_param(param_id id);
const param_info* find_param(const std::string& name);
const param_info* param_table(size_t* count);

class params
{
   public:
    params();
    float get(param_id id) const
    {
        return v_[id];
    }
    // Clamps to the range (rounds integers) and returns the applied value.
    float set(param_id id, float value);
    void reset();

   private:
    float v_[p_id_end] {};
};

}  // namespace rover_sample
