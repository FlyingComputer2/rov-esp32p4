// rover_sample messages, carried in MSP v2 frames (msp.hpp) on L3 upstream 3.
// Function ids are in the private range 0x4200-0x42ff. Payloads are packed
// little-endian; layouts are in the README (RC and telemetry protocol).
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace rover_sample
{

enum : uint16_t
{
    k_func_rc = 0x4201,
    k_func_get_param = 0x4210,
    k_func_set_param = 0x4211,
    k_func_save_params = 0x4212,
    k_func_cmd = 0x4213,
    k_func_telem = 0x4220,
};

// RC sources; the rover follows one at a time (rc_link).
enum : uint8_t
{
    k_src_phone = 1,
    k_src_udp = 2,
    k_src_console = 3,
};

enum : uint8_t
{
    k_rc_flag_arm = 0x01,
};

// No mode override in an RC frame: the rover uses its `mode` parameter.
constexpr uint8_t k_rc_mode_none = 0xff;

enum : uint8_t
{
    k_cmd_gyro_cal = 1,
    k_cmd_mag_cal_start = 2,
    k_cmd_mag_cal_stop = 3,
};

struct rc_frame
{
    uint16_t seq = 0;
    uint32_t t_us = 0;  // sender CLOCK_REALTIME in us, low 32 bits
    uint8_t src = 0;
    uint8_t flags = 0;
    uint8_t mode = k_rc_mode_none;
    int16_t steer = 0;  // -1000..1000, + = right
    int16_t thr = 0;    // -1000..1000, + = forward
    int16_t aux = 0;
};
constexpr size_t k_rc_bytes = 15;

enum class state : uint8_t
{
    disarmed = 0,
    armed = 1,
    failsafe = 2,
    fault = 3,
};

enum : uint8_t
{
    k_telem_imu_ok = 0x01,
    k_telem_mag_ok = 0x02,
    k_telem_gyro_cal = 0x04,
    k_telem_bts_enabled = 0x08,
    k_telem_deadman = 0x10,
    k_telem_imu_degraded = 0x20,
    k_telem_braking = 0x40,
};

struct telem_frame
{
    uint16_t seq = 0;
    uint32_t t_us = 0;
    uint16_t rc_seq = 0;
    uint16_t rc_age_ms = 0;
    uint8_t state = 0;
    uint8_t mode = 0;
    int16_t r_cmd = 0;      // 0.1 dps
    int16_t r_meas = 0;     // 0.1 dps
    int16_t steer_out = 0;  // per mille
    int16_t thr_out = 0;    // per mille
    uint16_t heading = 0;   // 0.01 deg
    int16_t roll = 0;       // 0.01 deg
    int16_t pitch = 0;      // 0.01 deg
    int16_t ax = 0;         // mg
    int16_t ay = 0;         // mg
    uint16_t v_est = 0;     // cm/s
    uint16_t i_motor = 0;   // 10 mA
    uint16_t rc_loss = 0;   // per mille, last second
    uint16_t rc_lat_ms = 0; // p50, last second
    uint8_t flags = 0;
};
constexpr size_t k_telem_bytes = 39;

std::vector<uint8_t> pack(const rc_frame& rc);
std::optional<rc_frame> unpack_rc(const std::vector<uint8_t>& payload);
std::vector<uint8_t> pack(const telem_frame& t);
std::optional<telem_frame> unpack_telem(const std::vector<uint8_t>& payload);

// Param payloads: GET `id:u8`, SET and replies `id:u8 value:f32`.
std::vector<uint8_t> pack_param(uint8_t id, float value);
bool unpack_param(const std::vector<uint8_t>& payload, uint8_t* id, float* value);

// Saturating conversion for telemetry fields.
int16_t sat16(float v);
uint16_t usat16(float v);

}  // namespace rover_sample
