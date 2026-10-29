// Attitude for telemetry: Mahony gyro + accel for roll / pitch, and a
// heading that integrates the yaw rate and is pulled toward the
// tilt-compensated magnetometer (time constant 5 s). Body frame: x forward,
// y left, z up (GY-521 flat, x arrow forward); the magnetometer uses the
// same axes.
#pragma once

namespace rover_sample
{

class ahrs
{
   public:
    // g in dps (bias removed), a in g; yaw_rate_dps + = right turn.
    void update(const float g_dps[3], const float a_g[3], float yaw_rate_dps, float dt);
    // m in gauss, hard-iron offsets removed.
    void mag(const float m[3], float dt);
    void reset();

    float roll_deg() const;
    float pitch_deg() const;
    // 0..360, clockwise from magnetic north; gyro-only until the first
    // magnetometer sample.
    float heading_deg() const
    {
        return heading_;
    }
    bool heading_valid() const
    {
        return mag_seen_;
    }

   private:
    float q_[4] {1, 0, 0, 0};
    float heading_ = 0;
    bool mag_seen_ = false;
};

}  // namespace rover_sample
