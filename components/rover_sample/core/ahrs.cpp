#include "ahrs.hpp"

#include <cmath>

#include "controller.hpp"

namespace rover_sample
{
namespace
{

constexpr float k_deg = 57.29578f;
constexpr float k_kp = 1.0f;  // Mahony accel correction gain
constexpr float k_mag_tau_s = 5.0f;

float wrap360(float d)
{
    d = std::fmod(d, 360.f);
    return d < 0 ? d + 360.f : d;
}

}  // namespace

void ahrs::reset()
{
    *this = ahrs();
}

void ahrs::update(const float g[3], const float a[3], float yaw_rate_dps, float dt)
{
    float gx = g[0] / k_deg, gy = g[1] / k_deg, gz = g[2] / k_deg;
    float& q0 = q_[0];
    float& q1 = q_[1];
    float& q2 = q_[2];
    float& q3 = q_[3];

    // Accel corrects only near 1 g (not while cornering or braking hard).
    const float an = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (an > 0.8f && an < 1.2f)
    {
        const float ax = a[0] / an, ay = a[1] / an, az = a[2] / an;
        // Estimated "up" in the body frame.
        const float vx = 2 * (q1 * q3 - q0 * q2);
        const float vy = 2 * (q0 * q1 + q2 * q3);
        const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        gx += k_kp * (ay * vz - az * vy);
        gy += k_kp * (az * vx - ax * vz);
        gz += k_kp * (ax * vy - ay * vx);
    }
    const float h = 0.5f * dt;
    const float a0 = q0, a1 = q1, a2 = q2, a3 = q3;
    q0 += (-a1 * gx - a2 * gy - a3 * gz) * h;
    q1 += (a0 * gx + a2 * gz - a3 * gy) * h;
    q2 += (a0 * gy - a1 * gz + a3 * gx) * h;
    q3 += (a0 * gz + a1 * gy - a2 * gx) * h;
    const float n = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (float& c : q_)
    {
        c /= n;
    }

    heading_ = wrap360(heading_ + yaw_rate_dps * dt);
}

void ahrs::mag(const float m[3], float dt)
{
    const float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
    // Up, forward and left in the body frame, then the field projected on
    // the horizontal plane.
    const float u[3] = {2 * (q1 * q3 - q0 * q2), 2 * (q0 * q1 + q2 * q3),
                        q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3};
    const float mu = m[0] * u[0] + m[1] * u[1] + m[2] * u[2];
    const float mh[3] = {m[0] - mu * u[0], m[1] - mu * u[1], m[2] - mu * u[2]};
    float f[3] = {1 - u[0] * u[0], -u[0] * u[1], -u[0] * u[2]};
    const float fn = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    if (fn < 0.1f)
    {
        return;  // nose straight up or down
    }
    for (float& c : f)
    {
        c /= fn;
    }
    const float l[3] = {u[1] * f[2] - u[2] * f[1], u[2] * f[0] - u[0] * f[2],
                        u[0] * f[1] - u[1] * f[0]};
    const float north_fwd = mh[0] * f[0] + mh[1] * f[1] + mh[2] * f[2];
    const float north_left = mh[0] * l[0] + mh[1] * l[1] + mh[2] * l[2];
    if (std::fabs(north_fwd) + std::fabs(north_left) < 1e-4f)
    {
        return;
    }
    // Facing east, north is on the left: heading +90.
    const float hm = wrap360(std::atan2(north_left, north_fwd) * k_deg);
    if (!mag_seen_)
    {
        mag_seen_ = true;
        heading_ = hm;
        return;
    }
    heading_ = wrap360(heading_ + wrap180(hm - heading_) * std::fmin(dt / k_mag_tau_s, 1.f));
}

float ahrs::roll_deg() const
{
    const float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
    return std::atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * k_deg;
}

float ahrs::pitch_deg() const
{
    const float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
    float s = 2 * (q0 * q2 - q3 * q1);
    s = std::fmin(std::fmax(s, -1.f), 1.f);
    return std::asin(s) * k_deg;
}

}  // namespace rover_sample
