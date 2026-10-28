#include "protocol.hpp"

#include <cmath>
#include <cstring>

namespace rover_sample
{
namespace
{

struct writer
{
    std::vector<uint8_t> out;
    void u8(uint8_t v)
    {
        out.push_back(v);
    }
    void u16(uint16_t v)
    {
        u8(static_cast<uint8_t>(v));
        u8(static_cast<uint8_t>(v >> 8));
    }
    void u32(uint32_t v)
    {
        u16(static_cast<uint16_t>(v));
        u16(static_cast<uint16_t>(v >> 16));
    }
};

struct reader
{
    const std::vector<uint8_t>& in;
    size_t pos = 0;
    uint8_t u8()
    {
        return in[pos++];
    }
    uint16_t u16()
    {
        const uint16_t lo = u8();
        return static_cast<uint16_t>(lo | (u8() << 8));
    }
    uint32_t u32()
    {
        const uint32_t lo = u16();
        return lo | (static_cast<uint32_t>(u16()) << 16);
    }
    int16_t i16()
    {
        return static_cast<int16_t>(u16());
    }
};

}  // namespace

std::vector<uint8_t> pack(const rc_frame& rc)
{
    writer w;
    w.u16(rc.seq);
    w.u32(rc.t_us);
    w.u8(rc.src);
    w.u8(rc.flags);
    w.u8(rc.mode);
    w.u16(static_cast<uint16_t>(rc.steer));
    w.u16(static_cast<uint16_t>(rc.thr));
    w.u16(static_cast<uint16_t>(rc.aux));
    return w.out;
}

std::optional<rc_frame> unpack_rc(const std::vector<uint8_t>& payload)
{
    if (payload.size() != k_rc_bytes)
    {
        return std::nullopt;
    }
    reader r {payload};
    rc_frame rc;
    rc.seq = r.u16();
    rc.t_us = r.u32();
    rc.src = r.u8();
    rc.flags = r.u8();
    rc.mode = r.u8();
    rc.steer = r.i16();
    rc.thr = r.i16();
    rc.aux = r.i16();
    if (rc.steer < -1000 || rc.steer > 1000 || rc.thr < -1000 || rc.thr > 1000)
    {
        return std::nullopt;
    }
    return rc;
}

std::vector<uint8_t> pack(const telem_frame& t)
{
    writer w;
    w.u16(t.seq);
    w.u32(t.t_us);
    w.u16(t.rc_seq);
    w.u16(t.rc_age_ms);
    w.u8(t.state);
    w.u8(t.mode);
    w.u16(static_cast<uint16_t>(t.r_cmd));
    w.u16(static_cast<uint16_t>(t.r_meas));
    w.u16(static_cast<uint16_t>(t.steer_out));
    w.u16(static_cast<uint16_t>(t.thr_out));
    w.u16(t.heading);
    w.u16(static_cast<uint16_t>(t.roll));
    w.u16(static_cast<uint16_t>(t.pitch));
    w.u16(static_cast<uint16_t>(t.ax));
    w.u16(static_cast<uint16_t>(t.ay));
    w.u16(t.v_est);
    w.u16(t.i_motor);
    w.u16(t.rc_loss);
    w.u16(t.rc_lat_ms);
    w.u8(t.flags);
    return w.out;
}

std::optional<telem_frame> unpack_telem(const std::vector<uint8_t>& payload)
{
    if (payload.size() != k_telem_bytes)
    {
        return std::nullopt;
    }
    reader r {payload};
    telem_frame t;
    t.seq = r.u16();
    t.t_us = r.u32();
    t.rc_seq = r.u16();
    t.rc_age_ms = r.u16();
    t.state = r.u8();
    t.mode = r.u8();
    t.r_cmd = r.i16();
    t.r_meas = r.i16();
    t.steer_out = r.i16();
    t.thr_out = r.i16();
    t.heading = r.u16();
    t.roll = r.i16();
    t.pitch = r.i16();
    t.ax = r.i16();
    t.ay = r.i16();
    t.v_est = r.u16();
    t.i_motor = r.u16();
    t.rc_loss = r.u16();
    t.rc_lat_ms = r.u16();
    t.flags = r.u8();
    return t;
}

std::vector<uint8_t> pack_param(uint8_t id, float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    writer w;
    w.u8(id);
    w.u32(bits);
    return w.out;
}

bool unpack_param(const std::vector<uint8_t>& payload, uint8_t* id, float* value)
{
    if (payload.size() != 5)
    {
        return false;
    }
    reader r {payload};
    *id = r.u8();
    const uint32_t bits = r.u32();
    std::memcpy(value, &bits, sizeof(bits));
    return std::isfinite(*value);
}

int16_t sat16(float v)
{
    if (!(v > -32768.f))
    {
        return -32768;
    }
    if (v > 32767.f)
    {
        return 32767;
    }
    return static_cast<int16_t>(std::lround(v));
}

uint16_t usat16(float v)
{
    if (!(v > 0.f))
    {
        return 0;
    }
    if (v > 65535.f)
    {
        return 65535;
    }
    return static_cast<uint16_t>(std::lround(v));
}

}  // namespace rover_sample
