#include "msp.hpp"

namespace rover_sample::msp
{

uint8_t crc8_dvb_s2(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; ++i)
    {
        crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5) : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

std::vector<uint8_t> encode(char dir, uint16_t func, const uint8_t* payload, size_t len)
{
    std::vector<uint8_t> out;
    out.reserve(k_overhead + len);
    out.push_back('$');
    out.push_back('X');
    out.push_back(static_cast<uint8_t>(dir));
    out.push_back(0);
    out.push_back(static_cast<uint8_t>(func));
    out.push_back(static_cast<uint8_t>(func >> 8));
    out.push_back(static_cast<uint8_t>(len));
    out.push_back(static_cast<uint8_t>(len >> 8));
    out.insert(out.end(), payload, payload + len);
    uint8_t crc = 0;
    for (size_t i = 3; i < out.size(); ++i)
    {
        crc = crc8_dvb_s2(crc, out[i]);
    }
    out.push_back(crc);
    return out;
}

bool decode(const uint8_t* data, size_t len, frame* out)
{
    if (len < k_overhead || data[0] != '$' || data[1] != 'X')
    {
        return false;
    }
    const char dir = static_cast<char>(data[2]);
    if (dir != k_to_rover && dir != k_from_rover && dir != k_error)
    {
        return false;
    }
    const size_t plen = data[6] | (data[7] << 8);
    if (len != k_overhead + plen)
    {
        return false;
    }
    uint8_t crc = 0;
    for (size_t i = 3; i < len - 1; ++i)
    {
        crc = crc8_dvb_s2(crc, data[i]);
    }
    if (crc != data[len - 1])
    {
        return false;
    }
    out->dir = dir;
    out->flag = data[3];
    out->func = static_cast<uint16_t>(data[4] | (data[5] << 8));
    out->payload.assign(data + 8, data + 8 + plen);
    return true;
}

}  // namespace rover_sample::msp
