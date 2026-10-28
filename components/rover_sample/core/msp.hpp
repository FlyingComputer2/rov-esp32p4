// MSP v2 framing, one frame per datagram:
//   '$' 'X' dir flag:u8 func:u16 len:u16 payload crc8
// little-endian, crc8 DVB-S2 over flag..payload. dir is '<' (to the rover),
// '>' (from the rover) or '!' (error reply).
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rover_sample::msp
{

constexpr size_t k_overhead = 9;
constexpr char k_to_rover = '<';
constexpr char k_from_rover = '>';
constexpr char k_error = '!';

struct frame
{
    char dir = k_to_rover;
    uint8_t flag = 0;
    uint16_t func = 0;
    std::vector<uint8_t> payload;
};

uint8_t crc8_dvb_s2(uint8_t crc, uint8_t byte);

std::vector<uint8_t> encode(char dir, uint16_t func, const uint8_t* payload, size_t len);
inline std::vector<uint8_t> encode(char dir, uint16_t func, const std::vector<uint8_t>& payload)
{
    return encode(dir, func, payload.data(), payload.size());
}

// True when data is exactly one valid frame (no trailing bytes).
bool decode(const uint8_t* data, size_t len, frame* out);

}  // namespace rover_sample::msp
