// udp_ota wire: one message per UDP datagram, little-endian, no version
// field. Every reply is the request type | 0x80 and goes to the request's
// sender, so several clients can talk to one server; one OTA runs at a time.
//
//   ota_start_request      01 | u16 fragment_count
//   ota_start_response     81 | u16 fragment_count | u8 status
//   ota_fragment_request   02 | u16 fragment | data
//   ota_fragment_response  82 | u16 fragment | u8 status
//   reboot_request         03
//   reboot_response        83 | u8 status
//   status_request         04
//   status_response        84 | u8 status | text
//
// Fragments are k_fragment_size bytes; only the last may be shorter (1 to
// k_fragment_size). They may arrive in any order and more than once.
#pragma once

#include <cstddef>
#include <cstdint>

namespace udp_ota
{

constexpr size_t k_fragment_size = 1024;
constexpr uint8_t k_response = 0x80;

enum class msg : uint8_t
{
    ota_start = 0x01,
    ota_fragment = 0x02,
    reboot = 0x03,
    status = 0x04,
};

enum class status : uint8_t
{
    ok = 0,
    einprogress = 1,  // another client's OTA is running
    einval = 2,       // malformed request, fragment out of range or wrong size
    enoent = 3,       // fragment without a started OTA
    eio = 4,          // flash begin / erase / write failed
    ebadmsg = 5,      // image complete but rejected (verify or set boot)
    efbig = 6,        // fragment_count * k_fragment_size exceeds the slot
};

const char* status_name(status s);

inline uint16_t get_u16(const uint8_t* p)
{
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

inline void put_u16(uint8_t* p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

}  // namespace udp_ota
