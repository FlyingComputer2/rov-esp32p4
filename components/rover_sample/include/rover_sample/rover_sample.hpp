// rover_sample: RC car on the rover P4. Steering servo, BTS7960 motor
// bridge, GY-521 (MPU-6050) and GY-271 (QMC5883L / HMC5883L) on the board's
// shared I2C. RC in and telemetry out are MSP v2 frames over a datagram
// transport (L3 upstream 3); tuning on a UDP console.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace rover_sample
{

// One datagram per call. send: false if not taken. receive: false when
// nothing is queued (never blocks).
struct transport
{
    std::function<bool(const uint8_t* data, size_t len)> send;
    std::function<bool(std::vector<uint8_t>* out)> receive;
};

// Waveshare ESP32-P4-WIFI6-DEV-KIT 40-pin header (schematic, header P6).
struct pins
{
    // I2C master bus the application created before main() (shared with the
    // camera SCCB and the codec: GPIO7 SDA, pin 3 / GPIO8 SCL, pin 5).
    int i2c_port = 0;
    int servo = 3;      // pin 19
    int bts_rpwm = 4;   // pin 18
    int bts_lpwm = 5;   // pin 16
    int bts_en = 6;     // pin 15, R_EN + L_EN, 10k pull-down to GND
    int bts_r_is = 21;  // pin 11, ADC1, through the IS divider
    int bts_l_is = 23;  // pin 7, ADC1, through the IS divider
};

struct config
{
    transport link;
    // UDP console, one command per datagram, reply to the sender.
    std::string console = "0.0.0.0:5120";
    pins pin;
    // The 250 Hz control task (FreeRTOS); the rest runs on the caller's
    // thread and one console thread that inherits the caller's settings.
    int control_core = 1;
    int control_prio = 16;
};

// Runs until reset; returns 1 only when it cannot start (no I2C bus, bad
// console address).
int main(const config& cfg);

}  // namespace rover_sample
