// rover_sample: RC car on the rover P4. Steering servo, BTS7960 motor
// bridge, GY-521 (MPU-6050) and GY-271 (QMC5883L / HMC5883L) on the board's
// shared I2C. RC in and telemetry out are MSP v2 frames over a datagram
// transport (L3 upstream 3); tuning on a UDP console. The RC link and the
// console run on a named app reactor (app_kit); only the 250 Hz control loop
// has a task of its own.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
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
    // I2C master bus the host created before applet init (shared with the
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
    pins pin;
    // The app reactor (app_kit) the RC link and the console run on.
    std::string reactor = "apps";
    // UDP console, one command per datagram, reply to the sender.
    std::string console = "0.0.0.0:5120";
    // RC link poll period: RC arrives at 50 Hz; the failsafe and the console
    // RC hold count on it.
    uint32_t link_poll_ms = 2;
    // The 250 Hz control task (FreeRTOS, PSRAM stack).
    int control_core = 1;
    int control_prio = 16;
};

class Applet
{
public:
    Applet();
    ~Applet();
    Applet(const Applet&) = delete;
    Applet& operator=(const Applet&) = delete;
    bool configure(const config& cfg);
    bool start();
    void stop();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rover_sample
