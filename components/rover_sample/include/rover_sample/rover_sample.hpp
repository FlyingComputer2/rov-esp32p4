// rover_sample: RC car on the rover P4. Steering servo, BTS7960 motor
// bridge, GY-521 (MPU-6050) and GY-271 (QMC5883L / HMC5883L) on the board's
// shared I2C. RC in and telemetry out are MSP v2 frames over a datagram
// transport (L3 upstream 3); tuning on a UDP console. The RC link and the
// console run on a named reactor (bfc::default_reactor_registry); only the 250 Hz control loop
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

// Actuator and sensor wiring. Defaults match main/board_pins.hpp on the
// Waveshare board; override per instance when testing on other harnesses.
struct pins
{
    // I2C master bus the host created before applet init (shared with the
    // camera SCCB and the codec: GPIO7 SDA, pin 3 / GPIO8 SCL, pin 5).
    int i2c_port = 0;
    /** CH1 steering servo PWM GPIO; -1 disables. */
    int servo = 3;
    /** CH2 / CH3 auxiliary servo PWM; -1 disables. */
    int servo_ch2 = -1;
    int servo_ch3 = -1;
    int bts_rpwm = 4;
    int bts_lpwm = 5;
    int bts_en = 6;
    int bts_r_is = 21;
    int bts_l_is = 23;
    /** VBUS sense ADC GPIO (any ADC unit); -1 disables. */
    int vbus_adc = -1;
};

struct config
{
    // The RC / telemetry upstream: an L3 QUEUE_FORWARDING upstream on two
    // named queues in bfc's queue registry (its rx = link_to_l3, tx =
    // link_from_l3).
    std::string link_to_l3;
    std::string link_from_l3;
    pins pin;
    // The reactor (bfc::default_reactor_registry) the RC link and the console
    // run on; start() registers on it before its thread runs.
    std::string reactor;
    // UDP console, one command per datagram, reply to the sender.
    std::string console = "0.0.0.0:5120";
    // RC frames are handled as they arrive (50 Hz); this tick runs the
    // failsafe, the console RC hold (50 Hz) and telemetry (telem_hz, 20 Hz).
    // Each tick is a reactor wake-up (a poll() pass on ESP-IDF): at 10 ms the
    // reactor took 5.4% of a core.
    uint32_t link_tick_ms = 20;
    // The 250 Hz control task (FreeRTOS, PSRAM stack).
    int control_core = 1;
    int control_prio = 16;
};

// rover_sample as an applet: configure() loads the parameters and finds the
// reactor; start() starts the control task and registers the link poll and
// the console on the reactor. Lives for the life of the program (no stop).
class Applet
{
public:
    Applet();
    ~Applet();
    Applet(const Applet&) = delete;
    Applet& operator=(const Applet&) = delete;
    // False when there is no I2C bus or no such reactor (once only).
    bool configure(const config& cfg);
    // False when the control task or the console cannot be started.
    bool start();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rover_sample
