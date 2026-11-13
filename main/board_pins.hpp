// GPIO allocation for the Waveshare ESP32-P4-WIFI6-DEV-KIT (40-pin header P6).
// Documented in docs/board-pins.md (wiring, LEDC timers, reserved pins).
// Servos CH1–CH3 use LEDC timer 2 (ch 4/0/1); BTS7960 uses timer 3 / ch 5–7 — see
// rover_sample/src/drivers.cpp. Actuator GPIOs avoid SDIO (14–19), krono 20,
// LED 22, C6 control (46–48, 47, 54), board I2C 7/8, the USB1.1 Type-C
// USB-Serial-JTAG (24/25), the 32 kHz crystal (0/1), the C6 UART (32/33) and
// the TF power switch (45). Header pin numbers as in the README (pin 1 = 3V3).
#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "rover_sample/rover_sample.hpp"

namespace rover_board
{

// --- Servos CH1–CH3 (SG90-class, separate 5–6 V supply, common GND) ---
constexpr gpio_num_t k_servo_ch1_pwm = GPIO_NUM_3;   // CH1 steering, header pin 19
constexpr gpio_num_t k_servo_ch2_pwm = GPIO_NUM_2;   // CH2 (RC aux), header pin 21
constexpr gpio_num_t k_servo_ch3_pwm = GPIO_NUM_26;  // CH3 (RC aux2), header pin 31

// --- BTS7960 / IBT-2 motor bridge (20 kHz PWM; EN tied R_EN+L_EN) ---
constexpr gpio_num_t k_bts_rpwm = GPIO_NUM_4;   // header pin 18
constexpr gpio_num_t k_bts_lpwm = GPIO_NUM_5;   // header pin 16
constexpr gpio_num_t k_bts_en = GPIO_NUM_6;     // header pin 15 (10k pulldown)
// Current sense: not wired on this car. Unconnected pins float, and a floating IS
// reading trips the 4 V overload fault. To use it, wire R_IS / L_IS through the
// divider to GPIO21 (header 11) / GPIO23 (header 7), both ADC1, and set them here.
constexpr gpio_num_t k_bts_r_is = GPIO_NUM_NC;
constexpr gpio_num_t k_bts_l_is = GPIO_NUM_NC;

// --- VBUS / battery divider (high side to VBUS, tap to the pin, low side to GND) ---
// GPIO53 = ADC2 ch4, header pin 35. On this board it is also PA_CTRL (0R R73 into
// the speaker amp's 74LVC1G08 enable gate): a high-Z input, so the reading is
// fine, but the amp may sit enabled (silent) while the bus is up. The only other
// ADC pins on the header are 20 (krono), 21/23 (BTS IS) and 22 (LED).
constexpr gpio_num_t k_vbus_sense = GPIO_NUM_53;

/** Actuator pins passed into rover_sample (I2C port filled by main). */
inline rover_sample::pins actuator_pins()
{
    rover_sample::pins p;
    p.servo = static_cast<int>(k_servo_ch1_pwm);
    p.servo_ch2 = static_cast<int>(k_servo_ch2_pwm);
    p.servo_ch3 = static_cast<int>(k_servo_ch3_pwm);
    p.bts_rpwm = static_cast<int>(k_bts_rpwm);
    p.bts_lpwm = static_cast<int>(k_bts_lpwm);
    p.bts_en = static_cast<int>(k_bts_en);
    p.bts_r_is = static_cast<int>(k_bts_r_is);
    p.bts_l_is = static_cast<int>(k_bts_l_is);
    p.vbus_adc = static_cast<int>(k_vbus_sense);
    return p;
}

}  // namespace rover_board
