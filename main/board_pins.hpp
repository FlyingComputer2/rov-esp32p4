// GPIO allocation for the Waveshare ESP32-P4-WIFI6-DEV-KIT (40-pin header P6).
// Documented in docs/board-pins.md (wiring, LEDC timers, reserved pins).
// Servos CH1–CH3 use LEDC timer 2 (ch 4/0/1); BTS7960 uses timer 3 / ch 5–7 — see
// rover_sample/src/drivers.cpp. Actuator GPIOs avoid SDIO (14–19), krono 20,
// LED 22, C6 control (46–48, 47, 54), and board I2C 7/8.
#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "rover_sample/rover_sample.hpp"

namespace rover_board
{

// --- Servos CH1–CH3 (SG90-class, separate 5–6 V supply, common GND) ---
constexpr gpio_num_t k_servo_ch1_pwm = GPIO_NUM_3;   // CH1 steering, header pin 19
constexpr gpio_num_t k_servo_ch2_pwm = GPIO_NUM_24;  // CH2 (RC aux)
constexpr gpio_num_t k_servo_ch3_pwm = GPIO_NUM_26;  // CH3 (param ch3)

// --- BTS7960 / IBT-2 motor bridge (20 kHz PWM; EN tied R_EN+L_EN) ---
constexpr gpio_num_t k_bts_rpwm = GPIO_NUM_4;   // header pin 18
constexpr gpio_num_t k_bts_lpwm = GPIO_NUM_5;   // header pin 16
constexpr gpio_num_t k_bts_en = GPIO_NUM_6;     // header pin 15 (10k pulldown)
constexpr gpio_num_t k_bts_r_is = GPIO_NUM_21;  // header pin 11, ADC1
constexpr gpio_num_t k_bts_l_is = GPIO_NUM_23;  // header pin 7, ADC1

// --- VBUS / battery divider (ADC1, high side of divider to VBUS, low to ADC) ---
constexpr gpio_num_t k_vbus_sense = GPIO_NUM_9;  // header pin 8, ADC1

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
