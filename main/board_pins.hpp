// GPIO allocation for the Waveshare ESP32-P4-WIFI6-DEV-KIT (40-pin header P6).
// Steering (LEDC timer 2) and BTS7960 (LEDC timer 3) stay off SDIO, C6 UART,
// krono GPIO20, status LED GPIO22, and board I2C GPIO7/8.
#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "rover_sample/rover_sample.hpp"

namespace rover_board
{

// --- Steering servo (SG90-class, separate 5–6 V supply, common GND) ---
constexpr gpio_num_t k_servo_pwm = GPIO_NUM_3;  // header pin 19

// --- BTS7960 / IBT-2 motor bridge (20 kHz PWM; EN tied R_EN+L_EN) ---
constexpr gpio_num_t k_bts_rpwm = GPIO_NUM_4;   // header pin 18
constexpr gpio_num_t k_bts_lpwm = GPIO_NUM_5;   // header pin 16
constexpr gpio_num_t k_bts_en = GPIO_NUM_6;     // header pin 15 (10k pulldown)
constexpr gpio_num_t k_bts_r_is = GPIO_NUM_21;  // header pin 11, ADC1
constexpr gpio_num_t k_bts_l_is = GPIO_NUM_23;  // header pin 7, ADC1

/** Actuator pins passed into rover_sample (I2C port filled by main). */
inline rover_sample::pins actuator_pins()
{
    rover_sample::pins p;
    p.servo = static_cast<int>(k_servo_pwm);
    p.bts_rpwm = static_cast<int>(k_bts_rpwm);
    p.bts_lpwm = static_cast<int>(k_bts_lpwm);
    p.bts_en = static_cast<int>(k_bts_en);
    p.bts_r_is = static_cast<int>(k_bts_r_is);
    p.bts_l_is = static_cast<int>(k_bts_l_is);
    return p;
}

}  // namespace rover_board
