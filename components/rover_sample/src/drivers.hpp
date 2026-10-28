// Peripheral drivers for rover_sample. All of them are used from the control
// task only (the deadman ISR excepted).
#pragma once

#include <atomic>
#include <cstdint>

#include "driver/gptimer.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"

#include "controller.hpp"

namespace rover_sample
{

// MPU-6050 (GY-521) at 0x68 / 0x69: gyro +-500 dps, accel +-4 g, 1 kHz.
class mpu6050
{
   public:
    bool probe_and_init(i2c_master_bus_handle_t bus, int lpf_hz);
    bool set_lpf(int lpf_hz);
    // g in dps, a in g, sensor axes.
    bool read(float g[3], float a[3]);
    bool present() const
    {
        return dev_ != nullptr;
    }
    uint8_t whoami() const
    {
        return who_;
    }

   private:
    bool write(uint8_t reg, uint8_t val);
    i2c_master_dev_handle_t dev_ = nullptr;
    uint8_t who_ = 0;
};

// GY-271: QMC5883L at 0x0d, or HMC5883L at 0x1e.
class magnetometer
{
   public:
    bool probe_and_init(i2c_master_bus_handle_t bus);
    // Gauss, sensor axes; false without a new sample.
    bool read(float m[3]);
    bool present() const
    {
        return dev_ != nullptr;
    }
    const char* kind() const
    {
        return qmc_ ? "qmc5883l" : (dev_ ? "hmc5883l" : "none");
    }

   private:
    i2c_master_dev_handle_t dev_ = nullptr;
    bool qmc_ = false;
};

// Steering servo on LEDC timer 2 / channel 4.
class servo
{
   public:
    bool init(int gpio, int hz);
    bool set_hz(int hz);
    void set_us(float us);
    float us() const
    {
        return us_;
    }

   private:
    int hz_ = 50;
    float us_ = 0;
    bool ok_ = false;
};

// BTS7960 (IBT-2): RPWM / LPWM / EN on LEDC timer 3 / channels 5-7, R_IS /
// L_IS on ADC1. EN is the deadman's cut-off.
class bts7960
{
   public:
    bool init(int rpwm, int lpwm, int en, int r_is, int l_is, int pwm_hz);
    void deinit();
    bool enabled() const
    {
        return ok_;
    }
    void apply(motor_cmd cmd, float duty);
    // Bridge current from IS, in A; is_div is the divider ratio, r_is the
    // module's IS resistor. Also tracks IS overload (fault()).
    float read_current(float is_div, float r_is_ohm, float dt);
    bool fault() const
    {
        return overload_t_ >= 0.05f;
    }
    // Puts EN back on LEDC after the deadman cut it.
    void reattach();

   private:
    void duty(int ch, float d);
    bool ok_ = false;
    int en_ = -1;
    int pins_[3] {-1, -1, -1};
    adc_oneshot_unit_handle_t adc_ = nullptr;
    adc_cali_handle_t cali_ = nullptr;
    int ch_r_ = -1, ch_l_ = -1;
    float overload_t_ = 0;
};

// One-shot GPTimer; kick() every control cycle. If 50 ms pass without one,
// the ISR forces the BTS EN pin low and latches tripped().
class deadman
{
   public:
    bool init(int en_gpio);
    void kick();
    bool tripped() const
    {
        return tripped_.load();
    }
    void clear();
    void set_en_gpio(int gpio)
    {
        en_gpio_.store(gpio);
    }

   private:
    static bool on_alarm(gptimer_handle_t, const gptimer_alarm_event_data_t*, void* arg);
    gptimer_handle_t timer_ = nullptr;
    std::atomic<int> en_gpio_ {-1};
    std::atomic<bool> tripped_ {false};
};

}  // namespace rover_sample
