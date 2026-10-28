#include "drivers.hpp"

#include <cmath>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_sig_map.h"

namespace rover_sample
{
namespace
{

const char* TAG = "rover_sample";

/* Short timeouts: a slow transfer must not starve the 4 ms loop past the
 * deadman. The bus is shared with the camera SCCB, whose transfers are a few
 * bytes at 100 kHz. */
constexpr int k_i2c_timeout_ms = 10;
constexpr uint32_t k_i2c_hz = 400000;

i2c_master_dev_handle_t add_device(i2c_master_bus_handle_t bus, uint8_t addr)
{
    if (i2c_master_probe(bus, addr, k_i2c_timeout_ms) != ESP_OK)
    {
        return nullptr;
    }
    i2c_device_config_t cfg = {};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address = addr;
    cfg.scl_speed_hz = k_i2c_hz;
    i2c_master_dev_handle_t dev = nullptr;
    if (i2c_master_bus_add_device(bus, &cfg, &dev) != ESP_OK)
    {
        return nullptr;
    }
    return dev;
}

bool reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    const uint8_t b[2] = {reg, val};
    return i2c_master_transmit(dev, b, 2, k_i2c_timeout_ms) == ESP_OK;
}

bool reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* out, size_t n)
{
    return i2c_master_transmit_receive(dev, &reg, 1, out, n, k_i2c_timeout_ms) == ESP_OK;
}

int16_t be16(const uint8_t* p)
{
    return static_cast<int16_t>((p[0] << 8) | p[1]);
}

int16_t le16(const uint8_t* p)
{
    return static_cast<int16_t>((p[1] << 8) | p[0]);
}

/* LEDC: the P4 has low-speed mode only. Timer 2 servo, timer 3 BTS; channels
 * 4-7. Timers / channels 0-1 / 0-3 are left free. */
constexpr ledc_mode_t k_mode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t k_servo_timer = LEDC_TIMER_2;
constexpr ledc_channel_t k_servo_ch = LEDC_CHANNEL_4;
constexpr ledc_timer_bit_t k_servo_bits = LEDC_TIMER_14_BIT;
constexpr ledc_timer_t k_bts_timer = LEDC_TIMER_3;
constexpr ledc_channel_t k_bts_ch[3] = {LEDC_CHANNEL_5, LEDC_CHANNEL_6, LEDC_CHANNEL_7};
constexpr ledc_timer_bit_t k_bts_bits = LEDC_TIMER_10_BIT;
constexpr int k_r = 0, k_l = 1, k_en = 2;

bool ledc_timer(ledc_timer_t t, ledc_timer_bit_t bits, int hz)
{
    ledc_timer_config_t cfg = {};
    cfg.speed_mode = k_mode;
    cfg.timer_num = t;
    cfg.duty_resolution = bits;
    cfg.freq_hz = static_cast<uint32_t>(hz);
    cfg.clk_cfg = LEDC_AUTO_CLK;
    return ledc_timer_config(&cfg) == ESP_OK;
}

bool ledc_channel(ledc_channel_t ch, ledc_timer_t t, int gpio)
{
    ledc_channel_config_t cfg = {};
    cfg.gpio_num = gpio;
    cfg.speed_mode = k_mode;
    cfg.channel = ch;
    cfg.timer_sel = t;
    cfg.duty = 0;
    return ledc_channel_config(&cfg) == ESP_OK;
}

/* MPU-6050 DLPF (CONFIG register) steps: bandwidth Hz -> DLPF_CFG. */
uint8_t dlpf_cfg(int hz)
{
    if (hz >= 188) return 1;
    if (hz >= 98) return 2;
    if (hz >= 42) return 3;
    if (hz >= 20) return 4;
    if (hz >= 10) return 5;
    return 6;
}

}  // namespace

bool mpu6050::write(uint8_t reg, uint8_t val)
{
    return reg_write(dev_, reg, val);
}

bool mpu6050::probe_and_init(i2c_master_bus_handle_t bus, int lpf_hz)
{
    for (uint8_t addr : {uint8_t(0x68), uint8_t(0x69)})
    {
        dev_ = add_device(bus, addr);
        if (!dev_)
        {
            continue;
        }
        uint8_t who = 0;
        // 0x68 MPU-6050, 0x70 MPU-6500, 0x72/0x98 clones.
        if (reg_read(dev_, 0x75, &who, 1) && (who == 0x68 || who == 0x70 || who == 0x72 || who == 0x98))
        {
            who_ = who;
            break;
        }
        i2c_master_bus_rm_device(dev_);
        dev_ = nullptr;
    }
    if (!dev_)
    {
        return false;
    }
    bool ok = write(0x6b, 0x80);  // PWR_MGMT_1: reset
    vTaskDelay(pdMS_TO_TICKS(100));
    ok = ok && write(0x6b, 0x01)   // clock: PLL on gyro X
         && write(0x19, 0x00)      // SMPLRT_DIV: 1 kHz with DLPF
         && set_lpf(lpf_hz)        // CONFIG
         && write(0x1b, 0x08)      // GYRO_CONFIG: +-500 dps
         && write(0x1c, 0x08);     // ACCEL_CONFIG: +-4 g
    if (!ok)
    {
        i2c_master_bus_rm_device(dev_);
        dev_ = nullptr;
        return false;
    }
    ESP_LOGI(TAG, "MPU whoami 0x%02x", who_);
    return true;
}

bool mpu6050::set_lpf(int lpf_hz)
{
    return dev_ && write(0x1a, dlpf_cfg(lpf_hz));
}

bool mpu6050::read(float g[3], float a[3])
{
    uint8_t b[14];
    if (!dev_ || !reg_read(dev_, 0x3b, b, sizeof(b)))
    {
        return false;
    }
    for (int i = 0; i < 3; ++i)
    {
        a[i] = be16(b + 2 * i) / 8192.f;
        g[i] = be16(b + 8 + 2 * i) / 65.5f;
    }
    return true;
}

bool magnetometer::probe_and_init(i2c_master_bus_handle_t bus)
{
    dev_ = add_device(bus, 0x0d);
    if (dev_)
    {
        qmc_ = true;
        // SET/RESET period, then OSR 512, 8 G, 100 Hz, continuous.
        if (reg_write(dev_, 0x0b, 0x01) && reg_write(dev_, 0x09, 0x19))
        {
            return true;
        }
        i2c_master_bus_rm_device(dev_);
        dev_ = nullptr;
    }
    qmc_ = false;
    dev_ = add_device(bus, 0x1e);
    if (dev_)
    {
        uint8_t id[3] = {};
        // 8-sample average, 75 Hz; +-1.3 Ga; continuous.
        if (reg_read(dev_, 0x0a, id, 3) && id[0] == 'H' && reg_write(dev_, 0x00, 0x78) &&
            reg_write(dev_, 0x01, 0x20) && reg_write(dev_, 0x02, 0x00))
        {
            return true;
        }
        i2c_master_bus_rm_device(dev_);
        dev_ = nullptr;
    }
    return false;
}

bool magnetometer::read(float m[3])
{
    uint8_t b[7];
    if (!dev_)
    {
        return false;
    }
    if (qmc_)
    {
        if (!reg_read(dev_, 0x00, b, 7) || !(b[6] & 0x01))  // status DRDY
        {
            return false;
        }
        for (int i = 0; i < 3; ++i)
        {
            m[i] = le16(b + 2 * i) / 3000.f;
        }
        return true;
    }
    if (!reg_read(dev_, 0x03, b, 6))
    {
        return false;
    }
    m[0] = be16(b) / 1090.f;      // X
    m[2] = be16(b + 2) / 1090.f;  // Z
    m[1] = be16(b + 4) / 1090.f;  // Y
    return true;
}

bool servo::init(int gpio, int hz)
{
    hz_ = hz;
    ok_ = ledc_timer(k_servo_timer, k_servo_bits, hz) && ledc_channel(k_servo_ch, k_servo_timer, gpio);
    return ok_;
}

bool servo::set_hz(int hz)
{
    if (!ok_)
    {
        return false;
    }
    if (ledc_set_freq(k_mode, k_servo_timer, static_cast<uint32_t>(hz)) != ESP_OK)
    {
        return false;
    }
    hz_ = hz;
    set_us(us_);
    return true;
}

void servo::set_us(float us)
{
    us_ = us;
    if (!ok_)
    {
        return;
    }
    const float period_us = 1e6f / hz_;
    const uint32_t duty = static_cast<uint32_t>(us / period_us * ((1u << k_servo_bits) - 1));
    ledc_set_duty(k_mode, k_servo_ch, duty);
    ledc_update_duty(k_mode, k_servo_ch);
}

bool bts7960::init(int rpwm, int lpwm, int en, int r_is, int l_is, int pwm_hz)
{
    pins_[k_r] = rpwm;
    pins_[k_l] = lpwm;
    pins_[k_en] = en;
    en_ = en;
    ok_ = ledc_timer(k_bts_timer, k_bts_bits, pwm_hz);
    for (int i = 0; i < 3 && ok_; ++i)
    {
        ok_ = ledc_channel(k_bts_ch[i], k_bts_timer, pins_[i]);
    }
    if (!ok_)
    {
        ESP_LOGE(TAG, "BTS7960 LEDC setup failed");
        return false;
    }

    adc_unit_t unit_r, unit_l;
    adc_channel_t ch_r, ch_l;
    if (adc_oneshot_io_to_channel(r_is, &unit_r, &ch_r) == ESP_OK &&
        adc_oneshot_io_to_channel(l_is, &unit_l, &ch_l) == ESP_OK && unit_r == unit_l)
    {
        adc_oneshot_unit_init_cfg_t ucfg = {};
        ucfg.unit_id = unit_r;
        if (adc_oneshot_new_unit(&ucfg, &adc_) == ESP_OK)
        {
            adc_oneshot_chan_cfg_t ccfg = {};
            ccfg.atten = ADC_ATTEN_DB_12;
            ccfg.bitwidth = ADC_BITWIDTH_DEFAULT;
            adc_oneshot_config_channel(adc_, ch_r, &ccfg);
            adc_oneshot_config_channel(adc_, ch_l, &ccfg);
            ch_r_ = ch_r;
            ch_l_ = ch_l;
            adc_cali_curve_fitting_config_t cal = {};
            cal.unit_id = unit_r;
            cal.atten = ADC_ATTEN_DB_12;
            cal.bitwidth = ADC_BITWIDTH_DEFAULT;
            if (adc_cali_create_scheme_curve_fitting(&cal, &cali_) != ESP_OK)
            {
                cali_ = nullptr;
            }
        }
    }
    if (!adc_)
    {
        ESP_LOGW(TAG, "BTS7960 IS on GPIO%d/%d: no ADC, current reads 0", r_is, l_is);
    }
    apply(motor_cmd::coast, 0);
    return true;
}

void bts7960::deinit()
{
    if (ok_)
    {
        apply(motor_cmd::coast, 0);
        for (int i = 0; i < 3; ++i)
        {
            ledc_stop(k_mode, k_bts_ch[i], 0);
        }
    }
    if (cali_)
    {
        adc_cali_delete_scheme_curve_fitting(cali_);
        cali_ = nullptr;
    }
    if (adc_)
    {
        adc_oneshot_del_unit(adc_);
        adc_ = nullptr;
    }
    ok_ = false;
}

void bts7960::duty(int i, float d)
{
    d = std::fmin(std::fmax(d, 0.f), 1.f);
    ledc_set_duty(k_mode, k_bts_ch[i], static_cast<uint32_t>(d * ((1u << k_bts_bits) - 1) + 0.5f));
    ledc_update_duty(k_mode, k_bts_ch[i]);
}

void bts7960::apply(motor_cmd cmd, float d)
{
    if (!ok_)
    {
        return;
    }
    switch (cmd)
    {
        case motor_cmd::coast:
            duty(k_en, 0);
            duty(k_r, 0);
            duty(k_l, 0);
            break;
        case motor_cmd::drive:
            // Off-phase is both low sides on (drive-brake), so the duty is the
            // average voltage.
            duty(k_r, d > 0 ? d : 0);
            duty(k_l, d < 0 ? -d : 0);
            duty(k_en, 1);
            break;
        case motor_cmd::brake:
            // Both inputs low = both low sides on; EN PWM sets the strength.
            duty(k_r, 0);
            duty(k_l, 0);
            duty(k_en, d);
            break;
    }
}

float bts7960::read_current(float is_div, float r_is_ohm, float dt)
{
    if (!adc_)
    {
        return 0;
    }
    auto volts = [&](int ch)
    {
        int raw = 0;
        if (adc_oneshot_read(adc_, static_cast<adc_channel_t>(ch), &raw) != ESP_OK)
        {
            return 0.f;
        }
        int mv = 0;
        if (cali_ && adc_cali_raw_to_voltage(cali_, raw, &mv) == ESP_OK)
        {
            return mv / 1000.f;
        }
        return raw * 3.3f / 4095.f;
    };
    // Only the conducting half-bridge sources IS current.
    const float v_is = std::fmax(volts(ch_r_), volts(ch_l_)) * is_div;
    // Above ~4 V at the IS pin the BTS is at its current limit or in fault
    // (I_IS(lim) ~4.5 mA into 1 k).
    overload_t_ = v_is > 4.0f ? overload_t_ + dt : 0.f;
    constexpr float k_ilis = 8500.f;
    return v_is / r_is_ohm * k_ilis;
}

void bts7960::reattach()
{
    if (ok_)
    {
        ledc_channel(k_bts_ch[k_en], k_bts_timer, en_);
        duty(k_en, 0);
    }
}

bool IRAM_ATTR deadman::on_alarm(gptimer_handle_t, const gptimer_alarm_event_data_t*, void* arg)
{
    auto* self = static_cast<deadman*>(arg);
    const int en = self->en_gpio_.load();
    if (en >= 0)
    {
        // Take EN off LEDC and drive it low: the bridge coasts.
        esp_rom_gpio_connect_out_signal(en, SIG_GPIO_OUT_IDX, false, false);
        gpio_ll_set_level(&GPIO, en, 0);
    }
    self->tripped_.store(true);
    return false;
}

bool deadman::init(int en_gpio)
{
    en_gpio_.store(en_gpio);
    gptimer_config_t cfg = {};
    cfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    cfg.direction = GPTIMER_COUNT_UP;
    cfg.resolution_hz = 1000000;
    if (gptimer_new_timer(&cfg, &timer_) != ESP_OK)
    {
        return false;
    }
    gptimer_event_callbacks_t cbs = {};
    cbs.on_alarm = on_alarm;
    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count = 50000;  // 50 ms
    return gptimer_register_event_callbacks(timer_, &cbs, this) == ESP_OK &&
           gptimer_set_alarm_action(timer_, &alarm) == ESP_OK && gptimer_enable(timer_) == ESP_OK &&
           gptimer_start(timer_) == ESP_OK;
}

void deadman::kick()
{
    if (timer_)
    {
        gptimer_set_raw_count(timer_, 0);
    }
}

void deadman::clear()
{
    if (!timer_)
    {
        return;
    }
    gptimer_set_raw_count(timer_, 0);
    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count = 50000;
    gptimer_set_alarm_action(timer_, &alarm);
    tripped_.store(false);
}

}  // namespace rover_sample
