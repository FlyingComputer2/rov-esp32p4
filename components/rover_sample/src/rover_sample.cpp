#include "rover_sample/rover_sample.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <sstream>
#include <thread>

#include "ahrs.hpp"
#include "controller.hpp"
#include "drivers.hpp"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_pthread.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "msp.hpp"
#include "nvs.h"
#include "params.hpp"
#include "protocol.hpp"
#include "rc_link.hpp"

namespace rover_sample
{
namespace
{

const char* TAG = "rover_sample";

constexpr int k_loop_ms = 4;  // 250 Hz
constexpr int k_mag_every = 5;  // 50 Hz
constexpr int k_probe_every = 500;  // retry a missing sensor every 2 s
constexpr int k_imu_fail_limit = 10;
constexpr const char* k_nvs_ns = "rover_sample";
constexpr const char* k_nvs_key = "params";

uint32_t realtime_us32()
{
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<uint32_t>(static_cast<uint64_t>(ts.tv_sec) * 1000000ull + ts.tv_nsec / 1000);
}

bool parse_host_port(const std::string& s, sockaddr_in* out)
{
    const auto colon = s.rfind(':');
    if (colon == std::string::npos)
    {
        return false;
    }
    *out = {};
    out->sin_family = AF_INET;
    out->sin_port = htons(static_cast<uint16_t>(std::atoi(s.c_str() + colon + 1)));
    return inet_pton(AF_INET, s.substr(0, colon).c_str(), &out->sin_addr) == 1 && out->sin_port != 0;
}

// What the control task publishes each cycle.
/* NVS writes flash, which needs a stack in internal RAM (the cache is off
 * meanwhile); rover_sample's threads are on PSRAM stacks, so NVS access runs
 * on a short-lived task with an internal stack. False if that task cannot be
 * created. */
bool on_internal_stack(const std::function<void()>& f)
{
    struct job
    {
        const std::function<void()>* f;
        SemaphoreHandle_t done;
    } j {&f, xSemaphoreCreateBinary()};
    if (!j.done)
    {
        return false;
    }
    /* The flash code accepts a stack in L2 RAM or RTC (LP) RAM, not PSRAM or
     * the SPM (0x30100000, which plain MALLOC_CAP_INTERNAL may return). With
     * live TX running, L2 RAM's DMA-capable part is down to a few hundred
     * bytes per block, so RTC RAM first. */
    static const UBaseType_t k_caps[] = {MALLOC_CAP_RTCRAM | MALLOC_CAP_8BIT,
                                         MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT};
    BaseType_t ok = pdFAIL;
    for (UBaseType_t caps : k_caps)
    {
        ok = xTaskCreatePinnedToCoreWithCaps(
            [](void* arg)
            {
                auto* jb = static_cast<job*>(arg);
                (*jb->f)();
                xSemaphoreGive(jb->done);
                vTaskDeleteWithCaps(nullptr);
            },
            "rs_nvs", 3 * 1024, &j, 5, nullptr, tskNO_AFFINITY, caps);
        if (ok == pdPASS)
        {
            break;
        }
    }
    if (ok != pdPASS)
    {
        ESP_LOGE(TAG, "NVS task not created (largest RTC block %u, internal DMA block %u)",
                 static_cast<unsigned>(heap_caps_get_largest_free_block(k_caps[0])),
                 static_cast<unsigned>(heap_caps_get_largest_free_block(k_caps[1])));
        vSemaphoreDelete(j.done);
        return false;
    }
    xSemaphoreTake(j.done, portMAX_DELAY);
    vSemaphoreDelete(j.done);
    return true;
}

struct status
{
    control_output out;
    float a[3] {};
    float roll = 0, pitch = 0, heading = 0;
    float current = 0;
    bool imu_present = false, imu_ok = false, mag_present = false, mag_ok = false;
    bool gyro_cal = false, bts_on = false, deadman = false, bts_fault = false;
    const char* mag_kind = "none";
    float servo_us = 0;
    uint32_t i2c_errors = 0;
    uint32_t overruns = 0;
    float loop_max_ms = 0;
    float mag_min[3] {}, mag_max[3] {};
    bool mag_cal_running = false;
};

class app
{
   public:
    explicit app(const config& cfg) : cfg_(cfg) {}
    int run();

   private:
    static void control_trampoline(void* arg)
    {
        static_cast<app*>(arg)->control_task();
    }
    void control_task();
    void link_loop();
    void console_loop(int fd);
    void handle_frame(const msp::frame& f);
    void send(char dir, uint16_t func, const std::vector<uint8_t>& payload);
    void send_telem();
    std::string command(const std::string& line);
    std::string set_param(const param_info& pi, float value, float* applied);
    std::string status_text();
    bool save_params();
    void load_params();

    config cfg_;
    i2c_master_bus_handle_t bus_ = nullptr;
    mpu6050 imu_;
    magnetometer mag_;
    servo servo_;
    bts7960 bts_;
    deadman deadman_;

    // Guarded by mu_.
    std::mutex mu_;
    params p_;
    rc_link link_;
    status st_;
    bool req_gyro_cal_ = true;
    int req_mag_cal_ = 0;  // 1 start, 2 stop
    float servo_override_us_ = 0;
    rc_frame console_rc_;
    int64_t console_rc_until_us_ = 0;
    uint16_t console_seq_ = 0;

    // Link thread only.
    uint16_t telem_seq_ = 0;
};

int app::run()
{
    if (i2c_master_get_bus_handle(static_cast<i2c_port_num_t>(cfg_.pin.i2c_port), &bus_) != ESP_OK)
    {
        ESP_LOGE(TAG, "no I2C%d bus: create it before rover_sample::main", cfg_.pin.i2c_port);
        return 1;
    }
    sockaddr_in addr {};
    if (!parse_host_port(cfg_.console, &addr))
    {
        ESP_LOGE(TAG, "bad console address %s", cfg_.console.c_str());
        return 1;
    }
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0 || bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0)
    {
        ESP_LOGE(TAG, "cannot bind console %s", cfg_.console.c_str());
        return 1;
    }
    timeval timeout {};
    timeout.tv_usec = 100 * 1000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    load_params();

    /* Internal RAM is short with 1080p live TX running: with rover_sample's
     * stacks internal (20 KB left) live TX fell to ~20 fps, its sender and FEC
     * ~5x slower; on PSRAM stacks it runs at 29 fps as without rover_sample.
     * So every rover_sample stack is in PSRAM (the caller's too, see main.cpp)
     * and only NVS access borrows an internal one (on_internal_stack). */
    esp_pthread_cfg_t prev {};
    const bool had_prev = esp_pthread_get_cfg(&prev) == ESP_OK;
    esp_pthread_cfg_t ccfg = esp_pthread_get_default_config();
    ccfg.stack_size = 6 * 1024;
    ccfg.prio = 5;
    ccfg.thread_name = "rs_console";
    ccfg.stack_alloc_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    esp_pthread_set_cfg(&ccfg);
    std::thread(&app::console_loop, this, fd).detach();
    if (had_prev)
    {
        esp_pthread_set_cfg(&prev);
    }

    if (xTaskCreatePinnedToCoreWithCaps(control_trampoline, "rs_control", 6 * 1024, this, cfg_.control_prio,
                                        nullptr, cfg_.control_core, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) !=
        pdPASS)
    {
        ESP_LOGE(TAG, "control task not created");
        return 1;
    }
    printf("rover_sample: console %s, RC/telemetry MSP v2 on the link\n", cfg_.console.c_str());
    link_loop();
    return 0;
}

void app::control_task()
{
    params p;
    {
        std::lock_guard<std::mutex> lock(mu_);
        p = p_;
    }
    imu_.probe_and_init(bus_, static_cast<int>(p.get(p_gyro_lpf_hz)));
    mag_.probe_and_init(bus_);
    printf("rover_sample: imu %s, mag %s\n", imu_.present() ? "mpu6050" : "none", mag_.kind());
    servo_.init(cfg_.pin.servo, static_cast<int>(p.get(p_servo_hz)));
    servo_.set_us(p.get(p_servo_center_us));
    int applied_servo_hz = static_cast<int>(p.get(p_servo_hz));
    int applied_lpf = static_cast<int>(p.get(p_gyro_lpf_hz));
    bool bts_on = false;
    int applied_bts_hz = 0;
    deadman_.init(-1);

    controller ctl;
    gyro_cal cal;
    ahrs att;
    int imu_fail = k_imu_fail_limit;
    uint32_t i2c_errors = 0, overruns = 0;
    float loop_max_ms = 0;
    float mag_min[3] {1e9f, 1e9f, 1e9f}, mag_max[3] {-1e9f, -1e9f, -1e9f};
    bool mag_cal = false, mag_ok = false;
    int64_t prev = esp_timer_get_time();
    uint32_t n = 0;
    TickType_t wake = xTaskGetTickCount();

    for (;;)
    {
        if (xTaskDelayUntil(&wake, pdMS_TO_TICKS(k_loop_ms)) == pdFALSE)
        {
            // Late: restart the schedule instead of catching up back to back.
            ++overruns;
            wake = xTaskGetTickCount();
        }
        const int64_t now = esp_timer_get_time();
        const float dt = std::fmin(std::fmax((now - prev) / 1e6f, 0.001f), 0.02f);
        prev = now;
        ++n;

        // Snapshot of everything other threads write.
        control_input in;
        float override_us;
        {
            std::lock_guard<std::mutex> lock(mu_);
            p = p_;
            if (req_gyro_cal_)
            {
                req_gyro_cal_ = false;
                cal.start();
            }
            if (req_mag_cal_ == 1)
            {
                mag_cal = true;
                for (int i = 0; i < 3; ++i)
                {
                    mag_min[i] = 1e9f;
                    mag_max[i] = -1e9f;
                }
            }
            else if (req_mag_cal_ == 2)
            {
                mag_cal = false;
            }
            req_mag_cal_ = 0;
            const uint32_t fs_ms = static_cast<uint32_t>(p.get(p_failsafe_ms));
            in.rc_ever = link_.ever();
            in.rc_valid = link_.ever() && now - link_.last_us() < int64_t(fs_ms) * 1000;
            const rc_frame& rc = link_.last();
            in.steer = rc.steer / 1000.f;
            in.thr = rc.thr / 1000.f;
            in.arm = rc.flags & k_rc_flag_arm;
            in.mode_req = rc.mode;
            override_us = servo_override_us_;
        }
        in.dt = dt;

        // Peripheral changes (the console allows these only while disarmed).
        if (static_cast<int>(p.get(p_servo_hz)) != applied_servo_hz)
        {
            applied_servo_hz = static_cast<int>(p.get(p_servo_hz));
            servo_.set_hz(applied_servo_hz);
        }
        if (static_cast<int>(p.get(p_gyro_lpf_hz)) != applied_lpf)
        {
            applied_lpf = static_cast<int>(p.get(p_gyro_lpf_hz));
            imu_.set_lpf(applied_lpf);
        }
        const bool want_bts = p.get(p_bts_enabled) >= 1.f;
        const int want_bts_hz = static_cast<int>(p.get(p_bts_pwm_hz));
        if (want_bts != bts_on || (bts_on && want_bts_hz != applied_bts_hz))
        {
            bts_.deinit();
            bts_on = want_bts && bts_.init(cfg_.pin.bts_rpwm, cfg_.pin.bts_lpwm, cfg_.pin.bts_en,
                                           cfg_.pin.bts_r_is, cfg_.pin.bts_l_is, want_bts_hz);
            applied_bts_hz = want_bts_hz;
            deadman_.set_en_gpio(bts_on ? cfg_.pin.bts_en : -1);
            printf("rover_sample: BTS7960 %s\n", bts_on ? "on" : "off");
        }

        // IMU.
        float g[3] {}, a[3] {};
        if (!imu_.present())
        {
            // Init holds the loop ~100 ms (MPU reset), past the deadman, so
            // only while disarmed; the trip it causes is expected.
            if (n % k_probe_every == 0 && ctl.current_state() == state::disarmed)
            {
                if (imu_.probe_and_init(bus_, applied_lpf))
                {
                    cal.start();
                }
                deadman_.clear();
                bts_.reattach();
            }
        }
        else if (imu_.read(g, a))
        {
            imu_fail = 0;
        }
        else
        {
            ++i2c_errors;
            ++imu_fail;
        }
        const bool imu_ok = imu_.present() && imu_fail < k_imu_fail_limit;
        if (imu_ok && imu_fail == 0)
        {
            if (!cal.done())
            {
                cal.add(g);
            }
            else if (ctl.current_state() == state::disarmed)
            {
                cal.track(g);
            }
        }
        float gc[3];
        for (int i = 0; i < 3; ++i)
        {
            gc[i] = g[i] - cal.bias()[i];
        }
        const float yaw = imu_ok && cal.done() ? gc[2] * p.get(p_imu_yaw_sign) : 0.f;
        if (imu_ok && imu_fail == 0)
        {
            att.update(gc, a, yaw, dt);
        }

        // Magnetometer, telemetry only.
        if (n % k_mag_every == 0)
        {
            if (!mag_.present())
            {
                if (n % k_probe_every == 0)
                {
                    mag_.probe_and_init(bus_);
                }
            }
            else
            {
                float m[3];
                if (mag_.read(m))
                {
                    mag_ok = true;
                    if (mag_cal)
                    {
                        for (int i = 0; i < 3; ++i)
                        {
                            mag_min[i] = std::fmin(mag_min[i], m[i]);
                            mag_max[i] = std::fmax(mag_max[i], m[i]);
                        }
                    }
                    const float mc[3] = {m[0] - p.get(p_mag_off_x), m[1] - p.get(p_mag_off_y),
                                         m[2] - p.get(p_mag_off_z)};
                    att.mag(mc, dt * k_mag_every);
                }
            }
        }

        // Motor current and bridge faults.
        float current = 0;
        if (bts_on)
        {
            current = bts_.read_current(p.get(p_is_div), p.get(p_is_r_ohm), dt);
        }

        in.imu_ok = imu_ok;
        in.gyro_cal = cal.done();
        in.yaw_rate_dps = yaw;
        in.current_a = current;
        in.bts_fault = bts_on && bts_.fault();
        in.deadman = deadman_.tripped();
        // The operator acknowledges a deadman trip by disarming.
        if (in.deadman && !in.arm)
        {
            deadman_.clear();
            bts_.reattach();
        }
        control_output out = ctl.step(in, p);

        // Servo: centre (from the trim) unless driven; disarmed may override.
        float s = out.servo;
        if (p.get(p_servo_reverse) >= 1.f)
        {
            s = -s;
        }
        const float c = p.get(p_servo_center_us);
        float us = s >= 0 ? c + s * (p.get(p_servo_max_us) - c) : c + s * (c - p.get(p_servo_min_us));
        if (out.st == state::disarmed && override_us > 0)
        {
            us = override_us;
        }
        servo_.set_us(us);
        if (bts_on)
        {
            bts_.apply(out.motor, out.duty);
        }
        deadman_.kick();

        const float loop_ms = (esp_timer_get_time() - now) / 1000.f;
        loop_max_ms = std::fmax(loop_max_ms, loop_ms);

        std::lock_guard<std::mutex> lock(mu_);
        if (out.st != state::disarmed)
        {
            servo_override_us_ = 0;
        }
        st_.out = out;
        for (int i = 0; i < 3; ++i)
        {
            st_.a[i] = a[i];
            st_.mag_min[i] = mag_min[i];
            st_.mag_max[i] = mag_max[i];
        }
        st_.roll = att.roll_deg();
        st_.pitch = att.pitch_deg();
        st_.heading = att.heading_deg();
        st_.current = current;
        st_.imu_present = imu_.present();
        st_.imu_ok = imu_ok;
        st_.mag_present = mag_.present();
        st_.mag_ok = mag_ok;
        st_.mag_kind = mag_.kind();
        st_.gyro_cal = cal.done();
        st_.bts_on = bts_on;
        st_.deadman = in.deadman;
        st_.bts_fault = in.bts_fault;
        st_.servo_us = us;
        st_.i2c_errors = i2c_errors;
        st_.overruns = overruns;
        st_.loop_max_ms = loop_max_ms;
        st_.mag_cal_running = mag_cal;
    }
}

void app::send(char dir, uint16_t func, const std::vector<uint8_t>& payload)
{
    const auto wire = msp::encode(dir, func, payload);
    if (cfg_.link.send)
    {
        cfg_.link.send(wire.data(), wire.size());
    }
}

void app::handle_frame(const msp::frame& f)
{
    if (f.dir != msp::k_to_rover)
    {
        return;
    }
    switch (f.func)
    {
        case k_func_rc:
        {
            auto rc = unpack_rc(f.payload);
            if (rc)
            {
                std::lock_guard<std::mutex> lock(mu_);
                link_.accept(*rc, esp_timer_get_time(), realtime_us32(),
                             static_cast<uint32_t>(p_.get(p_failsafe_ms)));
            }
            return;
        }
        case k_func_get_param:
        {
            const param_info* pi = f.payload.size() == 1 ? find_param(static_cast<param_id>(f.payload[0])) : nullptr;
            if (!pi)
            {
                send(msp::k_error, f.func, f.payload);
                return;
            }
            float v;
            {
                std::lock_guard<std::mutex> lock(mu_);
                v = p_.get(pi->id);
            }
            send(msp::k_from_rover, f.func, pack_param(pi->id, v));
            return;
        }
        case k_func_set_param:
        {
            uint8_t id;
            float v;
            const param_info* pi = unpack_param(f.payload, &id, &v) ? find_param(static_cast<param_id>(id)) : nullptr;
            float applied;
            if (!pi || !set_param(*pi, v, &applied).empty())
            {
                send(msp::k_error, f.func, f.payload);
                return;
            }
            send(msp::k_from_rover, f.func, pack_param(pi->id, applied));
            return;
        }
        case k_func_save_params:
            send(msp::k_from_rover, f.func, {static_cast<uint8_t>(save_params() ? 1 : 0)});
            return;
        case k_func_cmd:
        {
            if (f.payload.size() != 1)
            {
                send(msp::k_error, f.func, f.payload);
                return;
            }
            const std::string reply = command(f.payload[0] == k_cmd_gyro_cal        ? "gyro_cal"
                                              : f.payload[0] == k_cmd_mag_cal_start ? "mag_cal start"
                                              : f.payload[0] == k_cmd_mag_cal_stop  ? "mag_cal stop"
                                                                                    : "?");
            const int8_t rc = reply.rfind("error", 0) == 0 ? -1 : 0;
            send(msp::k_from_rover, f.func, {f.payload[0], static_cast<uint8_t>(rc)});
            return;
        }
        default:
            send(msp::k_error, f.func, {});
    }
}

void app::send_telem()
{
    telem_frame t;
    const int64_t now = esp_timer_get_time();
    {
        std::lock_guard<std::mutex> lock(mu_);
        const status& s = st_;
        t.rc_seq = link_.last().seq;
        t.rc_age_ms = link_.ever() ? usat16((now - link_.last_us()) / 1000.f) : 65535;
        t.state = static_cast<uint8_t>(s.out.st);
        t.mode = static_cast<uint8_t>(s.out.mode);
        t.r_cmd = sat16(s.out.r_cmd * 10);
        t.r_meas = sat16(s.out.r_meas * 10);
        t.steer_out = sat16(s.out.servo * 1000);
        t.thr_out = s.out.motor == motor_cmd::drive ? sat16(s.out.duty * 1000) : 0;
        t.heading = usat16(s.heading * 100);
        t.roll = sat16(s.roll * 100);
        t.pitch = sat16(s.pitch * 100);
        t.ax = sat16(s.a[0] * 1000);
        t.ay = sat16(s.a[1] * 1000);
        t.v_est = usat16(std::fabs(s.out.v_est) * 100);
        t.i_motor = usat16(s.current * 100);
        t.rc_loss = link_.loss_permille();
        t.rc_lat_ms = link_.latency_p50_ms();
        t.flags = (s.imu_ok ? k_telem_imu_ok : 0) | (s.mag_ok ? k_telem_mag_ok : 0) |
                  (s.gyro_cal ? k_telem_gyro_cal : 0) | (s.bts_on ? k_telem_bts_enabled : 0) |
                  (s.deadman ? k_telem_deadman : 0) | (s.out.imu_degraded ? k_telem_imu_degraded : 0) |
                  (s.out.motor == motor_cmd::brake ? k_telem_braking : 0);
    }
    t.seq = telem_seq_++;
    t.t_us = realtime_us32();
    send(msp::k_from_rover, k_func_telem, pack(t));
}

void app::link_loop()
{
    std::vector<uint8_t> buf;
    int64_t next_telem = 0, next_print = 0;
    for (;;)
    {
        while (cfg_.link.receive && cfg_.link.receive(&buf))
        {
            msp::frame f;
            if (msp::decode(buf.data(), buf.size(), &f))
            {
                handle_frame(f);
            }
        }
        const int64_t now = esp_timer_get_time();
        int telem_hz;
        {
            std::lock_guard<std::mutex> lock(mu_);
            // Console RC held for a while, re-sent at 50 Hz like a transmitter.
            if (now < console_rc_until_us_ && now - link_.last_us() >= 20000)
            {
                console_rc_.seq = ++console_seq_;
                console_rc_.t_us = realtime_us32();
                link_.accept(console_rc_, now, console_rc_.t_us,
                             static_cast<uint32_t>(p_.get(p_failsafe_ms)));
            }
            link_.tick(now);
            telem_hz = static_cast<int>(p_.get(p_telem_hz));
        }
        if (now >= next_telem)
        {
            next_telem = now + 1000000 / telem_hz;
            send_telem();
        }
        if (now >= next_print)
        {
            next_print = now + 10000000;
            printf("ROVER_SAMPLE %s\n", status_text().c_str());
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void app::console_loop(int fd)
{
    char buf[256];
    for (;;)
    {
        sockaddr_in from {};
        socklen_t from_len = sizeof(from);
        const ssize_t n = recvfrom(fd, buf, sizeof(buf) - 1, 0, reinterpret_cast<sockaddr*>(&from), &from_len);
        if (n <= 0)
        {
            continue;
        }
        buf[n] = '\0';
        const std::string reply = command(buf);
        sendto(fd, reply.data(), reply.size(), 0, reinterpret_cast<const sockaddr*>(&from), from_len);
    }
}

std::string app::set_param(const param_info& pi, float value, float* applied)
{
    std::lock_guard<std::mutex> lock(mu_);
    if ((pi.flags & k_param_disarmed_only) && st_.out.st != state::disarmed)
    {
        return "error armed\n";
    }
    *applied = p_.set(pi.id, value);
    return "";
}

std::string app::status_text()
{
    std::lock_guard<std::mutex> lock(mu_);
    const status& s = st_;
    static const char* states[] = {"disarmed", "armed", "failsafe", "fault"};
    static const char* modes[] = {"direct", "rate", "rate_hold"};
    const int64_t now = esp_timer_get_time();
    char b[640];
    snprintf(b, sizeof(b),
             "uptime_s=%lld state=%s mode=%s%s rc_src=%u rc_age_ms=%d rc_loss=%u rc_lat_ms=%u rc_frames=%" PRIu32
             " rc_rej=%" PRIu32 " imu=%s gyro_cal=%d mag=%s%s r_cmd=%.1f r_meas=%.1f steer=%.3f"
             " servo_us=%.0f motor=%s duty=%.3f v_est=%.2f i_a=%.2f bts=%d deadman=%d bts_fault=%d"
             " heading=%.1f roll=%.1f pitch=%.1f i2c_err=%" PRIu32 " overruns=%" PRIu32 " loop_max_ms=%.2f",
             static_cast<long long>(now / 1000000), states[static_cast<int>(s.out.st)], modes[static_cast<int>(s.out.mode)],
             s.out.imu_degraded ? "(imu)" : "", link_.source(),
             link_.ever() ? static_cast<int>((now - link_.last_us()) / 1000) : -1, link_.loss_permille(),
             link_.latency_p50_ms(), link_.frames(), link_.rejected(),
             s.imu_present ? (s.imu_ok ? "ok" : "err") : "none", s.gyro_cal, s.mag_kind,
             s.mag_present && !s.mag_ok ? "(no data)" : "", s.out.r_cmd, s.out.r_meas, s.out.servo,
             s.servo_us, s.out.motor == motor_cmd::drive ? "drive" : (s.out.motor == motor_cmd::brake ? "brake" : "coast"),
             s.out.duty, s.out.v_est, s.current, s.bts_on, s.deadman, s.bts_fault, s.heading, s.roll, s.pitch,
             s.i2c_errors, s.overruns, s.loop_max_ms);
    return b;
}

std::string app::command(const std::string& line)
{
    std::istringstream is(line);
    std::string cmd;
    is >> cmd;
    char b[96];

    if (cmd == "status")
    {
        return status_text() + "\n";
    }
    if (cmd == "list")
    {
        std::string out;
        size_t n;
        const param_info* t = param_table(&n);
        std::lock_guard<std::mutex> lock(mu_);
        for (size_t i = 0; i < n; ++i)
        {
            snprintf(b, sizeof(b), "%s=%g  # %s [%g..%g]%s\n", t[i].name, p_.get(t[i].id), t[i].unit, t[i].min,
                     t[i].max, (t[i].flags & k_param_disarmed_only) ? " disarmed only" : "");
            out += b;
        }
        return out;
    }
    if (cmd == "get" || cmd == "set")
    {
        std::string name;
        is >> name;
        const param_info* pi = find_param(name);
        if (!pi)
        {
            return "error unknown param\n";
        }
        float v;
        if (cmd == "set")
        {
            if (!(is >> v))
            {
                return "error value\n";
            }
            const std::string err = set_param(*pi, v, &v);
            if (!err.empty())
            {
                return err;
            }
        }
        else
        {
            std::lock_guard<std::mutex> lock(mu_);
            v = p_.get(pi->id);
        }
        snprintf(b, sizeof(b), "%s=%g\n", pi->name, v);
        return b;
    }
    if (cmd == "save")
    {
        return save_params() ? "ok\n" : "error nvs\n";
    }
    if (cmd == "load")
    {
        load_params();
        return "ok\n";
    }
    if (cmd == "defaults")
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (st_.out.st != state::disarmed)
        {
            return "error armed\n";
        }
        p_.reset();
        return "ok (not saved)\n";
    }
    if (cmd == "gyro_cal")
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (st_.out.st != state::disarmed)
        {
            return "error armed\n";
        }
        req_gyro_cal_ = true;
        return "ok, keep still 2 s\n";
    }
    if (cmd == "mag_cal")
    {
        std::string arg;
        is >> arg;
        std::lock_guard<std::mutex> lock(mu_);
        if (arg == "start")
        {
            req_mag_cal_ = 1;
            return "ok, rotate the car through every heading, then mag_cal stop\n";
        }
        if (arg == "stop")
        {
            req_mag_cal_ = 2;
            const status& s = st_;
            if (!s.mag_cal_running || s.mag_max[0] < s.mag_min[0])
            {
                return "error not running or no samples\n";
            }
            p_.set(p_mag_off_x, (s.mag_max[0] + s.mag_min[0]) / 2);
            p_.set(p_mag_off_y, (s.mag_max[1] + s.mag_min[1]) / 2);
            p_.set(p_mag_off_z, (s.mag_max[2] + s.mag_min[2]) / 2);
            snprintf(b, sizeof(b), "mag_off_x=%g mag_off_y=%g mag_off_z=%g (save to keep)\n", p_.get(p_mag_off_x),
                     p_.get(p_mag_off_y), p_.get(p_mag_off_z));
            return b;
        }
        return "error mag_cal start|stop\n";
    }
    if (cmd == "rc")
    {
        float steer, thr;
        int arm, hold_ms = 0;
        if (!(is >> steer >> thr >> arm))
        {
            return "error rc <steer -1..1> <thr -1..1> <arm 0|1> [hold_ms]\n";
        }
        is >> hold_ms;
        hold_ms = std::min(std::max(hold_ms, 0), 10000);
        rc_frame rc;
        rc.src = k_src_console;
        rc.steer = static_cast<int16_t>(std::lround(std::fmin(std::fmax(steer, -1.f), 1.f) * 1000));
        rc.thr = static_cast<int16_t>(std::lround(std::fmin(std::fmax(thr, -1.f), 1.f) * 1000));
        rc.flags = arm ? k_rc_flag_arm : 0;
        const int64_t now = esp_timer_get_time();
        std::lock_guard<std::mutex> lock(mu_);
        rc.seq = ++console_seq_;
        rc.t_us = realtime_us32();
        if (!link_.accept(rc, now, rc.t_us, static_cast<uint32_t>(p_.get(p_failsafe_ms))))
        {
            return "error another RC source is active\n";
        }
        console_rc_ = rc;
        console_rc_until_us_ = now + int64_t(hold_ms) * 1000;
        return "ok\n";
    }
    if (cmd == "servo_us")
    {
        float us = 0;
        is >> us;
        std::lock_guard<std::mutex> lock(mu_);
        if (st_.out.st != state::disarmed)
        {
            return "error armed\n";
        }
        servo_override_us_ = us > 0 ? std::fmin(std::fmax(us, 500.f), 2500.f) : 0.f;
        snprintf(b, sizeof(b), servo_override_us_ > 0 ? "servo_us=%g\n" : "servo_us off\n", servo_override_us_);
        return b;
    }
    return "error commands: status list get set save load defaults gyro_cal mag_cal rc servo_us\n";
}

/* NVS blob: (id:u8, value:f32 LE) pairs, so ids added later load with their
 * defaults and removed ids are skipped. */
bool app::save_params()
{
    std::vector<uint8_t> blob;
    size_t n;
    const param_info* t = param_table(&n);
    {
        std::lock_guard<std::mutex> lock(mu_);
        for (size_t i = 0; i < n; ++i)
        {
            const auto e = pack_param(t[i].id, p_.get(t[i].id));
            blob.insert(blob.end(), e.begin(), e.end());
        }
    }
    bool ok = false;
    on_internal_stack(
        [&]
        {
            nvs_handle_t h;
            if (nvs_open(k_nvs_ns, NVS_READWRITE, &h) != ESP_OK)
            {
                return;
            }
            ok = nvs_set_blob(h, k_nvs_key, blob.data(), blob.size()) == ESP_OK && nvs_commit(h) == ESP_OK;
            nvs_close(h);
        });
    return ok;
}

void app::load_params()
{
    std::vector<uint8_t> blob;
    on_internal_stack(
        [&]
        {
            nvs_handle_t h;
            if (nvs_open(k_nvs_ns, NVS_READONLY, &h) != ESP_OK)
            {
                return;
            }
            size_t len = 0;
            if (nvs_get_blob(h, k_nvs_key, nullptr, &len) == ESP_OK && len % 5 == 0)
            {
                blob.resize(len);
                if (nvs_get_blob(h, k_nvs_key, blob.data(), &len) != ESP_OK)
                {
                    blob.clear();
                }
            }
            nvs_close(h);
        });
    std::lock_guard<std::mutex> lock(mu_);
    int loaded = 0;
    for (size_t i = 0; i + 5 <= blob.size(); i += 5)
    {
        uint8_t id;
        float v;
        if (unpack_param({blob.begin() + i, blob.begin() + i + 5}, &id, &v) &&
            find_param(static_cast<param_id>(id)))
        {
            p_.set(static_cast<param_id>(id), v);
            ++loaded;
        }
    }
    printf("rover_sample: %d params from NVS\n", loaded);
}

}  // namespace

int main(const config& cfg)
{
    // Lives for the program; the control task holds a pointer.
    auto* a = new app(cfg);
    const int rc = a->run();
    if (rc != 0)
    {
        delete a;
    }
    return rc;
}

}  // namespace rover_sample
