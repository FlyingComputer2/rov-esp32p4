#include "sd_card.hpp"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <thread>

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_heap_caps.h"
#include "esp_ldo_regulator.h"
#include "esp_pthread.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

namespace rover_sd
{
namespace
{

/* Q1 (P-channel high-side switch) gate: low powers the card. */
constexpr gpio_num_t k_power_gpio = GPIO_NUM_45;
constexpr int k_power_on_level = 0;
/* Same channel and voltage as the proxy's boot-strap LDO (shared). */
constexpr int k_io_ldo_chan = 4;
constexpr int k_io_ldo_mv = 3300;

/* Mount attempts; the card is power-cycled between them, and the retries run
 * at the default 20 MHz instead of 40 MHz high speed (after a power cycle the
 * card's init failed with ESP_ERR_INVALID_CRC at 40 MHz on every attempt). */
constexpr int k_mount_attempts = 3;

/* The card's lines (slot 0 IO MUX): D0-D3, CLK, CMD. */
constexpr gpio_num_t k_card_lines[] = {GPIO_NUM_39, GPIO_NUM_40, GPIO_NUM_41,
                                       GPIO_NUM_42, GPIO_NUM_43, GPIO_NUM_44};

/* The write test runs only on request (`set sys sd_test run`), in its own
 * directory: a boot writes nothing, and the root directory is only written when
 * a top-level entry is created. The root directory was the one structure found
 * corrupted after the board lost power during card writes, and the boot test
 * used to create and delete a file in it on every boot. */
constexpr const char* k_test_dir = "/sdcard/sys";
constexpr const char* k_test_path = "/sdcard/sys/sdtest.tmp";
constexpr size_t k_test_chunk = 64 * 1024;
constexpr size_t k_test_bytes = 1024 * 1024;

sdmmc_card_t* s_card = nullptr;
int s_attempts = 0;

/* format_card(): 0 idle, 1 running, 2 ok, 3 failed. */
std::atomic<int> s_fmt_state {0};
/* run_test(): true while it runs. */
std::atomic<bool> s_test_running {false};
bool s_shutdown_registered = false;
esp_err_t s_fmt_err = ESP_OK;
int64_t s_fmt_ms = 0;

sdmmc_host_t make_host(int freq_khz)
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = freq_khz;
    return host;
}

/* Slot 0 is IO MUX only: the pins are fixed (CLK 43, CMD 44, D0-D3 39-42). */
sdmmc_slot_config_t make_slot()
{
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 4;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    return slot;
}

esp_vfs_fat_sdmmc_mount_config_t make_mount_cfg(bool format)
{
    esp_vfs_fat_sdmmc_mount_config_t mcfg = {};
    /* Only format_card() formats: the card holds recordings and photos, and a
     * glitch that looks like a missing filesystem must not erase them. */
    mcfg.format_if_mount_failed = format;
    /* The MKV recorder, a photo and udpfs (up to 4 handles) at once. */
    mcfg.max_files = 8;
    mcfg.allocation_unit_size = 64 * 1024;
    return mcfg;
}
esp_ldo_channel_handle_t s_ldo = nullptr;

/* Written once by mount() (app_main), read by status_line() afterwards. */
const char* s_fail_stage = "none";
esp_err_t s_fail_err = ESP_ERR_INVALID_STATE;
struct self_test
{
    bool ran = false;
    const char* result = "none"; /* not run since boot (`set sys sd_test run`) */
    uint32_t write_kbps = 0;
    uint32_t read_kbps = 0;
} s_test;

esp_err_t power_on()
{
    if (!s_ldo)
    {
        esp_ldo_channel_config_t ldo = {};
        ldo.chan_id = k_io_ldo_chan;
        ldo.voltage_mv = k_io_ldo_mv;
        const esp_err_t err = esp_ldo_acquire_channel(&ldo, &s_ldo);
        if (err != ESP_OK)
        {
            return err;
        }
    }
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << k_power_gpio;
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    gpio_set_level(k_power_gpio, k_power_on_level);
    /* Card VDD ramp before the first command. */
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

/* The card's lines as GPIOs: driven low (no pull) or released (input, no
 * pull). While the card is unpowered its lines must not be high: the slot's
 * internal pull-ups and the host's CLK / CMD drivers are on the 3.3 V IO
 * supply (LDO_VO4, which stays on for the C6), and they back-power the card
 * through its pins, so it never resets. Slot 0 must not be initialised (the
 * next mount puts the pins back on the IO MUX). */
void card_lines(bool low)
{
    uint64_t mask = 0;
    for (gpio_num_t g : k_card_lines)
    {
        mask |= 1ULL << g;
    }
    gpio_config_t io = {};
    io.pin_bit_mask = mask;
    io.mode = low ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io);
    if (low)
    {
        for (gpio_num_t g : k_card_lines)
        {
            gpio_set_level(g, 0);
        }
    }
}

void power_off()
{
    card_lines(true);
    gpio_set_level(k_power_gpio, !k_power_on_level);
}

/* Card power off long enough to reset it (lines low, see card_lines), then on
 * again with the lines released. */
void power_cycle()
{
    power_off();
    vTaskDelay(pdMS_TO_TICKS(500));
    card_lines(false);
    gpio_set_level(k_power_gpio, k_power_on_level);
    vTaskDelay(pdMS_TO_TICKS(20));
}

/* Before a software restart (OTA, udp_ota reboot): unmount, so FAT is
 * consistent and the card idle, then power it down cleanly instead of with the
 * reset. Panics do not run shutdown handlers. An open file (a recording) loses
 * what was not yet closed; the volume stays consistent. */
void shutdown_card()
{
    if (s_card)
    {
        esp_vfs_fat_sdcard_unmount(k_mount_point, s_card);
        s_card = nullptr;
    }
    power_off();
    esp_rom_delay_us(10 * 1000);
}

uint32_t kbps(size_t bytes, int64_t us)
{
    return us > 0 ? (uint32_t)((uint64_t)bytes * 1000000 / 1024 / (uint64_t)us) : 0;
}

/* 1 MB in 64 KB writes plus fsync, read back and compared, file removed. */
void run_self_test()
{
    s_test.ran = true;
    uint8_t* buf = (uint8_t*)heap_caps_malloc(k_test_chunk, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t* rd = (uint8_t*)heap_caps_malloc(k_test_chunk, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf || !rd)
    {
        s_test.result = "nomem";
        heap_caps_free(buf);
        heap_caps_free(rd);
        return;
    }
    for (size_t i = 0; i < k_test_chunk; ++i)
    {
        buf[i] = (uint8_t)(i * 31 + 7);
    }

    s_test.result = "write_failed";
    mkdir(k_test_dir, 0777);
    FILE* f = fopen(k_test_path, "wb");
    if (f)
    {
        const int64_t t0 = esp_timer_get_time();
        size_t done = 0;
        for (; done < k_test_bytes; done += k_test_chunk)
        {
            buf[0] = (uint8_t)(done / k_test_chunk);
            if (fwrite(buf, 1, k_test_chunk, f) != k_test_chunk)
            {
                break;
            }
        }
        const bool flushed = fflush(f) == 0 && fsync(fileno(f)) == 0;
        fclose(f);
        if (done == k_test_bytes && flushed)
        {
            s_test.write_kbps = kbps(k_test_bytes, esp_timer_get_time() - t0);
            s_test.result = "read_failed";
            f = fopen(k_test_path, "rb");
            if (f)
            {
                const int64_t t1 = esp_timer_get_time();
                bool same = true;
                size_t got = 0;
                for (; got < k_test_bytes; got += k_test_chunk)
                {
                    if (fread(rd, 1, k_test_chunk, f) != k_test_chunk)
                    {
                        break;
                    }
                    buf[0] = (uint8_t)(got / k_test_chunk);
                    same = same && memcmp(buf, rd, k_test_chunk) == 0;
                }
                fclose(f);
                if (got == k_test_bytes)
                {
                    s_test.read_kbps = kbps(k_test_bytes, esp_timer_get_time() - t1);
                    s_test.result = same ? "ok" : "mismatch";
                }
            }
        }
    }
    unlink(k_test_path);
    heap_caps_free(buf);
    heap_caps_free(rd);
}

}  // namespace

esp_err_t mount()
{
    if (s_card)
    {
        return ESP_OK;
    }
    esp_err_t err = power_on();
    if (err != ESP_OK)
    {
        s_fail_stage = "power";
        s_fail_err = err;
        printf("ROVER_SD_FAILED stage=power err=%s\n", esp_err_to_name(err));
        return err;
    }

    sdmmc_host_t host = make_host(SDMMC_FREQ_HIGHSPEED);
    const sdmmc_slot_config_t slot = make_slot();
    const esp_vfs_fat_sdmmc_mount_config_t mcfg = make_mount_cfg(false);

    sdmmc_card_t* card = nullptr;
    for (s_attempts = 1; s_attempts <= k_mount_attempts; ++s_attempts)
    {
        host.max_freq_khz = s_attempts == 1 ? SDMMC_FREQ_HIGHSPEED : SDMMC_FREQ_DEFAULT;
        err = esp_vfs_fat_sdmmc_mount(k_mount_point, &host, &slot, &mcfg, &card);
        if (err == ESP_OK)
        {
            break;
        }
        printf("ROVER_SD_RETRY attempt=%d freq_khz=%d err=%s\n", s_attempts, host.max_freq_khz,
               esp_err_to_name(err));
        if (s_attempts < k_mount_attempts)
        {
            power_cycle();
        }
    }
    if (err != ESP_OK)
    {
        s_fail_stage = "mount";
        s_fail_err = err;
        printf("ROVER_SD_FAILED stage=mount err=%s\n", esp_err_to_name(err));
        fflush(stdout);
        return err;
    }
    s_card = card;
    if (!s_shutdown_registered)
    {
        s_shutdown_registered = esp_register_shutdown_handler(shutdown_card) == ESP_OK;
    }
    const std::string line = status_line();
    printf("ROVER_SD %s\n", line.c_str());
    fflush(stdout);
    return ESP_OK;
}

int format_card()
{
    int expected = s_fmt_state.load();
    do
    {
        if (expected == 1)
        {
            return -EALREADY;
        }
    } while (!s_fmt_state.compare_exchange_weak(expected, 1));

    esp_pthread_cfg_t tc = esp_pthread_get_default_config();
    esp_pthread_cfg_t prev {};
    const bool had_prev = esp_pthread_get_cfg(&prev) == ESP_OK;
    tc.thread_name = "sd_format";
    tc.stack_size = 8 * 1024;
    tc.prio = 4;
    tc.stack_alloc_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT; /* SDMMC: the cache stays on */
    esp_pthread_set_cfg(&tc);
    std::thread([] {
        printf("ROVER_SD_FORMAT start mounted=%d\n", s_card != nullptr);
        fflush(stdout);
        const int64_t t0 = esp_timer_get_time();
        esp_err_t err = ESP_OK;
        esp_vfs_fat_sdmmc_mount_config_t mcfg = make_mount_cfg(true);
        if (s_card)
        {
            err = esp_vfs_fat_sdcard_format_cfg(k_mount_point, s_card, &mcfg);
        }
        else
        {
            sdmmc_host_t host = make_host(SDMMC_FREQ_DEFAULT);
            const sdmmc_slot_config_t slot = make_slot();
            sdmmc_card_t* card = nullptr;
            err = power_on();
            if (err == ESP_OK)
            {
                err = esp_vfs_fat_sdmmc_mount(k_mount_point, &host, &slot, &mcfg, &card);
            }
            if (err == ESP_OK)
            {
                s_card = card;
            }
        }
        s_fmt_ms = (esp_timer_get_time() - t0) / 1000;
        s_fmt_err = err;
        s_fmt_state.store(err == ESP_OK ? 2 : 3);
        const std::string line = status_line();
        printf("ROVER_SD_FORMAT %s ms=%" PRId64 " err=%s; %s\n", err == ESP_OK ? "ok" : "failed", s_fmt_ms,
               esp_err_to_name(err), line.c_str());
        fflush(stdout);
    }).detach();
    const esp_pthread_cfg_t restore = had_prev ? prev : esp_pthread_get_default_config();
    esp_pthread_set_cfg(&restore);
    return 0;
}

int run_test()
{
    if (!s_card)
    {
        return -ENODEV;
    }
    if (s_fmt_state.load() == 1 || s_test_running.exchange(true))
    {
        return -EALREADY;
    }
    esp_pthread_cfg_t tc = esp_pthread_get_default_config();
    esp_pthread_cfg_t prev {};
    const bool had_prev = esp_pthread_get_cfg(&prev) == ESP_OK;
    tc.thread_name = "sd_test";
    tc.stack_size = 6 * 1024;
    tc.prio = 4;
    tc.stack_alloc_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT; /* SDMMC: the cache stays on */
    esp_pthread_set_cfg(&tc);
    std::thread([] {
        run_self_test();
        s_test_running.store(false);
        const std::string line = status_line();
        printf("ROVER_SD_TEST %s\n", line.c_str());
        fflush(stdout);
    }).detach();
    const esp_pthread_cfg_t restore = had_prev ? prev : esp_pthread_get_default_config();
    esp_pthread_set_cfg(&restore);
    return 0;
}

std::string format_line()
{
    char out[64];
    switch (s_fmt_state.load())
    {
    case 1:
        return "running";
    case 2:
        snprintf(out, sizeof(out), "ok ms=%" PRId64, s_fmt_ms);
        return out;
    case 3:
        snprintf(out, sizeof(out), "failed err=%s", esp_err_to_name(s_fmt_err));
        return out;
    default:
        return "idle";
    }
}

bool mounted()
{
    return s_card != nullptr;
}

esp_err_t usage(uint64_t* total_bytes, uint64_t* free_bytes)
{
    if (!s_card)
    {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_vfs_fat_info(k_mount_point, total_bytes, free_bytes);
}

std::string status_line()
{
    if (s_fmt_state.load() == 1)
    {
        return "formatting";
    }
    if (s_test_running.load())
    {
        return "testing";
    }
    char out[256];
    if (!s_card)
    {
        snprintf(out, sizeof(out), "mounted=0 stage=%s err=%s attempts=%d", s_fail_stage,
                 esp_err_to_name(s_fail_err), s_attempts > k_mount_attempts ? k_mount_attempts : s_attempts);
        return out;
    }
    uint64_t total = 0;
    uint64_t free_bytes = 0;
    usage(&total, &free_bytes);
    const uint64_t card_bytes = (uint64_t)s_card->csd.capacity * s_card->csd.sector_size;
    snprintf(out, sizeof(out),
             "mounted=1 path=%s name=%s card_mb=%" PRIu64 " fs_mb=%" PRIu64 " free_mb=%" PRIu64
             " freq_khz=%d width=%d test=%s write_kbps=%" PRIu32 " read_kbps=%" PRIu32 " attempts=%d",
             k_mount_point, s_card->cid.name, card_bytes >> 20, total >> 20, free_bytes >> 20,
             s_card->real_freq_khz, 1 << s_card->log_bus_width, s_test.result, s_test.write_kbps,
             s_test.read_kbps, s_attempts);
    return out;
}

}  // namespace rover_sd
