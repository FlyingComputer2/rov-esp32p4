#include "sd_card.hpp"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_heap_caps.h"
#include "esp_ldo_regulator.h"
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

constexpr const char* k_test_path = "/sdcard/sdtest.tmp";
constexpr size_t k_test_chunk = 64 * 1024;
constexpr size_t k_test_bytes = 1024 * 1024;

sdmmc_card_t* s_card = nullptr;
esp_ldo_channel_handle_t s_ldo = nullptr;

/* Written once by mount() (app_main), read by status_line() afterwards. */
const char* s_fail_stage = "none";
esp_err_t s_fail_err = ESP_ERR_INVALID_STATE;
struct self_test
{
    bool ran = false;
    const char* result = "skipped";
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

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    /* Slot 0 is IO MUX only: the pins are fixed (CLK 43, CMD 44, D0-D3 39-42). */
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 4;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_sdmmc_mount_config_t mcfg = {};
    mcfg.format_if_mount_failed = true;
    mcfg.max_files = 4;
    mcfg.allocation_unit_size = 64 * 1024;

    sdmmc_card_t* card = nullptr;
    err = esp_vfs_fat_sdmmc_mount(k_mount_point, &host, &slot, &mcfg, &card);
    if (err != ESP_OK)
    {
        s_fail_stage = "mount";
        s_fail_err = err;
        printf("ROVER_SD_FAILED stage=mount err=%s\n", esp_err_to_name(err));
        fflush(stdout);
        return err;
    }
    s_card = card;
    run_self_test();
    const std::string line = status_line();
    printf("ROVER_SD %s\n", line.c_str());
    fflush(stdout);
    return ESP_OK;
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
    char out[256];
    if (!s_card)
    {
        snprintf(out, sizeof(out), "mounted=0 stage=%s err=%s", s_fail_stage,
                 esp_err_to_name(s_fail_err));
        return out;
    }
    uint64_t total = 0;
    uint64_t free_bytes = 0;
    usage(&total, &free_bytes);
    const uint64_t card_bytes = (uint64_t)s_card->csd.capacity * s_card->csd.sector_size;
    snprintf(out, sizeof(out),
             "mounted=1 path=%s name=%s card_mb=%" PRIu64 " fs_mb=%" PRIu64 " free_mb=%" PRIu64
             " freq_khz=%d width=%d test=%s write_kbps=%" PRIu32 " read_kbps=%" PRIu32,
             k_mount_point, s_card->cid.name, card_bytes >> 20, total >> 20, free_bytes >> 20,
             s_card->real_freq_khz, 1 << s_card->log_bus_width, s_test.result, s_test.write_kbps,
             s_test.read_kbps);
    return out;
}

}  // namespace rover_sd
