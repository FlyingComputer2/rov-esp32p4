#include "udp_ota/udp_ota.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "server.hpp"

namespace udp_ota
{
namespace
{

const char* TAG = "udp_ota";

const char* img_state_name(const esp_partition_t* p)
{
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(p, &st) != ESP_OK)
    {
        return "unknown";
    }
    switch (st)
    {
    case ESP_OTA_IMG_NEW: return "new";
    case ESP_OTA_IMG_PENDING_VERIFY: return "pending";
    case ESP_OTA_IMG_VALID: return "valid";
    case ESP_OTA_IMG_INVALID: return "invalid";
    case ESP_OTA_IMG_ABORTED: return "aborted";
    default: return "undefined";
    }
}

/* The inactive OTA slot. esp_ota_begin() would erase the whole image size up
 * front (seconds for ~1.8 MB, with the start reply waiting on it), so it is
 * opened with a 1-byte size (erases sector 0 only) and every other sector is
 * erased on its first write. Fragments arrive in any order:
 * esp_ota_write_with_offset. */
class ota_slot final : public backend
{
public:
    size_t capacity() override
    {
        const esp_partition_t* p = esp_ota_get_next_update_partition(nullptr);
        return p ? p->size : 0;
    }

    bool begin() override
    {
        abort();
        const esp_partition_t* running = esp_ota_get_running_partition();
        /* A finished image not booted yet: point the boot back at the running
         * one before its slot is erased. */
        if (esp_ota_get_boot_partition() != running)
        {
            esp_ota_set_boot_partition(running);
        }
        m_part = esp_ota_get_next_update_partition(nullptr);
        if (!m_part)
        {
            return false;
        }
        const esp_err_t err = esp_ota_begin(m_part, 1, &m_handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "begin %s: %s", m_part->label, esp_err_to_name(err));
            m_handle = 0;
            return false;
        }
        m_erased.assign((m_part->size + m_part->erase_size - 1) / m_part->erase_size, false);
        m_erased[0] = true;
        return true;
    }

    bool write(uint32_t offset, const uint8_t* data, size_t len) override
    {
        if (!m_handle)
        {
            return false;
        }
        const uint32_t es = m_part->erase_size;
        for (uint32_t s = offset / es; s <= (offset + len - 1) / es; ++s)
        {
            if (!m_erased[s])
            {
                const esp_err_t err = esp_partition_erase_range(m_part, s * es, es);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "erase sector %u: %s", (unsigned)s, esp_err_to_name(err));
                    return false;
                }
                m_erased[s] = true;
            }
        }
        const esp_err_t err = esp_ota_write_with_offset(m_handle, data, len, offset);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "write at %u: %s", (unsigned)offset, esp_err_to_name(err));
            return false;
        }
        return true;
    }

    status finish(uint32_t size) override
    {
        if (!m_handle)
        {
            return status::eio;
        }
        /* esp_ota_end frees the handle on every path. */
        esp_err_t err = esp_ota_end(m_handle);
        m_handle = 0;
        if (err == ESP_OK)
        {
            err = esp_ota_set_boot_partition(m_part);
        }
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "image %u B in %s rejected: %s", (unsigned)size, m_part->label, esp_err_to_name(err));
            return err == ESP_ERR_OTA_VALIDATE_FAILED ? status::ebadmsg : status::eio;
        }
        ESP_LOGI(TAG, "image %u B in %s set to boot", (unsigned)size, m_part->label);
        return status::ok;
    }

    void abort() override
    {
        if (m_handle)
        {
            esp_ota_abort(m_handle);
            m_handle = 0;
        }
    }

    std::string describe() override
    {
        const esp_partition_t* running = esp_ota_get_running_partition();
        const esp_partition_t* boot = esp_ota_get_boot_partition();
        const esp_app_desc_t* app = esp_app_get_description();
        char elf[17];
        esp_app_get_elf_sha256(elf, sizeof(elf));
        char line[256];
        std::snprintf(line, sizeof(line), "running=%s state=%s boot=%s project=%s version=%s built=%s_%s elf=%s",
                      running->label, img_state_name(running), boot ? boot->label : "none", app->project_name,
                      app->version, app->date, app->time, elf);
        std::string out(line);
        /* app->date is "Nov  5 2026": keep the line space-separated key=value. */
        const size_t b = out.find("built=");
        const size_t e = out.find(" elf=");
        for (size_t i = b; b != std::string::npos && i < e && i < out.size(); ++i)
        {
            if (out[i] == ' ')
            {
                out[i] = '_';
            }
        }
        return out;
    }

private:
    const esp_partition_t* m_part = nullptr;
    esp_ota_handle_t m_handle = 0;
    std::vector<bool> m_erased;
};

struct task_args
{
    config cfg;
    int fd;
};

uint32_t now_ms()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

void server_task(void* arg)
{
    auto* a = static_cast<task_args*>(arg);
    ota_slot slot;
    posix_file_store files(a->cfg.fs_dir);
    server srv(slot, a->cfg.idle_timeout_ms, a->cfg.on_active, a->cfg.fs_dir.empty() ? nullptr : &files);
    static uint8_t req[1600];
    static uint8_t reply[server::k_reply_max];
    for (;;)
    {
        sockaddr_in from {};
        socklen_t from_len = sizeof(from);
        const int n = recvfrom(a->fd, req, sizeof(req), 0, reinterpret_cast<sockaddr*>(&from), &from_len);
        if (n < 0)
        {
            srv.tick(now_ms());  // receive timeout
            continue;
        }
        peer p;
        p.addr = from.sin_addr.s_addr;
        p.port = from.sin_port;
        const size_t r = srv.handle(req, static_cast<size_t>(n), p, now_ms(), reply);
        if (r)
        {
            sendto(a->fd, reply, r, 0, reinterpret_cast<sockaddr*>(&from), from_len);
        }
        if (srv.reboot_requested())
        {
            printf("UDP_OTA reboot\n");
            fflush(stdout);
            /* Give the reply time to leave (over the air: L3 TX queue). */
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        }
    }
}

}  // namespace

esp_err_t start(const config& cfg)
{
    const int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0)
    {
        return ESP_FAIL;
    }
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(cfg.port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
    {
        close(fd);
        return ESP_FAIL;
    }
    timeval tv {};
    tv.tv_sec = 1;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    auto* a = new task_args {cfg, fd};
    const BaseType_t core = cfg.task_core < 0 ? tskNO_AFFINITY : cfg.task_core;
    if (xTaskCreatePinnedToCore(server_task, "udp_ota", cfg.task_stack, a, cfg.task_prio, nullptr, core) != pdPASS)
    {
        close(fd);
        delete a;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "UDP %u", (unsigned)cfg.port);
    return ESP_OK;
}

}  // namespace udp_ota
