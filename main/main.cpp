// Rover remote on the Waveshare ESP32-P4-WIFI6-DEV-KIT: Ethernet (IP101,
// DHCP), the winject SDIO proxy (C6 radio on UDP 22 / 9000), winject-l3
// using that radio over loopback with its config on SPIFFS, and vstreamer
// live TX (MIPI-CSI camera -> hardware H.264 -> RTP -> stream_sender) into
// winject-l3's video upstream (1) on loopback.

#include <inttypes.h>
#include <pthread.h>
#include <stdio.h>

#include <thread>

#include "WinjectL3.h"
#include "core/thread_spawn.hpp"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "vstreamer_idf/live_tx.hpp"
#include "winject_proxy.h"

namespace
{

const char* TAG = "rover";

constexpr int k_eth_phy_addr = 1;
constexpr int k_eth_phy_reset_gpio = 51;
constexpr const char* k_config_path = "/winject/winject.conf";
constexpr size_t k_l3_stack_bytes = 16 * 1024;

/* Video into winject-l3's upstream 1 (config/winject.conf: UDP server on
 * 22081, no FEC there; the GS delivers it to its receiver). vstreamer's RS FEC
 * protects the stream end to end; datagrams must stay within winject-l3's
 * 1445-byte application limit. Bench over the air (C6 -> RTL8812AU, ~7%
 * bursty frame loss): 6 Mbit/s with FEC 21/31 and 75 ms spread gave 0-0.4%
 * residual loss at 24.5-27 fps; 10/12 left 3.8% (3.6 fps) and 15/31 ran
 * core 0 out of time. Tunable at run time on the vstreamer m-plane (UDP 5090). */
constexpr const char* k_video_peer = "127.0.0.1:22081";
constexpr int k_video_mplane_port = 5090;

void on_got_ip(void*, esp_event_base_t, int32_t, void* data)
{
    const auto* ev = static_cast<const ip_event_got_ip_t*>(data);
    printf("ROVER_IP ip=" IPSTR "\n", IP2STR(&ev->ip_info.ip));
    fflush(stdout);
}

void on_eth(void*, esp_event_base_t, int32_t id, void*)
{
    if (id == ETHERNET_EVENT_CONNECTED)
    {
        ESP_LOGI(TAG, "Ethernet link up");
    }
    else if (id == ETHERNET_EVENT_DISCONNECTED)
    {
        ESP_LOGW(TAG, "Ethernet link down");
    }
}

esp_err_t start_ethernet()
{
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t* netif = esp_netif_new(&netif_cfg);
    esp_netif_set_hostname(netif, "rover-remote");

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr = k_eth_phy_addr;
    phy_cfg.reset_gpio_num = k_eth_phy_reset_gpio;
    esp_eth_mac_t* mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);
    esp_eth_phy_t* phy = esp_eth_phy_new_ip101(&phy_cfg);
    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth = nullptr;
    esp_err_t err = esp_eth_driver_install(&eth_cfg, &eth);
    if (err != ESP_OK)
    {
        return err;
    }
    esp_netif_attach(netif, esp_eth_new_netif_glue(eth));
    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, on_eth, nullptr);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, on_got_ip, nullptr);
    return esp_eth_start(eth);
}

esp_err_t mount_config()
{
    esp_vfs_spiffs_conf_t conf = {};
    conf.base_path = "/winject";
    conf.partition_label = "storage";
    conf.max_files = 4;
    conf.format_if_mount_failed = false;
    return esp_vfs_spiffs_register(&conf);
}

void* run_l3(void*)
{
    char arg0[] = "winject-l3";
    char arg1[64];
    snprintf(arg1, sizeof(arg1), "%s", k_config_path);
    char* argv[] = {arg0, arg1, nullptr};
    const int rc = winject_l3::main(2, argv);
    printf("ROVER_L3_EXIT rc=%d\n", rc);
    fflush(stdout);
    return nullptr;
}

void start_l3()
{
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, k_l3_stack_bytes);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    pthread_t t;
    const int err = pthread_create(&t, &attr, run_l3, nullptr);
    pthread_attr_destroy(&attr);
    if (err != 0)
    {
        ESP_LOGE(TAG, "winject-l3 thread: %d", err);
    }
}

void print_stats()
{
    winject_proxy_stats_t ps;
    sdio_link_host_stats_t ls;
    winject_proxy_get_stats(&ps);
    sdio_link_host_get_stats(&ls);
    printf("ROVER_PROXY_STATS mreq=%" PRIu32 " mrep=%" PRIu32 " mdrop=%" PRIu32
           " ddown=%" PRIu32 " ddown_drop=%" PRIu32 " dup=%" PRIu32 " dup_nopeer=%" PRIu32
           " dup_fail=%" PRIu32 " link_rx_bad=%" PRIu32 " link_rx_err=%" PRIu32
           " link_tx_err=%" PRIu32 " link_downs=%" PRIu32 " link_ups=%" PRIu32 "\n",
           ps.mplane_requests, ps.mplane_replies, ps.mplane_dropped, ps.dplane_down,
           ps.dplane_down_dropped, ps.dplane_up, ps.dplane_up_no_peer, ps.dplane_up_failed,
           ls.rx_bad, ls.rx_errors, ls.tx_errors, ls.link_downs, ls.link_ups);
    fflush(stdout);
}

vstreamer::idf::live_tx_config video_config()
{
    vstreamer::idf::live_tx_config cfg;
    cfg.peer = k_video_peer;
    cfg.mplane_port = k_video_mplane_port;
    cfg.src = {{"buffers", "6"}, {"fps", "30"},    {"bitrate", "6000000"},
               {"gop", "30"},    {"qp_min", "20"}, {"qp_max", "51"}};
    /* In order: fec_n before fec_k, since k may not exceed the current n. */
    cfg.snd = {{"mtu", "1400"},      {"max_datagram", "1445"}, {"max_kbps", "0"},
               {"telemetry", "off"}, {"fec", "block"},         {"fec_n", "31"},
               {"fec_k", "21"},      {"fec_spread_ms", "75"}};
    return cfg;
}

void run_video()
{
    const vstreamer::idf::live_tx_config cfg = video_config();
    for (;;)
    {
        const int rc = vstreamer::idf::run_live_tx(cfg);
        printf("ROVER_VIDEO_EXIT rc=%d, restarting in 3 s\n", rc);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/* vstreamer pipeline control needs a pthread, not app_main; run_live_tx blocks. */
void start_video()
{
    vstreamer::spawn_thread(vstreamer::thread_opts {"rover_video", 12 * 1024}, run_video)
        .detach();
}

}  // namespace

extern "C" void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    err = start_ethernet();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Ethernet: %s", esp_err_to_name(err));
    }

    winject_proxy_config_t cfg = WINJECT_PROXY_CONFIG_DEFAULT();
    err = winject_proxy_start(&cfg);
    if (err != ESP_OK)
    {
        printf("ROVER_PROXY_FAILED err=0x%x\n", err);
    }

    err = mount_config();
    if (err != ESP_OK)
    {
        printf("ROVER_CONFIG_FAILED err=%s\n", esp_err_to_name(err));
    }
    else
    {
        start_l3();
    }
    start_video();
    printf("ROVER_READY\n");
    fflush(stdout);

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        print_stats();
    }
}
