// Rover remote on the Waveshare ESP32-P4-WIFI6-DEV-KIT: Ethernet (IP101,
// DHCP), the winject SDIO proxy (C6 radio on UDP 22 / 9000) and winject-l3
// using that radio over loopback, with its config on SPIFFS.

#include <inttypes.h>
#include <pthread.h>
#include <stdio.h>

#include "WinjectL3.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "winject_proxy.h"

namespace
{

const char* TAG = "rover";

constexpr int k_eth_phy_addr = 1;
constexpr int k_eth_phy_reset_gpio = 51;
constexpr const char* k_config_path = "/winject/winject.conf";
constexpr size_t k_l3_stack_bytes = 16 * 1024;

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
    printf("ROVER_READY\n");
    fflush(stdout);

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        print_stats();
    }
}
