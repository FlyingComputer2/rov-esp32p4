// Rover remote on the Waveshare ESP32-P4-WIFI6-DEV-KIT: Ethernet (IP101,
// DHCP), the winject SDIO proxy (C6 radio, m-plane on UDP 22), winject-l3
// with its config on SPIFFS, and vstreamer live TX (MIPI-CSI camera ->
// hardware H.264 -> RTP -> stream_sender).
//
// The data paths stay in process, on bfc queues registered here and woken by
// one shared cv_reactor (k_dp_reactor):
//   winject-l3 --wifi_tx--> proxy -> SDIO -> C6     (radio.dplane_tx)
//   winject-l3 <--wifi_rx-- proxy <- SDIO <- C6     (radio.dplane_rx)
//   vstreamer --video_tx--> winject-l3 upstream 1   (upstream-1.rx)
//   vstreamer <--video_rx-- winject-l3 upstream 1   (upstream-1.tx)
// Upstream 0 is clock sync (krono SynchClient, UDP console 5110) and upstream
// 2 the bandwidth tester (app_bwtest, UDP console 5100), both on L3-owned
// queues; the tester's console also takes video_stop / video_start (pause
// and resume live TX). Upstream 3 is rover_sample (RC car: RC in, telemetry
// out, MSP v2; UDP console 5120), also on an L3-owned queue. The radio
// m-plane stays on UDP loopback.

#include <inttypes.h>
#include <pthread.h>
#include <stdio.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <bfc/cv_reactor.hpp>
#include <bfc/event_queue.hpp>
#include <bfc/shared_sized_buffer.hpp>

#include "WinjectL3.h"
#include "apps/bwtest/Bwtest.h"
#include "synch/SynchGpio.h"
#include "core/thread_spawn.hpp"
#include "driver/i2c_master.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "esp_pthread.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "rover_sample/rover_sample.hpp"
#include "vstreamer_idf/live_tx.hpp"
#include "winject_proxy.h"

namespace
{

const char* TAG = "rover";

constexpr int k_eth_phy_addr = 1;
constexpr int k_eth_phy_reset_gpio = 51;
constexpr const char* k_config_path = "/winject/winject.conf";
constexpr size_t k_l3_stack_bytes = 16 * 1024;

/* Registry names; config/winject.conf names the same ones. */
constexpr const char* k_dp_reactor = "rover_dp";
constexpr const char* k_wifi_tx = "wifi_tx";
constexpr const char* k_wifi_rx = "wifi_rx";
constexpr const char* k_video_tx = "video_tx";
constexpr const char* k_video_rx = "video_rx";

/* Capacities in datagrams; a full queue drops its oldest. The radio queues
 * carry every MPDU (~600/s at 6 Mbit/s of 1.4 kB frames). video_tx takes the
 * sender's FEC-spread releases, video_rx only the GS's link reports. */
constexpr size_t k_wifi_queue_capacity = 256;
constexpr size_t k_video_tx_capacity = 256;
constexpr size_t k_video_rx_capacity = 16;

/* Runs winject-l3's radio RX demux and upstream 1 (video), and wakes the proxy's
 * d-plane task (which does the blocking SDIO writes). Pinned to core 1: left
 * unpinned it is woken by vs_send (core 1, same priority) and so preempts
 * vs_isp_tx (core 0, priority 13) instead of taking core 1's idle time, and
 * core 0 ran out of time above ~8 Mbit/s. */
constexpr vstreamer::thread_opts k_dp_reactor_thread {"rover_dp", 8 * 1024, 1, 14};

/* Video into winject-l3's upstream 1 (config/winject.conf: QUEUE_FORWARDING,
 * no FEC there; the GS delivers it to its receiver). vstreamer's RS FEC
 * protects the stream end to end; datagrams must stay within winject-l3's
 * 1445-byte application limit, which the payloader's 1400-byte mtu (live TX
 * default) keeps them under. Bench over the air (C6 -> RTL8812AU): on
 * channel 13 (~7% isolated frame loss) 6 Mbit/s with FEC 21/31 and 75 ms
 * spread gave 0-0.7% residual loss at 22-27 fps, 10/12 left 3.8% (3.6 fps)
 * and 15/31 ran core 0 out of time; on channel 1 (~1% loss) every FEC tried
 * was clean. Bitrate (src bitrate), FEC and spread can be changed at run time
 * on the vstreamer m-plane (UDP 5090). */
constexpr int k_video_mplane_port = 5090;

/* Upstream 2 in config/winject.conf: the bandwidth tester's L3-owned queue. */
constexpr size_t k_bwtest_upstream = 2;
constexpr const char* k_bwtest_console = "0.0.0.0:5100";

/* Upstream 0: the clock-sync client's L3-owned queue; the ground P4 is the
 * server and its clock the reference. Interrupt mode pulses GPIO20 (header
 * pin 13, GND on pin 14), open drain, active low, to the ground's GPIO20
 * (JP1 pin 17). */
constexpr size_t k_synch_upstream = 0;
constexpr const char* k_synch_console = "0.0.0.0:5110";
constexpr int k_synch_pulse_gpio = 20;

/* Upstream 3: rover_sample's L3-owned queue (RC from the ground, telemetry
 * back). Its sensors share the board I2C with the camera SCCB and the codec:
 * GPIO7 SDA / GPIO8 SCL (header pins 3 / 5, ESP_I2C_SDA / _SCL), board
 * pull-ups 2.2 k. This file creates the bus before video starts; the camera
 * source then uses it instead of creating (and on teardown deleting) its own. */
constexpr size_t k_rover_sample_upstream = 3;
constexpr const char* k_rover_sample_console = "0.0.0.0:5120";
constexpr i2c_port_num_t k_board_i2c_port = I2C_NUM_0;
constexpr int k_board_i2c_sda = 7;
constexpr int k_board_i2c_scl = 8;

using dp_queue = bfc::reactive_circular_lossy_queue<bfc::shared_sized_buffer>;
using dp_queue_registry = bfc::cv_reactor_circular_lossy_queue_registry<bfc::shared_sized_buffer>;

bool register_queue(const char* name, size_t capacity)
{
    auto q = std::make_shared<dp_queue>();
    q->set_capacity(capacity);
    if (!dp_queue_registry::get_instance().add(name, q))
    {
        ESP_LOGE(TAG, "queue %s: already registered", name);
        return false;
    }
    return true;
}

/* Registers the d-plane queues and reactor and runs the reactor, before the
 * proxy, winject-l3 and vstreamer bind them. All of it lives as long as the
 * program; nothing is unregistered. */
bool start_dplane_queues()
{
    auto cv = std::make_shared<bfc::cv_reactor<>>();
    if (!bfc::cv_reactor_registry::get_instance().add(k_dp_reactor, cv) ||
        !register_queue(k_wifi_tx, k_wifi_queue_capacity) ||
        !register_queue(k_wifi_rx, k_wifi_queue_capacity) ||
        !register_queue(k_video_tx, k_video_tx_capacity) ||
        !register_queue(k_video_rx, k_video_rx_capacity))
    {
        return false;
    }
    vstreamer::spawn_thread(k_dp_reactor_thread, [cv] { cv->run(); }).detach();
    return true;
}

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

/* video_stop / video_start: pause and resume live TX (live_tx_config::
 * pause_request). Camera and encoder keep running; a real stop could not
 * restart, the 1080p encoder cannot be reopened after a run. */
std::atomic<bool> g_video_pause {false};

std::string video_stop_command(const std::string&)
{
    g_video_pause.store(true);
    return "ok\n";
}

std::string video_start_command(const std::string&)
{
    g_video_pause.store(false);
    return "ok\n";
}

/* Runs app_bwtest on its upstream's queue. Its sender and receive poller
 * inherit this thread's settings: low priority, unpinned, so video wins. */
void start_bwtest(std::shared_ptr<winject::UpstreamQueue> up)
{
    static bool started = false;
    if (started)
    {
        ESP_LOGW(TAG, "bwtest upstream replaced; restart to rebind the tester");
        return;
    }
    started = true;
    esp_pthread_cfg_t prev {};
    const bool had_prev = esp_pthread_get_cfg(&prev) == ESP_OK;
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.stack_size = 6 * 1024;
    cfg.prio = 5;
    cfg.thread_name = "bwtest";
    cfg.inherit_cfg = true;
    esp_pthread_set_cfg(&cfg);
    std::thread(
        [up = std::move(up)]
        {
            app_bwtest::Config bw;
            bw.upstream = up;
            bw.console = k_bwtest_console;
            bw.commands = {{"video_stop", video_stop_command},
                           {"video_start", video_start_command}};
            const int rc = app_bwtest::main(bw);
            printf("ROVER_BWTEST_EXIT rc=%d\n", rc);
            fflush(stdout);
        })
        .detach();
    if (had_prev)
    {
        esp_pthread_set_cfg(&prev);
    }
}

/* krono's datagram transport over an L3-owned upstream queue. */
krono::Transport synch_transport(std::shared_ptr<winject::UpstreamQueue> up)
{
    return krono::Transport {
        [up](const uint8_t* data, size_t len) { return up->try_send(data, len); },
        [up](std::vector<uint8_t>* out)
        {
            auto packet = up->receive();
            if (!packet)
            {
                return false;
            }
            const uint8_t* p = reinterpret_cast<const uint8_t*>(packet->data());
            out->assign(p, p + packet->size());
            return true;
        }};
}

/* Runs the clock-sync client on its upstream's queue, rtt mode at first
 * (`mode interrupt` on its console switches). Priority 12, well above the
 * tester, so its timestamps are taken on time; it sleeps between 1 ms polls,
 * so it costs video little. */
void start_synch(std::shared_ptr<winject::UpstreamQueue> up)
{
    static bool started = false;
    if (started)
    {
        ESP_LOGW(TAG, "synch upstream replaced; restart to rebind the client");
        return;
    }
    started = true;
    esp_pthread_cfg_t prev {};
    const bool had_prev = esp_pthread_get_cfg(&prev) == ESP_OK;
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.stack_size = 6 * 1024;
    cfg.prio = 12;
    cfg.thread_name = "synch";
    cfg.inherit_cfg = true;
    esp_pthread_set_cfg(&cfg);
    std::thread(
        [up = std::move(up)]
        {
            krono::ClientConfig sc;
            sc.transport = synch_transport(up);
            sc.console = k_synch_console;
            sc.options.pulse = krono::make_gpio_pulse(k_synch_pulse_gpio);
            const int rc = krono::client_main(sc);
            printf("ROVER_SYNCH_EXIT rc=%d\n", rc);
            fflush(stdout);
        })
        .detach();
    if (had_prev)
    {
        esp_pthread_set_cfg(&prev);
    }
}

/* Runs rover_sample on its upstream's queue. Its link loop (2 ms polls) runs
 * on this thread at priority 10; its 250 Hz control task is a FreeRTOS task
 * on core 1 at priority 16, short per cycle (one 14-byte I2C read). Not
 * inherited: rover_sample sets its own console thread. */
void start_rover_sample(std::shared_ptr<winject::UpstreamQueue> up)
{
    static bool started = false;
    if (started)
    {
        ESP_LOGW(TAG, "rover_sample upstream replaced; restart to rebind it");
        return;
    }
    started = true;
    esp_pthread_cfg_t prev {};
    const bool had_prev = esp_pthread_get_cfg(&prev) == ESP_OK;
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.stack_size = 6 * 1024;
    cfg.prio = 10;
    cfg.thread_name = "rover_sample";
    /* PSRAM stack: internal RAM is what live TX runs short of (rover_sample
     * moves its NVS access to a short-lived internal-stack thread). */
    cfg.stack_alloc_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    esp_pthread_set_cfg(&cfg);
    std::thread(
        [up = std::move(up)]
        {
            rover_sample::config rs;
            rs.link.send = [up](const uint8_t* data, size_t len) { return up->try_send(data, len); };
            rs.link.receive = [up](std::vector<uint8_t>* out)
            {
                auto packet = up->receive();
                if (!packet)
                {
                    return false;
                }
                const uint8_t* p = reinterpret_cast<const uint8_t*>(packet->data());
                out->assign(p, p + packet->size());
                return true;
            };
            rs.console = k_rover_sample_console;
            rs.pin.i2c_port = k_board_i2c_port;
            const int rc = rover_sample::main(rs);
            printf("ROVER_SAMPLE_EXIT rc=%d\n", rc);
            fflush(stdout);
        })
        .detach();
    if (had_prev)
    {
        esp_pthread_set_cfg(&prev);
    }
}

/* The board I2C (camera SCCB, ES8311, rover_sample's GY-521 / GY-271). Kept
 * for the life of the program. */
esp_err_t start_board_i2c()
{
    i2c_master_bus_config_t cfg = {};
    cfg.i2c_port = k_board_i2c_port;
    cfg.sda_io_num = static_cast<gpio_num_t>(k_board_i2c_sda);
    cfg.scl_io_num = static_cast<gpio_num_t>(k_board_i2c_scl);
    cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt = 7;
    i2c_master_bus_handle_t bus = nullptr;
    esp_err_t err = i2c_new_master_bus(&cfg, &bus);
    if (err != ESP_OK)
    {
        return err;
    }
    /* A reset during a sensor read (rover_sample reads the IMU at 250 Hz) can
     * leave the slave holding SDA low; the driver clears the bus only once it
     * has seen it busy, so the camera's sensor probe failed. Clock it free
     * (9 SCL pulses and a STOP) before anyone uses the bus. */
    return i2c_master_bus_reset(bus);
}

void* run_l3(void*)
{
    winject::Config cfg;
    /* Set before load(): its validation needs the callback for an L3-owned
     * queue upstream. */
    cfg.on_upstream_queue = [](size_t index, std::shared_ptr<winject::UpstreamQueue> up)
    {
        if (index == k_bwtest_upstream)
        {
            start_bwtest(std::move(up));
        }
        else if (index == k_synch_upstream)
        {
            start_synch(std::move(up));
        }
        else if (index == k_rover_sample_upstream)
        {
            start_rover_sample(std::move(up));
        }
    };
    std::string err;
    if (!cfg.load(k_config_path, &err))
    {
        printf("ROVER_L3_CONFIG_FAILED %s\n", err.c_str());
        fflush(stdout);
        return nullptr;
    }
    const int rc = winject_l3::main(cfg);
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
           " dup_fail=%" PRIu32 " dup_evict=%" PRIu32 " link_rx_bad=%" PRIu32
           " link_rx_err=%" PRIu32 " link_tx_err=%" PRIu32 " link_downs=%" PRIu32
           " link_ups=%" PRIu32 "\n",
           ps.mplane_requests, ps.mplane_replies, ps.mplane_dropped, ps.dplane_down,
           ps.dplane_down_dropped, ps.dplane_up, ps.dplane_up_no_peer, ps.dplane_up_failed,
           ps.dplane_up_evicted, ls.rx_bad, ls.rx_errors, ls.tx_errors, ls.link_downs,
           ls.link_ups);
    fflush(stdout);
}

vstreamer::idf::live_tx_config video_config()
{
    vstreamer::idf::live_tx_config cfg;
    cfg.queue_tx = k_video_tx;
    cfg.queue_rx = k_video_rx;
    cfg.queue_reactor = k_dp_reactor;
    cfg.mplane_port = k_video_mplane_port;
    cfg.src = {{"buffers", "3"}, {"fps", "30"},    {"bitrate", "6000000"},
               {"gop", "30"},    {"qp_min", "20"}, {"qp_max", "51"}};
    /* In order: fec_k before fec_n, since k may not exceed the current n (sender starts at
     * k=10, n=12). */
    cfg.snd = {{"telemetry", "off"}, {"fec", "block"},       {"fec_k", "6"},
               {"fec_n", "8"},       {"fec_spread_ms", "4"}};
    return cfg;
}

void run_video()
{
    vstreamer::idf::live_tx_config cfg = video_config();
    cfg.pause_request = &g_video_pause;
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

    /* Without the queues the proxy still starts (UDP d-plane), so POST
     * /update/p4 can replace this image; winject-l3 and video stay off. */
    const bool queues = start_dplane_queues();
    if (!queues)
    {
        printf("ROVER_QUEUES_FAILED\n");
    }

    winject_proxy_config_t cfg = WINJECT_PROXY_CONFIG_DEFAULT();
    /* This board wires the C6's GPIO9 (boot strap) to P4 GPIO22, outside the
     * LDO-powered GPIO39-48 domain the default (GPIO45, LDO_VO4) assumes. */
    cfg.link.boot_gpio = 22;
    cfg.link.boot_gpio_ldo = -1;
    if (queues)
    {
        cfg.dplane_tx_queue = k_wifi_tx;
        cfg.dplane_rx_queue = k_wifi_rx;
        cfg.dplane_reactor = k_dp_reactor;
    }
    err = winject_proxy_start(&cfg);
    if (err != ESP_OK)
    {
        printf("ROVER_PROXY_FAILED err=0x%x\n", err);
    }

    /* Before video: the camera source adopts this bus. */
    err = start_board_i2c();
    if (err != ESP_OK)
    {
        printf("ROVER_I2C_FAILED err=%s\n", esp_err_to_name(err));
    }

    if (queues)
    {
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
    }
    printf("ROVER_READY\n");
    fflush(stdout);

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        print_stats();
    }
}
