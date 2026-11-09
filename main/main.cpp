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
// Upstream 0 is clock sync (krono SynchClient, UDP console 5110), 2 the
// bandwidth tester (app_bwtest, UDP console 5100) and 3 rover_sample (RC car:
// RC in, telemetry out, MSP v2; UDP console 5120). Upstreams 7 and 8 are the
// tester's and the clock-sync client's m-planes, served directly on their
// queues (request in, reply out; no socket). All five are on named queues
// registered here (L3's side on rover_dp) and run as applets on one shared
// reactor, "apps" (see start_applets). The radio m-plane
// stays on UDP loopback. App and C6 updates go over the SDIO proxy's udp_ota
// (UDP 3232), on Ethernet or over the air through upstream 9 (a static UDP
// upstream to 127.0.0.1:3232).

#include <inttypes.h>
#include <pthread.h>
#include <stdio.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include <bfc/cv_reactor.hpp>
#include <bfc/event_queue.hpp>
#include <bfc/shared_sized_buffer.hpp>

#include "WinjectL3.h"
#include "winject_l3/Console.h"
#include "synch/SynchGpio.h"
#include "core/thread_spawn.hpp"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "esp_pthread.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "hal/wdt_hal.h"
#include "riscv/rv_utils.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "applets.hpp"
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
/* Core for the winject-l3 thread; see start_l3(). */
constexpr int k_l3_cpu = 1;

/* The L3 upstream reactor handles queues and sockets on a PSRAM stack.
 * The L3 main thread reads its config from flash on an internal stack. */
constexpr size_t k_l3_child_stack_bytes = 6 * 1024;
constexpr int k_l3_child_prio = 12;

/* Thread policy. Stacks go to PSRAM unless a thread writes flash (a PSRAM
 * stack is unusable while the cache is off) or is timing-critical: internal
 * RAM is what live TX runs short of (~10-20 KB left once the 1080p encoder
 * holds its reference frame). */
/* Configure an unpinned pthread with a PSRAM stack. */
esp_pthread_cfg_t psram_thread_cfg(const char* name, size_t stack, int prio, bool inherit = false)
{
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.thread_name = name;
    cfg.stack_size = stack;
    cfg.prio = prio;
    cfg.inherit_cfg = inherit;
    cfg.stack_alloc_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    return cfg;
}

/* esp_pthread_set_cfg() applies to every thread the calling thread creates
 * next. Sets cfg for this scope, then restores the previous config, or the
 * default when there was none, so it cannot leak into the caller's later
 * threads. */
class scoped_pthread_cfg
{
public:
    explicit scoped_pthread_cfg(const esp_pthread_cfg_t& cfg)
        : had_prev(esp_pthread_get_cfg(&prev) == ESP_OK)
    {
        esp_pthread_set_cfg(&cfg);
    }
    ~scoped_pthread_cfg()
    {
        const esp_pthread_cfg_t restore = had_prev ? prev : esp_pthread_get_default_config();
        esp_pthread_set_cfg(&restore);
    }
    scoped_pthread_cfg(const scoped_pthread_cfg&) = delete;
    scoped_pthread_cfg& operator=(const scoped_pthread_cfg&) = delete;

private:
    esp_pthread_cfg_t prev {};
    bool had_prev;
};

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
 * on the vstreamer m-plane (UDP 5090), where `set tx pause 1` / `0` also
 * pauses and resumes live TX. */
constexpr int k_video_mplane_port = 5090;

/* The applets' upstreams (config/winject.conf): QUEUE_FORWARDING on named
 * queues (`rx` = <name>_tx, the app's send side; `tx` = <name>_rx), L3's side
 * on reactor rover_dp. main.cpp registers them before winject-l3 starts; the
 * applets find them by name and are woken when L3 pushes.
 *   0 clock sync (krono client)        synch_tx / synch_rx
 *   2 bandwidth tester                 bwtest_tx / bwtest_rx
 *   3 rover_sample RC / telemetry      rc_tx / rc_rx
 *   7 tester m-plane                   bwtest_mp_tx / bwtest_mp_rx
 *   8 clock-sync m-plane               synch_mp_tx / synch_mp_rx
 * 7 and 8 serve the same commands as the UDP consoles; the ground is a
 * udp_server on 5102 / 5112 (replies to the last sender, one operator at a
 * time). */
struct app_upstream
{
    applets::upstream_names names;
    size_t capacity;
};
const app_upstream k_synch_up {{"synch_tx", "synch_rx"}, 16};
const app_upstream k_bwtest_up {{"bwtest_tx", "bwtest_rx"}, 256};
const app_upstream k_rc_up {{"rc_tx", "rc_rx"}, 16};
const app_upstream k_bwtest_mplane_up {{"bwtest_mp_tx", "bwtest_mp_rx"}, 16};
const app_upstream k_synch_mplane_up {{"synch_mp_tx", "synch_mp_rx"}, 16};

constexpr const char* k_bwtest_console = "0.0.0.0:5100";

/* The clock-sync client: the ground P4 is the server and its clock the
 * reference. Interrupt mode pulses GPIO20 (header pin 13, GND on pin 14),
 * open drain, active low, to the ground's GPIO20 (JP1 pin 17). */
constexpr const char* k_synch_console = "0.0.0.0:5110";
constexpr int k_synch_pulse_gpio = 20;

/* rover_sample: its sensors share the board I2C with the camera SCCB and the
 * codec: GPIO7 SDA / GPIO8 SCL (header pins 3 / 5, ESP_I2C_SDA / _SCL), board
 * pull-ups 2.2 k. This file creates the bus before video starts; the camera
 * source then uses it instead of creating (and on teardown deleting) its own. */
/* udp_ota (in the SDIO proxy): the proxy's tools/udp_ota.py <rover IP>:3232
 * on Ethernet; over the air the ground's UDP 2430 reaches it through upstream
 * 9 (config/winject.conf). It updates this app and the C6, and serves the
 * config partition's files. */
constexpr const char* k_ota_fs_dir = "/winject";
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

bool register_app_upstream(const app_upstream& u)
{
    return register_queue(u.names.to_l3.c_str(), u.capacity) && register_queue(u.names.from_l3.c_str(), u.capacity);
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
        !register_queue(k_video_rx, k_video_rx_capacity) ||
        !register_app_upstream(k_synch_up) || !register_app_upstream(k_bwtest_up) ||
        !register_app_upstream(k_rc_up) || !register_app_upstream(k_bwtest_mplane_up) ||
        !register_app_upstream(k_synch_mplane_up))
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

/* Live TX pause (live_tx_config::pause): set over the vstreamer m-plane
 * (`set tx pause 1|0`, UDP 5090). Owned here so it survives a live TX
 * restart. */
std::atomic<bool> g_video_pause {false};

/* udp_ota's on_ota_active. Video pauses while an image comes in (it would
 * take most of the airtime) or the C6 is written (its radio stalls on every
 * flash write), and goes back to what it was afterwards. */
void on_ota_active(bool active, void*)
{
    static bool paused_before = false;
    if (active)
    {
        paused_before = g_video_pause.exchange(true);
    }
    else
    {
        g_video_pause.store(paused_before);
    }
}

/* The firmware's IO reactor, a bfc::default_reactor<> registered as "apps" in
 * bfc::default_reactor_registry: the applets attach to it by name, and
 * winject-l3 runs its reactor (radio m-plane, TX timers, UDP upstreams) and its
 * UDP console on it (L3::Reactors). Socket, queue and timer callbacks only (no
 * flash), so a PSRAM stack, 16 KB as l3's own reactor thread had; priority 12,
 * as l3's console and upstream threads had. Threads its callbacks create set
 * their own settings. */
constexpr const char* k_apps_reactor = "apps";
std::shared_ptr<bfc::default_reactor<>> g_apps;

void create_apps_reactor()
{
    g_apps = std::make_shared<bfc::default_reactor<>>();
    if (!bfc::default_reactor_registry::get_instance().add(k_apps_reactor, g_apps))
    {
        printf("ROVER_APPS_REACTOR_FAILED\n");
    }
}

/* After the applets registered their callbacks (registration is not
 * thread-safe), for the life of the program. Pinned to core 1: left unpinned,
 * the RISC-V port pins a task to the core it is on when it first uses the FPU
 * (rover_sample's callbacks do), and on core 0 vs_isp_tx (priority 13, ~95%
 * at 6 Mbit/s) starved it: the consoles went silent and RC telemetry stopped
 * (as winject-l3's thread, see start_l3). */
void run_apps_reactor()
{
    esp_pthread_cfg_t cfg = psram_thread_cfg("apps", k_l3_stack_bytes, k_l3_child_prio);
    cfg.pin_to_core = 1;
    const scoped_pthread_cfg scope(cfg);
    std::thread([r = g_apps] { r->run(); }).detach();
}

/* Configures and starts the applets on the app reactor and their named
 * upstream queues, before winject-l3 starts (it binds the same queues). They
 * live for the program. */
void start_applets()
{
    static applets::BwtestApplet bwtest;
    applets::bwtest_config bw;
    bw.reactor = k_apps_reactor;
    bw.data = k_bwtest_up.names;
    /* test_send's sender: low priority so video wins. */
    bw.sender = psram_thread_cfg("bwtest", 6 * 1024, 5);
    bw.mplane = k_bwtest_mplane_up.names;
    bw.console = k_bwtest_console;
    if (!bwtest.configure(bw) || !bwtest.start())
    {
        printf("ROVER_BWTEST_EXIT rc=1\n");
    }

    /* rtt mode at first (`mode interrupt` on its console switches). */
    static applets::SynchClientApplet synch;
    applets::synch_client_config sc;
    sc.reactor = k_apps_reactor;
    sc.data = k_synch_up.names;
    /* Priority 12 so its timestamps are taken on time; it sleeps between 1 ms
     * polls, so it costs video little. */
    sc.worker = psram_thread_cfg("synch", 6 * 1024, 12);
    sc.mplane = k_synch_mplane_up.names;
    sc.console = k_synch_console;
    sc.options.pulse = krono::make_gpio_pulse(k_synch_pulse_gpio);
    if (!synch.configure(sc) || !synch.start())
    {
        printf("ROVER_SYNCH_EXIT rc=1\n");
    }

    /* Its 250 Hz control task (core 1, priority 16, one 14-byte I2C read per
     * cycle) is its own; its RC link and console (UDP 5120) are on the app
     * reactor. Needs the board I2C bus. */
    static rover_sample::Applet car;
    rover_sample::config rs;
    rs.reactor = k_apps_reactor;
    rs.link_to_l3 = k_rc_up.names.to_l3;
    rs.link_from_l3 = k_rc_up.names.from_l3;
    rs.console = k_rover_sample_console;
    rs.pin.i2c_port = k_board_i2c_port;
    if (!car.configure(rs) || !car.start())
    {
        printf("ROVER_SAMPLE_EXIT rc=1\n");
    }
    fflush(stdout);
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
    /* For the threads winject-l3 starts; the upstream callbacks below set
     * their own for their threads and restore this one. */
    const esp_pthread_cfg_t child =
        psram_thread_cfg("l3_up", k_l3_child_stack_bytes, k_l3_child_prio, true);
    esp_pthread_set_cfg(&child);
    winject::Config cfg;
    std::string err;
    if (!cfg.load(k_config_path, &err))
    {
        printf("ROVER_L3_CONFIG_FAILED %s\n", err.c_str());
        fflush(stdout);
        return nullptr;
    }
    // l3's UDP console, on the app reactor like l3 itself.
    winject::l3::console::Service console;
    auto started = [&](const winject::L3& l3)
    {
        if (!cfg.console_in.empty())
        {
            const winject::Status s =
                console.start(l3, cfg.console_in, k_apps_reactor);
            if (!s.ok())
            {
                printf("L3_CONSOLE_FAILED %s\n", s.error.c_str());
            }
        }
    };
    /* No threads of l3's own: its reactor runs on the app reactor (core 1, see
     * run_apps_reactor) and its upstream data path on the d-plane reactor
     * (rover_dp, core 1), next to the queues it serves. */
    const int rc = winject::L3::run(cfg, started, {},
                                    winject::L3::Reactors {k_apps_reactor, k_dp_reactor});
    console.stop();
    printf("ROVER_L3_EXIT rc=%d\n", rc);
    fflush(stdout);
    return nullptr;
}

/* winject-l3's console, UDP upstreams and timers run on this thread, at the
 * default pthread priority (5). It must be pinned to core 1: left unpinned,
 * the RISC-V port pins it to whichever core it is on the first time it uses
 * the FPU, which was core 0, where vs_isp_tx (priority 13) takes ~95% at
 * 6 Mbit/s. With video running the console and the m-plane upstreams 4-6 then
 * went seconds without an answer while core 1 sat ~58% idle. */
void start_l3()
{
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.pin_to_core = k_l3_cpu;
    const scoped_pthread_cfg scope(cfg);
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

/* Free heap in bytes: internal RAM is what live TX runs short of (its
 * threads' stacks and the 1080p encoder's reference frame). */
void print_heap(const char* when)
{
    printf("ROVER_HEAP %s internal=%u internal_largest=%u psram=%u\n", when,
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    fflush(stdout);
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
    /* On the app reactor (core 1), not a vs_mplane thread of its own. */
    cfg.mplane_reactor = k_apps_reactor;
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
    cfg.pause = &g_video_pause;
    for (;;)
    {
        const int rc = vstreamer::idf::run_live_tx(cfg);
        printf("ROVER_VIDEO_EXIT rc=%d, restarting in 3 s\n", rc);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/* Indicator LED for the glass-to-glass check: lit for 100 ms every 1000 ms
 * (the first lit at 1 s after boot). GPIO22 is not the C6 boot strap on this
 * board (that is GPIO47, see app_main). Driven from esp_timer, not from the
 * L3 or video threads. */
constexpr gpio_num_t k_led_gpio = GPIO_NUM_22;
constexpr int64_t k_led_period_us = 1000 * 1000;
constexpr int64_t k_led_on_us = 100 * 1000;
esp_timer_handle_t g_led_off_timer = nullptr;

void led_off_cb(void*)
{
    gpio_set_level(k_led_gpio, 0);
}

void led_on_cb(void*)
{
    gpio_set_level(k_led_gpio, 1);
    esp_timer_start_once(g_led_off_timer, k_led_on_us);
}

esp_err_t start_led_blink()
{
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << k_led_gpio;
    io.mode = GPIO_MODE_OUTPUT;
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK)
    {
        return err;
    }
    gpio_set_level(k_led_gpio, 0);

    esp_timer_create_args_t off_args = {};
    off_args.callback = led_off_cb;
    off_args.name = "led_off";
    err = esp_timer_create(&off_args, &g_led_off_timer);
    if (err != ESP_OK)
    {
        return err;
    }

    esp_timer_create_args_t on_args = {};
    on_args.callback = led_on_cb;
    on_args.name = "led_on";
    esp_timer_handle_t on_timer = nullptr;
    err = esp_timer_create(&on_args, &on_timer);
    if (err != ESP_OK)
    {
        return err;
    }
    return esp_timer_start_periodic(on_timer, k_led_period_us);
}

/* vstreamer pipeline control needs a pthread, not app_main; run_live_tx blocks. */
void start_video()
{
    // The supervisor's stack is in PSRAM. Video workers choose their own stacks.
    const auto cfg = psram_thread_cfg("rover_video", 12 * 1024, CONFIG_PTHREAD_TASK_PRIO_DEFAULT);
    const scoped_pthread_cfg scope(cfg);
    std::thread(run_video).detach();
}

}  // namespace

/* Workaround for an ESP32-P4 boot hang (seen on rev v3.1, IDF v5.5.5): esp_restart_noos() resets
 * only the CPU cores (RESET_REASON_CPU_SW). After that, the next boot often hangs at its first
 * L2-cache write-back to PSRAM (in practice inside the PSRAM memtest, right after "cpu_start:
 * Multicore app") until the bootloader's 9 s RTC watchdog does a system reset. A freshly OTA'd
 * image is still pending then, so the bootloader rolls it back. Boots after a system reset never
 * hang. So every software restart (esp_restart(), panics, cpu_start and USB-console restarts,
 * which all call esp_restart_noos) is turned into an RTC watchdog system reset; main's
 * CMakeLists links with -Wl,--wrap=esp_restart_noos. Shutdown handlers still run first (inside
 * esp_restart()). */
extern "C" void __real_esp_restart_noos(void);

extern "C" void IRAM_ATTR __wrap_esp_restart_noos(void)
{
    rv_utils_intr_global_disable();
    wdt_hal_context_t rwdt;
    wdt_hal_init(&rwdt, WDT_RWDT, 0, false);
    wdt_hal_write_protect_disable(&rwdt);
    /* A few slow-clock ticks: well under 10 ms. */
    wdt_hal_config_stage(&rwdt, WDT_STAGE0, 100, WDT_STAGE_ACTION_RESET_SYSTEM);
    wdt_hal_config_stage(&rwdt, WDT_STAGE1, 100, WDT_STAGE_ACTION_RESET_RTC);
    wdt_hal_set_flashboot_en(&rwdt, true);
    wdt_hal_enable(&rwdt);
    wdt_hal_write_protect_enable(&rwdt);
    /* The watchdog should fire within milliseconds; if it does not, restart the IDF way. */
    esp_rom_delay_us(100 * 1000);
    __real_esp_restart_noos();
}

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

    /* Indicator LED on GPIO22, 100 ms in every 1000 ms. */
    if (start_led_blink() != ESP_OK)
    {
        printf("ROVER_LED_FAILED\n");
    }

    err = start_ethernet();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Ethernet: %s", esp_err_to_name(err));
    }

    /* Before the applets, which attach to it by name. */
    create_apps_reactor();

    /* Before the proxy, whose udp_ota serves its files (ota_fs_*), and
     * whatever the queues do: a broken winject.conf can then still be
     * replaced over Ethernet. */
    const esp_err_t config_err = mount_config();
    if (config_err != ESP_OK)
    {
        printf("ROVER_CONFIG_FAILED err=%s\n", esp_err_to_name(config_err));
    }

    /* Without the queues the proxy still starts (UDP d-plane) and udp_ota
     * can replace this image over Ethernet; winject-l3 and video stay off. */
    const bool queues = start_dplane_queues();
    if (!queues)
    {
        printf("ROVER_QUEUES_FAILED\n");
    }

    winject_proxy_config_t cfg = WINJECT_PROXY_CONFIG_DEFAULT();
    /* C6 GPIO9 (boot strap) is wired to P4 GPIO47. GPIO46/48 are reserved
     * for the C6 UART recovery bridge; all three need LDO_VO4 at 3.3 V. */
    cfg.link.boot_gpio = 47;
    cfg.link.boot_gpio_ldo = 4;
    /* udp_ota on its default port (3232) updates this app and the C6. */
    cfg.ota_fs_dir = config_err == ESP_OK ? k_ota_fs_dir : nullptr;
    cfg.on_ota_active = on_ota_active;
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
        start_applets();
        run_apps_reactor();
        if (config_err == ESP_OK)
        {
            start_l3();
        }
        print_heap("before_video");
        start_video();
    }
    printf("ROVER_READY\n");
    fflush(stdout);

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        print_stats();
        print_heap("stats");
    }
}
