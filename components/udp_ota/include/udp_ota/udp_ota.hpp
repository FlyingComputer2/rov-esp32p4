// udp_ota: P4 app update over UDP (wire: core/wire.hpp, tool: tools/udp_ota.py).
// The server binds one UDP port and answers each request's sender, so it is
// reachable over Ethernet and, through an L3 UDP_STATIC upstream, over the
// air. The image goes into the inactive OTA slot; reboot_request boots it.
// ota_fs_* requests read and write one file at a time in fs_dir.
#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "esp_err.h"

namespace udp_ota
{

struct config
{
    uint16_t port = 3232;
    // Directory for ota_fs_* (the mounted config partition); empty = none.
    std::string fs_dir = "/winject";
    // An OTA, or an open file, with no request from its client for this long
    // is aborted / closed.
    uint32_t idle_timeout_ms = 30000;
    // true when an OTA starts, false when it completes, fails or times out
    // (e.g. to pause video). Runs on the server task.
    std::function<void(bool)> on_active;
    int task_prio = 5;
    int task_core = -1;  // -1 = unpinned
    uint32_t task_stack = 6 * 1024;
};

// Starts the server task. Call once, after the network stack is up.
esp_err_t start(const config& cfg);

}  // namespace udp_ota
