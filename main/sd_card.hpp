// TF (microSD) card on the Waveshare ESP32-P4-WIFI6-DEV-KIT, FAT at /sdcard.
//
// SDMMC slot 0 on its IO MUX pins (CLK 43, CMD 44, D0-D3 39-42), 4-bit,
// high speed (40 MHz). The C6 radio is on slot 1 of the same SDMMC host: the
// host driver is shared (sdmmc_host_init() skips when it is already up, and an
// unmount deinits only slot 0), and its transaction lock serialises the two
// slots, so a long card write delays C6 SDIO transfers by that long.
//
// Power: the slot's IO is on LDO_VO4 at 3.3 V, the same channel the SDIO proxy
// holds for the C6 boot strap (shared, not adjustable, so no UHS-I 1.8 V).
// The card's VDD goes through the Q1 switch on GPIO45, driven to its on level.
#pragma once

#include <cstdint>
#include <string>

#include "esp_err.h"

namespace rover_sd
{

constexpr const char* k_mount_point = "/sdcard";

/**
 * Powers the card and mounts it at /sdcard (3 attempts, the card power-cycled
 * in between with its lines held low; the retries at 20 MHz). Never formats: a
 * card without FAT (exFAT is not built in) stays unmounted until
 * format_card(). Writes nothing. Registers a shutdown handler that unmounts
 * and powers the card down before a software restart. Call after
 * winject_proxy_start(), which brings up the SDMMC host. Prints ROVER_SD ...
 * or ROVER_SD_FAILED ....
 */
esp_err_t mount();

/**
 * Starts the 1 MB write / read-back test (/sdcard/sys/sdtest.tmp, removed) on
 * a thread of its own; `get sys sd` shows test=, write_kbps= and read_kbps=
 * after. 0, -ENODEV (not mounted) or -EALREADY.
 */
int run_test();

/**
 * Starts formatting the card FAT32 (one partition, 64 KB clusters) on a
 * thread of its own: everything on it is lost. A mounted card is formatted in
 * place; an unmounted one (no FAT) is mounted with formatting allowed. The
 * caller makes sure nothing has files open on /sdcard. 0, or -EALREADY while a
 * format runs. format_line() reports it.
 */
int format_card();

/** `get sys sd_format`: "idle", "running", "ok ms=<n>" or "failed err=<name>". */
std::string format_line();

/**
 * One line for `get sys sd`: "mounted=1 name=... card_mb=... fs_mb=...
 * free_mb=... freq_khz=... width=... test=ok write_kbps=... read_kbps=...
 * attempts=...", "mounted=0 stage=power|mount err=<esp_err name> attempts=...",
 * "formatting" while a format runs or "testing" while run_test() does. test=
 * is "none" until a test ran.
 */
std::string status_line();

bool mounted();

/** Total and free bytes of the mounted volume. */
esp_err_t usage(uint64_t* total_bytes, uint64_t* free_bytes);

}  // namespace rover_sd
