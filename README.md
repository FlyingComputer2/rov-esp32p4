# rover-remote-esp32p4

Main application of the rover's ESP32-P4 (Waveshare ESP32-P4-WIFI6-DEV-KIT, ESP32-C6 on SDIO as the radio). One image runs:

- **winject SDIO proxy** ([winject-radio-esp32-sdio-proxy](https://github.com/winject-wireless/winject-radio-esp32-sdio-proxy)): drives the C6 ([winject-radio-esp32c6](https://github.com/winject-wireless/winject-radio-esp32c6)) and exposes it as a winject radio on UDP 22 (m-plane) and 9000 (d-plane), plus HTTP firmware update.
- **winject-l3** ([winject-l3](https://github.com/winject-wireless/winject-l3), `idf/winject_l3`): winject-manager on a pthread, using that radio over loopback (`winject.device = 127.0.0.1`).
- **vstreamer** ([vstreamer](https://github.com/FlyingComputer2/vstreamer), `idf/vstreamer`): live TX (`vstreamer::idf::run_live_tx`): OV5647 MIPI-CSI camera → ISP + hardware H.264 (1080p30, 6 Mbit/s) → RTP → `stream_sender` with RS FEC 21/31, into winject-l3's video upstream on loopback.

```
 camera ─▶ vstreamer live TX ──loopback UDP 22081──▶ winject-l3 ──loopback UDP 22 / 9000──▶ winject_proxy ──SDIO──▶ C6 ─▶ air
 rover apps ──UDP (Ethernet)──┘
```

The GS side is [rover-ground](https://github.com/FlyingComputer2/rover-ground) (winject-l3 + winject-radio-realtek on an RTL8812AU, `sdl_stream_receiver` on `127.0.0.1:21082`); the two `winject.conf`s pair up by domain, channel and upstream ids.

All three come in with CMake FetchContent at pinned commits (top-level `CMakeLists.txt`). vstreamer and winject-l3 both fetch bfc and ISA-L under the same FetchContent names, and the first declaration wins, so bump their pins together. To build against local checkouts:

```bash
idf.py -DFETCHCONTENT_SOURCE_DIR_WINJECT_L3=~/development/winject-l3 \
       -DFETCHCONTENT_SOURCE_DIR_WINJECT_SDIO_PROXY=~/development/winject-radio-esp32-sdio-proxy \
       -DFETCHCONTENT_SOURCE_DIR_VSTREAMER=~/development/vstreamer build
```

## Build and flash

Native ESP-IDF v5.5 (`. ~/esp/esp-idf-v5.5.5/export.sh`); P4 USB-UART on the Type-C "UART" port:

```bash
idf.py set-target esp32p4 build && idf.py -p /dev/ttyACM0 flash
```

`idf.py flash` also writes the SPIFFS partition built from [`config/`](config), mounted at `/winject`; winject-l3 reads `/winject/winject.conf`. After the first flash, update the app over HTTP (`curl --data-binary @build/rover-remote-esp32p4.bin http://<P4 IP>/update/p4`); the config partition is not touched by that.

Console (921600 baud): `ROVER_IP ip=…` (DHCP, hostname `rover-remote`), `ROVER_READY`, `ROVER_PROXY_STATS …` every 10 s, `ROVER_L3_EXIT rc=…` if the manager stops, `ROVER_VIDEO_EXIT rc=…` if live TX stops (it restarts after 3 s); `live TX …` and `sys cpu …` lines from vstreamer every 10 s; winject-l3 logs its own stats every `winject.stats_sec`.

## Config

[`config/winject.conf`](config/winject.conf): radio on loopback (channel 13, OFDM_24M, 15 dBm, domain 0xB00B), manager console on UDP 2400, and the GS's three upstreams with buses swapped:

| id | Stream | Rover bind | Manager FEC |
|----|--------|------------|-------------|
| 0 | WireGuard (nothing feeds it on the P4 yet) | `127.0.0.1:22180` | RS 1/3 |
| 1 | Video (vstreamer live TX on the P4, or apps on the rover's Ethernet) | `127.0.0.1:22081` | none: vstreamer's own RS FEC runs end to end |
| 2 | Radio link-test probe | `127.0.0.1:22100` | none |

The video settings (bitrate, FEC 21/31, `fec_spread_ms` 75, `max_datagram` 1445 for winject-l3's 1445-byte limit) are in `main/main.cpp`; they can be changed at run time on the vstreamer m-plane, UDP 5090 (`get/set src|pay|snd <key>`, `get sys stats`). The manager console and the vstreamer m-plane have no authentication; they are bound to all interfaces because the rover's Ethernet is an internal link.

## Status

Video runs end to end over the air on the bench (P4 + C6 → GS RTL8812AU, a few metres): about 7% frame loss on the air at OFDM_24M / 15 dBm, and with FEC 21/31 at 75 ms spread 0–0.7% residual RTP loss at 22–27 fps at the GS receiver. Core 0 stays ~25–30% idle and internal RAM above ~56 KB. Measured along the way:
- At 2 dBm (the GS's bench power for the RTL8812AU) nothing from the C6 arrives; it is heard from about 5 dBm.
- OFDM_36M loses ~11% on the air, 24M and 18M ~7%.
- FEC 10/12 left 3.8% residual (3.6 fps); 15/31 runs core 0 out of time beside the proxy and winject-l3.

Open points:
- The ~7% air loss floor does not improve at lower rates, so it is likely not SNR: GS receiver gain or bursts, not investigated.
- TX pacing on the P4: winject-l3's reactor and TX thread wait in FreeRTOS ticks (1 ms), coarser than on Linux.
- Reverse telemetry (receiver → `stream_sender`, for adaptive FEC) is off.
