# rover-remote-esp32p4

Main application of the rover's ESP32-P4 (Waveshare ESP32-P4-WIFI6-DEV-KIT, ESP32-C6 on SDIO as the radio). One image runs:

- **winject SDIO proxy** ([winject-radio-esp32-sdio-proxy](https://github.com/winject-wireless/winject-radio-esp32-sdio-proxy)): drives the C6 ([winject-radio-esp32c6](https://github.com/winject-wireless/winject-radio-esp32c6)) and exposes it as a winject radio: m-plane on UDP 22, d-plane on in-process queues, plus HTTP firmware update.
- **winject-l3** ([winject-l3](https://github.com/winject-wireless/winject-l3), `idf/winject_l3`): winject-manager on a pthread, using that radio (`radio.mplane = 127.0.0.1:22`, `radio.dplane_tx/_rx` queues).
- **vstreamer** ([vstreamer](https://github.com/FlyingComputer2/vstreamer), `idf/vstreamer`): live TX (`vstreamer::idf::run_live_tx`): OV5647 MIPI-CSI camera → ISP + hardware H.264 (1080p30, 6 Mbit/s) → RTP → `stream_sender` with RS FEC 21/31, into winject-l3's video upstream over queues.

```
 camera ─▶ vstreamer live TX ──video_tx / video_rx──▶ winject-l3 ──wifi_tx / wifi_rx──▶ winject_proxy ──SDIO──▶ C6 ─▶ air
                                                       └──── loopback UDP 22 (m-plane) ────┘
```

`main/main.cpp` registers the four queues (`bfc::cv_reactor_circular_lossy_queue_registry`) and one `bfc::cv_reactor` (`rover_dp`, its own thread) before starting the three, and keeps them for the life of the program. The proxy, winject-l3 (`radio.dplane_tx` / `radio.dplane_rx` / `radio.reactor`, upstream 1 `QUEUE_FORWARDING`) and vstreamer (`live_tx_config.queue_*`) bind them by name. A full queue drops its oldest datagram: the proxy counts it as `dup_evict`, winject-l3 in its metrics, vstreamer in `snd` stats.

The GS side is [rover-ground-esp32p4](https://github.com/FlyingComputer2/rover-ground-esp32p4): winject-l3, the SDIO proxy and the video receiver on a Guition P4 + C6. Its L3 config is built in (`l3_main()` in its `main/main.cpp`) and pairs with [`config/winject.conf`](config/winject.conf) by domain, channel and upstream id, with buses swapped. It carries only upstream 1 (video). The earlier GS, [rover-ground](https://github.com/FlyingComputer2/rover-ground) (winject-l3 + RTL8812AU on the opi5), is archived.

All three come in with CMake FetchContent at pinned commits (top-level `CMakeLists.txt`). All three fetch bfc under one FetchContent name and must use the same bfc (they share its registries); the top-level `CMakeLists.txt` declares it first, so it wins, and its pin should stay equal to theirs. vstreamer and winject-l3 also share ISA-L, so bump their pins together. To build against local checkouts:

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

[`config/winject.conf`](config/winject.conf): radio m-plane on loopback and d-plane on queues (channel 1, OFDM_24M, 15 dBm, domain 0xB00B), manager console on UDP 2400, and three upstreams. The ground P4 pairs only upstream 1 (buses swapped); 0 and 2 have no peer there yet:

| id | Stream | Rover endpoint | Manager FEC |
|----|--------|----------------|-------------|
| 0 | WireGuard (nothing feeds it on the P4 yet) | UDP `127.0.0.1:22180` | RS 1/3 |
| 1 | Video (vstreamer live TX on the P4 only) | queues `video_tx` (in) / `video_rx` (out) | none: vstreamer's own RS FEC runs end to end |
| 2 | Radio link-test probe | UDP `127.0.0.1:22100` | none |

The video settings (bitrate, FEC 21/31, `fec_spread_ms` 75; the payloader's 1400-byte `mtu` keeps datagrams under winject-l3's 1445-byte limit) are in `main/main.cpp`; they can be changed at run time on the vstreamer m-plane, UDP 5090 (`get/set src|pay|snd <key>`, `get sys stats`). The manager console and the vstreamer m-plane have no authentication; they are bound to all interfaces because the rover's Ethernet is an internal link.

## Status

Video runs end to end over the air on the bench (P4 + C6 → GS RTL8812AU, a few metres). Core 0 stays ~25–30% idle and internal RAM above ~56 KB. Measured along the way:
- At 2 dBm (the GS's bench power for the RTL8812AU) nothing from the C6 arrives; it is heard from about 5 dBm.
- Air loss depends on the channel, not on rate, power above ~10 dBm, or the GS receiver gain (RX IGI pinned at 0x3c; 0x30–0x54 all the same): losses are single frames, and each receiver has its own bad channels. C6 → GS: ~0.3–1.9% on channels 1–4, 6.5–9% on 5–13; GS → C6: ~5% on 3–4, ≤0.7% on 1, 2 and 13. Channel 1 is clean both ways (~1%).
- On channel 13 (the earlier pairing with the GS), FEC 21/31 at 75 ms spread gives 0–0.7% residual RTP loss at 22–27 fps; 10/12 left 3.8% (3.6 fps); 15/31 runs core 0 out of time beside the proxy and winject-l3. On channel 1 every FEC tried (10/12 to 21/31) was clean at 27–28 fps.

Open points:
- Channel 1 needs the GS on channel 1 too (the ground P4 is). The cause of the per-channel loss (local in-band interference or desense at each receiver) is not known; channel 1 may not be the clean one elsewhere.
- TX pacing on the P4: winject-l3's reactor and TX thread wait in FreeRTOS ticks (1 ms), coarser than on Linux.
- Reverse telemetry (receiver → `stream_sender`, for adaptive FEC) is off.
