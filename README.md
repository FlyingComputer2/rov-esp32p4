# rov-esp32p4

Main application of the rover's ESP32-P4 (Waveshare ESP32-P4-WIFI6-DEV-KIT, ESP32-C6 on SDIO as the radio). One image runs:

- **winject SDIO proxy** ([winject-radio-esp32-sdio-proxy](https://github.com/winject-wireless/winject-radio-esp32-sdio-proxy)): drives the C6 ([winject-radio-esp32c6](https://github.com/winject-wireless/winject-radio-esp32c6)) and exposes it as a winject radio: m-plane on UDP 22, d-plane on in-process queues, plus HTTP firmware update.
- **winject-l3** ([winject-l3](https://github.com/winject-wireless/winject-l3), `idf/winject_l3`): winject-manager on a pthread, using that radio (`radio.mplane = 127.0.0.1:22`, `radio.dplane_tx/_rx` queues).
- **vstreamer** ([vstreamer](https://github.com/FlyingComputer2/vstreamer), `idf/vstreamer`): live TX (`vstreamer::idf::run_live_tx`): OV5647 MIPI-CSI camera → ISP + hardware H.264 (1080p30, 6 Mbit/s) → RTP → `stream_sender` with RS FEC 21/31, into winject-l3's video upstream over queues.
- **krono** ([FlyingComputer2/krono](https://github.com/FlyingComputer2/krono), `idf/krono`): clock-sync client on upstream 0, an L3-owned queue; the ground P4 is the server and holds the reference clock. UDP console 5110 (see [Clock sync](#clock-sync)).
- **app_bwtest** (winject-l3, `idf/app_bwtest`): bandwidth tester on upstream 2, an L3-owned queue; UDP console 5100 (see [Bandwidth test](#bandwidth-test)).
- **rover_sample** (`components/rover_sample`): RC car (steering servo, BTS7960 motor bridge, GY-521 IMU, GY-271 compass) with rate-mode steering; RC in and telemetry out over upstream 3, an L3-owned queue; UDP console 5120 (see [RC car](#rc-car-rover_sample)).

```
 camera ─▶ vstreamer live TX ──video_tx / video_rx──▶ winject-l3 ──wifi_tx / wifi_rx──▶ winject_proxy ──SDIO──▶ C6 ─▶ air
                                                       └──── loopback UDP 22 (m-plane) ────┘
```

`main/main.cpp` registers the four queues (`bfc::cv_reactor_circular_lossy_queue_registry`) and one `bfc::cv_reactor` (`rover_dp`, its own thread) before starting the three, and keeps them for the life of the program. The proxy, winject-l3 (`radio.dplane_tx` / `radio.dplane_rx` / `radio.reactor`, upstream 1 `QUEUE_FORWARDING`) and vstreamer (`live_tx_config.queue_*`) bind them by name. A full queue drops its oldest datagram: the proxy counts it as `dup_evict`, winject-l3 in its metrics, vstreamer in `snd` stats.

The GS side is [gnd-esp32p4](https://github.com/FlyingComputer2/gnd-esp32p4): winject-l3, the SDIO proxy and the video receiver on a Guition P4 + C6. Its L3 config is built in (`l3_main()` in its `main/main.cpp`) and pairs with [`config/winject.conf`](config/winject.conf) by domain, channel and upstream id, with buses swapped. The earlier GS, [gnd](https://github.com/FlyingComputer2/gnd) (winject-l3 + RTL8812AU on the opi5), is archived.

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

`idf.py flash` also writes the SPIFFS partition built from [`config/`](config), mounted at `/winject`; winject-l3 reads `/winject/winject.conf`. After the first flash, update the app over HTTP (`curl --data-binary @build/rov-esp32p4.bin http://<P4 IP>/update/p4`); the config partition is not touched by that.

### C6 (Waveshare ESP32-P4-WIFI6-DEV-KIT)

| C6 | P4 | Notes |
|---|---|---|
| `U0RXD` / `U0TXD` | GPIO46 / GPIO48 | P4 UART1 TX / RX in the Waveshare recovery bridge; reserved in the normal SDIO application. GPIO37/38 remain the P4's UART0, shared with the board's USB-UART |
| `GPIO9` (boot strap) | GPIO47 | `main.cpp` sets `boot_gpio = 47`, `boot_gpio_ldo = 4` to power GPIO39–48 at 3.3 V |
| `CHIP_PU` | GPIO54 | Proxy default |
| SDIO | slot 1: CLK 18, CMD 19, D0–D3 14–17 | Proxy default |

Update the C6 over SDIO: `curl --data-binary @winject-esp32c6.bin http://<P4 IP>/update/c6`. For a UART flash, put gnd-esp32p4's [`tools/c6-uart-bridge`](https://github.com/FlyingComputer2/gnd-esp32p4/tree/main/tools/c6-uart-bridge) on the P4 with the Waveshare settings. It holds the C6 in its ROM downloader and bridges `/dev/ttyACM0` to the C6 at 115200:

```bash
cd gnd-esp32p4/tools/c6-uart-bridge
idf.py -B build-waveshare -D SDKCONFIG=build-waveshare/sdkconfig \
  -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.defaults.waveshare" -p /dev/ttyACM0 flash
python -m esptool --chip esp32c6 -p /dev/ttyACM0 -b 115200 --before no_reset --after no_reset flash_id
# write_flash as in winject-radio-esp32c6, then reflash this app with `idf.py flash`
# (the bridge replaces the bootloader and partition table)
```

Opening `/dev/ttyACM0` at 115200 resets the P4.

Console (921600 baud): `ROVER_IP ip=…` (DHCP, hostname `rover-remote`), `ROVER_READY`, `ROVER_PROXY_STATS …` every 10 s, `ROVER_L3_EXIT rc=…` if the manager stops, `ROVER_VIDEO_EXIT rc=…` if live TX stops (it restarts after 3 s); `live TX paused` / `live TX resumed` on `video_stop` / `video_start`; `live TX …` and `sys cpu …` lines from vstreamer every 10 s; winject-l3 logs its own stats every `winject.stats_sec`.

## Config

[`config/winject.conf`](config/winject.conf): radio m-plane on loopback and d-plane on queues (channel 1, OFDM_24M, 15 dBm, domain 0xB00B), manager console on UDP 2400, and four upstreams. The ground P4 carries the same four, buses swapped. Buses belong to the flow, not the index:

| id | Stream | Rover endpoint | Buses (tx / rx) | Manager FEC |
|----|--------|----------------|-----------------|-------------|
| 0 | Clock sync (krono SynchClient) | L3-owned queue, handed to the client by `main/main.cpp` | b2 / a1 | none |
| 1 | Video (vstreamer live TX) | queues `video_tx` (in) / `video_rx` (out) | c3 / d4 | none: vstreamer's own RS FEC runs end to end |
| 2 | Bandwidth tester (app_bwtest) | L3-owned queue, handed to the tester by `main/main.cpp` | f6 / e5 | none |
| 3 | RC car (rover_sample): RC in, telemetry out | L3-owned queue, handed to rover_sample by `main/main.cpp` | 17 / 28 | none: RC repeats every 20 ms |

Only the ids moved: video and the tester kept their buses, so they still pair with an older image on the other board. Sync needs both boards on this layout (an older image counts its frames as `rx_drop_bus`).

The video settings (bitrate, FEC 21/31, `fec_spread_ms` 75; the payloader's 1400-byte `mtu` keeps datagrams under winject-l3's 1445-byte limit) are in `main/main.cpp`; they can be changed at run time on the vstreamer m-plane, UDP 5090 (`get/set src|pay|snd <key>`, `get sys stats`). The manager console and the vstreamer m-plane have no authentication; they are bound to all interfaces because the rover's Ethernet is an internal link.

## Clock sync

krono synch runs on upstream 0. The ground P4 runs `SynchServer`, and its `CLOCK_REALTIME` is the reference. The rover runs `SynchClient`: once a second it measures its offset and steers its own `CLOCK_REALTIME`. It steps on the first sample or on an error over 20 ms, and slews with `adjtime` below that. Protocol and wire: krono [docs/synch.md](https://github.com/FlyingComputer2/krono/blob/main/docs/synch.md).

- **rtt** (default): `sync_request{trid}` / `sync_response{trid, server_time_us}`. The client applies the lowest-delay sample of the last 8, so its accuracy is bounded by queueing asymmetry on the air path (milliseconds under video).
- **interrupt**: the rover pulls the sync wire low, then sends `sync_interrupt_request{trid}`. The ground replies with the time its ISR stamped the falling edge, so the air delay drops out.

- **monitor**: steers by rtt and grades it. Each second the wire first reads the rover's clock error, without correcting it, then rtt corrects. `status` adds `wire_last_us`, `wire_mean_us`, `wire_rms_us`, `wire_min_us`, `wire_max_us` and `wire_samples` (rover minus ground, since `mode monitor` or `clr`). This is the accuracy figure for rtt under the current load.

Sync wire: active low, Hi-Z at rest. The rover's pin is open drain and released between pulses. The ground's pin is an input with the internal pull-up and is the wire's only pull-up.

| Signal | Rover (Waveshare 40-pin header) | Ground (Guition JP1) |
|--------|---------------------------------|----------------------|
| SYNC (active low) | GPIO20, pin 13 | GPIO20, pin 17 |
| GND | pin 14 | pin 6 |

Console on UDP 5110 (both boards), one command per datagram:

| Command | Effect |
|---------|--------|
| `status` | Client: `mode`, `synced`, `error_us` (rover clock minus ground clock at the last correction), `offset_us` / `delay_us` of the applied sample, `requests`, `replies`, `timeouts`, `stale`, `steps`, `slews`. Server: `time_us`, `requests`, `interrupt_requests`, `replies`, `no_edge`, `bad` |
| `mode <rtt\|interrupt\|monitor>` | Client only: switch the measurement |
| `clr` | Client only: zero the monitor statistics |

```bash
echo status | nc -u -w1 192.168.253.28 5110
echo "mode interrupt" | nc -u -w1 192.168.253.28 5110
```

## Bandwidth test

app_bwtest runs on both P4s on upstream 2. Its console is UDP 5100, one command per datagram, reply to the sender:

| Command | Effect |
|---------|--------|
| `test_send <offer_kbps> <duration_ms>` | Sends 1400-byte datagrams paced at `offer_kbps` for `duration_ms`, in the background (`error busy` while one runs) |
| `stats` | `snd_pkt`, `snd_byt` (offered), `snd_drop` (offered but never taken by L3: rejected, or evicted from the full queue), `rcv_pkt`, `rcv_byt`; latency: `lat_samples`, `lat_invalid`, `lat_last_us`, `lat_min_us`, `lat_avg_us`, `lat_max_us` |
| `clr` | Zeroes counters and latency statistics |
| `video_stop` / `video_start` | Rover only: pauses vstreamer live TX (nothing sent; camera and encoder keep running) / resumes it from a fresh IDR |

```bash
echo "test_send 2000 10000" | nc -u -w1 192.168.253.28 5100   # rover -> ground, 2 Mbit/s for 10 s
echo stats | nc -u -w1 192.168.253.25 5100                    # what the ground received
```

Each datagram includes a send timestamp. Receiver `stats` reports last, minimum,
average and maximum one-way latency in microseconds since `clr`. This measures
sender queue submission to receiver polling, including L3/FEC/radio queueing and
the receiver's 10 ms idle polling interval plus scheduler delays. It requires
synchronized realtime clocks (rover follows ground through krono). Legacy,
malformed, zero or future timestamps count in `lat_invalid` instead of latency
samples. Check `lat_samples` before interpreting zero-valued latency fields.
See [BW test details](https://github.com/winject-wireless/winject-l3/blob/main/docs/bwtest.md).

Air loss is the receiving L3's `lur` `rxgap` on upstream 2; `rx_push_drop` there counts datagrams the tester did not drain in time.

## RC car (rover_sample)

`components/rover_sample`: a 250 Hz control task (core 1, priority 16) for a car with a steering servo, a BTS7960 (IBT-2) motor bridge, a GY-521 (MPU-6050) and a GY-271 (QMC5883L or HMC5883L). `core/` is portable and host-tested (`tests/`); `src/` holds the IDF drivers and the app.

### Wiring (40-pin header)

| Signal | GPIO | Header pin | Notes |
|--------|------|------------|-------|
| I2C SDA / SCL | 7 / 8 | 3 / 5 | The board I2C (`ESP_I2C_SDA` / `_SCL`), shared with the camera SCCB (0x36) and the ES8311 (0x18); 2.2 k pull-ups on the board. GY-521 at 0x68, GY-271 at 0x0d (QMC) or 0x1e (HMC) |
| Servo PWM | 3 | 19 | Separate 5–6 V supply for the servo, common ground |
| BTS7960 RPWM / LPWM | 4 / 5 | 18 / 16 | 20 kHz |
| BTS7960 R_EN + L_EN | 6 | 15 | Tied together; 10 k pull-down to GND so a P4 reset coasts |
| BTS7960 R_IS / L_IS | 21 / 23 | 11 / 7 | ADC1, through a divider (IS reaches ~5 V); ratio in `is_div` |
| 3V3 / 5V / GND | | 1, 17 / 2, 4 / 6, 9, 14, 20, 25, 30, 34, 39 | |

`main/main.cpp` creates the board I2C bus before video starts; the camera source uses that bus rather than creating its own. The BTS7960 stays unconfigured (pins untouched) until `set bts_enabled 1`. Mount the GY-521 flat with its x arrow forward and the GY-271 in the same orientation: the body frame is x forward, y left, z up, and a right turn is a positive yaw rate (`imu_yaw_sign` flips it).

### Control

- **Steering (rate mode)**: the stick commands a yaw rate up to `rate_max_dps`, capped by grip (`a_lat_max / v`). The servo command is a bicycle-model feedforward (`atan(wheelbase · r / v)` over `steer_max_deg`, scaled by `kff`) plus PI on the gyro's yaw rate (`kp`, `ki`). Below `blend_v0` the output is plain stick steering, blending to the rate loop at `blend_v1`, since yaw rate cannot build at low speed; reverse always steers direct. The integrator freezes while blending or saturated. Modes: 0 direct, 1 rate, 2 rate with heading hold once the stick has been centred for 200 ms (`hold_kp`). Without the IMU or its calibration, steering is direct and telemetry flags it.
- **Speed**: there is no wheel sensor; `v_est` is a first-order model of the applied duty (`top_speed_mps`, `tau_v_s`).
- **Throttle**: ESC-style. Forward up to `thr_max_fwd`; stick back while moving brakes (both low sides on, EN PWM = `brake_pct`); neutral then back again below 0.3 m/s reverses (up to `thr_max_rev`); `rev_mode 1` reverses directly. Slew `thr_slew_ms` from 0 to full, 100 ms off between directions, duty scaled back above `i_max_a` (from IS).
- **Arming and failsafe**: boots disarmed; arms on the RC arm flag with the throttle at neutral and the gyro calibrated (2 s still at boot). No RC for `failsafe_ms` (300): brake 1 s, then coast, steering centred; back to armed only with the throttle at neutral. A deadman GPTimer cuts EN in its ISR if the control loop stalls for 50 ms; that and an IS overload are a fault, cleared by disarming.
- **Attitude**: Mahony (gyro + accel) for roll / pitch; heading integrates the yaw rate, pulled to the tilt-compensated magnetometer (5 s). The magnetometer is telemetry only: motor current disturbs it.

### RC and telemetry (upstream 3)

MSP v2, one frame per datagram: `'$' 'X' dir flag:u8 func:u16 len:u16 payload crc8`, little-endian, crc8 DVB-S2 over flag..payload; dir `<` to the rover, `>` from it, `!` error. The ground P4 bridges the frames unchanged to USB channel 2 (`msp-rc`, for the Android app) and UDP 5120 (bench tools, e.g. [`tools/rc_gamepad.py`](tools/rc_gamepad.py)).

| func | dir | Rate | Payload |
|------|-----|------|---------|
| 0x4201 RC | < | 50 Hz | `seq:u16 t_us:u32 src:u8 flags:u8 mode:u8 steer:i16 thr:i16 aux:i16` (15 B). steer / thr -1000..1000 (+ = right / forward); flags bit 0 = arm; mode 255 = rover default; src 1 phone, 2 UDP, 3 console; t_us = sender `CLOCK_REALTIME` µs, low 32 bits |
| 0x4220 TELEM | > | `telem_hz` (20) | `seq:u16 t_us:u32 rc_seq:u16 rc_age_ms:u16 state:u8 mode:u8 r_cmd:i16 r_meas:i16 steer_out:i16 thr_out:i16 heading:u16 roll:i16 pitch:i16 ax:i16 ay:i16 v_est:u16 i_motor:u16 rc_loss:u16 rc_lat_ms:u16 flags:u8` (39 B). Rates 0.1 dps, outputs ‰, angles 0.01°, accel mg, v cm/s, current 10 mA, loss ‰ and latency p50 over the last second. state 0 disarmed, 1 armed, 2 failsafe, 3 fault; flags: 0x01 imu ok, 0x02 mag ok, 0x04 gyro calibrated, 0x08 BTS enabled, 0x10 deadman, 0x20 IMU lost (direct), 0x40 braking |
| 0x4210 GET_PARAM | < / > | | `id:u8` → `id:u8 value:f32` |
| 0x4211 SET_PARAM | < / > | | `id:u8 value:f32` → the applied (clamped) value |
| 0x4212 SAVE_PARAMS | < / > | | → `ok:u8` |
| 0x4213 CMD | < / > | | `cmd:u8` (1 gyro_cal, 2 mag_cal start, 3 mag_cal stop) → `cmd:u8 rc:i8` |

Every RC frame carries the whole state, so a lost one is replaced 20 ms later; there is no FEC on upstream 3. The rover follows one source and switches only after it has been silent for `failsafe_ms`. Clocks are krono-synced (rover → ground; the phone over USB channel 3), so `rc_lat_ms` is the real one-way RC latency.

### Console (UDP 5120 on the rover)

One command per datagram, reply to the sender. `set` applies from the next control cycle; `save` writes NVS, which is loaded at boot.

| Command | Effect |
|---------|--------|
| `status` | State, mode, RC source / age / loss / latency, IMU / magnetometer, outputs, `v_est`, current, attitude, I2C errors, loop overruns and worst loop time |
| `list` | Every parameter with unit and range |
| `get <name>` / `set <name> <value>` | One parameter (clamped; the reply shows the applied value) |
| `save` / `load` / `defaults` | Write NVS / reload it / compiled defaults (not saved) |
| `gyro_cal` | Disarmed only; keep the car still 2 s |
| `mag_cal start` / `mag_cal stop` | Hard-iron calibration: turn the car through every heading in between; `stop` sets `mag_off_*` (then `save`) |
| `rc <steer> <thr> <arm> [hold_ms]` | Bench RC (-1..1), repeated at 50 Hz for `hold_ms` (max 10 s) |
| `servo_us <us>` | Disarmed only: raw servo pulse, for finding the endpoints; `servo_us 0` or arming ends it |

Parameters (`list` prints them; defaults are placeholders until the car is measured):

| Group | Parameters |
|-------|------------|
| Servo | `servo_min_us` 1100, `servo_center_us` 1500, `servo_max_us` 1900, `servo_reverse` 0, `servo_hz` 50 (disarmed only) |
| Geometry | `steer_max_deg` 30 (wheel angle at full throw), `wheelbase_m` 0.26 |
| Steering | `mode` 1, `rate_max_dps` 360, `a_lat_max` 8, `expo` 0.3, `kp` 0.004, `ki` 0.02, `kff` 1, `blend_v0` 0.5, `blend_v1` 1.5, `hold_kp` 2, `gyro_lpf_hz` 42, `imu_yaw_sign` -1 |
| Motor | `bts_enabled` 0, `bts_pwm_hz` 20000 (both disarmed only), `thr_max_fwd` 60, `thr_max_rev` 40, `thr_slew_ms` 300, `thr_deadband` 3, `rev_mode` 0, `brake_pct` 100, `i_max_a` 20, `top_speed_mps` 8, `tau_v_s` 0.8, `is_div` 2, `is_r_ohm` 1000 |
| Link | `failsafe_ms` 300, `telem_hz` 20 |
| Compass | `mag_off_x` / `_y` / `_z` (from `mag_cal`) |

```bash
echo status | nc -u -w1 192.168.253.28 5120
echo "servo_us 1250" | nc -u -w1 192.168.253.28 5120      # find full left
echo "set servo_min_us 1250" | nc -u -w1 192.168.253.28 5120
echo save | nc -u -w1 192.168.253.28 5120
tools/rc_gamepad.py --ground 192.168.253.25                # drive through the ground P4
```

Console: `ROVER_SAMPLE uptime_s=… state=…` (the `status` text) every 10 s; `rover_sample: imu …, mag …` at start.

Memory: live TX needs the internal RAM. With rover_sample's stacks there (20 KB left) it fell to ~20 fps, so all of rover_sample's stacks are in PSRAM (live TX 29 fps, ~35 KB internal left). NVS access, which writes flash and so needs an internal stack, runs on a short-lived task with its stack in RTC RAM. A `save` stalls the control loop for a few ms (flash write).

Host tests: `cmake -S tests -B build-tests && cmake --build build-tests && build-tests/rover_sample_test`.

## Status

Video runs end to end over the air on the bench (P4 + C6 → GS RTL8812AU, a few metres). Core 0 stays ~25–30% idle and internal RAM above ~56 KB. Measured along the way:
- At 2 dBm (the GS's bench power for the RTL8812AU) nothing from the C6 arrives; it is heard from about 5 dBm.
- Air loss depends on the channel, not on rate, power above ~10 dBm, or the GS receiver gain (RX IGI pinned at 0x3c; 0x30–0x54 all the same): losses are single frames, and each receiver has its own bad channels. C6 → GS: ~0.3–1.9% on channels 1–4, 6.5–9% on 5–13; GS → C6: ~5% on 3–4, ≤0.7% on 1, 2 and 13. Channel 1 is clean both ways (~1%).
- On channel 13 (the earlier pairing with the GS), FEC 21/31 at 75 ms spread gives 0–0.7% residual RTP loss at 22–27 fps; 10/12 left 3.8% (3.6 fps); 15/31 runs core 0 out of time beside the proxy and winject-l3. On channel 1 every FEC tried (10/12 to 21/31) was clean at 27–28 fps.

Open points:
- Channel 1 needs the GS on channel 1 too (the ground P4 is). The cause of the per-channel loss (local in-band interference or desense at each receiver) is not known; channel 1 may not be the clean one elsewhere.
- TX pacing on the P4: winject-l3's reactor and TX thread wait in FreeRTOS ticks (1 ms), coarser than on Linux.
- Reverse telemetry (receiver → `stream_sender`, for adaptive FEC) is off.
