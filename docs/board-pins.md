# Waveshare ESP32-P4-WIFI6-DEV-KIT — GPIO allocation

This board runs the rover app ([`main/main.cpp`](../main/main.cpp)) and the
`rover_sample` RC car applet. **Steering** and **BTS7960 (IBT-2)** actuator
GPIOs are defined in [`main/board_pins.hpp`](../main/board_pins.hpp). At boot,
`main` calls `rover_board::actuator_pins()` and passes the result into
`rover_sample::config::pin` before the applet starts.

Source code is the authoritative map for actuator GPIOs; this document explains
wiring, LEDC usage, pins the firmware already owns elsewhere, and what to
avoid when adding hardware.

## Actuators (servos CH1–CH3 + BTS7960)

| Signal | ESP32-P4 GPIO | 40-pin header | Driver | Notes |
|--------|---------------|---------------|--------|-------|
| **CH1** steering PWM | **3** | **19** | LEDC timer **2**, channel **4**, 14-bit, default 50 Hz (`servo_hz`) | Steering servo; separate 5–6 V supply, common GND |
| **CH2** aux PWM | **2** | **21** | LEDC timer **2**, channel **0** | RC `aux` (‑1..1 → `servo_*_us`); `ch2_failsafe` without live RC |
| **CH3** aux PWM | **26** | **31** | LEDC timer **2**, channel **1** | RC `aux2`; `ch3_failsafe` without live RC |
| BTS RPWM | **4** | **18** | LEDC timer **3**, channel **5**, 10-bit, default 20 kHz (`bts_pwm_hz`) | Forward bridge PWM |
| BTS LPWM | **5** | **16** | LEDC timer **3**, channel **6** | Reverse / brake low-side PWM |
| BTS R_EN | **6** | **15** | LEDC timer **3**, channel **7** | **10 kΩ pulldown to GND** so a P4 reset **coasts** the motor. Deadman GPTimer forces EN low if the 250 Hz control loop stalls 50 ms |
| BTS L_EN | **21** | **11** | Same LEDC channel **7** (second pin) | Separate wire for L_EN, same signal; its own 10 kΩ pulldown. The deadman drives it low too. Set `bts_l_en = -1` to tie L_EN to R_EN again |
| BTS R_IS | — | — | ADC1 | **Not wired, disabled** (`GPIO_NUM_NC` in `board_pins.hpp`). Current sense via divider (`is_div`, `is_r_ohm`) when used; GPIO23 (header 7) is a free ADC1 pin |
| BTS L_IS | — | — | ADC1 | Same |

## VBUS voltage (divider → ADC)

| Signal | ESP32-P4 GPIO | 40-pin header | Driver | Notes |
|--------|---------------|---------------|--------|-------|
| VBUS sense | **53** | **35** | ADC2 channel 4 | Divider from **VBUS+** (pack / BEC): top resistor to VBUS+, tap to GPIO **53**, bottom resistor to GND. Scale with `vbus_div` (default **12**, e.g. 110 kΩ / 10 kΩ; the pin must stay under ~3.1 V). GPIO53 is also the board's speaker-amp **PA_CTRL** (0R R73 into a 74LVC1G08 input): high-Z, so the reading is fine, but the amp may sit enabled (silent) |

Firmware reads VBUS every control cycle and prints `vbus_v` on the periodic `ROVER_SAMPLE` status line and in `status` on UDP 5120. Calibrate `vbus_div` against a meter (`set vbus_div <ratio>` then `save`).

LEDC assignments are fixed in
[`components/rover_sample/src/drivers.cpp`](../components/rover_sample/src/drivers.cpp):
timer **2** = all servos (CH1 ch **4**, CH2 ch **0**, CH3 ch **1**); timer **3** =
BTS (ch **5–7**). Timer 0–1 and LEDC channels 2–3 are unused.

The BTS7960 driver is **not** brought up until
`set bts_enabled 1` (disarmed only). Until then, those GPIOs are not configured
as PWM outputs.

### Bench bring-up (steering endpoints)

Disarmed only (`servo_us` on UDP console 5120):

```bash
echo "servo_us ch1 1250" | nc -u -w1 <rover-ip> 5120   # sweep CH1 left/right/center (ch2/ch3 likewise)
echo "set servo_min_us 1250" | nc -u -w1 <rover-ip> 5120
echo save | nc -u -w1 <rover-ip> 5120
```

See [RC car (rover_sample)](../README.md#rc-car-rover_sample) in the README for
parameters and arming rules.

## Sensors (shared I2C)

| Signal | GPIO | Header | Devices |
|--------|------|--------|---------|
| I2C SDA | **7** | **3** | Board bus: camera SCCB 0x36, ES8311 0x18, GY-521 0x68, GY-271 0x0d/0x1e |
| I2C SCL | **8** | **5** | 2.2 kΩ pull-ups on the kit |

`main` creates this bus (`I2C_NUM_0`) before video starts; `rover_sample` only
uses the existing handle (`config::pin::i2c_port`).

## Reserved by this firmware (do not reuse for actuators)

| GPIO | Header (if routed) | Consumer |
|------|-------------------|----------|
| 14–17 | (SDIO D0–D3) | C6 radio SDIO data |
| 18, 19 | — | C6 SDIO **CLK / CMD** (not the same as header pins 18/19) |
| 20 | **13** | krono clock-sync pulse (open drain, active low) |
| 22 | 12 | Status LED (1 s period, 100 ms on) |
| 37, 38 | USB-UART path | P4 UART0 console |
| 46, 48 | C6 UART bridge | Recovery / SDIO co-processor (reserved in normal app) |
| 47 | C6 boot strap control | SDIO proxy `boot_gpio` |
| 0, 1 | 23, 22 | 32.768 kHz crystal (0R R29 / R33) |
| 24, 25 | 27, 28 | USB1.1 Type-C = USB-Serial-JTAG (also on the header through 0R R105 / R106) |
| 32, 33 | 26, 29 | C6 UART (flash bridge wiring) |
| 45 | 40 | TF card power switch (Q1 gate) |
| 51 | — | Ethernet PHY reset (IP101) |
| 54 | C6 `CHIP_PU` | SDIO proxy default |

## Power and ground on the 40-pin header

| Rail | Header pins (typical) |
|------|------------------------|
| 3V3 | 1, 17 |
| 5V | 2, 4 |
| GND | 6, 9, 14, 20, 25, 30, 34, 39 |

Tie servo and motor logic grounds to the P4 ground. High motor current should
not flow through the header signal ground alone; use a solid return to the
battery or supply negative.

## Changing pins

1. Edit constants in [`main/board_pins.hpp`](../main/board_pins.hpp).
2. Rebuild and flash the rover app.
3. Update this file and the wiring table in [`README.md`](../README.md#wiring-40-pin-header).
4. Keep actuator GPIOs off SDIO, krono GPIO20, the status LED, C6 control
   pins, and the board I2C (7/8) unless you also change those subsystems.

For a different carrier board, you can still override `rover_sample::pins` in
tests or a custom `main` without changing the applet defaults in
[`rover_sample.hpp`](../components/rover_sample/include/rover_sample/rover_sample.hpp).
