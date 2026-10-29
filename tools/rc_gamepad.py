#!/usr/bin/env python3
"""Bench RC for rover_sample through the ground P4's RC bridge (UDP 5120).

Sends MSP v2 RC frames (func 0x4201, src 2) at 50 Hz and prints the rover's
telemetry (func 0x4220) once a second. t_us is this host's CLOCK_REALTIME;
the opi5 is the ground's SNTP source and the rover follows the ground, so on
the opi5 rc_lat_ms is the real one-way RC latency.

Gamepad (python3-evdev): left stick X steers, right trigger throttle, left
trigger brake / reverse, Start arms, Select disarms. Without a gamepad,
--steer / --thr / --arm send constant values (e.g. to test the failsafe:
stop the script and watch the rover brake).

  tools/rc_gamepad.py --ground 192.168.253.25
  tools/rc_gamepad.py --ground 192.168.253.25 --steer 0.3 --thr 0 --arm --seconds 5
"""
import argparse
import select
import socket
import struct
import time

FUNC_RC = 0x4201
FUNC_TELEM = 0x4220
SRC_UDP = 2
STATES = ['disarmed', 'armed', 'failsafe', 'fault']
MODES = ['direct', 'rate', 'rate_hold']
TELEM = struct.Struct('<HIHHBBhhhhHhhhhHHHHB')


def crc8_dvb_s2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def msp_encode(direction, func, payload):
    body = struct.pack('<BHH', 0, func, len(payload)) + payload
    return b'$X' + direction + body + bytes([crc8_dvb_s2(body)])


def msp_decode(data):
    if len(data) < 9 or data[:2] != b'$X':
        return None
    flag, func, length = struct.unpack_from('<BHH', data, 3)
    if len(data) != 9 + length or crc8_dvb_s2(data[3:-1]) != data[-1]:
        return None
    return chr(data[2]), func, data[8:-1]


def realtime_us32():
    return (time.time_ns() // 1000) & 0xFFFFFFFF


def find_gamepad():
    try:
        import evdev
    except ImportError:
        return None
    for path in evdev.list_devices():
        dev = evdev.InputDevice(path)
        caps = dev.capabilities()
        if evdev.ecodes.EV_ABS in caps and evdev.ecodes.EV_KEY in caps:
            return dev
    return None


class Pad:
    def __init__(self, dev):
        import evdev
        self.e = evdev.ecodes
        self.dev = dev
        self.info = {code: info for code, info in dev.capabilities()[self.e.EV_ABS]}
        self.steer = self.gas = self.brake = 0.0
        self.arm = False

    def norm(self, code, value, centred):
        info = self.info.get(code)
        if not info:
            return 0.0
        span = info.max - info.min
        x = (value - info.min) / span if span else 0.0
        return x * 2 - 1 if centred else x

    def poll(self):
        while True:
            r, _, _ = select.select([self.dev.fd], [], [], 0)
            if not r:
                return
            for ev in self.dev.read():
                if ev.type == self.e.EV_ABS:
                    if ev.code == self.e.ABS_X:
                        self.steer = self.norm(ev.code, ev.value, True)
                    elif ev.code in (self.e.ABS_RZ, self.e.ABS_GAS):
                        self.gas = self.norm(ev.code, ev.value, False)
                    elif ev.code in (self.e.ABS_Z, self.e.ABS_BRAKE):
                        self.brake = self.norm(ev.code, ev.value, False)
                elif ev.type == self.e.EV_KEY and ev.value == 1:
                    if ev.code == self.e.BTN_START:
                        self.arm = True
                    elif ev.code == self.e.BTN_SELECT:
                        self.arm = False

    def values(self):
        steer = 0.0 if abs(self.steer) < 0.05 else self.steer
        return steer, self.gas - self.brake, self.arm


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--ground', default='192.168.253.25')
    p.add_argument('--port', type=int, default=5120)
    p.add_argument('--rate', type=float, default=50)
    p.add_argument('--mode', type=int, default=255, help='0 direct, 1 rate, 2 rate+hold, 255 rover default')
    p.add_argument('--steer', type=float, default=0.0)
    p.add_argument('--thr', type=float, default=0.0)
    p.add_argument('--arm', action='store_true')
    p.add_argument('--seconds', type=float, default=0, help='stop after this long (0 = until Ctrl-C)')
    p.add_argument('--no-pad', action='store_true')
    a = p.parse_args()

    pad = None if a.no_pad else find_gamepad()
    pad = Pad(pad) if pad else None
    print('gamepad:', pad.dev.name if pad else 'none (constant values)')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    dest = (a.ground, a.port)
    seq = 0
    period = 1.0 / a.rate
    start = next_send = next_print = time.monotonic()
    last_telem = None
    while not a.seconds or time.monotonic() - start < a.seconds:
        now = time.monotonic()
        if now >= next_send:
            next_send += period
            if pad:
                pad.poll()
                steer, thr, arm = pad.values()
            else:
                steer, thr, arm = a.steer, a.thr, a.arm
            steer = max(-1.0, min(1.0, steer))
            thr = max(-1.0, min(1.0, thr))
            payload = struct.pack('<HIBBBhhh', seq & 0xFFFF, realtime_us32(), SRC_UDP, 1 if arm else 0,
                                  a.mode, round(steer * 1000), round(thr * 1000), 0)
            sock.sendto(msp_encode(b'<', FUNC_RC, payload), dest)
            seq += 1
        try:
            while True:
                data, _ = sock.recvfrom(512)
                f = msp_decode(data)
                if f and f[1] == FUNC_TELEM and len(f[2]) == TELEM.size:
                    last_telem = TELEM.unpack(f[2])
        except BlockingIOError:
            pass
        if now >= next_print:
            next_print += 1.0
            if last_telem:
                (_, _, rc_seq, rc_age, st, mode, r_cmd, r_meas, steer_o, thr_o, hdg, roll, pitch, ax, ay,
                 v_est, i_m, loss, lat, flags) = last_telem
                print(f'{STATES[st] if st < 4 else st} {MODES[mode] if mode < 3 else mode} '
                      f'r_cmd={r_cmd / 10:.1f} r_meas={r_meas / 10:.1f} steer={steer_o / 1000:+.3f} '
                      f'thr={thr_o / 1000:+.3f} v={v_est / 100:.2f} i={i_m / 100:.2f}A hdg={hdg / 100:.1f} '
                      f'roll={roll / 100:.1f} pitch={pitch / 100:.1f} rc_age={rc_age} rc_loss={loss / 10:.1f}% '
                      f'rc_lat={lat}ms flags=0x{flags:02x}')
            else:
                print('no telemetry yet')
        time.sleep(max(0.0, min(next_send, next_print) - time.monotonic()))


if __name__ == '__main__':
    main()
