#!/usr/bin/env python3
"""Update a P4 app over udp_ota (components/udp_ota/core/wire.hpp).

  udp_ota.py <host:port> status
  udp_ota.py <host:port> reboot
  udp_ota.py <host:port> flash <app.bin> [--reboot] [--window N]

Rover over Ethernet: <rover IP>:3232. Rover over the air: <ground IP>:2430
(ground L3 upstream 9 -> air -> rover upstream 9 -> 127.0.0.1:3232).
flash sends the image as 1024-byte fragments with up to --window in flight
and retransmits unanswered ones; the reply to the fragment that completes the
image says whether the P4 accepted it. With --reboot it then boots the image
and waits until the P4 answers status with the new image's ELF SHA.
Python stdlib only.
"""

import argparse
import os
import socket
import struct
import sys
import time

FRAGMENT = 1024
START, FRAGMENT_REQ, REBOOT, STATUS = 0x01, 0x02, 0x03, 0x04
RESP = 0x80
STATUS_NAMES = ["OK", "EINPROGRESS", "EINVAL", "ENOENT", "EIO", "EBADMSG", "EFBIG"]


def status_name(code):
    return STATUS_NAMES[code] if code < len(STATUS_NAMES) else "E%d" % code


class OtaError(Exception):
    pass


def parse_addr(s):
    host, _, port = s.rpartition(":")
    if not host or not port.isdigit():
        raise SystemExit("address must be host:port, got %r" % s)
    return (socket.gethostbyname(host), int(port))


def app_elf_sha(image):
    """First 8 bytes of app_elf_sha256 from the image's esp_app_desc_t, as
    esp_app_get_elf_sha256(buf, 17) prints it; None if not an app image."""
    off = 24 + 8  # image header + first segment header
    if len(image) < off + 176 or struct.unpack_from("<I", image, off)[0] != 0xABCD5432:
        return None
    return image[off + 144 : off + 152].hex()


def app_desc(image):
    off = 32
    if len(image) < off + 176 or struct.unpack_from("<I", image, off)[0] != 0xABCD5432:
        return None

    def cstr(a, b):
        return image[off + a : off + b].split(b"\0", 1)[0].decode(errors="replace")

    return {
        "version": cstr(16, 48),
        "project": cstr(48, 80),
        "built": "%s %s" % (cstr(96, 112), cstr(80, 96)),
    }


class Client:
    def __init__(self, addr):
        self.addr = addr
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))

    def recv(self, timeout):
        self.sock.settimeout(max(timeout, 0.001))
        try:
            data, src = self.sock.recvfrom(2048)
        except socket.timeout:
            return None
        return data if src == self.addr else None

    def request(self, msg, want_type, tries=10, timeout=1.0):
        """Sends msg until a reply of want_type arrives; returns it."""
        for _ in range(tries):
            self.sock.sendto(msg, self.addr)
            deadline = time.monotonic() + timeout
            while True:
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                data = self.recv(left)
                if data and data[0] == want_type:
                    return data
        raise OtaError("no reply from %s:%d" % self.addr)

    def status(self, tries=5, timeout=1.0):
        r = self.request(bytes([STATUS]), STATUS | RESP, tries, timeout)
        if len(r) < 2 or r[1] != 0:
            raise OtaError("status: %s" % status_name(r[1] if len(r) > 1 else 255))
        return r[2:].decode(errors="replace")

    def reboot(self):
        r = self.request(bytes([REBOOT]), REBOOT | RESP, tries=5)
        if r[1] != 0:
            raise OtaError("reboot: %s" % status_name(r[1]))

    def flash(self, image, window, log=print):
        count = (len(image) + FRAGMENT - 1) // FRAGMENT
        if count == 0 or count > 0xFFFF:
            raise OtaError("image of %d bytes does not fit u16 fragments" % len(image))
        t0 = time.monotonic()
        r = self.request(struct.pack("<BH", START, count), START | RESP, tries=30, timeout=1.0)
        if len(r) < 4 or struct.unpack_from("<H", r, 1)[0] != count:
            raise OtaError("bad start response %r" % r)
        if r[3] != 0:
            raise OtaError("start: %s" % status_name(r[3]))

        frags = [image[i * FRAGMENT : (i + 1) * FRAGMENT] for i in range(count)]
        acked = [False] * count
        sent_at = {}  # fragment -> monotonic time of its last send
        tries = [0] * count
        nxt = 0  # next never-sent fragment
        done = 0
        srtt = None
        retransmits = 0
        last_log = t0

        def send(i):
            self.sock.sendto(struct.pack("<BH", FRAGMENT_REQ, i) + frags[i], self.addr)
            sent_at[i] = time.monotonic()
            tries[i] += 1

        while done < count:
            now = time.monotonic()
            rto = max(0.2, 3 * srtt) if srtt else 1.0
            for i, t in list(sent_at.items()):
                if now - t >= rto:
                    if tries[i] >= 20:
                        raise OtaError("fragment %d: no reply after %d tries" % (i, tries[i]))
                    retransmits += 1
                    send(i)
            while nxt < count and len(sent_at) < window:
                send(nxt)
                nxt += 1
            wait = min(sent_at.values()) + rto - time.monotonic() if sent_at else 0.05
            data = self.recv(wait)
            if not data or data[0] != FRAGMENT_REQ | RESP or len(data) < 4:
                continue
            i, st = struct.unpack_from("<H", data, 1)[0], data[3]
            if i >= count or acked[i]:
                continue
            if st != 0:
                raise OtaError("fragment %d: %s" % (i, status_name(st)))
            if i in sent_at:
                if tries[i] == 1:  # Karn: time only first sends
                    rtt = time.monotonic() - sent_at[i]
                    srtt = rtt if srtt is None else 0.875 * srtt + 0.125 * rtt
                del sent_at[i]
            acked[i] = True
            done += 1
            if time.monotonic() - last_log >= 2:
                last_log = time.monotonic()
                log("  %d/%d fragments, %d retransmits" % (done, count, retransmits))
        dt = time.monotonic() - t0
        log(
            "image accepted: %d B in %.1f s (%.0f kB/s), %d fragments, %d retransmits, rtt %.0f ms"
            % (len(image), dt, len(image) / dt / 1000, count, retransmits, (srtt or 0) * 1000)
        )


def wait_for_image(client, elf, timeout_s, log=print):
    deadline = time.monotonic() + timeout_s
    time.sleep(2)
    while time.monotonic() < deadline:
        try:
            st = client.status(tries=1, timeout=1.0)
        except OtaError:
            continue
        log("status: " + st)
        if elf is None or ("elf=" + elf) in st:
            return st
        raise OtaError("P4 answers with another image (rolled back?), want elf=%s" % elf)
    raise OtaError("P4 did not answer within %d s after reboot" % timeout_s)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("addr", help="host:port of the udp_ota server (or the ground relay)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    sub.add_parser("reboot")
    fl = sub.add_parser("flash")
    fl.add_argument("image")
    fl.add_argument("--window", type=int, default=8, help="fragments in flight (default 8)")
    fl.add_argument("--reboot", action="store_true", help="boot the new image and wait for it")
    fl.add_argument("--boot-timeout", type=int, default=90)
    args = ap.parse_args()

    client = Client(parse_addr(args.addr))
    try:
        if args.cmd == "status":
            print(client.status())
        elif args.cmd == "reboot":
            client.reboot()
            print("rebooting")
        else:
            with open(args.image, "rb") as f:
                image = f.read()
            desc = app_desc(image)
            elf = app_elf_sha(image)
            if desc:
                print(
                    "image: %s %s built %s elf=%s, %d B"
                    % (desc["project"], desc["version"], desc["built"], elf, len(image))
                )
            else:
                print("image: %d B (no esp_app_desc_t found)" % len(image))
            print("before: " + client.status())
            client.flash(image, max(1, args.window))
            if args.reboot:
                client.reboot()
                print("rebooting")
                wait_for_image(client, elf, args.boot_timeout)
    except OtaError as e:
        print("ERROR " + str(e), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
