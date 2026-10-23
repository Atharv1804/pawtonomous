#!/usr/bin/env python3
"""Drive the robot from the Mac keyboard, straight to the ESP32 (no ROS, NO obstacle stopping).

    python3 ~/robot_ws/tools/keyboard_drive.py [--host 192.168.0.189]      (WiFi, needs the key)
    python3 ~/robot_ws/tools/keyboard_drive.py --serial /dev/cu.usbserial-0001   (USB cable to the ESP32)

    Arrow keys   hold: up = forward, left = spin left, right = pivot right,
                 down = turn around (the right wheels cannot reverse, see the firmware)
    Space        stop now
    + / -        faster / slower
    q            quit (sends STOP)
    1 2 3 4      motor test: left fwd / left back / right fwd / right back (one side only)

Only for first tests with the wheels OFF the ground or in an open space: the
ultrasonic safety layer is bypassed. The ESP32 also stops by itself 0.5 s after
the last command, so closing this window or losing WiFi stops the motors.
Needs the shared key in ~/robot_ws/.robot_udp_key (same as ROBOT_KEY on the ESP32).
"""

import argparse
import curses
import glob
import os
import socket
import termios
import time

SEND_HZ = 20
HOLD_S = 0.5        # keep driving this long after the last key event (covers key-repeat delay)
SPEED_STEP = 20
SPEED_MIN, SPEED_MAX = 120, 255  # PWM out of 255; the robot is heavy (min_pwm 150 in base.yaml)


def load_key():
    path = os.path.expanduser("~/robot_ws/.robot_udp_key")
    with open(path) as f:
        return f.read().strip()


class SerialLink:
    """ESP32 over USB (115200 8N1), stdlib only. Opening the port resets the ESP32 (~1 s boot)."""

    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(self.fd)
        a[0] = 0                                                    # iflag: raw
        a[1] = 0                                                    # oflag: raw
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL         # 8N1
        a[3] = 0                                                    # lflag: no echo/canonical
        a[4] = a[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        self.label = path

    def send(self, cmd):
        os.write(self.fd, (cmd + "\n").encode("ascii"))

    def read(self):
        try:
            return os.read(self.fd, 4096).decode("ascii", "replace")
        except BlockingIOError:
            return ""


def ping_serial(path, wait=8.0):
    """Send PING until PONG comes back (the ESP32 reboots when the port opens)."""
    link, got, t = SerialLink(path), "", time.time()
    while time.time() - t < wait:
        link.send("PING")
        time.sleep(0.3)
        got += link.read()
        if "PONG" in got:
            return True, round(time.time() - t, 1)
    return False, got[-200:]


def run(scr, host, port, key, speed, serial_path=None):
    curses.curs_set(0)
    scr.nodelay(True)
    if serial_path:
        link = SerialLink(serial_path)
        time.sleep(1.5)                       # ESP32 reboots when the port opens
        send = link.send
        addr = ("USB", serial_path)
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addr = (socket.gethostbyname(host), port)

        def send(cmd):
            sock.sendto(("%s %s" % (key, cmd)).encode("ascii"), addr)

    moves = {
        # The right wheels can only go forward (dead RPWM input), so: back = turn around by
        # spinning left, right = pivot on the left wheels. See RIGHT_FORWARD_ONLY in the firmware.
        curses.KEY_UP: (1, 1, "FORWARD"),
        curses.KEY_DOWN: (-1, 1, "TURN AROUND (spin left)"),
        curses.KEY_LEFT: (-1, 1, "SPIN LEFT"),
        curses.KEY_RIGHT: (1, 0, "PIVOT RIGHT"),
        ord("1"): (1, 0, "TEST: LEFT side forward"),
        ord("2"): (-1, 0, "TEST: LEFT side backward"),
        ord("3"): (0, 1, "TEST: RIGHT side forward"),
        ord("4"): (0, -1, "TEST: RIGHT side backward"),
    }
    current = None          # (l_sign, r_sign, label)
    last_key = 0.0
    try:
        while True:
            ch = scr.getch()
            while ch != -1:
                if ch in moves:
                    current, last_key = moves[ch], time.time()
                elif ch == ord(" "):
                    current = None
                elif ch in (ord("+"), ord("=")):
                    speed = min(SPEED_MAX, speed + SPEED_STEP)
                elif ch in (ord("-"), ord("_")):
                    speed = max(SPEED_MIN, speed - SPEED_STEP)
                elif ch in (ord("q"), ord("Q")):
                    return
                ch = scr.getch()

            if current and time.time() - last_key > HOLD_S:
                current = None
            if current:
                send("VEL %d %d" % (current[0] * speed, current[1] * speed))
            else:
                send("STOP")

            scr.erase()
            scr.addstr(0, 0, "ESP32 %s %s   (NO obstacle stopping - keep the area clear)" % addr)
            scr.addstr(2, 0, "Arrows = drive (hold)   Space = stop   +/- = speed   q = quit")
            scr.addstr(3, 0, "Motor test (hold): 1 = left fwd  2 = left back  3 = right fwd  4 = right back")
            scr.addstr(5, 0, "Speed: %d / 255" % speed)
            scr.addstr(6, 0, "Now:   %s" % (current[2] if current else "STOPPED"))
            scr.refresh()
            time.sleep(1.0 / SEND_HZ)
    finally:
        for _ in range(3):
            send("STOP")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="robot-motors.local", help="ESP32 name or IP")
    ap.add_argument("--port", type=int, default=4210)
    ap.add_argument("--speed", type=int, default=230, help="start PWM (120-255)")
    ap.add_argument("--serial", nargs="?", const="auto", help="drive over USB (default: first /dev/cu.usbserial-*)")
    ap.add_argument("--ping", action="store_true", help="only check the USB link (PING/PONG), no driving")
    a = ap.parse_args()
    serial_path = None
    if a.serial:
        found = sorted(glob.glob("/dev/cu.usbserial-*") + glob.glob("/dev/cu.SLAB*") + glob.glob("/dev/cu.wchusbserial*"))
        serial_path = found[0] if a.serial == "auto" and found else a.serial
        if a.serial == "auto" and not found:
            raise SystemExit("No ESP32 USB port found (is the data cable plugged in?)")
    if a.ping:
        ok, info = ping_serial(serial_path or "/dev/cu.usbserial-0001")
        raise SystemExit(("PONG after %ss - USB link OK" % info) if ok else ("no PONG: %r" % info))
    key = "" if serial_path else load_key()
    curses.wrapper(run, a.host, a.port, key, max(SPEED_MIN, min(SPEED_MAX, a.speed)), serial_path)
    print("Stopped.")


if __name__ == "__main__":
    main()
