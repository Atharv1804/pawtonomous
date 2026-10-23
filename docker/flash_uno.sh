#!/bin/bash
# Flash a compiled sketch (.hex) onto the Uno that is plugged into the Pi.
#   ~/robot_ws/docker/flash_uno.sh ~/uno_ultrasonic.ino.hex
# Stops the robot software while flashing (it holds the serial port), then starts it again.
set -e
HEX="$(realpath "$1")"
PORT="$(ls /dev/serial/by-id/usb-Arduino* 2>/dev/null | head -1)"
[ -f "$HEX" ] || { echo "no hex file: $1"; exit 1; }
[ -n "$PORT" ] || { echo "no Arduino found in /dev/serial/by-id"; exit 1; }
WAS_RUNNING=$(docker ps -q -f name=^robot$)
[ -n "$WAS_RUNNING" ] && docker stop robot >/dev/null
docker run --rm -v /dev:/dev --device-cgroup-rule='c 166:* rmw' -v "$HEX:/fw.hex:ro" robot_ws:jazzy \
  avrdude -p atmega328p -c arduino -P "$PORT" -b 115200 -D -U flash:w:/fw.hex:i
[ -n "$WAS_RUNNING" ] && docker start robot >/dev/null && echo "robot software restarted"
