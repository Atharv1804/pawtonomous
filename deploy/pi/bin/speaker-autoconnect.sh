#!/bin/bash
# Keep the ALTO Bluetooth speaker connected: cheap speakers do not reconnect by themselves.
# After each (re)connect set the volume to 100% (Atharv wants it loud; it can distort the robot mic - the Mac mic is the main one).
MAC=41:42:86:48:B0:9E
VOLUME=1.0
while true; do
  if ! bluetoothctl info "$MAC" 2>/dev/null | grep -q "Connected: yes"; then
    if bluetoothctl --timeout 8 connect "$MAC" >/dev/null 2>&1; then
      sleep 3
      wpctl set-volume @DEFAULT_AUDIO_SINK@ "$VOLUME"
    fi
  fi
  sleep 20
done
