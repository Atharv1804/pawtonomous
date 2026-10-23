#!/bin/bash
# Show the robot's animated face full screen on the touchscreen. Run by the systemd user service
# robot-face (tools/robot-face.service). Reopens the browser if it closes.
URL=http://127.0.0.1:8766/face
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)} WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}
until [ -S "$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY" ]; do sleep 2; done                 # wait for the desktop
until curl -s -m 2 http://127.0.0.1:8766/health >/dev/null; do sleep 2; done   # wait for the pet brain
while true; do
  chromium --kiosk --app="$URL" --ozone-platform=wayland --noerrdialogs --disable-infobars \
    --no-first-run --password-store=basic --disable-translate --disable-features=TranslateUI \
    --overscroll-history-navigation=0 --disable-pinch \
    --user-data-dir="$HOME/.cache/robot-face-chromium" >/dev/null 2>&1
  sleep 3
done
