#!/bin/bash
# Mac <-> Pi link for the robot (run by the LaunchAgent com.atharv.pi-telegram-tunnel).
# 1) End stale link sessions on the Pi: if the previous ssh died without the Pi noticing, its sshd
#    session keeps the forwarded ports (1080, 1234, ...) and every reconnect fails with
#    "remote port forwarding failed" (that happened 134 times on 2026-10-23).
#    Tunnel sessions show up as exactly "sshd-session: atharv" (no tty, no command); interactive
#    logins (@pts/N) and command sessions (@notty) are left alone.
ssh -o BatchMode=yes -o ConnectTimeout=10 pi 'pkill -u "$(id -u)" -f "^sshd-session: atharv$"' 2>/dev/null
sleep 1
# 2) The link itself (forwards documented in CLAUDE.md "Current architecture").
exec /usr/bin/ssh -N -o BatchMode=yes -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -o ServerAliveCountMax=3 -o ConnectTimeout=10 -R 127.0.0.1:1080 -R 127.0.0.1:11434:127.0.0.1:11434 -R 127.0.0.1:1234:127.0.0.1:1234 -R 127.0.0.1:8766:127.0.0.1:8766 -R 127.0.0.1:8768:127.0.0.1:8768 -L 127.0.0.1:18765:127.0.0.1:8765 -L 127.0.0.1:18767:127.0.0.1:8767 -L 127.0.0.1:18642:127.0.0.1:8642 pi 
