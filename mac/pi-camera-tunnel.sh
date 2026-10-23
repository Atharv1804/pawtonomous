#!/bin/bash
# The camera video (Pi -> Mac, ~0.5 MB/s) gets its own SSH connection: in the shared tunnel it queued
# every small message (Body API, face screen, voice) behind video frames, adding 1-3 s of delay.
# Watchdog: if the camera stops answering through the tunnel twice in a row, exit so launchd restarts it.
/usr/bin/ssh -N -o BatchMode=yes -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=3 \
  -o ConnectTimeout=10 -L 127.0.0.1:18767:127.0.0.1:8767 pi &
pid=$!
sleep 8
fails=0
while kill -0 $pid 2>/dev/null; do
  if curl -s -m 8 -o /dev/null http://127.0.0.1:18767/health; then fails=0; else fails=$((fails + 1)); fi
  if [ $fails -ge 3 ]; then echo "$(date) camera tunnel stalled, restarting" >&2; kill $pid; break; fi
  sleep 10
done
wait $pid
