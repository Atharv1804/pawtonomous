#!/bin/bash
# The camera video (Pi -> Mac, ~0.5 MB/s) gets its own SSH connection: in the shared tunnel it queued
# every small message (Body API, face screen, voice) behind video frames, adding 1-3 s of delay.
exec /usr/bin/ssh -N -o BatchMode=yes -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
  -o ConnectTimeout=10 -L 127.0.0.1:18767:127.0.0.1:8767 pi
