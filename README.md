# Pawtonomous: "Jarvis", an AI robot pet that sees, listens, talks and explores

A low-cost companion robot car with a **local AI brain**: it recognises faces, follows or finds
people, explores a room on its own using its camera, answers questions out loud, shows its mood on an
animated face, and can be controlled by voice ("Hey Jarvis…") or Telegram. Almost everything runs on
hardware you own (a Raspberry Pi 5 + a laptop). There are no paid APIs and no cloud camera.

> Status: working prototype. Voice, vision, face recognition, Telegram, the animated face, motor
> control and camera-guided exploring work. Known hardware limits are listed at the end.

## What it can do
| Say / send | What happens |
|---|---|
| "Hey Jarvis, who do you see?" | names the people it recognises (instant, no LLM) |
| "Hey Jarvis, what do you see?" | describes the camera view (~1-2 s, local vision model) |
| "Hey Jarvis, follow me" / "follow Atharv" | turns to and follows that person, ~1 m behind |
| "Hey Jarvis, follow me straight" | straight-line follow test mode (no turning) |
| "Hey Jarvis, come here" / "find Atharv" | finds the person (spin + explore) and drives to them |
| "Hey Jarvis, go explore" | curious exploring: camera + small vision model pick the way, it slows near corners, stops to comment on things it finds ("Ooh, a big shiny ball!") and briefly shows its camera view on screen |
| "Hey Jarvis, stop" | stops everything at once |
| Telegram: a photo + "this is Rahul, learn his face" | learns a face from ONE photo |
| Telegram: "send me a picture of what you see" | sends a photo from the robot |
| anything else | answered by the local LLM agent (Hermes + gpt-oss-20b) |

## How it works
```
              ┌──────────────────────────── Mac (the heavy brain) ─────────────────────────────┐
  MacBook mic │ voice loop ─┐   LM Studio: gpt-oss-20b (agent LLM)    Ollama: qwen3.5:2b (scene)│
              │             ├─► mac_ai: Whisper (Apple GPU), Piper TTS                          │
              │ pet brain: OpenCV YuNet + SFace (faces) + NanoDet (people), follow / find /      │
              │            curious-explore controller, face-screen state                       │
              └───────────────▲──────────────────────────── SSH tunnel (one link) ─────────────┘
                              │ camera MJPEG, Body API, speaker          ▲ LLM, brain, speech
              ┌───────────────┴──────────── Raspberry Pi 5 (the body) ───┴──────────────────────┐
  robot mic ─►│ voice loop ("Hey Jarvis"), Hermes Agent + Telegram, MCP robot tools (22)          │
  camera ────►│ pi_io: camera pass-through (no decoding, ~2% CPU), speaker playback, battery      │
  Uno+sonars ►│ ROS 2 Jazzy (Docker): ultrasonic bridge → SAFETY REFLEX → motor bridge ─ UDP ─► ESP32 → 2×BTS7960 → 4 motors
  screen ◄────│ Chromium kiosk: animated eyes + mouth, status line, mini camera view             │
              └──────────────────────────────────────────────────────────────────────────────────┘
```
* **Two speeds of thinking.** Reflexes run locally and fast: obstacle stop, instant voice
  commands, 8-12 Hz face and person tracking. The LLM only plans ("find Atharv") and hands the task to
  the pet brain, which carries it out by itself and can be interrupted any time.
* **Safety first.** ROS `safety_reflex` sits between every behaviour and the motors (side obstacles,
  stale sensor data, 0.5 s command timeout). The ESP32 has its own 0.5 s watchdog and only accepts
  commands signed with a shared secret. Only the owner's Telegram account can control it.
* **The Pi stays cool.** Vision, speech and the LLMs run on the Mac. The Pi went from ~85% CPU / 63 °C
  to ~15% / ~50 °C.

## Hardware
Raspberry Pi 5 (8 GB) on a Waveshare UPS HAT (E) · ESP32 DevKit + 2× BTS7960 + 4 DC motors + 3S LiPo ·
Arduino Uno + 3× HC-SR04 (TRIG/ECHO: left D2/D3, centre D4/D5, right **D8/D9**) · USB camera, USB mic array,
Bluetooth speaker, 7" HDMI touchscreen · a MacBook (Apple silicon, 24 GB) as the off-board brain.

ESP32 → BTS7960: left R_EN 17, L_EN 16, RPWM 22, LPWM 23 · right R_EN 32, L_EN 33, RPWM **26**, LPWM 25.
Driver VCC → ESP32 3V3, all grounds common, battery only on B+/B−.

## Repository layout
| Path | What |
|---|---|
| `src/robot_base/` | ROS 2 package: ultrasonic bridge, safety reflex, behaviours, motor bridge, Body API (+113 tests) |
| `firmware/esp32_motor/` | motor controller: UDP + USB serial, secret key, smooth ramp, watchdog, wiring workarounds |
| `firmware/uno_ultrasonic/` | 3 ultrasonic sensors → serial |
| `tools/pet/` | **pet brain** (Mac): `vision.py`, `control.py`, `intents.py`, `brain.py`, `face.html` |
| `tools/voice_loop.py` | "Hey Jarvis" loop (runs on the Pi and on the Mac, shared turn lock) |
| `tools/robot_mcp_server.py` | 22 MCP tools for the agent (drive, follow, find, look, listen, say, battery…) |
| `tools/pi_io.py`, `tools/robot_media.py`, `tools/robot_power.py` | Pi camera/speaker/battery services |
| `tools/keyboard_drive.py` | arrow-key driving over WiFi or USB (with single-wheel test keys) |
| `mac/mac_ai.py`, `mac/pi-tunnel.sh` | Mac speech service and the self-healing Mac↔Pi tunnel |
| `deploy/` | systemd user services (Pi), launchd agents (Mac), audio fixes, Hermes settings + `SOUL.md` |
| `docs/ROBOT_BRIEF.md` | what the agent knows about its own body |

## Setup (short version)
1. **Firmware.** Flash `firmware/uno_ultrasonic` to the Uno. Copy `firmware/esp32_motor/secrets.h.example` to
   `secrets.h` (WiFi + a long random `ROBOT_KEY`) and flash the ESP32. Put the same key in `~/robot_ws/.robot_udp_key` on the Pi.
2. **Pi:** Docker (`docker/robot.sh build && docker/robot.sh start`; `MOTORS=true` enables the motors),
   Python venv in `tools/.venv` (mcp, opencv-python-headless, faster-whisper, piper-tts, pyopen-wakeword,
   sounddevice, smbus2), the systemd user services in `deploy/pi/`, Hermes per `deploy/pi/hermes/hermes-settings.md`.
3. **Mac:** LM Studio (`lms load openai/gpt-oss-20b --context-length 65536 --parallel 1`), Ollama
   (`ollama pull qwen3.5:2b`), venv `mac/.venv` (opencv-python-headless, mlx-whisper, piper-tts, pyopen-wakeword,
   sounddevice), the launchd agents in `deploy/mac/launchagents/`. Models: OpenCV zoo YuNet / SFace / NanoDet in `models/`.
4. **Tests:** `cd src/robot_base && PYTHONPATH=. pytest test` (113 tests, no hardware needed).

## Known limitations
* The right motor driver cannot reverse: right turns are pivots and "back" turns around (firmware
  `RIGHT_FORWARD_ONLY`).
* The centre ultrasonic is ignored (it misreads), so the camera + vision model cover the area straight ahead.
  Low objects near the floor can still be missed.
* Distances and angles are open-loop (no wheel encoders).
* The smart features need the Mac awake on the same network as the Pi. Without it, the Pi falls back to
  basic local speech.

## Credits
Built with ROS 2, Hermes Agent (Nous Research), LM Studio + gpt-oss-20b, Ollama + Qwen, OpenCV model zoo,
Whisper (MLX / faster-whisper), Piper, openWakeWord, and the Model Context Protocol.
