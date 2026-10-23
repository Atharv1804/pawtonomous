#!/usr/bin/env python3
"""Hands-free voice for the robot: "Hey Jarvis" -> listen -> Hermes -> speak the answer.

Runs on the Pi as a systemd user service (see tools/robot-voice.service). Everything
except the thinking is offline: wake word (openWakeWord "hey_jarvis"), speech-to-text
(faster-whisper) and the voice (Piper, via robot_media.say).

Flow per turn:
  wake chime -> record until ~0.8 s of silence (max 12 s) -> "got it" blip -> transcribe
  -> Hermes API server on 127.0.0.1:8642 (same session for follow-ups within CONVERSATION_GAP_S)
  -> speak reply. Falls back to `hermes -z` if the API server is not enabled.
The mic is ignored while Jarvis thinks and talks, so it does not wake itself up.

Environment (optional): VOICE_WHISPER_MODEL (default base.en), VOICE_WAKE_THRESHOLD (0.5), VOICE_WAKE_CONFIRM (2),
VOICE_HERMES (path to the hermes command).
"""

import glob
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
import wave

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "pet"))
import intents  # noqa: E402  instant local commands (stop, come here, find X, ...)
import robot_media  # noqa: E402  Piper voice + Whisper loader shared with the MCP server
import robot_power  # noqa: E402

ON_MAC = sys.platform == "darwin"   # the same loop runs on the Mac (MacBook mic) and on the Pi (robot mic)
WHO = "mac" if ON_MAC else "pi"
RATE = 16000
FRAME = 1280                      # 80 ms of 16 kHz int16 audio (what openWakeWord expects)
FRAME_S = FRAME / RATE
WAKE_THRESHOLD = float(os.environ.get("VOICE_WAKE_THRESHOLD", "0.5"))
# A real "hey jarvis" scores high for several frames in a row; a stray sound spikes one.
# Measured on this robot: a clear "Hey Jarvis" held >= 0.8 for only 2 frames, so 3 never fired.
WAKE_CONFIRM_FRAMES = int(os.environ.get("VOICE_WAKE_CONFIRM", "2"))
START_TIMEOUT_S = 6.0             # give up if nobody starts talking after the chime
SPEECH_START_FRAMES = 5           # ~0.4 s of loud audio = speech (the 0.24 s chime and its echo are shorter)
END_SILENCE_S = 0.8               # this much quiet after speech = done talking
CHIME_ECHO_S = 0.2 if ON_MAC else 0.6   # Pi: Bluetooth plays the chime ~0.3-0.5 s late; Mac: own speaker
MAX_UTTERANCE_S = 12.0
CONVERSATION_GAP_S = 300          # follow-ups within 5 minutes continue the same Hermes session
HERMES = os.environ.get("VOICE_HERMES", os.path.expanduser("~/.local/bin/hermes"))
API_URL = os.environ.get("HERMES_API_URL", "http://127.0.0.1:8642") + "/v1/chat/completions"   # Mac: :18642
# Whisper hint: makes the robot's own vocabulary (and its name) more likely to be heard correctly.
STT_HINT = "Hey Jarvis. Talking to Jarvis, a robot car with distance sensors, a camera, a microphone and a speaker."
VOICE_PREFIX = ("[Spoken to you through the robot's microphone. Answer in 1-3 short spoken sentences, "
                "plain words, no markdown, lists or emojis. Do not call robot_say: your reply is spoken "
                "aloud automatically.] ")


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


def tone(path, notes, seconds=0.12, volume=9000):
    """Write a short beep sequence (frequencies in Hz) to a WAV file."""
    with wave.open(path, "wb") as w:
        w.setnchannels(1), w.setsampwidth(2), w.setframerate(22050)
        for f in notes:
            n = int(22050 * seconds)
            fade = np.minimum(1, np.minimum(np.arange(n), np.arange(n)[::-1]) / 300.0)
            w.writeframes((volume * fade * np.sin(2 * np.pi * f * np.arange(n) / 22050)).astype(np.int16).tobytes())


def play(path):
    """Chimes: on the Mac its own speaker (instant feedback that the Mac heard you), on the Pi the robot's."""
    cmd = ["afplay", "-v", "0.4", path] if ON_MAC else ["pw-play", path]
    subprocess.run(cmd, capture_output=True, env=robot_media._env(), timeout=10)


def rms(frame):
    return math.sqrt(float(np.mean(frame.astype(np.float32) ** 2))) if len(frame) else 0.0


class Mic:
    """Continuous 16 kHz mono capture through PipeWire (shared with other apps)."""

    def __init__(self):
        self.p = subprocess.Popen(["pw-record", "--rate", str(RATE), "--channels", "1", "--format", "s16",
                                   "--raw", "-"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                  env=robot_media._env(), bufsize=0)

    def read(self):
        data = b""
        while len(data) < FRAME * 2:
            chunk = self.p.stdout.read(FRAME * 2 - len(data))
            if not chunk:
                raise RuntimeError("microphone stream ended")
            data += chunk
        return np.frombuffer(data, dtype=np.int16)

    def drain(self, seconds):
        """Throw away audio for `seconds` (e.g. our own chime / speech still echoing)."""
        for _ in range(int(seconds / FRAME_S)):
            self.read()

    def close(self):
        self.p.kill()


class MacMic(Mic):
    """MacBook microphone via sounddevice. CoreAudio gives 48 kHz: average every 3 samples -> 16 kHz.
    (macOS asks once for microphone permission for this Python.)"""

    def __init__(self):
        import queue
        import sounddevice as sd
        self.q = queue.Queue(maxsize=200)

        def cb(indata, frames, t, status):
            try:
                self.q.put_nowait(bytes(indata))
            except queue.Full:
                pass
        # Always the built-in mic: AirPods/Bluetooth headsets become the default input when they
        # connect and then send silence while not worn (that broke "hey jarvis" on 2026-10-23).
        want = os.environ.get("VOICE_MAC_MIC", "MacBook Pro Microphone").lower()
        device = next((i for i, d in enumerate(sd.query_devices())
                       if d["max_input_channels"] > 0 and want in d["name"].lower()), None)
        log("Mac microphone: %s" % (sd.query_devices(device)["name"] if device is not None else "system default"))
        self.stream = sd.RawInputStream(samplerate=48000, channels=1, dtype="int16", blocksize=FRAME * 3,
                                        callback=cb, device=device)
        self.stream.start()

    def read(self):
        raw = np.frombuffer(self.q.get(timeout=5), dtype=np.int16)
        return raw[: len(raw) // 3 * 3].reshape(-1, 3).mean(axis=1).astype(np.int16)

    def close(self):
        self.stream.stop()
        self.stream.close()


class WakeWord:
    def __init__(self):
        from pyopen_wakeword import OpenWakeWord, OpenWakeWordFeatures
        import pyopen_wakeword
        path = glob.glob(os.path.join(os.path.dirname(pyopen_wakeword.__file__), "**", "hey_jarvis.tflite"),
                         recursive=True)[0]
        self.features = OpenWakeWordFeatures.from_builtin()
        self.model = OpenWakeWord.from_model(path)
        self.streak = 0

    def heard(self, frame):
        over = False
        for emb in self.features.process_streaming(frame.tobytes()):
            for score in self.model.process_streaming(emb):
                self.best = max(getattr(self, "best", 0.0), score)
                over = over or score >= WAKE_THRESHOLD
        self.streak = self.streak + 1 if over else 0
        if self.streak >= WAKE_CONFIRM_FRAMES:
            self.streak = 0
            return True
        return False

    def reset(self):
        self.streak = 0
        self.features.reset()
        self.model.reset()


def record_utterance(mic, noise_floor):
    """Record from the first speech until END_SILENCE_S of quiet. Returns int16 array or None."""
    speech_level = max(noise_floor * 2.5, 400.0)
    frames, started, quiet, waited, run, peak = [], False, 0.0, 0.0, [], 0.0
    while True:
        f = mic.read()
        loud = rms(f) > speech_level
        if not started:
            waited += FRAME_S
            run = run + [f] if loud else []           # consecutive loud frames so far
            if len(run) >= SPEECH_START_FRAMES:
                started, frames = True, run
            elif waited > START_TIMEOUT_S:
                return None
            continue
        frames.append(f)
        level = rms(f)
        peak = max(peak, level)
        # "Still talking" is relative to this speaker's own loudness, so a noisy room (people,
        # music) does not keep the recording open until MAX_UTTERANCE_S.
        talking = level > max(speech_level * 0.8, peak * 0.25)
        quiet = 0.0 if talking else quiet + FRAME_S
        if quiet >= END_SILENCE_S or len(frames) * FRAME_S >= MAX_UTTERANCE_S:
            log("utterance: %.1fs, noise %.0f, start level %.0f, peak %.0f, ended by %s" % (
                len(frames) * FRAME_S, noise_floor, speech_level, peak,
                "silence" if quiet >= END_SILENCE_S else "max length"))
            return np.concatenate(frames)


def transcribe(audio):
    """Mac GPU Whisper (accurate, keeps the Pi cool); falls back to the Pi's small model."""
    text, where = robot_media.stt(audio.astype(np.int16))
    transcribe.where = where
    return text


WAKE_ONLY = re.compile(r"^\W*((hey|hi|ok|okay|a)\W+)?(jarvis|jarvi|travis|service)\W*$", re.I)


def strip_wake(text):
    """Drop a leading 'hey jarvis' that leaked into the recording; '' if that was all."""
    if WAKE_ONLY.match(text or "") or not re.search(r"[A-Za-z]", text or ""):   # wake word or "!" (chime)
        return ""
    return re.sub(r"^\W*((hey|hi|ok|okay)\W+)?jarvis\W+", "", text or "", flags=re.I).strip()


def newest_session_id():
    out = subprocess.run([HERMES, "sessions", "list"], capture_output=True, text=True, timeout=60).stdout
    m = re.search(r"(local-[0-9a-f]{16,}|\d{8}_\d{6}_[0-9a-f]+)", re.sub(r"\x1b\[[0-9;]*m", "", out))
    return m.group(1) if m else None


def _api_key():
    keyfile = os.path.expanduser(os.environ.get("HERMES_API_KEY_FILE", ""))
    if keyfile and os.path.exists(keyfile):                 # Mac: a copy of the Pi's API_SERVER_KEY
        return open(keyfile).read().strip()
    try:
        for line in open(os.path.expanduser("~/.hermes/.env")):
            if line.startswith("API_SERVER_KEY="):
                return line.split("=", 1)[1].strip()
    except OSError:
        pass
    return ""


def ask_hermes_api(text, session):
    """Ask the already-running Hermes gateway (no CLI start-up). Returns (reply, session) or (None, session)."""
    key = _api_key()
    if not key:
        return None, session
    headers = {"Authorization": "Bearer " + key, "Content-Type": "application/json"}
    if session:
        headers["X-Hermes-Session-Id"] = session
    body = {"model": "hermes-agent", "messages": [{"role": "user", "content": VOICE_PREFIX + text}]}
    try:
        with urllib.request.urlopen(urllib.request.Request(API_URL, json.dumps(body).encode(), headers),
                                    timeout=180) as r:
            new_session = r.headers.get("X-Hermes-Session-Id") or session
            reply = (json.load(r)["choices"][0]["message"].get("content") or "").strip()
    except (urllib.error.URLError, OSError, ValueError, KeyError, IndexError) as e:
        log("api server failed: %s" % e)
        return None, session
    if not reply or reply.startswith("⚠"):
        log("hermes gave no answer: %r" % reply[:200])
        return None, new_session
    return reply, new_session


def ask_hermes(text, session):
    cmd = [HERMES] + (["--resume", session] if session else []) + ["-z", VOICE_PREFIX + text]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=180, cwd=os.path.expanduser("~"))
    reply = re.sub(r"\x1b\[[0-9;]*m", "", r.stdout).strip()
    if r.returncode != 0 or not reply or reply.startswith("⚠"):
        log("hermes failed (code %s): %s %s" % (r.returncode, reply[-200:], r.stderr.strip()[-200:]))
        return None
    return reply


def face(name, seconds=30.0, status=None):
    """Show an expression (and optionally a status line) on the screen; harmless if the brain is down."""
    robot_media._pet("POST", "/expression", {"name": name, "seconds": seconds}, timeout=0.5)
    if status is not None:
        robot_media._pet("POST", "/status", {"text": status, "seconds": seconds}, timeout=0.5)


def fast_path(text):
    """Handle simple commands locally in well under a second. Returns the spoken reply or None."""
    it = intents.parse(text)
    if it is None:
        return None
    if it["action"] == "pet":
        if it["mode"] == "idle":
            robot_media._pet("POST", "/stop", {}, timeout=2)
        else:
            r = robot_media._pet("POST", "/command", {"mode": it["mode"], "target": it.get("target", "")}, timeout=2)
            if r is None:
                return "My movement brain is not running right now."
        return it["reply"]
    if it["action"] == "see":
        face("thinking", 5)
        return robot_media.describe_scene() or "I can't see clearly right now."
    if it["action"] == "battery":
        if ON_MAC:                                           # the battery HAT is read on the Pi
            raw = robot_media._http(robot_media.PI_IO_URL + "/battery")
            try:
                b = json.loads(raw) if raw else {"ok": False}
            except ValueError:
                b = {"ok": False}
        else:
            b = robot_power.battery_status()
        if not b.get("ok"):
            return "I can't read my battery right now."
        extra = (" and charging" if b["charging"] else
                 ", about %d hours %d minutes left" % divmod(b["minutes_left"], 60) if "minutes_left" in b else "")
        return "My battery is at %d percent%s." % (b["percent"], extra)
    if it["action"] == "who":
        raw = robot_media._pet("GET", "/state", timeout=2)
        try:
            seeing = json.loads(raw)["seeing"] if raw else []
        except ValueError:
            seeing = []
        names = sorted({d["name"].capitalize() for d in seeing if d["name"] != "unknown"})
        unknown = sum(1 for d in seeing if d["name"] == "unknown" and d["kind"] == "face")
        if not seeing:
            return "I don't see anyone right now."
        parts = ([" and ".join(names)] if names else []) + (["%d person%s I don't know" % (unknown, "" if unknown == 1 else "s")] if unknown else [])
        return "I can see " + (", and ".join(parts) if parts else "someone") + "."
    return None


def speakable(text):
    text = re.sub(r"[*_`#>|]+", "", text)              # markdown symbols
    text = re.sub(r"https?://\S+", "a link", text)
    return re.sub(r"\s+", " ", text).strip()


def main():
    tmp = tempfile.mkdtemp(prefix="voice_")
    wake_wav, ack_wav = os.path.join(tmp, "wake.wav"), os.path.join(tmp, "ack.wav")
    tone(wake_wav, [660, 990])
    tone(ack_wav, [520], seconds=0.08)

    log("loading speech model (%s)..." % os.environ.setdefault("ROBOT_WHISPER_MODEL",
                                                               os.environ.get("VOICE_WHISPER_MODEL", "base.en")))
    wake, mic = WakeWord(), (MacMic() if ON_MAC else Mic())
    noise = 300.0
    session, last_turn = None, 0.0
    log("ready - say 'Hey Jarvis'")
    debug, n, loudest = bool(os.environ.get("VOICE_DEBUG")), 0, 0.0

    while True:
        f = mic.read()
        level = rms(f)
        if level < noise * 2:                         # learn room noise only from quiet frames,
            noise = 0.98 * noise + 0.02 * level       # so speech does not raise the bar for speech
        fired = wake.heard(f)
        if debug:                                     # VOICE_DEBUG=1: loudness + best wake score every 2 s
            n, loudest = n + 1, max(loudest, rms(f))
            if n % 25 == 0:
                log("debug: loudest %.0f  noise %.0f  best wake score %.2f" % (loudest, noise, getattr(wake, "best", 0)))
                loudest, wake.best = 0.0, 0.0
        if not fired:
            continue

        claim = robot_media._pet("POST", "/voice_claim", {"who": WHO, "seconds": 60}, timeout=1)
        if claim and not json.loads(claim).get("ok", True):   # the other microphone is already on it
            log("wake word (the %s mic is handling it)" % json.loads(claim).get("holder"))
            wake.reset()
            continue
        log("wake word")
        face("listening", 15, "Listening...")
        play(wake_wav)
        mic.drain(CHIME_ECHO_S)
        audio = record_utterance(mic, noise)
        if audio is None:
            log("no speech after wake word")
            face("neutral", 0.1)
            robot_media._pet("POST", "/voice_claim", {"who": WHO, "seconds": 0}, timeout=1)
            wake.reset()
            continue
        play(ack_wav)
        face("thinking", 10, "Understanding...")
        t0 = time.time()
        text = strip_wake(transcribe(audio))
        if not text:                                    # only the tail of "hey jarvis": keep listening
            log("heard only the wake word; listening again")
            audio = record_utterance(mic, noise)
            if audio is not None:
                play(ack_wav)
                t0 = time.time()
                text = strip_wake(transcribe(audio))
        log("heard (%.1fs audio, %.1fs stt on %s): %r" % (len(audio) / RATE, time.time() - t0,
                                                         getattr(transcribe, "where", "?"), text))
        quick = fast_path(text) if len(text) >= 2 else None
        if len(text) < 2:
            face("sad", 2, "Didn't catch that")
            robot_media.say("Sorry, I didn't catch that.")
        elif quick:
            log("fast path (%.2fs): %r" % (time.time() - t0, quick))
            face("happy", 2, "")
            robot_media.say(quick)
        else:
            face("thinking", 60, "Thinking: \"%s\"" % (text[:40] + ("..." if len(text) > 40 else "")))
            if time.time() - last_turn > CONVERSATION_GAP_S:
                session = None                          # long pause: start a fresh conversation
            t1 = time.time()
            reply, session = ask_hermes_api(text, session)
            if reply is None and not _api_key() and os.path.exists(HERMES):   # no API server: use the CLI
                reply = ask_hermes(text, session)
                if reply and session is None:
                    session = newest_session_id()
            log("reply in %.1fs: %r" % (time.time() - t1, (reply or "")[:200]))
            if reply:
                last_turn = time.time()
                face("happy", 1, "")
                robot_media.say(speakable(reply))
            else:
                face("sad", 3, "My brain is not reachable")
                robot_media.say("Sorry, my brain is not reachable right now.")
        robot_media._pet("POST", "/voice_claim", {"who": WHO, "seconds": 0}, timeout=1)
        # drop whatever the mic picked up while we were busy (including our own voice)
        mic.close()
        mic = MacMic() if ON_MAC else Mic()
        mic.drain(0.3)
        wake.reset()


if __name__ == "__main__":
    main()
