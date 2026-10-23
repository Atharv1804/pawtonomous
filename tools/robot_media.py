"""Camera, speaker and microphone of the robot, for the MCP server (robot_mcp_server.py).

These talk to the Pi's own devices directly (not through ROS), so they work even
while the motors / ESP32 are not connected. Heavy libraries (piper, faster-whisper)
are imported only when first used.

Environment (all optional):
  ROBOT_CAMERA        camera device, default: first /dev/v4l/by-id/*video-index0
  ROBOT_CAPTURE_DIR   where photos go, default ~/robot_ws/captures (keeps the newest 30)
  ROBOT_PIPER_VOICE   Piper .onnx voice, default ~/.hermes/cache/piper-voices/en_US-lessac-medium.onnx
  ROBOT_WHISPER_MODEL faster-whisper model name, default "base"
  ROBOT_FFMPEG        ffmpeg binary, default Hermes's bundled one, else "ffmpeg" on PATH
"""

import array
import glob
import json
import math
import os
import shutil
import subprocess
import tempfile
import threading
import time
import urllib.request
import wave

MAX_PHOTOS = 30
PET_URL = os.environ.get("PET_URL", "http://127.0.0.1:8766")   # pet brain (runs on the Mac, via the tunnel)
MAC_AI_URL = os.environ.get("MAC_AI_URL", "http://127.0.0.1:8768")   # Mac speech service (Whisper GPU, Piper)
PI_IO_URL = os.environ.get("PI_IO_URL", "http://127.0.0.1:8767")     # Pi camera pass-through (pi_io.py)
STT_HINT = "Hey Jarvis. Talking to Jarvis, a robot car with distance sensors, a camera, a microphone and a speaker."
_whisper = None
_whisper_lock = threading.Lock()
_audio_lock = threading.Lock()   # one sound in or out at a time (the mic would hear the speaker)


def _env():
    """Environment for PipeWire tools (pw-play / pw-record) when started outside a desktop session."""
    env = dict(os.environ)
    env.setdefault("XDG_RUNTIME_DIR", "/run/user/%d" % os.getuid())
    return env


def _ffmpeg():
    found = sorted(glob.glob(os.path.expanduser("~/.hermes/tools/ffmpeg-*/bin/ffmpeg")))
    return os.environ.get("ROBOT_FFMPEG") or (found[-1] if found else shutil.which("ffmpeg") or "ffmpeg")


def _camera():
    found = sorted(glob.glob("/dev/v4l/by-id/*video-index0"))
    return os.environ.get("ROBOT_CAMERA") or (found[0] if found else "/dev/video0")


def _http(url, data=None, ctype="application/json", timeout=3.0, method=None):
    """POST/GET helper: response bytes, or None if the service is not reachable."""
    try:
        req = urllib.request.Request(url, data, {"Content-Type": ctype}, method=method)
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.read()
    except OSError:
        return None


def _pet(method, path, payload=None, timeout=3.0):
    """Talk to the pet brain; None if it is not running."""
    data = json.dumps(payload).encode() if payload is not None else None
    return _http(PET_URL + path, data, timeout=timeout, method=method)


def _prune(folder):
    for old in sorted(glob.glob(os.path.join(folder, "photo_*.jpg")))[:-MAX_PHOTOS]:
        os.remove(old)


def take_photo(width=1280, height=720):
    """Save one camera frame as JPEG. Returns {"ok", "path", ...}.
    While the pet brain runs it owns the camera, so the frame comes from it (instant)."""
    folder = os.path.expanduser(os.environ.get("ROBOT_CAPTURE_DIR", "~/robot_ws/captures"))
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, "photo_%s.jpg" % time.strftime("%Y%m%d_%H%M%S"))
    jpg = _http(PI_IO_URL + "/frame.jpg") or _pet("GET", "/frame.jpg")
    if jpg and jpg[:2] == b"\xff\xd8":
        with open(path, "wb") as f:
            f.write(jpg)
        _prune(folder)
        return {"ok": True, "path": path, "size_bytes": len(jpg), "source": "live camera stream",
                "note": "camera is mounted high; it cannot see the floor close to the car"}
    cmd = [_ffmpeg(), "-loglevel", "error", "-y", "-f", "v4l2", "-input_format", "mjpeg",
           "-video_size", "%dx%d" % (width, height), "-i", _camera(),
           # skip the first frames: auto-exposure needs ~0.5 s to settle
           "-vf", r"select=gte(n\,15)", "-frames:v", "1", "-q:v", "3", path]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.TimeoutExpired) as e:
        return {"ok": False, "error": "camera capture failed: %s" % e}
    if r.returncode != 0 or not os.path.exists(path):
        return {"ok": False, "error": "camera capture failed: %s" % (r.stderr.strip()[-300:] or "no output")}
    _prune(folder)
    return {"ok": True, "path": path, "size_bytes": os.path.getsize(path),
            "note": "camera is mounted high and may be tilted; it cannot see the floor close to the car"}


def say(text):
    """Speak text through the robot's speaker (offline Piper voice). Returns when finished."""
    text = (text or "").strip()
    if not text:
        return {"ok": False, "error": "nothing to say"}
    if os.environ.get("ROBOT_SPEAK_VIA") == "pi_io":           # running on the Mac: robot speaker via tunnel
        wav = _http(MAC_AI_URL + "/tts", json.dumps({"text": text}).encode(), timeout=15)
        if not wav or wav[:4] != b"RIFF":
            return {"ok": False, "error": "Mac speech service did not return audio"}
        out = _http(PI_IO_URL + "/play?envelope=1", wav, "audio/wav", timeout=60)
        return {"ok": bool(out), "spoken": text}
    fd, wav_path = tempfile.mkstemp(suffix=".wav")
    os.close(fd)
    try:
        wav = _http(MAC_AI_URL + "/tts", json.dumps({"text": text}).encode(), timeout=15)   # Mac: keeps the Pi cool
        if wav and wav[:4] == b"RIFF":
            with open(wav_path, "wb") as f:
                f.write(wav)
        else:                                                     # Mac unreachable: synthesise here
            voice = os.path.expanduser(os.environ.get(
                "ROBOT_PIPER_VOICE", "~/.hermes/cache/piper-voices/en_US-lessac-medium.onnx"))
            try:
                from piper import PiperVoice
            except ImportError:
                return {"ok": False, "error": "piper-tts is not installed and the Mac speech service is offline"}
            with wave.open(wav_path, "wb") as w:
                PiperVoice.load(voice).synthesize_wav(text, w)
        envelope, step = speech_envelope(wav_path)
        with _audio_lock:
            # mouth animation on the face screen; Bluetooth plays ~0.3 s behind pw-play's start
            _pet("POST", "/talk", {"envelope": envelope, "step": step, "delay": 0.3}, timeout=0.5)
            r = subprocess.run(["pw-play", wav_path], capture_output=True, text=True, timeout=120, env=_env())
        if r.returncode != 0:
            return {"ok": False, "error": "speaker playback failed: %s" % r.stderr.strip()[-300:]}
        with wave.open(wav_path) as w:
            seconds = w.getnframes() / float(w.getframerate())
        return {"ok": True, "spoken": text, "seconds": round(seconds, 1)}
    except (OSError, subprocess.TimeoutExpired) as e:
        return {"ok": False, "error": "speaking failed: %s" % e}
    finally:
        os.remove(wav_path)


def speech_envelope(wav_path, step=0.05):
    """Loudness (0..1) every `step` seconds of a 16-bit WAV: drives the face's mouth."""
    with wave.open(wav_path) as w:
        rate, data = w.getframerate(), w.readframes(w.getnframes())
    samples = array.array("h", data)
    n = max(1, int(rate * step))
    env = []
    for i in range(0, len(samples), n):
        chunk = samples[i:i + n]
        env.append(math.sqrt(sum(s * s for s in chunk) / len(chunk)) if chunk else 0.0)
    peak = max(env) if env else 0.0
    return [round(min(1.0, e / peak * 1.2), 2) if peak else 0.0 for e in env], step


def _load_whisper():
    global _whisper
    with _whisper_lock:
        if _whisper is None:
            # China: huggingface.co is blocked; the mirror works without the xet backend
            os.environ.setdefault("HF_ENDPOINT", "https://hf-mirror.com")
            os.environ.setdefault("HF_HUB_DISABLE_XET", "1")
            from faster_whisper import WhisperModel
            _whisper = WhisperModel(os.environ.get("ROBOT_WHISPER_MODEL", "base"),
                                    device="cpu", compute_type="int8")
        return _whisper


def stt(pcm_int16):
    """Speech to text for 16 kHz mono int16 samples (numpy array or bytes).
    Uses the Mac (Whisper large-v3-turbo on its GPU) and falls back to the Pi's small model."""
    raw = pcm_int16 if isinstance(pcm_int16, bytes) else pcm_int16.tobytes()
    out = _http(MAC_AI_URL + "/stt", raw, "application/octet-stream", timeout=20)
    if out:
        try:
            r = json.loads(out)
            if r.get("ok"):
                return r["text"], "mac"
        except ValueError:
            pass
    import numpy as np
    audio = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
    segments, _ = _load_whisper().transcribe(audio, language="en", vad_filter=True, beam_size=1,
                                             initial_prompt=STT_HINT)
    return " ".join(s.text.strip() for s in segments).strip(), "pi"


def describe_scene(question="Describe what you see in one short spoken sentence."):
    """Camera frame -> small vision model on the Mac (Ollama via the tunnel). Returns text or None."""
    import base64
    jpg = _http(PI_IO_URL + "/frame.jpg")
    if not jpg:
        return None
    body = {"model": os.environ.get("ROBOT_SCENE_MODEL", "qwen3.5:2b"), "stream": False, "think": False,
            "keep_alive": "24h", "options": {"num_ctx": 4096, "temperature": 0.2},
            "messages": [{"role": "user", "content": question, "images": [base64.b64encode(jpg).decode()]}]}
    out = _http(os.environ.get("OLLAMA_URL", "http://127.0.0.1:11434") + "/api/chat",
                json.dumps(body).encode(), timeout=30)
    try:
        return json.loads(out)["message"]["content"].strip() if out else None
    except (ValueError, KeyError):
        return None


def level_dbfs(wav_path):
    """Loudness of a 16-bit mono WAV in dBFS (0 = maximum, about -60 or lower = silence)."""
    with wave.open(wav_path) as w:
        samples = array.array("h", w.readframes(w.getnframes()))
    if not samples:
        return -120.0
    rms = math.sqrt(sum(s * s for s in samples) / len(samples))
    return round(20 * math.log10(max(rms, 1) / 32768.0), 1)


def listen(seconds=5.0):
    """Record from the robot's microphone and transcribe it offline. Returns {"ok", "text", ...}."""
    seconds = max(1.0, min(float(seconds), 30.0))
    fd, wav_path = tempfile.mkstemp(suffix=".wav")
    os.close(fd)
    try:
        with _audio_lock:
            # pw-record has no duration option: run it for `seconds`, then stop it
            p = subprocess.Popen(["pw-record", "--rate", "16000", "--channels", "1", "--format", "s16", wav_path],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True, env=_env())
            time.sleep(seconds)
            p.terminate()
            _, err = p.communicate(timeout=5)
        if not os.path.exists(wav_path) or os.path.getsize(wav_path) < 1000:
            return {"ok": False, "error": "microphone recording failed: %s" % (err or "").strip()[-300:]}
        loudness = level_dbfs(wav_path)
        with wave.open(wav_path) as w:
            text, where = stt(w.readframes(w.getnframes()))
        return {"ok": True, "text": text, "heard_speech": bool(text), "seconds": seconds, "level_dbfs": loudness,
                "transcribed_on": where}
    except (OSError, subprocess.TimeoutExpired) as e:
        return {"ok": False, "error": "listening failed: %s" % e}
    finally:
        if os.path.exists(wav_path):
            os.remove(wav_path)
