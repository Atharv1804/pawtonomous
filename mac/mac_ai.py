#!/usr/bin/env python3
"""Mac AI service: heavy speech work for the robot, so the Pi stays cool.

  POST /stt   body = 16 kHz mono int16 PCM (raw)  -> {"text", "seconds"}
              Whisper large-v3-turbo on the Apple GPU (mlx-whisper): accurate and fast.
  POST /tts   {"text": "..."}                     -> WAV bytes (Piper, same voice as on the Pi)
  GET  /health

Listens on 127.0.0.1:8768 on the Mac; the Pi reaches it through the SSH tunnel
(-R 8768), so it is never exposed to the network.
Run by the LaunchAgent com.atharv.robot-ai (mac/robot-ai.plist).
"""

import io
import json
import os
import threading
import time
import wave
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

PORT = int(os.environ.get("MAC_AI_PORT", "8768"))
STT_MODEL = os.environ.get("MAC_STT_MODEL", "mlx-community/whisper-large-v3-turbo")
VOICE = os.path.expanduser(os.environ.get("MAC_PIPER_VOICE", "~/robot_ws/models/en_US-lessac-medium.onnx"))
SCENE_MODEL = os.environ.get("ROBOT_SCENE_MODEL", "qwen3.5:2b")
STT_HINT = "Hey Jarvis. Talking to Jarvis, a robot car with distance sensors, a camera, a microphone and a speaker."
_lock = threading.Lock()                 # one GPU transcription at a time
_voice = None


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


def transcribe(pcm):
    import mlx_whisper
    audio = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
    with _lock:
        r = mlx_whisper.transcribe(audio, path_or_hf_repo=STT_MODEL, language="en", initial_prompt=STT_HINT,
                                   condition_on_previous_text=False, verbose=None)
    return (r.get("text") or "").strip()


def synthesize(text):
    global _voice
    from piper import PiperVoice
    if _voice is None:
        _voice = PiperVoice.load(VOICE)
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        _voice.synthesize_wav(text, w)
    return buf.getvalue()


class H(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        if not isinstance(body, bytes):
            body = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self, limit):
        n = int(self.headers.get("Content-Length") or 0)
        return self.rfile.read(n) if 0 < n <= limit else b""

    def do_GET(self):
        if self.path == "/health":
            return self._send(200, {"ok": True, "stt_model": STT_MODEL})
        return self._send(404, {"ok": False})

    def do_POST(self):
        t = time.time()
        try:
            if self.path == "/stt":
                pcm = self._body(16000 * 2 * 60)
                if len(pcm) < 3200:
                    return self._send(400, {"ok": False, "error": "send >= 0.1 s of 16 kHz int16 mono PCM"})
                text = transcribe(pcm)
                log("stt %.1fs audio in %.2fs: %r" % (len(pcm) / 32000, time.time() - t, text))
                return self._send(200, {"ok": True, "text": text, "seconds": round(time.time() - t, 2)})
            if self.path == "/tts":
                text = str(json.loads(self._body(20000) or b"{}").get("text", "")).strip()
                if not text:
                    return self._send(400, {"ok": False, "error": "no text"})
                wav = synthesize(text)
                log("tts %d chars in %.2fs" % (len(text), time.time() - t))
                return self._send(200, wav, "audio/wav")
        except Exception as e:                        # report, never crash the server
            log("error %s: %r" % (self.path, e))
            return self._send(500, {"ok": False, "error": str(e)})
        return self._send(404, {"ok": False})


def keep_warm():
    """Tiny transcription every minute so macOS keeps the Whisper weights in RAM (a swapped-out
    model made one request take 8 s instead of 0.5 s while memory was tight)."""
    while True:
        try:
            transcribe(b"\x00\x00" * 8000)
        except Exception as e:
            log("warm-up failed: %r" % e)
        try:                                   # also keep the camera scene model (Ollama) paged in
            import urllib.request
            body = json.dumps({"model": SCENE_MODEL, "prompt": "hi", "stream": False, "keep_alive": "24h",
                               "options": {"num_predict": 1}}).encode()
            urllib.request.urlopen(urllib.request.Request("http://127.0.0.1:11434/api/generate", body,
                                                          {"Content-Type": "application/json"}), timeout=30).read()
        except Exception as e:
            log("scene warm-up failed: %r" % e)
        time.sleep(60)


def main():
    threading.Thread(target=keep_warm, daemon=True).start()
    log("mac_ai on http://127.0.0.1:%d (stt %s)" % (PORT, STT_MODEL))
    ThreadingHTTPServer(("127.0.0.1", PORT), H).serve_forever()


if __name__ == "__main__":
    main()
