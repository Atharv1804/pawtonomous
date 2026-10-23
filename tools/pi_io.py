#!/usr/bin/env python3
"""Pi I/O server: the Pi's camera and speaker for the brain running on the Mac.

Deliberately light, because the Pi overheats: camera frames are passed through as the
camera's own JPEGs (v4l2-ctl MJPEG, never decoded here), and sound is only played.

HTTP on 127.0.0.1:8767 (reached from the Mac through the SSH tunnel):
  GET  /health
  GET  /frame.jpg          newest camera frame (JPEG)
  GET  /stream             multipart MJPEG stream (the Mac brain reads this)
  GET  /battery            UPS HAT battery status (robot_power.py)
  POST /play               body = WAV bytes; plays on the speaker, returns when finished
                           (?envelope=1 also drives the face's mouth via the pet brain)
"""

import glob
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_media  # noqa: E402  (_env, _pet, speech_envelope, _audio_lock)
import robot_power  # noqa: E402

PORT = int(os.environ.get("PI_IO_PORT", "8767"))
WIDTH, HEIGHT, FPS = 640, 480, 15
latest = {"jpg": None, "t": 0.0, "n": 0}
cond = threading.Condition()


def camera_loop():
    while True:
        found = sorted(glob.glob("/dev/v4l/by-id/*video-index0"))
        dev = os.environ.get("ROBOT_CAMERA") or (found[0] if found else "/dev/video0")
        subprocess.run(["v4l2-ctl", "-d", dev, "--set-fmt-video=width=%d,height=%d,pixelformat=MJPG" % (WIDTH, HEIGHT),
                        "--set-parm=%d" % FPS, "-c", "exposure_dynamic_framerate=0"], capture_output=True)
        p = subprocess.Popen(["v4l2-ctl", "-d", dev, "--stream-mmap", "--stream-to=-"],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        buf = b""
        while True:
            chunk = p.stdout.read1(65536)
            if not chunk:
                break
            buf += chunk
            # frames arrive back to back: split where one JPEG ends (FFD9) and the next starts (FFD8)
            while True:
                i = buf.find(b"\xff\xd9\xff\xd8")
                if i < 0:
                    break
                with cond:
                    latest.update(jpg=buf[:i + 2], t=time.time(), n=latest["n"] + 1)
                    cond.notify_all()
                buf = buf[i + 2:]
            if len(buf) > 4_000_000:            # garbage without frame boundaries: drop it
                buf = b""
        p.kill()
        time.sleep(2)                           # camera unplugged: retry


def recompress(jpg, quality):
    """Smaller JPEG (same size in pixels) for the WiFi link; ~3 ms on the Pi 5."""
    try:
        import cv2
        import numpy as np
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        ok, out = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return out.tobytes() if ok else jpg
    except Exception:                           # no OpenCV / bad frame: send the original
        return jpg


def play(wav_bytes, envelope=False):
    fd, path = tempfile.mkstemp(suffix=".wav")
    with os.fdopen(fd, "wb") as f:
        f.write(wav_bytes)
    try:
        with robot_media._audio_lock:
            if envelope:
                env, step = robot_media.speech_envelope(path)
                robot_media._pet("POST", "/talk", {"envelope": env, "step": step, "delay": 0.3}, timeout=0.5)
            r = subprocess.run(["pw-play", path], capture_output=True, text=True, timeout=120, env=robot_media._env())
        return r.returncode == 0, r.stderr.strip()[-200:]
    finally:
        os.remove(path)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        if not isinstance(body, bytes):
            body = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/health":
            return self._send(200, {"ok": True, "frames": latest["n"], "age_s": round(time.time() - latest["t"], 2)})
        if path == "/battery":
            return self._send(200, robot_power.battery_status())
        if path == "/frame.jpg":
            if latest["jpg"] is None or time.time() - latest["t"] > 3:
                return self._send(503, {"ok": False, "error": "no fresh camera frame"})
            return self._send(200, latest["jpg"], "image/jpeg")
        if path == "/stream":
            # ?fps=8&q=60: fewer, re-compressed frames for the Mac over slow WiFi (fresh, not queued);
            # no parameters = every camera frame untouched (the robot's own screen).
            q = parse_qs(urlparse(self.path).query)
            fps = float(q.get("fps", ["0"])[0] or 0)
            quality = int(q.get("q", ["0"])[0] or 0)
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last, sent = -1, 0.0
            try:
                while True:
                    if fps > 0:
                        time.sleep(max(0.0, 1.0 / fps - (time.time() - sent)))
                    with cond:
                        cond.wait_for(lambda: latest["n"] != last, timeout=5)
                        jpg, last = latest["jpg"], latest["n"]
                    sent = time.time()
                    if jpg and quality:
                        jpg = recompress(jpg, quality)
                    if jpg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg))
                        self.wfile.write(jpg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
        return self._send(404, {"ok": False, "error": "not found"})

    def do_POST(self):
        url = urlparse(self.path)
        if url.path == "/play":
            n = int(self.headers.get("Content-Length") or 0)
            if not 44 < n < 20_000_000:
                return self._send(400, {"ok": False, "error": "send WAV bytes"})
            data = self.rfile.read(n)
            ok, err = play(data, parse_qs(url.query).get("envelope", ["0"])[0] == "1")
            return self._send(200 if ok else 500, {"ok": ok, "error": err})
        return self._send(404, {"ok": False, "error": "not found"})


def main():
    threading.Thread(target=camera_loop, daemon=True).start()
    print(time.strftime("%H:%M:%S"), "pi_io on http://127.0.0.1:%d" % PORT, flush=True)
    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()
