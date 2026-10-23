#!/usr/bin/env python3
"""Pet brain: the robot's fast local loop (no cloud, no waiting).

  camera -> vision (faces, who they are, people) ~8-10 Hz
         -> controller (look_at / follow / go_to / find / wander) 10 Hz
         -> Body API /drive or /act explore (ROS safety reflex still between us and the motors)
  + animated face for the screen (http://127.0.0.1:8766/face), greetings, mood.

Hermes (via MCP tools) and the voice loop send it high-level commands over HTTP; it carries
them out on its own and can be interrupted any time (new command or /stop).

HTTP on 127.0.0.1:8766 only (not reachable from the network):
  GET  /health /state /faces /events?since=N /frame.jpg[?annotated=1] /face /display
  POST /command {"mode","target","timeout"}   POST /stop   POST /autonomy {"enabled"}
  POST /learn {"name","image_path"?}   POST /forget {"name"}
  POST /talk {"envelope":[0..1,...],"step":0.05,"delay":0.3}   POST /expression {"name","seconds"}
  POST /vision_pause {"seconds"}   (0 = resume; the voice loop frees the CPU for Whisper)
"""

import base64
import difflib
import hmac
import json
import random
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))           # tools/ (robot_media)

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import control  # noqa: E402
import vision  # noqa: E402

PORT = int(os.environ.get("PET_PORT", "8766"))
BODY = os.environ.get("ROBOT_API_URL", "http://127.0.0.1:8765").rstrip("/")
BODY_TOKEN = os.environ.get("ROBOT_API_TOKEN", "")
PI_IO = os.environ.get("PI_IO_URL", "http://127.0.0.1:8767").rstrip("/")         # Pi speaker (pi_io.py)
MAC_AI = os.environ.get("MAC_AI_URL", "http://127.0.0.1:8768").rstrip("/")       # text -> speech
GREET_EVERY_S = 15 * 60       # say hi to the same person at most this often
GREET_AFTER_ABSENT_S = 5 * 60  # ...and only if they were away this long
IDLE_WANDER_AFTER_S = 3 * 60   # autonomy: wander a bit after this long idle with nobody around
SLEEPY_AFTER_S = 5 * 60
OLLAMA = os.environ.get("OLLAMA_URL", "http://127.0.0.1:11434").rstrip("/")
CURIOUS_MODEL = os.environ.get("PET_CURIOUS_MODEL", "qwen3.5:2b")   # small vision model on the Mac, ~1 s
CURIOUS_PROMPT = (
    "You are the eyes of a small robot car. This photo is what it sees straight ahead. Reply with JSON only:\n"
    '{"path_clear": true or false (is the floor straight ahead free of obstacles, furniture, walls or '
    'people for about one metre?), "ahead": "far" or "medium" or "near" (how close is the nearest '
    'obstacle, wall or corner straight ahead?), "turn": "left" or "right" (which side looks more open), '
    '"interesting": "a short curious, friendly comment of at most 8 words about the most interesting OBJECT '
    'or place you see, for example Ooh, a big shiny ball! If the most notable thing is a person, just greet '
    'them warmly (for example Hi there, friend!) and never describe their body, clothes or looks. Empty '
    'string if nothing stands out"}')
REMARK_EVERY_S = 15.0
GREETING = "Hi everyone, I'm Jarvis, Atharv's pet! It's really nice to meet you all!"
GOODBYE = "Bye everyone! Thank you for meeting me!"
DEMO_LINES = [GOODBYE, GREETING, "Yummy! Thank you!", "Say cheese!", "Got it! Sending it to Atharv's Telegram.",
              "Following you!", "Okay, stopping."]
TELEGRAM_FILE = os.path.expanduser("~/robot_ws/.telegram_bot")
REMOTE_PORT = int(os.environ.get("PET_REMOTE_PORT", "8770"))      # phone remote, reachable on the WiFi
REMOTE_TOKEN_FILE = os.path.expanduser("~/robot_ws/.remote_token")  # secret part of the remote's URL   # TELEGRAM_BOT_TOKEN=..., TELEGRAM_ALLOWED_USERS=...
VISION_HZ = float(os.environ.get("PET_VISION_HZ", "8"))   # 8/s tracks people fine and leaves CPU for voice
cv2.setNumThreads(int(os.environ.get("PET_CV_THREADS", "2")))


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


class Body:
    """Tiny client for the ROS Body API (agent_bridge)."""

    def _call(self, method, path, payload=None, timeout=2.0):
        headers = {"Content-Type": "application/json"}
        if BODY_TOKEN:
            headers["Authorization"] = "Bearer " + BODY_TOKEN
        data = json.dumps(payload).encode() if payload is not None else None
        try:
            with urllib.request.urlopen(urllib.request.Request(BODY + path, data, headers, method=method),
                                        timeout=timeout) as r:
                return json.loads(r.read() or b"{}")
        except (urllib.error.URLError, OSError, ValueError):
            return None

    def state(self):
        return self._call("GET", "/state") or {}

    def drive(self, linear, angular):
        return self._call("POST", "/drive", {"linear": round(linear, 3), "angular": round(angular, 3), "agent": "pet"})

    def explore(self, seconds):
        return self._call("POST", "/act", {"behavior": "explore", "seconds": seconds, "agent": "pet", "wait": False})

    def stop(self):
        return self._call("POST", "/stop", {})


class Brain:
    def __init__(self):
        self.camera = vision.Camera()
        self.db = vision.FaceDB()
        self.perception = vision.Perception(self.db)
        self.body = Body()
        self.ctrl = control.Controller()
        self.lock = threading.RLock()
        self.dets, self.boxes, self.dets_stamp, self.vision_hz = [], [], 0.0, 0.0
        self.last_seen = {}                  # name -> {"t", "bearing_deg", "kind"}
        self.last_greet = {}
        self.events, self.event_n = deque(maxlen=200), 0
        self.autonomy = os.environ.get("PET_AUTONOMY", "1") != "0"
        self.exploring_until = 0.0
        self.idle_since = time.time()
        self.anyone_seen = time.time()
        self.ranges, self.sensors_ok = {}, False
        # display state
        self.expr, self.expr_until = "neutral", 0.0
        self.base_expr = "neutral"
        self.talk = {"envelope": [], "step": 0.05, "start": 0.0}
        self.gaze = 0.0
        self.vision_paused_until = 0.0       # the voice loop pauses vision while Whisper needs the CPU
        self.voice_claim = {"who": "", "until": 0.0}   # Pi mic and Mac mic: only one answers a "hey jarvis"
        self.voice_status = {"text": "", "until": 0.0}   # set by the voice loops: listening / thinking / ...
        self.curious_cmd = (0.0, 0.0, 0.0)               # (linear, angular, until) from the curious loop
        self.curious_status = ""
        self.last_remark = 0.0
        self.cam_pip = {"until": 0.0, "caption": ""}     # mini camera feed on the face screen
        self.last_pip = 0.0
        # demo / pet tricks shown on the face screen
        self.anim = {"name": "", "until": 0.0}           # paw_wave | spin_eyes | countdown | flash
        self.view = "face"                               # "face" or "camera" (full-screen annotated feed)
        self.photo = {"until": 0.0, "id": 0}              # selfie on screen
        self.selfie_jpg = b""
        self.selfie_cancel = threading.Event()            # BACK on the screen / STOP: no photo
        self.frame_wh = (640, 480)
        self.ptt = {"on": False, "since": 0.0}            # push-to-talk from the phone remote
        self.last_heard = {"text": "", "t": 0.0}

    # ---------- events / speech / display ----------
    def event(self, ev):
        with self.lock:
            self.event_n += 1
            self.events.append(dict(ev, n=self.event_n, t=round(time.time(), 1)))
        log("event %s" % ev)

    def say(self, text):
        """Speak on the robot: Mac synthesises (mac_ai /tts), the Pi only plays (pi_io /play)."""
        def run():
            try:
                req = urllib.request.Request(MAC_AI + "/tts", json.dumps({"text": text}).encode(),
                                             {"Content-Type": "application/json"})
                wav = urllib.request.urlopen(req, timeout=15).read()
                urllib.request.urlopen(urllib.request.Request(PI_IO + "/play?envelope=1", wav,
                                                              {"Content-Type": "audio/wav"}), timeout=60).read()
            except OSError as e:
                log("say failed: %s" % e)
        threading.Thread(target=run, daemon=True).start()

    def show_camera(self, seconds=5.0, caption=""):
        """Show the small camera picture-in-picture on the face screen for a few seconds."""
        with self.lock:
            self.cam_pip = {"until": time.time() + seconds, "caption": caption[:50]}
            self.last_pip = time.time()

    def set_expression(self, name, seconds=3.0):
        with self.lock:
            self.expr, self.expr_until = name, time.time() + seconds

    def status_text(self, now):
        """One line for the bottom of the face: the voice stage if active, else the current task."""
        if now < self.voice_status["until"] and self.voice_status["text"]:
            return self.voice_status["text"]
        t = self.talk
        if t["envelope"] and t["start"] <= now <= t["start"] + len(t["envelope"]) * t["step"]:
            return "Speaking"
        c = self.ctrl.cmd
        who = (c.target or "").strip()
        who = "you" if who.lower() in control.ANY_TARGETS else who.capitalize()
        if c.mode == "find":
            return "Looking for %s - %s" % (who, "spinning to look around" if c.phase == "spin" else "exploring")
        if c.mode == "go_to":
            return "Going to %s" % who
        if c.mode == "follow":
            return "Following %s" % who
        if c.mode == "follow_straight":
            return "Following %s" % who
        if c.mode == "look_at":
            return "Turning to look at %s" % who
        if c.mode == "wander":
            return "Exploring on my own"
        if c.mode == "curious":
            return self.curious_status or "Exploring"
        dets = self.dets if now - self.dets_stamp < 1 else []
        faces = [d for d in dets if d.kind == "face"]
        owner = any(d.name == vision.OWNER for d in dets)
        others = sum(1 for d in faces if d.name != vision.OWNER)   # only the owner is ever named
        if owner:
            return "I can see %s%s" % (vision.OWNER.capitalize(),
                                       " and %d other %s" % (others, "person" if others == 1 else "people") if others else "")
        if dets:
            return "I can see someone"
        return "Say \"Hey Jarvis\""

    def display(self):
        now = time.time()
        with self.lock:
            expr = self.expr if now < self.expr_until else self.base_expr
            dets = self.dets if now - self.dets_stamp < 1.0 else []
            return {"expression": expr, "gaze_x": round(self.gaze, 2), "now": now,
                    "talk": self.talk, "mode": self.ctrl.cmd.mode, "status": self.status_text(now),
                    "camera": self.cam_pip if now < self.cam_pip["until"] else None,
                    "anim": self.anim if now < self.anim["until"] else None,
                    "view": self.view,
                    "photo": self.photo["id"] if now < self.photo["until"] else 0,
                    "friends": self.perception.friends.count(),
                    "faces": sum(1 for d in dets if d.kind == "face"),
                    "owner_here": any(d.name == vision.OWNER for d in dets),
                    "boxes": self.screen_boxes(now)}

    def screen_boxes(self, now):
        """Face boxes for the robot screen, which shows the Pi's own camera feed and draws these on top
        (0..1 coordinates). Only the owner is named; everyone else is "?"."""
        if now - self.dets_stamp > 1.0:
            return []
        w, h = self.frame_wh
        out = []
        for kind, name, x1, y1, x2, y2 in self.boxes:
            owner = name == vision.OWNER
            if kind != "face" and not owner:
                continue
            out.append({"kind": kind, "label": name.capitalize() if owner else "?", "owner": owner,
                        "x": round(x1 / w, 3), "y": round(y1 / h, 3),
                        "w": round((x2 - x1) / w, 3), "h": round((y2 - y1) / h, 3)})
        return out

    # ---------- commands ----------
    def resolve_name(self, target):
        """Speech-to-text often misspells names ("Athorv"): map to the closest learned face."""
        t = (target or "").strip().lower()
        known = list(self.db.names())
        if not t or t in control.ANY_TARGETS or t in known:
            return t
        close = difflib.get_close_matches(t, known, n=1, cutoff=0.6)
        if close:
            log("name %r -> %r (closest learned face)" % (t, close[0]))
            return close[0]
        return t

    def command(self, mode, target="", timeout=None):
        target = self.resolve_name(target)
        now = time.time()
        with self.lock:
            prev = self.ctrl.cmd.mode
            self.ctrl.set(mode, target, timeout, now)
        if prev != "idle" or mode == "idle":
            self.body.drive(0.0, 0.0)            # cancel what was running (also stops explore)
        self.exploring_until = 0.0
        self.event({"mode": mode, "target": target or "", "result": "started"})
        return {"ok": True, "mode": mode, "target": target or ""}

    def stop(self):
        if self.anim["name"] == "countdown":
            self.cancel_photo()                          # STOP during "Say cheese" = no photo
        with self.lock:
            self.ctrl.stop(time.time())
        self.exploring_until = 0.0
        self.body.stop()
        self.event({"mode": "idle", "result": "stopped"})
        return {"ok": True}

    def learn(self, name, image_path="", image_b64=""):
        if image_b64:
            img = cv2.imdecode(np.frombuffer(base64.b64decode(image_b64), np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                return {"ok": False, "error": "could not decode the photo"}
            source = "photo"
        elif image_path:
            img = cv2.imread(os.path.expanduser(image_path))
            if img is None:
                return {"ok": False, "error": "cannot read image %s" % image_path}
            source = "photo"
        else:
            img, stamp = self.camera.latest()
            if img is None or time.time() - stamp > 2:
                return {"ok": False, "error": "no fresh camera frame"}
            source = "camera"
        try:
            emb, nfaces = self.perception.learn_from_image(img)
            if emb is None:
                return {"ok": False, "error": "no face found in the %s; use a clear, front-facing photo" % source}
            if max(img.shape[:2]) > 1280:
                s = 1280 / max(img.shape[:2])
                img = cv2.resize(img, None, fx=s, fy=s)
            det = self.perception._photo_detector()
            found = self.perception.faces(img, det)
            extra = self.perception.augmented_embeddings(img, max(found, key=lambda f: f[2] * f[3])) if found else []
        except cv2.error as e:
            return {"ok": False, "error": "face detection failed on this image: %s" % str(e)[-160:]}
        try:
            for e in [emb] + extra[1:]:                       # extra[0] is the original again
                name, views = self.db.add(name, e)
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        note = "learned the largest of %d faces" % nfaces if nfaces > 1 else "learned"
        self.event({"learned_face": name, "views": views, "source": source})
        return {"ok": True, "name": name, "views": views, "faces_in_image": nfaces, "note": note}

    def state(self):
        now = time.time()
        with self.lock:
            c = self.ctrl.cmd
            dets = [d.__dict__ for d in self.dets] if now - self.dets_stamp < 1.0 else []
            return {
                "mode": c.mode, "target": c.target, "running_s": round(now - c.started, 1) if c.mode != "idle" else 0,
                "phase": c.phase, "autonomy": self.autonomy, "seeing": dets,
                "last_seen": {n: {"seconds_ago": round(now - v["t"], 1), "bearing_deg": v["bearing_deg"],
                                  "kind": v["kind"]} for n, v in self.last_seen.items()},
                "known_faces": sorted(self.db.names()), "friends_met": self.perception.friends.count(),
                "camera_fps": round(self.camera.fps, 1),
                "vision_hz": round(self.vision_hz, 1), "camera_error": self.camera.error,
                "ranges_m": self.ranges, "sensors_ok": self.sensors_ok,
                "note": "bearing_deg > 0 = to the right; distances are rough estimates from box size",
            }

    # ---------- loops ----------
    def vision_loop(self):
        n, t0, last_stamp, next_t = 0, time.time(), 0.0, 0.0
        while True:
            time.sleep(max(0.0, next_t - time.time()))
            next_t = time.time() + 1.0 / VISION_HZ
            if time.time() < self.vision_paused_until:
                continue
            img, stamp = self.camera.latest()
            if img is None or stamp == last_stamp:
                time.sleep(0.01)
                continue
            last_stamp = stamp
            try:
                dets, boxes = self.perception.process(img, time.time())
            except Exception as e:                       # never let one bad frame kill the loop
                log("vision error: %r" % e)
                time.sleep(0.5)
                continue
            now = time.time()
            with self.lock:
                self.dets, self.boxes, self.dets_stamp = dets, boxes, now
                self.frame_wh = (img.shape[1], img.shape[0])
                for d in dets:
                    if d.name != "unknown":
                        self._maybe_greet(d.name, now)
                        self.last_seen[d.name] = {"t": now, "bearing_deg": d.bearing_deg, "kind": d.kind}
                if dets:
                    self.anyone_seen = now
            n += 1
            if now - t0 >= 2:
                self.vision_hz, n, t0 = n / (now - t0), 0, now

    def _maybe_greet(self, name, now):
        prev = self.last_seen.get(name, {}).get("t", 0.0)
        if (self.autonomy and self.ctrl.cmd.mode in ("idle", "wander")
                and now - prev > GREET_AFTER_ABSENT_S and now - self.last_greet.get(name, 0.0) > GREET_EVERY_S):
            self.last_greet[name] = now
            self.set_expression("happy", 4)
            self.say("Hi %s!" % name.capitalize())
            self.event({"greeted": name})

    def control_loop(self):
        period, last = 0.1, time.time()
        while True:
            time.sleep(max(0.0, period - (time.time() - last)))
            now = time.time()
            dt, last = now - last, now
            st = self.body.state()
            self.ranges = st.get("ranges_m", {}) or {}
            self.sensors_ok = bool((st.get("behavior") or {}).get("sensors_ok"))
            with self.lock:
                dets = self.dets if now - self.dets_stamp < 1.0 else []
                act = self.ctrl.step(dets, self.ranges, now, dt)
                mode = self.ctrl.cmd.mode
                self.perception.people_every = 1 if mode == "follow_straight" else 3   # bodies every frame
                self._update_mood(dets, now)
            if act.say:
                self.say(act.say)
            if act.expression:
                self.set_expression(act.expression, 0.5 if act.expression in ("focused", "searching", "curious") else 3)
            if act.event:
                self.event(act.event)
            if act.finished:
                self.body.drive(0.0, 0.0)
                self.exploring_until = 0.0
                self.idle_since = now
                continue
            if mode == "idle":
                self._autonomy_tick(now)
                continue
            self.idle_since = now
            if mode == "curious":                          # driven by the vision-model loop
                lin, ang, until = self.curious_cmd
                self.exploring_until = 0.0
                self.body.drive(lin, ang) if now < until else self.body.drive(0.0, 0.0)
                continue
            if act.explore:
                if now > self.exploring_until:             # (re)start the obstacle-aware explore behaviour
                    self.body.explore(30)
                    self.exploring_until = now + 29
            else:
                self.exploring_until = 0.0
                self.body.drive(act.linear, act.angular)

    # ---------- demo / pet tricks (hard-coded: instant, no LLM) ----------
    def set_anim(self, name, seconds):
        with self.lock:
            self.anim = {"name": name, "until": time.time() + seconds, "start": time.time()}

    def _nudge(self, linear, seconds):
        """A small fixed movement (the control loop is idle during tricks)."""
        t = time.time()
        while time.time() - t < seconds:
            self.body.drive(linear, 0.0)
            time.sleep(0.1)
        self.body.drive(0.0, 0.0)

    def cancel_photo(self):
        """BACK during "Say cheese": no photo is taken; BACK on a shown photo closes it."""
        self.selfie_cancel.set()
        with self.lock:
            if self.anim["name"] in ("countdown", "flash"):
                self.anim = {"name": "", "until": 0.0}
            self.photo["until"] = 0.0
        return {"ok": True, "cancelled": True}

    def trick(self, name):
        """Run one trick in the background so the voice loop is free at once (and can say stop)."""
        if name == "cancel":
            return self.cancel_photo()
        if name == "snap":
            threading.Thread(target=self._trick_snap, daemon=True).start()
            return {"ok": True, "trick": name}
        if name in ("view_camera", "view_face"):
            self.view = "camera" if name == "view_camera" else "face"
            self.set_expression("happy", 1.5)
            return {"ok": True, "view": self.view}
        with self.lock:
            self.ctrl.stop(time.time())                    # tricks replace any running task
        self.body.drive(0.0, 0.0)
        threading.Thread(target=getattr(self, "_trick_" + name), daemon=True).start()
        self.event({"trick": name})
        return {"ok": True, "trick": name}

    def _trick_greet(self):
        self.view = "face"
        self.set_anim("paw_wave", 5.0)
        self.set_expression("happy", 5.5)
        self.say(GREETING)
        time.sleep(5.0)
        self.set_expression("neutral", 0.1)

    def _trick_bye(self):
        self.view = "face"
        self.set_anim("paw_wave", 3.5)
        with self.lock:
            self.anim["text"] = "Bye!"
        self.set_expression("happy", 4.0)
        self.say(GOODBYE)
        time.sleep(3.5)
        self.set_expression("neutral", 0.1)

    def _trick_sit(self):
        self.view = "face"
        self.set_expression("focused", 1.0)
        self._nudge(0.12, 0.35)                          # a little shuffle, then settle down
        time.sleep(0.3)
        self.set_expression("sleeping", 600)              # eyes closed until the next command

    def _trick_treat(self):
        self.view = "face"
        self.set_anim("spin_eyes", 3.0)
        self.set_expression("happy", 3.5)
        self.say("Yummy! Thank you!")
        time.sleep(3.0)
        self._nudge(0.15, 0.5)                           # happy little step forward
        self.set_expression("sleepy", 600)

    def _trick_selfie(self):
        self.view = "face"
        self.selfie_cancel.clear()
        self.set_anim("countdown", 3.8)                  # the screen shows the live camera + 3-2-1 + BACK
        self.say("Say cheese!")
        if self.selfie_cancel.wait(3.8):
            self.say("Okay, no photo.")
            self.event({"selfie": "cancelled"})
            return
        img, stamp = self._full_frame()
        if img is None:
            self.say("Oops, I can't see right now.")
            return
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        self.selfie_jpg = jpg.tobytes()
        folder = os.path.expanduser("~/robot_ws/captures")
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, "selfie_%s.jpg" % time.strftime("%Y%m%d_%H%M%S"))
        with open(path, "wb") as f:
            f.write(self.selfie_jpg)
        self.set_anim("flash", 0.4)
        with self.lock:
            self.photo = {"until": time.time() + 7.0, "id": self.photo["id"] + 1}
        self.set_expression("happy", 7.5)
        sent = self._telegram_photo(self.selfie_jpg, "Selfie from Jarvis 🐾")
        self.say("Got it! Sending it to Atharv's Telegram." if sent else "Got it! I saved our selfie.")
        self.event({"selfie": path, "telegram": sent})

    def _full_frame(self):
        """A full-quality picture straight from the Pi (the live stream to the Mac is compressed)."""
        try:
            data = urllib.request.urlopen(PI_IO + "/frame.jpg", timeout=3).read()
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                return img, time.time()
        except OSError as e:
            log("full frame failed, using the stream: %s" % e)
        return self.camera.latest()

    def _capture(self, prefix):
        img, stamp = self._full_frame()
        if img is None:
            return None, ""
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        folder = os.path.expanduser("~/robot_ws/captures")
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, "%s_%s.jpg" % (prefix, time.strftime("%Y%m%d_%H%M%S")))
        with open(path, "wb") as f:
            f.write(jpg.tobytes())
        return jpg.tobytes(), path

    def _trick_snap(self):
        """Camera-view button: instant photo -> flash -> shown briefly -> straight to Telegram (no countdown)."""
        jpg, path = self._capture("snap")
        if jpg is None:
            return
        self.selfie_jpg = jpg
        self.set_anim("flash", 0.3)
        with self.lock:
            self.photo = {"until": time.time() + 3.5, "id": self.photo["id"] + 1}
        sent = self._telegram_photo(jpg, "Snapshot from Jarvis")
        self.event({"snap": path, "telegram": sent})

    def _telegram_photo(self, jpg, caption):
        """Send a photo to the owner's Telegram straight from the Mac (its VPN reaches Telegram)."""
        try:
            cfg = dict(l.strip().split("=", 1) for l in open(TELEGRAM_FILE) if "=" in l)
            token, chat = cfg["TELEGRAM_BOT_TOKEN"], cfg["TELEGRAM_ALLOWED_USERS"].split(",")[0].strip()
        except (OSError, KeyError, ValueError):
            log("selfie: no Telegram settings in %s" % TELEGRAM_FILE)
            return False
        boundary = "pawtonomous%d" % int(time.time())
        parts = []
        for k, v in (("chat_id", chat), ("caption", caption)):
            parts.append(("--%s\r\nContent-Disposition: form-data; name=\"%s\"\r\n\r\n%s\r\n" % (boundary, k, v)).encode())
        parts.append(("--%s\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"selfie.jpg\"\r\n"
                      "Content-Type: image/jpeg\r\n\r\n" % boundary).encode() + jpg + b"\r\n")
        body = b"".join(parts) + ("--%s--\r\n" % boundary).encode()
        try:
            req = urllib.request.Request("https://api.telegram.org/bot%s/sendPhoto" % token, body,
                                         {"Content-Type": "multipart/form-data; boundary=%s" % boundary})
            with urllib.request.urlopen(req, timeout=20) as r:
                return bool(json.load(r).get("ok"))
        except (OSError, ValueError) as e:
            log("selfie: Telegram send failed: %s" % str(e)[:120])
            return False

    # ---------- curious exploring: camera + small vision model on the Mac ----------
    def _look(self):
        """Ask the vision model about the current camera frame. Returns a dict or None."""
        img, stamp = self.camera.latest()
        if img is None or time.time() - stamp > 2:
            return None
        small = cv2.resize(img, (448, 336))
        ok, jpg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 75])
        body = {"model": CURIOUS_MODEL, "stream": False, "think": False, "format": "json", "keep_alive": "24h",
                "options": {"num_ctx": 4096, "temperature": 0.4, "num_predict": 80},
                "messages": [{"role": "user", "content": CURIOUS_PROMPT,
                              "images": [base64.b64encode(jpg.tobytes()).decode()]}]}
        try:
            req = urllib.request.Request(OLLAMA + "/api/chat", json.dumps(body).encode(),
                                         {"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=8) as r:
                return json.loads(json.load(r)["message"]["content"])
        except (OSError, ValueError, KeyError) as e:
            log("curious: vision model failed: %r" % e)
            return None

    def _curious_do(self, lin, ang, secs, status, expression="curious"):
        with self.lock:
            self.curious_cmd = (lin, ang, time.time() + secs)
            self.curious_status = status
        self.set_expression(expression, secs + 0.5)

    def curious_loop(self):
        """Explore like a curious pet: short forward bursts while the camera says the way is clear, turn
        away from obstacles (mostly left: the right wheels cannot reverse), stop to admire and comment on
        interesting things, and pause at random to look around. Interrupted at once by any new command."""
        while True:
            if self.ctrl.cmd.mode != "curious":
                self.curious_cmd = (0.0, 0.0, 0.0)
                time.sleep(0.2)
                continue
            t0 = time.time()
            v = self._look()
            if self.ctrl.cmd.mode != "curious":
                continue
            if v is None:                                    # cannot see: stand still, try again
                log("curious: no camera picture or vision answer (camera: %s) - standing still"
                    % (self.camera.error or "ok"))
                self._curious_do(0.0, 0.0, 1.0, "Can't see right now - waiting", "sad")
                time.sleep(1.0)
                continue
            remark = str(v.get("interesting") or "").strip()
            ahead = str(v.get("ahead") or "medium").lower()
            sides = [d for d in ((self.ranges or {}).get("left"), (self.ranges or {}).get("right")) if d is not None]
            side_min = min(sides) if sides else 9.0
            # Careful near corners/walls: the heavy robot needs ~full power to move at all, so "slow" means
            # SHORT steps with a fresh look in between, not lower power.
            if ahead == "near" or side_min < 0.35:
                caution = "near"
            elif ahead == "medium" or side_min < 0.6:
                caution = "medium"
            else:
                caution = "far"
            clear = bool(v.get("path_clear")) and not (ahead == "near" and side_min < 0.6)
            turn = str(v.get("turn") or "left").lower()
            log("curious: clear=%s ahead=%s side=%.2f -> %s, turn=%s remark=%r (%.1fs)" % (
                clear, ahead, side_min, caution, turn, remark, time.time() - t0))
            now = time.time()
            if remark and now - self.last_remark > REMARK_EVERY_S and random.random() < 0.7:
                self.last_remark = now
                pause = random.uniform(2.5, 4.5)
                self._curious_do(0.0, 0.0, pause, remark[:60], "happy")
                self.show_camera(pause + 1.0, remark)       # show what it is talking about
                self.say(remark)
                time.sleep(pause)
            elif now - self.last_pip > 30:                    # every ~30 s: a peek at what it sees
                self.show_camera(5.0, "What I see")
                continue
            elif random.random() < 0.1:                      # stop now and then to look around
                pause = random.uniform(1.5, 3.5)
                self._curious_do(0.0, 0.0, pause, "Looking around...", "curious")
                time.sleep(pause)
            elif clear:
                speed, step, label = {"far": (0.22, random.uniform(1.2, 1.6), "Exploring - way looks clear"),
                                      "medium": (0.15, 0.6, "Something ahead - going slowly"),
                                      "near": (0.1, 0.3, "Careful - creeping forward")}[caution]
                self._curious_do(speed, 0.0, step, label)
                time.sleep(min(step, 1.0))
            else:
                # Obstacle: turn toward the open side. Left = tank spin; right = slower pivot.
                left = turn != "right" or random.random() < 0.3
                secs = random.uniform(0.8, 1.8) if caution == "far" else random.uniform(0.6, 1.0)
                self._curious_do(0.0, 1.0 if left else -1.0, secs,
                                 "Something's in the way - turning %s" % ("left" if left else "right"), "focused")
                time.sleep(secs)

    def _update_mood(self, dets, now):
        target = control.pick_target(dets, self.ctrl.cmd.target if self.ctrl.cmd.mode != "idle" else "any")
        if target is not None:
            self.gaze = max(-1.0, min(1.0, target.bearing_deg / (vision.HFOV_DEG / 2)))
        else:
            self.gaze *= 0.9
        if any(d.name != "unknown" for d in dets):
            self.base_expr = "happy"
        elif dets:
            self.base_expr = "curious"
        elif now - self.anyone_seen > SLEEPY_AFTER_S:
            self.base_expr = "sleepy"
        else:
            self.base_expr = "neutral"

    def _autonomy_tick(self, now):
        if (self.autonomy and self.sensors_ok and now - self.idle_since > IDLE_WANDER_AFTER_S
                and now - self.anyone_seen > 60):
            self.idle_since = now
            log("autonomy: bored, exploring curiously for 60 s")
            self.command("curious", timeout=60)

    def start(self):
        self.camera.start()
        threading.Thread(target=self.vision_loop, daemon=True).start()
        threading.Thread(target=self.control_loop, daemon=True).start()
        threading.Thread(target=self.curious_loop, daemon=True).start()


def make_handler(brain):
    face_html = open(os.path.join(HERE, "face.html"), "rb").read()

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
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                return json.loads(self.rfile.read(n) or b"{}") if 0 < n < 30_000_000 else {}
            except ValueError:
                return {}

        def do_GET(self):
            url = urlparse(self.path)
            q = parse_qs(url.query)
            if url.path == "/health":
                return self._send(200, {"ok": True})
            if url.path == "/state":
                return self._send(200, brain.state())
            if url.path == "/ptt":
                if brain.ptt["on"] and time.time() - brain.ptt["since"] > 25:   # forgotten: auto-off
                    brain.ptt = {"on": False, "since": time.time()}
                return self._send(200, {"on": brain.ptt["on"]})
            if url.path == "/display":
                return self._send(200, brain.display())
            if url.path == "/faces":
                return self._send(200, {"known_faces": brain.db.names()})
            if url.path == "/events":
                since = int(q.get("since", ["0"])[0])
                with brain.lock:
                    evs = [e for e in brain.events if e["n"] > since]
                return self._send(200, {"events": evs, "next": brain.event_n})
            if url.path == "/frame.jpg":
                img, stamp = brain.camera.latest()
                if img is None:
                    return self._send(503, {"ok": False, "error": brain.camera.error or "no frame yet"})
                if q.get("annotated", ["0"])[0] == "1":
                    with brain.lock:
                        img = vision.annotate(img, brain.boxes)
                w = int(q.get("w", ["0"])[0] or 0)           # the Mac shrinks it: the Pi only displays
                if 64 <= w < img.shape[1]:
                    img = cv2.resize(img, (w, int(img.shape[0] * w / img.shape[1])), interpolation=cv2.INTER_AREA)
                ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
                return self._send(200, jpg.tobytes(), "image/jpeg")
            if url.path == "/selfie.jpg":
                if not brain.selfie_jpg:
                    return self._send(404, {"ok": False, "error": "no selfie yet"})
                return self._send(200, brain.selfie_jpg, "image/jpeg")
            if url.path in ("/", "/face"):
                return self._send(200, face_html, "text/html; charset=utf-8")
            return self._send(404, {"ok": False, "error": "not found"})

        def do_POST(self):
            url, p = urlparse(self.path), self._json()
            try:
                if url.path == "/command":
                    return self._send(200, brain.command(str(p.get("mode", "idle")), str(p.get("target", "")),
                                                         p.get("timeout")))
                if url.path == "/stop":
                    return self._send(200, brain.stop())
                if url.path == "/autonomy":
                    brain.autonomy = bool(p.get("enabled", True))
                    return self._send(200, {"ok": True, "autonomy": brain.autonomy})
                if url.path == "/learn":
                    return self._send(200, brain.learn(str(p.get("name", "")), str(p.get("image_path", "")),
                                                       str(p.get("image_b64", ""))))
                if url.path == "/forget":
                    return self._send(200, {"ok": True, "existed": brain.db.forget(str(p.get("name", "")))})
                if url.path == "/talk":
                    env = [max(0.0, min(1.0, float(x))) for x in (p.get("envelope") or [])][:4000]
                    with brain.lock:
                        brain.talk = {"envelope": env, "step": float(p.get("step", 0.05)),
                                      "start": time.time() + float(p.get("delay", 0.3))}
                    return self._send(200, {"ok": True})
                if url.path == "/voice_claim":
                    who, secs, now = str(p.get("who", "?")), float(p.get("seconds", 8)), time.time()
                    with brain.lock:
                        c = brain.voice_claim
                        if secs <= 0:                      # release (only by the holder)
                            if c["who"] == who:
                                brain.voice_claim = {"who": "", "until": 0.0}
                            return self._send(200, {"ok": True})
                        if c["until"] > now and c["who"] != who:
                            return self._send(200, {"ok": False, "holder": c["who"]})
                        brain.voice_claim = {"who": who, "until": now + min(secs, 120.0)}
                    return self._send(200, {"ok": True})
                if url.path == "/vision_pause":
                    brain.vision_paused_until = time.time() + max(0.0, min(15.0, float(p.get("seconds", 5))))
                    return self._send(200, {"ok": True})
                if url.path == "/status":
                    with brain.lock:
                        brain.voice_status = {"text": str(p.get("text", ""))[:60],
                                              "until": time.time() + float(p.get("seconds", 10))}
                    return self._send(200, {"ok": True})
                if url.path == "/ptt":
                    on = bool(p.get("on"))
                    brain.ptt = {"on": on, "since": time.time()}
                    return self._send(200, {"ok": True, "on": on})
                if url.path == "/heard":
                    brain.last_heard = {"text": str(p.get("text", ""))[:200], "t": time.time()}
                    return self._send(200, {"ok": True})
                if url.path == "/demo":
                    action = str(p.get("action", ""))
                    if not hasattr(brain, "_trick_" + action) and action not in ("view_camera", "view_face", "cancel"):
                        return self._send(400, {"ok": False, "error": "unknown trick %r" % action})
                    return self._send(200, brain.trick(action))
                if url.path == "/view":                      # the dot on the screen toggles this
                    v = str(p.get("view", ""))
                    brain.view = v if v in ("face", "camera") else ("face" if brain.view == "camera" else "camera")
                    return self._send(200, {"ok": True, "view": brain.view})
                if url.path == "/show_camera":
                    brain.show_camera(float(p.get("seconds", 5)), str(p.get("caption", "")))
                    return self._send(200, {"ok": True})
                if url.path == "/expression":
                    brain.set_expression(str(p.get("name", "neutral")), float(p.get("seconds", 3)))
                    return self._send(200, {"ok": True})
            except (ValueError, TypeError) as e:
                return self._send(400, {"ok": False, "error": str(e)})
            except Exception as e:                       # never drop the connection: report the error
                log("POST %s failed: %r" % (url.path, e))
                return self._send(500, {"ok": False, "error": "%s: %s" % (type(e).__name__, e)})
            return self._send(404, {"ok": False, "error": "not found"})

    return H


def remote_token():
    """Secret for the phone remote URL (created once, kept in a git-ignored file)."""
    try:
        return open(REMOTE_TOKEN_FILE).read().strip()
    except OSError:
        import secrets
        tok = secrets.token_urlsafe(9)
        with open(REMOTE_TOKEN_FILE, "w") as f:
            f.write(tok)
        os.chmod(REMOTE_TOKEN_FILE, 0o600)
        return tok


REMOTE_ACTIONS = {   # button -> what the brain does
    "follow": ("command", {"mode": "follow_straight", "target": "any", "timeout": 180}),
    "explore": ("command", {"mode": "curious", "timeout": 120}),
    "stop": ("stop", None),
    "greet": ("demo", "greet"), "bye": ("demo", "bye"), "sit": ("demo", "sit"), "treat": ("demo", "treat"),
    "selfie": ("demo", "selfie"), "cancel": ("demo", "cancel"),
    "view_camera": ("demo", "view_camera"), "view_face": ("demo", "view_face"),
}


def make_remote_handler(brain, token):
    page = open(os.path.join(HERE, "remote.html"), "rb").read()
    prefix = "/r/" + token

    class R(BaseHTTPRequestHandler):
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

        def _authorized(self):
            path = urlparse(self.path).path
            return hmac.compare_digest(path[:len(prefix)], prefix) and path[len(prefix):len(prefix) + 1] in ("", "/")

        def do_GET(self):
            if not self._authorized():
                return self._send(404, {"ok": False})
            sub = urlparse(self.path).path[len(prefix):]
            if sub in ("", "/"):
                return self._send(200, page, "text/html; charset=utf-8")
            if sub == "/state":
                now = time.time()
                with brain.lock:
                    st = {"status": brain.status_text(now), "mode": brain.ctrl.cmd.mode, "view": brain.view,
                          "talk": brain.ptt["on"], "heard": brain.last_heard["text"],
                          "heard_ago": round(now - brain.last_heard["t"], 1) if brain.last_heard["t"] else None,
                          "friends": brain.perception.friends.count(),
                          "owner_here": any(d.name == vision.OWNER for d in brain.dets) if now - brain.dets_stamp < 1 else False}
                return self._send(200, st)
            return self._send(404, {"ok": False})

        def do_POST(self):
            if not self._authorized():
                return self._send(404, {"ok": False})
            n = int(self.headers.get("Content-Length") or 0)
            try:
                p = json.loads(self.rfile.read(n) or b"{}") if 0 < n < 4096 else {}
            except ValueError:
                p = {}
            sub = urlparse(self.path).path[len(prefix):]
            if sub == "/talk":
                on = bool(p.get("on"))
                brain.ptt = {"on": on, "since": time.time()}
                return self._send(200, {"ok": True, "talk": on})
            if sub == "/act":
                kind, arg = REMOTE_ACTIONS.get(str(p.get("action", "")), (None, None))
                if kind == "command":
                    return self._send(200, brain.command(arg["mode"], arg["target"] if "target" in arg else "", arg["timeout"]))
                if kind == "stop":
                    return self._send(200, brain.stop())
                if kind == "demo":
                    return self._send(200, brain.trick(arg))
                return self._send(400, {"ok": False, "error": "unknown action"})
            return self._send(404, {"ok": False})

    return R


def main():
    brain = Brain()
    brain.start()
    token = remote_token()
    remote = ThreadingHTTPServer(("0.0.0.0", REMOTE_PORT), make_remote_handler(brain, token))
    threading.Thread(target=remote.serve_forever, daemon=True).start()
    log("phone remote on port %d (secret URL in %s)" % (REMOTE_PORT, REMOTE_TOKEN_FILE))
    server = ThreadingHTTPServer(("127.0.0.1", PORT), make_handler(brain))
    log("pet brain on http://127.0.0.1:%d (known faces: %s)" % (PORT, ", ".join(brain.db.names()) or "none"))
    server.serve_forever()


if __name__ == "__main__":
    main()
