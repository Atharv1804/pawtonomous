"""Camera + fast local perception (OpenCV, CPU, no internet).

  - YuNet face detector        models/face_detection_yunet_2023mar.onnx     (~35 ms / 640x480 frame)
  - SFace face recogniser      models/face_recognition_sface_2021dec.onnx   (128-d embedding per face)
  - NanoDet COCO detector      models/object_detection_nanodet_2022nov.onnx (people seen from behind)
All from the OpenCV model zoo (downloaded from hf-mirror.com/opencv/...).

Known faces live in ~/robot_ws/faces/<name>.npz (several embeddings each, added by learn()).
"""

import glob
import math
import os
import re
import sys
import threading
import time

import cv2
import numpy as np

from control import Detection

MODELS = os.path.expanduser(os.environ.get("PET_MODELS", "~/robot_ws/models"))
FACES_DIR = os.path.expanduser(os.environ.get("PET_FACES", "~/robot_ws/faces"))
HFOV_DEG = float(os.environ.get("PET_CAMERA_HFOV", "70"))      # horizontal field of view (estimate)
ROTATE = os.environ.get("PET_CAMERA_ROTATE", "none")            # none | cw90 | ccw90 | 180
MATCH_THRESHOLD = float(os.environ.get("PET_FACE_MATCH", "0.42"))   # SFace cosine; stricter than OpenCV's 0.363
                                                                    # so other people are not called Atharv
AUTO_LEARN_SCORE = 0.55     # confidently recognised on the robot's own camera -> keep that view too
AUTO_LEARN_EVERY_S = 20.0   # ...at most this often per person (until MAX_VIEWS)
MAX_VIEWS = 30
TRACK_KEEP_S = 2.0          # keep a name on a face track this long after the last confident match
FACE_HEIGHT_M = 0.22        # chin-to-forehead box height, for a rough distance estimate
DARK_V = 60                 # HSV brightness below this = "very dark" (black clothes, hair)
PERSON_HEIGHT_M = 1.0       # visible part of a person (camera is high and close): very rough
PERSON_CLASS = 0
_ROTATIONS = {"cw90": cv2.ROTATE_90_CLOCKWISE, "ccw90": cv2.ROTATE_90_COUNTERCLOCKWISE, "180": cv2.ROTATE_180}


def safe_name(name):
    n = re.sub(r"[^a-z0-9_\-]+", "_", (name or "").strip().lower()).strip("_")
    if not n:
        raise ValueError("name must contain letters or digits")
    return n[:40]


CAMERA_URL = os.environ.get("PET_CAMERA_URL", "")   # e.g. http://127.0.0.1:18767/stream (Pi camera via tunnel)


class Camera(threading.Thread):
    """Keeps the newest frame: from the Pi's MJPEG stream (PET_CAMERA_URL, brain on the Mac)
    or a local USB camera. Reconnects if it disappears."""

    def __init__(self, width=640, height=480):
        super().__init__(daemon=True)
        self.size = (width, height)
        self.lock = threading.Lock()
        self.frame, self.stamp, self.fps, self.error = None, 0.0, 0.0, ""

    def _open(self):
        found = sorted(glob.glob("/dev/v4l/by-id/*video-index0"))
        cap = cv2.VideoCapture(os.environ.get("ROBOT_CAMERA") or (found[0] if found else 0), cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _store(self, f):
        if ROTATE in _ROTATIONS:
            f = cv2.rotate(f, _ROTATIONS[ROTATE])
        with self.lock:
            self.frame, self.stamp, self.error = f, time.time(), ""

    def _run_url(self):
        """Read a multipart MJPEG stream (pi_io.py /stream) and decode frames here (on the Mac)."""
        import urllib.request
        n, t0 = 0, time.time()
        while True:
            try:
                with urllib.request.urlopen(CAMERA_URL, timeout=10) as r:
                    buf = b""
                    while True:
                        chunk = r.read1(65536) if hasattr(r, "read1") else r.read(65536)
                        if not chunk:
                            break
                        buf += chunk
                        while True:
                            a = buf.find(b"\xff\xd8")
                            b = buf.find(b"\xff\xd9", a + 2) if a >= 0 else -1
                            if a < 0 or b < 0:
                                break
                            # the camera's JPEGs embed a thumbnail ending in FFD9: take the LAST FFD9
                            # before the next frame header instead of the first one
                            nxt = buf.find(b"--frame", b)
                            if nxt < 0:
                                break
                            end = buf.rfind(b"\xff\xd9", a, nxt)
                            f = cv2.imdecode(np.frombuffer(buf[a:end + 2], np.uint8), cv2.IMREAD_COLOR)
                            buf = buf[nxt:]
                            if f is not None:
                                self._store(f)
                                n += 1
                        if len(buf) > 4_000_000:
                            buf = b""
                        if time.time() - t0 >= 2:
                            self.fps, n, t0 = n / (time.time() - t0), 0, time.time()
            except OSError as e:
                self.error = "camera stream: %s" % e
            time.sleep(2)

    def run(self):
        if CAMERA_URL:
            return self._run_url()
        cap, n, t0 = None, 0, time.time()
        while True:
            if cap is None or not cap.isOpened():
                cap = self._open()
                if not cap.isOpened():
                    self.error = "camera not available"
                    time.sleep(2)
                    continue
            ok, f = cap.read()
            if not ok:
                self.error = "camera read failed"
                cap.release()
                cap = None
                time.sleep(1)
                continue
            self._store(f)
            n += 1
            if time.time() - t0 >= 2:
                self.fps, n, t0 = n / (time.time() - t0), 0, time.time()

    def latest(self):
        with self.lock:
            return (None, 0.0) if self.frame is None else (self.frame.copy(), self.stamp)


class FaceDB:
    def __init__(self, folder=FACES_DIR):
        self.folder = folder
        os.makedirs(folder, exist_ok=True)
        self.lock = threading.Lock()
        self.people = {}            # name -> (N,128) normalised embeddings
        self.reload()

    def reload(self):
        people = {}
        for p in glob.glob(os.path.join(self.folder, "*.npz")):
            if os.path.basename(p) == "friends.npz":         # the FriendBook's file, not a person
                continue
            try:
                people[os.path.basename(p)[:-4]] = np.load(p)["emb"]
            except (OSError, KeyError, ValueError):
                continue
        with self.lock:
            self.people = people

    def add(self, name, emb):
        name = safe_name(name)
        emb = emb.reshape(1, -1) / np.linalg.norm(emb)
        with self.lock:
            embs = np.vstack([self.people[name], emb]) if name in self.people else emb
            self.people[name] = embs[-MAX_VIEWS:]
            np.savez(os.path.join(self.folder, name + ".npz"), emb=self.people[name])
            return name, len(self.people[name])

    def forget(self, name):
        name = safe_name(name)
        with self.lock:
            existed = self.people.pop(name, None) is not None
        path = os.path.join(self.folder, name + ".npz")
        if os.path.exists(path):
            os.remove(path)
        return existed

    def names(self):
        with self.lock:
            return {n: len(e) for n, e in self.people.items()}

    def match(self, emb):
        emb = emb.reshape(-1) / np.linalg.norm(emb)
        best, best_score = "unknown", 0.0
        with self.lock:
            for name, embs in self.people.items():
                s = float(np.max(embs @ emb))
                if s > best_score:
                    best, best_score = name, s
        return (best, best_score) if best_score >= MATCH_THRESHOLD else ("unknown", best_score)


OWNER = os.environ.get("PET_OWNER", "atharv")
FRIEND_SAME = 0.45          # SFace cosine: same friend as one already in the list


class FriendBook:
    """People met WHILE the owner is around, remembered as 'friends' (embeddings only, no photos).
    Saved to faces/friends.npz so the list survives restarts."""

    def __init__(self, folder=FACES_DIR):
        self.path = os.path.join(folder, "friends.npz")
        self.lock = threading.Lock()
        try:
            self.embs = list(np.load(self.path)["emb"])
        except (OSError, KeyError, ValueError):
            self.embs = []
        self.met_today = 0

    def meet(self, emb):
        emb = emb.reshape(-1) / np.linalg.norm(emb)
        with self.lock:
            if self.embs and max(float(e @ emb) for e in self.embs) >= FRIEND_SAME:
                return False                                   # already a friend
            self.embs.append(emb)
            self.met_today += 1
            np.savez(self.path, emb=np.array(self.embs))
            return True

    def count(self):
        with self.lock:
            return len(self.embs)


class Perception:
    def __init__(self, facedb):
        self.db = facedb
        self.det = cv2.FaceDetectorYN.create(os.path.join(MODELS, "face_detection_yunet_2023mar.onnx"), "",
                                             (320, 320), 0.75, 0.3, 20)
        self.rec = cv2.FaceRecognizerSF.create(os.path.join(MODELS, "face_recognition_sface_2021dec.onnx"), "")
        sys.path.insert(0, MODELS)
        from nanodet import NanoDet                              # OpenCV zoo helper (models/nanodet.py)
        self.persons = NanoDet(os.path.join(MODELS, "object_detection_nanodet_2022nov.onnx"), prob_threshold=0.45)
        # This model export has 3 levels (strides 8/16/32) and returns all class maps first, then all
        # box maps; the zoo helper assumes 4 interleaved levels. Pair outputs by size ourselves.
        self.persons.strides = (8, 16, 32)
        self.persons.anchors_mlvl = self.persons.anchors_mlvl[:3]
        self.tracks = []        # [{"box", "name", "until"}] keeps a name on a person seen from behind
        self.face_tracks = []   # same idea for faces: a recognised face stays named while it moves
        self.friends = FriendBook()
        self.new_friends = 0    # read by the brain to react
        self.last_auto_learn = {}
        self.n = 0
        self.people_every = 3                            # brain sets 1 while following (smoother)

    def faces(self, img, detector=None):
        h, w = img.shape[:2]
        det = detector or self.det
        det.setInputSize((w, h))
        _, faces = det.detect(img)
        return [] if faces is None else list(faces)

    def _photo_detector(self):
        # OpenCV 5's new DNN engine cannot change input size once a net has run, so photos (any size)
        # get a fresh detector instead of the camera one (fixed 640x480).
        return cv2.FaceDetectorYN.create(os.path.join(MODELS, "face_detection_yunet_2023mar.onnx"), "",
                                         (320, 320), 0.7, 0.3, 20)

    def embedding(self, img, face_row):
        return self.rec.feature(self.rec.alignCrop(img, face_row)).reshape(-1)

    def _person_boxes(self, img):
        h, w = img.shape[:2]
        nd = self.persons
        nd.net.setInput(nd.pre_process(cv2.resize(img, (416, 416))))
        outs = nd.net.forward(nd.net.getUnconnectedOutLayersNames())
        by_size = lambda o: -o.reshape(-1, o.shape[-1]).shape[0]           # biggest map = stride 8
        cls = sorted([o for o in outs if o.shape[-1] == 80], key=by_size)
        reg = sorted([o for o in outs if o.shape[-1] != 80], key=by_size)
        out = nd.post_process([x for pair in zip(cls, reg) for x in pair])
        boxes = []
        for row in (out if len(out) else []):
            if int(row[5]) == PERSON_CLASS:
                x1, y1, x2, y2 = row[0] * w / 416, row[1] * h / 416, row[2] * w / 416, row[3] * h / 416
                boxes.append((x1, y1, x2, y2, float(row[4])))
        return boxes

    @staticmethod
    def _bearing(cx, w):
        return (cx / w - 0.5) * HFOV_DEG

    @staticmethod
    def _distance(h_px, img_w, real_m):
        focal = (img_w / 2) / math.tan(math.radians(HFOV_DEG / 2))
        return round(real_m * focal / max(h_px, 1), 2)

    def process(self, img, now):
        """Detect + recognise. Returns (detections, debug_boxes)."""
        h, w = img.shape[:2]
        dets, boxes = [], []
        self.face_tracks = [t for t in self.face_tracks if t["until"] > now]
        faces = self.faces(img)[:4]
        strangers = []                                      # (embedding) of clear unknown faces this frame
        for f in faces:
            x, y, fw, fh = (float(v) for v in f[:4])
            name, sim = "unknown", 0.0
            if fh >= 36:                                    # too small to recognise reliably
                emb = self.embedding(img, f)
                name, sim = self.db.match(emb)
                if name == "unknown" and fh >= 48:
                    strangers.append(emb)
                if name != "unknown" and sim >= AUTO_LEARN_SCORE and len(faces) == 1:
                    self._auto_learn(name, emb, now)
            box = (x, y, x + fw, y + fh)
            if name == "unknown":                           # moving / turned slightly: keep the track's name
                for t in self.face_tracks:
                    if _iou(t["box"], box) > 0.3:
                        name = t["name"]
            if name != "unknown":
                self.face_tracks = [t for t in self.face_tracks if _iou(t["box"], box) <= 0.3]
                self.face_tracks.append({"box": box, "name": name, "until": now + TRACK_KEEP_S})
            dets.append(Detection(name=name, kind="face", bearing_deg=round(self._bearing(x + fw / 2, w), 1),
                                  distance_m=self._distance(fh, w, FACE_HEIGHT_M), score=round(sim, 2)))
            boxes.append(("face", name, x, y, x + fw, y + fh))
        # Only ONE face can be the owner: keep the best match, everyone else becomes "unknown" ("?").
        face_idx = [i for i, d in enumerate(dets) if d.kind == "face" and d.name == OWNER]
        if len(face_idx) > 1:
            best = max(face_idx, key=lambda i: dets[i].score)
            for i in face_idx:
                if i != best:
                    dets[i].name = "unknown"
                    boxes[i] = (boxes[i][0], "unknown") + tuple(boxes[i][2:])
        if strangers and any(d.name == OWNER for d in dets):  # met while the owner is here -> friend
            for emb in strangers:
                if self.friends.meet(emb):
                    self.new_friends += 1
        self.n += 1
        if self.n % self.people_every == 0:                 # people detector every 3rd frame (1 while following)
            self.tracks = [t for t in self.tracks if t["until"] > now]
            for x1, y1, x2, y2, conf in self._person_boxes(img):
                name = "unknown"
                for d, b in zip(dets, boxes):                # a recognised face inside this person box
                    if d.name != "unknown" and x1 <= (b[2] + b[4]) / 2 <= x2 and y1 <= (b[3] + b[5]) / 2 <= y2:
                        name = d.name
                if name == "unknown":
                    for t in self.tracks:                   # same place as a named person a moment ago
                        if _iou(t["box"], (x1, y1, x2, y2)) > 0.3:
                            name = t["name"]
                if name != "unknown":
                    self.tracks.append({"box": (x1, y1, x2, y2), "name": name, "until": now + 4.0})
                dets.append(Detection(name=name, kind="person", bearing_deg=round(self._bearing((x1 + x2) / 2, w), 1),
                                      distance_m=self._distance(y2 - y1, w, PERSON_HEIGHT_M), score=round(conf, 2),
                                      dark=torso_darkness(img, x1, y1, x2, y2),
                                      box=(x1 / w, y1 / h, x2 / w, y2 / h)))
                boxes.append(("person", name, x1, y1, x2, y2))
            self._persons_cache = [d for d in dets if d.kind == "person"], [b for b in boxes if b[0] == "person"]
        else:
            pd, pb = getattr(self, "_persons_cache", ([], []))
            dets, boxes = dets + pd, boxes + pb
        return dets, boxes

    def _auto_learn(self, name, emb, now):
        """Self-improvement: store a confident camera view so one photo is enough to start with."""
        if now - self.last_auto_learn.get(name, 0.0) < AUTO_LEARN_EVERY_S:
            return
        if self.db.names().get(name, 0) >= MAX_VIEWS:
            return
        self.last_auto_learn[name] = now
        self.db.add(name, emb)

    def augmented_embeddings(self, img, face_row):
        """Several embeddings from ONE photo: original, mirrored (phone selfies are often mirrored),
        slightly zoomed out/in, blurred and darker (closer to what the robot's webcam sees)."""
        variants = [img, cv2.flip(img, 1)]
        h, w = img.shape[:2]
        variants.append(cv2.GaussianBlur(img, (5, 5), 0))
        variants.append(cv2.convertScaleAbs(img, alpha=0.7, beta=-10))
        variants.append(cv2.convertScaleAbs(img, alpha=1.2, beta=15))
        small = cv2.resize(img, (w // 2, h // 2))
        variants.append(cv2.resize(small, (w, h)))                       # low-resolution look
        embs = []
        det = self._photo_detector()
        for v in variants:
            found = self.faces(v, det)
            if found:
                embs.append(self.embedding(v, max(found, key=lambda f: f[2] * f[3])))
        return embs

    def learn_from_image(self, img):
        """Embedding of the largest face in an image (photo from Telegram, file, or camera)."""
        if max(img.shape[:2]) > 1280:                         # big phone photos: work on a smaller copy
            s = 1280 / max(img.shape[:2])
            img = cv2.resize(img, None, fx=s, fy=s)
        faces = self.faces(img, self._photo_detector())
        if not faces:
            return None, 0
        best = max(faces, key=lambda f: f[2] * f[3])
        return self.embedding(img, best), len(faces)


def _iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def torso_darkness(img, x1, y1, x2, y2):
    """Share of very dark pixels in the middle of a person box (shirt / back): black clothes score high,
    even from behind, so "follow me" can follow Atharv without seeing his face."""
    bw, bh = x2 - x1, y2 - y1
    crop = img[max(0, int(y1 + bh * 0.2)):int(y1 + bh * 0.7), max(0, int(x1 + bw * 0.25)):int(x2 - bw * 0.25)]
    if crop.size == 0:
        return 0.0
    v = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)[:, :, 2]
    return round(float((v < DARK_V).mean()), 2)


def annotate(img, boxes):
    """Demo-friendly markings: known faces get their name, unknown faces a big "?";
    whole-body boxes are thin and only labelled when they belong to a known person."""
    out = img.copy()
    for kind, name, x1, y1, x2, y2 in boxes:
        p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
        if kind == "face":
            known = name != "unknown"
            color = (80, 220, 80) if known else (0, 170, 255)
            known = name == OWNER                          # demo: only the owner gets a name, others "?"
            color = (80, 220, 80) if known else (0, 170, 255)
            label = name.capitalize() if known else "?"
            cv2.rectangle(out, p1, p2, color, 3)
            scale = 0.9 if known else 1.3
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
            ty = max(th + 6, p1[1] - 8)
            cv2.rectangle(out, (p1[0], ty - th - 6), (p1[0] + tw + 10, ty + 4), color, -1)
            cv2.putText(out, label, (p1[0] + 5, ty), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 2)
        else:
            color = (80, 220, 80) if name == OWNER else (200, 160, 60)
            cv2.rectangle(out, p1, p2, color, 1)
            if name == OWNER:
                cv2.putText(out, name.capitalize(), (p1[0] + 4, p2[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return out
