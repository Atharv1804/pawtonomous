"""Body API: a small token-protected HTTP/JSON interface to the robot.

Pure Python (standard library only, no ROS imports) so it can be tested on its
own. The ROS node `agent_bridge` plugs a "backend" into it. Hermes (through the
MCP wrapper in tools/), the claw agent, the Seeed Watcher - anything that can
make an HTTP request - drives the car through this one interface.

Endpoints (JSON in/out; send header  Authorization: Bearer <token>):
  GET  /health                 -> {"ok": true}            (no auth)
  GET  /state                  -> ranges, safety, active behaviour, lease
  GET  /events?since=N         -> recent finished behaviours
  POST /act   {"behavior": "forward|turn|explore|scan|stop", ...params,
               "agent": "hermes", "wait": true, "timeout": 30}
  POST /drive {"linear": m/s, "angular": rad/s, "agent": "pet"}
                               -> direct velocity for live control loops (face tracking,
                                  following). Stops any running behaviour. Resend at >= 5 Hz:
                                  the safety reflex stops the car 0.5 s after the last one.
  POST /stop                   -> emergency stop (always allowed)
  POST /lock  {"agent": "watcher", "seconds": 30}   -> only that agent may /act
  POST /unlock {"agent": "watcher"}

Guardrails live here AND in the controller, because the caller may be an LLM:
distances/angles/durations are clamped; there is no reverse command (the car
has no rear sensors).
"""

import hmac
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

ALLOWED = ("forward", "turn", "explore", "scan", "stop")
MAX_DRIVE_LINEAR = 0.35   # m/s; no reverse (no rear sensors)
MAX_DRIVE_ANGULAR = 1.5   # rad/s
LOCAL_HOSTS = ("127.0.0.1", "localhost", "::1")
MAX_BODY = 8192


class ApiError(Exception):
    def __init__(self, message, status=400):
        super().__init__(message)
        self.message = message
        self.status = status


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _num(payload, key, default=None):
    v = payload.get(key, default)
    if v is None:
        raise ApiError("%s is required" % key)
    if isinstance(v, bool):
        raise ApiError("%s must be a number" % key)
    try:
        v = float(v)
    except (TypeError, ValueError):
        raise ApiError("%s must be a number" % key)
    if math.isnan(v) or math.isinf(v):
        raise ApiError("%s must be a finite number" % key)
    return v


def normalize_act(payload):
    """Validate and clamp an /act request. Returns (cmd, wait, timeout)."""
    if not isinstance(payload, dict):
        raise ApiError("body must be a JSON object")
    b = payload.get("behavior")
    if b not in ALLOWED:
        raise ApiError("behavior must be one of: " + ", ".join(ALLOWED))
    cmd = {"behavior": b, "agent": str(payload.get("agent", "unknown"))[:32]}
    if b == "forward":
        d = _num(payload, "distance", 0.5)
        if d <= 0:
            raise ApiError("distance must be > 0 (reversing is not supported: no rear sensors)")
        cmd["distance"] = clamp(d, 0.05, 2.0)
    elif b == "turn":
        deg = _num(payload, "degrees")
        if deg == 0:
            raise ApiError("degrees must not be 0")
        cmd["degrees"] = clamp(deg, -360.0, 360.0)
    elif b == "explore":
        cmd["seconds"] = clamp(_num(payload, "seconds", 30.0), 1.0, 300.0)
    wait = payload.get("wait", b in ("forward", "turn", "scan"))
    timeout = clamp(_num(payload, "timeout", 60.0), 1.0, 120.0)
    return cmd, bool(wait), timeout


def normalize_drive(payload):
    """Validate and clamp a /drive request. Returns (linear, angular, agent)."""
    if not isinstance(payload, dict):
        raise ApiError("body must be a JSON object")
    linear = _num(payload, "linear", 0.0)
    angular = _num(payload, "angular", 0.0)
    if linear < 0:
        raise ApiError("linear must be >= 0 (reversing is not supported: no rear sensors)")
    agent = str(payload.get("agent", "unknown"))[:32]
    return clamp(linear, 0.0, MAX_DRIVE_LINEAR), clamp(angular, -MAX_DRIVE_ANGULAR, MAX_DRIVE_ANGULAR), agent


class EventLog:
    """Thread-safe ring buffer of finished behaviours, with waiting."""

    def __init__(self, maxlen=200):
        self._cv = threading.Condition()
        self._events = []
        self._n = 0
        self._maxlen = maxlen

    def append(self, event):
        with self._cv:
            self._n += 1
            ev = dict(event, seq=self._n, t=round(time.time(), 3))
            self._events.append(ev)
            del self._events[: -self._maxlen]
            self._cv.notify_all()
            return ev

    def since(self, n):
        with self._cv:
            return [e for e in self._events if e["seq"] > n], self._n

    def wait_for(self, cid, timeout):
        """Block until an event with this id arrives (or timeout). Returns it or None."""
        deadline = time.monotonic() + timeout
        with self._cv:
            while True:
                for e in reversed(self._events):
                    if e.get("id") == cid:
                        return e
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cv.wait(remaining)


class Lease:
    """Lets one agent (e.g. the Watcher 'god agent') temporarily own driving."""

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self.holder = None
        self.until = 0.0

    def _active(self, now):
        return self.holder is not None and now < self.until

    def acquire(self, agent, seconds):
        with self._lock:
            now = self._clock()
            if self._active(now) and self.holder != agent:
                raise ApiError("locked by %s for another %.0f s" % (self.holder, self.until - now), 409)
            self.holder, self.until = agent, now + clamp(seconds, 1.0, 300.0)

    def release(self, agent):
        with self._lock:
            if self.holder == agent:
                self.holder, self.until = None, 0.0

    def check(self, agent):
        with self._lock:
            now = self._clock()
            if self._active(now) and self.holder != agent:
                raise ApiError("locked by %s for another %.0f s" % (self.holder, self.until - now), 409)

    def status(self):
        with self._lock:
            now = self._clock()
            if self._active(now):
                return {"holder": self.holder, "seconds_left": round(self.until - now, 1)}
            return {"holder": None, "seconds_left": 0}


class ApiServer:
    """backend must provide: get_state()->dict, submit(cmd)->id, emergency_stop(), log (EventLog)."""

    def __init__(self, backend, host="127.0.0.1", port=8765, token=""):
        if host not in LOCAL_HOSTS and not token:
            raise ValueError("a token is required when binding to a non-local address")
        self.backend = backend
        self.token = token or ""
        self.lease = Lease()
        self.httpd = ThreadingHTTPServer((host, port), self._make_handler())
        self.thread = None

    @property
    def port(self):
        return self.httpd.server_address[1]

    def start(self):
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()

    def shutdown(self):
        self.httpd.shutdown()
        self.httpd.server_close()

    # ---------------------------------------------------------------
    def _make_handler(self):
        server = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.0"

            def log_message(self, fmt, *args):  # keep the ROS console quiet
                pass

            def _send(self, status, obj):
                body = json.dumps(obj).encode()
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _auth(self):
                if not server.token:
                    return
                sent = self.headers.get("Authorization", "")
                if not hmac.compare_digest(sent, "Bearer " + server.token):
                    raise ApiError("unauthorized", 401)

            def _json(self):
                n = int(self.headers.get("Content-Length") or 0)
                if n > MAX_BODY:
                    raise ApiError("body too large", 413)
                raw = self.rfile.read(n) if n else b"{}"
                try:
                    return json.loads(raw or b"{}")
                except ValueError:
                    raise ApiError("invalid JSON")

            def _handle(self, method):
                try:
                    url = urlparse(self.path)
                    if method == "GET" and url.path == "/health":
                        return self._send(200, {"ok": True})
                    self._auth()
                    if method == "GET" and url.path == "/state":
                        st = dict(server.backend.get_state())
                        st["lease"] = server.lease.status()
                        return self._send(200, st)
                    if method == "GET" and url.path == "/events":
                        q = parse_qs(url.query)
                        since = int(q.get("since", ["0"])[0])
                        evs, nxt = server.backend.log.since(since)
                        return self._send(200, {"events": evs, "next": nxt})
                    if method == "POST" and url.path == "/act":
                        cmd, wait, timeout = normalize_act(self._json())
                        if cmd["behavior"] != "stop":
                            server.lease.check(cmd["agent"])
                        cid = server.backend.submit(cmd)
                        if not wait:
                            return self._send(200, {"ok": True, "id": cid, "started": True})
                        ev = server.backend.log.wait_for(cid, timeout)
                        return self._send(200, {"ok": True, "id": cid, "finished": ev is not None, "event": ev})
                    if method == "POST" and url.path == "/drive":
                        linear, angular, agent = normalize_drive(self._json())
                        server.lease.check(agent)
                        server.backend.drive(linear, angular)
                        return self._send(200, {"ok": True, "linear": linear, "angular": angular})
                    if method == "POST" and url.path == "/stop":
                        server.backend.emergency_stop()
                        return self._send(200, {"ok": True})
                    if method == "POST" and url.path == "/lock":
                        p = self._json()
                        server.lease.acquire(str(p.get("agent", "unknown"))[:32], _num(p, "seconds", 30.0))
                        return self._send(200, {"ok": True, "lease": server.lease.status()})
                    if method == "POST" and url.path == "/unlock":
                        server.lease.release(str(self._json().get("agent", "unknown"))[:32])
                        return self._send(200, {"ok": True, "lease": server.lease.status()})
                    raise ApiError("not found", 404)
                except ApiError as e:
                    self._send(e.status, {"ok": False, "error": e.message})
                except Exception as e:  # never crash the server thread
                    self._send(500, {"ok": False, "error": "internal error: %s" % e})

            def do_GET(self):
                self._handle("GET")

            def do_POST(self):
                self._handle("POST")

        return Handler
