"""Tests the Body API over real HTTP with a fake backend (no ROS)."""

import importlib.util
import json
import os
import threading
import urllib.error
import urllib.request

import pytest

from robot_base.api import (ApiError, ApiServer, EventLog, Lease, normalize_act, normalize_drive)


class FakeBackend:
    def __init__(self):
        self.log = EventLog()
        self.cmds = []
        self.stopped = 0
        self.auto_finish = True

    def get_state(self):
        return {"ranges": {"left": None, "center": 1.2, "right": 0.8}, "safety": "clear"}

    def submit(self, cmd):
        cid = "id%d" % (len(self.cmds) + 1)
        self.cmds.append(dict(cmd, id=cid))
        if self.auto_finish:
            threading.Timer(0.05, lambda: self.log.append(
                {"id": cid, "behavior": cmd["behavior"], "result": "ok", "data": {}})).start()
        return cid

    def emergency_stop(self):
        self.stopped += 1

    def drive(self, linear, angular):
        self.drives = getattr(self, "drives", []) + [(linear, angular)]


def http(port, method, path, body=None, token=None):
    req = urllib.request.Request(
        "http://127.0.0.1:%d%s" % (port, path),
        data=json.dumps(body).encode() if body is not None else None,
        method=method, headers={"Content-Type": "application/json"})
    if token:
        req.add_header("Authorization", "Bearer " + token)
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


@pytest.fixture
def server():
    be = FakeBackend()
    s = ApiServer(be, "127.0.0.1", 0, token="secret")
    s.start()
    yield s, be
    s.shutdown()


# ---------------- normalize_act ----------------
def test_normalize_clamps_values():
    cmd, wait, timeout = normalize_act({"behavior": "forward", "distance": 99})
    assert cmd["distance"] == 2.0 and wait is True
    assert normalize_act({"behavior": "turn", "degrees": -999})[0]["degrees"] == -360
    assert normalize_act({"behavior": "explore", "seconds": 9999})[0]["seconds"] == 300
    assert normalize_act({"behavior": "explore"})[1] is False   # explore doesn't wait by default


@pytest.mark.parametrize("p", [
    {}, {"behavior": "fly"}, {"behavior": "forward", "distance": -1},
    {"behavior": "forward", "distance": 0}, {"behavior": "forward", "distance": "x"},
    {"behavior": "forward", "distance": float("nan")}, {"behavior": "turn"},
    {"behavior": "turn", "degrees": 0}, {"behavior": "turn", "degrees": True}, [], "forward",
])
def test_normalize_rejects_bad_input(p):
    with pytest.raises(ApiError):
        normalize_act(p)


# ---------------- Lease ----------------
def test_lease_blocks_others_and_expires():
    now = [0.0]
    lease = Lease(clock=lambda: now[0])
    lease.acquire("watcher", 30)
    lease.check("watcher")
    with pytest.raises(ApiError) as e:
        lease.check("hermes")
    assert e.value.status == 409
    now[0] = 31
    lease.check("hermes")             # expired


def test_lease_cannot_be_stolen_but_can_be_released():
    lease = Lease(clock=lambda: 0.0)
    lease.acquire("watcher", 30)
    with pytest.raises(ApiError):
        lease.acquire("hermes", 30)
    lease.release("hermes")           # wrong owner: no effect
    assert lease.status()["holder"] == "watcher"
    lease.release("watcher")
    lease.acquire("hermes", 30)


# ---------------- EventLog ----------------
def test_eventlog_since_and_wait():
    log = EventLog(maxlen=3)
    for i in range(5):
        log.append({"id": str(i), "result": "ok"})
    evs, nxt = log.since(0)
    assert [e["id"] for e in evs] == ["2", "3", "4"] and nxt == 5
    assert log.wait_for("4", 0.1)["id"] == "4"
    assert log.wait_for("nope", 0.1) is None


# ---------------- HTTP ----------------
def test_health_needs_no_token(server):
    s, _ = server
    assert http(s.port, "GET", "/health") == (200, {"ok": True})


def test_auth_required(server):
    s, _ = server
    assert http(s.port, "GET", "/state")[0] == 401
    assert http(s.port, "GET", "/state", token="wrong")[0] == 401
    code, body = http(s.port, "GET", "/state", token="secret")
    assert code == 200 and body["ranges"]["center"] == 1.2 and "lease" in body


def test_act_waits_for_result(server):
    s, be = server
    code, body = http(s.port, "POST", "/act",
                      {"behavior": "forward", "distance": 0.5, "agent": "hermes"}, "secret")
    assert code == 200 and body["finished"] and body["event"]["result"] == "ok"
    assert be.cmds[0]["distance"] == 0.5 and be.cmds[0]["agent"] == "hermes"


def test_act_no_wait_returns_immediately(server):
    s, be = server
    be.auto_finish = False
    code, body = http(s.port, "POST", "/act", {"behavior": "explore", "seconds": 5}, "secret")
    assert code == 200 and body["started"] and "event" not in body


def test_act_timeout_reports_unfinished(server):
    s, be = server
    be.auto_finish = False
    code, body = http(s.port, "POST", "/act",
                      {"behavior": "forward", "distance": 0.5, "timeout": 1}, "secret")
    assert code == 200 and body["finished"] is False and body["event"] is None


def test_act_rejects_bad_requests(server):
    s, be = server
    assert http(s.port, "POST", "/act", {"behavior": "forward", "distance": -2}, "secret")[0] == 400
    assert http(s.port, "POST", "/act", {"behavior": "rm -rf"}, "secret")[0] == 400
    assert be.cmds == []


def test_stop_always_works_even_when_locked(server):
    s, be = server
    assert http(s.port, "POST", "/lock", {"agent": "watcher", "seconds": 30}, "secret")[0] == 200
    code, body = http(s.port, "POST", "/act",
                      {"behavior": "forward", "distance": 0.5, "agent": "hermes"}, "secret")
    assert code == 409 and "watcher" in body["error"]
    assert http(s.port, "POST", "/stop", {}, "secret")[0] == 200 and be.stopped == 1
    assert http(s.port, "POST", "/act", {"behavior": "stop", "agent": "hermes"}, "secret")[0] == 200
    code, _ = http(s.port, "POST", "/act",
                   {"behavior": "forward", "distance": 0.5, "agent": "watcher"}, "secret")
    assert code == 200


def test_events_endpoint(server):
    s, be = server
    http(s.port, "POST", "/act", {"behavior": "turn", "degrees": 30}, "secret")
    code, body = http(s.port, "GET", "/events?since=0", token="secret")
    assert code == 200 and body["events"][0]["behavior"] == "turn" and body["next"] == 1


def test_unknown_path_404(server):
    s, _ = server
    assert http(s.port, "GET", "/nope", token="secret")[0] == 404


def test_non_local_bind_requires_token():
    with pytest.raises(ValueError):
        ApiServer(FakeBackend(), "0.0.0.0", 0, token="")


# ---------------- MCP wrapper against the real HTTP API ----------------
def _load_mcp_module():
    path = os.path.join(os.path.dirname(__file__), "..", "..", "..", "tools", "robot_mcp_server.py")
    spec = importlib.util.spec_from_file_location("robot_mcp_server", os.path.abspath(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_mcp_tools_end_to_end(server, monkeypatch):
    pytest.importorskip("mcp")
    s, be = server
    monkeypatch.setenv("ROBOT_API_URL", "http://127.0.0.1:%d" % s.port)
    monkeypatch.setenv("ROBOT_API_TOKEN", "secret")
    m = _load_mcp_module()
    assert m.robot_state()["ranges"]["right"] == 0.8
    r = m.robot_forward(0.4)
    assert r["finished"] and be.cmds[-1]["behavior"] == "forward" and be.cmds[-1]["agent"] == "hermes"
    assert m.robot_turn(-90)["event"]["result"] == "ok"
    assert m.robot_scan()["finished"]
    be.auto_finish = False
    assert m.robot_explore(10)["started"]
    assert m.robot_stop()["ok"] and be.stopped == 1
    assert "next" in m.robot_events(0)


def test_mcp_reports_unreachable_robot_cleanly(monkeypatch):
    pytest.importorskip("mcp")
    monkeypatch.setenv("ROBOT_API_URL", "http://127.0.0.1:1")
    m = _load_mcp_module()
    r = m.robot_state()
    assert r["ok"] is False and "cannot reach" in r["error"]


def test_mcp_wrong_token_is_reported(server, monkeypatch):
    pytest.importorskip("mcp")
    s, _ = server
    monkeypatch.setenv("ROBOT_API_URL", "http://127.0.0.1:%d" % s.port)
    monkeypatch.setenv("ROBOT_API_TOKEN", "nope")
    m = _load_mcp_module()
    assert m.robot_state() == {"ok": False, "error": "unauthorized"}


# ---------------- /drive (live velocity for the pet brain) ----------------

def test_normalize_drive_clamps_and_refuses_reverse():
    assert normalize_drive({"linear": 9, "angular": -9, "agent": "pet"}) == (0.35, -1.5, "pet")
    assert normalize_drive({}) == (0.0, 0.0, "unknown")
    with pytest.raises(ApiError):
        normalize_drive({"linear": -0.1})


def test_drive_endpoint_and_lease(server):
    s, be = server
    assert http(s.port, "POST", "/drive", {"linear": 0.2, "angular": 0.5, "agent": "pet"}, "secret")[0] == 200
    assert be.drives == [(0.2, 0.5)]
    assert http(s.port, "POST", "/drive", {"linear": 0.2}, None)[0] == 401
    assert http(s.port, "POST", "/lock", {"agent": "watcher", "seconds": 30}, "secret")[0] == 200
    assert http(s.port, "POST", "/drive", {"linear": 0.2, "agent": "pet"}, "secret")[0] == 409
