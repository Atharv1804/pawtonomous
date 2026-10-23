#!/usr/bin/env python3
"""MCP server that lets an agent (Hermes) drive the car through the Body API.

Not part of the ROS package: it talks plain HTTP to the `agent_bridge` node, so
it can run in its own venv (pip install mcp).

Environment:
  ROBOT_API_URL    default http://127.0.0.1:8765
  ROBOT_API_TOKEN  default "" (set it if the API requires a token)
  ROBOT_AGENT      name shown in logs / used for locks, default "hermes"
"""

import base64
import glob
import json
import os
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_media  # noqa: E402  camera / speaker / mic (direct, not through ROS)
import robot_power  # noqa: E402  UPS HAT battery (read-only I2C)

try:  # mcp >= 2.0 renamed FastMCP to MCPServer
    from mcp.server.mcpserver import MCPServer as _Server
except ImportError:  # mcp 1.x
    from mcp.server.fastmcp import FastMCP as _Server

mcp = _Server("robot-car")


def _call(method, path, payload=None, timeout=75):
    url = os.environ.get("ROBOT_API_URL", "http://127.0.0.1:8765").rstrip("/") + path
    headers = {"Content-Type": "application/json"}
    token = os.environ.get("ROBOT_API_TOKEN", "")
    if token:
        headers["Authorization"] = "Bearer " + token
    data = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        try:
            return json.loads(e.read())
        except Exception:
            return {"ok": False, "error": "HTTP %d" % e.code}
    except Exception as e:  # connection refused, timeout, ...
        return {"ok": False, "error": "cannot reach the robot API: %s" % e}


PET_URL = os.environ.get("PET_URL", "http://127.0.0.1:8766")
OLLAMA_URL = os.environ.get("OLLAMA_URL", "http://127.0.0.1:11434")      # the Mac's Ollama, via the SSH tunnel
SCENE_MODEL = os.environ.get("ROBOT_SCENE_MODEL", "qwen3.5:2b")   # small: stays loaded beside gpt-oss, ~1 s


def _pet(method, path, payload=None, timeout=10):
    """Call the pet brain (fast local perception + behaviours)."""
    data = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(PET_URL + path, data, {"Content-Type": "application/json"}, method=method)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read() or b"{}")
    except Exception as e:
        return {"ok": False, "error": "pet brain not reachable (robot-pet service): %s" % e}


def _latest_user_image(max_age_s=1800):
    """Newest photo the user sent (Telegram etc.); Hermes caches inbound images here."""
    files = []
    for d in ("~/.hermes/cache/images", "~/.hermes/image_cache"):
        files += [f for f in glob.glob(os.path.join(os.path.expanduser(d), "*")) if os.path.isfile(f)]
    files = [f for f in files if time.time() - os.path.getmtime(f) < max_age_s]
    return max(files, key=os.path.getmtime) if files else ""


def _agent():
    return os.environ.get("ROBOT_AGENT", "hermes")


def _act(behavior, wait, **params):
    return _call("POST", "/act", dict(behavior=behavior, agent=_agent(), wait=wait, **params))


@mcp.tool()
def robot_state() -> dict:
    """Current robot status: distances in metres from the left/center/right
    ultrasonic sensors (null = nothing detected), the safety reflex status,
    the behaviour currently running, and whether sensors are healthy.

    IMPORTANT: the camera is mounted high and CANNOT see the floor near the
    car. The ultrasonics are the only close-range obstacle sensing, and they
    only look forward (no rear sensor). Check this before moving."""
    return _call("GET", "/state")


@mcp.tool()
def robot_forward(distance_m: float = 0.5) -> dict:
    """Drive forward roughly distance_m metres (0.05-2.0). Distance is
    open-loop (time x speed), so it is approximate. Stops early with result
    'blocked' if an obstacle is in the way. Waits until finished."""
    return _act("forward", True, distance=distance_m)


@mcp.tool()
def robot_turn(degrees: float) -> dict:
    """Rotate in place roughly `degrees` (positive = left/counter-clockwise,
    negative = right, max 360). Approximate (no encoders). Waits until done."""
    return _act("turn", True, degrees=degrees)


@mcp.tool()
def robot_scan() -> dict:
    """Spin 360 degrees in 30-degree steps and measure the distance straight
    ahead at each heading. Returns ranges_m (index 0 = original heading, then
    counter-clockwise) and best_heading_deg (most open direction). Use it to
    decide where to go, then robot_turn to that heading. Takes ~10-15 s."""
    return _act("scan", True, timeout=90)


@mcp.tool()
def robot_explore(seconds: float = 30, wait: bool = False) -> dict:
    """Wander autonomously for up to `seconds` (1-300): steers away from
    walls, turns toward open space, escapes when stuck. By default returns
    immediately; poll robot_state / robot_events or call robot_stop to end it.
    Set wait=true to block until it finishes."""
    return _act("explore", wait, seconds=seconds, timeout=min(seconds + 10, 120))


@mcp.tool()
def robot_stop() -> dict:
    """Stop all motion immediately (always allowed, even if another agent
    holds the driving lock). Also cancels any pet task (find/follow/wander)."""
    pet = _pet("POST", "/stop", {})
    body = _call("POST", "/stop", {})
    return {"ok": bool(body.get("ok")), "body": body, "pet": pet}


@mcp.tool()
def robot_events(since: int = 0) -> dict:
    """Recent finished behaviours (id, behavior, result, data). Pass the
    'next' value from the previous call as `since` to get only new ones."""
    return _call("GET", "/events?since=%d" % int(since))


@mcp.tool()
def robot_look() -> dict:
    """Take a photo with the robot's camera and return its file path. To see
    what is in it, pass the path to your image tool (vision_analyze). The
    camera is mounted high: good for the room, people and far objects, blind
    to the floor near the car. Takes ~2 s. To SEND the photo to the person
    (e.g. on Telegram), put `MEDIA:<path>` on its own line in your reply."""
    return robot_media.take_photo()


@mcp.tool()
def robot_say(text: str) -> dict:
    """Say `text` out loud through the robot's own speaker (offline voice).
    Use it to talk to people in the room. Keep it short (1-3 sentences).
    Returns when finished speaking."""
    return robot_media.say(text)


@mcp.tool()
def robot_listen(seconds: float = 5) -> dict:
    """Record `seconds` (1-30) from the robot's microphone and transcribe it
    offline. Returns the text heard (empty if nobody spoke) and the sound
    level in dBFS (around -60 or lower = silence). Tell people to speak
    before calling it; the first call loads the speech model (~15 s)."""
    return robot_media.listen(seconds)


@mcp.tool()
def robot_battery() -> dict:
    """Battery of the Pi's UPS HAT (the robot's brain power, not the motor
    battery): percent, voltage, whether it is charging, and minutes_left
    (or minutes_to_full while charging). Mention it if below ~20%."""
    return robot_power.battery_status()


# ---------------- pet brain: fast local vision + autonomous behaviours ----------------
# These return immediately; the pet brain then runs the task on its own at 10 Hz
# (face/person tracking, obstacle avoidance) until done, stopped, or replaced.

@mcp.tool()
def robot_find_person(name: str = "any", timeout_s: float = 180) -> dict:
    """Search the room for a person and drive to them: spins looking, then
    explores, recognising faces locally (fast). `name` = a learned face
    (see robot_who_is_here known_faces) or "any". Returns at once; follow
    progress with robot_pet_events (results: found, arrived, timeout)."""
    return _pet("POST", "/command", {"mode": "find", "target": name, "timeout": timeout_s})


@mcp.tool()
def robot_go_to_person(name: str = "any") -> dict:
    """Drive up to a person who is (or was just) visible and stop ~0.8 m in
    front of them. Use robot_find_person if they are not in view."""
    return _pet("POST", "/command", {"mode": "go_to", "target": name})


@mcp.tool()
def robot_follow(name: str = "any", timeout_s: float = 300) -> dict:
    """Follow a person (keeps facing them, stays ~1 m behind) until stopped
    or timeout. Works from behind too once they were recognised."""
    return _pet("POST", "/command", {"mode": "follow", "target": name, "timeout": timeout_s})


@mcp.tool()
def robot_look_at(name: str = "any") -> dict:
    """Turn in place to face a person (searches by turning if not visible)."""
    return _pet("POST", "/command", {"mode": "look_at", "target": name})


@mcp.tool()
def robot_wander(seconds: float = 60) -> dict:
    """Explore the room on its own for `seconds` like a curious pet: the camera and a
    small vision model pick the way, it turns away from obstacles, and it stops to
    comment out loud on interesting things it sees."""
    return _pet("POST", "/command", {"mode": "curious", "timeout": seconds})


@mcp.tool()
def robot_who_is_here() -> dict:
    """Who the camera sees right now (name or 'unknown', face or person,
    bearing: + = right of centre, rough distance), when/where each known
    person was last seen, the current pet task, and the known faces."""
    return _pet("GET", "/state")


@mcp.tool()
def robot_learn_face(name: str, image_path: str = "", from_camera: bool = False) -> dict:
    """Remember a person's face so the robot can recognise and find them.
    Sources: image_path (a file), from_camera=true (the person stands in front
    of the camera), or neither = the latest photo the user sent (e.g. on
    Telegram). Call it several times with different photos for better accuracy."""
    if not image_path and not from_camera:
        image_path = _latest_user_image()
        if not image_path:
            return {"ok": False, "error": "no recent photo from the user; ask them to send one, or use from_camera"}
    payload = {"name": name}
    if not from_camera:                     # the brain runs on the Mac: send the photo itself, not a Pi path
        try:
            payload["image_b64"] = base64.b64encode(open(os.path.expanduser(image_path), "rb").read()).decode()
        except OSError as e:
            return {"ok": False, "error": "cannot read %s: %s" % (image_path, e)}
    return _pet("POST", "/learn", payload, timeout=30)


@mcp.tool()
def robot_forget_face(name: str) -> dict:
    """Delete a learned face."""
    return _pet("POST", "/forget", {"name": name})


@mcp.tool()
def robot_pet_events(since: int = 0) -> dict:
    """What the pet brain did: tasks started/finished (found, arrived,
    timeout, stopped), people greeted, faces learned. Pass `next` as since."""
    return _pet("GET", "/events?since=%d" % int(since))


@mcp.tool()
def robot_set_autonomy(enabled: bool) -> dict:
    """Autonomy on = the robot greets people it knows and wanders on its own
    when bored. Off = it only acts on commands."""
    return _pet("POST", "/autonomy", {"enabled": enabled})


@mcp.tool()
def robot_describe_scene(question: str = "Describe what you see in one or two sentences.") -> dict:
    """FAST look: the current camera view answered by a local vision model on
    Atharv's Mac (~1 s, no limits). Use it for 'what do you see', 'is there a
    red chair', 'which way is the door'. Faces/identities: use robot_who_is_here
    instead (more reliable). Falls back with an error if the Mac is offline;
    then use robot_look + vision_analyze."""
    try:
        with urllib.request.urlopen(os.environ.get("PI_IO_URL", "http://127.0.0.1:8767") + "/frame.jpg",
                                    timeout=5) as r:
            jpg = r.read()
    except Exception:
        shot = robot_media.take_photo()
        if not shot.get("ok"):
            return shot
        jpg = open(shot["path"], "rb").read()
    body = {"model": SCENE_MODEL, "stream": False, "think": False, "keep_alive": "24h",
            "options": {"num_ctx": 4096, "temperature": 0.2},
            "messages": [{"role": "user", "content": question, "images": [base64.b64encode(jpg).decode()]}]}
    t = time.time()
    try:
        req = urllib.request.Request(OLLAMA_URL + "/api/chat", json.dumps(body).encode(),
                                     {"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=60) as r:
            answer = json.load(r)["message"]["content"].strip()
    except Exception as e:
        return {"ok": False, "error": "Mac vision model not reachable (Mac asleep?): %s" % e}
    return {"ok": True, "answer": answer, "seconds": round(time.time() - t, 1), "model": SCENE_MODEL}


if __name__ == "__main__":
    mcp.run()
