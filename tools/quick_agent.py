#!/usr/bin/env python3
"""Fast spoken answers + full robot control for the showcase: one local LLM call (~1 s) with tools.

Hermes is great over Telegram but takes ~6 s per spoken question. For "Hey Jarvis" at a showcase the
voice loop asks this instead: gpt-oss-20b in LM Studio on the Mac (reached from the Pi through the tunnel
at the same 127.0.0.1:1234), a short persona, and a handful of tools that drive the robot through the
pet brain and the Body API (which still goes through the ROS safety reflex).

ask(text, history) -> (reply, history). reply is None if the model is unreachable (caller falls back).
"""

import json
import os
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_media  # noqa: E402  (_pet, _http, describe_scene)

ON_MAC = sys.platform == "darwin"
LLM_URL = os.environ.get("QUICK_LLM_URL", "http://127.0.0.1:1234").rstrip("/") + "/v1/chat/completions"
MODEL = os.environ.get("QUICK_LLM_MODEL", "openai/gpt-oss-20b")
BODY_URL = os.environ.get("ROBOT_API_URL", "http://127.0.0.1:18765" if ON_MAC else "http://127.0.0.1:8765")
MAX_FORWARD_M = 1.5
HISTORY_TURNS = 6

SYSTEM = """You are Jarvis, a small, friendly, curious robot pet on wheels at a tech showcase. Atharv, a first-year \
university student, built you. Visitors talk to you by saying "Hey Jarvis". Your words are spoken aloud by your \
speaker and shown on your screen, so answer in 1-2 short, warm, plain sentences: no lists, markdown or emojis.

About you (use only when asked): your body is a Raspberry Pi 5 running ROS 2, an ESP32 driving four motors, \
ultrasonic sensors as whiskers, a camera, microphones, a speaker and a touchscreen face. Your brain runs locally \
on Atharv's MacBook: gpt-oss-20b answers questions, OpenCV models recognise faces and people, Whisper hears, Piper \
speaks, and a small Qwen vision model helps you explore. You can follow people, explore, do tricks and take selfies.

You can really move. When someone asks you to move, go somewhere, turn, come, follow, explore, stop, do a trick or \
look, CALL THE TOOL, then say in a few words what you did. Moves are short and careful: at most 1.5 metres at a \
time. "The front" means forward. You cannot see the floor right in front of you, so prefer small moves. \
Never pretend you did something you did not do."""

TOOLS = [
    {"type": "function", "function": {
        "name": "move", "description": "Drive the robot body now: turn first (degrees, positive = left, "
        "negative = right), then go forward (metres, 0-1.5). The robot cannot drive backwards; to go back, "
        "turn 180 then go forward.",
        "parameters": {"type": "object", "properties": {
            "turn_deg": {"type": "number", "description": "-180..180, 0 for none"},
            "forward_m": {"type": "number", "description": "0..1.5, 0 for none"}}}}},
    {"type": "function", "function": {
        "name": "behaviour", "description": "Start an ongoing behaviour: explore (wander around curiously), "
        "follow_me (follow the person talking), come_here (find and drive to the nearest person), "
        "look_at_me (turn to face the nearest person), stop (stand still).",
        "parameters": {"type": "object", "properties": {"name": {"type": "string", "enum": [
            "explore", "follow_me", "come_here", "look_at_me", "stop"]}}, "required": ["name"]}}},
    {"type": "function", "function": {
        "name": "trick", "description": "Pet tricks with animations on the screen: greet (wave hello), bye "
        "(wave goodbye), sit, treat (happy dance), selfie (3-2-1 photo), camera_view (show what you see on "
        "the screen), face_view (back to your face).",
        "parameters": {"type": "object", "properties": {"name": {"type": "string", "enum": [
            "greet", "bye", "sit", "treat", "selfie", "camera_view", "face_view"]}}, "required": ["name"]}}},
    {"type": "function", "function": {
        "name": "look", "description": "Look through your camera and get a short description of what is in "
        "front of you. Use it when asked what you see or about things around you.",
        "parameters": {"type": "object", "properties": {}}}},
]

BEHAVIOURS = {"explore": ("curious", 600), "follow_me": ("follow_straight", 180), "come_here": ("go_to", 60),
              "look_at_me": ("look_at", 15)}
TRICKS = {"camera_view": "view_camera", "face_view": "view_face"}


def _body(path, payload, timeout):
    data = robot_media._http(BODY_URL + path, json.dumps(payload).encode(), timeout=timeout)
    try:
        return json.loads(data) if data else {"ok": False, "error": "body not reachable"}
    except ValueError:
        return {"ok": False, "error": "bad answer from body"}


def _act(behavior, **params):
    r = _body("/act", dict(behavior=behavior, agent="jarvis-voice", wait=True, **params), timeout=20)
    return (r.get("event") or {}).get("result") or r.get("error") or ("finished" if r.get("finished") else "?")


def run_tool(name, args):
    """Execute one tool call; returns a short result dict for the model."""
    if name == "move":
        turn = max(-180.0, min(180.0, float(args.get("turn_deg") or 0)))
        fwd = max(0.0, min(MAX_FORWARD_M, float(args.get("forward_m") or 0)))
        robot_media._pet("POST", "/stop", {}, timeout=2)          # the brain stops driving first
        out = {}
        if abs(turn) >= 5:
            out["turn"] = _act("turn", degrees=turn)
        if fwd >= 0.05:
            out["forward"] = _act("forward", distance=fwd)
        return out or {"result": "nothing to do"}
    if name == "behaviour":
        b = args.get("name")
        if b == "stop":
            robot_media._pet("POST", "/stop", {}, timeout=2)
            return {"result": "stopped"}
        if b in BEHAVIOURS:
            mode, timeout = BEHAVIOURS[b]
            r = robot_media._pet("POST", "/command", {"mode": mode, "target": "any", "timeout": timeout}, timeout=2)
            return {"result": "started" if r else "brain not reachable"}
    if name == "trick":
        t = TRICKS.get(args.get("name"), args.get("name"))
        r = robot_media._pet("POST", "/demo", {"action": t}, timeout=2)
        return {"result": "done, the screen shows it" if r else "brain not reachable"}
    if name == "look":
        return {"seeing": robot_media.describe_scene() or "I can't see clearly right now."}
    return {"error": "unknown tool %s" % name}


def _chat(messages):
    body = {"model": MODEL, "messages": messages, "tools": TOOLS, "max_tokens": 300,
            "reasoning_effort": "low", "temperature": 0.6}
    req = urllib.request.Request(LLM_URL, json.dumps(body).encode(), {"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.load(r)["choices"][0]["message"]


def ask(text, history=None, log=print):
    """One spoken turn: up to 3 rounds of tool calls, then a short spoken reply."""
    history = list(history or [])[-HISTORY_TURNS * 2:]
    messages = [{"role": "system", "content": SYSTEM}] + history + [{"role": "user", "content": text}]
    try:
        for _ in range(3):
            t0 = time.time()
            msg = _chat(messages)
            calls = msg.get("tool_calls") or []
            log("quick agent: %.1fs, tools=%s" % (time.time() - t0, [c["function"]["name"] for c in calls]))
            if not calls:
                reply = (msg.get("content") or "").strip()
                if not reply:
                    return None, history
                return reply, history + [{"role": "user", "content": text}, {"role": "assistant", "content": reply}]
            messages.append({"role": "assistant", "content": msg.get("content") or "", "tool_calls": calls})
            for c in calls:
                try:
                    args = json.loads(c["function"].get("arguments") or "{}")
                except ValueError:
                    args = {}
                result = run_tool(c["function"]["name"], args)
                log("quick agent: %s(%s) -> %s" % (c["function"]["name"], args, result))
                messages.append({"role": "tool", "tool_call_id": c.get("id", ""), "content": json.dumps(result)})
        return "Done!", history + [{"role": "user", "content": text}, {"role": "assistant", "content": "Done!"}]
    except (OSError, ValueError, KeyError, IndexError) as e:
        log("quick agent failed: %s" % e)
        return None, history


def keep_warm(every_s=240):
    """Background: ask a tiny question now and then so the model keeps the persona + tools cached
    (a cold first question takes ~9 s instead of ~1-2 s)."""
    while True:
        try:
            _chat([{"role": "system", "content": SYSTEM}, {"role": "user", "content": "Say hi in two words."}])
        except (OSError, ValueError, KeyError, IndexError):
            pass
        time.sleep(every_s)


if __name__ == "__main__":                                   # quick manual test: python3 quick_agent.py "hi"
    print(ask(" ".join(sys.argv[1:]) or "Who built you?")[0])
