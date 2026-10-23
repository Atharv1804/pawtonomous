"""Instant voice commands: recognise simple requests locally so they act in milliseconds,
without waiting for the cloud LLM. Anything not matched here goes to Hermes as usual.

parse(text) -> dict like {"action": "pet", "mode": "find", "target": "atharv", "reply": "..."}
            or {"action": "battery"} / {"action": "who"} / None
"""

import re

_NAME = r"([a-z][a-z'\-]{1,30})"
_NOT_NAMES = {"the", "a", "an", "it", "that", "this", "something", "my", "your", "me", "him", "her",
              "them", "us", "out", "around", "here", "there", "home", "back", "sleep", "bed"}


def _clean(text):
    t = (text or "").lower()
    t = re.sub(r"^\W*(hey|hi|ok|okay)?\W*jarvis\W*", "", t)        # leading "hey jarvis,"
    t = re.sub(r"[^a-z0-9' \-]+", " ", t)
    return re.sub(r"\s+", " ", t).strip()


# Demo / pet tricks: hard-coded so they run instantly (no LLM), tolerant to different wording.
_DEMO = [
    ("bye", r"\b(say )?(bye|goodbye|good bye|see you)\b"),
    ("greet", r"\b(say (hi|hello|hey)|greet|wave)\b|\b(hi|hello|hey)\b.*\b(judges?|every ?(one|body)|all|audience|guys|people|friends)\b"),
    ("selfie", r"\bselfie\b|\btake (a|our|my) (photo|picture|pic)\b"),
    ("treat", r"\b(treat|treats|snack|cookie|biscuit)\b"),
    ("view_camera", r"\b(camera|vision) (view|mode|feed)\b|\bshow (them |everyone |us |the judges )?(what|how) you see\b"
                    r"|\bshow (your|the) (camera|vision|eyes)\b"),
    ("view_face", r"\b(normal|face) (view|mode)\b|\bshow your face\b|\bback to (normal|your face)\b"),
    ("sit", r"\bsit\b|\bsit down\b|\bsitting\b"),
]


def parse(text):
    t = _clean(text)
    if not t:
        return None
    words = t.split()
    for action, pattern in _DEMO:
        if len(words) <= 12 and re.search(pattern, t):
            return {"action": "demo", "demo": action}
    # --- stop: short utterances only, so "don't stop talking about ..." goes to the LLM ---
    if len(words) <= 4 and re.search(r"\b(stop|stay|freeze|halt|wait|enough|cancel)\b", t):
        return {"action": "pet", "mode": "idle", "reply": "Okay, stopping."}
    if re.search(r"\b(come (here|to me|over here|back)|over here)\b", t):
        return {"action": "pet", "mode": "go_to", "target": "any", "reply": "Coming!"}
    if re.search(r"\bfollow (me|us) (straight|in a straight line)\b|\bstraight follow\b", t):
        return {"action": "pet", "mode": "follow_straight", "target": "any", "reply": "Following you in a straight line."}
    if re.search(r"\bfollow (me|us) around\b", t):
        return {"action": "pet", "mode": "follow", "target": "any", "reply": "Following you."}
    if re.search(r"\bfollow (me|us)\b", t):        # demo default: straight line, turns are weak
        return {"action": "pet", "mode": "follow_straight", "target": "any", "reply": "Following you!"}
    if re.search(r"\blook at (me|us)\b", t):
        return {"action": "pet", "mode": "look_at", "target": "any", "reply": "Looking at you."}
    m = re.search(r"\b(?:find|look for|search for|go to|go find|where is|where's|follow)\s+" + _NAME + r"\b", t)
    if m and m.group(1) not in _NOT_NAMES:
        name = m.group(1)
        mode = "follow" if t.startswith("follow") or " follow " in " %s " % t.split(name)[0] else "find"
        return {"action": "pet", "mode": mode, "target": name,
                "reply": ("Following %s." if mode == "follow" else "Looking for %s.") % name.capitalize()}
    if len(words) <= 6 and re.search(r"\b(explore|wander|go (play|around|explore)|look around)\b", t):
        return {"action": "pet", "mode": "curious", "reply": "Ooh, let's explore!"}
    if len(words) <= 8 and re.search(r"\b(what (do|can) you see|what'?s in front of you|describe (the room|what you see|your surroundings)|what are you looking at)\b", t):
        return {"action": "see"}
    if len(words) <= 8 and re.search(r"\bbattery\b", t):
        return {"action": "battery"}
    if len(words) <= 8 and re.search(r"\b(who (do you see|is (here|there|around))|who can you see)\b", t):
        return {"action": "who"}
    return None
