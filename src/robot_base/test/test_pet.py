"""Pet brain logic (tools/pet/control.py, intents.py): pure Python, no camera or motors."""
import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "tools", "pet")))

import control  # noqa: E402
import intents  # noqa: E402
from control import Controller, Detection  # noqa: E402

FAR = {"left": None, "center": 3.0, "right": None}


def face(name="atharv", bearing=0.0, dist=2.0):
    return Detection(name=name, kind="face", bearing_deg=bearing, distance_m=dist, score=0.8)


def test_turns_toward_target_with_ros_sign_convention():
    c = Controller(); c.set("look_at", "atharv", now=0)
    a = c.step([face(bearing=20)], FAR, now=0.1, dt=0.1)
    assert a.angular < 0 and a.linear == 0          # target on the right -> turn right (negative)
    a = c.step([face(bearing=-20)], FAR, now=0.2, dt=0.1)
    assert a.angular > 0
    a = c.step([face(bearing=1)], FAR, now=0.3, dt=0.1)
    assert a.finished and a.event["result"] == "ok"


def test_go_to_drives_then_arrives_and_respects_sonar():
    c = Controller(); c.set("go_to", "atharv", now=0)
    a = c.step([face(dist=2.5)], FAR, now=0.1, dt=0.1)
    assert 0 < a.linear <= control.MAX_LINEAR
    a = c.step([face(dist=2.5, bearing=40)], FAR, now=0.2, dt=0.1)
    assert a.linear == 0 and a.angular < 0            # not aligned: turn first
    a = c.step([face(dist=2.5)], {"center": 0.4}, now=0.3, dt=0.1)
    assert a.linear == 0 and not a.finished           # obstacle in between: stop, but not "arrived"
    a = c.step([face(dist=0.7)], FAR, now=0.4, dt=0.1)
    assert a.finished and a.event["result"] == "arrived" and a.say == "Here I am, atharv!"
    assert c.cmd.mode == "idle"


def test_find_spins_then_explores_then_switches_to_go_to():
    c = Controller(); c.set("find", "atharv", now=0)
    t, a = 0.0, None
    while c.cmd.phase == "spin" and t < 30:
        t += 0.1
        a = c.step([], FAR, now=t, dt=0.1)
        assert a.angular > 0 and not a.explore
    assert c.cmd.phase == "explore" and abs(t - control.SPIN_FULL_RAD / control.SEARCH_TURN) < 0.3
    a = c.step([face(name="someone_else")], FAR, now=t + 0.1, dt=0.1)
    assert a.explore                                  # wrong person: keep exploring
    a = c.step([face(bearing=10)], FAR, now=t + 0.2, dt=0.1)
    assert a.event["result"] == "found" and c.cmd.mode == "go_to" and "Found you" in a.say


def test_find_times_out_with_message():
    c = Controller(); c.set("find", "atharv", timeout=5, now=0)
    a = c.step([], FAR, now=6, dt=0.1)
    assert a.finished and a.event["result"] == "timeout" and "atharv" in a.say


def test_lost_target_holds_then_searches_toward_last_side():
    c = Controller(); c.set("follow", "any", now=0)
    c.step([face(bearing=25)], FAR, now=0.1, dt=0.1)
    assert c.step([], FAR, now=0.5, dt=0.1).angular == 0          # brief hold
    assert c.step([], FAR, now=3.0, dt=0.1).angular < 0           # then search to the right


def test_pick_target_any_prefers_known_and_near():
    dets = [face("unknown", 0, 1.0), face("atharv", 10, 2.0)]
    assert control.pick_target(dets, "any").name == "atharv"
    assert control.pick_target(dets, "Atharv").name == "atharv"
    assert control.pick_target(dets, "bob") is None


def test_intents():
    assert intents.parse("Hey Jarvis, stop!")["mode"] == "idle"
    assert intents.parse("come here")["mode"] == "go_to"
    assert intents.parse("Follow me please")["mode"] == "follow"
    p = intents.parse("Jarvis, find Atharv")
    assert p["mode"] == "find" and p["target"] == "atharv"
    p = intents.parse("go to Priya")
    assert p["mode"] == "find" and p["target"] == "priya"
    assert intents.parse("follow Rahul")["mode"] == "follow"
    assert intents.parse("go explore")["mode"] == "curious"
    assert intents.parse("how much battery do you have")["action"] == "battery"
    assert intents.parse("who do you see")["action"] == "who"
    assert intents.parse("tell me a story about a robot that would never stop dancing") is None
    assert intents.parse("find the remote") is None or intents.parse("find the remote")["target"] != "the"
    assert intents.parse("what's the weather like") is None


def test_see_intent():
    assert intents.parse("What do you see?")["action"] == "see"
    assert intents.parse("Jarvis, what's in front of you")["action"] == "see"


def test_follow_any_locks_onto_first_known_person():
    c = Controller(); c.set("follow", "any", now=0)
    c.step([face("atharv", 5, 2.0)], FAR, now=0.1, dt=0.1)
    assert c.cmd.target == "atharv"
    a = c.step([face("rahul", -5, 1.5)], FAR, now=0.2, dt=0.1)   # someone else walks by
    assert control.pick_target([face("rahul", -5, 1.5)], c.cmd.target) is None and a.angular == 0


def test_follow_straight_never_turns():
    c = Controller(); c.set("follow_straight", "any", now=0)
    a = c.step([face("atharv", 10, 2.5)], FAR, now=0.1, dt=0.1)
    assert a.angular == 0 and a.linear > 0                       # ahead and far: drive straight
    a = c.step([face("atharv", 35, 2.5)], FAR, now=0.2, dt=0.1)
    assert a.angular == 0 and a.linear == 0                      # off to the side: stop, do not turn
    a = c.step([face("atharv", 0, 0.8)], FAR, now=0.3, dt=0.1)
    assert a.linear == 0                                         # close enough: stop
    a = c.step([], FAR, now=3.0, dt=0.1)
    assert a.linear == 0 and a.angular == 0                      # lost: stand still, no search spin
    assert intents.parse("follow me straight")["mode"] == "follow_straight"
    assert intents.parse("follow me")["mode"] == "follow"
