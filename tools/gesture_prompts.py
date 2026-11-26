#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Spoken gesture prompts for a labelled hand tracking recording.

Speaks one instruction at a time ("right hand: pinch, palm down", "open", "fist", ...) through the default audio
output (espeak-ng), so whoever wears the headset needs no screen, and writes when each was asked for, on
CLOCK_MONOTONIC like the camera recording (PSVR2_RECORD_CAMERA) and tools/vrhandlog. The labels then tell the
offline analysis what the hand was really doing at every moment.

Output CSV: start_ns,end_ns,hand,gesture,orientation. start_ns is when the spoken prompt has finished plus a short
reaction time (HEARD_REACTION_S; people act once they've heard the instruction: a long prompt takes up to 3 s to say,
and the 2026-11-25 sessions, labelled from the speech's start, were off by up to 3 s), end_ns the same moment of the
next prompt (the gesture is held until the next instruction is heard). The hold time runs from the end of the speech.
Usage: gesture_prompts.py out.csv [rounds]
"""
import shutil
import subprocess
import sys
import time

REACTION_S = 1.0  # the old labels' allowance after the speech began (hands/retime_labels.py re-times those)
HEARD_REACTION_S = 0.5  # after the speech ends (0.3 for the quick steps)
HOLD_S = 3.0  # a gesture
REST_S = 2.0  # "open" between gestures
SYSTEM_S = 5.0  # the dashboard gesture: time to raise an open palm, then close it into a fist


def now_ns():
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def say(text):
    """Start speaking; returns the process (wait() on it for the end of the speech)."""
    if shutil.which("espeak-ng") is None:
        print(f"PROMPT: {text}", flush=True)
        return None
    return subprocess.Popen(["espeak-ng", "-s", "150", text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


POSITION_S = 4.0  # a hand position (tracking section)


def tracking_steps():
    """Hand positions for left/right tracking parity: the same moves for each hand, one hand right after the other
    (a lowered hand was found for the right hand but lost for the left in free use). gesture "position", orientation
    names the position; "both" steps move both hands."""
    steps = []
    for hand in ("left", "right"):
        for where, text in (
            ("front", f"{hand} hand open in front of you, the other hand down"),
            ("low_front", f"lower your {hand} hand to belly height, still open"),
            ("side", f"drop your {hand} hand down by your side, relaxed"),
            ("raise_slow", f"slowly raise your {hand} hand up in front of you"),
            ("lower_slow", f"slowly lower your {hand} hand to your side"),
            ("thigh", f"rest your {hand} hand on your thigh"),
            ("raise_fast", f"quickly raise your {hand} hand in front of you"),
            ("out_wide", f"move your {hand} hand out to the {hand}, at shoulder height"),
            ("high", f"raise your {hand} hand above your head height"),
        ):
            steps.append((hand, "position", where, POSITION_S, text))
    for where, text in (
        ("both_front", "both hands open in front of you"),
        ("both_side", "both hands down by your sides"),
        ("both_raise", "raise both hands in front of you"),
        ("both_low", "both hands at belly height"),
        ("both_raise", "raise both hands in front of you"),
        ("both_wave", "move both hands around slowly, like reaching for things"),
    ):
        steps.append(("both", "position", where, POSITION_S + (4.0 if where == "both_wave" else 0.0), text))
    return steps


def schedule(rounds):
    """(hand, gesture, orientation, hold seconds, spoken text): the hand position section, then the gestures, right
    hand first, each gesture followed by open."""
    steps = tracking_steps()
    for hand in ("right", "left"):
        steps.append((hand, "open", "any", REST_S + 1, f"{hand} hand. Hold it open in front of you"))
        for r in range(rounds):
            for gesture, orientation, text in (
                ("pinch", "palm_down", "pinch, palm down"),
                ("pinch", "palm_side", "pinch, palm sideways"),
                ("pinch", "palm_toward", "pinch, palm toward you"),
                ("fist", "any", "fist"),
                ("claw", "any", "claw, fingers bent, not touching"),
                ("point", "any", "point with your index finger"),
            ):
                steps.append((hand, gesture, orientation, HOLD_S, text))
                steps.append((hand, "open", "any", REST_S, "open"))
        # The dashboard gesture (raised open palm toward the face, then a fist), then closing it the same way.
        for action in ("open", "close"):
            steps.append((hand, "system", f"{action}_dashboard", SYSTEM_S,
                          f"{action} the dashboard: raise your open {hand} hand, palm toward your face, "
                          "then make a fist"))
            steps.append((hand, "open", "any", REST_S + 1, "let go, and lower your hand"))
    return steps


def focus_steps(group, rounds):
    """One gesture group at a time (GESTURE_PROMPTS_SET=<group>), so each can be made good before the next:
    pinch (pinch vs open hand: held, moving, high, low, palm up, slow pinches, quick taps and bursts of three; relaxed,
    half-open and moving open hands),
    fist (held and quick fists vs open), pointclaw (point and claw, which must not click). Both hands, left first,
    rounds times. Each step: (hand, gesture, orientation, seconds, spoken text, reaction seconds)."""
    steps = []
    for r in range(rounds):
        for hand in ("left", "right"):
            steps.append((hand, "open", "any", REST_S + 1, f"{hand} hand. Hold it open in front of you", REACTION_S))
            if group == "pinch":
                for gesture, orientation, hold, text in (
                    ("pinch", "hold", HOLD_S, "pinch and hold"),
                    ("open", "any", REST_S, "open"),
                    ("pinch", "hold_move", 5.0, "pinch and hold, and move your hand slowly left and right"),
                    ("open", "moving", 4.0, "open hand, keep moving it around"),
                    ("pinch", "hold_vertical", 5.0, "pinch and hold, and move your hand slowly up and down"),
                    ("open", "any", REST_S, "open"),
                    ("pinch", "high", HOLD_S, "pinch with your hand up at eye height"),
                    ("open", "high", REST_S, "open, keep it up"),
                    ("pinch", "low", HOLD_S, "pinch with your hand low, at belly height"),
                    ("open", "low", REST_S, "open, keep it low"),
                    ("pinch", "palm_up", HOLD_S, "pinch with your palm facing up"),
                    ("open", "palm_up", REST_S, "open, palm up"),
                    ("open", "relaxed", HOLD_S, "relax your hand, fingers loosely curled"),
                    ("open", "half", HOLD_S, "half open hand"),
                ):
                    steps.append((hand, gesture, orientation, hold, text, REACTION_S))
                # Slow pinches: the closing and opening are "transition" (not used to fit; they time the click).
                for _ in range(2):
                    steps.append((hand, "transition", "slow_close", 2.5,
                                  "slowly bring your thumb and index finger together", 0.5))
                    steps.append((hand, "pinch", "slow_hold", 2.0, "hold the pinch", 0.3))
                    steps.append((hand, "transition", "slow_open", 2.5, "slowly open", 0.5))
                    steps.append((hand, "open", "any", REST_S, "open", REACTION_S))
                # Fast pinches: single quick taps, then bursts of three ("burst": count the clicks, not fitted).
                steps.append((hand, "open", "tap_ready", 2.0, "quick taps. Pinch when I say pinch", REACTION_S))
                for _ in range(6):
                    steps.append((hand, "pinch", "tap", 0.7, "pinch", 0.35))
                    steps.append((hand, "open", "tap_release", 0.7, "open", 0.35))
                for _ in range(2):
                    steps.append((hand, "burst", "three", 3.0, "pinch three times quickly", 0.5))
                    steps.append((hand, "open", "any", REST_S, "open", REACTION_S))
            elif group == "fist":
                for gesture, orientation, hold, text in (
                    ("fist", "hold", HOLD_S, "make a fist and hold it"),
                    ("open", "any", REST_S, "open"),
                    ("fist", "hold_move", 5.0, "fist, and move your hand around"),
                    ("open", "moving", 4.0, "open hand, keep moving it around"),
                    ("open", "relaxed", HOLD_S, "relax your hand, fingers loosely curled"),
                ):
                    steps.append((hand, gesture, orientation, hold, text, REACTION_S))
                for _ in range(4):
                    steps.append((hand, "fist", "quick", 1.5, "fist", 0.4))
                    steps.append((hand, "open", "quick", 1.5, "open", 0.4))
            elif group == "pointclaw":
                for gesture, orientation, hold, text in (
                    ("point", "hold", HOLD_S, "point with your index finger"),
                    ("open", "any", REST_S, "open"),
                    ("point", "hold_move", 5.0, "point, and move your hand around"),
                    ("open", "any", REST_S, "open"),
                    ("claw", "hold", HOLD_S, "claw, fingers bent, not touching"),
                    ("open", "any", REST_S, "open"),
                    ("claw", "hold_move", 5.0, "claw, and move your hand around"),
                    ("open", "any", REST_S, "open"),
                ):
                    steps.append((hand, gesture, orientation, hold, text, REACTION_S))
            else:
                raise SystemExit(f"unknown GESTURE_PROMPTS_SET {group!r} (pinch, fist, pointclaw)")
            steps.append((hand, "open", "any", REST_S + 1, "let go, and lower your hand", REACTION_S))
    return steps


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    rounds = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    import os
    if os.environ.get("GESTURE_PROMPTS_SET"):
        # One round: the earlier benchmark is the other session to test against (shorter sessions, less video).
        steps = focus_steps(os.environ["GESTURE_PROMPTS_SET"], int(sys.argv[2]) if len(sys.argv) > 2 else 1)
    else:
        steps = [s + (REACTION_S,) for s in schedule(rounds)]
    total = sum(s[3] for s in steps)
    with open(sys.argv[1], "w") as out:
        out.write("start_ns,end_ns,hand,gesture,orientation\n")
        p = say(f"Gesture test, about {int(total / 60) + 1} minutes. Starting in five seconds.")
        time.sleep(5.0)
        prev = None  # (heard ns, hand, gesture, orientation) of the prompt before
        for hand, gesture, orientation, hold, text, reaction in steps:
            p = say(text)
            if p is not None:
                p.wait()  # the instruction has been heard
            heard = now_ns() + int((HEARD_REACTION_S if reaction >= HEARD_REACTION_S else 0.3) * 1e9)
            if prev is not None:
                out.write(f"{prev[0]},{heard},{prev[1]},{prev[2]},{prev[3]}\n")
                out.flush()
            prev = (heard, hand, gesture, orientation)
            time.sleep(hold)
        if prev is not None:
            out.write(f"{prev[0]},{now_ns()},{prev[1]},{prev[2]},{prev[3]}\n")
        say("Done. Thank you.")
        time.sleep(2.0)
    print(f"{len(steps)} prompts written to {sys.argv[1]}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
