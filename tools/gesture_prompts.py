#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Spoken gesture prompts for a labelled hand tracking recording.

Speaks one instruction at a time ("right hand: pinch, palm down", "open", "fist", ...) through the default audio
output (espeak-ng), so whoever wears the headset needs no screen, and writes when each was asked for, on
CLOCK_MONOTONIC like the camera recording (PSVR2_RECORD_CAMERA) and tools/vrhandlog. The labels then tell the
offline analysis what the hand was really doing at every moment.

Output CSV: start_ns,end_ns,hand,gesture,orientation. start_ns is REACTION_S after the prompt began (time to react),
end_ns when the next prompt starts. Usage: gesture_prompts.py out.csv [rounds]
"""
import shutil
import subprocess
import sys
import time

REACTION_S = 1.0
HOLD_S = 3.0  # a gesture
REST_S = 2.0  # "open" between gestures
SYSTEM_S = 5.0  # the dashboard gesture: time to raise an open palm, then close it into a fist


def now_ns():
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def say(text):
    """Speak without waiting (the hold starts with the prompt); returns the process."""
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


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    rounds = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    steps = schedule(rounds)
    total = sum(s[3] for s in steps)
    with open(sys.argv[1], "w") as out:
        out.write("start_ns,end_ns,hand,gesture,orientation\n")
        p = say(f"Gesture test, about {int(total / 60) + 1} minutes. Starting in five seconds.")
        time.sleep(5.0)
        for hand, gesture, orientation, hold, text in steps:
            began = now_ns()
            p = say(text)
            time.sleep(hold)
            out.write(f"{began + int(REACTION_S * 1e9)},{now_ns()},{hand},{gesture},{orientation}\n")
            out.flush()
        say("Done. Thank you.")
        time.sleep(2.0)
    print(f"{len(steps)} prompts written to {sys.argv[1]}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
