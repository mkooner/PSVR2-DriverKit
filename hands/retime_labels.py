#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Re-time gesture labels to when the hand really changed: gesture_prompts.py labelled each prompt from a fixed
reaction time after the speech began, but long prompts take up to 3 s to say and people act when they've heard them
(2026-11-25 19:51: "pinch with your hand up at eye height", 2.44 s, the pinch appeared ~3.0 s in and lasted into the
next "open" label). New window: from this prompt's speech end + REACTION to the next prompt's speech end + REACTION.
Speech lengths from espeak-ng (as spoken, -s 150). The prompt list is rebuilt from tools/gesture_prompts.py and checked
against the label rows.
usage: retime_labels.py labels.csv set out.csv [reaction_s]   (set: benchmark | pinch | fist | pointclaw)"""
import csv, os, subprocess, sys, tempfile, wave, importlib.util
labels, which, out = sys.argv[1:4]
REACTION = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5
here = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('gp', os.path.join(here, '..', 'tools', 'gesture_prompts.py'))
gp = importlib.util.module_from_spec(spec); spec.loader.exec_module(gp)
steps = [s + (gp.REACTION_S,) for s in gp.schedule(2)] if which == 'benchmark' else gp.focus_steps(which, 1)
rows = list(csv.DictReader(open(labels)))
assert len(rows) == len(steps), f"{len(rows)} label rows vs {len(steps)} prompts"
for r, s in zip(rows, steps):
    assert (r['hand'], r['gesture'], r['orientation']) == (s[0], s[1], s[2]), (r, s[:3])
cache = {}
def spoken(text):
    if text not in cache:
        with tempfile.NamedTemporaryFile(suffix='.wav') as f:
            subprocess.run(['espeak-ng', '-s', '150', '-w', f.name, text], check=True)
            w = wave.open(f.name); cache[text] = w.getnframes() / w.getframerate()
    return cache[text]
began = [int(r['start_ns']) - int(s[5] * 1e9) for r, s in zip(rows, steps)]
heard = [b + int((spoken(s[4]) + REACTION) * 1e9) for b, s in zip(began, steps)]
with open(out, 'w', newline='') as o:
    w = csv.writer(o); w.writerow(['start_ns', 'end_ns', 'hand', 'gesture', 'orientation'])
    for i, (r, s) in enumerate(zip(rows, steps)):
        end = heard[i + 1] if i + 1 < len(rows) else int(r['end_ns']) + int((spoken(s[4]) + REACTION) * 1e9)
        w.writerow([heard[i], max(end, heard[i] + 1), r['hand'], r['gesture'], r['orientation']])
shift = [(h - int(r['start_ns'])) / 1e9 for h, r in zip(heard, rows)]
print(f"{len(rows)} labels re-timed; start moved by {min(shift):+.2f} to {max(shift):+.2f} s (mean {sum(shift)/len(shift):+.2f})", file=sys.stderr)
