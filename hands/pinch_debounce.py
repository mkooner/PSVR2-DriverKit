#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Pinch (trigger) rules on hand_sim's per-frame MediaPipe readings, scored by what's felt: pinch prompts caught, new
presses in non-pinch prompts, glitch presses (held < 0.15 s), re-presses (< 0.15 s after letting go).
usage: pinch_debounce.py sim.csv vrhandlog.csv labels.csv"""
import csv, sys
sim, log, lab = sys.argv[1:4]
t0 = next(int(l.split('t0_mono_ns')[1]) for l in open(log) if 't0_mono_ns' in l)
L = list(csv.DictReader(open(lab)))
rows = list(csv.DictReader(open(sim)))
def run(side, on, off, press_ms, release_ms, fist_block_ms):
    """Pinch on below `on` for press_ms, off above `off` for release_ms; a fist (Closed_Fist, 2 readings) blocks and
    releases; no press within fist_block_ms after the hand starts closing into a fist (needs a look-ahead: the pinch
    waits press_ms anyway, a fist seen in that time cancels it)."""
    out = []; pin = False; cand = None; rel = None; fist = False; fc = 0; last_key = None
    for r in rows:
        t = float(r['t_s'])
        valid = r[side + '_valid'] == '1' and r[side + '_mp_valid'] == '1'
        if not valid:
            if pin: pin = False
            cand = rel = None; fist = False; fc = 0; out.append((t, pin)); continue
        d = float(r[side + '_mp_pinch']); c = int(r[side + '_mp_category']); sc = float(r[side + '_mp_score'])
        key = (d, c, sc)
        if key != last_key:  # a new camera frame
            isf = c == 1 and sc >= 0.5
            fc = fc + 1 if isf != fist else 0
            if fc >= 2: fist = not fist; fc = 0
            last_key = key
        if fist:
            pin = False; cand = rel = None
        elif not pin:
            if d < on:
                cand = cand if cand is not None else t
                if (t - cand) * 1000 >= press_ms: pin = True; rel = None
            else:
                cand = None
        else:
            if d > off:
                rel = rel if rel is not None else t
                if (t - rel) * 1000 >= release_ms: pin = False; cand = None
            else:
                rel = None
        out.append((t, pin))
    return out
def score(series_l, series_r):
    res = {}
    for side, S in (('left', series_l), ('right', series_r)):
        presses = []; down = None; prev = False
        for t, v in S:
            if v and not prev: down = t
            if prev and not v and down is not None: presses.append((down, t))
            prev = v
        short = sum(1 for a, b in presses if b - a < 0.15)
        repress = sum(1 for (a1, b1), (a2, b2) in zip(presses, presses[1:]) if a2 - b1 < 0.15)
        pin = pin_n = false = false_n = 0
        for p in L:
            if p['hand'] != side: continue
            a = (int(p['start_ns']) - t0) / 1e9; b = (int(p['end_ns']) - t0) / 1e9 + 1
            new = any(a <= s <= b for s, e in presses)
            if p['gesture'] == 'pinch': pin_n += 1; pin += any(s <= b and e >= a for s, e in presses)
            if p['gesture'] in ('open', 'fist', 'point', 'claw'): false_n += 1; false += new
        res[side] = (len(presses), short, repress, pin, pin_n, false, false_n)
    return res
print("rule: presses, glitch (<0.15 s), re-pressed (<0.15 s), pinch prompts caught, new presses in non-pinch prompts")
for on, off in ((0.50, 0.65), (0.45, 0.65), (0.40, 0.60)):
    for press_ms, release_ms in ((33, 33), (60, 60), (80, 80), (100, 100), (80, 150), (120, 120)):
        r = score(run('left', on, off, press_ms, release_ms, 0), run('right', on, off, press_ms, release_ms, 0))
        txt = "  ".join(f"{s[0]}: {v[0]:2d} presses, glitch {v[1]:2d}, re {v[2]:2d}, pinch {v[3]}/{v[4]}, false {v[5]:2d}/{v[6]}" for s, v in r.items())
        print(f"  on<{on} off>{off} press {press_ms:3d} ms release {release_ms:3d} ms | {txt}")
