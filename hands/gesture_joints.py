#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Do Mercury's fitted 3D hands tell the prompted gestures apart? From driver_ht_replay HT_REPLAY_JOINTS (OpenXR
joints, head frame) and the benchmark's gesture labels: per prompt type, thumb-index tip distance / palm width and
finger curl (fingertip-to-wrist / knuckle-to-wrist, mean of index..little; ~2 straight, ~1 curled into a fist).
usage: gesture_joints.py joints.csv gesture-labels.csv first_recording_frame_ns"""
import csv, sys, math, bisect, collections
jf, lab, first = sys.argv[1], sys.argv[2], int(sys.argv[3])
T = {0: [], 1: []}
def P(r, j): return tuple(float(r[3 + j * 8 + k]) for k in range(3))
def dist(a, b): return math.dist(a, b)
for line in open(jf):
    r = line.split(',')
    if r[2] != '1':
        continue
    h = int(r[1]); t = first + float(r[0]) * 1e9
    wrist = P(r, 1)
    palm_w = dist(P(r, 7), P(r, 22))
    if palm_w < 1e-4:
        continue
    pinch = dist(P(r, 5), P(r, 10)) / palm_w
    curls = [dist(P(r, tip), wrist) / max(dist(P(r, prox), wrist), 1e-4) for tip, prox in ((10, 7), (15, 12), (20, 17), (25, 22))]
    T[h].append((t, pinch, sum(curls) / 4, curls[0]))
ts = {h: [x[0] for x in T[h]] for h in T}
rows = [r for r in csv.DictReader(open(lab)) if r['hand'] in ('left', 'right') and r['gesture'] in ('pinch', 'fist', 'open', 'claw', 'point')]
q = lambda a, p: sorted(a)[int(p * (len(a) - 1))] if a else float('nan')
for hand in ('left', 'right'):
    h = 0 if hand == 'left' else 1
    print(f"== {hand} hand: pinch distance / palm width (mean, 10%-90%) | finger curl, all four (mean) | index curl")
    for g in ('pinch', 'open', 'claw', 'point', 'fist'):
        vals = []
        for r in rows:
            if r['hand'] != hand or r['gesture'] != g:
                continue
            i, j = bisect.bisect_left(ts[h], int(r['start_ns'])), bisect.bisect_right(ts[h], int(r['end_ns']))
            vals += T[h][i:j]
        if not vals:
            print(f"  {g:6s} no tracked frames"); continue
        p = [v[1] for v in vals]; c = [v[2] for v in vals]; ic = [v[3] for v in vals]
        print(f"  {g:6s} n {len(vals):4d}  pinch {sum(p)/len(p):.2f} ({q(p,.1):.2f}-{q(p,.9):.2f})  curl {sum(c)/len(c):.2f} ({q(c,.1):.2f}-{q(c,.9):.2f})  index {sum(ic)/len(ic):.2f}")
