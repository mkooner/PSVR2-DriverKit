#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Each camera's MediaPipe gesture readings during the prompts (MERCURY_DETECTION_LOG "G" lines from hand_sim with
MERCURY_MP_BOTH_VIEWS=1): does one camera tell pinch / fist / open apart better? Also the two cameras combined.
usage: gesture_views.py detection.log gesture-labels.csv replay_to_session_ns"""
import csv, sys, collections
log, lab, off = sys.argv[1], sys.argv[2], int(sys.argv[3])
G = collections.defaultdict(dict)  # (ts, hand) -> view -> (valid, presence, pinch, cat, score)
for line in open(log):
    if not line.startswith('G,'):
        continue
    _, ts, h, v, ok, pres, pinch, cat, sc, used = line.strip().split(',')
    G[(int(ts) + off, int(h))][int(v)] = (ok == '1', float(pres), float(pinch), int(cat), float(sc))
P = [r for r in csv.DictReader(open(lab)) if r['hand'] in ('left', 'right') and r['gesture'] in ('pinch', 'fist', 'open', 'claw', 'point')]
keys = sorted(G)
import bisect
tsl = [k[0] for k in keys]
def readings(r):
    h = 0 if r['hand'] == 'left' else 1
    a, b = int(r['start_ns']), int(r['end_ns'])
    i, j = bisect.bisect_left(tsl, a), bisect.bisect_right(tsl, b)
    return [G[keys[k]] for k in range(i, j) if keys[k][1] == h]
FIST = 1
for hand in ('left', 'right'):
    print(f"== {hand} hand (view 0 / view 1 / both combined: pinch = smaller distance, fist = either view)")
    for g in ('pinch', 'open', 'claw', 'point', 'fist'):
        rows = [x for r in P if r['hand'] == hand and r['gesture'] == g for x in readings(r)]
        out = []
        for name in ('v0', 'v1', 'both'):
            vals = []
            for x in rows:
                vs = [x[v] for v in (0, 1) if v in x and x[v][0] and x[v][1] >= 0.5]
                if name != 'both':
                    vs = [x[int(name[1])]] if int(name[1]) in x and x[int(name[1])][0] and x[int(name[1])][1] >= 0.5 else []
                if not vs:
                    continue
                d = min(y[2] for y in vs)
                fist = any(y[3] == FIST and y[4] >= 0.5 for y in vs)
                vals.append((d, fist))
            if vals:
                below = 100 * sum(d < 0.5 for d, f in vals) / len(vals)
                fr = 100 * sum(f for d, f in vals) / len(vals)
                mean = sum(d for d, f in vals) / len(vals)
                out.append(f"{name}: n {len(vals):4d} dist {mean:.2f} <0.5 {below:3.0f}% fist {fr:3.0f}%")
            else:
                out.append(f"{name}: none")
        print(f"  {g:6s} " + " | ".join(out))
