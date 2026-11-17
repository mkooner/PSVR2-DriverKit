#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Replay the room capture's floor measurement on recorded runs.

Finds stretches where a position-tracked Sense controller lay still (as the capture requires: drift < 1 cm, over 2.5 s;
turning is checked as < 2 degrees in total since the log has no gyro rate), and for each computes what the capture would: the lowest LED of the controller outline
(harness/outline.csv, the driver's own LED model) minus the LED radius, the grip position and heading. Two controllers
lying on the same surface should give the same height; the spread within one rest shows the measurement noise.

Usage: rest_floor.py <combined.log> [...]
"""
import math, re, sys
import numpy as np

HERE = __file__.rsplit('/', 1)[0]
OUT = {'L': [], 'R': []}
for line in open(f'{HERE}/harness/outline.csv'):
    h, x, y, z = line.strip().split(',')
    OUT[h].append((float(x), float(y), float(z)))
OUT = {k: np.array(v) for k, v in OUT.items()}
LED_R = 0.006

def rot(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

AXES = {'forward': np.array([0, 0, -1.0]), 'up': np.array([0, 1.0, 0]), 'right': np.array([1.0, 0, 0])}

def heading(R, axis='forward'):
    # As ovrd_room_heading_of_axis: the yaw of where that grip axis points, NaN within 30 degrees of vertical.
    f = R @ AXES[axis]
    h = math.hypot(f[0], f[2])
    return math.atan2(-f[0], -f[2]) if h > 0.5 else float('nan')

def most_level_axis(R):
    # As ovrd_room_most_level_axis (ties keep the earlier axis: forward, up, right).
    best = 'forward'
    for a in ('up', 'right'):
        if math.hypot(*(R @ AXES[a])[[0, 2]]) > math.hypot(*(R @ AXES[best])[[0, 2]]):
            best = a
    return best

pat = re.compile(r'FINAL (\d+) ([01]) flags ([0-9a-f]+) pos (\S+) (\S+) (\S+) rot (\S+) (\S+) (\S+) (\S+)')
for path in sys.argv[1:]:
    rows = {0: [], 1: []}
    for line in open(path, errors='replace'):
        m = pat.search(line)
        if m:
            t, hand, fl = int(m[1]), int(m[2]), int(m[3], 16)
            rows[hand].append((t, fl, *map(float, m.groups()[3:])))
    print(f'== {path}')
    for hand in (0, 1):
        r = rows[hand]
        side = 'L' if hand == 0 else 'R'
        i = 0
        rests = []
        while i < len(r):
            if (r[i][1] & 0x30) != 0x30:
                i += 1; continue
            j = i
            p0 = np.array(r[i][2:5]); q0 = r[i][5:9]
            while j + 1 < len(r) and (r[j+1][1] & 0x30) == 0x30 and np.linalg.norm(np.array(r[j+1][2:5]) - p0) < 0.01:
                # The driver checks the gyro rate; the logged orientation is noisier than that, so check the total
                # turn since the rest began instead (under 2 degrees).
                qa, qb = np.array(q0), np.array(r[j+1][5:9])
                if 2*math.acos(min(1, abs(float(qa @ qb)))) > math.radians(2): break
                j += 1
            dur = (r[j][0]-r[i][0])*1e-9
            if dur >= 2.5:
                lows, heads = [], []
                axis = most_level_axis(rot(r[i][5:9]))
                for k in range(i, j+1):
                    R = rot(r[k][5:9]); p = np.array(r[k][2:5])
                    lows.append(min((R @ o + p)[1] for o in OUT[side]) - LED_R)
                    heads.append(heading(R, axis))
                R = rot(r[i][5:9])
                up = R.T @ np.array([0, 1.0, 0])  # world up in grip frame: which side it rests on
                hs = np.array(heads)
                ok = np.isfinite(hs)
                mean = math.atan2(np.sin(hs[ok]).mean(), np.cos(hs[ok]).mean()) if ok.any() else float('nan')
                sd = math.degrees(np.sqrt(np.mean(np.angle(np.exp(1j * (hs[ok] - mean))) ** 2))) if ok.any() else float('nan')
                rests.append((r[i][0], dur, np.mean(lows), np.std(lows), mean, p0, up, axis, sd, ok.mean()))
                i = j + 1
            else:
                i += 1
        for t, dur, low, sd, hd, p0, up, axis, hsd, frac in rests:
            print(f'  {side} rest at {t/1e9:9.1f}s for {dur:5.1f}s: lowest point y {low:+.4f} m (sd {sd*1000:.1f} mm), '
                  f'grip {p0[0]:+.3f} {p0[1]:+.3f} {p0[2]:+.3f}, heading ({axis}) {math.degrees(hd):6.1f} deg '
                  f'(sd {hsd:.2f} deg, defined {frac*100:.0f}%), up in grip {up[0]:+.2f} {up[1]:+.2f} {up[2]:+.2f}')
