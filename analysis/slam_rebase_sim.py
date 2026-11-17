#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Simulate the headset SLAM reset handling (psvr2.c, slam rebase) on recorded runs.

The headset's SLAM sometimes re-origins its position at the current head (the position becomes exactly 0, the
orientation carries on). After steady tracking (>= STEADY_S since the last reset) the driver keeps the position
continuous across such a reset: it adds an offset so the new position is where the old one was heading. Resets
during startup or in a reset loop are left alone (continuing would keep accumulated drift) and counted as a tracking
world change, which drops the room calibration.

Reports per run: resets, rebased, world changes; the largest head step after the first 10 s with and without.
Usage: slam_rebase_sim.py record-*.csv
"""
import sys
import numpy as np

STEADY_S = 5.0
MAX_SPEED = 2.0      # m/s, clamp for the continuation velocity
ORIGIN_EPS = 1e-4    # m, "exactly at the origin"
MIN_JUMP = 0.005     # m, the previous position must be away from the origin

def simulate(S):
    t = S[:, 0] * 1e-9
    raw = S[:, 2:5]
    off = np.zeros(3)
    out = np.zeros_like(raw)
    last_reset = None  # time of the last reset (rebased or not)
    start = t[0]
    resets = rebased = world_changes = 0
    for i in range(len(raw)):
        p = raw[i]
        if i > 0 and np.linalg.norm(p) < ORIGIN_EPS and np.linalg.norm(raw[i-1]) > MIN_JUMP:
            resets += 1
            since = t[i] - (last_reset if last_reset is not None else start)
            if since >= STEADY_S and i >= 2:
                dt_prev = t[i-1] - t[i-2]
                v = (out[i-1] - out[i-2]) / dt_prev if dt_prev > 0 else np.zeros(3)
                sp = np.linalg.norm(v)
                if sp > MAX_SPEED:
                    v *= MAX_SPEED / sp
                predicted = out[i-1] + v * (t[i] - t[i-1])
                off = predicted - p
                rebased += 1
            else:
                world_changes += 1
            last_reset = t[i]
        out[i] = p + off
    return t, raw, out, resets, rebased, world_changes

def max_step_after(t, x, after=10.0):
    m = (t - t[0]) > after
    d = np.linalg.norm(np.diff(x, axis=0), axis=1)
    d = d[m[1:]]
    return d.max() if len(d) else 0.0, int((d > 0.05).sum())

for path in sys.argv[1:]:
    S = np.array([[float(x) for x in l.split(',')[1:10]] for l in open(path) if l.startswith('S,')])
    if len(S) < 100:
        continue
    t, raw, out, r, rb, wc = simulate(S)
    a, an = max_step_after(t, raw)
    b, bn = max_step_after(t, out)
    drift = np.linalg.norm(out[-1] - raw[-1])
    print(f'{path.split("/")[-1][7:22]} {t[-1]-t[0]:5.0f}s: resets {r:2d}, rebased {rb:2d}, world changes {wc:2d}; '
          f'after 10 s max step {a*100:5.1f} -> {b*100:5.1f} cm, steps > 5 cm {an} -> {bn}; final offset {drift*100:5.1f} cm')
