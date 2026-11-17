#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""How to carry a Sense controller through a gap in optical fixes (pssense "OPT" debug lines).

For every gap between consecutive optical fixes (> 50 ms), predict where the controller reappears from the fixes
before the gap, and compare with where it did reappear (the jump the user sees on reacquisition, if the shown
position followed the prediction). Strategies: hold still; coast on the velocity of the last fixes (no speed cap),
decaying with a time constant tau (80 ms is the driver's current coast, which also zeroes the velocity above 2 m/s).
Usage: ctrl_gap_coast.py combined.log [...]
"""
import re, sys
import numpy as np

pat = re.compile(r'OPT (\d+) ([01]) (\S+) (\S+) (\S+) ')
allres = []
for path in sys.argv[1:]:
    rows = {0: {}, 1: {}}
    for line in open(path, errors='replace'):
        m = pat.search(line)
        if m:
            rows[int(m[2])][int(m[1])] = np.array([float(m[3]), float(m[4]), float(m[5])])
    for h in (0, 1):
        ts = np.array(sorted(rows[h])); P = np.array([rows[h][t] for t in ts])
        for i in range(3, len(ts) - 1):
            gap = (ts[i+1] - ts[i]) * 1e-9
            if gap < 0.05 or gap > 1.0:
                continue
            # velocity from the last fixes before the gap (they must be consecutive frames, <= 40 ms apart)
            if (ts[i] - ts[i-2]) * 1e-9 > 0.08:
                continue
            v = (P[i] - P[i-2]) / ((ts[i] - ts[i-2]) * 1e-9)
            truth = P[i+1]
            allres.append((gap, np.linalg.norm(v), P[i], v, truth))

def predict(p, v, gap, tau, cap):
    sp = np.linalg.norm(v)
    if cap is not None and sp >= cap:
        v = v * 0
    if tau is None:
        travel = gap
    elif tau == 0:
        travel = 0
    else:
        travel = tau * (1 - np.exp(-gap / tau))
    return p + v * travel

strategies = [('hold', 0, None), ('current (80 ms, 0 above 2 m/s)', 0.08, 2.0), ('coast 80 ms', 0.08, None),
              ('coast 150 ms', 0.15, None), ('coast 300 ms', 0.30, None), ('constant velocity', None, None)]
print(f'{len(allres)} gaps from {len(sys.argv)-1} runs')
for (glo, ghi) in [(0.05, 0.15), (0.15, 0.3), (0.3, 1.0)]:
    for (slo, shi) in [(0, 1.0), (1.0, 2.0), (2.0, 100)]:
        sel = [r for r in allres if glo <= r[0] < ghi and slo <= r[1] < shi]
        if len(sel) < 5:
            continue
        line = f'gap {glo*1000:.0f}-{ghi*1000:.0f} ms, speed {slo}-{shi} m/s ({len(sel)}):'
        for name, tau, cap in strategies:
            e = [np.linalg.norm(predict(p, v, g, tau, cap) - truth) * 100 for g, s, p, v, truth in sel]
            line += f' | {name} {np.median(e):.1f}/{np.percentile(e, 90):.1f}'
        print(line)
print('(reacquisition jump in cm, median/p90)')
