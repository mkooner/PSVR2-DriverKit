#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Per-hand share of time SteamVR got a valid hand pose, from vrhandlog CSVs, over 5 s windows where at least one hand
was valid >= 20% (hands in use; leaves out hands-down / headset-off stretches). Same metric for every session.
usage: hand_parity.py vrhandlog-*.csv"""
import csv, sys
for path in sys.argv[1:]:
    rows = list(csv.DictReader(l for l in open(path, errors='replace') if not l.startswith('#')))
    if not rows or 'left_valid' not in rows[0]:
        continue
    win = {}
    for r in rows:
        try: t = float(r['t_s'])
        except (TypeError, ValueError): continue
        w = win.setdefault(int(t // 5), [0, 0, 0])
        w[0] += 1; w[1] += r['left_valid'] == '1'; w[2] += r['right_valid'] == '1'
    use = [w for w in win.values() if max(w[1], w[2]) >= 0.2 * w[0]]
    n = sum(w[0] for w in use)
    if not n:
        print(f"{path[10:25]}: no hand use"); continue
    l = 100 * sum(w[1] for w in use) / n; r = 100 * sum(w[2] for w in use) / n
    print(f"{path[10:25]}: in use {len(use)*5:4d} s of {len(win)*5:4d} s; left {l:5.1f}%  right {r:5.1f}%  gap {r-l:+5.1f}")
