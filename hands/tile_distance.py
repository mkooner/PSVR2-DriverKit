#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Distance of Mercury's detections by stereo (MERCURY_DETECTION_LOG from driver_ht_replay), and whether the hand they
started was then tracked: do real pickups and false ones separate by distance?
usage: tile_distance.py detection.log  (stereo R, T and the 640 px / 120 deg pinhole views of the PS VR2 setup)"""
import sys, math, collections
import numpy as np
R = np.array([[0.86326, 0.01051, -0.50465], [-0.00444, 0.99990, 0.01323], [0.50474, -0.00918, 0.86322]])
T = np.array([-0.0762, 0.0008, -0.0207])
f = 320.0 / math.tan(math.radians(60)); c = 320.0
D = collections.defaultdict(list); F = collections.defaultdict(dict); frames = []
for line in open(sys.argv[1]):
    p = line.strip().split(',')
    if p[0] == 'D':
        D[(int(p[1]), int(p[4]))].append((int(p[2]), int(p[3]), float(p[5]), float(p[6]), float(p[7]), float(p[8])))
    elif p[0] == 'F':
        F[int(p[1])][int(p[2])] = int(p[3]); frames.append(int(p[1]))
frames = sorted(set(frames)); idx = {t: i for i, t in enumerate(frames)}
def ray(u, v): d = np.array([(u - c) / f, (v - c) / f, 1.0]); return d / np.linalg.norm(d)
def triangulate(a, b):
    """a: (u, v) in view 0, b: (u, v) in view 1 -> distance from camera 0 (m) and ray gap (m)."""
    d0 = ray(*a); c1 = -R.T @ T; d1 = R.T @ ray(*b)
    w = -c1; A = np.array([[d0 @ d0, -d0 @ d1], [d0 @ d1, -d1 @ d1]]); rhs = np.array([d0 @ (c1), d1 @ (c1)])
    try: s, t = np.linalg.solve(A, rhs)
    except np.linalg.LinAlgError: return None, None
    p0 = s * d0; p1 = c1 + t * d1
    return float(np.linalg.norm((p0 + p1) / 2)), float(np.linalg.norm(p0 - p1))
rows = []
for (ts, hand), dets in D.items():
    tiles = [d for d in dets if d[1] >= 0 and d[2] >= 0.55]
    if not tiles: continue
    i = idx.get(ts)
    if i is None: continue
    was = F[frames[i - 1]].get(hand, 0) if i > 0 else 0
    if was: continue  # only pickups (the hand wasn't tracked the frame before)
    best = max(tiles, key=lambda d: d[2])
    after = [F[frames[j]].get(hand, 0) for j in range(i, min(i + 60, len(frames)))]
    kept = sum(after) / max(len(after), 1)
    other = [d for d in dets if d[0] != best[0] and d[2] >= 0.3]
    dist = gap = None
    if other:
        o = max(other, key=lambda d: d[2])
        a, b = ((best[3], best[4]), (o[3], o[4])) if best[0] == 0 else ((o[3], o[4]), (best[3], best[4]))
        dist, gap = triangulate(a, b)
    rows.append((ts, hand, best[0], best[2], best[5], kept, dist, gap))
real = [r for r in rows if r[5] >= 0.9]; false = [r for r in rows if r[5] <= 0.3]; mid = [r for r in rows if 0.3 < r[5] < 0.9]
print(f"tile pickups (score >= 0.55, hand not tracked the frame before): {len(rows)}; then tracked >=90% of the next second: {len(real)}, <=30%: {len(false)}, in between: {len(mid)}")
for name, grp in (("tracked after (likely real)", real), ("dropped after (likely false)", false), ("in between", mid)):
    st = [r for r in grp if r[6] is not None]
    ds = sorted(r[6] for r in st)
    q = lambda a, p: a[int(p * (len(a) - 1))] if a else float('nan')
    print(f"  {name:28s}: {len(grp):4d}; seen by the other camera {len(st)} ({len(st)/max(len(grp),1):.0%}); stereo distance median {q(ds,.5):.2f} m, 10% {q(ds,.1):.2f}, 90% {q(ds,.9):.2f}; box size median {q(sorted(r[4] for r in grp),.5):.0f} px")
for lim in (0.6, 0.8, 1.0, 1.2):
    keep_r = sum(1 for r in real if r[6] is None or r[6] <= lim); keep_f = sum(1 for r in false if r[6] is None or r[6] <= lim)
    keep_r2 = sum(1 for r in real if r[6] is not None and r[6] <= lim); keep_f2 = sum(1 for r in false if r[6] is not None and r[6] <= lim)
    print(f"  rule 'stereo distance <= {lim} m': keeps real {keep_r2}/{len(real)}, false {keep_f2}/{len(false)};  same but single-camera pickups allowed: real {keep_r}/{len(real)}, false {keep_f}/{len(false)}")
