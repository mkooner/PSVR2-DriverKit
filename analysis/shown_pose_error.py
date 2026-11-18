#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""
Head-motion error left in each shown frame, in the plugin's driver direct mode (logs/direct-<stamp>.csv).

For every frame Monado's compositor showed (R rows) the pose the game rendered with and the device's own prediction
for the frame's display time are compared with where the head really was at that time (T rows: the device's pose
sampled at Present and PostPresent, interpolated to the display time). All poses are in the head device's space.

Monado's compositor re-warps each frame's rotation from the render pose to its own latest prediction, so:
  - rotation: what remains after the re-warp is about Monado's own prediction error (the "pred" columns),
  - position: is not re-warped, the render pose's position error shows as it is.
The frame-to-frame change of these errors is what shows as jitter; the steady part is latency.

Usage: shown_pose_error.py <direct-*.csv>
"""
import math
import sys

import numpy as np


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def qconj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def rotvec_deg(q):
    if q[3] < 0:
        q = -q
    s = np.linalg.norm(q[:3])
    if s < 1e-12:
        return np.zeros(3)
    return q[:3] / s * math.degrees(2 * math.atan2(s, q[3]))


def slerp(a, b, t):
    d = float(np.dot(a, b))
    if d < 0:
        b, d = -b, -d
    if d > 0.9995:
        q = a + t * (b - a)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - t) * th) * a + math.sin(t * th) * b) / math.sin(th)


def load(path):
    R, T = [], []
    for line in open(path):
        if line.startswith('R,'):
            x = line.strip().split(',')
            R.append((int(x[1]), int(x[2]), int(x[3]), np.array([float(a) for a in x[4:11]]),
                      np.array([float(a) for a in x[11:18]])))
        elif line.startswith('T,'):
            x = line.strip().split(',')
            T.append((int(x[1]), np.array([float(a) for a in x[2:9]])))
    T.sort(key=lambda r: r[0])
    return R, T


def truth_at(tt, T, t, max_gap):
    i = np.searchsorted(tt, t)
    if i <= 0 or i >= len(tt):
        return None
    t0, t1 = tt[i - 1], tt[i]
    if t1 - t0 > max_gap:  # no samples around it (SteamVR stalled)
        return None
    a = (t - t0) / (t1 - t0)
    p0, p1 = T[i - 1][1], T[i][1]
    return np.r_[p0[:3] + a * (p1[:3] - p0[:3]), slerp(p0[3:] / np.linalg.norm(p0[3:]), p1[3:] / np.linalg.norm(p1[3:]), a)]


def errors(pose, truth):
    rot = rotvec_deg(qmul(pose[3:] / np.linalg.norm(pose[3:]), qconj(truth[3:])))
    pos = 1000 * (pose[:3] - truth[:3])
    return rot, pos


def analyse(R, T):
    tt = np.array([r[0] for r in T], dtype=np.float64)
    # Samples come twice a frame; a gap much longer than usual means SteamVR stalled there.
    max_gap = 3 * float(np.median(np.diff(tt))) if len(tt) > 2 else 30e6
    max_gap = max(max_gap, 15e6)
    rows = []
    for host, fid, disp, render, pred in R:
        tr = truth_at(tt, T, disp, max_gap)
        tr_prev = truth_at(tt, T, disp - 11e6, max_gap)
        if tr is None or tr_prev is None:
            continue
        if np.linalg.norm(tr[:3]) < 1e-6 or np.linalg.norm(tr_prev[:3]) < 1e-6:
            continue  # not tracked yet (startup): the device reports position 0
        rr, rp = errors(render, tr)
        pr, pp = errors(pred, tr)
        speed = np.linalg.norm(rotvec_deg(qmul(tr[3:], qconj(tr_prev[3:])))) / 0.011
        lin = np.linalg.norm(tr[:3] - tr_prev[:3]) / 0.011 * 1000
        rows.append((disp, rr, rp, pr, pp, speed, lin))
    return rows


def pc(v, p):
    return float(np.percentile(v, p)) if len(v) else float('nan')


def report(rows):
    if len(rows) < 10:
        print("too few frames with the true pose around their display time")
        return
    rr = np.array([r[1] for r in rows]); rp = np.array([r[2] for r in rows])
    pr = np.array([r[3] for r in rows]); pp = np.array([r[4] for r in rows])
    speed = np.array([r[5] for r in rows]); lin = np.array([r[6] for r in rows])
    n = len(rows)
    print(f"{n} shown frames (head speed: median {pc(speed, 50):.1f} deg/s, {pc(lin, 50):.0f} mm/s)")

    def block(name, m):
        if m.sum() < 5:
            return
        mm = m[1:] & m[:-1]
        print(f"  {name} ({m.sum()} frames)")
        for label, rot, pos in (("game's render pose (SteamVR's prediction)", rr, rp),
                                ("device's own prediction at submit       ", pr, pp)):
            ra = np.linalg.norm(rot[m], axis=1); pa = np.linalg.norm(pos[m], axis=1)
            rj = np.linalg.norm(np.diff(rot, axis=0)[mm], axis=1)
            pj = np.linalg.norm(np.diff(pos, axis=0)[mm], axis=1)
            print(f"    {label}: rotation off median {pc(ra, 50):.3f} p90 {pc(ra, 90):.3f} deg, frame-to-frame "
                  f"{pc(rj, 50):.3f} / p90 {pc(rj, 90):.3f} deg; position off median {pc(pa, 50):.2f} p90 "
                  f"{pc(pa, 90):.2f} mm, frame-to-frame {pc(pj, 50):.2f} / p90 {pc(pj, 90):.2f} mm")

    block("all", np.ones(n, bool))
    block("head still (<3 deg/s, <20 mm/s)", (speed < 3) & (lin < 20))
    block("turning (20-200 deg/s)", (speed > 20) & (speed < 200))
    block("moving (>100 mm/s)", lin > 100)
    print("  Rotation in the game's render pose is re-warped by Monado (what remains is about the device's own")
    print("  prediction error); position is not re-warped, so the render pose's position error is what you see.")


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    R, T = load(sys.argv[1])
    if not R or not T:
        print("no R/T rows in", sys.argv[1], "- needs the plugin's direct mode with Monado's compositor (2026-11-18)")
        return 1
    report(analyse(R, T))
    return 0


if __name__ == '__main__':
    sys.exit(main())
