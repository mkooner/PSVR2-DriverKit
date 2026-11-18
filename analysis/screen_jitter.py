#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""
On-screen jitter in driver direct mode: what Monado's compositor actually showed, per displayed frame, against where
the head really was when it lit up.

Inputs:
  compframes-<stamp>.csv  Monado compositor (XRT_COMPOSITOR_FRAME_LOG): W rows (re-warp head pose for the predicted
                          display time, the client frame used = layer 0 timestamp), P rows (present), B rows (vblanks)
  direct-<stamp>.csv      the plugin (OVRD_DIRECT_MODE_LOG): R rows (the pose the game drew each frame with, keyed by
                          the frame's display time = layer 0 timestamp), T rows (the head's pose every 2 ms)

Per displayed frame:
  shown time   first vblank after the present, plus the median (predicted display - that vblank) as the photon offset;
               without vblanks, the predicted display time
  slip         shown time - predicted display time (a frame landing on a different refresh than planned)
  rot error    re-warp head rotation vs the head at the shown time: what remains on screen for distant content
  pos error    position the game drew with (not re-warped) vs the head at the shown time
  repeat       the same client frame as the previous displayed frame (a game frame shown twice)
  age          shown time - the client frame's own display time (how stale the game frame on screen is)
Frame-to-frame changes of these errors are what shows as jitter; the angular size at 1 m / 0.5 m adds the position
error seen at that distance.

Usage: screen_jitter.py compframes.csv direct.csv
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


def floats(x):
    return np.array([float(v) for v in x])


def load(comp_path, direct_path):
    W, P, B, R, T = [], {}, [], {}, []
    for line in open(comp_path):
        x = line.strip().split(',')
        if x[0] == 'W' and len(x) >= 13:
            W.append((int(x[1]), int(x[2]), int(x[3]), int(x[4]), int(x[5]), floats(x[6:13])))
        elif x[0] == 'P' and len(x) >= 4:
            P[int(x[2])] = (int(x[1]), int(x[3]))
        elif x[0] == 'B' and len(x) >= 2:
            B.append(int(x[1]))
    for line in open(direct_path):
        x = line.strip().split(',')
        if x[0] == 'R' and len(x) >= 18:
            R[int(x[3])] = floats(x[4:11])
        elif x[0] == 'T' and len(x) >= 9:
            T.append((int(x[1]), floats(x[2:9])))
    T.sort(key=lambda r: r[0])
    B.sort()
    return W, P, np.array(B, dtype=np.int64), R, T


class Truth:
    def __init__(self, T):
        self.t = np.array([r[0] for r in T], dtype=np.float64)
        self.p = [r[1] for r in T]
        self.gap = max(3 * float(np.median(np.diff(self.t))) if len(self.t) > 2 else 10e6, 6e6)

    def at(self, t):
        i = np.searchsorted(self.t, t)
        if i <= 0 or i >= len(self.t) or self.t[i] - self.t[i - 1] > self.gap:
            return None
        a = (t - self.t[i - 1]) / (self.t[i] - self.t[i - 1])
        p0, p1 = self.p[i - 1], self.p[i]
        if np.linalg.norm(p0[:3]) < 1e-6 or np.linalg.norm(p1[:3]) < 1e-6:
            return None  # not tracked yet
        q = slerp(p0[3:] / np.linalg.norm(p0[3:]), p1[3:] / np.linalg.norm(p1[3:]), a)
        return np.r_[p0[:3] + a * (p1[:3] - p0[:3]), q]


def analyse(W, P, B, R, T):
    truth = Truth(T)
    # Photon offset: predicted display time - the vblank the frame was presented for (median), when vblanks exist.
    shown_vblank = {}
    if len(B) > 10:
        for _, fid, pred, _, _, _ in W:
            if fid in P:
                i = np.searchsorted(B, P[fid][0])
                if i < len(B):
                    shown_vblank[fid] = int(B[i])
    offs = [pred - shown_vblank[fid] for _, fid, pred, _, _, _ in W if fid in shown_vblank]
    photon = float(np.median(offs)) if offs else 0.0
    rows = []
    prev_ts = None
    for now, fid, pred, nlayers, ts, head in W:
        shown = shown_vblank[fid] + photon if fid in shown_vblank else float(pred)
        tr = truth.at(shown)
        tr_prev = truth.at(shown - 11.1e6)
        if tr is None or tr_prev is None:
            prev_ts = ts
            continue
        rot = rotvec_deg(qmul(head[3:] / np.linalg.norm(head[3:]), qconj(tr[3:])))
        render = R.get(ts)
        pos = 1000 * (render[:3] - tr[:3]) if render is not None else np.full(3, np.nan)
        speed = np.linalg.norm(rotvec_deg(qmul(tr[3:], qconj(tr_prev[3:])))) / 0.0111
        lin = 1000 * np.linalg.norm(tr[:3] - tr_prev[:3]) / 0.0111
        rows.append(dict(t=shown, fid=fid, slip=(shown - pred) / 1e6, age=(shown - ts) / 1e6, rot=rot, pos=pos,
                         repeat=(ts == prev_ts),
                         speed=speed, lin=lin, has_render=render is not None, vblank=fid in shown_vblank))
        prev_ts = ts
    shown_ts = set(w[4] for w in W)
    skipped = sum(1 for d in R if d not in shown_ts and W and W[0][4] <= d <= W[-1][4])
    return rows, photon, skipped


def pc(v, p):
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    return float(np.percentile(v, p)) if len(v) else float('nan')


def report(rows, photon, skipped, n_vblank):
    n = len(rows)
    if n < 10:
        print("too few displayed frames with the head's pose around them")
        return
    t0 = rows[0]['t']
    rot = np.array([r['rot'] for r in rows])
    pos = np.array([r['pos'] for r in rows])
    speed = np.array([r['speed'] for r in rows])
    lin = np.array([r['lin'] for r in rows])
    slip = np.array([r['slip'] for r in rows])
    rep = np.array([r['repeat'] for r in rows])
    has_v = np.mean([r['vblank'] for r in rows])
    print(f"{n} displayed frames, {(rows[-1]['t'] - t0) / 1e9:.0f} s; shown times from vblanks for "
          f"{100 * has_v:.0f}% ({n_vblank} vblanks; photon offset {photon / 1e6:+.2f} ms), else predicted")
    print(f"  game frames shown twice: {rep.sum()} ({100 * rep.mean():.2f}%); game frames never shown: {skipped}")
    print(f"  slip (shown - predicted): p1 {pc(slip, 1):+.2f} p50 {pc(slip, 50):+.2f} p99 {pc(slip, 99):+.2f} ms; "
          f"off by > half a refresh: {int((np.abs(slip) > 5.6).sum())}")
    age = np.array([r['age'] for r in rows])
    print(f"  age of the game frame on screen (shown - its display time): p50 {pc(age, 50):+.2f} p90 {pc(age, 90):+.2f} "
          f"p99 {pc(age, 99):+.2f} ms; older than 1.5 refreshes: {int((age > 16.7).sum())} frames")
    rot_sw = np.r_[np.nan, np.linalg.norm(np.diff(rot, axis=0), axis=1)]
    pos_sw = np.r_[np.nan, np.linalg.norm(np.diff(pos, axis=0), axis=1)]
    rot_a = np.linalg.norm(rot, axis=1)
    pos_a = np.linalg.norm(pos, axis=1)
    # Angular error of content at distance d: rotation error plus position error seen from d.
    ang1 = rot_a + np.degrees(pos_a / 1000.0)
    ang05 = rot_a + np.degrees(pos_a / 500.0)
    sw1 = rot_sw + np.degrees(pos_sw / 1000.0)
    groups = (('all', np.ones(n, bool)), ('head still (<3 deg/s, <20 mm/s)', (speed < 3) & (lin < 20)),
              ('turning (20-200 deg/s)', (speed > 20) & (speed < 200)), ('moving (>100 mm/s)', lin > 100))
    for name, m in groups:
        if m.sum() < 5:
            continue
        print(f"  {name} ({m.sum()} frames)")
        print(f"    rotation on screen   off p50 {pc(rot_a[m], 50):.3f} p90 {pc(rot_a[m], 90):.3f} p99 "
              f"{pc(rot_a[m], 99):.3f} deg; frame-to-frame p50 {pc(rot_sw[m], 50):.3f} p90 {pc(rot_sw[m], 90):.3f} "
              f"p99 {pc(rot_sw[m], 99):.3f} deg")
        print(f"    position on screen   off p50 {pc(pos_a[m], 50):.2f} p90 {pc(pos_a[m], 90):.2f} p99 "
              f"{pc(pos_a[m], 99):.2f} mm; frame-to-frame p50 {pc(pos_sw[m], 50):.2f} p90 {pc(pos_sw[m], 90):.2f} "
              f"p99 {pc(pos_sw[m], 99):.2f} mm")
        print(f"    content at 1 m / 0.5 m off p90 {pc(ang1[m], 90):.3f} / {pc(ang05[m], 90):.3f} deg; at 1 m "
              f"frame-to-frame p90 {pc(sw1[m], 90):.3f} p99 {pc(sw1[m], 99):.3f} deg; repeats "
              f"{int(rep[m].sum())}")
    # The largest frame-to-frame jumps at 1 m, with what happened in that frame.
    order = np.argsort(-np.nan_to_num(sw1, nan=-1))[:12]
    print("  largest frame-to-frame jumps (content at 1 m):")
    for i in sorted(order):
        if not np.isfinite(sw1[i]):
            continue
        cause = []
        if rep[i]:
            cause.append('game frame repeated')
        if i > 0 and rep[i - 1]:
            cause.append('after a repeat')
        if abs(slip[i]) > 5.6:
            cause.append(f'slip {slip[i]:+.1f} ms')
        if rot_sw[i] > 2 * np.degrees(pos_sw[i] / 1000.0):
            cause.append('mostly rotation')
        else:
            cause.append('mostly position')
        print(f"    t={(rows[i]['t'] - t0) / 1e9:7.2f} s  jump {sw1[i]:.3f} deg (rot {rot_sw[i]:.3f}, pos "
              f"{pos_sw[i]:.2f} mm), head {speed[i]:.0f} deg/s {lin[i]:.0f} mm/s: {', '.join(cause)}")


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    W, P, B, R, T = load(sys.argv[1], sys.argv[2])
    if not W or not T:
        print("need W rows (XRT_COMPOSITOR_FRAME_LOG) and T rows (OVRD_DIRECT_MODE_LOG)")
        return 1
    rows, photon, skipped = analyse(W, P, B, R, T)
    report(rows, photon, skipped, len(B))
    return 0


if __name__ == '__main__':
    sys.exit(main())
