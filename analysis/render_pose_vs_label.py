#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""
Does SteamVR label the game's frame with the pose the game rendered it with?

In the plugin's driver direct mode SteamVR hands the game's own eye images to our driver (SubmitLayer, layer 0) with a
head pose (mHmdPose, logged as S rows in logs/direct-<stamp>.csv). Monado's compositor re-warps each image from that
pose to the latest one, so if the label differs from the pose the game actually rendered with (the view poses the game
submitted in xrEndFrame, V rows of tools/xr_frame_log), the difference is never corrected and shows as image error.

The two poses are in different spaces (the game's OpenXR reference space, SteamVR's world) that differ by a constant
rigid transform, so the constant part is estimated (median) and only the per-frame deviation from it is reported:
that deviation is how far each shown frame lands off. A constant lag at a constant turn rate is absorbed into the
constant part (it is latency, not jitter); the frame-to-frame change of the error is what shows as jitter.

Usage: render_pose_vs_label.py <xr-frames-*.csv> <direct-*.csv>
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


def qrot(q, v):
    return qmul(qmul(q, np.r_[v, 0.0]), qconj(q))[:3]


def qangle_deg(q):
    return math.degrees(2 * math.acos(min(1.0, abs(q[3]))))


def qrotvec_deg(q):
    """Rotation vector (axis * angle) in degrees, so errors of opposite sign stay apart."""
    if q[3] < 0:
        q = -q
    s = np.linalg.norm(q[:3])
    if s < 1e-12:
        return np.zeros(3)
    return q[:3] / s * math.degrees(2 * math.atan2(s, q[3]))


def load_game(path):
    """V rows: (call_ns, head position, head orientation) from the first projection layer."""
    out = []
    for line in open(path):
        if not line.startswith('V,'):
            continue
        x = line.strip().split(',')
        if len(x) < 24 or x[3] != '0':
            continue
        # V,call,displayTime,layer,space,swapchain,x,y,w,h, view0 p(3) q(4), view1 p(3) q(4)
        v0 = np.array([float(a) for a in x[10:17]])
        v1 = np.array([float(a) for a in x[17:24]])
        out.append((int(x[1]), (v0[:3] + v1[:3]) / 2, v0[3:7] / np.linalg.norm(v0[3:7])))
    return out


def load_labels(path):
    """S rows of layer 0 whose image belongs to the game (a swap set of the game's process)."""
    set_pid = {}
    for line in open(path):
        if line.startswith('C,'):
            x = line.strip().split(',')
            for h in x[7:10]:
                set_pid[h] = (x[2], int(x[4]), int(x[5]))
    pids = {}
    for pid, w, h in set_pid.values():
        pids.setdefault(pid, (w, h))
    out = []
    for line in open(path):
        if not line.startswith('S,'):
            continue
        x = line.strip().split(',')
        if x[2] != '0' or x[11] not in set_pid:
            continue
        out.append((int(x[1]), set_pid[x[11]], np.array([float(a) for a in x[4:7]]),
                    np.array([float(a) for a in x[7:11]]), float(x[3])))
    return out, pids


def analyse(game, labels, game_size=None):
    """Pairs each label with the game's newest submit before it; returns the per-frame deviations."""
    gt = np.array([g[0] for g in game])
    pairs = []
    for t, (pid, w, h), p, q, pred in labels:
        if game_size is not None and (w, h) != game_size:
            continue
        i = np.searchsorted(gt, t) - 1
        if i < 0:
            continue
        g = game[i]
        pairs.append((t, t - g[0], g[1], g[2], p, q / np.linalg.norm(q), i))
    if len(pairs) < 10:
        return None
    # Constant space change R: SteamVR = R * game. Estimate from all frames (quaternion average with sign fix).
    rs = []
    for _, _, gp, gq, p, q, _ in pairs:
        r = qmul(q, qconj(gq))
        if r[3] < 0:
            r = -r
        rs.append(r)
    R = np.median(np.array(rs), axis=0)
    R /= np.linalg.norm(R)
    ts = np.array([p - qrot(R, gp) for _, _, gp, gq, p, q, _ in pairs])
    T = np.median(ts, axis=0)
    ang, rv, perr, pos, speed, age, reuse = [], [], [], [], [], [], []
    prev = None
    for k, (t, dt, gp, gq, p, q, gi) in enumerate(pairs):
        dq = qmul(qmul(qconj(R), q), qconj(gq))  # label vs game, in the game's space
        ang.append(qangle_deg(dq))
        rv.append(qrotvec_deg(dq))
        perr.append(1000 * (p - qrot(R, gp) - T))
        pos.append(np.linalg.norm(perr[-1]))
        age.append(dt / 1e6)
        reuse.append(prev is not None and gi == prev[0])
        if prev is not None:
            speed.append(qangle_deg(qmul(gq, qconj(prev[1]))) / max(1e-3, (t - prev[2]) / 1e9))
        else:
            speed.append(0.0)
        prev = (gi, gq, t)
    rv = np.array(rv)
    perr = np.array(perr)
    return dict(n=len(pairs), ang=np.array(ang), pos=np.array(pos),
                jit_deg=np.linalg.norm(np.diff(rv, axis=0), axis=1), jit_mm=np.linalg.norm(np.diff(perr, axis=0), axis=1), speed=np.array(speed), age=np.array(age),
                reuse=np.array(reuse), R_deg=qangle_deg(R), T=T)


def report(r):
    def pc(v, p):
        return np.percentile(v, p) if len(v) else float('nan')

    print(f"{r['n']} game frames paired with SteamVR's label (constant space change {r['R_deg']:.1f} deg, "
          f"{np.linalg.norm(r['T']) * 1000:.0f} mm)")
    print(f"  game submit -> SteamVR SubmitLayer: median {pc(r['age'], 50):.1f} ms, p99 {pc(r['age'], 99):.1f} ms; "
          f"same game frame labelled again {100 * r['reuse'].mean():.1f}%")
    for name, m in (('all', np.ones(r['n'], bool)), ('head still (<5 deg/s)', r['speed'] < 5),
                    ('turning (20-200 deg/s)', (r['speed'] > 20) & (r['speed'] < 200))):
        a, p = r['ang'][m], r['pos'][m]
        if len(a) == 0:
            continue
        print(f"  {name:24s} {m.sum():5d} frames: label off by median {pc(a, 50):.3f} deg / {pc(p, 50):.2f} mm, "
              f"p90 {pc(a, 90):.3f} deg / {pc(p, 90):.2f} mm, p99 {pc(a, 99):.3f} deg / {pc(p, 99):.2f} mm")
    # Frame-to-frame change of the error: what shows as jitter after Monado's re-warp.
    j, jm = r['jit_deg'], r['jit_mm']
    print(f"  frame-to-frame change of the error (what shows as jitter): rotation median {pc(j, 50):.3f} deg, "
          f"p90 {pc(j, 90):.3f}, p99 {pc(j, 99):.3f}; position median {pc(jm, 50):.2f} mm, p90 {pc(jm, 90):.2f}, "
          f"p99 {pc(jm, 99):.2f}")
    for name, m in (('head still', r['speed'][1:] < 5), ('turning', (r['speed'][1:] > 20) & (r['speed'][1:] < 200))):
        if m.sum():
            print(f"    {name:10s}: rotation p50 {pc(j[m], 50):.3f} / p90 {pc(j[m], 90):.3f} deg, position p50 "
                  f"{pc(jm[m], 50):.2f} / p90 {pc(jm[m], 90):.2f} mm")


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    game = load_game(sys.argv[1])
    labels, pids = load_labels(sys.argv[2])
    if not game:
        print("no V rows (pose rows) in", sys.argv[1], "- the frame log layer needs the 2026-11-18 version")
        return 1
    # The game's swap sets: the process whose images match the game's submitted image size.
    size = None
    for line in open(sys.argv[1]):
        if line.startswith('V,'):
            x = line.split(',')
            size = (int(x[8]), int(x[9]))
            break
    r = analyse(game, labels, size)
    if r is None:
        print(f"no SteamVR layer-0 frames with the game's image size {size} in", sys.argv[2])
        return 1
    report(r)
    return 0


if __name__ == '__main__':
    sys.exit(main())
