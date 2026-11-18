#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Which moment of the headset's motion each frame was drawn with, and whether that jumps by a whole refresh.

For every frame vrframetiming logged (the pose the game rendered it with, m_HmdPose), find the time in the headset pose
stream whose orientation matches it best (to 0.25 ms), relative to the frame's own time. Normally that is a steady
lead; frames drawn one refresh early or late show as separate peaks one frame period away. While the head turns, such
a frame shows the view one frame's motion behind or ahead of its neighbours: a small jitter that vanishes when still.

Pose stream: our plugin's OVRD_POSE_LOG, or vrframetiming's own poses.csv (SteamVR's pose, works with any driver).
Usage: steamvr_frame_shift.py poses.csv frames.csv [--min-speed 40]
"""
import argparse
import importlib.util
import os

import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("tl", os.path.join(here, "steamvr_timeline.py"))
tl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tl)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("poses")
    ap.add_argument("frames")
    ap.add_argument("--min-speed", type=float, default=40, help="only frames turning faster than this (deg/s)")
    a = ap.parse_args()

    tp, qp, pp, _, valid = tl.load_pose_log(a.poses)
    ok = valid > 0
    tp, qp = tp[ok], qp[ok]
    fr = tl.load_frames(a.frames)
    links = []
    with open(a.frames) as f:
        for l in f:
            if l.startswith("# link "):
                x = l.split()
                links.append(float(x[2]) - float(x[3]))
    if len(links) > 10:
        links = np.array(links)
        b = np.round(links / 0.0005)
        v, c = np.unique(b, return_counts=True)
        off = float(np.median(links[np.abs(b - v[np.argmax(c)]) <= 2]))
        how = "exact (vsync link)"
    else:
        off = float(np.min(fr["mono_s"] - fr["t_s"]))
        how = "approximate (no vsync link in this file): absolute lead may be off by a frame, the peaks are not"
    tf = fr["t_s"] + off
    M = np.stack([np.c_[fr["m00"], fr["m01"], fr["m02"]], np.c_[fr["m10"], fr["m11"], fr["m12"]],
                  np.c_[fr["m20"], fr["m21"], fr["m22"]]], 1)
    keep = (fr["pose_valid"] > 0) & (tf > tp[0] + 1) & (tf < tp[-1] - 1)
    tf, M, period = tf[keep], M[keep], np.median(np.diff(fr["t_s"]))
    for i in range(1, len(qp)):
        if np.dot(qp[i], qp[i - 1]) < 0:
            qp[i] = -qp[i]

    def rot_at(t):
        q = np.stack([np.interp(t, tp, qp[:, k]) for k in range(4)], 1)
        q /= np.linalg.norm(q, axis=1, keepdims=True)
        return tl.quat_to_mat(q)

    lags = np.arange(-0.05, 0.0601, 0.00025)
    err = np.empty((len(lags), len(tf)))
    for k, L in enumerate(lags):
        err[k] = np.degrees(np.linalg.norm(tl.rotvec(M @ np.transpose(rot_at(tf + L), (0, 2, 1))), axis=1))
    R = rot_at(tf)
    speed = np.r_[0, np.degrees(np.linalg.norm(tl.rotvec(R[1:] @ np.transpose(R[:-1], (0, 2, 1))), axis=1)) / np.diff(tf)]
    best = lags[np.argmin(err, 0)] * 1000
    m = speed > a.min_speed
    if m.sum() < 20:
        print("not enough turning frames")
        return
    lead = np.median(best[m])
    d = best[m] - lead
    p = period * 1000
    early, late, usual = (d < -0.6 * p).mean(), (d > 0.6 * p).mean(), (np.abs(d) < 0.3 * p).mean()
    print(f"{m.sum()} frames turning > {a.min_speed:.0f} deg/s, refresh {p:.2f} ms, clock link {how}")
    print(f"drawn with the head's pose at {lead:+.1f} ms from the frame's time (median)")
    print(f"one refresh early: {100 * early:.1f}%   usual: {100 * usual:.1f}%   one refresh late: {100 * late:.1f}%")
    # SteamVR's own timestamps (ms from the frame's vsync) for usual vs shifted frames, when logged.
    names = [n for n in ("wait_get_poses_ms", "new_poses_ready_ms", "new_frame_ready_ms", "comp_update_start_ms",
                         "client_interval_ms") if n in fr.dtype.names]
    g = fr[keep][m]
    for n in names:
        x = g[n]
        print(f"  {n:22s} usual {np.median(x[np.abs(d) < 0.3 * p]):7.2f}  early {np.median(x[d < -0.6 * p]) if early else float('nan'):7.2f}"
              f"  late {np.median(x[d > 0.6 * p]) if late else float('nan'):7.2f} ms")
    # What a perfect pipeline would show: the head at the usual lead for every frame. How far each drawn frame is from
    # that, in degrees: the jitter in what the game drew (frames a refresh off, and the frame-to-frame timing wobble).
    Mi = M[m] @ np.transpose(rot_at(tf[m] + lead / 1000), (0, 2, 1))
    ev = np.degrees(np.linalg.norm(tl.rotvec(Mi), axis=1))
    shifted = np.abs(d) > 0.5 * p
    print(f"drawn pose vs the head's real pose at that lead: median {np.median(ev):.3f}, p90 {np.percentile(ev, 90):.3f}, "
          f"p99 {np.percentile(ev, 99):.3f} deg; off > 0.2 deg in {100 * (ev > 0.2).mean():.1f}% of frames "
          f"({100 * (shifted & (ev > 0.2)).sum() / max((ev > 0.2).sum(), 1):.0f}% of those a refresh off, the rest timing wobble "
          f"p10 {np.percentile(d[~shifted], 10):+.1f} / p90 {np.percentile(d[~shifted], 90):+.1f} ms)")
    h, e = np.histogram(d, bins=np.arange(-1.5 * p, 1.5 * p + 0.01, p / 6))
    for i in range(len(h)):
        print(f"  {e[i]:+6.1f}..{e[i + 1]:+6.1f} ms {'#' * int(60 * h[i] / max(h.max(), 1))} {h[i]}")


if __name__ == "__main__":
    main()
