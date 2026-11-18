#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Tell tracking jitter from compositor judder in a SteamVR session.

Inputs (one session):
  pose log   OVRD_POSE_LOG from the SteamVR plugin (POSE_LOG=1 ./launch-steamvr.sh): every headset pose we handed
             SteamVR, ~1 kHz, CLOCK_MONOTONIC.
  frames     tools/vrframetiming: SteamVR's per-frame timing and the headset pose the game rendered each frame with.

For every frame it measures the rendered pose's jitter: how far the head's motion over that frame departs from the
motion over the frames around it (real head motion changes smoothly between 11 ms frames; jitter doesn't). Frames
whose jitter is over the threshold are labelled:
  tracking      the poses we submitted jumped too around that time (driver/tracking side)
  frame timing  the frame or its neighbours were late, dropped, mispresented or reprojected (compositor/game side)
  prediction    neither: SteamVR's own extrapolation (velocities we sent) moved the rendered pose
and the session is summarised per 30 s, so a problem that grows over the session shows as a trend.

Usage: steamvr_timeline.py pose-log.csv frames.csv [--rot-deg 0.05] [--pos-mm 0.5] [--window 30]
"""
import argparse
import sys

import numpy as np

# m_nReprojectionFlags bits that mean this frame was re-warped: late on CPU (0x01) or GPU (0x02), or motion smoothing
# (0x08). 0x04 only says async reprojection is switched on (openvr.h VRCompositor_ReprojectionAsync).
REPROJECTED = 0x01 | 0x02 | 0x08


def read_csv(path):
    """CSV with a header row; lines starting with # (notes, the end-of-log summary) are skipped."""
    with open(path) as f:
        lines = [l for l in f if not l.startswith("#")]
    n = lines[0].count(",")
    lines = [lines[0]] + [l for l in lines[1:] if l.count(",") == n and l.endswith("\n")]  # drop a cut-off last line
    return np.genfromtxt(lines, delimiter=",", names=True)


def load_pose_log(path):
    """The head's motion: the pose for the submit time. With OVRD_POSE_MODE=display the submitted pose is for the
    display time, and the log has the pose for now as well (nqx.. npz): that one."""
    d = read_csv(path)
    t = d["submit_ns"] * 1e-9
    if "nqx" in d.dtype.names:
        q = np.c_[d["nqx"], d["nqy"], d["nqz"], d["nqw"]]
        p = np.c_[d["npx"], d["npy"], d["npz"]]
    else:
        q = np.c_[d["qx"], d["qy"], d["qz"], d["qw"]]
        p = np.c_[d["px"], d["py"], d["pz"]]
    return t, q, p, d["get_pose_us"], d["valid"]


def load_frames(path):
    return read_csv(path)


def quat_to_mat(q):
    x, y, z, w = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def rotvec(R):
    """Rotation vectors (radians) of rotation matrices."""
    tr = np.clip((np.trace(R, axis1=-2, axis2=-1) - 1) / 2, -1, 1)
    ang = np.arccos(tr)
    v = np.stack([R[..., 2, 1] - R[..., 1, 2], R[..., 0, 2] - R[..., 2, 0], R[..., 1, 0] - R[..., 0, 1]], -1)
    s = np.sin(ang)
    k = np.where(s > 1e-9, ang / (2 * np.maximum(s, 1e-12)), 0.5)
    return v * k[..., None]


def interp_pose(t, q, p, tn):
    """Poses at times tn: positions linear, rotations normalised linear (fine between 1 ms samples)."""
    q = q.copy()
    for i in range(1, len(q)):  # same hemisphere as the previous sample
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]
    qn = np.stack([np.interp(tn, t, q[:, k]) for k in range(4)], 1)
    qn /= np.linalg.norm(qn, axis=1, keepdims=True)
    pn = np.stack([np.interp(tn, t, p[:, k]) for k in range(3)], 1)
    return qn, pn


def step_jitter(R, p, k=2):
    """Per sample: how far each step (rotation and position) departs from the mean of its neighbours' steps.

    Steps are in the world frame (R_i R_{i-1}^T), so a smooth turn gives nearly equal steps. Returns degrees and
    millimetres for each sample (NaN at the ends)."""
    n = len(R)
    dr = np.full((n, 3), np.nan)
    dr[1:] = rotvec(R[1:] @ np.transpose(R[:-1], (0, 2, 1)))
    dp = np.full((n, 3), np.nan)
    dp[1:] = p[1:] - p[:-1]
    jr = np.full(n, np.nan)
    jp = np.full(n, np.nan)
    for i in range(k + 1, n - k):
        nb = [j for j in range(i - k, i + k + 1) if j != i]
        jr[i] = np.linalg.norm(dr[i] - dr[nb].mean(0))
        jp[i] = np.linalg.norm(dp[i] - dp[nb].mean(0))
    return np.degrees(jr), jp * 1000


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pose_log")
    ap.add_argument("frames")
    ap.add_argument("--rot-deg", type=float, default=0.05, help="rendered rotation jitter threshold (deg/frame)")
    ap.add_argument("--pos-mm", type=float, default=0.5, help="rendered position jitter threshold (mm/frame)")
    ap.add_argument("--window", type=float, default=30, help="summary window (s)")
    ap.add_argument("--events", type=int, default=15, help="list this many of the worst events")
    a = ap.parse_args()

    tp, qp, pp, cost_us, pvalid = load_pose_log(a.pose_log)
    fr = load_frames(a.frames)
    if len(fr) < 20 or len(tp) < 100:
        sys.exit("not enough data")

    # Clock link: a frame is always read after it happened, so mono_s - t_s >= offset; the smallest difference over the
    # session is the offset (to within the poll's luck, a few ms over thousands of frames).
    offset = np.min(fr["mono_s"] - fr["t_s"])
    link = "smallest read delay (late by up to a frame or two: rows appear once a frame is done)"
    # Exact link when vrframetiming logged "# link <vsync monotonic> <current frame t_s> ...": the current frame's t_s is
    # the vsync it started on, normally the last one, so the most common difference (to 0.5 ms) is the offset.
    links = []
    with open(a.frames) as f:
        for l in f:
            if l.startswith("# link "):
                x = l.split()
                links.append(float(x[2]) - float(x[3]))
    if len(links) > 10:
        links = np.array(links)
        bins = np.round(links / 0.0005)
        vals, counts = np.unique(bins, return_counts=True)
        mode = vals[np.argmax(counts)]
        near = links[np.abs(bins - mode) <= 2]
        offset = float(np.median(near))
        link = f"SteamVR's last vsync and the current frame's time ({len(near)} of {len(links)} reads agree to 1 ms)"
    vs = []
    with open(a.frames) as f:
        for l in f:
            if l.startswith("# vsync "):
                x = l.split()
                vs.append((float(x[2]), int(x[3])))
    if len(vs) > 10 and not isinstance(links, np.ndarray):
        vs = np.array(vs)
        idx = np.searchsorted(fr["frame"], vs[:, 1])
        ok_ = (idx < len(fr)) & (fr["frame"][np.minimum(idx, len(fr) - 1)] == vs[:, 1])
        if ok_.sum() > 10:
            offset = np.median(vs[ok_, 0] - fr["t_s"][idx[ok_]])
            link = f"SteamVR's vsync counter ({ok_.sum()} matches)"
        else:
            link += "; vsync counter didn't match frame indices"
    tf = fr["t_s"] + offset
    ok = fr["pose_valid"] > 0
    M = np.stack([np.c_[fr["m00"], fr["m01"], fr["m02"]], np.c_[fr["m10"], fr["m11"], fr["m12"]],
                  np.c_[fr["m20"], fr["m21"], fr["m22"]]], 1)
    pf = np.c_[fr["m03"], fr["m13"], fr["m23"]]
    jr, jp = step_jitter(M, pf)

    budget = np.median(np.diff(fr["t_s"]))
    gap = np.r_[budget, np.diff(fr["t_s"])]

    # The submitted poses on their own even grid at the frame rate (interpolated, so neither the 1 kHz sampling nor
    # the frames' timing adds anything), same measure.
    tg = np.arange(tp[0], tp[-1], budget)
    qg, pg = interp_pose(tp, qp, pp, tg)
    Rg = quat_to_mat(qg)
    sr_g, sp_g = step_jitter(Rg, pg)

    # The game's pose is SteamVR's prediction from the submitted pose ~20-30 ms ahead. Find that lead from the data:
    # the shift that best lines up the rendered and submitted turning speeds.
    def speed(R, t):
        v = np.full(len(R), np.nan)
        v[1:] = np.degrees(np.linalg.norm(rotvec(R[1:] @ np.transpose(R[:-1], (0, 2, 1))), axis=1)) / np.diff(t)
        return v
    vr_ = speed(M, tf)
    vs_ = speed(Rg, tg)
    best, lead = -2, 0.0
    good = np.isfinite(vr_) & (gap < 1.5 * budget)
    for L in np.arange(-0.02, 0.1, 0.001):
        x = np.interp(tf + L, tg, np.nan_to_num(vs_))
        c = np.corrcoef(vr_[good], x[good])[0, 1]
        if c > best:
            best, lead = c, L
    # Submitted jitter seen by each frame: the worst on the grid within 1.5 frames of the time its pose came from.
    gi = np.searchsorted(tg, tf + lead)
    sr = np.array([np.nanmax(np.r_[sr_g[max(i - 2, 0):i + 2], 0]) for i in gi])
    sp = np.array([np.nanmax(np.r_[sp_g[max(i - 2, 0):i + 2], 0]) for i in gi])

    # Frame timing trouble: this frame or a neighbour reprojected / dropped / mispresented, or its interval off.
    bad = ((fr["reproj_flags"].astype(int) & REPROJECTED) != 0) | (fr["dropped"] > 0) | (fr["mispresented"] > 0) | (np.abs(gap - budget) > 0.3 * budget)
    # The jitter measure compares a frame with 2 frames each side, so trouble up to 2 frames away shows in it.
    bad_near = np.convolve(bad.astype(float), np.ones(5), mode="same") > 0
    # Submitted pose jumped around the frame's time (the rendered pose is predicted ~20-30 ms ahead of it).
    sub_near = (sr > a.rot_deg * 0.5) | (sp > a.pos_mm * 0.5)

    jit = ok & ((jr > a.rot_deg) | (jp > a.pos_mm))
    label = np.where(sub_near, "tracking", np.where(bad_near, "frame timing", "prediction"))

    t0 = tp[0]
    print(f"{len(fr)} frames over {tf[-1] - t0:.0f} s, frame interval {budget * 1000:.2f} ms; "
          f"{len(tp)} submitted poses ({len(tp) / (tp[-1] - tp[0]):.0f} Hz)")
    print(f"clock link: frame time + {offset:.4f} s = CLOCK_MONOTONIC, from {link}; times below from the start of the pose log")
    print(f"rendered pose leads the submitted one by {lead * 1000:.0f} ms (SteamVR's prediction; speed match r={best:.3f})")
    gp = np.diff(tp) * 1000
    print(f"pose submission gaps: median {np.median(gp):.2f} ms, p99.9 {np.percentile(gp, 99.9):.2f} ms, "
          f"max {gp.max():.1f} ms, gaps > 5 ms: {(gp > 5).sum()};  GetPose cost median {np.median(cost_us):.0f} us, "
          f"p99.9 {np.percentile(cost_us, 99.9):.0f} us")
    print(f"jitter thresholds: rotation {a.rot_deg} deg/frame, position {a.pos_mm} mm/frame\n")
    print(f"{'window':>11} {'frames':>6} {'reproj%':>7} {'drop%':>6} {'gpu ms':>6} {'jitter':>6} "
          f"{'tracking':>8} {'timing':>7} {'predict':>7} {'rend rot p99':>12} {'sub rot p99':>11}")
    w = a.window
    for s in np.arange(t0, tf[-1], w):
        m = (tf >= s) & (tf < s + w)
        if m.sum() < 10:
            continue
        jm = jit & m
        print(f"{s - t0:5.0f}-{s - t0 + w:<5.0f} {m.sum():6d} {100 * ((fr['reproj_flags'][m].astype(int) & REPROJECTED) != 0).mean():7.1f} "
              f"{100 * (fr['dropped'][m] > 0).mean():6.1f} {np.median(fr['app_gpu_ms'][m]):6.2f} {jm.sum():6d} "
              f"{(jm & (label == 'tracking')).sum():8d} {(jm & (label == 'frame timing')).sum():7d} "
              f"{(jm & (label == 'prediction')).sum():7d} {np.nanpercentile(jr[m], 99):11.3f}° "
              f"{np.nanpercentile(sr[m], 99):10.3f}°")
    tot = jit.sum()
    print(f"\nall: {tot} jittery frames ({100 * tot / max(ok.sum(), 1):.1f}%): "
          f"tracking {(jit & (label == 'tracking')).sum()}, frame timing {(jit & (label == 'frame timing')).sum()}, "
          f"prediction {(jit & (label == 'prediction')).sum()}")
    if tot and a.events:
        score = np.nan_to_num(jr / a.rot_deg) + np.nan_to_num(jp / a.pos_mm)
        worst = np.argsort(-np.where(jit, score, -1))[: min(a.events, tot)]
        print("\nworst events:")
        for i in sorted(worst):
            print(f"  {tf[i] - t0:8.3f} s  rendered {jr[i]:.3f}° {jp[i]:.2f} mm  submitted {sr[i]:.3f}° {sp[i]:.2f} mm  "
                  f"reproj {int(fr['reproj_flags'][i]) & REPROJECTED} dropped {int(fr['dropped'][i])} mis {int(fr['mispresented'][i])} "
                  f"interval {gap[i] * 1000:.1f} ms gpu {fr['app_gpu_ms'][i]:.2f} ms -> {label[i]}")


if __name__ == "__main__":
    main()
