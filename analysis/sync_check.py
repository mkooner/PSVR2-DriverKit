#!/usr/bin/env python3
"""Is SteamVR direct mode in sync? From FRAME_LOG (logs/frames-<stamp>.csv) and the game's frame-timing layer
(logs/xr-frames-<stamp>-<game>.csv).

    python3 -I analysis/sync_check.py logs/frames-<stamp>.csv logs/xr-frames-<stamp>-<game>.csv [--skip 15]

1. The pose given to Monado for each game image vs the pose the game really drew with (identified exactly among the
   frame poses handed to SteamVR, U rows). In sync: the same pose (0 frames apart) while turning.
2. When Monado showed each game image minus the display time its pose was predicted for. In sync: ~0 ms.
3. When Monado showed it minus the display time the game was told (informational: SteamVR's own timing).
Before the fix (2026-11-18 12:01): 1. SteamVR's label 1 frame newer in 92% of turning frames; 2./3. +22.5 ms.
"""
import argparse
import collections
import sys

import numpy as np


def qmul(a, b):
    x1, y1, z1, w1 = a.T
    x2, y2, z2, w2 = b.T
    return np.c_[w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                 w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2]


C = np.array([-1, -1, -1, 1])


def angle(q):
    return np.degrees(2 * np.arctan2(np.linalg.norm(q[:, :3], axis=1), np.abs(q[:, 3])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('frames')
    ap.add_argument('xrframes')
    ap.add_argument('--skip', type=float, default=15.0, help='seconds after the first game frame to leave out')
    a = ap.parse_args()

    U, L, F, P = [], [], [], []
    for line in open(a.frames):
        x = line.strip().split(',')
        try:
            if x[0] == 'U':
                U.append([int(x[1]), int(x[2])] + [float(v) for v in x[3:10]])
            elif x[0] == 'L':
                L.append([int(x[1]), int(x[2]), int(x[3]), int(x[4]), int(x[5])])
            elif x[0] == 'F':
                F.append([int(v) for v in x[1:12]])
            elif x[0] == 'P':
                P.append([int(x[1]), int(x[2])])
        except (ValueError, IndexError):
            pass
    U, L, F, P = np.array(U), np.array(L, dtype=np.int64), np.array(F, dtype=np.int64), np.array(P, dtype=np.int64)
    W = [ln.split(',') for ln in open(a.xrframes) if ln.startswith('W,')]
    off = np.median([int(w[2]) - int(w[3]) for w in W])  # mono - xr
    E = np.array([[int(e[1]), int(e[2]), int(e[3]) + off] for e in
                  (ln.split(',') for ln in open(a.xrframes) if ln.startswith('E,'))])
    V = np.array([[int(v[1]), int(v[2]) + off] + [float(t) for t in v[10:17]] for v in
                  (ln.strip().split(',') for ln in open(a.xrframes) if ln.startswith('V,') and ln.split(',')[3] == '0')])
    if len(U) == 0 or len(L) == 0 or len(V) == 0:
        print('missing rows (U/L in the frame log, V in the game log)')
        return 1
    ut, uq = U[:, 1], U[:, 5:9]
    tidx = {int(t): i for i, t in enumerate(ut)}
    t_game0 = V[0, 0]
    # Application layer 0 of each SteamVR frame, with the game frame it shows (its last xrEndFrame before SubmitLayer).
    rows = []
    for t, layer, lab, used, app in L:
        if layer != 0 or not app or t < t_game0 + a.skip * 1e9:
            continue
        j = np.searchsorted(E[:, 1], t) - 1
        if j < 0:
            continue
        n = np.searchsorted(V[:, 0], E[j, 0] - 1)
        if n >= len(V) or abs(V[n, 0] - E[j, 0]) > 2e6:
            continue
        rows.append((t, lab, used, n, E[j, 2]))
    if not rows:
        print('no application frames after --skip')
        return 1
    vq = V[:, 5:9]
    vstep = np.r_[0, angle(qmul(vq[1:], vq[:-1] * C))]
    # Game space -> driver space: a fixed rotation, from still frames (all candidate poses agree there).
    rel = []
    for t, lab, used, n, disp in rows:
        k = tidx.get(int(lab))
        if k is not None and vstep[n] < 0.05:
            rel.append(qmul(vq[n:n + 1], (uq[k] * C)[None])[0])
    rel = np.array(rel)
    rel *= np.sign(rel[:, 3:4])
    R = rel.mean(0)
    R /= np.linalg.norm(R)
    gd = qmul(np.repeat((R * C)[None], len(vq), 0), vq)
    used_vs_game, label_vs_game, turning = [], [], []
    for t, lab, used, n, disp in rows:
        k_lab, k_used = tidx.get(int(lab)), tidx.get(int(used))
        cand = np.arange(max(0, (k_lab or 0) - 6), min(len(uq), (k_lab or 0) + 3))
        if k_lab is None or len(cand) == 0:
            continue
        d = angle(qmul(np.repeat(gd[n:n + 1], len(cand), 0), uq[cand] * C))
        k_game = cand[np.argmin(d)]
        if d.min() > 0.05:
            continue
        label_vs_game.append(k_lab - k_game)
        used_vs_game.append((k_used - k_game) if k_used is not None else None)
        turning.append(vstep[n] > 0.3)
    turning = np.array(turning)
    print(f"{len(rows)} game frames after {a.skip:.0f} s, {len(label_vs_game)} with the game's pose identified, "
          f"{turning.sum()} turning (> 0.3 deg/frame)")
    c_lab = collections.Counter(np.array(label_vs_game)[turning])
    c_used = collections.Counter([u for u, tr in zip(used_vs_game, turning) if tr and u is not None])
    nonfixed = sum(1 for u, tr in zip(used_vs_game, turning) if tr and u is None)
    tot = max(1, turning.sum())
    print(f"1. pose given to Monado minus the game's pose, frames (turning): {dict(sorted(c_used.items()))}, "
          f"label kept {nonfixed}  -> in sync {100 * c_used.get(0, 0) / tot:.1f}%")
    print(f"   (SteamVR's own label minus the game's pose: {dict(sorted(c_lab.items()))})")
    # 2. and 3.: Monado's display of each submitted SteamVR frame vs the pose time / the game's display time
    sub = F[(F[:, 7] & 1) != 0]
    pres_to_disp = dict(zip(sub[:, 4], sub[:, 2]))
    err_pose, err_told = [], []
    for t, lab, used, n, disp in rows:
        k = np.searchsorted(P[:, 0], t)
        if k >= len(P):
            continue
        shown = pres_to_disp.get(int(P[k, 0]))
        if shown is None:
            continue
        if used:
            err_pose.append((shown - used) / 1e6)
        err_told.append((shown - disp) / 1e6)
    for name, e in (('2. shown minus the time its pose was for', err_pose),
                    ('3. shown minus the display time the game was told', err_told)):
        if e:
            e = np.array(e)
            print(f"{name}: median {np.median(e):+.2f} ms (p10 {np.percentile(e, 10):+.2f}, p90 "
                  f"{np.percentile(e, 90):+.2f}), {len(e)} frames")
    return 0


if __name__ == '__main__':
    sys.exit(main())
