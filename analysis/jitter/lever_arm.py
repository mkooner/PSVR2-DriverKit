#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Is the SLAM head point's motion mostly rotation about a fixed pivot (the neck)?

Fits SLAM velocity v = w x (R r) + u over turning samples (r: the pivot -> SLAM point offset in the head frame,
u: the pivot's own velocity, treated as noise), and reports r, how much of the velocity it explains, and whether r is
consistent between recordings. If it is, position can be split into a slow pivot (filtered) plus R(t) r from the
2 kHz orientation.
Usage: lever_arm.py record.csv [...]
"""
import sys
import numpy as np

def rotm(q):
    x, y, z, w = q.T
    return np.stack([np.stack([1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], -1),
                     np.stack([2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], -1),
                     np.stack([2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)], -1)], 1)

def skew(w):
    z = np.zeros(len(w))
    return np.stack([np.stack([z, -w[:, 2], w[:, 1]], -1), np.stack([w[:, 2], z, -w[:, 0]], -1),
                     np.stack([-w[:, 1], w[:, 0], z], -1)], 1)

for path in sys.argv[1:]:
    S = np.array([[float(x) for x in l.split(',')[2:10]] for l in open(path) if l.startswith('S,')])
    if len(S) < 2000:
        continue
    t = (S[:, 0] - S[0, 0]) * 1e-9; p = S[:, 1:4]; q = S[:, 4:8]
    m = t > 20
    t, p, q = t[m], p[m], q[m]
    # central differences over +-2 samples
    k = np.arange(2, len(t) - 2)
    dt = t[k+2] - t[k-2]
    ok = (dt > 0.05) & (dt < 0.08)
    k = k[ok]; dt = dt[ok]
    v = (p[k+2] - p[k-2]) / dt[:, None]
    R = rotm(q)
    dR = np.einsum('nij,nkj->nik', R[k+2], R[k-2])  # R2 R1^T
    # angular velocity (world) from dR
    ang = np.arccos(np.clip((np.trace(dR, axis1=1, axis2=2) - 1) / 2, -1, 1))
    axis = np.stack([dR[:, 2, 1] - dR[:, 1, 2], dR[:, 0, 2] - dR[:, 2, 0], dR[:, 1, 0] - dR[:, 0, 1]], 1)
    axis /= np.maximum(np.linalg.norm(axis, axis=1, keepdims=True), 1e-12)
    w = axis * (ang / dt)[:, None]
    turning = np.degrees(np.linalg.norm(w, axis=1)) > 30
    # drop steps that are resets / jumps (> 2 m/s)
    sane = np.linalg.norm(v, axis=1) < 2.0
    sel = turning & sane
    if sel.sum() < 100:
        print(path, 'too few turning samples'); continue
    A = np.einsum('nij,njk->nik', skew(w[sel]), R[k][sel]).reshape(-1, 3)
    b = v[sel].reshape(-1)
    r, *_ = np.linalg.lstsq(A, b, rcond=None)
    pred = (A @ r).reshape(-1, 3)
    vv = v[sel]
    explained = 1 - np.sum((vv - pred) ** 2) / np.sum((vv - vv.mean(0)) ** 2)
    name = path.split('/')[-1][7:22]
    print(f'{name}: {sel.sum()} turning samples; r = ({r[0]*100:+.1f}, {r[1]*100:+.1f}, {r[2]*100:+.1f}) cm '
          f'(|r| {np.linalg.norm(r)*100:.1f} cm); rotation explains {explained*100:.0f}% of the head point velocity; '
          f'residual speed p50 {np.median(np.linalg.norm(vv-pred,axis=1))*100:.1f} cm/s vs speed p50 '
          f'{np.median(np.linalg.norm(vv,axis=1))*100:.1f} cm/s')
