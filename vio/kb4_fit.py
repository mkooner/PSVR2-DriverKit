#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Fit Basalt's Kannala-Brandt (kb4) fisheye model to the PS VR2 cameras' own calibration (Sony's 20 coefficient
rational + thin prism + tilt model, a line by line port of psvr2_distort/psvr2_undistort in Monado's
t_camera_models.hpp), from the calibration block a record file holds (V lines, type 17).
Usage: kb4_fit.py record.csv [out.json]   -> per camera kb4 params and the fit error in pixels.
"""
import sys, struct, json, numpy as np

def read_block(path):
    for l in open(path):
        if l.startswith('V,'):
            p = l.rstrip('\n').split(',')
            if p[2] == '17':
                return bytes.fromhex(p[4])
    raise SystemExit('no calibration block (V,...,17) in ' + path)

def parse(block):
    assert block[:4] == b'SIE\x01'
    ndist, nrel = block[15], block[16]
    off = 24; cams = []
    for i in range(ndist):
        cid, w, h = struct.unpack_from('<IHH', block, off)
        K = np.array(struct.unpack_from('<9f', block, off + 8)).reshape(3, 3)
        p = np.array(struct.unpack_from('<20d', block, off + 44))
        cams.append(dict(id=cid, w=w, h=h, K=K, p=p)); off += 228
    rels = []
    for i in range(nrel):
        to, frm = block[off], block[off + 1]
        R = np.array(struct.unpack_from('<9f', block, off + 4)).reshape(3, 3)
        t = np.array(struct.unpack_from('<3f', block, off + 40))
        rels.append(dict(to=to, frm=frm, R=R, t=t)); off += 52
    return cams, rels

def distort(p, x, y):
    x2, y2 = x * x, y * y; r2 = x2 + y2; two_xy = 2 * x * y; r8 = r2 ** 4; r10 = r8 * r2
    num = 1 + ((p[4] * r2 + p[1]) * r2 + p[0]) * r2 + p[16] * r10 * r2 + p[15] * r10 + p[14] * r8
    den = 1 + ((p[7] * r2 + p[6]) * r2 + p[5]) * r2 + p[19] * r10 * r2 + p[18] * r10 + p[17] * r8
    radial = np.where(np.abs(den) > 1e-9, num / np.where(np.abs(den) > 1e-9, den, 1), 1.0)
    dx = (p[9] * r2 + p[8]) * r2 + (x2 + x2 + r2) * p[3] + radial * x + two_xy * p[2]
    dy = (p[11] * r2 + p[10]) * r2 + two_xy * p[3] + (y2 + y2 + r2) * p[2] + radial * y
    ca, sa, cb, sb = np.cos(p[12]), np.sin(p[12]), np.cos(p[13]), np.sin(p[13])
    ry_x = cb * dx + sb; ry_y = dy; ry_z = cb - sb * dx
    vx = ry_x; vy = ca * ry_y - sa * ry_z; vz = sa * ry_y + ca * ry_z
    return vx / vz, vy / vz

def undistort(p, tx, ty):
    x, y = tx.copy(), ty.copy(); h = 1e-4
    for _ in range(20):
        rx, ry = distort(p, x, y); ex, ey = tx - rx, ty - ry
        a1, a2 = distort(p, x + h, y); b1, b2 = distort(p, x, y + h)
        j00, j10, j01, j11 = (a1 - rx) / h, (a2 - ry) / h, (b1 - rx) / h, (b2 - ry) / h
        det = j00 * j11 - j01 * j10; ok = np.abs(det) > 1e-12; det = np.where(ok, det, 1)
        x = x + np.where(ok, (j11 * ex - j01 * ey) / det, ex); y = y + np.where(ok, (j00 * ey - j10 * ex) / det, ey)
    rx, ry = distort(p, x, y)
    return x, y, np.hypot(tx - rx, ty - ry)

def kb4_project(q, X, Y, Z):
    fx, fy, cx, cy, k1, k2, k3, k4 = q
    r = np.hypot(X, Y); th = np.arctan2(r, Z); t2 = th * th
    d = th * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))
    s = np.where(r > 1e-12, d / np.where(r > 1e-12, r, 1), 1.0)
    return fx * s * X + cx, fy * s * Y + cy

def fit_kb4(X, Y, Z, u, v, q0):
    q = np.array(q0, float); lam = 1e-3
    def res(q):
        pu, pv = kb4_project(q, X, Y, Z); return np.concatenate([pu - u, pv - v])
    r = res(q)
    for it in range(200):
        J = np.empty((r.size, 8))
        for k in range(8):
            dq = np.zeros(8); dq[k] = 1e-6 * max(1, abs(q[k])); J[:, k] = (res(q + dq) - r) / dq[k]
        A = J.T @ J; g = J.T @ r
        step = np.linalg.solve(A + lam * np.diag(np.diag(A)), -g)
        rn = res(q + step)
        if rn @ rn < r @ r:
            q, r, lam = q + step, rn, lam * 0.3
            if np.linalg.norm(step) < 1e-10 * (1 + np.linalg.norm(q)): break
        else:
            lam *= 10
    return q, r

def main():
    block = read_block(sys.argv[1]); cams, rels = parse(block)
    out = {}
    for c in cams:
        K, p, w, h = c['K'], c['p'], c['w'], c['h']
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        uu, vv = np.meshgrid(np.linspace(4, w - 5, 80), np.linspace(4, h - 5, 80)); uu, vv = uu.ravel(), vv.ravel()
        tx, ty = (uu - cx) / fx, (vv - cy) / fy
        x, y, err = undistort(p, tx, ty)
        # keep pixels Sony's model actually maps (Newton converged, inside the circle where the lens has image)
        rr = np.hypot(uu - cx, vv - cy)
        good = (err < 1e-6) & np.isfinite(x) & (rr < 0.5 * min(w, h) * 0.98)
        n = np.sqrt(x * x + y * y + 1); X, Y, Z = x / n, y / n, 1 / n
        X, Y, Z, u, v = X[good], Y[good], Z[good], uu[good], vv[good]
        q, r = fit_kb4(X, Y, Z, u, v, [fx, fy, cx, cy, 0, 0, 0, 0])
        e = np.hypot(r[:r.size // 2], r[r.size // 2:])
        fov = 2 * np.degrees(np.arctan2(np.hypot(X, Y), Z).max())
        print(f"camera {c['id']}: {w}x{h}, fx {fx:.1f} fy {fy:.1f} cx {cx:.1f} cy {cy:.1f}; {good.sum()} grid points, "
              f"field of view {fov:.0f} deg; kb4 fit error px mean {e.mean():.3f} p99 {np.percentile(e,99):.3f} max {e.max():.3f}")
        print('   kb4 fx fy cx cy k1..k4:', ' '.join(f'{v:.6g}' for v in q))
        out[c['id']] = dict(camera_type='kb4', intrinsics=dict(zip(['fx','fy','cx','cy','k1','k2','k3','k4'], map(float, q))),
                            resolution=[w, h], fit_err_px_p99=float(np.percentile(e, 99)))
    for r_ in rels:
        ang = np.degrees(np.arccos(np.clip((np.trace(r_['R']) - 1) / 2, -1, 1)))
        print(f"relation cam{r_['frm']} -> cam{r_['to']}: rotation {ang:.2f} deg, translation {np.round(r_['t'],4)} (|t| {np.linalg.norm(r_['t'])*1000:.1f} mm)")
    if len(sys.argv) > 2:
        json.dump(dict(cameras=out, relations=[dict(to=r_['to'], frm=r_['frm'], R=r_['R'].tolist(), t=r_['t'].tolist()) for r_ in rels]),
                  open(sys.argv[2], 'w'), indent=1)

main()
