#!/usr/bin/env python3
"""
Fits camera i's pose in camera 0's frame from a tracker dataset by maximising a soft LED-on-blob score (Gaussian of the
distance from each predicted visible LED to its nearest blob, sigma shrinking 20 -> 3px), with camera 0's controller
poses as the reference. Coarse start: random rotations scored on the controller centres; then Nelder-Mead on 6 DoF.
usage: fit_camera2.py <dataset> <camera> [n_rot] [known t cm x y z: score that too]
"""
import sys
import numpy as np
sys.path.insert(0, __file__.rsplit('/', 1)[0])
import ctdataset as D
from fit_camera import qmul, inv, qtom, rodrigues, QF, FLIP


def nelder_mead(f, x0, step, iters=400):
    n = len(x0)
    pts = [x0] + [x0 + np.eye(n)[i] * step[i] for i in range(n)]
    vals = [f(p) for p in pts]
    for _ in range(iters):
        o = np.argsort(vals)
        pts = [pts[i] for i in o]
        vals = [vals[i] for i in o]
        c = np.mean(pts[:-1], 0)
        xr = c + (c - pts[-1])
        fr = f(xr)
        if fr < vals[0]:
            xe = c + 2 * (c - pts[-1])
            fe = f(xe)
            pts[-1], vals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = xr, fr
        else:
            xc = c + 0.5 * (pts[-1] - c)
            fc = f(xc)
            if fc < vals[-1]:
                pts[-1], vals[-1] = xc, fc
            else:
                pts = [pts[0]] + [pts[0] + 0.5 * (p - pts[0]) for p in pts[1:]]
                vals = [vals[0]] + [f(p) for p in pts[1:]]
    i = int(np.argmin(vals))
    return pts[i], vals[i]


def main():
    path, ci = sys.argv[1], int(sys.argv[2])
    nrot = int(sys.argv[3]) if len(sys.argv) > 3 else 20000
    known = np.array(list(map(float, sys.argv[4:7]))) / 100 if len(sys.argv) >= 7 else None
    cams, S, dev = D.load(path)
    K = cams[ci]['K']
    sol = {}
    for s in S:
        if s['cam'] == 0 and s['cam_pose'] is not None:
            for d, st in s['states'].items():
                if st['found'] is not None:
                    p, q = st['found']
                    sol.setdefault(s['ts'], []).append((d, FLIP @ p, qtom(qmul(QF, q))))
    rng = np.random.default_rng(2)
    fr = []
    chain = None
    for s in S:
        if s['cam'] == ci and len(s['blobs']) >= 4 and s['ts'] in sol:
            if chain is None:
                c0 = min((x for x in S if x['cam'] == 0 and x['cam_pose'] is not None), key=lambda x: abs(x['ts'] - s['ts']))
                chain = (qtom(qmul(inv(c0['cam_pose'][1]), s['cam_pose'][1])),
                         D.qrot(inv(c0['cam_pose'][1]), s['cam_pose'][0] - c0['cam_pose'][0]))
            fr.append(s)
    idx = rng.choice(len(fr), size=min(400, len(fr)), replace=False)
    F = len(idx)
    L = 34
    P = np.full((F, L, 3), np.nan)
    N = np.zeros((F, L, 3))
    C = np.zeros((F, 3))
    BM = 40
    B = np.full((F, BM, 2), 1e9)
    for a, k in enumerate(idx):
        s = fr[k]
        j = 0
        for d, p, R in sol[s['ts']]:
            for led in dev[d].values():
                if j < L:
                    P[a, j] = p + R @ led['pos']
                    N[a, j] = R @ led['normal']
                    j += 1
        C[a] = sol[s['ts']][0][1]
        bb = np.array([b['cu'] for b in s['blobs']])[:BM]
        B[a, :len(bb)] = bb
    valid = ~np.isnan(P[..., 0])
    P = np.nan_to_num(P)
    Rc, tc = chain
    print(f"camera {ci}: {len(fr)} frames ({F} used)")

    def score(R, t, sigma):
        c = np.einsum('flk,kj->flj', P - t, R) @ FLIP
        z = c[..., 2]
        u = K[0, 0] * c[..., 0] / np.where(z > 1e-3, z, 1) + K[0, 2]
        v = K[1, 1] * c[..., 1] / np.where(z > 1e-3, z, 1) + K[1, 2]
        view = t - P
        view /= np.linalg.norm(view, axis=-1, keepdims=True) + 1e-9
        vis = valid & (np.sum(N * view, -1) > np.cos(np.radians(60))) & (z > 0.05) & (u > 0) & (u < 1016) & (v > 0) & (v < 1016)
        d2 = (u[..., None] - B[:, None, :, 0]) ** 2 + (v[..., None] - B[:, None, :, 1]) ** 2
        dmin = np.sqrt(d2.min(-1))
        soft = np.exp(-dmin ** 2 / (2 * sigma ** 2)) * vis
        return soft.sum() / max(1, vis.sum()) * np.sqrt(vis.mean() + 1e-9), (dmin < 3)[vis].mean() if vis.any() else 0, vis.sum()

    def from_x(R0, x):
        return R0 @ rodrigues(x[:3]), tc + x[3:]

    # coarse
    Q = rng.normal(size=(nrot, 4))
    Q /= np.linalg.norm(Q, axis=1)[:, None]
    coarse = []
    for q in Q:
        R = qtom(q)
        coarse.append((score(R, tc, 40)[0], tuple(q)))
    coarse.sort(reverse=True)
    starts = [('chain', Rc)] + [(f'coarse#{k}', qtom(np.array(q))) for k, (sc, q) in enumerate(coarse[:5])]
    out = []
    for name, R0 in starts:
        x = np.zeros(6)
        for sigma in (20, 10, 6, 3):
            x, _ = nelder_mead(lambda xx: -score(*from_x(R0, xx), sigma)[0], x,
                               [0.05, 0.05, 0.05, 0.01, 0.01, 0.01] if sigma > 6 else [0.01] * 3 + [0.003] * 3, 300)
        R, t = from_x(R0, x)
        sc, hit, nv = score(R, t, 3)
        out.append((hit, name, R, t, nv))
    out.sort(key=lambda r: -r[0])
    for hit, name, R, t, nv in out[:4]:
        ang = np.degrees(np.arccos(np.clip((np.trace(Rc.T @ R) - 1) / 2, -1, 1)))
        print(f"  from {name:9s}: LEDs within 3px {hit * 100:5.1f}% ({nv} visible) | t {np.round(t * 100, 2)} cm (chain "
              f"{np.round(tc * 100, 2)}) | {ang:.1f} deg from the chain rotation")
    print(f"  reference: chain as is {score(Rc, tc, 3)[1] * 100:.1f}%", end='')
    if known is not None:
        print(f", chain rotation with t {np.round(known * 100, 2)}: {score(Rc, known, 3)[1] * 100:.1f}%", end='')
    print()
    hit, name, R, t, nv = out[0]
    np.save(path + f'.cam{ci}_fit.npy', np.concatenate([R.ravel(), t]))


if __name__ == '__main__':
    main()
