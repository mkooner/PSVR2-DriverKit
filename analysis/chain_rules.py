#!/usr/bin/env python3
"""
Tests convention rules for the driver's camera chain against a tracker dataset: each rule maps a camera's stored
relative pose (Q, p) from its parent into a corrected one; cameras are then composed along the parents (1<-0, 2<-0,
3<-2) and scored by camera 0's controller LEDs landing on each camera's blobs (within 3px).
usage: chain_rules.py <dataset recorded with the stored chain (before 76f7cee31)>
"""
import sys
import itertools
import numpy as np
sys.path.insert(0, __file__.rsplit('/', 1)[0])
import ctdataset as D
from fit_camera import qmul, inv, qtom, QF, FLIP

PARENT = {1: 0, 2: 0, 3: 2}


def build(path):
    cams, S, dev = D.load(path)
    sol = {}
    for s in S:
        if s['cam'] == 0 and s['cam_pose'] is not None:
            for d, st in s['states'].items():
                if st['found'] is not None:
                    p, q = st['found']
                    sol.setdefault(s['ts'], []).append((d, FLIP @ p, qtom(qmul(QF, q))))
    c0 = [s for s in S if s['cam'] == 0 and s['cam_pose'] is not None]
    ref = c0[len(c0) // 2]
    chain = {}
    for i in (1, 2, 3):
        si = min((x for x in S if x['cam'] == i and x['cam_pose'] is not None), key=lambda x: abs(x['ts'] - ref['ts']))
        chain[i] = (qtom(qmul(inv(ref['cam_pose'][1]), si['cam_pose'][1])),
                    D.qrot(inv(ref['cam_pose'][1]), si['cam_pose'][0] - ref['cam_pose'][0]))
    chain[0] = (np.eye(3), np.zeros(3))
    # stored relative poses from the parent (cam0 frame composition: X_i = X_par * rel_i)
    rel = {}
    for i in (1, 2, 3):
        Rp, tp = chain[PARENT[i]]
        Ri, ti = chain[i]
        rel[i] = (Rp.T @ Ri, Rp.T @ (ti - tp))
    rng = np.random.default_rng(3)
    data = {}
    for i in (1, 2, 3):
        fr = [s for s in S if s['cam'] == i and len(s['blobs']) >= 4 and s['ts'] in sol]
        fr = [fr[k] for k in rng.choice(len(fr), size=min(300, len(fr)), replace=False)]
        pts = []
        for s in fr:
            P = []
            N = []
            for d, p, R in sol[s['ts']]:
                for led in dev[d].values():
                    P.append(p + R @ led['pos'])
                    N.append(R @ led['normal'])
            pts.append((np.array(P), np.array(N), np.array([b['cu'] for b in s['blobs']])))
        data[i] = (cams[i]['K'], pts)
    return chain, rel, data


def hit_rate(K, pts, R, t):
    hits = tot = 0
    for P, N, B in pts:
        c = ((P - t) @ R) @ FLIP
        z = c[:, 2]
        view = t - P
        view /= np.linalg.norm(view, axis=1)[:, None]
        ok = (np.sum(N * view, 1) > np.cos(np.radians(60))) & (z > 0.05)
        u = np.stack([K[0, 0] * c[ok, 0] / z[ok] + K[0, 2], K[1, 1] * c[ok, 1] / z[ok] + K[1, 2]], 1)
        inimg = (u[:, 0] > 0) & (u[:, 0] < 1016) & (u[:, 1] > 0) & (u[:, 1] < 1016)
        for uu in u[inimg]:
            tot += 1
            hits += np.min(np.linalg.norm(B - uu, axis=1)) < 3
    return hits / max(1, tot), tot


F = FLIP
RULES_R = {'Q': lambda Q: Q, 'Q^T': lambda Q: Q.T, 'FQF': lambda Q: F @ Q @ F, 'FQ^TF': lambda Q: F @ Q.T @ F}
RULES_T = {'p': lambda Q, p: p, 'Q^T p': lambda Q, p: Q.T @ p, 'F p': lambda Q, p: F @ p,
           'F Q^T p': lambda Q, p: F @ Q.T @ p, 'Q^T F p': lambda Q, p: Q.T @ F @ p, '-Q^T p': lambda Q, p: -Q.T @ p}


def main():
    chain, rel, data = build(sys.argv[1])
    for i in (1, 2, 3):
        print(f"camera {i} (parent {PARENT[i]}): stored chain {hit_rate(*data[i], *chain[i])[0] * 100:.1f}%")
    results = []
    for (rn, rf), (tn, tf) in itertools.product(RULES_R.items(), RULES_T.items()):
        for scope in ('all', 'not cam1'):
            X = {0: (np.eye(3), np.zeros(3))}
            for i in (1, 2, 3):
                Q, p = rel[i]
                if scope == 'not cam1' and i == 1:
                    Qn, pn = Q, Q.T @ p  # camera 1 as verified (76f7cee31)
                else:
                    Qn, pn = rf(Q), tf(Q, p)
                Rp, tp = X[PARENT[i]]
                X[i] = (Rp @ Qn, tp + Rp @ pn)
            h = [hit_rate(*data[i], *X[i])[0] for i in (1, 2, 3)]
            results.append((min(h), rn, tn, scope, h))
    results.sort(key=lambda r: -r[0])
    for m, rn, tn, scope, h in results[:8]:
        print(f"  rotation {rn:6s} translation {tn:8s} ({scope:8s}): cam1 {h[0] * 100:5.1f}% cam2 {h[1] * 100:5.1f}% cam3 {h[2] * 100:5.1f}%")


if __name__ == '__main__':
    main()
