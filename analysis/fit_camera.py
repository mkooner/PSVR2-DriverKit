#!/usr/bin/env python3
"""
Fits a camera's pose relative to camera 0 from a tracker dataset, without trusting the driver's camera chain:
camera 0's controller poses (world) are re-expressed in camera 0's frame, and camera i's pose (R, t, OpenXR axes, in
camera 0's frame) is searched so the controller's LEDs project onto camera i's blobs.
  1. coarse: random rotations (t at the chain's value), scored by the controller centre landing within 40px of a blob
  2. fine: 6-DoF Gauss-Newton on LED -> nearest blob reprojection (gated, shrinking), robust
usage: fit_camera.py <dataset> <camera index> [n_rotations]
"""
import sys
import numpy as np
sys.path.insert(0, __file__.rsplit('/', 1)[0])
import ctdataset as D


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def inv(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def qtom(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rodrigues(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


QF = np.array([1.0, 0, 0, 0])
FLIP = np.diag([1.0, -1, -1])  # OpenXR camera axes <-> OpenCV


def main():
    path, ci = sys.argv[1], int(sys.argv[2])
    nrot = int(sys.argv[3]) if len(sys.argv) > 3 else 20000
    cams, S, dev = D.load(path)
    K = cams[ci]['K']
    sol = {}
    for s in S:
        if s['cam'] != 0 or s['cam_pose'] is None:
            continue
        for d, st in s['states'].items():
            if st['found'] is not None:
                p, q = st['found']  # device in cam0 (OpenCV)
                sol.setdefault(s['ts'], []).append((d, FLIP @ p, qtom(qmul(QF, q))))  # device in cam0 (OpenXR)
    frames = []
    chain = None
    for s in S:
        if s['cam'] != ci or len(s['blobs']) < 4 or s['ts'] not in sol:
            continue
        if chain is None:
            c0 = min((x for x in S if x['cam'] == 0 and x['cam_pose'] is not None), key=lambda x: abs(x['ts'] - s['ts']))
            q0, qi = c0['cam_pose'][1], s['cam_pose'][1]
            chain = (qtom(qmul(inv(q0), qi)), D.qrot(inv(q0), s['cam_pose'][0] - c0['cam_pose'][0]))
        frames.append((np.array([b['cu'] for b in s['blobs']]), sol[s['ts']]))
    print(f"camera {ci}: {len(frames)} frames with camera 0 controller poses and camera {ci} blobs")
    Rc, tc = chain

    def project(R, t, P):  # P: points in cam0 OpenXR frame (N,3) -> pixels, z
        c = (P - t) @ R  # into cam i OpenXR
        c = c @ FLIP  # OpenCV
        z = c[:, 2]
        return np.stack([K[0, 0] * c[:, 0] / z + K[0, 2], K[1, 1] * c[:, 1] / z + K[1, 2]], 1), z

    centres = np.array([f[1][0][1] for f in frames])

    def coarse_score(R, t):
        uv, z = project(R, t, centres)
        ok = (z > 0.1) & (uv[:, 0] > -50) & (uv[:, 0] < 1066) & (uv[:, 1] > -50) & (uv[:, 1] < 1066)
        hit = 0
        for k in np.nonzero(ok)[0]:
            if np.min(np.linalg.norm(frames[k][0] - uv[k], axis=1)) < 40:
                hit += 1
        return hit / len(frames)

    rng = np.random.default_rng(1)
    best = []
    # vectorised coarse pass on centres only
    Q = rng.normal(size=(nrot, 4))
    Q /= np.linalg.norm(Q, axis=1)[:, None]
    for q in Q:
        R = qtom(q)
        uv, z = project(R, tc, centres)
        ok = z > 0.1
        if ok.mean() < 0.5:
            continue
        sc = 0
        for k in np.nonzero(ok)[0][::4]:
            if np.min(np.linalg.norm(frames[k][0] - uv[k], axis=1)) < 40:
                sc += 1
        best.append((sc, tuple(q)))
    best.sort(reverse=True)
    print(f"  chain rotation coarse score {coarse_score(Rc, tc) * 100:.0f}%")

    def leds_world(frame):
        out = []
        for d, p, R in frame[1]:
            for L in dev[d].values():
                out.append((p + R @ L['pos'], R @ L['normal']))
        return out

    LW = [leds_world(f) for f in frames]

    def fine(R, t, gates=(60, 30, 15, 8, 5)):
        for gate in gates:
            for it in range(8):
                rows = []
                for f, lw in zip(frames, LW):
                    P = np.array([a for a, n in lw])
                    N = np.array([n for a, n in lw])
                    cam_pos = t
                    view = (cam_pos - P)
                    view /= np.linalg.norm(view, axis=1)[:, None]
                    vis = np.sum(N * view, 1) > np.cos(np.radians(70))
                    if not vis.any():
                        continue
                    uv, z = project(R, t, P[vis])
                    good = z > 0.05
                    for u, pt in zip(uv[good], P[vis][good]):
                        dd = np.linalg.norm(f[0] - u, axis=1)
                        j = np.argmin(dd)
                        if dd[j] < gate:
                            rows.append((pt, f[0][j]))
                if len(rows) < 30:
                    return R, t, 0, 0
                Pm = np.array([r[0] for r in rows])
                Bm = np.array([r[1] for r in rows])

                def resid(x):
                    R2 = R @ rodrigues(x[:3])
                    uv, _ = project(R2, t + x[3:], Pm)
                    return (uv - Bm).ravel()

                r0 = resid(np.zeros(6))
                J = np.zeros((len(r0), 6))
                for k in range(6):
                    e = np.zeros(6)
                    e[k] = 1e-4 if k < 3 else 1e-4
                    J[:, k] = (resid(e) - r0) / 1e-4
                w = 1 / np.maximum(np.abs(r0), 1.0)  # robust
                dx = np.linalg.lstsq(J * w[:, None], -r0 * w, rcond=None)[0]
                R = R @ rodrigues(dx[:3])
                t = t + dx[3:]
        rr = np.abs(resid(np.zeros(6))).reshape(-1, 2)
        e = np.linalg.norm(rr, axis=1)
        return R, t, len(rows), np.median(e)

    results = []
    cands = [(Rc, 'chain')] + [(qtom(np.array(q)), f'coarse #{k}') for k, (sc, q) in enumerate(best[:6])]
    for R0, name in cands:
        R, t, n, med = fine(R0.copy(), tc.copy())
        if n:
            # hit rate: visible LEDs within 3px of a blob
            hits = tot = 0
            for f, lw in zip(frames, LW):
                P = np.array([a for a, nn in lw])
                N = np.array([nn for a, nn in lw])
                view = t - P
                view /= np.linalg.norm(view, axis=1)[:, None]
                vis = np.sum(N * view, 1) > np.cos(np.radians(60))
                uv, z = project(R, t, P[vis])
                inimg = (z > 0.05) & (uv[:, 0] >= 0) & (uv[:, 0] < 1016) & (uv[:, 1] >= 0) & (uv[:, 1] < 1016)
                for u in uv[inimg]:
                    tot += 1
                    hits += np.min(np.linalg.norm(f[0] - u, axis=1)) < 3
            ang_chain = np.degrees(np.arccos(np.clip((np.trace(Rc.T @ R) - 1) / 2, -1, 1)))
            results.append((hits / max(1, tot), name, R, t, n, med, ang_chain, tot))
    results.sort(key=lambda r: -r[0])
    for hr, name, R, t, n, med, ac, tot in results[:4]:
        print(f"  from {name:10s}: {n} matches, median reprojection {med:.2f}px | LEDs within 3px {hr * 100:.1f}% of {tot}"
              f" | t {np.round(t * 100, 2)} cm (chain {np.round(tc * 100, 2)}) | rotation {ac:.1f} deg from the chain")
    if results:
        hr, name, R, t, *_ = results[0]
        np.save(path + f'.cam{ci}_fit.npy', np.concatenate([R.ravel(), t]))


if __name__ == '__main__':
    main()
