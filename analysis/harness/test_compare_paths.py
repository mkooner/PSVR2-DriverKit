#!/usr/bin/env python3
# Synthetic check of compare_paths.py (Monado compositor log format): the game's poses are in another space (rotated
# 30 deg about y, shifted), with a planted +-1 mm alternating position error and every 40th frame repeated. The fitted
# space transform must not hide the error: expected position swing 2.0 mm, rotation swing ~0, repeats found.
import math, os, sys, tempfile
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import compare_paths as m

P_NS = 11_122_000


def head(t):
    a = math.radians(60) * t  # turning 60 deg/s, swaying 0.15 m/s
    return np.r_[0.15 * t, 1.6, 0.0, 0, math.sin(a / 2), 0, math.cos(a / 2)]


Rg = np.array([0, math.sin(math.radians(15)), 0, math.cos(math.radians(15))])  # game space = Rg^-1 * (device - Tg)
Tg = np.array([0.3, -0.2, 0.5])


def to_game(p):
    q = m.qmul(m.qconj(Rg), p[3:])
    x = m.qrot(m.qconj(Rg), p[:3] - Tg)
    return np.r_[x, q]


f7 = lambda v: ','.join('%.7f' % x for x in v)
ok = True
with tempfile.TemporaryDirectory() as d:
    path = os.path.join(d, 'compframes-test.csv')
    t0 = 2_000_000_000
    with open(path, 'w') as f:
        for k in range(int(14e9 / 2e6)):
            t = t0 + k * 2_000_000
            f.write('T,%d,%s\n' % (t, f7(head(t / 1e9))))
        prev = None
        for i in range(1000):
            vb = t0 + 500_000_000 + i * P_NS
            pred = vb + 4_000_000
            f.write('B,%d\nP,%d,%d,%d\n' % (vb, vb - 3_000_000, i + 1, vb))
            ts = pred if i % 40 != 39 else prev
            f.write('W,%d,%d,%d,1,%d,%s\n' % (vb - 5_000_000, i + 1, pred, ts, f7(head(pred / 1e9))))
            g = head(ts / 1e9)
            g[0] += 0.001 if i % 2 else -0.001
            gp = to_game(g)
            e0 = gp.copy(); e1 = gp.copy()
            e0[:3] += m.qrot(gp[3:], np.array([-0.032, 0, 0])); e1[:3] += m.qrot(gp[3:], np.array([0.032, 0, 0]))
            f.write('Y,%d,%d,%s,%s\n' % (i + 1, ts, f7(e0), f7(e1)))
            prev = ts
    frames, truth, info = m.load_monado(path)
    r = m.measure(frames, truth)
    sw = np.linalg.norm(np.diff(r['gpos'], axis=0), axis=1)
    rs = np.linalg.norm(np.diff(r['grot'], axis=0), axis=1)
    rep = ~r['repeat'][1:] & ~r['repeat'][:-1]
    checks = [(abs(np.median(sw[rep]) - 2.0) < 0.05, 'planted position swing found through the space fit: %.3f mm (2.0)' % np.median(sw[rep])),
              (np.median(rs[rep]) < 0.01, 'no rotation swing invented: %.4f deg' % np.median(rs[rep])),
              (int(r['repeat'].sum()) == 25, 'repeats found: %d (25)' % int(r['repeat'].sum())),
              (abs(info['photon_ms'] - 4.0) < 0.01, 'photon offset %.2f ms (4.00)' % info['photon_ms'])]
    for good, what in checks:
        print('%s: %s' % ('PASS' if good else 'FAIL', what)); ok &= good
sys.exit(0 if ok else 1)
