#!/usr/bin/env python3
# Synthetic check of shown_pose_error.py: a head turning 60 deg/s and moving 200 mm/s; the render pose is the true
# pose at the display time plus a planted position error alternating +-1.5 mm (3 mm frame-to-frame) and a constant
# 0.5 deg rotation; the prediction is exact. The analysis must recover those numbers.
import math, os, sys, tempfile
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import shown_pose_error as m


def truth(t):
    a = math.radians(60) * t
    return np.r_[0.2 * t, 1.6, 0.0, 0, math.sin(a / 2), 0, math.cos(a / 2)]


ok = True
with tempfile.TemporaryDirectory() as d:
    path = os.path.join(d, 'direct.csv')
    off = np.array([math.sin(math.radians(0.25)), 0, 0, math.cos(math.radians(0.25))])  # 0.5 deg about x
    with open(path, 'w') as f:
        for k in range(900):
            now = 1.0 + k / 90.0
            for s in (now, now + 0.004):
                p = truth(s)
                f.write('T,%d,%s\n' % (s * 1e9, ','.join('%.7f' % v for v in p)))
            disp = now + 0.02
            tr = truth(disp)
            r = tr.copy()
            r[0] += 0.0015 if k % 2 else -0.0015
            r[3:] = m.qmul(off, tr[3:])
            f.write('R,%d,%d,%d,%s,%s\n' % ((now + 0.002) * 1e9, k, disp * 1e9, ','.join('%.7f' % v for v in r),
                                           ','.join('%.7f' % v for v in tr)))
    R, T = m.load(path)
    rows = m.analyse(R, T)
    rp = np.array([x[2] for x in rows]); rr = np.array([x[1] for x in rows]); pp = np.array([x[4] for x in rows])
    pj = np.median(np.linalg.norm(np.diff(rp, axis=0), axis=1))
    ra = np.median(np.linalg.norm(rr, axis=1))
    pe = np.median(np.linalg.norm(pp, axis=1))
    checks = [(abs(pj - 3.0) < 0.1, 'position frame-to-frame %.2f mm (planted 3.0)' % pj),
              (abs(ra - 0.5) < 0.02, 'rotation off %.3f deg (planted 0.5)' % ra),
              (pe < 0.05, 'exact prediction reads %.3f mm' % pe),
              (len(rows) > 850, '%d of 900 frames used' % len(rows))]
    for good, what in checks:
        print('%s: %s' % ('PASS' if good else 'FAIL', what))
        ok &= good
sys.exit(0 if ok else 1)
