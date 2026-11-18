#!/usr/bin/env python3
# Synthetic check of screen_jitter.py with planted faults: every 50th displayed frame repeats the previous game frame,
# frames 300-309 are re-warped 0.5 deg off, frames 500-519 were drawn +-1 mm off (alternating), frame 700 misses its
# vblank (shown one refresh late). Everything else is exact. The analysis must find each and read ~0 otherwise.
import math, os, sys, tempfile
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import screen_jitter as m

P_NS = 11_122_000
PHOTON = 1_360_000


def head(t):  # t in s: turning 60 deg/s about y, swaying 0.1 m/s in x
    a = math.radians(60) * t
    return np.r_[0.1 * t, 1.6, 0.0, 0.0, math.sin(a / 2), 0.0, math.cos(a / 2)]


def fmt(v):
    return ','.join('%.7f' % x for x in v)


ok = True
with tempfile.TemporaryDirectory() as d:
    comp, direct = os.path.join(d, 'comp.csv'), os.path.join(d, 'direct.csv')
    t0 = 1_000_000_000
    with open(comp, 'w') as fc, open(direct, 'w') as fd:
        for k in range(0, int(12e9 / 2e6)):  # truth every 2 ms
            t = t0 + k * 2_000_000
            fd.write('T,%d,%s\n' % (t, fmt(head(t / 1e9))))
        prev_disp = None
        for f in range(1000):
            vb = t0 + 500_000_000 + f * P_NS
            pred = vb + PHOTON
            fc.write('B,%d\n' % vb)
            present = vb - 3_000_000 if f != 700 else vb + 1_000_000  # frame 700 misses its vblank
            fc.write('P,%d,%d,%d\n' % (present, f + 1, vb))
            disp = pred if f % 50 != 49 else prev_disp  # repeat: the previous game frame again
            shown = vb + PHOTON if f != 700 else vb + P_NS + PHOTON
            w = head(pred / 1e9)  # re-warp target: the head at the predicted time
            if 300 <= f < 310:
                off = np.array([math.sin(math.radians(0.25)), 0, 0, math.cos(math.radians(0.25))])
                w[3:] = m.qmul(off, w[3:])
            fc.write('W,%d,%d,%d,1,%d,%s\n' % (vb - 5_000_000, f + 1, pred, disp, fmt(w)))
            if f % 50 != 49:
                r = head(disp / 1e9)
                if 500 <= f < 520:
                    r[0] += 0.001 if f % 2 else -0.001
                fd.write('R,%d,%d,%d,%s,%s\n' % (vb - 9_000_000, f + 1, disp, fmt(r), fmt(r)))
            prev_disp = disp
    W, P, B, R, T = m.load(comp, direct)
    rows, photon, skipped = m.analyse(W, P, B, R, T)
    rot = np.array([np.linalg.norm(r['rot']) for r in rows]); pos = np.array([np.linalg.norm(r['pos']) for r in rows])
    slips = [r for r in rows if abs(r['slip']) > 5.6]
    reps = sum(r['repeat'] for r in rows)
    fid = np.array([r['fid'] for r in rows])
    spike = rot[(fid >= 301) & (fid <= 310)]
    calm = rot[(fid < 290) | (fid > 320)]
    # Position: the alternating +-1 mm and the repeats (drawn for the previous frame: 60 deg/s... and 0.1 m/s * 11 ms)
    pos_spike = pos[(fid >= 501) & (fid <= 520)]
    checks = [
        (abs(photon - PHOTON) < 1e4, 'photon offset recovered (%.2f ms)' % (photon / 1e6)),
        (reps >= 19, 'repeats found (%d of 19)' % reps),
        (len(slips) == 1 and abs(slips[0]['slip'] - 11.122) < 0.1 and slips[0]['fid'] == 701,
         'the one late frame found, +1 refresh (%s)' % [round(s['slip'], 2) for s in slips]),
        (abs(np.median(spike) - 0.5) < 0.02, 'rotation spike 0.5 deg found (%.3f)' % np.median(spike)),
        (np.percentile(calm[np.isfinite(calm)], 90) < 0.7 and np.median(calm) < 0.01,
         'rotation elsewhere ~0 (median %.4f deg; the late frame and repeats aside)' % np.median(calm)),
        (abs(np.median(pos_spike) - 1.0) < 0.05, 'position error 1 mm found (%.3f)' % np.median(pos_spike)),
    ]
    for good, what in checks:
        print('%s: %s' % ('PASS' if good else 'FAIL', what))
        ok &= good
    m.report(rows, photon, skipped, len(B))
sys.exit(0 if ok else 1)
