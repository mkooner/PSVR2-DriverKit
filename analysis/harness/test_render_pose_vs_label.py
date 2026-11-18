#!/usr/bin/env python3
# Synthetic check of render_pose_vs_label.py: labels equal to the game's pose give ~0; labels that are 11 ms ahead
# in every other frame (as late frames) while turning 100 deg/s give a frame-to-frame error change of ~1.1 deg; a
# constant space change between the two is removed (a constant lag is absorbed into it: that is latency, not jitter).
import math, os, sys, tempfile
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import render_pose_vs_label as m


def yaw_q(a):
    return np.array([0, math.sin(a / 2), 0, math.cos(a / 2)])


def write(dirn, label_shift_s, turn_dps):
    xr = os.path.join(dirn, 'xr.csv'); dm = os.path.join(dirn, 'dm.csv')
    R = yaw_q(math.radians(37)); T = np.array([0.3, -0.1, 0.5])
    with open(xr, 'w') as fx, open(dm, 'w') as fd:
        fd.write('C,0,999,43,3040,3100,1,11,12,13,-1\n')
        for k in range(900):
            t = 1.0 + k / 90.0
            a = math.radians(turn_dps) * t
            gq = yaw_q(a); gp = np.array([0.1 * math.sin(t), 1.6, 0.0])
            v = [*(gp - [0.032, 0, 0]), *gq, *(gp + [0.032, 0, 0]), *gq]
            fx.write('V,%d,%d,0,7,66,0,0,3040,3100,%s\n' % (t * 1e9, t * 1e9, ','.join('%.7f' % x for x in v)))
            sh = label_shift_s if k % 2 else 0.0
            a2 = math.radians(turn_dps) * (t + sh)
            lq = m.qmul(R, yaw_q(a2)); gp2 = np.array([0.1 * math.sin(t + sh), 1.6, 0.0])
            lp = m.qrot(R, gp2) + T
            fd.write('S,%d,0,0.017,%f,%f,%f,%f,%f,%f,%f,%d,0,0,0,0,0,1,1,0,0\n' % (
                (t + 0.004) * 1e9, *lp, *lq, 11 + k % 3))
    return xr, dm


ok = True
with tempfile.TemporaryDirectory() as d:
    for shift, turn, expect in ((0.0, 100, 0.0), (0.011, 100, 1.1)):
        xr, dm = write(d, shift, turn)
        game = m.load_game(xr); labels, _ = m.load_labels(dm)
        r = m.analyse(game, labels, (3040, 3100))
        med = float(np.median(r['jit_deg']))
        good = abs(med - expect) < 0.1 and abs(r['R_deg'] - 37) < 1.0
        print('%s: label %.0f ms off in every other frame at %d deg/s -> frame-to-frame error change %.3f deg '
              '(expected %.1f), space change %.1f deg' % ('PASS' if good else 'FAIL', shift * 1000, turn, med, expect,
                                                         r['R_deg']))
        ok &= good
sys.exit(0 if ok else 1)
