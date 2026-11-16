#!/usr/bin/env python3
"""
Searches one convention for the calibration block's camera relations that explains every camera: from the chain the
driver built (Q, p per camera, parents 1<-0, 2<-0, 3<-2), recover the stored relation (O, t) for each possible storage
direction (the driver inverts when from > to), map it through a candidate convention, and score each camera by camera
0's controller LEDs landing on its blobs. usage: chain_unify.py <dataset recorded with the stored chain>
"""
import sys
import itertools
import numpy as np
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from chain_rules import build, hit_rate, F

ROT = {'O': lambda O: O, 'O^T': lambda O: O.T, 'FOF': lambda O: F @ O @ F, 'FO^TF': lambda O: F @ O.T @ F}
TR = {'t': lambda O, t: t, 'Ft': lambda O, t: F @ t, 'Ot': lambda O, t: O @ t, 'O^Tt': lambda O, t: O.T @ t,
      'FOt': lambda O, t: F @ O @ t, 'FO^Tt': lambda O, t: F @ O.T @ t, 'OFt': lambda O, t: O @ F @ t,
      'O^TFt': lambda O, t: O.T @ F @ t}


def invpose(R, t):
    return R.T, -R.T @ t


def main():
    chain, rel, data = build(sys.argv[1])
    res = []
    for (rn, rf), (tn, tf), direction in itertools.product(ROT.items(), TR.items(), ('to-in-from', 'from-in-to')):
        X = {0: (np.eye(3), np.zeros(3))}
        per = []
        flags = []
        for i, par in ((1, 0), (2, 0), (3, 2)):
            best = (-1, None, None)
            for inverted in (False, True):
                Q, p = rel[i]
                O, t = (Q, p) if not inverted else invpose(Q, p)
                R, tt = rf(O), tf(O, t)
                # stored relation maps parent<->child; "inverted" means it was stored child->parent
                child_in_parent = (R, tt) if (direction == 'to-in-from') != inverted else invpose(R, tt)
                Rp, tp = X[par]
                Ri, ti = Rp @ child_in_parent[0], tp + Rp @ child_in_parent[1]
                h = hit_rate(*data[i], Ri, ti)[0]
                if h > best[0]:
                    best = (h, (Ri, ti), inverted)
            X[i] = best[1]
            per.append(best[0])
            flags.append(best[2])
        res.append((min(per), rn, tn, direction, per, flags))
    res.sort(key=lambda r: -r[0])
    for m, rn, tn, d, per, flags in res[:6]:
        print(f"rotation {rn:6s} translation {tn:6s} {d:10s}: cam1 {per[0] * 100:5.1f}% cam2 {per[1] * 100:5.1f}% "
              f"cam3 {per[2] * 100:5.1f}% | stored inverted (from > to): {flags}")


if __name__ == '__main__':
    main()
