#!/usr/bin/env python3
"""
Simulates a non-disruptive hold check: instead of moving the blink to the window edges and turning the LEDs off,
nudge it +-delta around the centre and compare the brightness of this controller's matched LEDs at the two sides.

Geometry (measured): blink B = 1500us, exposure E ~1060us, window W = B + E. With the blink centred the exposure is
fully inside it (plateau +-(B-E)/2 = +-220us); brightness is the overlapped fraction of the exposure.
Latching: the controller applies new timing at most every ~250ms (PSVR2-Documentation), so each side is held for one
latch and measured over the frames in it (~12 at 60Hz, minus the ~3 frames settling).
Noise: per-frame brightness varies with distance, angle and motion; modelled as a slow multiplicative drift
(controller moving, +-30% over seconds) plus per-frame noise (sigma 15%).
"""
import random

B, E = 1500.0, 1060.0
W = B + E
HALF = W / 2


def overlap(err):
    """Fraction of the exposure [0, E] covered by the blink whose centre is err from the window centre."""
    start = (E - B) / 2 + err
    lo, hi = max(0.0, start), min(E, start + B)
    return max(0.0, hi - lo) / E


def run(drift_ppm, init_err, delta, ratio_thresh, seconds=300, seed=1, frames_per_side=9):
    rng = random.Random(seed)
    t, err, trim = 0.0, init_err, 0.0
    last, interval, streak = 0.0, 3.0, 0
    corr, tint, est_start = 0.0, 0.0, 0.0
    probes = wrong = 0
    lost = None
    worst = 0.0
    min_vis = 1.0
    scene = 1.0
    dt = 1 / 60
    while t < seconds:
        scene = min(1.5, max(0.5, scene * (1 + rng.gauss(0, 0.01))))  # slow brightness change from motion
        err += (trim - drift_ppm) * dt
        t += dt
        worst = max(worst, abs(err))
        if lost is None and abs(err) >= HALF:
            lost = t
        if t - last < interval:
            continue
        dtp = t - last
        last = t
        probes += 1

        def measure(offset):
            # frames in one latch, each with the slow scene factor drifting along
            vals = []
            nonlocal scene
            for _ in range(frames_per_side):
                scene = min(1.5, max(0.5, scene * (1 + rng.gauss(0, 0.01))))
                vals.append(overlap(err + offset) * scene * (1 + rng.gauss(0, 0.15)))
            return sum(vals) / len(vals)

        b_minus = measure(-delta)
        b_plus = measure(+delta)
        min_vis = min(min_vis, overlap(err - delta), overlap(err + delta))
        e = 0
        if b_minus < ratio_thresh * b_plus:
            e = 1  # the early side is dimmer: the blink sits early, move it later
        elif b_plus < ratio_thresh * b_minus:
            e = -1
        truth = 0 if abs(err) < 120 else (1 if err < 0 else -1)
        if e != 0 and e != truth:
            wrong += 1
        tint += trim * dtp
        if e:
            err += e * 100
            corr += e * 100
        T = t - est_start
        if T >= 10:
            rate = (corr + tint) / T
            if abs(rate) <= 50:
                trim = rate
        if e == 0:
            streak += 1
            if streak >= 3:
                interval = min(30.0, interval * 2)
        else:
            streak = 0
            interval = 3.0
    return probes, wrong, lost, worst, min_vis


if __name__ == "__main__":
    for delta in (300.0, 400.0, 500.0):
        for thresh in (0.85, 0.80):
            lost = 0
            worst = 0
            wrong = 0
            probes = 0
            minvis = 1.0
            n = 0
            for d in (47, 25, 10, 3, -3, -10, -25, -47):
                for ie in (400, 200, 0, -200, -400):
                    for seed in (1, 2, 3):
                        p, w, lo, mx, mv = run(d, ie, delta, thresh, seed=seed)
                        n += 1
                        probes += p
                        wrong += w
                        lost += lo is not None
                        worst = max(worst, mx)
                        minvis = min(minvis, mv)
            print(f"nudge +-{delta:.0f}us, dimmer-side threshold {thresh:.2f}: {n} runs | lost {lost} | worst error {worst:.0f}us "
                  f"(loss {HALF:.0f}) | checks/min {probes / n / 5:.1f} | wrong-direction corrections {100 * wrong / max(1, probes):.1f}% "
                  f"| LEDs exposed during checks >= {100 * minvis:.0f}%")
