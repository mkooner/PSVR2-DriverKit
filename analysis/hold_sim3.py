#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""
LED timing hold (nudge check) with what the 00:06 run showed and dither_sim.py didn't model: the probe interval
doubling to 30 s while "centred" with no drift rate learned yet, the controller untracked for stretches (probes read
"not in view": no correction), the tracker losing a controller whose LEDs are poorly exposed, and recalibration after
the 20 s keep. Exposures as measured (window 2106-2440 us with a 1500 us blink: E 606-940 us), drift 12-15 ppm as
measured, and +-47 ppm as the worst clock seen.

Logic variants:
  current  ratio 0.80 -> one 100 us step; interval x2 after 3 centred probes, up to 30 s
  cap      as current, but the interval stays <= 6 s until a drift rate has been measured
  prop     correction = half the error estimated from the brightness balance (deadband, clamp 200 us)
  both     prop + cap
Metrics over 300 s: share of the in-view time the LEDs are at least half exposed, timing losses (|err| past the window),
recalibrations, worst error, probes per minute, share of probes with the LEDs under half exposed.
"""
import random

B = 1500.0


def overlap(err, E):
    start = (E - B) / 2 + err
    lo, hi = max(0.0, start), min(E, start + B)
    return max(0.0, hi - lo) / E


def run(logic, E, drift_ppm, seed, seconds=300.0, delta=300.0, tracked_mean=6.0, E_after=None, E_change_t=40.0):
    rng = random.Random(seed)
    E0 = E
    t, err, trim = 0.0, rng.gauss(0, 60), 0.0
    last, interval, streak = 0.0, 3.0, 0
    corr, tint, hold_start = 0.0, 0.0, 0.0
    rate_known = False
    seen_since_calib, unseen_since = False, None
    probes = bad_probes = recals = losses = 0
    in_view_time = good_time = 0.0
    worst = 0.0
    scene = 1.0
    dt = 1 / 60
    # untracked/out-of-view stretches: tracked mean 6 s, untracked mean 1.5 s
    tracked_seg, seg_end = True, rng.expovariate(1 / tracked_mean)
    lost_now = False
    while t < seconds:
        if t >= seg_end:
            tracked_seg = not tracked_seg
            seg_end = t + rng.expovariate(1 / (tracked_mean if tracked_seg else 1.5))
        E = E_after if (E_after is not None and t >= E_change_t) else E0
        W = B + E
        HALF = W / 2
        E = E_after if (E_after is not None and t >= E_change_t) else E0
        W = B + E
        HALF = W / 2
        scene = min(1.5, max(0.5, scene * (1 + rng.gauss(0, 0.01))))
        err += (trim - drift_ppm) * dt
        t += dt
        worst = max(worst, abs(err))
        ov = overlap(err, E)
        in_view_time += dt
        if ov >= 0.5:
            good_time += dt
        if abs(err) >= HALF and not lost_now:
            losses += 1
            lost_now = True
        if abs(err) < HALF:
            lost_now = False
        tracked = tracked_seg and ov > 0.35  # the tracker needs the LEDs reasonably exposed
        unseen = not tracked
        if t - last < (1.0 if unseen else interval):
            continue
        dtp = t - last
        last = t
        probes += 1

        def measure(offset):
            if not tracked_seg:
                return 0.0
            o = overlap(err + offset, E)
            if o < 0.15:
                return 0.0
            vals = [o * scene * (1 + rng.gauss(0, 0.15)) for _ in range(6)]
            return sum(vals) / len(vals)

        bm, bp = measure(-delta), measure(+delta)
        if min(overlap(err - delta, E), overlap(err + delta, E)) < 0.5:
            bad_probes += 1
        in_view = bm > 0 or bp > 0
        c = 0.0
        if in_view:
            if logic in ("prop", "both"):
                s = bp + bm
                ratio = (bp - bm) / s if s > 0 else 0.0
                if abs(ratio) > 0.04:
                    c = max(-200.0, min(200.0, 0.5 * ratio * (W / 2 - delta)))
                # one side dark: at least a full step toward the lit side
                if bm == 0 and bp > 0:
                    c = max(c, 100.0)
                if bp == 0 and bm > 0:
                    c = min(c, -100.0)
            else:
                if bm < 0.80 * bp:
                    c = 100.0
                elif bp < 0.80 * bm:
                    c = -100.0
        tint += trim * dtp
        if c:
            err += c
            corr += c
        T = t - hold_start
        if in_view and T >= 10:
            rate = (corr + tint) / T
            if abs(rate) <= 50:
                trim = rate
                rate_known = True
        # interval
        if in_view and abs(c) < 50:
            streak += 1
            cap = 6.0 if (logic in ("cap", "both") and not rate_known) else 30.0
            if streak >= 3:
                interval = min(cap, interval * 2)
        else:
            streak = 0
            interval = 3.0
        # loss handling like count_hold_apply
        if in_view:
            seen_since_calib, unseen_since = True, None
        else:
            if unseen_since is None:
                unseen_since = t
            keep = 20.0 if seen_since_calib else 3.0
            if t - unseen_since >= keep and tracked_seg:
                # recalibrate: ~3 s sweep, window found again; trim kept, hold restarts
                recals += 1
                t += 3.0
                err = rng.gauss(0, 60)
                corr = tint = 0.0
                hold_start = t
                rate_known = False if logic in ("current", "prop") else rate_known
                seen_since_calib, unseen_since = False, None
                interval, streak = 3.0, 0
    return dict(good=good_time / in_view_time, losses=losses, recals=recals, worst=worst,
                ppm=probes / (seconds / 60), bad=bad_probes / max(1, probes))


if __name__ == "__main__":
    scen = [
        ("00:06 run: tracked 20 s stretches, 18 ppm, exposure 940 -> 606 us at 40 s", dict(E=940.0, tracked_mean=20.0, E_after=606.0), (18, -18, 15, -15)),
        ("tracked 20 s stretches, exposure 606 us throughout", dict(E=606.0, tracked_mean=20.0), (18, -18, 15, -15)),
        ("tracked 20 s stretches, exposure 940 -> 606 us, 47 ppm", dict(E=940.0, tracked_mean=20.0, E_after=606.0), (47, -47)),
        ("short tracked stretches (6 s), exposure 940 -> 606 us, 18 ppm", dict(E=940.0, tracked_mean=6.0, E_after=606.0), (18, -18)),
    ]
    for title, kw, drifts in scen:
        print(title)
        for logic in ("current", "cap", "prop", "both"):
            agg = dict(good=0, losses=0, recals=0, worst=0, ppm=0, bad=0)
            n = 0
            for d in drifts:
                for seed in range(10):
                    rr = run(logic, kw["E"], d, seed, tracked_mean=kw["tracked_mean"], E_after=kw.get("E_after"))
                    n += 1
                    for k in ("good", "ppm", "bad"):
                        agg[k] += rr[k]
                    agg["losses"] += rr["losses"]
                    agg["recals"] += rr["recals"]
                    agg["worst"] = max(agg["worst"], rr["worst"])
            print(f"  {logic:8s} LEDs >=50% exposed {100 * agg['good'] / n:5.1f}% | timing losses {agg['losses'] / n:4.2f}/run | "
                  f"recals {agg['recals'] / n:4.2f}/run | worst {agg['worst']:5.0f} us | probes/min {agg['ppm'] / n:4.1f} | "
                  f"probes leaving LEDs <50% {100 * agg['bad'] / n:4.1f}%")
