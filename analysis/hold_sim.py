#!/usr/bin/env python3
"""
Simulates the count calibration hold, old (before ba1a74acb) vs new logic, with the 23:39 run's measured numbers.

Coordinates: err = (blink position actually applied) - (window centre), in us. The window drifts at d us/s (ppm)
against the untrimmed mapping, the trim advances the blink at `trim` us/s, so err changes by (trim - d) per second.
A probe at centre + x tests visibility at err + x; the LEDs are visible iff |err + x| < W/2.
A left probe going dark (window moved later, err negative) gives e = +1 and moves the centre +100us (err += 100).

Measured (steamvr-20261115-233951.log): B = 1500us, W = 2559us; one controller's LEDs 13-38 blobs when visible;
the other controller lit and steady 13-25 blobs; background 0-8; calibration off-mean ~7 (so old threshold ~10);
probes every 3s (1s while unseen); step 100us; bad initial trim from the re-search artifact +500ppm.
Counts switch within one 25us edge step in the calibration data, so visibility is modelled as a step.
"""
import random
import sys

B, W = 1500.0, 2559.0
HALF = W / 2


def run(logic, d, initial_trim, other_lit=True, seconds=120, seed=1):
    rng = random.Random(seed)
    own = lambda e: max(0.0, rng.gauss(22, 5)) if abs(e) < HALF else 0.0
    t, err, trim = 0.0, 0.0, initial_trim
    last_probe, unseen = 0.0, 0
    corr_sum, trim_int, corrections = 0.0, 0.0, 0
    lost_at, first_wrong_centred, log = None, None, []
    dt = 1 / 60
    while t < seconds:
        err += (trim - d) * dt
        t += dt
        if lost_at is None and abs(err) >= HALF:
            lost_at = t
        if t - last_probe < (1.0 if unseen else 3.0):
            continue
        dtp, last_probe = t - last_probe, t
        oth = max(0.0, rng.gauss(19, 4)) if other_lit else 0.0
        bg = max(0.0, rng.gauss(3, 2))
        total = lambda x: own(err + x) + oth + bg
        if logic == "old":
            p = (W - B) / 2 - 100
            c, l, r = total(0), total(-p), total(+p)
            in_view = c > 10.0
            left_on, right_on = l >= 0.75 * c, r >= 0.75 * c
        else:
            p = W / 2 - 150
            off = oth + bg + rng.gauss(0, 1.5)  # this controller's LEDs off, the rest unchanged
            c, l, r = total(0) - off, total(-p) - off, total(+p) - off
            in_view, left_on, right_on = c > 3, l > 3, r > 3
        e = 0
        if in_view and not left_on and right_on:
            e = 1
        elif in_view and left_on and not right_on:
            e = -1
        if logic == "old":
            if e:
                err += e * 100
                if dtp > 1.0:
                    trim = max(-500, min(500, trim + 0.5 * e * 100 / dtp))
        else:
            trim_int += trim * dtp
            if e:
                err += e * 100
                corr_sum += e * 100
                corrections += 1
            if in_view and t >= 10:
                rate = (corr_sum + trim_int) / t
                if abs(rate) <= 50:
                    trim = rate
        verdict = "not in view" if not in_view else ("centred" if e == 0 else ("later" if e > 0 else "earlier"))
        if verdict == "centred" and abs(err) >= HALF and first_wrong_centred is None:
            first_wrong_centred = t
        unseen = 0 if in_view else unseen + 1
        log.append((round(t), verdict, round(err), round(trim, 1)))
        if logic == "new" and unseen >= 3:
            log.append((round(t), "RECALIBRATE", round(err), round(trim, 1)))
            break
    return dict(lost_at=lost_at, wrong_centred=first_wrong_centred, err=err, trim=trim, log=log)


def show(name, res, n=6):
    lost = f"LEDs lost at {res['lost_at']:.0f}s" if res["lost_at"] else "never lost"
    wc = f", reported 'centred' while lost from {res['wrong_centred']:.0f}s" if res["wrong_centred"] else ""
    print(f"  {name:6s}: {lost}{wc}; final error {res['err']:+.0f}us, trim {res['trim']:+.1f}ppm; last probes {res['log'][-n:]}")


if __name__ == "__main__":
    cases = [
        ("23:39 case: bad +500ppm trim, real drift 5ppm, other controller lit", 5, 500, True),
        ("no bad trim, real drift +47ppm (worst measured clock), other lit", 47, 0, True),
        ("no bad trim, real drift -3ppm, other lit", -3, 0, True),
        ("no bad trim, real drift +47ppm, other controller off", 47, 0, False),
    ]
    for title, d, trim0, other in cases:
        print(title)
        for logic in ("old", "new"):
            # The new code no longer learns the +500ppm from re-searches: it starts with whatever trim the previous
            # hold left (0 on first calibration), so run it from 0 in the 23:39 case and also from +500 to show
            # the hold recovers even if a bad trim slipped in.
            if logic == "new" and trim0 != 0:
                show("new", run("new", d, 0, other))
                show("new*", run("new", d, trim0, other))
            else:
                show(logic, run(logic, d, trim0, other))
        print()
