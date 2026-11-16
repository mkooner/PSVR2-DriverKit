# PS VR2 controller LED sync — test results

Status: 2026-11-16. One headset, two Sense controllers, Linux, SteamVR. "Hardware" rows come from real runs; rows
marked "simulation" or "offline replay" do not. Single runs per build (no repeated trials yet); sample counts given.
Not measured yet: CPU and memory overhead, end-to-end latency, stationary jitter (no ground truth rig).

## Hardware runs (selected)

| Run | Build | Bluetooth (reports/s, round trip) | Camera ts vs host | Calibration | Solves (L/R) | Notes |
|---|---|---|---|---|---|---|
| 22:33 | `ea5285d87`- | 47/s, 0.2-1.3 s | +75..+81 ms | 0 done | 22 | congested; old adapter |
| 23:00 | `b7d360481` | 10-24/s, 9-11 ms | +23..+29 ms | 1 done | 1549 | new adapter |
| 23:39 | `4bf00b8f0` | ~23/s, 9-11 ms | **-8 ms** | **2 done** | 511/400 (one lost at ~15 s) | timestamp fix |
| 23:54 | `ba1a74acb` | ~23/s | -8 ms | 3 done, held ~50 s | 526/212 | controllers above/behind head |
| 00:04 | `4d525e6bb` | ~23/s | -8 ms | 2 done | 1923/2536 | in front; cameras disagree |

Head pose (snaps per stats period): 0.1-0.4 in runs before 22:33, 0.5-2.5 after.

## Simulation: hold logic (`psvr2-test/analysis/hold_sim.py`)

Measured inputs: window 2559 us, blink 1500 us, own LEDs 13-38 blobs, other controller 13-25, background 0-8.

| Scenario | Old | New |
|---|---|---|
| +500 ppm bad trim, 5 ppm drift, other lit | lost 3 s, reports "centred" (reproduces 23:39) | never lost, trim +4.4 ppm |
| +47 ppm drift, other lit | held, error -721 us | never lost, trim +46.4 ppm, error -75 us |
| -3 ppm drift | error +650 us | trim -2.0 ppm, error +126 us |
| +47 ppm, other off | lost 17 s | never lost |

## Offline replay: camera chain (`psvr2-test/analysis/variants.py`, 00:04 run, 2621 verified solves)

| Chain | Cam 0-1 disagreement | Below / front | Tilt vs IMU (L/R) |
|---|---|---|---|
| As built | 43.9 cm | 33% / 99% | 9.6 / 14.2 deg |
| Mount R0^T, relation as stored | 3.6 cm | 94% / 100% | 4.9 / 7.7 deg |

## Remaining issues

Camera chain (step 1, proposed), cameras 2/3 unverified, one-way clock bias, LED->IMU rotation, solve rate,
head shake.
