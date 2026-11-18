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

## Plan steps 1-6 (2026-11-16), offline / simulation results, not yet tested on hardware

| Step | Commit | Offline check | Result |
|---|---|---|---|
| 1 Camera chain (mount R0^T, relations as stored) | `81f905182` | Replay of 2621 verified solves through the driver's chain with Monado's math | cam 0-1 disagreement 43.9 -> 3.6 cm; +-15 deg outward; 94% below / 100% in front; tilt 9.6/14.2 -> 4.9/7.7 deg |
| 2 IMU times on an IMU-timestamp clock | `624f58dea` | `analysis/clock_sim.c`, driver estimator, synthetic reports from measured numbers | old: counter gap + 0.18 ms (3.18 ms at 0 gap, 1003 ms at 1 s); new: 15 us median / 24 us p95 beyond the one-way bias. Echo estimate logged only |
| 3 LED -> IMU -79.77 deg (configurable) | `5b2429c4b` | Pure X refit on the same solves, per hand and per half run | -30/-29 deg residual, halves -30/-31, -28/-30; tilt 20.4/23.3 -> 5.7/11.7 deg; trades off with camera pitch by 3-4 deg |
| 4 Per-controller visibility | none | 00:04 hold probes | Both controllers held ~45 s with own-LED differencing; no interference observed: no change made |
| 5 Tracking state to SteamVR | none | 00:04 FINAL flags | Running_OK 61/74%, explicit Fallback_RotationOnly otherwise, 25/22 spans (mean 0.63/0.32 s, longest 8.7/3.2 s), all recovered: already correct |
| 6 Camera rig offset (configurable, no default) | `d8a34cfb7` | `analysis/rig_offset.py` self-test | recovers a known offset to <1 cm; the 00:04 run has 0 usable windows: needs the dedicated recording |
