# PS VR2 controller tracking — root causes

Status: 2026-11-16. Measurements are from hardware runs (logs in the psvr2-test kit) unless marked as simulation or
offline replay. Ground truth for absolute controller position is not available; the checks below use internal
consistency (cameras against each other, optical against the controllers' IMUs, geometry).

## Already fixed (on `main`, confirmed on hardware)

| Cause | Evidence | Fix |
|---|---|---|
| Bluetooth send flooding | 47 reports/s per controller, `BT socket write error (11)`, controllers dropping | Rate cap, send only on change, 100 ms keepalive (`1d0051370`, `ea5285d87`); 0 errors since |
| LED samples marked applied before the controller had them | count calibration saw more blobs with LEDs off than on | Accept only the echo of the sample's report or later (`569c82671`) |
| Camera timestamps 23-81 ms in the future, drifting ~120 us/s | LD stats `ts host-now` +23..+81 ms in every run 21:36-23:34 | Offset follows the VTS clock tracker, re-latch > 2 ms, 10 us/frame slew (`4bf00b8f0`); -8 ms since |
| Double OpenCV->OpenXR camera flip | controllers behind the head in 87% of solves | Removed for the calibration mount (`4d525e6bb`) |
| Hold fooled by the other controller, bogus 662 ppm drift | 23:39: one controller lost at ~15 s, probes said "centred" | Own-LED differencing, edge-margin probes, drift from the hold only (`ba1a74acb`); simulation reproduces the old failure and holds in 4/4 scenarios |

## Current root cause: wrong camera-to-camera chain

**Symptom (00:04 run):** right controller tracked but "in the wrong spot", left flickering.

**Evidence:**
1. Solves of the same controller < 50 ms apart jump > 10 cm in 0.7-5.9% of same-camera pairs but 31-42% of
   different-camera pairs: each camera is self-consistent, the cameras disagree.
2. Offline replay with the driver's own code and Monado's math (harness `psvr2-test/analysis/harness`, validated: it
   reproduces the driver's logged camera positions to the last digit), on the 2621 solves whose camera label is proven
   (they reproduce the driver's world position within 5 mm):

| Chain | Camera 0-1 disagreement (median) | Controllers below / in front of head | Cameras 0 / 1 facing | Tilt vs IMU, best constant LED-IMU rotation (L / R) |
|---|---|---|---|---|
| As built: mount `Rx(-15)*R0`, relation rotation transposed | **43.9 cm** | 33% / 99% | yaw -15/+19, pitch +30/+25 | 9.6 / 14.2 deg |
| **Mount `Rx(-15)*R0^T`, relation rotation as stored** | **3.6 cm** | 94% / 100% | yaw **+15/-15**, pitch 0/-2 | **4.9 / 7.7 deg** |

The candidate puts cameras 0 and 1 30 deg apart, symmetric and facing outward, which matches the independently
measured 30.6 deg for the bottom pair in PSVR2-Documentation. Three independent measures agree.

**Why:** the chain built camera i as `Rx(-15) * R0 * (R relation)^T`; the calibration's matrices are evidently stored
in the other convention (row/column) from the one the code assumed for both the 0x6f0 mount and the relations.

**Not verified:** cameras 2 and 3 (39 and 3 verified solves); the relation direction rule for them; the camera
positions relative to the head (camera 0 is at the head origin; Sony's offsets are not in the calibration block).

## Secondary causes (evidence, not yet fixed)

- **IMU timestamps biased by the one-way clock:** host<->controller uses a one-way minimum-delay estimate, biased by the
  minimum Bluetooth latency (documented -6 ms on a synthetic link; our measured round trips are 9-15 ms). The LED
  calibration absorbs a constant bias, but IMU samples are placed that much off the optical frames in fusion.
- **Configured LED->IMU rotation:** with the candidate chain, the driver's `Rx(-50.27)` leaves 20-23 deg of tilt
  mismatch while a fitted constant leaves 5-8 deg; worth refitting once the chain is fixed.
- **Multi-controller visibility by total blob count:** works only with on/off differencing under the token; PSVR2Toolkit
  moved to per-controller matched-LED counts.
- **Solve rate:** ~10/s of a possible 60/s per controller.

## Unexplained

- **Head shake** (0.9-2.5 pose-filter snaps per stats period in recent runs vs 0.1-0.4 earlier): no code path from the
  controller changes to head tracking was found; needs an A/B run with the controllers off and with
  `PSVR2_POSE_FILTER=0`.
