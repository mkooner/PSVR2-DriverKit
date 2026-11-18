# PS VR2 controller LED sync — design and plan

Status: proposal, 2026-11-16. Nothing below is implemented unless marked done. Each step lists the evidence it rests
on and the measurement that will show whether it worked, checked offline on recorded data first where possible.

## Timing architecture (as built, after the fixes so far)

```
controller 3 MHz clock --(BT input @29/@49, host receive time)--> pssense_clock (min-delay, Theil-Sen skew)
headset VTS us clock   --(USB IMU records)--> windowed min-delay tracker --> LD offset (slewed 10 us/frame)
camera exposure (LD header VTS timestamp) --> host time --> timing event (+ blob count) --> LED sync
LED sync (count calibration, hold) --> PRESCAN absolute position on a latched host->controller map, re-latched every
  500 ms (sequence number), advanced by the optically measured drift --> output report
optical solves (per camera, OpenXR camera axes, camera poses from the calibration chain) + IMU --> fusion --> SteamVR
```

## Plan, in order

| Step | Change | Evidence | Success measure |
|---|---|---|---|
| 1 | **Camera chain:** mount `Rx(-15)*R0^T`, relation rotation as stored (`psvr2_constellation_tracking.c`) | Offline replay: camera 0-1 disagreement 43.9 -> 3.6 cm; 30 deg symmetric yaw (doc: 30.6); tilt 9.6/14.2 -> 4.9/7.7 deg | Hardware: different-camera jumps > 10 cm from 31-42% to near the same-camera rate (1-6%); controllers at the hands |
| 2 | **Echo clock:** two-way host-timestamp echo, minimum round trip per 1 s window, landmark least-squares rate (`pssense_clock.h`) | Protocol doc: +-RTT/2 bound, -25 us vs -6 ms one-way bias | Offline: offset vs one-way estimate on recorded reports; hardware: lower fusion overrides |
| 3 | **Refit LED->IMU rotation** from recorded solves once step 1 holds | 20-23 deg with the configured value vs 5-8 deg fitted | Tilt residual 5-8 deg with the new constant |
| 4 | **Per-controller visibility:** count the blobs the tracker assigned to this controller (constellation samples) in the hold, keep on/off differencing for calibration | PSVR2Toolkit `be18951`; 23:39 failure | Both controllers held for the whole run |
| 5 | **Tracking state to SteamVR:** explicit untracked/out-of-range after a timeout instead of a held valid pose | PSMove bridge anti-pattern; mission rule | Pose flags in FINAL logs |
| 6 | **Camera offsets:** camera rig position relative to the head (Sony's config value is not in the block) | Camera 0 at the head origin | Estimate from recorded data, check against the headset geometry |
| 7 | **Head shake A/B** (controllers off; `PSVR2_POSE_FILTER=0`) | Unexplained regression | Snaps per period |

## Calibration, drift and recovery (done, `ba1a74acb` and earlier)

- Calibration: one controller at a time under a token, every step measured with the LEDs at the test offset and then
  off, judged by the difference (> 3 blobs); sweep from the last known window; 25 us edges; accept a plausible window
  after one retry; 20 s limit; LEDs-always-on visibility check after 2 empty sweeps.
- Hold: probes at centre +- (W/2 - 150 us) every 3 s (1 s while unseen) under the token, own-LED differencing,
  100 us corrections, drift = (corrections + trim integral) / time after 10 s within +-50 ppm, recalibrate after 3
  unseen probes.

## Diagnostics

Existing: `PSSENSE_LOG=debug` (send stats, clock, LED round trip, calibration steps, hold probes, OPT/OUT/FINAL poses),
`PSVR2_RECORD_FILE` (IMU, SLAM, LD header and raw LED rows, origin poses, VD blocks incl. calibration),
`psvr2-test/summarize.sh`, `psvr2-test/analysis/`. To add with step 2: a per-report clock CSV (host send, echo, device
time, offset, RTT).
