# PS VR2 test kit

For the Monado branch `psvr2-headset-constellation` (worktree `../trial`, build in `../trial/build`).

- `launch-steamvr.sh`: run SteamVR on our build's Monado plugin. Temporarily swaps Sony's driver out of
  `~/.config/openvr/openvrpaths.vrpath` and restores it on exit (`--restore` if a run was killed).
- `run-test.sh [seconds]`: run `monado-service` alone with tracking debug logs.
- `summarize.sh <log>`: stage-by-stage summary of a run (both scripts call it).
- `pose-sim/`: head pose simulator, old vs new pose pipeline. `./build.sh`, then
  `./pose_sim [slam_hz] [prediction_ms] [seconds]` or `./pose_sim --replay logs/record-*.csv`.

A/B the head pose filter with `POSE_FILTER=0 ./launch-steamvr.sh`. Logs and raw IMU/SLAM recordings go to `logs/`.
