# PS VR2 test kit

For the Monado branch `psvr2-headset-constellation` (worktree `../trial`, build in `../trial/build`).

- `launch-steamvr.sh`: run SteamVR on our build's Monado plugin. Temporarily swaps Sony's driver out of
  `~/.config/openvr/openvrpaths.vrpath` and restores it on exit (`--restore` if a run was killed).
- `run-test.sh [seconds]`: run `monado-service` alone with tracking debug logs.
- `summarize.sh <log>`: stage-by-stage summary of a run (both scripts call it).
- `pose-sim/`: head pose simulator, old vs new pose pipeline. `./build.sh`, then
  `./pose_sim [slam_hz] [prediction_ms] [seconds]` or `./pose_sim --replay logs/record-*.csv`.

Options (environment, combine as needed):
- `POSE_FILTER=0`: previous head pose behaviour, for A/B testing.
- `PSVR2_FLOOR_HEIGHT=1.2`: head height above the floor when the headset starts (default 1.6 m, standing).
- `OVRD_PLAY_AREA_SIZE=3`: size of the driver-provided play area in meters (default 2).
- `ROOM_SETUP=1`: skip the driver-provided play area and run SteamVR's own room setup.
 Logs and raw IMU/SLAM recordings go to `logs/`.
