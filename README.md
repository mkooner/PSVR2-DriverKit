# PSVR2-DriverKit

PlayStation VR2 on Linux: 6DoF head tracking, Sense controller tracking and SteamVR support through Monado, plus the
PSVR2-DriverKit launcher and the tools used to measure and debug it. Private work in progress.

## Layout

| Path | What |
|---|---|
| `monado/` | Monado fork (BSL-1.0): upstream Monado + Beyley's PS VR2 work + this project. PS VR2 headset driver (`src/xrt/drivers/psvr2`: SLAM/IMU pose filter, cameras, eye tracking), Sense controllers (`src/xrt/drivers/pssense`: constellation LED tracking, IMU fusion), and the SteamVR plugin (`src/xrt/state_trackers/steamvr_drv`), including driver direct mode into Monado's compositor (`ovrd_direct_mode.hpp`). |
| `launcher-gui/` | PSVR2-DriverKit: device status, start/stop SteamVR and monado-service, launch OpenXR apps, driver settings, headset refresh rate (Rust, eframe). |
| `launch-steamvr.sh`, `launch-openxr.sh` | Run SteamVR on the Monado plugin, or an OpenXR app on monado-service, with logging (see below). |
| `analysis/` | Analyses of the recorded logs (frame timing, pose and on-screen jitter, controller tracking), with synthetic tests in `analysis/harness/`. |
| `tools/`, `pose-sim/`, `ctreplay/`, `vio/`, ... | Probes (SteamVR frame timing, poses), the OpenXR frame-timing layer, replay tools for the pose filter and the controller tracker, Basalt VIO pipeline. |
| `docs/` | Research notes and reports. |

## Build

    cmake -S monado -B monado/build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
    ninja -C monado/build                  # monado-service, the SteamVR plugin (monado/build/steamvr-monado), tests
    (cd launcher-gui && cargo build --release)   # launcher-gui/target/release/PSVR2-DriverKit

The scripts look for the Monado build in `../trial/build` (the original working layout); in this repository point
them at it with `MONADO_BUILD=$PWD/monado/build` (and `MONADO_SRC=$PWD/monado` for the tools built against the
sources).

---

## Test kit details

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
