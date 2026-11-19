# Room setup and playspace for the PS VR2 (SteamVR driver)

What the driver does with SteamVR's room (chaperone), why, and what has been checked where. The audit behind the
2026-11-18 changes, with the OpenVR sources, is in `docs/room-setup-audit-2026-11-18.md`.

## How the PS VR2 driver (driver_monado_psvr2) gives SteamVR its tracking and room (2026-11-18)

- **Devices.** One HMD (`TrackedDeviceClass_HMD`) and two controllers (`TrackedDeviceClass_Controller`, role hints
  LeftHand / RightHand), all with `Prop_TrackingSystemName_String` "monado_psvr2". The HMD reports
  `Prop_HmdTrackingStyle_Int32` = InsideOutCameras (officially only UI wording) and the universe
  (`Prop_CurrentUniverseId_Uint64`).
- **One tracking space.** The headset's SLAM, carried across its tracking restarts by the headset driver, is the
  driver space for all three devices; the controllers are tracked relative to the headset. The room transform
  (`T_room_raw`, below) goes into every device's `qWorldFromDriver`, the same value for all, so SteamVR's "raw"
  space is the room and the devices can't separate.
- **The driver's own chaperone file** (as Sony's driver keeps `playstation_vr2/chaperone_info.vrchap`):
  `<SteamVR config>/monado_psvr2/chaperone_info.vrchap`, published as `Prop_DriverProvidedChaperonePath_String`.
  SteamVR's Room Setup writes the rooms drawn there; SteamVR's own file is no longer touched by this driver.
  - Universe **5023**: this session (the headset starts a new tracking world at every start: all 26 recordings of
    2026-11-18 began at 0 0 0). Reset every start, with a temporary square around the start position (`PLAY_AREA`,
    default 2 m; `PLAY_AREA=0`: none). Not your room.
  - Universe **5022**: the saved room. Active once a floor capture anchors the room in this session. It only ever
    holds a room drawn in Room Setup: the driver never makes up bounds for it.
- **Room Setup input.** Room Setup is a legacy-input app; our legacy binding gives it the controllers' poses
  (`/pose/raw`), the trigger and haptics (the missing poses were why tracing didn't work before).
- **Tracking restarts** are carried from the start of the session (before: only after 20 s), and the room is never
  dropped any more: a restart or a jump is logged, and the room stays where it was (give or take the carry's error,
  cm); a new capture on the mark refines it.
- **When Room Setup starts before the room is anchored**, the left controller buzzes three times and the log says
  how to anchor first (a room drawn in the session universe is gone at the next start).

### SteamVR inside Steam's container (Steam games) (2026-11-19)

Steam runs games inside its Steam Linux Runtime container, and they can only connect to a SteamVR running there too.
The full Monado build's plugin needs 293 libraries (190 not in Steam's runtime), so SteamVR used to run on the host
(`VALVE_SKIP_RUNTIME_SAFETY`) and Steam games failed at their first VR call (Hellsweeper: `xrCreateInstance failed`,
then fine once it started its own SteamVR). The container-ready plugin is a separate minimal build (only the PS VR2,
Sense and simulated drivers; no OpenCV/GStreamer/onnxruntime/SDL/SLAM; 25 libraries) with the 13 the runtime lacks
(Ceres, SuiteSparse, glog, gflags, jsoncpp, libjpeg, openvr_api) bundled next to it (`bundle_libs.py`, DT_RPATH
`$ORIGIN`):

```
cmake -S trial -B trial/build-steamvr -DOVRN_BUNDLE_LIBS=ON \
  $(for d in $(grep -oE '^XRT_BUILD_DRIVER_[A-Z0-9_]+' trial/build/CMakeCache.txt | sort -u); do \
    case $d in XRT_BUILD_DRIVER_PSVR2|XRT_BUILD_DRIVER_PSSENSE|XRT_BUILD_DRIVER_SIMULATED) ;; *) echo -D$d=OFF;; esac; done) \
  -DXRT_HAVE_GST=OFF -DXRT_HAVE_ONNXRUNTIME=OFF -DXRT_HAVE_SDL2=OFF -DXRT_FEATURE_DEBUG_GUI=OFF -DXRT_HAVE_OPENCV=OFF \
  -DXRT_FEATURE_SLAM=OFF -DXRT_FEATURE_SERVICE=OFF -DXRT_FEATURE_RENDERDOC=OFF -DXRT_HAVE_BLUETOOTH=OFF -DXRT_HAVE_HIDAPI=OFF
ninja -C trial/build-steamvr driver_monado_psvr2 ovrn_driver_harness
```

`launch-steamvr.sh` (and so the DriverKit) uses it when it's there and starts SteamVR normally, inside the
container; `HOST_STEAMVR=1` (or no build-steamvr) is the old host start. Check without SteamVR:
`~/.local/share/Steam/steamapps/common/SteamLinuxRuntime_sniper/run -- trial/build-steamvr/tests/ovrn_driver_harness`
(all modes pass inside the container; the full build's plugin fails there at `libgstapp-1.0.so.0`). The Basalt referee
is off in this build (it needs the SLAM module).

### Default and custom play area (2026-11-19)

SteamVR gets the DriverKit's default play area (Settings > SteamVR > "Default play area", `PLAY_AREA`, a square, 2 m
by default) until you draw your own room in Room Setup: in this session's universe and in the saved room (the driver
writes its square, marked `monado_rectangle`, into an undrawn room). A room drawn in Room Setup replaces it and is kept.
On the Launcher tab, under the controllers:
- **Set Default Play Area** (SteamVR closed): takes the drawn room out of the driver's file (backed up in `logs/`), so
  the next start uses the default again. The floor anchor stays.
- **Set Custom Play Area**: opens Room Setup (SteamVR running: asked through `tools/vrlaunch`; not running: started
  with `ROOM_SETUP=1`).

### Passthrough (seeing the room through the headset)

The driver gives SteamVR the headset's bottom camera pair, the same cameras and images Sony's driver shows, so
SteamVR's own **Room View** works, in Room Setup or any time.
- SteamVR > Settings > Camera: turn **Enable Camera** on (once). Then Room View from the dashboard's camera button
  (or the "double-click system button" option in the same settings page).
- Greyscale, side by side stereo, brightened (`OVRN_CAMERA_GAMMA`, default 0.5; lower is brighter).
  `OVRN_CAMERA=0` turns the cameras off for SteamVR.
- The headset streams these images (camera mode 0x10) only while SteamVR wants them: on StartVideoStream, or while
  SteamVR's internal camera frame queue has a reader (Room View in SteamVR 2.19 never sent StartVideoStream,
  2026-11-18), then goes back to its default mode 2 s after the last reader left. Log: "ovrn: passthrough wanted
  (...)" / "passthrough on" / "passthrough not wanted any more" / "passthrough off (N frames)".
- **See-through, as on Sony's driver (2026-11-19):** the headset's function button shows the whole camera picture and
  pressing it again goes back to the game. Sony's driver draws the camera itself (direct mode) and asks SteamVR to
  pause the game and render less; here SteamVR draws it (it owns the display): for the session the driver sets
  SteamVR's camera on with the opaque view (camera/enableCamera, roomViewStyle 4 "Opaque Pass-Through", roomView 1),
  and puts your own camera settings back at shutdown (also after a crash, at the next start). While the view is shown
  (SteamVR's VREvent_RoomViewShown / Hidden) it sets Prop_DriverRequestsApplicationPause_Bool and
  Prop_DriverRequestsReducedRendering_Bool, as Sony's driver does. `OVRN_SEE_THROUGH=0` leaves the camera settings
  alone. Checked offline: `tests/ovrn_driver_harness --see-through`.
- **Camera placement for the picture (2026-11-19):** SteamVR is given the cameras' true directions and positions
  (`psvr2_bottom_cameras_in_head`, from the calibration block: camera 0 28.9 deg down 17.4 left, camera 1 29.3 down
  17.2 right, 78.9 mm apart; measured from motion 28.2 down 16.1 left). Before, it got the controller tracker's legacy
  camera chain (camera 0 "0.5 up, 14.7 right", ~40 deg off), which put the picture at the wrong angle (warped and
  swimming). The controller tracker keeps its own chain. Test: `OVRN_CAMERA_TEST_BLOCK=analysis/harness/block.bin
  tests/tests_ovrn_camera "*placement*"`.
- **The headset's own button** (under the visor, the PS5's passthrough button) toggles Room View: the driver reports
  it as the headset's system button, and its compositor binding
  (`resources/input/vrcompositor_binding_playstation_vr2_hmd.json`, the headset profile's `default_bindings`) maps a
  press to SteamVR's "Toggle Room View" (as Valve's own Vive Pro binding maps a double press). Without that binding
  SteamVR ignored the button (2026-11-18). `OVRN_HEAD_BUTTON=off` doesn't report it. The 30 s "Room: devices" log
  line ends with the number of presses the driver saw.
- How: `IVRCameraComponent` on the headset (published OpenVR interface; lens model Extended_FTheta fitted to the
  cameras' own calibration, camera-to-head transforms from the headset's calibration), and also SteamVR's internal
  camera frame queue `/lighthouse/camera/raw_frames` when SteamVR offers it (not in Valve's published header; how
  PSVR2Toolkit gets the picture shown with SteamVR 2.x). Code: `steamvr_psvr2/ovrn_camera.hpp`.
- Checked offline (`tests/tests_ovrn_camera`): the BC4 decode (layout, crop, brightening), the lens fit (recovers a
  fisheye model's coefficients), the published frame path with a fake backend, and a real recorded frame decoded
  (desk, monitor and hands visible, `OVRN_CAMERA_TEST_RECORDING=logs/camera-*.pv2c`). Not checked: SteamVR actually
  showing it, and whether the picture lines up with the room (camera-to-head transforms, lens model).

### Controllers that connect late

A Sense controller that connects after SteamVR started (it wakes a moment later than the other) is added while
SteamVR runs: every 2 s while one is missing the driver looks for it, creates it, joins it to the headset's LED
tracking and adds it to SteamVR ("ovrn: left controller connected after the start ... added").
`OVRN_CONTROLLER_HOTPLUG=0` turns it off. Checked offline with `tests/ovrn_driver_harness --late` (simulated).

### Using it (manual validation)

1. Start SteamVR with the driver (`./launch-steamvr.sh` or the DriverKit launcher).
2. **Anchor the room** (each session): put the left controller on its floor mark, the same way round as always,
   crouch next to it and hold Create for 2.5 s. Short buzz: done (log: `Room: captured ...`, then `Room: SteamVR
   universe 5022 (the saved room)`). Long buzz: rejected, the reason is in the log.
3. **First time only: draw the room.** SteamVR > Room Setup > Room-scale. The controllers should show as ready;
   calibrate the floor; trace the room holding the trigger; accept. Log: `Room: Room Setup committed: universe 5022:
   ... N walls ... geometry valid` (or the problems found).
4. **Play:** walk towards a wall: SteamVR draws its boundary. It must stay in place while turning, walking, covering
   a controller, and after the headset's tracking restarts.
5. **Restart SteamVR:** repeat step 2 on the same mark; the boundary is back where it was.

### Diagnostics (all `Room:` lines in vrserver's log)

`chaperone file ... prepared`, `at start` / `now active` / `SteamVR changed the chaperone file` / `Room Setup
committed` (universe, play area, walls, perimeter, floor area, winding, standing origin, problems), `state` every 30 s
(universe, anchor, room transform, head raw -> world), `devices in the world frame` every 30 s (head and controllers),
captures, tracking restarts and jumps.

### Automated checks

- `tests/tests_ovrn_room.cpp` (14 cases): transforms and captures (below), the driver-owned file with its own
  universes (folder created, no made-up room bounds, a drawn room kept), and the boundary checker
  (`ovrd_room_check_universe`): SteamVR's own rectangle and a traced L-shaped room valid in both directions; an open
  boundary, crossing walls, a repeated corner, a play area outside the room, a wall off the floor, a slanted quad, a
  short quad and a missing standing origin reported.
- `tests/tests_psvr2_slam_carry.cpp`: tracking restarts carried from the start, including a burst like the 17:24 run
  (no step bigger than walking).
- `ovrn_driver_harness --room`: the whole start-up against a scratch SteamVR config folder: the published file path,
  both universes in it, the session universe active, SteamVR's own file untouched, inside-out tracking style.
- Replay (`pose-sim/steamvr_sim`) of three 2026-11-18 recordings: the largest start-up position step is 4-5 mm (a
  4.7 m restart at 6.7 s in the 16:18 run is carried).

Not checked offline: SteamVR's Room Setup itself (needs the headset): tracing, the commit into our file, the
boundary in games, Room Setup's events reaching the driver (the file watch logs the commit either way).

## The old plugin (driver_monado) and the original design

The old plugin (`steamvr_drv`) was removed on 2026-11-18. Its room code lives on in the PS VR2 driver as
`steamvr_psvr2/ovrn_room.cpp`, `ovrn_room.hpp` and `ovrn_room_state.hpp` (same functions, same saved files:
`~/.config/monado/ovrd_room.json`). The section below is kept for its design notes.

## Coordinate frames

All are right handed, Y up, −Z forward, metres.

| Frame | Origin | Kept across sessions |
|---|---|---|
| **raw** | The headset SLAM origin, which is where the head was at power on, raised 1.6 m (`PSVR2_FLOOR_HEIGHT`). It is level: SLAM "up" matches gravity to 0.8° (median, 16:36 run). | No. It moves every session and on every SLAM reset. |
| **room** | Fixed to the physical room. The floor is y = 0. SteamVR universe 2 is drawn in this frame. | Yes, through the saved anchor. |
| **SteamVR standing** | The room frame, plus the `standing` pose that SteamVR's Room Setup saves in universe 2. | Yes, in SteamVR's own file. |

- The headset and both controllers are reported in the raw frame.
- The controllers are placed through the head pose at each camera frame, and the constellation tracking origin copies the headset's offset. So all three devices share one frame.

**`T_room_raw`** is the room transform:
- `p_room = R·p_raw + t`, where R is a rotation about Y only.
- It is sent as `qWorldFromDriverRotation` / `vecWorldFromDriverTranslation`, the same value for the headset and both controllers. It is applied once, and it never changes the poses themselves.
- This is the mechanism OpenVR-SpaceCalibrator uses (`tracked_device_provider.cpp`).

## Universes

- **3: this session only.**
  - Standing is identity, so the floor is the 1.6 m guess.
  - There is a 2 m square around the start position, or nothing with `ROOM_SETUP=1`.
  - The driver rewrites this entry on every start.
- **2: the saved room.**
  - Used only after a capture has anchored the room in this session.
  - The first time, the driver seeds a plain entry for it. Room Setup's Accept then replaces that with the drawn room.
- **SteamVR's room file:** `~/.local/share/Steam/config/chaperone_info.vrchap`.
  - The driver finds it through `openvrpaths.vrpath`.
  - It writes atomically and never over a concurrent SteamVR write.
  - It backs up the original file once, as `.monado-backup`.
  - Other universes are never touched (Sony's driver keeps its own file anyway).
- **Why not the old driver-provided JSON:** with `Prop_DriverProvidedChaperoneJson_String` set, SteamVR doesn't save rooms drawn in Room Setup. This was found by reverse-engineering vrserver (Quest1-SteamOS `docs/boundary.md`), not from Valve's docs.
- **Where this design comes from:** the universe 2 / 3 split and the seeding follow lbgos/rift-s-linux (`ovrd_chaperone_origin.cpp`, `ovrd_world_universe.hpp`).

## Floor capture

The left Sense controller, with its Create button held for 2.5 s while the controller rests still.

1. **Put the controller on the floor.** For the room anchor, use a spot you can find again, for example a tape mark, and put it down the same way round each time.
2. **Crouch next to it.** In the recorded runs, controllers were never tracked more than 0.69 m from the head (95% of samples within 0.55 m). A controller on the floor while you stand is about 1.5 m away.
3. **Hold Create.**
   - A short buzz (0.25 s) means it worked.
   - A long buzz (0.9 s) means it was rejected; the reason is in the log as `Room: capture rejected: ...`.

What a capture measures:
- **Floor:** the lowest point of the controller. That's the lowest of its 17 LEDs (from the driver's LED model) minus the LED radius, averaged over the hold.
- **Anchor:** the grip position (x, z) and a heading.
  - The heading comes from the grip axis lying most level at the first capture, saved as `axis`. A controller put down rests on its ring with its forward axis pointing up (every recorded rest: world up ≈ (0, 0.27, −0.96) in the grip frame).
  - Later captures use the same axis.

What a capture does:
- **First capture:** defines the room frame. The floor goes to y = 0; nothing turns or shifts.
- **Later captures:** match the saved anchor, which puts the room back in place. Saved in `~/.config/monado/ovrd_room.json`.

Rejections:
- The controller moved more than 1 cm, or turned faster than 0.2 rad/s.
- The controller wasn't position tracked.
- The floor is less than 0.5 m below the head.
- The headset's tracking re-originated within the last 5 s.
- At a later capture, the saved heading axis is within 30° of vertical.

## Tracking restarts and jumps (old text: the room used to be dropped; since 2026-11-18 it is kept and logged)

- The headset's SLAM re-originates and the driver can't carry the position over. A reset is a jump of at least 5 cm landing within 1 cm of the origin. After the first 20 s, and at least 1 s after the previous reset, the driver carries the position across it (`psvr2_slam_carry_push`) and the room stays. Resets during startup or in a reset loop replace the tracking world; the driver counts those in `psvr2_get_slam_reset_count`, and the room is dropped.
- The head moves more than 0.3 m within 50 ms (tracking runaway).

Commands: `echo forget > $XDG_RUNTIME_DIR/monado-room-command` deletes the saved anchor. `anchor-test` (debug only) switches to universe 2 without a capture.

## Launcher options

| Variable | Effect |
|---|---|
| `ROOM_SETUP=1` | No play area in universe 3 (SteamVR's Room Setup starts on its own) |
| `ROOM_FORGET=1` | Delete the saved anchor before starting |
| `RECORD_IMAGES=1` | Save both bottom camera images every 2 s (Stage 3: can they drive a relocalizer?) |
| `OVRD_ROOM=0` | Previous behaviour: a fixed 2 m area in universe 2, nothing saved |

## Checked offline

- **Unit tests (`tests/tests_ovrn_room.cpp`, 9 cases, 3,563 checks):**
  - Transform composition and inversion: `(a·b)(p) = a(b(p))`, `inverse(a)(a(p)) = p`, and heights are kept.
  - First-capture frame.
  - Recovery in a new session: for 100 random session offsets (yaw and shift), every point lands where it did in session 1.
  - Placement-error sensitivity: 1° + 1 cm moves a corner 2 m away by under 5 cm.
  - Heading of a resting controller.
  - The room file:
    - created when missing, and an unchanged second run changes nothing;
    - a drawn room and another driver's universe are kept, and universe 3 is reset;
    - a file that isn't a room file is left alone.
  - Anchor file round trip and validation.
- **Floor measurement on recorded runs** (`analysis/rest_floor.py`, plus `analysis/harness/outline.c` for the driver's LED model):
  - Within each rest the measurement is steady: lowest-point spread 0.6–2.6 mm, heading spread 0.1–0.8°.
  - The heading was defined in all 29 rests.
  - Not checked: accuracy against a known floor. No recorded rest was on a surface of known height.
- **The real capture code on recorded runs** (`analysis/harness/capture_replay.cpp`, which links `ovrn_room.cpp`, with Create held the whole time):
  - Three captures from one placement agreed to 4 mm in position, 7 mm in floor height and 1.1° in heading.
  - Rests 0.37–0.49 m below the head (a lap or a desk) were rejected.
  - Some rests 0.53–0.56 m below the head passed. A seated lap and a crouched floor overlap, so the gesture has to be deliberate.
- **SLAM resets in 78 recordings** (`analysis/slam_rebase_sim.py`):
  - Resets are position-only and come in bursts during the first ~15 s. In today's builds there were none after 15 s, except one at 15.8 s.
  - Carrying the position on across a reset didn't reduce any run's largest jump after startup and left offsets of up to 12.7 cm, so it isn't done.
  - The 0.3 m / 50 ms rule never fired during play in 25 runs. It fired only at startup and when the headset was taken off.
- **SteamVR with Monado's simulated headset** (SteamVR 2.18.4, `vrprobe` in the scratchpad):
  - SteamVR read the prepared file: "Found universe 3 in chaperone file".
  - The calibration state is 1 (OK). It was 201 ("base stations uninitialized") in a control run with the old room-setup mode.
  - `GetControllerStateWithPose` succeeds for every device.
  - The runtime switch to universe 2 works ("Found universe 2"); the state there is 203 (no play area yet).
  - Room Setup can't run in simulation, because there's no display for SteamVR's compositor.

## Not known yet (needs the headset)

1. Does Room Setup now show the headset and controllers as Ready, and can you draw a room in universe 2?
   - The control run showed that the call room setup uses succeeds even in the old mode, so "Off" had another cause.
   - The only measured difference is the calibration state (201 before, 1/203 now).
2. Does the controller stay tracked on the floor when you crouch next to it?
3. Is the floor accurate? Rest a controller on the floor, capture, and compare SteamVR's floor with the real one.
4. Is the room still in place after restarting SteamVR, then the headset? Put the controller back on its mark and capture again.
5. Stage 3: run with `RECORD_IMAGES=1` and check whether the images are exposed for the room or dark like the LED frames.

## Not done

- **Automatic room recognition (Stage 3).**
  - The headset streams camera mode 10 images from the bottom pair (2 × 1000×1000, BC4) during every run, alongside the LED tracking. The driver discards them unless a debug view is open.
  - A relocalizer like Rift S Linux's (`rift_s_world_anchor.cpp`: ORB features, PnP solve, about 1,800 lines) would need those images to show room detail.
  - `RECORD_IMAGES=1` collects the evidence first.
- **Room Setup's floor step and passthrough.** Rift S Linux drives these through undocumented SteamVR 2.17 internals (properties 11003/11004, event 809, `IVRMailbox`). They're left out, because a SteamVR update can break them.
