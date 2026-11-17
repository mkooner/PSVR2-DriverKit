# Room setup and playspace for the PS VR2 (SteamVR driver)

What the driver does with SteamVR's room (chaperone), why, and what has been checked where.

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

## The room is dropped (back to universe 3) when

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

- **Unit tests (`tests/tests_ovrd_room.cpp`, 9 cases, 3,563 checks):**
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
- **The real capture code on recorded runs** (`analysis/harness/capture_replay.cpp`, which links `ovrd_room.cpp`, with Create held the whole time):
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
