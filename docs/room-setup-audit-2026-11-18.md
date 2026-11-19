# SteamVR Room Setup, Chaperone and playspace: audit and plan (2026-11-18)

Phase 1 of the room setup task: what the driver does today, what SteamVR needs (official vs driver practice), what is
missing, and the plan. No code was changed for this audit.

Sources: our code (`trial/src/xrt/state_trackers/steamvr_psvr2/`, `steamvr_drv/ovrd_room*`, the input resources in
`trial/src/xrt/targets/steamvr_drv/resources/input/`), SteamVR's logs of the 15:43 Room Setup attempt
(`~/.local/share/Steam/logs/vrclient_steamvr_room_setup.txt`, `psvr2-test/logs/steamvr-20261118-154331-vrserver.txt`),
SteamVR's and Sony's chaperone files, 26 headset recordings, the OpenVR headers and driver documentation
(`openvr/docs/Driver_API_Documentation.md`, "DOC"; `openvr_driver.h`, "DH"; ours is byte-identical to upstream,
SteamVR 2.15.6 headers), ALVR, WiVRn, OpenVR-SpaceCalibrator, PSVR2Toolkit, and `strings` of SteamVR's binaries
(marked BINARY: undocumented inference).

## 1. What our driver does today

| Area | Today | Status |
|---|---|---|
| Driver | `IServerTrackedDeviceProvider` (`ovrn_provider.cpp`), devices via `IVRServerDriverHost::TrackedDeviceAdded`, poses via `TrackedDevicePoseUpdated` from a 1 kHz pose thread, properties via `IVRProperties` | OK |
| Device classes | HMD `TrackedDeviceClass_HMD`, controllers `TrackedDeviceClass_Controller` (Prop_DeviceClass is inferred from that, DOC:2352) | OK |
| Roles | `Prop_ControllerRoleHint_Int32` LeftHand / RightHand (`ovrn_controller.hpp:92`) | OK |
| Identity | `Prop_TrackingSystemName_String` "monado_psvr2", manufacturer, model, serial, controller type, input profile on every device (`ovrn_device.hpp:43-68`) | OK |
| Tracking style | `Prop_HmdTrackingStyle_Int32` = 3 (InsideOutCameras) only inside the optional Sony property group (`OVRN_SONY_PROPS=display`), so normally **not set** | Missing (documented effect: UI text only, DH:735) |
| One tracking space | Headset and controllers share the headset's frame: the controllers are placed through the head pose at each camera frame and the constellation origin copies the headset's offset. One room transform goes into every device's `qWorldFromDriver` (`setWorld`, same value for all) | OK by design |
| Pose validity | HMD: valid / Running_OK, Running_OutOfRange when orientation-only; controllers: Running_OK or Fallback_RotationOnly when position is lost; Uninitialized when no data | OK |
| Universes | Universe 3 = this session (reset to a plain frame + square every start); universe 2 = the saved room, active only after the floor-capture gesture (`OvrdRoom`, `ovrd_room_state.hpp`) | See problems |
| Chaperone file | The driver edits SteamVR's global `~/.local/share/Steam/config/chaperone_info.vrchap` atomically before SteamVR reads it (`ovrd_room_prepare_chaperone`) | Works, but see problems |
| Driver-provided chaperone | `Prop_DriverProvidedChaperoneJson_String` only with the room code off (`OVRD_ROOM=0`) | Not used normally |
| Floor | 1.6 m below the head at the start (`PSVR2_FLOOR_HEIGHT`) until a floor capture | Guess until captured |
| Floor capture / anchor | Left controller resting on the floor (on a mark), Create held 2.5 s: measures the floor from the controller's lowest LED and anchors the room's position and heading; saved in `~/.config/monado/ovrd_room.json` (versioned) | Works offline (replays); never confirmed in the headset; undiscoverable |
| Relocalization | SLAM resets: after the first 20 s the headset driver carries the position across them (`psvr2_slam_carry_push`); resets in the first 20 s replace the world, and the room code then **drops the room** (back to universe 3) | Problem |
| Room Setup input | Room Setup is a legacy-input app; SteamVR gives it our `legacy_binding_playstation_vr2_sense_controller.json` | **Broken** (below) |

## 2. What SteamVR needs (official vs practice)

**Official (DOC / DH):**
- `Prop_CurrentUniverseId_Uint64` on the HMD selects the one active chaperone universe (DOC:3528); not 0 or 1.
- Chaperone JSON / file: `Prop_DriverProvidedChaperonePath_String` (a file the driver owns) or
  `Prop_DriverProvidedChaperoneJson_String` (higher priority), on the HMD (DOC:3523).
- Per universe: `collision_bounds` (quads of 4 vertices on a vertical plane, relative to the setup-standing origin),
  `play_area` [width, depth], `standing` and `seated` {translation, yaw} relating raw tracking space to each origin,
  optional `setup_standing2` (defaults to standing). Standing y = 0 must be the floor (DOC:3518, 3532-3561).
- The chaperone lives in raw tracking space; `qWorldFromDriver` converts driver space to raw space (DH:2905-2928).
  So if the driver keeps its *raw* space fixed to the room, a saved chaperone stays in place.
- `IVRChaperoneSetup` is a client interface only (openvr.h); a server driver has no chaperone API besides the
  properties above.
- There is no public API for a driver to report relocalization (BINARY: SteamVR has a private one for its own
  drivers, `IVRChaperoneServerDriver`).

**Real-world details the docs get wrong (from SteamVR's actual files and binaries):** the key is `"jsonid"` (not
`json_id`), `universeID` is a string, `time` is ctime text; quads are bottom, top, top, bottom; walls 2.43 m.

**Driver practice:**
- ALVR: universe fixed at 2, `qWorldFromDriver` identity, recentres poses itself, and overwrites SteamVR's chaperone
  with a rectangle from the headset's Guardian through the client API (`VR_Init(VRApplication_Background)` on a
  separate thread inside the driver, `ChaperoneUpdater.cpp`). It does not keep SteamVR Room Setup rooms.
- SpaceCalibrator: moves devices between tracking systems by premultiplying an offset into `qWorldFromDriver` (the
  same mechanism our room transform uses), and re-applies saved bounds when SteamVR resets them.
- WiVRn: no SteamVR driver (an OpenXR runtime).
- Sony's driver: its own chaperone file (`Prop_DriverProvidedChaperonePath_String` =
  `~/.local/share/Steam/config/playstation_vr2/chaperone_info.vrchap`), universe 4, a room drawn in Room Setup
  stored there (7 walls, 1.84 x 1.91 m, Oct 29), with a standing pose at map coordinates (-33.8, 6.5, 25.6) m. It
  persists because Sony's driver **relocalizes** the headset against a saved map on the PC
  (`sceMapDb.bin`, `sceBoundaryMeta.bin`; PSVR2Toolkit hooks `RelocalizerThread::RunLocalizer`, fed by camera
  images). SteamVR's Room Setup wrote into that driver-owned file.
- Room Setup itself (BINARY): a Unity app on the legacy API: `GetControllerStateWithPose`,
  `GetTrackedDeviceIndexForControllerRole`, the legacy trigger (Axis1), `bPoseIsValid`; it commits through
  `IVRChaperoneSetup` (`SetWorkingCollisionBoundsInfo`, `SetWorkingPlayAreaSize`,
  `SetWorkingStandingZeroPoseToRawTrackingPose`, `CommitWorkingCopy`). It has a floor "Calibrate" step and a manual
  floor-height field. Commits can be refused: "no universe was set", "tracking origin is too far from 0".

## 3. Problems found

1. **Room Setup can't trace (root cause found).** Our legacy binding maps buttons, trigger and stick exactly like
   Sony's but has **no pose** (`/user/hand/*/pose/raw` -> `/actions/legacy/in/Left_Pose` / `Right_Pose`) and no
   haptics. Room Setup reads the trigger but gets no controller pose, so nothing traces. The old plugin uses the
   same file, which fits Room Setup never having worked with either.
2. **The tracking frame isn't repeatable across sessions.** All 26 recordings today start the headset's SLAM at
   exactly (0, 0, 0): the headset starts a new world every time our driver connects (a SteamVR restart included),
   unlike Sony's relocalized map. A room saved in raw space is therefore only valid again after the floor-capture
   anchor (the existing universe 2 design) or a relocalizer.
3. **A room drawn in the session universe (3) is wiped at the next start**, by design, and nothing tells the user to
   do the floor capture first. The current file even has a plain 2 m square stored in universe 2 that the driver
   now treats as "a drawn room".
4. **Relocalization drops the room.** A SLAM reset that isn't carried (all resets in the first 20 s, reset loops)
   switches back to universe 3, so the boundary jumps or disappears, the opposite of what's wanted.
5. **Inside-out tracking style isn't reported** (`Prop_HmdTrackingStyle_Int32` = 3), though Sony's driver reports
   it. Documented effect: UI wording only.
6. **We edit SteamVR's global chaperone file**, which other drivers share. A driver-owned file
   (`Prop_DriverProvidedChaperonePath_String`, as Sony's driver does) keeps our rooms separate, and Room Setup writes
   into it.
7. Not wrong but missing: structured diagnostics of the room state, and tests for boundary geometry.

## 4. Plan

**Phase A (small, likely to make Room Setup usable immediately):**
1. Legacy binding: add the controller poses and haptics, as Sony's file has them (shared resource; both plugins).
2. HMD: `Prop_HmdTrackingStyle_Int32` = InsideOutCameras by default.
3. Validation: Room Setup traces with our controllers; SteamVR commits the room (`VREvent_ChaperoneRoomSetupCommitted`
   / the file).

**Phase B (the playspace model, the bigger change):**
1. A driver-owned chaperone file (`~/.local/share/Steam/config/monado_psvr2/chaperone_info.vrchap`) through
   `Prop_DriverProvidedChaperonePath_String`, instead of editing SteamVR's global file. Keep our own universe ids.
2. One explicit transform chain, raw SLAM -> driver (carried across resets) -> room (`qWorldFromDriver`), documented
   and logged; standing/seated/setup-standing are SteamVR's, stored in our file.
3. Persistence across sessions = re-anchoring the room transform. Today: the controller-on-a-mark floor capture.
   Make it the guided path (log/notification, and the room stays in the saved universe while waiting, with the
   boundary hidden until anchored rather than wiped).
4. Relocalization: never drop the room on a reset; compensate the jump in the driver->room transform (the SLAM
   carry, extended to the first 20 s; this touches the headset driver, which also changes OpenXR mode).
5. Diagnostics (room state, universe, transform, floor, bounds, loads/saves, resets) and tests (transform chain,
   boundary geometry: winding, closure, no duplicates, floor plane, relation to setup-standing).

**Phase C (optional, large): automatic room recognition.** A camera relocalizer like Sony's (or rift-s-linux's,
about 1,800 lines) using the headset's bottom cameras, so the room comes back without the floor capture. This is
what makes "restarts don't destroy calibration" automatic; without it, a SteamVR restart needs one floor capture.

## 5. Manual validation (after Phase A / B)

1. Start SteamVR with the driver. If no room is anchored: put the left controller on its floor mark, crouch, hold
   Create 2.5 s (short buzz = OK).
2. SteamVR > Room Setup > Room-scale: the controllers show as ready; calibrate the floor; trace the room holding the
   trigger; accept.
3. Start a game: the boundary appears when walking towards a wall; it stays in place while turning, walking,
   covering a controller, and after the headset's tracking restarts.
4. Restart SteamVR: do the floor capture on the same mark; the boundary is back where it was.
