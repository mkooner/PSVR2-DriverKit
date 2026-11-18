# PS VR2 controller LED sync — research

Status: research complete, 2026-11-16. Sources cloned and read; citations are to files and functions at the commits
listed. "Unverified" marks anything that depends on closed binaries or that we could not check.

## Sources

| # | Source | Kind | Commit / version |
|---|---|---|---|
| 1 | [PSVR2Toolkit](https://github.com/BnuuySolutions/PSVR2Toolkit) | Implementation (hooks Sony's closed SteamVR driver) | main `a7e799a`; v1.0.0-experimental-1 `9e24e6e`, -2 `b89b78a`, -3 `9900be1`; v0.2.1 `f3008f2` |
| 2 | RealSupremium custom-controller-sync = PSVR2Toolkit PR #24 | Implementation | closed unmerged, head `5a0c780`; re-landed as `5a8688b`; the guide's Actions run 18515528451 is gone (404), build probably from `bd2d4ca` (unverified) |
| 3 | [PSVR2-PC-Tracking-Guide](https://github.com/BlueberryWolf/PSVR2-PC-Tracking-Guide) | Community guide, no code | — |
| 4 | [PSVR2-Documentation](https://github.com/ctrl-shift-win-b/PSVR2-Documentation) | Protocol documentation, claims tagged [V] verified / [U] unconfirmed / [?] unknown | HEAD `1619530`; initial `eb17809` had code references |
| 5 | [psmoveapi](https://github.com/thp/psmoveapi) | Implementation | `256b629` |
| 6 | [PSMoveService](https://github.com/psmoveservice/PSMoveService) | Implementation (archived) | `22bbe20` |
| 7 | [PSMoveSteamVRBridge](https://github.com/HipsterSloth/PSMoveSteamVRBridge) | Implementation | `529c716` |
| 8 | [RiftDK2](https://github.com/facebookarchive/RiftDK2) + [Meta blog](https://developers.meta.com/vr/blog/open-source-release-of-rift-dk2/) | Hardware + firmware release | `42363e7`; `facebookarchive/oculus-ift-1` does not exist |
| — | Steam Frame + Sense driver (Reddit, PC Guide) | Community report only, no code found | — |

No other public PS VR2 controller tracking implementation was found.

## 1. PSVR2Toolkit (`projects/psvr2_openvr_driver_ex/`)

**LED latency calibration** — `driver_hooks/libpad_hooks.cpp`, `libpad_SetSyncLedBaseTimeHook` (hooks Sony's
`libpad_SetSyncLedBaseTime`, RVA 0x1C27D0). Per controller (`ControllerContext controllerCtx[2]`), both at once:
- 300 ms of `LED_ALL_OFF`; threshold = LED count + 3.
- Then forced `PRESCAN`, period 32; `fullWindow = oneSubGridTime*period + camExposure`; a step only every 4 tracker
  frames (`g_opticalFrameIndex - lastSyncFrame >= 4`).
- Sweep by `fullWindow` until on; binary search left edge, then right edge in `[left+0.9fw, left+1.1fw]`, to 25 us;
  confirm at +-50 us; any failure `resetCalibration()`.
- `latencyOffset = left + (right-left)/2`, then Sony's normal PRESCAN/BROAD/BG/STABLE flow continues, with periods
  rewritten 32/32/20/9 and STABLE/BG offsets `oneSubGridTime*(20-9)` (`libpad_SetSyncLedCommandHook`).
- **Visibility is a per-controller count of LEDs matched by Sony's tracker**, `IrTrackerHooks::g_controllerLedCount`,
  from `IrTracker__AssociateLEDsHook` (`ir_tracker_hooks.cpp`), since `be18951` "Use solved controller LED data for
  simultaneous LED sync". The original custom-controller-sync (`bd2d4ca`) used the **total blob count of cameras 0+1**
  from the LD payload, baseline + 6, one controller at a time (the other forced off), a step every 50 ms.
- Reset: disconnect, confirm failure, or (only ~0.5 s after calibration) 500 ms untracked.

**Clock** — `sense_controller.h` `AddTimestampOffsetSample`, `libpad_hostToDeviceHook`/`libpad_deviceToHostHook`.
Device time in 1/3 us, wraps at 2^32/3 us (~23.9 min, `GetWraparoundDifference`). Sample = device/3 - host receive;
the offset tracks the **maximum** sample (minimum latency). exp-1/2: running max with linear decay and a
"floor hit" rate controller, +-2.5 us/sample slew. **exp-3 `09f14e7` "Simplify timestamp offset logic"** (latency was
being overestimated): offset lerps toward an EMA of the samples at 0.025/s, still snaps up to larger samples, output
slew 30 us/s, reset if 10 ms off. From the code it pulls toward the *mean*, so with heavy jitter it can sag toward mean
latency; later `29eea15` removes the slew. `latencyOffset` (from the calibration) is added to host->device.

**Camera** — `usb_thread_hooks.cpp` `CaesarUsbThreadImage__pollHook`: image host time = header timestamp +
`CaesarManager::getIMUTimestampOffset()` (the headset IMU clock offset), the same pattern our driver uses for LD.

**Camera to head** — `driver_hooks/hmd_device_hooks.cpp`: `Prop_CameraToHeadTransform` for cameras 0/1 =
`Rx(-15) * R` with R read from calibration offsets 0x6f0 / 0x7dc, translation from Sony's internal
`ConfigManager::getCameraConfig(i)->position` (not in the calibration block).

## 2. PSVR2-Documentation (protocol)

- Sense BT input 0x31 @29 IMU timestamp (3 MHz, +-15 ppm), @45 echo of the host timestamp, @49 device timestamp;
  400-530 Hz.
- Output 0x31: @17 host timestamp (us, echoed at input @45), @21 LED phase, @22 sequence number (latch), @23
  period_id (x 50 us emission), @24 cycle_position, @28 cycle_length (1/3 ns; 0x02FBB402 = 16683.35 us).
  **Latch rule [V]:** position/period/LEDs only apply when the sequence number changes; re-latching every frame
  suppresses emission; >= 250 ms between latches is recommended. The doc also says Monado's settings block is one byte
  off on its device (no tag byte) — **not consistent with our hardware, where the LED commands take effect; open.**
- Clock: one-way min-delay envelope is biased by the minimum latency (synthetic: -6003 us); **two-way echo
  (host timestamp out @17, echoed @45) with the minimum round trip per 1 s window gives the offset within +-RTT/2,
  -25 us on the synthetic link**; RTT p50 15.5 ms. Rate from per-window landmarks by least squares: 0.35 ppm.
- Headset LD stream: VTS timestamp, frame period 16682.8 us, no exposure field. Cameras 0/1 are the **bottom pair,
  30.6 deg apart in yaw**. Camera relation conventions: not documented.

## 3. Independent implementations

- **psmoveapi** `psmove_tracker_blinking_calibration` / `psmove_tracker_get_diff`: LEDs on/off difference images
  (unsynchronised, 50 ms sleeps), multi-blink consistency, LED dimming; fixed manual exposure. Transfers: on/off
  differencing and consistency checks. Specific: HSV colour.
- **PSMoveService** `computeSpherePoseForControllerFromMultipleTrackers`: unweighted mean of per-camera estimates with
  **no disagreement rejection**; optical packets stamped with processing time; variance from blob area. Transfers:
  area-based variance, prediction ROIs, per-camera timeouts. Anti-patterns: processing-time stamps, ungated averaging.
- **PSMoveSteamVRBridge** `UpdateTrackingState`: keeps `poseIsValid`/`Running_OK` after optical loss, drops
  velocities. Cautionary.
- **Rift DK2** `Headset/Firmware/src/sequence.c`: hardware exposure sync (cable or 85 kHz IR), 10-frame 1-bit LED IDs,
  one device clock for IMU, vsync and exposure (`timestamper.c`), LED keep-alive shutoff. Transfers: single timebase,
  per-exposure records, keep-alive. Specific: the sync hardware.

## Recommendation matrix

| Technique | Source | Relevance | Adopt? |
|---|---|---|---|
| Optical edge-search latency calibration (off baseline, sweep, 25 us edges, confirm, centre) | PSVR2Toolkit | PS VR2 itself | **Yes** — already adopted (count calibration) |
| Per-controller matched-LED visibility instead of total blob count | PSVR2Toolkit main `be18951` | PS VR2, solves multi-controller confusion | **Yes**, as a complement to on/off differencing |
| Hand back to BROAD/BG/STABLE after calibration | PSVR2Toolkit | PS VR2 | Later, once the hold is proven |
| Latch at most ~1-4 Hz, never every frame | PSVR2-Documentation [V] | PS VR2 | **Yes** — already 500 ms |
| Two-way echo clock, min-RTT per window | PSVR2-Documentation [V] | Fixes the one-way bias in IMU timestamps | **Yes** (IMU/optical alignment) |
| Landmark least-squares rate | PSVR2-Documentation | Rate to 0.35 ppm | Yes |
| On/off difference counting | psmoveapi | Ambient rejection | Already adopted |
| Gate / weight per-camera estimates | PSMoveService (as missing) | Multi-camera | Yes, after the extrinsics fix |
| Explicit not-tracking state to the runtime | PSMove bridge (as anti-pattern) | Output | Yes |
| Hardware sync | DK2 | Not available on PS VR2 | No |
