# SteamVR ghosting / duplicated images — research report (2026-11-18)

Symptom (user): duplicated / ghosted images through SteamVR, also with the headset still; the same game on Monado's
OpenXR runtime is smooth. Setup: PS VR2 over DisplayPort, NVIDIA RTX 5080, Arch Linux, SteamVR 2.18.4, 90 Hz, our
plugin (Monado steamvr_drv) with IVRDisplayComponent (SteamVR owns the display, direct display via DRM lease).

Sources read: ALVR (`master` 2945d90e for the Linux direct mode, removed from `main` in PR #3450; issue #2034, PR #2481,
PRs #3371/#3398/#3414/#3425, issue #3412), ASVR (`main` 486fbed), ValveSoftware/openvr (0924064, docs, wiki, issues
#175/#358/#359/#415/#722/#1128/#1517), SteamVR-for-Linux issues (#917, #938, #1011, #1040, #1046, #1058, #952), and the
game's source and logs (ZeldaWWHD-VR runtime/src/vr, runtime/src/gfx/vulkan, captures/wwhd.log).

## 1. What is established by measurement (before this research)

| Item | Result |
|---|---|
| Our poses to SteamVR | ~950 Hz, smooth, gaps < 3.4 ms; SteamVR's prediction from them exact (vrposeprobe) |
| Plugin per-frame cost | RunFrame < 0.25 ms |
| SteamVR compositor counters (normal mode) | repeats 0.08-0.35 %, drops 0.6-0.8 %, no throttling / motion smoothing |
| Game in steady play on SteamVR (wwhd.log) | 89.1-89.6 fps capped 90, 0 frames shown again, SteamVR frame smoothing 3/897 frames |
| Head-motion error of shown frames, head still | 0.013 deg / 0.03 mm (direct mode measurement) |

So with a still head, pose, prediction and frame pacing do not produce visible error. The duplication must come from
image content or image handling.

## 2. Findings per source

### ALVR
- Issue #2034 / PR #2481 (pose-to-frame matching): SteamVR's returned pose didn't always match the client's saved
  pose, so frames were shown with the wrong rotation. Fixed by sending the frame's own view parameters with it. With a
  still head every candidate pose is nearly identical: **cannot cause duplication with a still head.** Our measurement
  also shows SteamVR labels the game's frame with exactly its render pose. Not applicable.
- Linux direct mode (master only): no fence on NVIDIA (dma-bufs carry no implicit fences; vrcompositor's own writes
  carry "pre-signaled placeholder fences"), so reads can mix two frames, often one eye. Fix in PR #3425 (closed, MIT
  fork): reverse-engineered per-image timeline semaphore from SteamVR's shared memory, and not holding vrcompositor in
  PostPresent (a full-frame hold made vrcompositor's WaitForAcquire time out and render into the image being read: a
  split, half-frozen image). **Applies to our direct-mode prototype only**, not to the normal SteamVR path.
- Vulkan layer (main): invents a display for vrcompositor; irrelevant for a real DisplayPort headset.

### ASVR
- Virtual headset; copies each frame synchronously (PostPresent waits for its copy fence) so SteamVR can never reuse
  an image still being read. Implicit-sync poll on the fd, RADV-only (no NVIDIA testing). Not applicable to the normal
  path; reference for making our direct mode safe.

### Valve OpenVR / SteamVR-for-Linux
- VsyncEvent offset: "the time in seconds until the upcoming vsync" (wiki Driver-direct-mode.md), positive = future.
  Our direct-mode vsync alignment assumed the opposite sign.
- Direct-mode component: no documented Vulkan sync contract (openvr #1517 open); Valve: "never really intended to be
  implemented outside of wrapping the Oculus SDK" (#415). With a direct-mode driver SteamVR's async reprojection is off.
- **SteamVR 2.18.x Linux compositor bugs** (Valve-acknowledged, fixed in 2.19.2/2.19.3 betas): right-eye ghosting
  (#938), stale per-eye images up to 3 frames old (#1046 user analysis), judder + double images after menus (#1058),
  right-eye duplicated image (#1011, open). Reported with vrlink / Steam Frame, not wired headsets.
- #1040: PS VR2 (Ignition/PSVR2Toolkit) on NVIDIA, Wayland, 2.18.4: async reprojection crashes the compositor.
- #917: with async off, missed frames give "severe double images while turning"; wrong Prop_DisplayFrequency gives
  head-swim.

### The game (ZeldaWWHD-VR)
- No Monado-specific path; the SteamVR name check only affects eye crop (off by default). Headset reported as
  "monado" → measured as not streamed, so no runtime-dependent pacing difference in steady play.
- Repeats frames by design only below the refresh rate (45/30 fps) or when a frame is late (keep-alive): 0 in steady
  play at 90 fps, a few during loading.
- **Possible ordering race (source reading, unconfirmed):** with the record worker (default on, Linux desktop), the
  eye-copy command buffer can be submitted by the worker after the end-frame thread has already released the swapchain
  image and called xrEndFrame. The runtime would then show the image last written ~3 frames earlier; moving content
  jumps back and forth: duplication also with a still head. `WWHD_VK_RECORD_WORKER=0` rules it out.
  Checked in backend.cpp: the normal path records the eyes inside the desktop window's present (3521), where the
  submit is synchronous (`producer_presentation_recording` → `execute()`); the race is only on the fallback path when
  the desktop window had no image ready (4416-4419: `record_eyes` + `flush_async`, which queues to the worker via
  `submit_record_commands` without waiting before `frame_end`). How often that path runs is not logged.

## 3. Feature comparison (what is worth porting)

| Feature | Source | Problem it solves | Applies to our normal path? | Recommendation |
|---|---|---|---|---|
| Frame carries its own view params | ALVR PR #2481 | wrong pose for a frame (streaming) | No (SteamVR owns display; pose already exact) | Reject |
| Per-image timeline-semaphore wait | ALVR PR #3425 `SteamVrSync.h` | reading SteamVR images before written (NVIDIA) | Direct mode only | Adapt if direct mode is pursued |
| Hold only vrcompositor-owned sets in PostPresent | ALVR PR #3425 | WaitForAcquire timeouts, split images | Direct mode only | Adapt if direct mode is pursued |
| Synchronous copy before PostPresent returns | ASVR CDirectModeOutput | image reuse while read | Direct mode only | Reference |
| GetFrameTiming clears reprojection flags | ALVR, ASVR, header | false throttling | Direct mode only (done) | Done |
| VsyncEvent = time until next vsync | Valve wiki | prediction timing | Direct mode only | Fix sign (remove our alignment) |
| Vulkan layer / DRM lease shim | ALVR main | no real display | No | Reject |

## 4. Where duplication can enter, ranked by evidence

1. **SteamVR 2.18.4's own Linux compositor bugs** — Valve acknowledged stale per-eye images / double images in 2.18.x,
   fixed in 2.19.2/2.19.3 betas. Matches "duplicated, also still" if the content moves. Not confirmed for a wired
   headset. **Test: SteamVR beta (no code change).**
2. **Game's swapchain release racing its own eye copy** — found by source reading only. Would show on any runtime
   whose compositor reads the image promptly after release; timing differs between SteamVR and Monado. **Test:
   `WWHD_VK_RECORD_WORKER=0` on normal SteamVR (no code change).**
3. **SteamVR reprojection switching** — present during loading, rare in steady play (3/897). Weak.
4. Our direct-mode prototype has known unsafe image reads on NVIDIA (no fence, PostPresent hold): its ghosting
   result can't be used to rule anything in or out. It needs the ALVR #3425 fixes before it is a valid comparison.

Not supported by the evidence: head tracking, pose prediction, pose-to-frame matching, frame pacing in steady play.

## 5. Recommended next experiments (no code changes)

1. Normal SteamVR (no DIRECT_MODE), game with `WWHD_VK_RECORD_WORKER=0`: if the duplication is gone → the game's
   race (fix in the game: order the release after the worker's submit).
2. Normal SteamVR on the current SteamVR beta (2.19.3+), record worker default: if gone → SteamVR 2.18.4's bug.
3. Open question for the user: was Sony's driver (PSVR2Toolkit/Ignition) smooth on this same Linux machine and
   SteamVR 2.18.4? If yes, (2) is less likely.

Clean-up pending approval: remove the direct-mode vsync alignment (wrong sign assumption, no visible effect).

## 6. Update after the user's answer (Sony's driver has no duplication, same machine, SteamVR 2.18.4)

- SteamVR 2.18.4's own bugs and the game's release race become unlikely (the same SteamVR and game would show them with
  Sony's driver too). The difference must be in what our driver gives SteamVR.
- Sony's driver (Ignition, `SteamVR_Plug-In/bin/linux64/driver_playstation_vr2.so` + win64 dll) ships no
  default.vrsettings.
- Compositor log, Sony run (2026-11-17 14:53, PID 95230) vs ours (2026-11-18 08:07), everything else identical
  (async disabled "by user setting or a direct mode driver" / auto-disabled on this Vulkan environment, direct mode
  enabled, FIFO_LATEST_READY, mode 6, 89.9 Hz, recommended 3400x3468):
  - hidden area 22.3 % (Sony) vs 0 % (ours)
  - raw ideal render target scale 2.35 (Sony) vs 1.82 (ours): the lens distortion SteamVR derives from the driver
    differs.
- Our ComputeDistortion is Monado's own distortion function with the same UV conventions as Monado's compositor mesh
  (u_distortion_mesh.c run_func: (0,0) top left, identity view rotation for PS VR2), so SteamVR gets the same lens
  correction as the smooth Monado path: the 1.82 vs 2.35 difference is Monado's lens model vs Sony's, present on the
  smooth path too.
- Only one Sony-driver session is in the logs and it ran SteamVR Home only; there is no record of the game on Sony's
  driver.

## 7. Head-motion-only symptom (user, after updating to the SteamVR beta: still there, direct and normal mode;
##    only when moving the head, not with controllers or forward/back locomotion)

Evidence gathered (no new session; probe logs steamvr-probe-*.csv from our driver, 6 sessions):

| Check | Result |
|---|---|
| Angular velocity frame | We send head-local, as Valve's lighthouse driver does (Monado steamvr_lh device.cpp:1300 rotates it to world). Valve's doc says "driver world space" but its own driver doesn't; SteamVR's extrapolation of our poses matches real motion (below), so the frame is right. |
| Rotation: SteamVR's +20 ms prediction from our poses vs real motion, turning | median 0.054-0.164 deg, p90 swing 0.15-0.32 deg; ideal constant-rate predictor 0.048-0.151 / 0.15-0.28: within ~10 %. Our rotation data is as good as SteamVR's predictor allows. |
| Display timing properties | Already equal to Sony's (VsyncToPhotons 0.001360, PhotonsToVblank 0.010300, logged by Sony's driver). |
| Position: SteamVR's +20 ms prediction, turning in place | median 0.43-0.60 mm, p90 1.1-2.5 mm vs ideal 0.08 / 0.3 mm (5x); frame-to-frame swing equal to ideal (p90 ~0.2 mm). Source: psvr2_pose_filter_get linear velocity = SLAM-derived pivot velocity (accel fusion off by default) + lever-arm term; the pivot moves when the head turns and the SLAM velocity lags. Smooth drift, not a shake; same filter velocity on the Monado path. |
| ALVR head pose | Sent already predicted with zero velocity: SteamVR does no head prediction (HMD.cpp OnPoseUpdated). |
| PSVR2Toolkit | Passes Sony's head pose through unchanged; Sony's head data can't be read from source. |

What is still unknown and can't be found by research: what Sony's driver sends for the head (rotation/position
prediction quality, velocities, update cadence), and whether the frame timing SteamVR's compositor achieves differs
with Sony's driver. Sony's driver is closed; only measuring through SteamVR's client API can show it.

## 8. Plan

Step 1 — tool (small, diagnostic only): vrposeprobe also logs the velocities SteamVR returns (vVelocity,
vAngularVelocity) and the pose update cadence; analysis/probe_compare.py turns any probe log into the table above
(rotation/position error and swing vs the ideal floor, still / turning / moving). Verified on the existing logs.

Step 2 — one session with Sony's driver (needs the user; SteamVR Home is enough, ~2 min): vrposeprobe and
vrframetiming running; head still 20 s, slow turns 20 s, fast turns 20 s. Same for our driver right after, same moves.
Decision:
  - Sony's rotation or position numbers clearly better than ours → the head pose we send is the cause; the specific
    metric that differs says which part (rotation timing, position velocity, cadence). Fix designed from that and
    verified offline by replaying recorded IMU/SLAM through the pose filter before any headset test.
  - Same numbers → the poses are not the difference; compare the two vrframetiming logs (presents, mispresents,
    reprojection flags, compositor timing) — the difference is then in frame timing, and that log says where.
Step 3 — implement only the change the comparison points to, behind a switch, offline-verified; one headset run;
"fixed" only if the user sees it gone.
