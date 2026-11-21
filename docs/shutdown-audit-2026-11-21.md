# Shutdown audit: SteamVR driver (OpenVR) and OpenXR session handling, 2026-11-21

Scope: DriverKit's SteamVR plugin (`monado/src/xrt/state_trackers/steamvr_psvr2`, "ovrn") against Valve's
[OpenVR Driver API documentation](https://github.com/ValveSoftware/openvr/blob/master/docs/Driver_API_Documentation.md),
and the OpenXR side (Monado's runtime, DriverKit's OpenXR tools) against the
[OpenXR 1.1 specification](https://registry.khronos.org/OpenXR/specs/1.1/html/xrspec.html).

Trigger: SteamVR crashes when a game exits, with `Destroy() is failed!!` in the log. In the user's logs
(2026-11-20/21) that crash comes before vrserver starts shutting down, the driver's shutdown then completes, and the
message isn't printed by this code: the crash itself is in another SteamVR process (see the end). The audit still
found three places where the driver breaks an OpenVR rule at shutdown; they are fixed here.

## 1. OpenVR compliance, function by function

| Function / rule | Before | After |
|---|---|---|
| `HmdDriverFactory` returns only implemented interfaces; others NULL + `VRInitError_Init_InterfaceNotFound` (`ovrn_provider.cpp`) | OK | OK |
| Resource use minimal before `Init` (Steam's client also loads the library, for the watchdog) | OK: the global provider does nothing until `Init` | OK |
| `ITrackedDeviceServerDriver::Deactivate` frees device-specific resources | **No**: the headset's camera (2 threads, the stream, SteamVR's frame queue) lived on until `Cleanup` | Camera stops in the headset's `deactivate()` |
| No runtime call for a device after `Deactivate` returns | **No**: the 1 kHz pose thread read the device index without a lock, so a `TrackedDevicePoseUpdated` (and twice a second `SetDisplayEyeToHead` + `SetFloatProperty`) could land just after `Deactivate` returned; the camera worker kept writing to SteamVR's frame queue until `Cleanup` | Those calls run under a per-device lock that `Deactivate` takes (`withActive`); the camera stops in `Deactivate` |
| Device objects valid until `Cleanup` (`TrackedDeviceAdded`) | OK: owned by the provider, released in `Cleanup` | OK |
| No premature destruction | OK | OK |
| Worker threads stop before the objects they use are destroyed | OK: `Cleanup` joins the pose thread before releasing devices; the camera joins its threads in `shutdown()`; the backend joins the hotplug thread first and destroys Monado's devices (and their USB, tracker and Mercury threads) before the instance | OK |
| No double destruction between `Deactivate` and `Cleanup` | OK (`Deactivate` only cleared the index) | OK: `Camera::shutdown()` runs once, the destructor's call does nothing after it |
| `IServerTrackedDeviceProvider::Cleanup` releases everything | OK: pose thread, devices, backend, SteamVR settings restored, `VR_CLEANUP_SERVER_DRIVER_CONTEXT` last | OK |
| `RunFrame` doesn't block | OK | OK |
| Driver unloaded without `Cleanup` | OK: `~Provider` joins the pose thread | OK |

Calls made on SteamVR's own thread (`RunFrame`: inputs, skeletons, universe, haptics) already check `active()`,
and `Deactivate` runs on that thread too, so they can't race with it.

## 2. Resource ownership

```
SteamVR vrserver (owns the process, calls Init / RunFrame / Activate / Deactivate / Cleanup)
└── Provider  (ovrn_provider.cpp, global g_provider)
    ├── pose thread  ─ reads m_hmd, m_live[], m_liveHands[]; joined first in Cleanup
    ├── Hmd ────────── TrackedDevice: index + container under m_activeMutex
    │   └── Camera ── watcher thread, worker thread, SteamVR frame queue  (stopped at Hmd::deactivate)
    ├── Controller x2, HandDevice x2 ── TrackedDevice each
    └── Backend (MonadoBackend, owned by the provider, stopped last)
        ├── hotplug thread            (joined first in stop)
        ├── camera callback           (m_cameraMutex; none in flight after cameraStop)
        └── Monado: xrt_instance → xrt_system → xrt_system_devices (+ space overseer)
            ├── PS VR2 headset device ── USB thread, LED/SLAM/camera streams
            ├── Sense controllers ────── Bluetooth HID threads, LED sync
            └── controller tracker, Mercury hand tracking ── their own threads
```

The plugin owns no OpenXR handles: SteamVR's compositor owns the display, swapchains and the app's session.
Monado's OpenXR objects (instance, session, swapchains, spaces, actions) exist only in monado-service mode and
belong to Monado's state tracker.

## 3. Shutdown sequence (after the fixes)

1. SteamVR: `Deactivate()` on each device (headset first in practice).
   1. The device takes `m_activeMutex`: waits for a pose / IPD call in flight, then clears its index and container.
   2. Headset only: `Camera::shutdown()` — watcher quit and joined, stream stopped (backend `cameraStop`, worker joined), frame queue destroyed.
2. SteamVR: `Cleanup()`.
   1. Pose thread stopped and joined.
   2. Controllers, hands, headset released (the camera's destructor finds it already shut down).
   3. `Backend::stop()`: hotplug thread joined, eye tracking ended, space overseer → system devices (USB, Bluetooth, tracker, Mercury threads end) → system → instance destroyed; Sense controllers disconnected over Bluetooth (at most 1 s each).
   4. SteamVR settings put back (async reprojection, see-through camera settings).
   5. `VR_CLEANUP_SERVER_DRIVER_CONTEXT()`.

The same order shows in the 2026-11-21 13:12 log: "VR server shutting down" → "passthrough off" → the controllers'
Bluetooth disconnects → "your SteamVR camera settings put back" (before the fix the camera stopped in step 2.2, not 1.2).

## 4. OpenXR

- **Monado's runtime (upstream, not changed by DriverKit):** `xrEndSession` returns `XR_ERROR_SESSION_NOT_RUNNING`
  / `XR_ERROR_SESSION_NOT_STOPPING` outside `STOPPING`, as the spec requires (`oxr_session.c`); destroying a session
  destroys its children first through Monado's handle tree, then its compositor and `xrt_session`. No violation found.
- **DriverKit's OpenXR test tools** (`tools/xreyetest.c`, `tools/xr_frame_log/layer_test_client.c`): they react
  only to `READY`. They don't call `xrEndSession` on `STOPPING`, don't stop on `EXITING` / `LOSS_PENDING`, keep
  calling frame / input functions if the runtime stops the session, and don't check `xrDestroySession` /
  `xrDestroyInstance` results. Destroying a running session is allowed by the spec, so their normal exit is valid;
  the gap is a runtime-initiated stop. They're developer tools, not part of a game's exit, so not changed here.

## 5. Fixes (minimal, in this commit)

- `ovrn_device.hpp`: `m_activeMutex`; `Deactivate` takes it while clearing the index and container, then calls the
  new `deactivate()` hook; `withActive(fn)` runs the pose thread's calls under it; `submitPose` uses it.
- `ovrn_hmd.hpp`: the IPD update (`SetDisplayEyeToHead`, `Prop_UserIpdMeters_Float`) through `withActive`;
  `deactivate()` shuts the camera down.
- `ovrn_camera.hpp`: `shutdown()` (the destructor's old body, once only) and no new stream after it.

The lock is held only around each call to SteamVR, on the pose thread (not the render path); `Deactivate` waits at
most for one call in flight, as Valve's sample drivers wait for their pose thread there. No sleeps.

## 6. Regression tests (`monado/tests/ovrn_driver_harness.cpp`)

- Five start / stop rounds with the simulated backend (was two): `Init` → `Activate` → poses → `Deactivate` every
  device → `Cleanup`, as SteamVR across game exits and restarts (one round on real hardware: a USB device can't be
  re-claimed in one process).
- No pose and no eye-to-head update for the headset counted from the moment `Deactivate` returns (was after 20 ms,
  which hid a call in flight); the other devices keep sending until theirs.
- Every device deactivated before `Cleanup`, each checked the same way.
- Unchanged: no pose after `Cleanup`, settings restored, the library unloads.

Not run here (Windows): build and run `ovrn_driver_harness` on Linux.

## 7. The `Destroy() is failed!!` crash

Not from this driver: the message isn't in this code, the crash dump is written before vrserver begins shutting
down, and the driver's shutdown completes afterwards. Every logged session pairs it with `Initialize() is failed!!!`
at start, among SteamVR's own window process (vrmonitor) start-up lines, and it follows the Steam client quitting.
Other SteamVR-on-Linux reports show the same pair with other headsets. Report it to Valve with the crash ID.
