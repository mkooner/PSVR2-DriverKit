# PSVR2 eye tracking on Linux

PSVR2-DriverKit reads the PlayStation VR2's built-in eye tracking and passes it on, both to SteamVR games and to
OpenXR apps on Monado.

Repository: <https://github.com/mkooner/PSVR2-DriverKit>

## Where gaze goes

| Runtime | Interface | Status |
|---|---|---|
| SteamVR | SteamVR's eye-tracking input on the headset | Works |
| OpenXR (Monado) | `XR_EXT_eye_gaze_interaction` | Works |

No setup is needed: it's on whenever the driver runs. Apps and games have to support eye tracking themselves, for
example for foveated rendering or eye contact on avatars.

## Limitations

- **No per-user calibration yet.** The gaze direction comes from the headset's own estimate, so it may be off by a
  few degrees for some people. Per-user calibration is planned.
- Only apps that read SteamVR's or OpenXR's eye-gaze input use it.

[SteamVR](psvr2-steamvr-linux.md) · [OpenXR](psvr2-openxr-linux.md) · [Compatibility](compatibility.md)
