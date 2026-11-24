# PSVR2 with OpenXR on Linux (Monado, without SteamVR)

PSVR2-DriverKit's `monado/` directory is a fork of [Monado](https://monado.freedesktop.org/), the open-source OpenXR
runtime, with this project's PS VR2 headset and Sense controller drivers. Native Linux OpenXR apps can run on it
directly through `monado-service`, without SteamVR.

Repository: <https://github.com/mkooner/PSVR2-DriverKit>

## What OpenXR apps get

| | Status |
|---|---|
| Headset 6DoF tracking, distortion, IPD | Stable |
| Sense controllers: 6DoF, inputs, haptics | Stable |
| Eye gaze (`XR_EXT_eye_gaze_interaction`) | Works, no per-user calibration |
| Bare-hand tracking | Experimental (driver compiled with hand tracking) |
| Passthrough | Not available on Monado |

## Run an OpenXR app

**From DriverKit:** Launcher tab → **Start monado-service**, then **Launch app** and pick the app.

**From a terminal:**
```
~/.local/share/PSVR2-DriverKit/kit/launch-openxr.sh /path/to/openxr-app [arguments]
```
The script checks that nothing else holds the headset (SteamVR, another monado-service), starts our
`monado-service`, and runs the app with `XR_RUNTIME_JSON` pointing at our build's runtime manifest. It stops the
service when the app exits and keeps the log in `logs/`.

**By hand:** start `monado-service` from the build, then run the app with
`XR_RUNTIME_JSON=<build>/openxr_monado-dev.json`.

## SteamVR or Monado?

- **SteamVR:** SteamVR games, Windows games through Proton, Room View and the wall camera, SteamVR's room setup.
- **Monado:** native Linux OpenXR apps without SteamVR, for example development or testing.

Only one of them can use the headset at a time.

## Limitations

- Native Linux OpenXR apps only. Windows games run through Proton and SteamVR instead.
- No passthrough layer on Monado.

[Installation](psvr2-installation.md) · [SteamVR](psvr2-steamvr-linux.md) · [Troubleshooting](troubleshooting.md)
