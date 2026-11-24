# PSVR2 with SteamVR on Linux

PSVR2-DriverKit includes a native SteamVR driver for the PlayStation VR2 (`steamvr-monado-psvr2`). Once it's
registered, Steam starts SteamVR with the PS VR2 like any other headset, from the Library or from a VR game. The
SteamVR driver is built from Monado's PS VR2 and Sense drivers, and runs inside Steam's runtime container: the
libraries that container lacks are bundled with it.

Repository: <https://github.com/mkooner/PSVR2-DriverKit>

## Set it up

1. Install and compile with the [installation guide](psvr2-installation.md).
2. DriverKit → Compile Driver tab → **Register our driver**. This takes any other registered PS VR2 driver off
   SteamVR's list, because two drivers can't both run the headset.
3. **Turn on PS VR2 features** (once, SteamVR closed) for Room View and the wall camera.
4. Start SteamVR from Steam.

Or start it from DriverKit → Launcher → **Start SteamVR**. That runs `launch-steamvr.sh`, which first checks the
setup and reports in its pre-flight list: udev rules, controller Bluetooth connections, the high-priority GPU queue,
and whether another program holds the headset. It also collects and summarizes the logs when SteamVR exits.

## What SteamVR gets

- **The headset:** 6DoF tracking, lens distortion, the IPD from the dial, the wear sensor, and the function button
  (Room View). Eye tracking goes to SteamVR's eye-tracking input.
- **The Sense controllers:** 6DoF tracking, every button, stick, trigger and grip, the touch and finger-distance
  sensors, haptics, a hand skeleton for games that animate fingers, and the PS button for the dashboard.
- **Bindings:** PS VR2 input profiles, SteamVR dashboard bindings, and layouts for older games without SteamVR Input:
  shown as Oculus Touch by default, or Vive, Index or native PS VR2 (Settings → Legacy games' controllers).
- **Room setup in the headset:** draw your play area from the dashboard's Playspace menu.
- **Room View** and the camera view near the play-area walls. See [passthrough](psvr2-passthrough.md).

## Settings worth knowing

DriverKit's Settings tab. The SteamVR driver reads them when SteamVR starts:

| Setting | Default | What it does |
|---|---|---|
| Render size per eye (%) | 140 | Of the panel's 2000×2040 per eye. Lower it on slower GPUs. |
| Display refresh (Hz) | 89.91 | 89.91 or 119.88. |
| Legacy games' controllers | touch | What games without SteamVR Input see: touch, vive, knuckles or native. |
| Headset cameras in SteamVR | on | `on`, `noimages` or `off` (for Intel Arc, see [troubleshooting](troubleshooting.md)). |
| Lighter camera picture | off | Half-size camera picture for slower PCs. |
| Hand tracking (experimental) | off | Needs a driver compiled with hand tracking. |
| Start WayVR with SteamVR | off | Desktop in VR, see [Wayland](psvr2-wayland.md). |

## Windows games through Proton

Proton games that support OpenVR or OpenXR through SteamVR run as usual. For the best results in VR, use
[UMU-Proton](https://github.com/Open-Wine-Components/umu-proton). Start SteamVR first, then the game.

## Limitations

- **Async reprojection** needs SteamVR's compositor to get a high-priority GPU queue: use DriverKit's Allow high
  priority. It isn't available on Intel Arc GPUs, so late frames there are shown again unreprojected.
- **SteamVR's own Desktop panel** in the dashboard is blank on Wayland
  ([SteamVR bug](https://github.com/ValveSoftware/SteamVR-for-Linux/issues/693)); see [Wayland](psvr2-wayland.md).
- The headset's own tracking resets once in the first 10–30 s, and the controllers take about 5 s to start tracking.

[Troubleshooting](troubleshooting.md) · [Compatibility](compatibility.md) · [FAQ](faq.md)
