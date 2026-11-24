# PlayStation VR2 (PSVR2) Linux driver: what PSVR2-DriverKit is

PSVR2-DriverKit runs the Sony PlayStation VR2 headset and its Sense controllers natively on Linux. You don't need
Windows, Wine or Sony's Windows driver. It includes:

- a **native SteamVR driver** for the headset and the Sense controllers;
- the PS VR2 drivers inside a fork of **[Monado](https://monado.freedesktop.org/)**, the open-source OpenXR runtime,
  so native OpenXR apps can run **without SteamVR**;
- the **PSVR2-DriverKit app**, which installs the packages, compiles the driver, registers it with Steam and
  launches it.

Repository: <https://github.com/mkooner/PSVR2-DriverKit>

## How it works

```
 PS VR2 headset ──(Sony PC adapter: DisplayPort + USB)──► Linux PC
 Sense controllers ──(Bluetooth)───────────────────────►   │
                                                           ▼
                          Monado PS VR2 + Sense drivers (this repository's monado/)
                             │                                  │
              SteamVR driver (steamvr-monado-psvr2)     monado-service (OpenXR)
                             │                                  │
                  SteamVR games, Proton games           native OpenXR apps
```

- **Head tracking** comes from the headset's own inside-out tracking, smoothed by our pose filter.
- **Controller tracking** comes from the headset's cameras seeing the Sense controllers' LEDs, fused with the
  controllers' motion sensors.
- **The display** belongs to SteamVR or Monado, which lease it from your desktop (DRM leasing). The driver supplies
  the lens distortion, the IPD from the headset's dial and the display timing.

## What works today

See the full table with status in [compatibility](compatibility.md). In short:

| | Status |
|---|---|
| SteamVR driver, headset 6DoF, Sense controllers 6DoF, all inputs, haptics, finger skeleton | Stable |
| OpenXR apps on Monado without SteamVR | Stable |
| Room View passthrough and the camera near play-area walls (SteamVR) | Stable, not on Intel Arc |
| Eye tracking (SteamVR and OpenXR) | Works, no per-user calibration yet |
| Bare-hand tracking | Experimental, off by default |
| Desktop in VR through WayVR | Works, off by default |

## What you need

- The PS VR2, **Sony's PS VR2 PC adapter**, and a GPU with a free DisplayPort output.
- A Linux PC with Steam (the native package, not Flatpak or Snap) and SteamVR.
- An Arch-, Debian- or Fedora-based distribution for the automatic package install. Others work if you install the
  packages yourself.

Start with the [installation guide](psvr2-installation.md).

## Related pages

- [Installation](psvr2-installation.md) · [SteamVR on Linux](psvr2-steamvr-linux.md) ·
  [OpenXR on Linux](psvr2-openxr-linux.md) · [Sense controllers](psvr2-sense-controllers.md) ·
  [Eye tracking](psvr2-eye-tracking.md) · [Passthrough](psvr2-passthrough.md) · [Wayland](psvr2-wayland.md) ·
  [Troubleshooting](troubleshooting.md) · [FAQ](faq.md) · [How it compares](PROJECT_COMPARISON.md)
