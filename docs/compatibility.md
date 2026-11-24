# PSVR2 Linux compatibility: hardware, GPUs, distributions and features

What PSVR2-DriverKit supports and what has actually been tested. "Tested" means a session by the maintainer or a
player who sent logs; "expected" means it should work but nobody has confirmed it yet.

Repository: <https://github.com/mkooner/PSVR2-DriverKit> · Report your setup:
[hardware compatibility report](https://github.com/mkooner/PSVR2-DriverKit/issues/new?template=hardware_report.yml)

## Hardware

| Device | Status |
|---|---|
| PlayStation VR2 headset | Supported |
| Sony PS VR2 PC adapter | Required (DisplayPort + USB) |
| PS VR2 Sense controllers (Bluetooth) | Supported |
| Original PlayStation VR (PS4) | Not supported by this project (Monado has its own driver) |

## GPUs

| GPU | Status |
|---|---|
| NVIDIA (proprietary driver) | Tested (RTX 5080): everything works |
| AMD (Mesa RADV) | Expected; no session reported to us yet |
| Intel Arc (Mesa ANV) | Tested (A750, xe and i915): runs; **no async reprojection**; Room View can crash SteamVR's compositor, so set Headset cameras to `off` |

## Desktops

| Desktop | Status |
|---|---|
| GNOME Wayland (Shell 51) | Tested |
| KDE Plasma Wayland, Sway, Hyprland | Expected (DRM leasing); untested by us |
| X11 | Expected; untested by us |

See [Wayland](psvr2-wayland.md).

## Distributions

`install-deps.sh` installs packages on Arch-based (pacman), Debian-based (apt) and Fedora-based (dnf) systems.
Tested: CachyOS (Arch-based). Experimental hand tracking needs Debian 13, Ubuntu 25.04, Fedora 43 or newer, or an
Arch-based system (onnxruntime packages).

## Features

| Feature | SteamVR | OpenXR (Monado) | Status |
|---|---|---|---|
| Headset 6DoF tracking | ✓ | ✓ | Stable |
| Distortion correction, IPD dial | ✓ | ✓ | Stable |
| Sense controllers 6DoF | ✓ | ✓ | Stable (drops in bright sunlight) |
| Buttons, sticks, triggers, touch, finger distance | ✓ | ✓ | Stable |
| Haptics | ✓ | ✓ | Stable |
| Finger skeleton from controller sensors | ✓ | – | Stable |
| Eye tracking | ✓ | ✓ | Works; no per-user calibration |
| Room View passthrough, wall camera | ✓ | – | Stable (not on Intel Arc) |
| Room setup in the headset | ✓ | – | Stable |
| Bare-hand tracking | ✓ | ✓ | Experimental |
| Desktop in VR (WayVR) | ✓ | – | Experimental, untested with a headset |
| Async reprojection | GPU-dependent | Monado's own | Not on Intel Arc |
| Windows | – | – | Not supported |

[Troubleshooting](troubleshooting.md) · [FAQ](faq.md)
