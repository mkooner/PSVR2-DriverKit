# PS VR2 on Linux: how the projects compare

Several projects bring the PlayStation VR2 to Linux in different ways. This page compares what each one
**documents**, so you can choose the one that fits. It isn't a judgement of their quality: each takes a different
approach, and they've all moved the PS VR2 on Linux forward. PSVR2-DriverKit itself builds on Monado and credits
PSVR2Toolkit as a reference.

Sources were read on 2026-11-23. Where a project's own documentation is silent, the table says **not documented**.
Statements from the community wiki [vronlinux.org](https://vronlinux.org/docs/hardware/psvr2/) are marked as such.
Corrections are welcome: [open an issue](https://github.com/mkooner/PSVR2-DriverKit/issues).

## The projects

- **PSVR2-DriverKit** (this project, BSL-1.0): native SteamVR driver plus a Monado fork with PS VR2 and Sense
  drivers, and a GUI app to install and run them. <https://github.com/mkooner/PSVR2-DriverKit>
- **psvr2-linux-adapter** (GPL-2.0): "An open project to bring the Sony PlayStation VR2 to Linux". A kernel module,
  a userspace library and a SteamVR driver. Describes itself as "early-stage". <https://github.com/unterschall/psvr2-linux-adapter>
- **Monado** (upstream): the open-source OpenXR runtime for Linux and Android. Its own supported-hardware list doesn't
  name the PS VR2; vronlinux.org describes upstream Monado PS VR2 support. <https://monado.freedesktop.org/>
- **PSVR2Toolkit** (MIT): "Unofficial modification for the official PlayStation VR2 driver/app". A Windows driver
  mod that runs on Linux through its Ignition layer (Wine/Proton). <https://github.com/BnuuySolutions/PSVR2Toolkit>

## Comparison

| | PSVR2-DriverKit | psvr2-linux-adapter | Monado (upstream) | PSVR2Toolkit (Linux via Ignition) |
|---|---|---|---|---|
| Approach | Native Linux (Monado drivers + SteamVR driver) | Native Linux (kernel module + SteamVR driver) | Native Linux OpenXR runtime | Sony's Windows driver under Wine/Proton |
| SteamVR | Yes | Yes | Not documented for PS VR2 | Yes |
| OpenXR without SteamVR | Yes (monado-service) | Planned ("OpenXR shims") | Yes | Not documented |
| Headset 6DoF | Yes | Yes | Yes (per vronlinux.org) | Yes (play area set up with its Unity tool) |
| Sense controller 6DoF | Yes | Not documented | 3DoF (per vronlinux.org) | Yes (per vronlinux.org; "custom LED sync") |
| Controller haptics | Yes | Listed as a help-wanted area | Not documented | Yes ("PCM haptics"); adaptive triggers |
| Eye tracking | Yes (SteamVR and OpenXR; no per-user calibration) | Yes (`/dev/psvr2-gaze`) | Yes, OpenXR (per vronlinux.org) | Yes, with calibration (some features need a headset jailbreak) |
| Passthrough / Room View | Yes, SteamVR | Not documented (cameras exposed via V4L2) | No (per vronlinux.org) | Listed on Windows; "unsupported or unstable" on Linux |
| Needs Sony's PlayStation VR2 App | Optional (controller models and icons) | Not documented | Not documented | Yes |
| Wayland | Works with DRM leasing (tested GNOME); WayVR integration | "Wayland display acquisition" is a help-wanted item | Wayland options documented | Not documented |
| Installation | GUI app (AppImage or source): installs packages, compiles, registers | Scripts (DKMS), Arch PKGBUILD | Distribution packages or source | Manual steps (install script, DLL swap) |
| GUI management | Yes (PSVR2-DriverKit) | No | No | Not documented for Linux |
| Status | Releases v0.1, v0.2 (Nov 2026) | Early-stage, no releases shown | Mature runtime | Active; "some issues and missing features" on Linux |

## What PSVR2-DriverKit adds

- **One native stack for SteamVR and OpenXR:** the same Monado PS VR2 and Sense drivers serve both, with no Windows
  driver underneath.
- **6DoF Sense controllers on native Linux**, with every input, touch and finger-distance sensors, haptics and a
  hand skeleton.
- **SteamVR camera features:** Room View, the wall camera, room setup in the headset.
- **A GUI that does the setup:** packages, udev rules, compile, registration with Steam, launch with pre-flight
  checks and log collection.

## When another project may suit you better

- You want **features that rely on Sony's own driver** (its eye calibration, PCM haptics, adaptive triggers): see
  PSVR2Toolkit.
- You want **kernel-level device interfaces** (V4L2 cameras, `/dev` gaze and pose nodes) for your own software: see
  psvr2-linux-adapter.
- You only need **an OpenXR runtime** and already use Monado packages: upstream Monado.
