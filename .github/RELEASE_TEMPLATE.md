<!--
Release notes template. Copy into a new GitHub release and fill in every section. Leave a section out rather than
guess. Only list hardware someone actually tested this version on. Mark the release as a pre-release unless it has
been tested on a headset.
-->
# PSVR2-DriverKit vX.Y

Released YYYY-MM-DD.

PlayStation VR2 on Linux: native SteamVR driver and OpenXR through Monado, with 6DoF headset and Sense controller
tracking, eye tracking, Room View passthrough and haptics.

## New features
- **Feature** (stable / experimental): what it does and how to turn it on.

## Fixes
- What was wrong, what changed. Link the issue: #NN.

## Known limitations
- Carry over the ones still true from the README's Known limitations.

## Supported distributions
- Automatic package install: Arch-based (pacman), Debian-based (apt), Fedora-based (dnf).
- Tested for this release: <distribution, desktop, session type>.

## Tested hardware
| GPU / driver | Desktop | Result |
|---|---|---|
| <GPU, driver version> | <desktop, Wayland/X11> | <what was tested and how it went> |

## Install
1. Download `PSVR2-DriverKit-x86_64-vX.Y.AppImage` below, `chmod +x` it and run it.
2. Compile Driver tab: **Install dependencies**, then **Compile Driver**, **Register our driver**, **Turn on PS VR2
   features**.

Full guide: https://github.com/mkooner/PSVR2-DriverKit/blob/main/docs/psvr2-installation.md

## Upgrade
- From a previous version: run **Compile Driver** again in your DriverKit (`<install folder>/kit`). It updates and
  rebuilds in place. A new AppImage is only needed for changes to the app's first-run flow.

## Downloads
| File | SHA-256 |
|---|---|
| `PSVR2-DriverKit-x86_64-vX.Y.AppImage` | `sha256sum` output |
