# PSVR2 Linux troubleshooting

Common problems with PSVR2-DriverKit and how to fix them. If yours isn't here, open an issue with your logs:
<https://github.com/mkooner/PSVR2-DriverKit/issues/new/choose>

## Getting logs

Start SteamVR from DriverKit → Launcher → **Start SteamVR**, or run `launch-steamvr.sh` from the kit folder.
When SteamVR exits, the logs are collected in `<install folder>/kit/logs/` (default
`~/.local/share/PSVR2-DriverKit/kit/logs/`) and summarized in the terminal: stages, startup timeline, headset
tracking resets, headset presence. Zip the `logs` folder for an issue.

## The controllers aren't found

- **udev rules missing:** the pre-flight list says `no udev rules for the PS VR2 and Sense controllers`. Run
  **Install dependencies** (Compile Driver tab) or `./install-deps.sh`, then turn the controllers off and on.
- **Not connected:** `paired but not connected: press its PS button`. Press PS on each controller.
- **Never paired:** hold **Create + PS** until the light flashes, and pair it in your Bluetooth settings.
- **Tracking takes a few seconds:** hold both controllers in front of the headset. They need about 5 s to start.

## Something else holds the headset

The pre-flight list stops with one of these:

- `SteamVR is already running`: close it first.
- `monado-service is running`: the SteamVR driver runs Monado itself. Stop the other one.
- `Ignition server (Sony driver helper) is running`: left over from running Sony's Windows driver through
  Wine/Proton. Stop it (`pkill -f ignition_server.exe`).

## The picture flickers or judders when frames are late

- SteamVR needs a **high-priority GPU queue** for async reprojection. In the pre-flight list,
  `[??] SteamVR may not use a high-priority GPU queue` means it's missing. Press DriverKit → Compile Driver tab →
  SteamVR features → **Allow high priority**.
- **Intel Arc:** SteamVR can't get async reprojection with Intel's Linux drivers. With `xe` the priority is refused;
  with `i915` there's no separate compute queue. Lower the game's settings or the render size, so frames arrive on
  time.
- Lower Settings → Render size per eye (%) if your GPU can't keep up at 140%.

## SteamVR crashes when Room View or room setup opens (Intel Arc)

On Intel Arc, SteamVR's compositor can crash while it builds the camera view (a Mesa driver issue). Set Settings →
**Headset cameras in SteamVR** to `off`. You lose Room View, but everything else works.

## Compile Driver fails

- Run **Install dependencies** first. Missing packages are the usual cause.
- Hand tracking needs OpenCV and onnxruntime. Untick Compile hand tracking if your distribution has no onnxruntime
  package (Ubuntu 24.04, for example).
- Include the logs from the install folder (default `~/.local/share/PSVR2-DriverKit/`): `download.log`,
  `configure.log`, `compile.log` and `driverkit-build.log`, whichever exist.

## A game opens on the desktop instead of the headset

- Start SteamVR **first**, wait for its home, then the game.
- Windows games through Proton: use [UMU-Proton](https://github.com/Open-Wine-Components/umu-proton), and check that
  the game supports OpenVR or OpenXR. Its log says which VR backends it tried.

## SteamVR's Desktop panel is blank

That's a SteamVR issue on Wayland ([#693](https://github.com/ValveSoftware/SteamVR-for-Linux/issues/693)). Use WayVR
instead: see [Wayland](psvr2-wayland.md).

## Steam from Flatpak or Snap

These break the headset's display lease and reprojection. Use your distribution's native Steam package.

## Head position jumps once after starting

The headset resets its own tracking once in the first 10–30 s. The driver carries the position over the reset, so
it shouldn't jump. If it does, send logs: the summary's "Headset tracking resets" section shows it.

[Compatibility](compatibility.md) · [FAQ](faq.md) · [Installation](psvr2-installation.md)
