# PSVR2-DriverKit

**PlayStation VR2 on Linux.** Headset, Sense controllers and eye tracking, with full SteamVR integration and OpenXR
through [Monado](https://monado.freedesktop.org/). Also included: the PSVR2-DriverKit app, which compiles, installs and
runs everything.

## Features

### Headset
- **6DoF head tracking** from the headset's own inside-out tracking, smoothed by our pose filter.
- **Display:** distortion correction, IPD from the headset's dial.
- **Wear sensor:** SteamVR knows when the headset is on your head.
- **Function button** (under the visor): opens Room View, SteamVR's camera passthrough.
- **Passthrough cameras:**
  - **Room View** in SteamVR, the full camera picture;
  - **the camera view near your play-area walls.**
- **Eye tracking:** for OpenXR apps (`XR_EXT_eye_gaze_interaction`) and for SteamVR games (SteamVR's eye-tracking
  input). No per-user calibration yet.

### Sense controllers
- **6DoF tracking:** the controllers' LEDs, tracked by the headset's cameras, fused with their motion sensors.
- **Every input:**
  - all buttons, sticks, triggers and the grip;
  - the touch sensors on the trigger, grip, stick and face buttons;
  - the finger-distance sensors.
- **Haptics.**
- **Finger tracking:** a hand skeleton driven by the Sense's touch and distance sensors, for games that animate hands.
- **PS button:** opens the SteamVR dashboard.

### Hand tracking (experimental)
- **Bare-hand tracking** with the headset's cameras, using Monado's Mercury hand tracker, while a Sense controller
  is off. Off by default: turn it on in DriverKit's Settings tab.
- **Finger tracking** in SteamVR games, a pinch as the trigger, a closed hand as the grip; palm toward your face and
  pinch for the dashboard.
- Needs a driver built with hand tracking (onnxruntime and OpenCV).

### SteamVR integration
- **A native SteamVR driver.** Registered with Steam, so Steam starts SteamVR with it like any headset, from the
  Library or from a game.
- **Runs inside Steam's runtime container.** The libraries Steam's runtime lacks are bundled with the driver.
- **PS VR2 input profiles and bindings:** Sense controller bindings for SteamVR and for older games.
- **Room setup in the headset:** draw your play area from the dashboard's Playspace menu, like Steam Frame.
- **Room View and the wall camera,** switched on with one button in DriverKit.
- **The PS VR2 and Sense controller models and icons** in SteamVR, when Sony's PlayStation VR2 App is installed (see
  the install guide).

### OpenXR with Monado
- **Native OpenXR apps run directly on Monado's runtime** (monado-service), without SteamVR.

### PSVR2-DriverKit app
- **Compile Driver tab:** downloads this repository and builds everything into one folder. It can also register the
  driver with SteamVR, turn on SteamVR's camera features, and show package install commands for your distribution.
- **Launcher tab:**
  - start and stop SteamVR with the driver;
  - set the play area;
  - start monado-service and launch OpenXR apps.
- **Settings tab:** driver options.
- **Device status:** headset and controllers.

## Install guide

![PSVR2-DriverKit, Compile Driver tab](docs/images/driverkit-compile.png)

### What you need
- **A Linux PC with Steam and SteamVR installed.** Run SteamVR once so it creates its settings.
- **The PS VR2 connected through Sony's PC adapter.**
- **Recommended: Sony's PlayStation VR2 App from Steam** (free). The driver uses its Sense controller models and PS VR2
  icons, so SteamVR shows the real controllers instead of generic ones. Sony's files aren't in this repository: the
  compile step copies them from your Steam library, so **install the app before compiling** (or compile again after
  installing it). Without it everything works, but SteamVR shows other controllers and generic icons.

### Steps
1. **Install the build packages.** DriverKit's Compile Driver tab lists the command for Arch/CachyOS/Manjaro,
   Debian/Ubuntu and Fedora, with a Copy button. Paste it into a terminal.
2. **Build the DriverKit app once from source:**
   ```
   git clone https://github.com/mkooner/PSVR2-DriverKit.git
   cd PSVR2-DriverKit/launcher-gui && cargo build --release
   ./target/release/PSVR2-DriverKit
   ```
3. **Compile Driver tab → Compile Driver.**
   - **Where it installs:** by default `~/.local/share/PSVR2-DriverKit`. Choose another folder with Browse.
   - **What it builds:** it downloads the latest version of this repository and builds the SteamVR driver,
     monado-service, the launch scripts and tools, and a new copy of DriverKit, all into that folder. A progress bar
     shows the build.
   - **What happens after:** everything uses the new build, and the new DriverKit is in `<install folder>/kit`. Use
     that one from now on; compiling again later updates it.
4. **Register our driver** (same tab). From now on Steam starts SteamVR with the PS VR2 driver. Unregister removes only
   our driver.
5. **Turn on PS VR2 features** (same tab), once, with SteamVR closed. This switches on SteamVR's camera features for the
   PS VR2: Room View with the full camera picture, and the camera near your play-area walls. Undo puts your previous
   settings back.
6. **Start SteamVR from Steam** (or start a VR game). On the first start, set up your room from the dashboard's
   Playspace menu.

### Using it
- **SteamVR games:** start them from Steam as usual.
- **PS button:** opens the SteamVR dashboard.
- **Headset's function button:** Room View.
- **OpenXR apps without SteamVR:** DriverKit → Launcher → Start monado-service, then Launch app.
- **Windows games through Proton:** see the Proton note below.

## Known limitations
- Controller tracking drops in bright sunlight.
- The controllers' initial calibration takes about 5 seconds to start in SteamVR.
- For the best results with Proton in VR, use [UMU-Proton](https://github.com/Open-Wine-Components/umu-proton).

## Credits
- **[Monado](https://monado.freedesktop.org/)** (all the Monado developers, BSL-1.0): the OpenXR runtime this project
  is built on and part of, including the PS VR2 headset and Sense controller drivers this project extends.
- **[ALVR](https://github.com/alvr-org/ALVR)** (MIT): our reference implementation for the SteamVR driver's structure
  (provider, tracked devices, headset and controllers) and for the controller hand skeleton.
- **Mercury** (Moshi Turner and Collabora, part of Monado, BSL-1.0): the hand tracker and models behind hand tracking.
- **[PSVR2Toolkit](https://github.com/BnuuySolutions/PSVR2Toolkit)** (Bnuuy Solutions, MIT): our reference
  implementation for:
  - the SteamVR camera interfaces behind Room View, and the passthrough cameras' lens model;
  - the PS VR2 properties SteamVR expects;
  - reporting eye tracking to SteamVR.
- **Device pictures in PSVR2-DriverKit:** from Wikimedia Commons: the headset by C.Suthorn
  ([CC BY-SA 4.0](https://commons.wikimedia.org/wiki/File:PSVR2_(Non-Stereoscopic).png)), the controllers by
  ArlindoPereira ([CC BY 4.0](https://commons.wikimedia.org/wiki/File:PlayStation_VR2.jpg)).

## Repository layout

| Path | What |
|---|---|
| `monado/` | The Monado fork (BSL-1.0): PS VR2 headset driver (`src/xrt/drivers/psvr2`), Sense controllers (`src/xrt/drivers/pssense`), the SteamVR driver (`src/xrt/state_trackers/steamvr_psvr2`, built as `steamvr-monado-psvr2`). |
| `launcher-gui/` | PSVR2-DriverKit (Rust, eframe). |
| `build-driver.sh` | What Compile Driver runs: clone/update, build, bundle the libraries for Steam's runtime, copy Sony's models and icons, install. |
| `launch-steamvr.sh`, `launch-openxr.sh`, `launch-dev.sh` | Run SteamVR or an OpenXR app with the driver, with logging; `launch-dev.sh` uses a development build without registering it. |
| `analysis/`, `tools/`, `ctreplay/`, `pose-sim/`, `vio/` | Measurement, replay and test tools used during development. |
| `docs/` | Research notes and the README's images. |

## Building by hand (developers)

    cmake -S monado -B monado/build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
    ninja -C monado/build                        # monado-service, the SteamVR driver, tests
    (cd launcher-gui && cargo build --release)   # launcher-gui/target/release/PSVR2-DriverKit

The scripts find the build through `~/.config/PSVR2-DriverKit/driver.env`, which Compile Driver writes; for a build
of your own, point them at it with `MONADO_BUILD=$PWD/monado/build`. Logs go to `logs/`.
