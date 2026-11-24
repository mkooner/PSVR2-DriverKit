# PSVR2 Sense controllers on Linux

PSVR2-DriverKit supports the PlayStation VR2 Sense controllers with full 6DoF tracking, every input and haptics,
in SteamVR and in OpenXR apps on Monado.

Repository: <https://github.com/mkooner/PSVR2-DriverKit>

## Pairing

1. Hold **Create + PS** on a controller until its light flashes.
2. Pair it in your desktop's Bluetooth settings (or with `bluetoothctl`).
3. Repeat for the other controller. Afterwards a press of the PS button reconnects it.

The SteamVR launcher's pre-flight list shows which controllers are paired and connected.

## Tracking

- **6DoF:** the headset's cameras see the controllers' LEDs; the driver identifies them by their blink pattern and
  fuses the camera positions with the controllers' motion sensors.
- Tracking starts about 5 seconds after the controllers come into view. Hold them in front of the headset.
- It drops in bright sunlight, where the cameras can't separate the LEDs from the light around them.

## Inputs

- All buttons (Cross, Circle, Square, Triangle, Create, Options, PS), sticks with click, L1/R1, L2/R2 triggers.
- Capacitive touch on the triggers, grips, sticks and face buttons.
- Finger-distance sensors, used for a hand skeleton in games that animate fingers.
- **Haptics.**

## Bindings in SteamVR

- PS VR2 Sense input profiles, with default bindings for SteamVR's dashboard. The PS button opens the dashboard.
- Games without SteamVR Input see the controllers as Oculus Touch by default. Settings → Legacy games' controllers
  changes that to Vive, Index (knuckles) or the native PS VR2 layout. A game's own binding in SteamVR's controller
  settings still takes priority.
- With Sony's PlayStation VR2 App installed before compiling, SteamVR shows the real Sense controller models.

## WayVR

The driver also ships Sense bindings for the WayVR desktop overlay: double-press Triangle, triggers to click, L1/R1 to
grab. See [Wayland](psvr2-wayland.md).

## Limitations

- Bright sunlight interrupts tracking.
- The left controller can be slightly shakier than the right (under investigation).

[Troubleshooting](troubleshooting.md) · [Compatibility](compatibility.md)
