# PSVR2 on Linux: frequently asked questions

Repository: <https://github.com/mkooner/PSVR2-DriverKit>

### Does the PlayStation VR2 work on Linux?
Yes. PSVR2-DriverKit runs the headset and the Sense controllers natively on Linux, with SteamVR and with OpenXR apps
on Monado. See [what works](compatibility.md).

### Do I need Windows, Wine or Sony's PC driver?
No. The driver is native Linux code: Monado's PS VR2 drivers plus our own SteamVR driver. Sony's **PC adapter** is
still needed to connect the headset. Sony's free **PlayStation VR2 App** is optional: when installed, its controller
models and icons are copied so SteamVR shows the real controllers.

### Do the Sense controllers have 6DoF tracking?
Yes. The headset's cameras track the controllers' LEDs, fused with their motion sensors. All inputs, the touch
sensors and haptics work too. See [Sense controllers](psvr2-sense-controllers.md).

### Does eye tracking work?
Yes, in SteamVR (its eye-tracking input) and in OpenXR (`XR_EXT_eye_gaze_interaction`). There's no per-user
calibration yet. See [eye tracking](psvr2-eye-tracking.md).

### Is there passthrough?
In SteamVR: Room View and the camera near your play-area walls. Not on Monado. See [passthrough](psvr2-passthrough.md).

### Does it work on Wayland?
Yes, on compositors with DRM leasing. Tested on GNOME Wayland. See [Wayland](psvr2-wayland.md).

### Can I play Windows VR games?
Yes, through Proton and SteamVR, if the game supports OpenVR or OpenXR. UMU-Proton works best for VR.

### Which GPUs work?
NVIDIA is tested. Intel Arc runs, without async reprojection. AMD is expected to work, but nobody has reported a
session yet. See [compatibility](compatibility.md).

### Is hand tracking supported?
Experimentally: bare-hand tracking with the headset's cameras (Monado's Mercury tracker), off by default, in a
driver compiled with hand tracking.

### How is this different from other PS VR2 Linux projects?
See the [project comparison](PROJECT_COMPARISON.md).

### Is it free?
Yes. It's open source under the Boost Software License 1.0, like Monado.

### How do I report a problem?
[Open an issue](https://github.com/mkooner/PSVR2-DriverKit/issues/new/choose) and attach the zipped
`kit/logs` folder. See [troubleshooting](troubleshooting.md#getting-logs).
