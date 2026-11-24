# Contributing to PSVR2-DriverKit

Thank you for helping bring the PlayStation VR2 to Linux. You can help in many ways; most of them don't need code.

## Test reports

The most valuable contribution right now is a test session on hardware we haven't seen yet: AMD GPUs, KDE Plasma,
Sway, Hyprland, X11, other distributions.

1. Start SteamVR from DriverKit → Launcher → **Start SteamVR** (or `kit/launch-steamvr.sh`).
2. Use the headset for a few minutes, then quit SteamVR.
3. Zip `<install folder>/kit/logs/` (default `~/.local/share/PSVR2-DriverKit/kit/logs/`).
4. Open a [hardware compatibility report](https://github.com/mkooner/PSVR2-DriverKit/issues/new?template=hardware_report.yml)
   and attach the zip.

The logs contain your Linux user name in file paths, and device serial numbers. Look through them before posting
them publicly if that matters to you.

## Bugs, installation problems, feature requests

Use the [issue templates](https://github.com/mkooner/PSVR2-DriverKit/issues/new/choose). Please search existing
issues first.

## Code

- **Where things are:** see the README's repository layout. Driver code lives in `monado/` (Monado fork,
  BSL-1.0); the DriverKit app in `launcher-gui/` (Rust).
- **Build:** see "Building by hand" in the README. Run the plugin's tests with
  `ninja -C monado/build && ctest --test-dir monado/build -R ovrn`.
- **Style:** follow the surrounding code. Monado code uses its `.clang-format`.
- **Tracking changes must not slow the tracking path:** measure the per-frame cost.
- **Fix causes, not symptoms:** if a change is a workaround, say so in the code and the pull request.
- **Test offline first:** recorded data, the driver harness (`ovrn_driver_harness`) and unit tests, before asking
  for a headset session.
- **Commits:** one logical change per commit, with a message that says what changed and why.

## Documentation and website

User docs are in `docs/`, the website in `site/` (plain HTML fragments built by `site/build.py`, Python standard
library only). Check your changes with:

```
python3 site/build.py && python3 site/check.py --markdown
```

Only describe features that work. Mark experimental ones as experimental.

## License

By contributing you agree that your contribution is licensed under the [Boost Software License 1.0](LICENSE), the
project's license.
