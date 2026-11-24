# PSVR2 on Wayland (and X11)

Does the PS VR2 work on a Wayland desktop? Yes, as long as your compositor lets SteamVR or Monado **lease** the
headset's display: the `wp_drm_lease_device_v1` protocol.

Repository: <https://github.com/mkooner/PSVR2-DriverKit>

## Display leasing

SteamVR and Monado take the headset's display away from your desktop and drive it directly. On Wayland your
compositor has to offer DRM leasing; on X11 the X server does.

| Desktop | Status with PSVR2-DriverKit |
|---|---|
| GNOME Wayland | Works (tested on GNOME Shell 51: SteamVR logs `wp_drm_lease_device_v1`) |
| KDE Plasma Wayland, Sway, Hyprland and other compositors with DRM leasing | Expected to work (Valve lists them as supported); not yet tested by us |
| Compositors without DRM leasing | SteamVR can't use the headset |
| X11 | Supported by SteamVR; not yet tested by us |

Older guidance says GNOME Wayland lacks DRM leasing. Current GNOME has it: a GNOME Shell 51 session leased the
headset to SteamVR without trouble.

## Your desktop inside VR (WayVR)

SteamVR's own Desktop panel in the dashboard is blank on Wayland. That's a SteamVR issue
([SteamVR-for-Linux #693](https://github.com/ValveSoftware/SteamVR-for-Linux/issues/693)), not ours, and we can't
fix it. Instead, PSVR2-DriverKit works with [WayVR](https://github.com/wayvr-org/wayvr), a separate open-source
overlay that shows Wayland and X11 screens in SteamVR.

1. Install WayVR: Arch AUR package `wayvr`, or its AppImage in `~/Applications`. It isn't part of this project.
2. DriverKit → Settings → **Start WayVR with SteamVR**. DriverKit starts WayVR once SteamVR's compositor runs,
   restarts it if it stops unexpectedly, and stops it afterwards.
3. The first time, your desktop asks which screens to share. Pick them in the order WayVR asks.

Sense controller bindings for WayVR come with the driver:

| WayVR | Sense controller |
|---|---|
| Show / hide | double-press Triangle |
| Click, pointer | L2/R2 press, touch |
| Grab (move, resize) | L1/R1 |
| Scroll | sticks |
| Right / middle click | touch Triangle or Circle / Square or Cross while clicking |

Works with SteamVR or Monado. With SteamVR, close SteamVR's dashboard (PS button) before using WayVR: WayVR gets no
controller input while the dashboard is open.

With Monado: Settings → Monado section → Start WayVR with monado-service.

[Troubleshooting](troubleshooting.md) · [Compatibility](compatibility.md)
