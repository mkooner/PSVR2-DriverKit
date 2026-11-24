# PSVR2 passthrough on Linux: Room View and the wall camera

PSVR2-DriverKit gives SteamVR the PlayStation VR2's cameras, so you can see your room through the headset.

Repository: <https://github.com/mkooner/PSVR2-DriverKit>

## What you get

- **Room View:** SteamVR's camera view of your surroundings. Press the headset's **function button** (under the
  visor) to toggle it, or use the SteamVR dashboard.
- **The camera near play-area walls:** when you get close to your boundary, SteamVR shows the camera picture there.
- **Camera in room setup:** SteamVR's room setup can show the camera while you draw the play area.

## Turn it on

DriverKit → Compile Driver tab → **Turn on PS VR2 features**, once, with SteamVR closed. This switches on SteamVR's
camera features for the PS VR2. **Undo** puts your previous SteamVR settings back.

## Settings

| Setting | Values | Use |
|---|---|---|
| Headset cameras in SteamVR | `on` (default), `noimages`, `off` | `off` removes the camera from SteamVR completely. |
| Lighter camera picture | off (default) / on | Half-size picture (1016×508 instead of 2032×1016), for PCs where Room View stutters. |

The driver sends the camera picture 15 times a second while Room View is hidden and at the full rate while it's
shown, to keep SteamVR's uploads to the GPU small.

## Limitations

- **SteamVR only.** Monado has no passthrough for the PS VR2.
- **Intel Arc GPUs:** SteamVR's compositor can crash while it builds the camera view (a Mesa driver issue). Set
  Headset cameras in SteamVR to `off` on those PCs.
- The picture is the headset's grey tracking cameras, not colour.

[Troubleshooting](troubleshooting.md) · [SteamVR](psvr2-steamvr-linux.md)
