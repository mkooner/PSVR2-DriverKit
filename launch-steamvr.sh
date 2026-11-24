#!/usr/bin/env bash
# Launch SteamVR on our Monado build (psvr2-headset-constellation) for PS VR2 testing.
#
# Usage:
#   ./launch-steamvr.sh            launch, wait until SteamVR exits, restore config, summarize logs
#   ./launch-steamvr.sh --restore  only restore the OpenVR config (if a previous run was killed)
#
# What it does:
#   1. Backs up ~/.config/openvr/openvrpaths.vrpath and registers ONLY our build's SteamVR plugin
#      (build/steamvr-monado-psvr2, driver_monado_psvr2), so Sony's PlayStation VR2 App driver doesn't fight over the
#      headset.
#   2. Starts SteamVR normally, inside Steam's Linux Runtime container, with the container-ready plugin from
#      ../trial/build-steamvr (a minimal build with its libraries bundled next to it, OVRN_BUNDLE_LIBS): Steam games
#      run inside that container too and can only connect to a SteamVR running there (2026-11-19). Without that
#      build, or with HOST_STEAMVR=1, SteamVR runs on the host (VALVE_SKIP_RUNTIME_SAFETY, the old way: the full
#      build's libraries aren't in the container) and Steam games can't connect.
#   3. Enables the PS VR2 / Sense / constellation debug logs.
#   4. When SteamVR exits (or on Ctrl+C), restores the original config and copies + summarizes the logs.
#
# Monado runs inside SteamVR's vrserver with this plugin, so don't run monado-service at the same time.

set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
# Where DriverKit's Compile Driver tab (build-driver.sh) put the driver: used unless MONADO_BUILD is given.
DK_ENV="${XDG_CONFIG_HOME:-$HOME/.config}/PSVR2-DriverKit/driver.env"
if [ -z "${MONADO_BUILD:-}" ] && [ -f "$DK_ENV" ]; then
	MONADO_BUILD="$(. "$DK_ENV" && printf '%s' "${MONADO_BUILD:-}")"
	[ -n "$MONADO_BUILD" ] && [ -d "$MONADO_BUILD" ] || unset MONADO_BUILD
fi
BUILD="${MONADO_BUILD:-$HERE/../trial/build}"
# The container-ready plugin (build-steamvr: cmake -S trial -B trial/build-steamvr with only the PS VR2 drivers and
# -DOVRN_BUNDLE_LIBS=ON, see ROOM.md / README): preferred unless MONADO_BUILD is given.
STEAMVR_BUILD="${STEAMVR_BUILD:-$HERE/../trial/build-steamvr}"
if [ "${IN_CONTAINER:-0}" = 1 ] && [ -z "${MONADO_BUILD:-}" ] && [ -f "$STEAMVR_BUILD/steamvr-monado-psvr2/bin/linux64/libceres.so.4" ]; then
	BUILD="$STEAMVR_BUILD"
fi
# The PS VR2 SteamVR driver (trial/src/xrt/state_trackers/steamvr_psvr2): SteamVR owns the display. The previous plugin
# (steamvr_drv, driver_monado) was removed on 2026-11-18.
PLUGIN="$BUILD/steamvr-monado-psvr2"
DRIVER=driver_monado_psvr2
# Defaults: SteamVR's own async reprojection setting (ASYNC=1 asks the driver to turn it on for the session, as in the
# 2026-11-18 17:44 run; off by default since 2026-11-21) and the SteamVRLinuxFixes layer off. The driver applies them
# itself however SteamVR is started (ovrn_provider.cpp applyCompositorDefaults); ASYNC=1 / LINUX_FIXES=1 reach it as
# OVRN_ASYNC=1 / OVRN_LINUX_FIXES=1.
export ASYNC="${ASYNC:-0}"
LINUX_FIXES="${LINUX_FIXES:-0}"
[ "$ASYNC" = 1 ] && export OVRN_ASYNC=1
[ "$LINUX_FIXES" = 1 ] && export OVRN_LINUX_FIXES=1
VRPATH="$HOME/.config/openvr/openvrpaths.vrpath"
BACKUP="$VRPATH.psvr2-test-backup"
STEAM_ROOT="${STEAM_ROOT:-$HOME/.local/share/Steam}"
STEAMVR="${STEAMVR_DIR:-$STEAM_ROOT/steamapps/common/SteamVR}"
STEAM_LOGS="$STEAM_ROOT/logs"
LOGDIR="$HERE/logs"
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="$LOGDIR/steamvr-$STAMP.log"

restore_config() {
	if [ -f "$BACKUP" ]; then
		mv -f "$BACKUP" "$VRPATH"
		echo "Restored original OpenVR config ($VRPATH)."
	fi
	async_restore
}

# ASYNC=1 writes enableLinuxVulkanAsync into SteamVR's settings for this session only: take it out again when SteamVR
# has closed, so other drivers (Sony's) keep SteamVR's default.
ASYNC_MARK="$HOME/.config/openvr/psvr2-test-async-set"
async_restore() {
	[ -f "$ASYNC_MARK" ] || return 0
	pgrep -x vrserver >/dev/null && return 0 # SteamVR still running
	python3 -I - "$(cat "$ASYNC_MARK")" <<'PYEOF'
import json, sys
path = sys.argv[1]
try:
    with open(path) as f:
        cfg = json.load(f)
except (OSError, ValueError):
    sys.exit(0)
if cfg.get("steamvr", {}).pop("enableLinuxVulkanAsync", None) is not None:
    with open(path, "w") as f:
        json.dump(cfg, f, indent=3)
    print("Async reprojection setting removed from SteamVR's settings again (SteamVR default).")
PYEOF
	rm -f "$ASYNC_MARK"
}

# When SteamVR is closed, disconnect the Sense controllers over Bluetooth so they don't stay on (like Sony's
# controllers do when the host drops them, they then power down). CONTROLLERS_OFF=0 leaves them connected.
controllers_off() {
	[ "${CONTROLLERS_OFF:-1}" = 1 ] || return 0
	command -v bluetoothctl >/dev/null || return 0
	pgrep -x vrserver >/dev/null && return 0 # SteamVR still running (e.g. --restore while it runs)
	bluetoothctl devices 2>/dev/null | grep 'PlayStation VR2 Sense' | while read -r _ mac name; do
		if bluetoothctl info "$mac" 2>/dev/null | grep -q 'Connected: yes'; then
			if bluetoothctl disconnect "$mac" >/dev/null 2>&1; then
				echo "Disconnected $name ($mac)."
			else
				echo "Couldn't disconnect $name ($mac)."
			fi
		fi
	done
}

if [ "${1-}" = "--restore" ]; then
	restore_config
	exit 0
fi

echo "== Pre-flight"
fail() { echo "  [!!] $*"; exit 1; }
[ -f "$PLUGIN/driver.vrdrivermanifest" ] || fail "plugin not built: $PLUGIN (build with XRT_FEATURE_STEAMVR_PLUGIN=ON)"
[ -f "$PLUGIN/bin/linux64/$DRIVER.so" ] || fail "$DRIVER.so missing in $PLUGIN"
[ -x "$STEAMVR/bin/vrstartup.sh" ] || fail "SteamVR not found at $STEAMVR (set STEAMVR_DIR)"
[ -f "$VRPATH" ] || fail "no OpenVR config at $VRPATH: run SteamVR once normally first"
command -v python3 >/dev/null || fail "python3 is needed to edit the OpenVR config"
if pgrep -x vrserver >/dev/null || pgrep -x vrmonitor >/dev/null; then
	fail "SteamVR is already running, close it first"
fi
if pgrep -x monado-service >/dev/null; then
	fail "monado-service is running; the SteamVR plugin runs Monado itself, stop it first"
fi
if pgrep -f 'ignition_server.exe' >/dev/null; then
	# Left over from Sony's driver (Ignition route); it holds the headset, so our driver can't open it.
	echo "  [!!] Ignition server (Sony driver helper) is running and holds the headset."
	if [ -t 0 ]; then
		read -r -p "       Stop it now? It restarts the next time you use Sony's driver. [y/N] " a
		[ "${a,,}" = y ] || fail "stop it first: pkill -f ignition_server.exe"
		pkill -f 'ignition_server.exe'
		sleep 2
	else
		fail "stop it first: pkill -f ignition_server.exe"
	fi
fi
missing=$(ldd "$PLUGIN/bin/linux64/$DRIVER.so" | grep -c 'not found')
[ "$missing" = 0 ] || fail "$DRIVER.so has $missing unresolved libraries (ldd)"
echo "  [ok] plugin: $PLUGIN"
echo "  [ok] SteamVR: $STEAMVR"
lsusb | grep -qi '054c:0cde' && echo "  [ok] PS VR2 headset on USB" || echo "  [??] PS VR2 headset not seen on USB (054c:0cde)"
pgrep -x steam >/dev/null && echo "  [ok] Steam is running" || echo "  [??] Steam isn't running; start it first if SteamVR complains"
# The Sense controllers reach the driver over Bluetooth only: say which are paired and connected (a user's 2026-11-23
# runs had none connected and the logs didn't show it). Only a warning: one turned on after the start is picked up.
if command -v bluetoothctl >/dev/null; then
	sense=$(bluetoothctl devices 2>/dev/null | grep 'PlayStation VR2 Sense' || true)
	if [ -z "$sense" ]; then
		echo "  [??] no PS VR2 Sense controller paired over Bluetooth: pair each once (hold Create + PS until the light flashes, then pair it in the Bluetooth settings)"
	else
		while read -r _ mac name; do
			if bluetoothctl info "$mac" 2>/dev/null | grep -q 'Connected: yes'; then
				echo "  [ok] $name ($mac) connected over Bluetooth"
			else
				echo "  [??] $name ($mac) paired but not connected: press its PS button"
			fi
		done <<< "$sense"
	fi
else
	echo "  [??] bluetoothctl not found: can't check the Sense controllers' Bluetooth connection"
fi
# udev rules: without them only root can open the headset and the controllers, and Monado finds no controllers without
# saying why (a user's 2026-11-23 PC). install-deps.sh installs xr-hardware's PS VR2 entries when none cover them.
rules=""
for f in /etc/udev/rules.d/*.rules /run/udev/rules.d/*.rules /usr/lib/udev/rules.d/*.rules /lib/udev/rules.d/*.rules; do
	[ -r "$f" ] || continue
	if grep -qi '054c' "$f" && grep -qi '0cde' "$f" && grep -qi '0e45' "$f" && grep -qi '0e46' "$f"; then
		rules="$f"
		break
	fi
done
if [ -n "$rules" ]; then
	echo "  [ok] udev rules for the PS VR2 and Sense controllers: $rules"
else
	echo "  [!!] no udev rules for the PS VR2 and Sense controllers: the controllers won't be found. Run DriverKit's"
	echo "       Install dependencies (Compile Driver tab), or ./install-deps.sh, then turn the controllers off and on"
fi
# SteamVR's high-priority GPU queue (CAP_SYS_NICE on vrcompositor-launcher, SteamVR's own setup step): without it the
# compositor has no async reprojection, so a game's missed frames show as the screen dropping to SteamVR's fallback
# and back, a bright/dark flicker (a user's Arc A750, 2026-11-23: 1134 timeouts in 2 minutes, 0 reprojected).
# Only reported: DriverKit's Allow high priority button (Compile Driver tab) grants it when the user chooses to.
LAUNCHER="$STEAMVR/bin/linux64/vrcompositor-launcher"
if command -v getcap >/dev/null && [ -f "$LAUNCHER" ]; then
	if getcap "$LAUNCHER" 2>/dev/null | grep -qi cap_sys_nice; then
		echo "  [ok] SteamVR may use a high-priority GPU queue (async reprojection)"
	else
		echo "  [??] SteamVR may not use a high-priority GPU queue: no reprojection, so a game's missed frames flicker"
		echo "       (SteamVR's setup step; DriverKit: Compile Driver tab, SteamVR features, Allow high priority)"
	fi
fi
# WayVR (https://github.com/wayvr-org/wayvr, GPL-3.0): a separate desktop overlay program the user installs; we only
# start it. WAYVR=1 (DriverKit Settings: Start WayVR with SteamVR) runs `wayvr --openvr --wait` as the user, in this
# graphical session, before SteamVR, as WayVR's README asks (started by SteamVR, inside Steam's runtime, its screens
# don't work), and stops it after SteamVR. The Sense controllers' WayVR bindings come with our plugin.
WAYVR="${WAYVR:-0}"
WAYVR_BIN=""
find_wayvr() {
	if command -v wayvr >/dev/null; then
		command -v wayvr
		return
	fi
	local f
	for f in "$HOME"/Applications/*.AppImage "$HOME"/AppImages/*.AppImage "$HOME"/.local/bin/*.AppImage; do
		case "${f##*/}" in [Ww]ay[Vv][Rr]*) [ -x "$f" ] && { echo "$f"; return; } ;; esac
	done
}
# WayVR's own single-instance pid file ($XDG_RUNTIME_DIR/wayvr.pid): the pid when that WayVR still runs.
wayvr_running_pid() {
	local f="${XDG_RUNTIME_DIR:-/tmp}/wayvr.pid" p
	[ -r "$f" ] || return 1
	p=$(tr -dc 0-9 <"$f")
	[ -n "$p" ] && kill -0 "$p" 2>/dev/null && echo "$p"
}
WAYVR_BIN=$(find_wayvr)
if [ "$WAYVR" = 1 ]; then
	if [ -z "$WAYVR_BIN" ]; then
		echo "  [!!] WayVR (desktop in VR) is on but not installed: Arch: the AUR package wayvr; others: the AppImage from"
		echo "       https://github.com/wayvr-org/wayvr/releases in ~/Applications. SteamVR starts without it."
		WAYVR=0
	else
		echo "  [ok] WayVR: $WAYVR_BIN"
		# Its mouse and keyboard go through /dev/uinput; without access it runs, but clicks and typing do nothing.
		if [ -w /dev/uinput ]; then
			echo "  [ok] WayVR may send mouse and keyboard input (/dev/uinput)"
		else
			echo "  [??] WayVR can't send mouse or keyboard input: no write access to /dev/uinput (WayVR asks for the"
			echo "       input group: sudo usermod -aG input $USER, then log out and in)"
		fi
		if [ -n "${WAYLAND_DISPLAY:-}" ]; then
			# Wayland: screens come through the desktop's screen-cast portal and PipeWire (WayVR picks its own method).
			if command -v busctl >/dev/null; then
				if v=$(busctl --user get-property org.freedesktop.portal.Desktop /org/freedesktop/portal/desktop \
					org.freedesktop.portal.ScreenCast version 2>/dev/null); then
					echo "  [ok] screen-cast portal (version ${v#u }): WayVR asks which screens to show the first time"
				else
					echo "  [??] no screen-cast portal answers (xdg-desktop-portal and your desktop's portal backend):"
					echo "       WayVR can't show your screens"
				fi
			fi
			if [ -S "${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/pipewire-0" ]; then
				echo "  [ok] PipeWire running"
			else
				echo "  [??] PipeWire isn't running: WayVR can't show your screens"
			fi
		elif [ -n "${DISPLAY:-}" ]; then
			echo "  [ok] X11 desktop ($DISPLAY): WayVR's X11 capture (slower, no HiDPI scaling; restart it after display changes)"
		else
			echo "  [??] no Wayland or X11 desktop in this environment: WayVR can't show your screens"
		fi
		if p=$(wayvr_running_pid); then
			echo "  [ok] WayVR already running (pid $p): left as it is, not started again or stopped"
		fi
	fi
elif [ -n "$WAYVR_BIN" ]; then
	echo "  [--] WayVR installed ($WAYVR_BIN); DriverKit Settings: Start WayVR with SteamVR to start it with each session"
fi
if [ -f "$BACKUP" ]; then
	echo "  [!!] a backup from a previous run exists; restoring it before continuing"
	restore_config
fi

echo
echo "== Registering our plugin (Sony's driver disabled for this session)"
cp -f "$VRPATH" "$BACKUP"
trap 'restore_config; controllers_off; declare -F wayvr_stop >/dev/null && wayvr_stop' EXIT
trap 'echo; echo "Interrupted, stopping SteamVR..."; pkill -INT -x vrmonitor; pkill -INT -x vrserver; sleep 3' INT TERM

python3 -I - "$BACKUP" "$VRPATH" "$PLUGIN" <<'EOF'
import json, sys
src, dst, plugin = sys.argv[1:4]
with open(src) as f:
    cfg = json.load(f)
old = cfg.get("external_drivers") or []
cfg["external_drivers"] = [plugin]
with open(dst, "w") as f:
    json.dump(cfg, f, indent="\t")
for d in old:
    if d != plugin:
        print(f"  disabled for this session: {d}")
print(f"  registered: {plugin}")
EOF

mkdir -p "$LOGDIR"

# SteamVR blocks drivers after a crash ("safe mode"), e.g. after Sony's driver times out. A blocked monado driver
# is silently skipped, so clear that flag for it (SteamVR is closed at this point, so the edit sticks).
VRSETTINGS="$STEAM_ROOT/config/steamvr.vrsettings"
if [ -f "$VRSETTINGS" ]; then
	cp -f "$VRSETTINGS" "$LOGDIR/steamvr.vrsettings.backup-$STAMP"
	python3 -I - "$VRSETTINGS" "$DRIVER" <<'EOF'
import json, sys
path = sys.argv[1]
with open(path) as f:
    cfg = json.load(f)
drv = cfg.setdefault(sys.argv[2], {})
if drv.get("blocked_by_safe_mode"):
    print("  unblocked " + sys.argv[2] + " (was blocked by a previous SteamVR safe mode event)")
drv["blocked_by_safe_mode"] = False
drv["enable"] = True
# Async reprojection: SteamVR's own default on NVIDIA under Linux is off ("Async: Automatically disabled as default
# based on the detected Vulkan environment" in vrcompositor.txt, RTX 5080). With ASYNC=1 (OVRN_ASYNC=1) the driver asks
# SteamVR for it itself (IVRSettings, ovrn_provider.cpp applyCompositorDefaults) and takes it out again at shutdown.
import os
print("  async reprojection: " + ("asked for by the driver (ASYNC=1)" if os.environ.get("ASYNC", "0") == "1"
                                  else "SteamVR's own setting (ASYNC=0, the default)"))
with open(path, "w") as f:
    json.dump(cfg, f, indent=3)
EOF
fi

# Sense controller render models: link Sony's from the PlayStation VR2 App install into our plugin's resources
# (linked, not copied or committed, they're Sony's files). Without them, SteamVR's Oculus Touch models stand in.
SONY_MODELS="$STEAM_ROOT/steamapps/common/PlayStation VR2 App/SteamVR_Plug-In/resources/rendermodels"
[ -d "$SONY_MODELS" ] || SONY_MODELS="$HOME/.steam/steam/steamapps/common/PlayStation VR2 App/SteamVR_Plug-In/resources/rendermodels"
if [ -d "$SONY_MODELS/playstation_vr2_sense_left" ] && [ -d "$SONY_MODELS/playstation_vr2_sense_right" ]; then
	mkdir -p "$PLUGIN/resources/rendermodels"
	for side in left right; do
		# A compiled install has its own copy (build-driver.sh): keep it.
		[ -d "$PLUGIN/resources/rendermodels/playstation_vr2_sense_$side" ] && [ ! -L "$PLUGIN/resources/rendermodels/playstation_vr2_sense_$side" ] && continue
		ln -sfn "$SONY_MODELS/playstation_vr2_sense_$side" "$PLUGIN/resources/rendermodels/playstation_vr2_sense_$side"
	done
	export OVRD_PSSENSE_RENDER_MODELS=1
	echo "  linked PS VR2 Sense render models from the PlayStation VR2 App"
fi
# Sony's PS VR2 icons (status icons, binding screen pictures), linked the same way.
SONY_ICONS="$(dirname "$SONY_MODELS")/icons"
if [ -f "$SONY_ICONS/playstation_vr2_sense_left.svg" ]; then
	mkdir -p "$PLUGIN/resources/icons"
	if [ ! -d "$PLUGIN/resources/icons/playstation_vr2" ] || [ -L "$PLUGIN/resources/icons/playstation_vr2" ]; then
		ln -sfn "$SONY_ICONS" "$PLUGIN/resources/icons/playstation_vr2"
	fi
	# The driver then gives SteamVR the PS VR2 headset and Sense controller status icons (ovrn_backend_monado.cpp);
	# without it SteamVR shows generic devices.
	export OVRD_PSVR2_SONY_ICONS=1
	echo "  linked PS VR2 icons from the PlayStation VR2 App"
fi

# Room calibration (see ROOM.md): the driver keeps SteamVR's universe 3 for an unanchored session (a 2 m square around
# where you started) and switches to universe 2, your saved room, after a floor capture (left controller on the floor,
# crouch next to it, hold Create 2.5 s). Draw the room in SteamVR's Room Setup once you're in universe 2.
#   ROOM_SETUP=1   open SteamVR's Room Setup as soon as SteamVR is up (tools/vrlaunch, through SteamVR). Before you
#                  accept the traced room, do our floor capture if the room isn't anchored yet (left controller on its
#                  floor mark, crouch, hold Create until the short buzz): it ties the room to that spot so it comes
#                  back in place after a restart (SteamVR's own floor step only sets the floor height). No temporary
#                  square in the session universe meanwhile.
#   ROOM_FORGET=1  delete the saved room anchor before starting (to set the room up again)
#   OVRD_ROOM=0    the previous behaviour: a fixed 2 m play area handed to SteamVR, nothing saved
#   PLAY_AREA=<m>  side of the square play area the driver gives SteamVR (default 2). The saved room gets
#                  it too, so SteamVR's Room Setup isn't needed (a room drawn there is kept).
#   SCALE=<%>      per-eye render size as a percentage of the panel's eye size (default 140 = 2800x2856 as Monado renders for OpenXR; 100 = the panel's 2000x2040; 170 = Sony's 3400x3468)
[ -n "${PLAY_AREA:-}" ] && export OVRD_PLAY_AREA_SIZE="$PLAY_AREA" && echo "  PLAY_AREA=$PLAY_AREA m"
[ -n "${SCALE:-}" ] && export XRT_COMPOSITOR_SCALE_PERCENTAGE="$SCALE" && echo "  SCALE=$SCALE%: per-eye render size"
#   SONY_PROPS=<groups>  report Sony's display properties: "all", or a comma list of throttle, framerate,
#                  syslayer, texindex, supersample, dsc, edid, display, mask (default: none)
[ -n "${SONY_PROPS:-}" ] && export OVRN_SONY_PROPS="$SONY_PROPS" && echo "  SONY_PROPS=$SONY_PROPS: Sony's display properties reported"
if [ "${ROOM_SETUP:-0}" = 1 ]; then
	export OVRD_PLAY_AREA_SIZE=0
	echo "  ROOM_SETUP=1: Room Setup opens once SteamVR is up"
fi
if [ "${ROOM_FORGET:-0}" = 1 ]; then
	rm -f "$HOME/.config/monado/ovrd_room.json" && echo "  ROOM_FORGET=1: deleted the saved room anchor"
fi
# RECORD_SLAM=1: record every raw SLAM record from the headset (for finding its tracking-lost state)
if [ "${RECORD_SLAM:-0}" = 1 ]; then
	export PSVR2_RECORD_SLAM_RAW="$LOGDIR/slamraw-$STAMP.pv2c"
	echo "  RECORD_SLAM=1: raw SLAM records to $PSVR2_RECORD_SLAM_RAW"
fi
# REFRESH=90 (default) or REFRESH=120: the PS VR2 panel's refresh rate (PSVR2_REFRESH_HZ 89.91 / 119.88). SteamVR and
# Monado direct both pick the display mode to match. XRT_COMPOSITOR_DESIRED_MODE=<index> still forces a mode in Monado.
case "${REFRESH:-90}" in
90) export PSVR2_REFRESH_HZ="${PSVR2_REFRESH_HZ:-89.91}" ;;
120) export PSVR2_REFRESH_HZ="${PSVR2_REFRESH_HZ:-119.88}" ;;
*) echo "  [!!] REFRESH=${REFRESH} not supported (90 or 120), using 90"; export PSVR2_REFRESH_HZ=89.91 ;;
esac
echo "  refresh: ${PSVR2_REFRESH_HZ} Hz (REFRESH=90|120)"
# BASALT=1: Basalt visual-inertial tracking as the referee of the headset's own position (PSVR2_BASALT, see
# psvr2_basalt.h): catches the headset's position jumps in fast turns. Turns on camera mode 0x10 and accelerometer
# fusion. Needs the local Basalt build (../basalt-install, or BASALT_LIB=/path/libbasalt.so).
if [ "${BASALT:-0}" = 1 ]; then
	export PSVR2_BASALT=1
	export VIT_SYSTEM_LIBRARY_PATH="${BASALT_LIB:-$HERE/../basalt-install/lib/libbasalt.so}"
	if [ -f "$VIT_SYSTEM_LIBRARY_PATH" ]; then
		echo "  BASALT=1: Basalt referee on ($VIT_SYSTEM_LIBRARY_PATH)"
	else
		echo "  [!!] BASALT=1 but $VIT_SYSTEM_LIBRARY_PATH is missing: the driver will run without the referee"
	fi
fi
# HAND_TRACKING=1: optical hand tracking from the bottom cameras (Mercury). Hands appear as Index-style controllers
# with skeletons when the Sense controllers are off (turn them off first); pinch = trigger. Right hand only so far.
if [ "${HAND_TRACKING:-0}" = 1 ]; then
	export PSVR2_HAND_TRACKING=1
	echo "  HAND_TRACKING=1: bottom camera hand tracking on (Sense controllers off to use hands)"
fi
# RECORD_CAMERA=1: record the bottom camera pair continuously (30/s, ~30 MB/s) for the offline hand tracking prototype
# (hands/hand_replay). RECORD_CAMERA_HZ changes the rate.
if [ "${RECORD_CAMERA:-0}" = 1 ]; then
	export PSVR2_RECORD_CAMERA="$LOGDIR/camera-$STAMP.pv2c"
	export PSVR2_RECORD_CAMERA_HZ="${RECORD_CAMERA_HZ:-30}"
	echo "  RECORD_CAMERA=1: camera transfers to $PSVR2_RECORD_CAMERA at up to ${PSVR2_RECORD_CAMERA_HZ}/s"
fi
# RECORD_IMAGES=1: save a pair of bottom camera images every 2 s (to judge a room relocalizer, ~1 MB each).
if [ "${RECORD_IMAGES:-0}" = 1 ]; then
	export PSVR2_RECORD_IMAGES="$LOGDIR/images-$STAMP"
	mkdir -p "$PSVR2_RECORD_IMAGES"
	echo "  RECORD_IMAGES=1: camera images to $PSVR2_RECORD_IMAGES"
fi

# Inside Steam's container (the normal way, what Steam games need) when the plugin carries its libraries; otherwise,
# or with HOST_STEAMVR=1, on the host as before.
# Default: on the host (2026-11-19: inside the container SteamVR started slowly, didn't find the headset and reported
# "Shared IPC Compositor Init Failed"). IN_CONTAINER=1 tries the container start with the build-steamvr plugin.
if [ "${IN_CONTAINER:-0}" = 1 ] && [ "${HOST_STEAMVR:-0}" != 1 ] && [ -f "$PLUGIN/bin/linux64/libceres.so.4" ]; then
	IN_CONTAINER=1
	unset VALVE_SKIP_RUNTIME_SAFETY
	echo "  SteamVR inside Steam's runtime container (Steam games can connect); plugin $PLUGIN"
else
	IN_CONTAINER=0
	export VALVE_SKIP_RUNTIME_SAFETY=1
	echo "  SteamVR on the host (IN_CONTAINER=1: inside Steam's container, experimental)"
fi
# Log level, the same default as launch-openxr.sh (info). LOG_LEVEL=debug: the full debug logs (controller tracking,
# sensor fusion with a solver report per solve: ~12000 reports / 900k lines a minute in vrserver, 2026-11-18 audit).
LOG_LEVEL="${LOG_LEVEL:-info}"
export XRT_LOG="$LOG_LEVEL"
export PSVR2_LOG="$LOG_LEVEL"
export PSSENSE_LOG="$LOG_LEVEL"
export CONSTELLATION_TRACKER_LOG="$LOG_LEVEL"
echo "  log level: $LOG_LEVEL (LOG_LEVEL=debug for the full debug logs)"
# RECORD_DATASET=1 ./launch-steamvr.sh: record the constellation tracker's input (blobs, camera poses, IMU) for
# offline replay of tracker changes (large files).
if [ "${RECORD_DATASET:-0}" = "1" ]; then
	export CONSTELLATION_TRACKER_DATA_RECORDER_OUTPUT="$LOGDIR/dataset-$(date +%Y%m%d-%H%M%S)"
	echo "Recording the tracker dataset to $CONSTELLATION_TRACKER_DATA_RECORDER_OUTPUT"
fi
export T_LED_SYNC_REFINEMENT_LOG="$LOG_LEVEL"
export T_CONSTELLATION_SENSOR_FUSION_LOG=${FUSION_LOG:-$LOG_LEVEL}
# Head pose: record raw IMU/SLAM for offline replay, and allow A/B against the old behaviour with
# POSE_FILTER=0 ./script.sh
export PSVR2_POSE_FILTER="${POSE_FILTER:-1}"
# TRACKING=0: no measurement at all, to rule it out: no headset/controller recordings, no direct-mode or compositor
# frame logs, no SteamVR-side tools, and the game starts without the frame-timing layer.
if [ "${TRACKING:-0}" = 1 ]; then
	export PSVR2_RECORD_FILE="$LOGDIR/record-$(date +%Y%m%d-%H%M%S)-filter${PSVR2_POSE_FILTER}.csv"
	# The Sense controllers' IMU (gyro, accelerometer, orientation), one CSV per controller, for offline analysis.
	export PSSENSE_RECORD_FILE="$LOGDIR/sense-$STAMP"
else
	unset PSVR2_RECORD_FILE PSSENSE_RECORD_FILE
	echo "  TRACKING=0: all recordings and frame logs off"
	# POSELOG=1 (below) needs the headset's own recording (raw gyro), and only that one.
	if [ "${POSELOG:-0}" = 1 ]; then
		export PSVR2_RECORD_FILE="$LOGDIR/record-$(date +%Y%m%d-%H%M%S)-filter${PSVR2_POSE_FILTER}.csv"
		echo "  POSELOG=1: headset recording to $PSVR2_RECORD_FILE"
	fi
fi
# JITTER_CHECK=1: record what tells tracking jitter from compositor judder, and analyse it when SteamVR exits:
#   - SteamVR's own head pose 1000 times a second (tools/vrframetiming, steamvr-vrposes-<stamp>.csv)
#   - SteamVR's per-frame timing and the pose the game rendered each frame with (tools/vrframetiming, 30 s summaries)
#   - analysis/steamvr_timeline.py on both: jitter per 30 s, each event labelled tracking / frame timing / prediction
# SLAM corrections are brought in at a smoothly ramped rate that the reported velocity includes (driver default,
# PSVR2_SMOOTH_CORRECTION_MS=30): offline the velocity matched the pose's motion 4-5x better and SteamVR's prediction
# landed closer, with the pose itself no jerkier. SMOOTH_CORRECTION=0 turns it off (the earlier blend).
if [ "${SMOOTH_CORRECTION:-1}" = 0 ]; then
	export PSVR2_SMOOTH_CORRECTION_MS=0
	echo "  SMOOTH_CORRECTION=0: earlier correction blend (velocity without the correction's motion)"
fi
# LINUX_FIXES=0: turn off the VK_LAYER_BNUUY_steamvr_linux_fixes Vulkan layer (installed with Ignition, implicit, so it
# loads into vrcompositor in every session). Since a SteamVR update it only half-applies: it switches the compositor's
# present mode to FIFO_LATEST_READY but can't find WaitForPendingPresent to patch ("WaitForPendingPresent target
# function not resolved" in Steam/logs/vrcompositor-linux.txt, every session since 2026-11-04).
# Off by default (LINUX_FIXES=1 turns it on): on SteamVR 2.19.3 its WaitForPendingPresent patch
# finds nothing, but it still replaces vrcompositor's GPU event queries using struct offsets from an older SteamVR
# build (vrcompositor_patches.cpp), and the user saw clearly more judder with it (2026-11-18 17:44 vs 17:46).
if [ "${LINUX_FIXES:-1}" = 0 ]; then
	export DISABLE_STEAMVR_LINUX_FIXES=1
	echo "  LINUX_FIXES=0: SteamVR Linux fixes layer off (compositor runs SteamVR's own present path)"
fi
# Tracking (off by default, TRACKING=1 turns it on): SteamVR's per-frame timing with the pose the game rendered each frame
# with, and SteamVR's own head pose 1000 times a second (tools/vrframetiming: steamvr-frames-<stamp>.csv,
# steamvr-vrposes-<stamp>.csv). JITTER_CHECK=1 adds SteamVR's pose prediction probe and per-thread CPU sampling.
if [ "${TRACKING:-0}" = 1 ] || [ "${JITTER_CHECK:-0}" = 1 ]; then
	FRAMES_CSV="$LOGDIR/steamvr-frames-$STAMP.csv"
	if [ -x "$HERE/tools/vrframetiming" ]; then
		echo "  tracking: frame timing to $FRAMES_CSV"
	else
		echo "  [!!] tracking: tools/vrframetiming isn't built (see its Build: line)"
	fi
fi

# WayVR supervision: started once SteamVR's compositor runs (wayvr_start below), restarted after 5, 15 and 60 s if it
# dies while SteamVR runs (then left off, with the reason in its log), stopped after SteamVR exits. Not before SteamVR,
# as WayVR's README puts it for its own --wait: with --openvr it connects at once, and OpenVR then starts SteamVR as
# WayVR's child (2026-11-23 22:23: connected 1.5 s after starting, SteamVR's log in WayVR's output). What WayVR's
# README warns against is SteamVR starting WayVR (inside Steam's runtime); here this script starts it, outside it.
# The AppImage variables of DriverKit's own AppImage are taken out: WayVR reads APPIMAGE as its own path.
WAYVR_SUP_PID=""
WAYVR_STOP_FLAG=""
wayvr_supervise() {
	local delays=(5 15 60) n=0 pid rc
	while :; do
		env -u APPIMAGE -u APPDIR -u ARGV0 -u OWD -u PSVR2_DRIVERKIT_FROM_APPIMAGE \
			"$WAYVR_BIN" --openvr --wait --log-to "$WAYVR_LOG" >>"$WAYVR_LOG.out" 2>&1 &
		pid=$!
		echo "$pid" >"$WAYVR_STOP_FLAG.pid"
		wait "$pid"
		rc=$?
		[ -e "$WAYVR_STOP_FLAG" ] && return 0
		if ! pgrep -x vrserver >/dev/null; then
			echo "  WayVR exited (code $rc) as SteamVR ended"
			return 0
		fi
		if [ "$n" -ge ${#delays[@]} ]; then
			echo "  [!!] WayVR stopped (code $rc) for the 4th time; not restarted. Its log: $WAYVR_LOG"
			return 1
		fi
		echo "  [??] WayVR stopped unexpectedly (code $rc); restarting in ${delays[$n]} s (log: $WAYVR_LOG)"
		sleep "${delays[$n]}"
		[ -e "$WAYVR_STOP_FLAG" ] && return 0
		n=$((n + 1))
	done
}
wayvr_stop() {
	[ -n "$WAYVR_SUP_PID" ] || return 0
	: >"$WAYVR_STOP_FLAG"
	local pid i
	pid=$(cat "$WAYVR_STOP_FLAG.pid" 2>/dev/null)
	if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
		kill -TERM "$pid" 2>/dev/null
		for i in $(seq 1 50); do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
		if kill -0 "$pid" 2>/dev/null; then
			kill -KILL "$pid" 2>/dev/null
			echo "  WayVR didn't stop within 5 s; killed"
		else
			echo "  WayVR stopped"
		fi
	fi
	kill "$WAYVR_SUP_PID" 2>/dev/null
	wait "$WAYVR_SUP_PID" 2>/dev/null
	rm -f "$WAYVR_STOP_FLAG" "$WAYVR_STOP_FLAG.pid"
	WAYVR_SUP_PID=""
}
# Once SteamVR runs: wait (up to 60 s) for its compositor, then start WayVR, so it finds SteamVR instead of starting it.
wayvr_start() {
	[ "$WAYVR" = 1 ] || return 0
	if wayvr_running_pid >/dev/null; then
		return 0
	fi
	local i
	for i in $(seq 1 60); do
		pgrep -x vrcompositor >/dev/null && break
		pgrep -x vrserver >/dev/null || return 0
		sleep 1
	done
	if ! pgrep -x vrcompositor >/dev/null; then
		echo "  [??] SteamVR's compositor didn't start within 60 s: WayVR not started"
		return 0
	fi
	WAYVR_LOG="$LOGDIR/wayvr-$STAMP.log"
	WAYVR_STOP_FLAG="$LOGDIR/.wayvr-stop-$STAMP"
	rm -f "$WAYVR_STOP_FLAG"
	wayvr_supervise &
	WAYVR_SUP_PID=$!
	echo "  WayVR started (log: $WAYVR_LOG). Close SteamVR's dashboard (PS button), then double-press Triangle"
}

echo
echo "== Starting SteamVR (close SteamVR or press Ctrl+C here when done)"
echo "   Hold both Sense controllers in front of the headset after it shows up."
if [ "$IN_CONTAINER" = 1 ]; then
	"$STEAMVR/bin/vrstartup.sh" >"$LOG" 2>&1 &
else
	"$STEAMVR/bin/vrstartup.sh" --valve-skip-runtime-safety >"$LOG" 2>&1 &
fi

# Wait for vrserver to appear, then for it to exit.
for _ in $(seq 1 60); do
	pgrep -x vrserver >/dev/null && break
	sleep 1
done
if ! pgrep -x vrserver >/dev/null; then
	echo "  vrserver never started; see $LOG and $STEAM_LOGS/vrstartup.txt"
else
	echo "  vrserver running (pid $(pgrep -x vrserver | head -1))"
	wayvr_start
	FT_PID=""
	if [ -n "${FRAMES_CSV:-}" ] && [ -x "$HERE/tools/vrframetiming" ]; then
		"$HERE/tools/vrframetiming" 36000 "$FRAMES_CSV" "$LOGDIR/steamvr-vrposes-$STAMP.csv" \
			2>"$LOGDIR/steamvr-frames-$STAMP-summary.txt" &
		FT_PID=$!
	fi
	# POSELOG=1: what SteamVR hands a game as the head pose, once per refresh (tools/vrposelog, the tool behind the
	# 2026-11-16 comparison with Sony's driver; analysis/jitter/logjitter.py), with the headset's own recording.
	POSELOG_PID=""
	if [ "${POSELOG:-0}" = 1 ] && [ -x "$HERE/tools/vrposelog" ]; then
		( sleep 15; "$HERE/tools/vrposelog" 36000 "$LOGDIR/vrposelog-$STAMP.csv" 2>"$LOGDIR/vrposelog-$STAMP.txt" ) &
		POSELOG_PID=$!
		echo "  POSELOG=1: SteamVR's head pose per refresh to $LOGDIR/vrposelog-$STAMP.csv"
	fi
	PROBE_PID=""
	if [ "${JITTER_CHECK:-0}" = 1 ] && [ -x "$HERE/tools/vrposeprobe" ]; then
		# SteamVR's own pose prediction at fixed times, independent of any app (analysis/steamvr_probe.py).
		"$HERE/tools/vrposeprobe" 36000 "$LOGDIR/steamvr-probe-$STAMP.csv" 2>/dev/null &
		PROBE_PID=$!
	fi
	CPU_PID=""
	if [ "${JITTER_CHECK:-0}" = 1 ]; then
		# Per-thread CPU use and CPU wait of the game, vrserver (our driver), vrcompositor.
		python3 -I "$HERE/tools/cpusample.py" "$LOGDIR/steamvr-cpu-$STAMP.csv" ${GAME_PROCESS:-wwhd} vrserver vrcompositor \
			2>"$LOGDIR/steamvr-cpu-$STAMP-summary.txt" &
		CPU_PID=$!
	fi
	# GAME=/path/to/openxr-app: start it once the compositor is up, with the OpenXR frame-timing layer
	# (tools/xr_frame_log: each frame's predicted display time and period as SteamVR's OpenXR runtime gives them).
	GAME_PID=""
	if [ -n "${GAME:-}" ]; then
		for _ in $(seq 1 60); do pgrep -x vrcompositor >/dev/null && break; sleep 1; done
		sleep 5
		echo "  starting $GAME with the frame-timing layer (log: $LOGDIR/steamvr-$STAMP-game.txt)"
		# The game's frame-timing layer (call times and poses of each game frame, written by the game's process): with
		# TRACKING=1, and with FRAME_LOG=1 for analysis/sync_check.py. It was on in the smooth OpenXR sessions.
		if [ "${TRACKING:-0}" = 1 ] || [ "${FRAME_LOG:-0}" = 1 ]; then
			"$HERE/tools/xr_frame_log/run-with-frame-log.sh" "$GAME" >"$LOGDIR/steamvr-$STAMP-game.txt" 2>&1 &
		else
			"$GAME" >"$LOGDIR/steamvr-$STAMP-game.txt" 2>&1 &
		fi
		GAME_PID=$!
	fi
	# ROOM_SETUP=1: open Room Setup once the compositor is up and the devices have settled (2026-11-19: it waited for
	# the floor capture first, but the capture can be done inside Room Setup, before accepting).
	ROOM_SETUP_PID=""
	if [ "${ROOM_SETUP:-0}" = 1 ]; then
		if [ -x "$HERE/tools/vrlaunch" ]; then
			(
				from=$(stat -c %s "$STEAM_LOGS/vrserver.txt" 2>/dev/null || echo 0)
				for _ in $(seq 1 60); do pgrep -x vrcompositor >/dev/null && break; sleep 1; done
				sleep 5
				pgrep -x vrserver >/dev/null || exit 0
				echo "  ROOM_SETUP=1: opening Room Setup"
				"$HERE/tools/vrlaunch" openvr.tool.steamvr_room_setup 2>&1 | grep -v -i breakpad | sed 's/^/  /'
				if ! tail -c +$((from + 1)) "$STEAM_LOGS/vrserver.txt" 2>/dev/null | grep -q "(the saved room)"; then
					echo "  ROOM_SETUP=1: the room isn't anchored yet: before accepting, put the left controller on its"
					echo "               floor mark, crouch next to it and hold Create until the short buzz"
				fi
			) &
			ROOM_SETUP_PID=$!
		else
			echo "  ROOM_SETUP=1: tools/vrlaunch isn't built (see its Build: line); open Room Setup from SteamVR"
		fi
	fi
	while pgrep -x vrserver >/dev/null; do sleep 2; done
	[ -n "$ROOM_SETUP_PID" ] && kill "$ROOM_SETUP_PID" 2>/dev/null
	echo "  SteamVR exited."
	wayvr_stop
	if [ -n "$GAME_PID" ]; then
		wait "$GAME_PID" 2>/dev/null
		echo; echo "== The game's OpenXR frame timing on SteamVR"
		sed -n '/^[0-9]* frames over/,$p' "$LOGDIR/steamvr-$STAMP-game.txt"
	fi
	[ -n "${POSELOG_PID:-}" ] && pkill -x vrposelog 2>/dev/null
	for pid in $FT_PID $CPU_PID $PROBE_PID ${POSELOG_PID:-}; do
		kill -INT "$pid" 2>/dev/null
		wait "$pid" 2>/dev/null
	done
fi

echo
echo "== Collecting logs"
for f in vrserver.txt vrstartup.txt vrmonitor.txt vrcompositor.txt; do
	[ -f "$STEAM_LOGS/$f" ] && cp -f "$STEAM_LOGS/$f" "$LOGDIR/steamvr-$STAMP-$f"
done
# Monado's output can land in either the launcher log or vrserver.txt, summarize both together.
cat "$LOG" "$LOGDIR/steamvr-$STAMP-vrserver.txt" 2>/dev/null >"$LOGDIR/steamvr-$STAMP-combined.log"
echo "  logs in $LOGDIR/steamvr-$STAMP-*"
echo

"$HERE/summarize.sh" "$LOGDIR/steamvr-$STAMP-combined.log"

# The poses SteamVR itself reports (tools/vrframetiming), the driver's own pose log having gone with the old plugin.
POSES="$LOGDIR/steamvr-vrposes-$STAMP.csv"
if [ "${JITTER_CHECK:-0}" = 1 ] && [ -s "$POSES" ] && [ -s "${FRAMES_CSV:-}" ]; then
	echo
	echo "== Jitter check (tracking vs frame timing vs prediction)"
	cat "$LOGDIR/steamvr-frames-$STAMP-summary.txt" 2>/dev/null
	echo; echo "CPU per thread:"; cat "$LOGDIR/steamvr-cpu-$STAMP-summary.txt" 2>/dev/null
	python3 -I "$HERE/analysis/steamvr_timeline.py" "$POSES" "$FRAMES_CSV" | tee "$LOGDIR/steamvr-jitter-$STAMP.txt"
	echo; echo "== Frames drawn one refresh early or late (the pose each frame was drawn with)"
	python3 -I "$HERE/analysis/steamvr_frame_shift.py" "$POSES" "$FRAMES_CSV" | tee -a "$LOGDIR/steamvr-jitter-$STAMP.txt"
	if [ -s "$LOGDIR/steamvr-probe-$STAMP.csv" ]; then
		echo; echo "== SteamVR's own pose prediction (vrserver, no app involved)"
		python3 -I "$HERE/analysis/steamvr_probe.py" "$POSES" "$LOGDIR/steamvr-probe-$STAMP.csv" | tee -a "$LOGDIR/steamvr-jitter-$STAMP.txt"
	fi
fi
