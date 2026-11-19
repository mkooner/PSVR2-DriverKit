#!/usr/bin/env bash
# Launch SteamVR on our Monado build (psvr2-headset-constellation) for PS VR2 testing.
#
# Usage:
#   ./launch-steamvr.sh            launch, wait until SteamVR exits, restore config, summarize logs
#   ./launch-steamvr.sh --restore  only restore the OpenVR config (if a previous run was killed)
#
# What it does:
#   1. Backs up ~/.config/openvr/openvrpaths.vrpath and registers ONLY our build's SteamVR plugin
#      (build/steamvr-monado), so Sony's PlayStation VR2 App driver doesn't fight over the headset.
#   2. Starts SteamVR with VALVE_SKIP_RUNTIME_SAFETY=1 (Valve's own vrstartup.sh switch), so vrserver runs on
#      the host instead of inside the Steam Linux Runtime container. Our driver links host libraries
#      (Ceres, OpenCV, libusb, ...) that don't exist inside the container.
#   3. Enables the PS VR2 / Sense / constellation debug logs.
#   4. When SteamVR exits (or on Ctrl+C), restores the original config and copies + summarizes the logs.
#
# Monado runs inside SteamVR's vrserver with this plugin, so don't run monado-service at the same time.

set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
BUILD="${MONADO_BUILD:-$HERE/../trial/build}"
# PLUGIN_DIR=<build>/steamvr-monado-upstream: register another build of the plugin (e.g. Monado's upstream
# steamvr_drv, built against the same PS VR2 driver, for an A/B of the adapter). Default: ours.
PLUGIN="${PLUGIN_DIR:-$BUILD/steamvr-monado}"
DRIVER=driver_monado
# NATIVE=1 (the default): the rebuilt driver (trial/src/xrt/state_trackers/steamvr_psvr2, driver_monado_psvr2): SteamVR
# owns the display. The old plugin's options (DIRECT_MODE, OVRD_*) don't apply to it. NATIVE=0: the old plugin
# (steamvr_drv, driver_monado); DIRECT_MODE=1 or PLUGIN_DIR=... select it too unless NATIVE=1 is given.
if [ -z "${NATIVE:-}" ]; then
	if [ "${DIRECT_MODE:-0}" = 1 ] || [ -n "${PLUGIN_DIR:-}" ]; then
		NATIVE=0
	else
		NATIVE=1
	fi
fi
echo "  NATIVE=$NATIVE: $([ "$NATIVE" = 1 ] && echo "rebuilt driver (driver_monado_psvr2)" || echo "old plugin (driver_monado)")"
if [ "$NATIVE" = 1 ]; then
	PLUGIN="$BUILD/steamvr-monado-psvr2"
	DRIVER=driver_monado_psvr2
	[ "${DIRECT_MODE:-0}" = 1 ] && { echo "NATIVE=1 has no direct mode (SteamVR owns the display)"; exit 1; }
	# Defaults for the rebuilt driver, the user's best run (2026-11-18 17:44): SteamVR's async reprojection on and the
	# SteamVRLinuxFixes layer off. The driver applies them itself however SteamVR is started (ovrn_provider.cpp
	# applyCompositorDefaults); ASYNC=0 / LINUX_FIXES=1 here tell it not to (OVRN_ASYNC / OVRN_LINUX_FIXES).
	export ASYNC="${ASYNC:-1}"
	LINUX_FIXES="${LINUX_FIXES:-0}"
	[ "$ASYNC" = 0 ] && export OVRN_ASYNC=0
	[ "$LINUX_FIXES" = 1 ] && export OVRN_LINUX_FIXES=1
fi
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
if [ -f "$BACKUP" ]; then
	echo "  [!!] a backup from a previous run exists; restoring it before continuing"
	restore_config
fi

echo
echo "== Registering our plugin (Sony's driver disabled for this session)"
cp -f "$VRPATH" "$BACKUP"
trap 'restore_config; controllers_off' EXIT
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
	NATIVE="$NATIVE" ASYNC_MARK="$ASYNC_MARK" python3 -I - "$VRSETTINGS" "$DRIVER" <<'EOF'
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
# based on the detected Vulkan environment" in vrcompositor.txt, RTX 5080). With NATIVE=1 the driver asks SteamVR for
# it itself (IVRSettings, ovrn_provider.cpp applyCompositorDefaults) and takes it out again at shutdown; ASYNC=0
# reaches it as OVRN_ASYNC=0. For the old plugin (NATIVE=0), ASYNC=1 forces it here for this session, and
# async_restore takes the setting out again when SteamVR closes.
import os
sv = cfg.setdefault("steamvr", {})
if os.environ.get("NATIVE") == "1":
    print("  async reprojection: " + ("asked for by the driver" if os.environ.get("ASYNC", "1") == "1"
                                      else "SteamVR default (ASYNC=0)"))
elif os.environ.get("ASYNC", "0") == "1":
    sv["enableLinuxVulkanAsync"] = True
    print("  async reprojection forced on for this session (ASYNC=1; ASYNC=0 for SteamVR's default)")
    with open(os.environ["ASYNC_MARK"], "w") as f:
        f.write(path)
else:
    sv.pop("enableLinuxVulkanAsync", None)
    print("  async reprojection: SteamVR default (ASYNC=1 forces it on)")
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
		ln -sfn "$SONY_MODELS/playstation_vr2_sense_$side" "$PLUGIN/resources/rendermodels/playstation_vr2_sense_$side"
	done
	export OVRD_PSSENSE_RENDER_MODELS=1
	echo "  linked PS VR2 Sense render models from the PlayStation VR2 App"
fi
# Sony's PS VR2 icons (status icons, binding screen pictures), linked the same way.
SONY_ICONS="$(dirname "$SONY_MODELS")/icons"
if [ -f "$SONY_ICONS/playstation_vr2_sense_left.svg" ]; then
	mkdir -p "$PLUGIN/resources/icons"
	ln -sfn "$SONY_ICONS" "$PLUGIN/resources/icons/playstation_vr2"
	export OVRD_PSVR2_SONY_ICONS=1
	echo "  linked PS VR2 icons from the PlayStation VR2 App"
fi

# Room calibration (see ROOM.md): the driver keeps SteamVR's universe 3 for an unanchored session (a 2 m square around
# where you started) and switches to universe 2, your saved room, after a floor capture (left controller on the floor,
# crouch next to it, hold Create 2.5 s). Draw the room in SteamVR's Room Setup once you're in universe 2.
#   ROOM_SETUP=1   open SteamVR's Room Setup for you once the room is anchored: start SteamVR, do the floor capture
#                  (left controller on its floor mark, crouch, hold Create until the short buzz), and Room Setup opens
#                  (tools/vrlaunch, through SteamVR). No temporary square in the session universe meanwhile.
#   ROOM_FORGET=1  delete the saved room anchor before starting (to set the room up again)
#   OVRD_ROOM=0    the previous behaviour: a fixed 2 m play area handed to SteamVR, nothing saved
#   PLAY_AREA=<m>  side of the square play area the driver gives SteamVR (default 2). With NATIVE=1 the saved room gets
#                  it too, so SteamVR's Room Setup isn't needed (a room drawn there is kept).
#   SCALE=<%>      per-eye render size as a percentage of the panel's eye size (NATIVE=1, the default: 140 = 2800x2856 as Monado renders for OpenXR; 100 = the panel's 2000x2040; 170 = Sony's 3400x3468)
[ -n "${PLAY_AREA:-}" ] && export OVRD_PLAY_AREA_SIZE="$PLAY_AREA" && echo "  PLAY_AREA=$PLAY_AREA m"
[ -n "${SCALE:-}" ] && export XRT_COMPOSITOR_SCALE_PERCENTAGE="$SCALE" && echo "  SCALE=$SCALE%: per-eye render size"
#   SONY_PROPS=<groups>  (NATIVE=1, the default) report Sony's display properties: "all", or a comma list of throttle, framerate,
#                  syslayer, texindex, supersample, dsc, edid, display, mask (default: none)
[ -n "${SONY_PROPS:-}" ] && export OVRN_SONY_PROPS="$SONY_PROPS" && echo "  SONY_PROPS=$SONY_PROPS: Sony's display properties reported"
if [ "${ROOM_SETUP:-0}" = 1 ]; then
	export OVRD_PLAY_AREA_SIZE=0
	echo "  ROOM_SETUP=1: Room Setup opens after the floor capture (left controller on its mark, hold Create)"
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

export VALVE_SKIP_RUNTIME_SAFETY=1
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
	unset PSVR2_RECORD_FILE PSSENSE_RECORD_FILE OVRD_POSE_LOG OVRD_DIRECT_MODE_LOG XRT_COMPOSITOR_FRAME_LOG
	echo "  TRACKING=0: all recordings and frame logs off"
	# POSELOG=1 (below) needs the headset's own recording (raw gyro), and only that one.
	if [ "${POSELOG:-0}" = 1 ]; then
		export PSVR2_RECORD_FILE="$LOGDIR/record-$(date +%Y%m%d-%H%M%S)-filter${PSVR2_POSE_FILTER}.csv"
		echo "  POSELOG=1: headset recording to $PSVR2_RECORD_FILE"
	fi
fi
# JITTER_CHECK=1: record what tells tracking jitter from compositor judder, and analyse it when SteamVR exits:
#   - every headset pose the plugin hands SteamVR (OVRD_POSE_LOG, ~9 MB a minute)
#   - SteamVR's per-frame timing and the pose the game rendered each frame with (tools/vrframetiming, 30 s summaries)
#   - analysis/steamvr_timeline.py on both: jitter per 30 s, each event labelled tracking / frame timing / prediction
# SLAM corrections are brought in at a smoothly ramped rate that the reported velocity includes (driver default,
# PSVR2_SMOOTH_CORRECTION_MS=30): offline the velocity matched the pose's motion 4-5x better and SteamVR's prediction
# landed closer, with the pose itself no jerkier. SMOOTH_CORRECTION=0 turns it off (the earlier blend).
if [ "${SMOOTH_CORRECTION:-1}" = 0 ]; then
	export PSVR2_SMOOTH_CORRECTION_MS=0
	echo "  SMOOTH_CORRECTION=0: earlier correction blend (velocity without the correction's motion)"
fi
# ANGVEL_AVG=<ms>: send SteamVR the mean angular velocity of the last N ms (OVRD_ANGVEL_AVG_MS; offline slightly worse
# than the newest sample). Off by default.
if [ -n "${ANGVEL_AVG:-}" ] && [ "${ANGVEL_AVG}" != 0 ]; then
	export OVRD_ANGVEL_AVG_MS="$ANGVEL_AVG"
	echo "  ANGVEL_AVG=$ANGVEL_AVG: angular velocity averaged over $ANGVEL_AVG ms"
fi
# DIRECT_MODE=1: driver direct mode (OVRD_DIRECT_MODE, ovrd_direct_mode.hpp). SteamVR's compositor hands its finished
# eye images to our plugin instead of driving the headset's display; Monado's compositor, running inside vrserver,
# re-warps them to the latest head pose, distorts them and shows them on the headset, paced by its own frame timing
# (as Monado direct does for OpenXR apps). Every frame is logged (logs/direct-<stamp>.csv, 10 s summaries
# "Direct mode (10 s)" in the log). If Monado's compositor can't start, the frames are only logged and the headset
# stays dark. Off by default.
if [ "${DIRECT_MODE:-0}" = 1 ]; then
	export OVRD_DIRECT_MODE=1
	# TRACKING=0 (below) leaves out this log and the compositor's.
	[ "${TRACKING:-0}" = 1 ] && export OVRD_DIRECT_MODE_LOG="$LOGDIR/direct-$STAMP.csv"
	# CAPTURE=<s>: once, <s> seconds after the game's layers appear, save that frame's layer images
	# (logs/direct-<stamp>-layer<N>.rgba, analysis/layer_capture.py makes PNGs). Off by default: on 2026-11-18 11:25 it
	# froze the frames for 1.8 s and SteamVR then ran at 30 fps for 8 s.
	export OVRD_DIRECT_MODE_CAPTURE="${CAPTURE:-0}"
	# What reached the screen, per displayed frame (Monado's compositor: re-warp pose, client frame used, present
	# and vblank times): analysis/screen_jitter.py logs/compframes-<stamp>.csv logs/direct-<stamp>.csv
	[ "${TRACKING:-0}" = 1 ] && export XRT_COMPOSITOR_FRAME_LOG="$LOGDIR/compframes-$STAMP.csv"
	# FRAME_LOG=1: frame timing only (logs/frames-<stamp>.csv, analysis/frame_log.py), built not to disturb the frames:
	# live threads only copy ~60 bytes into memory, an idle-priority thread writes the file once a second, no pose
	# reads. Independent of TRACKING, so it can run with everything else off.
	# OVERLAY=0: only SteamVR's scene layer goes to Monado (its dashboard/overlay layer is left out).
	[ "${OVERLAY:-1}" = 0 ] && export OVRD_DIRECT_MODE_LAYERS=1 && echo "  OVERLAY=0: SteamVR's overlay layer off"
	[ "${FRAME_LOG:-0}" = 1 ] && export OVRD_FRAME_LOG="$LOGDIR/frames-$STAMP.csv" && echo "  FRAME_LOG=1: frame timing to $OVRD_FRAME_LOG"
	echo "  DIRECT_MODE=1: SteamVR's frames are shown by Monado's compositor (plugin direct mode)"
fi
# LINUX_FIXES=0: turn off the VK_LAYER_BNUUY_steamvr_linux_fixes Vulkan layer (installed with Ignition, implicit, so it
# loads into vrcompositor in every session). Since a SteamVR update it only half-applies: it switches the compositor's
# present mode to FIFO_LATEST_READY but can't find WaitForPendingPresent to patch ("WaitForPendingPresent target
# function not resolved" in Steam/logs/vrcompositor-linux.txt, every session since 2026-11-04).
# With NATIVE=1 it is off by default (LINUX_FIXES=1 turns it on): on SteamVR 2.19.3 its WaitForPendingPresent patch
# finds nothing, but it still replaces vrcompositor's GPU event queries using struct offsets from an older SteamVR
# build (vrcompositor_patches.cpp), and the user saw clearly more judder with it (2026-11-18 17:44 vs 17:46).
if [ "${LINUX_FIXES:-1}" = 0 ]; then
	export DISABLE_STEAMVR_LINUX_FIXES=1
	echo "  LINUX_FIXES=0: SteamVR Linux fixes layer off (compositor runs SteamVR's own present path)"
fi
# OVRD_POSE_MODE=steamvr|measured|novelocity|display and OVRD_POSE_REPLAY=logs/steamvr-pose-<stamp>.csv (a recorded
# head motion instead of the headset's) pass through to the plugin: one variable per run, same input with the replay.
[ -n "${OVRD_POSE_MODE:-}" ] && echo "  OVRD_POSE_MODE=$OVRD_POSE_MODE"
[ -n "${OVRD_POSE_REPLAY:-}" ] && echo "  OVRD_POSE_REPLAY=$OVRD_POSE_REPLAY (recorded head motion, not the headset's)"
# Tracking (off by default, TRACKING=1 turns it on): the same per-frame measurements as launch-openxr.sh, so every
# session can be compared with analysis/screen_jitter.py:
#   - every headset pose the plugin hands SteamVR (OVRD_POSE_LOG)
#   - SteamVR's per-frame timing with the pose the game rendered each frame with, and SteamVR's own head pose 1000 times
#     a second (tools/vrframetiming: steamvr-frames-<stamp>.csv, steamvr-vrposes-<stamp>.csv)
#   - with DIRECT_MODE=1 also Monado's compositor (compframes-<stamp>.csv, set above) and the plugin's direct log
# JITTER_CHECK=1 adds SteamVR's pose prediction probe and per-thread CPU sampling.
# Direct mode measures itself (compositor + plugin logs, the 11:25 setup that didn't change frame timing), so these
# SteamVR-side tools run only in normal mode unless JITTER_CHECK=1 asks for them.
if { [ "${TRACKING:-0}" = 1 ] && [ "${DIRECT_MODE:-0}" != 1 ]; } || [ "${JITTER_CHECK:-0}" = 1 ]; then
	export OVRD_POSE_LOG="$LOGDIR/steamvr-pose-$STAMP.csv"
	FRAMES_CSV="$LOGDIR/steamvr-frames-$STAMP.csv"
	if [ -x "$HERE/tools/vrframetiming" ]; then
		echo "  tracking: submitted poses to $OVRD_POSE_LOG, frame timing to $FRAMES_CSV"
	else
		echo "  [!!] tracking: tools/vrframetiming isn't built (see its Build: line); poses only"
	fi
fi

echo
echo "== Starting SteamVR (close SteamVR or press Ctrl+C here when done)"
echo "   Hold both Sense controllers in front of the headset after it shows up."
"$STEAMVR/bin/vrstartup.sh" --valve-skip-runtime-safety >"$LOG" 2>&1 &

# Wait for vrserver to appear, then for it to exit.
for _ in $(seq 1 60); do
	pgrep -x vrserver >/dev/null && break
	sleep 1
done
if ! pgrep -x vrserver >/dev/null; then
	echo "  vrserver never started; see $LOG and $STEAM_LOGS/vrstartup.txt"
else
	echo "  vrserver running (pid $(pgrep -x vrserver | head -1))"
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
	# ROOM_SETUP=1: wait for the driver to anchor the room (its "Room: SteamVR universe ... (the saved room)" line in
	# vrserver.txt, after the floor capture), then ask SteamVR to open Room Setup. Room Setup first would draw the room
	# in the session universe, which is gone at the next start.
	ROOM_SETUP_PID=""
	if [ "${ROOM_SETUP:-0}" = 1 ]; then
		if [ -x "$HERE/tools/vrlaunch" ]; then
			(
				for _ in $(seq 1 60); do pgrep -x vrcompositor >/dev/null && break; sleep 1; done
				echo "  ROOM_SETUP=1: waiting for the floor capture (left controller on its mark, crouch, hold Create)"
				# Poll the log's new part once a second (a tail -F pipeline kept running after the match, 2026-11-18).
				from=$(stat -c %s "$STEAM_LOGS/vrserver.txt" 2>/dev/null || echo 0)
				found=0
				for _ in $(seq 1 1200); do
					if tail -c +$((from + 1)) "$STEAM_LOGS/vrserver.txt" 2>/dev/null | grep -q "(the saved room)"; then
						found=1
						break
					fi
					pgrep -x vrserver >/dev/null || break
					sleep 1
				done
				if [ "$found" = 1 ]; then
					sleep 2
					echo "  ROOM_SETUP=1: room anchored, opening Room Setup"
					"$HERE/tools/vrlaunch" openvr.tool.steamvr_room_setup 2>&1 | grep -v -i breakpad | sed 's/^/  /'
				else
					echo "  ROOM_SETUP=1: no floor capture within 20 min; open Room Setup later with tools/vrlaunch"
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

# A plugin without our pose log (PLUGIN_DIR=...-upstream): measure against SteamVR's own pose samples instead.
if [ "${JITTER_CHECK:-0}" = 1 ] && [ ! -s "${OVRD_POSE_LOG:-}" ] && [ -s "$LOGDIR/steamvr-vrposes-$STAMP.csv" ]; then
	echo "  (no plugin pose log: measuring against SteamVR's pose samples)"
	OVRD_POSE_LOG="$LOGDIR/steamvr-vrposes-$STAMP.csv"
fi
if [ "${JITTER_CHECK:-0}" = 1 ] && [ -s "${OVRD_POSE_LOG:-}" ] && [ -s "${FRAMES_CSV:-}" ]; then
	echo
	echo "== Jitter check (tracking vs frame timing vs prediction)"
	cat "$LOGDIR/steamvr-frames-$STAMP-summary.txt" 2>/dev/null
	echo; echo "CPU per thread:"; cat "$LOGDIR/steamvr-cpu-$STAMP-summary.txt" 2>/dev/null
	python3 -I "$HERE/analysis/steamvr_timeline.py" "$OVRD_POSE_LOG" "$FRAMES_CSV" | tee "$LOGDIR/steamvr-jitter-$STAMP.txt"
	echo; echo "== Frames drawn one refresh early or late (the pose each frame was drawn with)"
	python3 -I "$HERE/analysis/steamvr_frame_shift.py" "$OVRD_POSE_LOG" "$FRAMES_CSV" | tee -a "$LOGDIR/steamvr-jitter-$STAMP.txt"
	if [ -s "$LOGDIR/steamvr-probe-$STAMP.csv" ]; then
		echo; echo "== SteamVR's own pose prediction (vrserver, no app involved)"
		python3 -I "$HERE/analysis/steamvr_probe.py" "$OVRD_POSE_LOG" "$LOGDIR/steamvr-probe-$STAMP.csv" | tee -a "$LOGDIR/steamvr-jitter-$STAMP.txt"
	fi
fi
