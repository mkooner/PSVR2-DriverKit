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
PLUGIN="$BUILD/steamvr-monado"
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
}

if [ "${1-}" = "--restore" ]; then
	restore_config
	exit 0
fi

echo "== Pre-flight"
fail() { echo "  [!!] $*"; exit 1; }
[ -f "$PLUGIN/driver.vrdrivermanifest" ] || fail "plugin not built: $PLUGIN (build with XRT_FEATURE_STEAMVR_PLUGIN=ON)"
[ -f "$PLUGIN/bin/linux64/driver_monado.so" ] || fail "driver_monado.so missing in $PLUGIN"
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
missing=$(ldd "$PLUGIN/bin/linux64/driver_monado.so" | grep -c 'not found')
[ "$missing" = 0 ] || fail "driver_monado.so has $missing unresolved libraries (ldd)"
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
trap 'restore_config' EXIT
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
	python3 -I - "$VRSETTINGS" <<'EOF'
import json, sys
path = sys.argv[1]
with open(path) as f:
    cfg = json.load(f)
drv = cfg.setdefault("driver_monado", {})
if drv.get("blocked_by_safe_mode"):
    print("  unblocked driver_monado (was blocked by a previous SteamVR safe mode event)")
drv["blocked_by_safe_mode"] = False
drv["enable"] = True
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

# ROOM_SETUP=1 ./launch-steamvr.sh runs SteamVR's own room setup instead of the driver-provided 2m play area.
if [ "${ROOM_SETUP:-0}" = 1 ]; then
	export OVRD_PLAY_AREA_SIZE=0
	echo "  ROOM_SETUP=1: not providing a play area, SteamVR room setup will run"
fi

export VALVE_SKIP_RUNTIME_SAFETY=1
export XRT_LOG=debug
export PSVR2_LOG=debug
export PSSENSE_LOG=debug
export CONSTELLATION_TRACKER_LOG=debug
export T_LED_SYNC_REFINEMENT_LOG=debug
# Head pose: record raw IMU/SLAM for offline replay, and allow A/B against the old behaviour with
# POSE_FILTER=0 ./script.sh
export PSVR2_POSE_FILTER="${POSE_FILTER:-1}"
export PSVR2_RECORD_FILE="$LOGDIR/record-$(date +%Y%m%d-%H%M%S)-filter${PSVR2_POSE_FILTER}.csv"

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
	while pgrep -x vrserver >/dev/null; do sleep 2; done
	echo "  SteamVR exited."
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
