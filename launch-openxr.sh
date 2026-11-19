#!/usr/bin/env bash
# Launch any OpenXR app directly on our Monado build (no SteamVR), for PS VR2 testing.
#
# Usage:
#   ./launch-openxr.sh /path/to/app [app arguments...]
#
# What it does:
#   1. Checks nothing else holds the headset (SteamVR, another monado-service).
#   2. Starts our build's monado-service (the OpenXR runtime with the PS VR2 driver) and waits until it's ready.
#   3. Runs the app with XR_RUNTIME_JSON pointing at our build's runtime manifest, so it uses Monado whatever the
#      system's default OpenXR runtime is.
#   4. When the app exits (or on Ctrl+C), stops monado-service and keeps its log in logs/.
#
# BASALT=1 turns on the Basalt referee (see launch-steamvr.sh). Driver options pass straight through, e.g.:
#   PSVR2_ACCEL_FUSION=1 PSVR2_GYRO_BIAS_TAU=5 ./launch-openxr.sh ~/apps/hello_xr -g Vulkan2
# Native Linux OpenXR apps only; Windows apps run through Proton/SteamVR instead.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
BUILD="${MONADO_BUILD:-$HERE/../trial/build}"
SERVICE="$BUILD/src/xrt/targets/service/monado-service"
MANIFEST="$BUILD/openxr_monado-dev.json"
LOGDIR="$HERE/logs"
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="$LOGDIR/monado-$STAMP.log"
SOCKET="${XDG_RUNTIME_DIR:-/tmp}/monado_comp_ipc"

if [ $# -lt 1 ]; then
	echo "usage: $0 /path/to/openxr-app [arguments...]"
	exit 1
fi
APP="$1"
shift
if [ ! -e "$APP" ]; then
	echo "App not found: $APP"
	exit 1
fi
if [ ! -x "$APP" ]; then
	echo "Not executable: $APP (chmod +x it, or give the binary inside the app's folder)"
	exit 1
fi
case "$APP" in
*.exe | *.EXE)
	echo "$APP is a Windows app: run it through Proton/SteamVR (launch-steamvr.sh), not directly on Monado."
	exit 1
	;;
esac
for f in "$SERVICE" "$MANIFEST"; do
	if [ ! -e "$f" ]; then
		echo "Missing $f: build Monado first (ninja in $BUILD)"
		exit 1
	fi
done
if pgrep -x vrserver >/dev/null || pgrep -x vrmonitor >/dev/null; then
	echo "SteamVR is running and holds the headset: close it first."
	exit 1
fi
if pgrep -x monado-service >/dev/null; then
	echo "A monado-service is already running: stop it first (pkill -x monado-service)."
	exit 1
fi

mkdir -p "$LOGDIR"
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
# SLAM corrections are brought in at a smoothly ramped rate that the reported velocity includes (driver default,
# PSVR2_SMOOTH_CORRECTION_MS=30): offline the velocity matched the pose's motion 4-5x better and SteamVR's prediction
# landed closer, with the pose itself no jerkier. SMOOTH_CORRECTION=0 turns it off (the earlier blend).
if [ "${SMOOTH_CORRECTION:-1}" = 0 ]; then
	export PSVR2_SMOOTH_CORRECTION_MS=0
	echo "  SMOOTH_CORRECTION=0: earlier correction blend (velocity without the correction's motion)"
fi
# The head pose recording (IMU, SLAM, and the panel's own frame counter "D" lines, analysis/panel_frames.py), as the
# SteamVR launcher writes it, so both paths can be compared.
export PSVR2_RECORD_FILE="${PSVR2_RECORD_FILE:-$LOGDIR/record-$STAMP-openxr.csv}"
rm -f "$SOCKET" # a stale socket from a crashed service would block the new one
export XRT_LOG="${XRT_LOG:-info}"
# TRACKING=1: what Monado's compositor shows per displayed frame (re-warp pose, the game's layer pose, presents,
# vblanks) and the head's pose every 2 ms (analysis/compare_paths.py). OFF by default: with it on, the 2026-11-18 11:41
# session's game frame work doubled (4.1 -> 10.8 ms) and half the frames missed their refresh; the OpenXR path stays
# as it was unless this is asked for.
if [ "${TRACKING:-0}" = 1 ]; then
	export XRT_COMPOSITOR_FRAME_LOG="${XRT_COMPOSITOR_FRAME_LOG:-$LOGDIR/compframes-$STAMP-openxr.csv}"
	echo "  tracking: $XRT_COMPOSITOR_FRAME_LOG"
fi
export PSVR2_LOG="${PSVR2_LOG:-info}"

# JITTER_CHECK=1: Monado's app pacer decisions in the log (U_PACING_APP_LOG=debug: when the app is woken from
# xrWaitFrame and why) and per-thread CPU use / CPU wait of the app and monado-service (tools/cpusample.py).
if [ "${JITTER_CHECK:-0}" = 1 ]; then
	export U_PACING_APP_LOG=debug
	echo "  JITTER_CHECK=1: pacer log on, CPU per thread to $LOGDIR/monado-cpu-$STAMP.csv"
fi
echo "Starting monado-service ($LOG)..."
# The service exits when its stdin closes (it waits for Enter), so give it one that stays open.
"$SERVICE" > "$LOG" 2>&1 < <(sleep infinity) &
SERVICE_PID=$!

stop_service() {
	if kill -0 "$SERVICE_PID" 2>/dev/null; then
		echo "Stopping monado-service..."
		kill -INT "$SERVICE_PID" 2>/dev/null
		for _ in $(seq 1 50); do
			kill -0 "$SERVICE_PID" 2>/dev/null || break
			sleep 0.1
		done
		kill -KILL "$SERVICE_PID" 2>/dev/null
	fi
	echo "monado-service log: $LOG"
}
trap 'echo; stop_service; exit 130' INT TERM

for _ in $(seq 1 100); do
	[ -S "$SOCKET" ] && break
	if ! kill -0 "$SERVICE_PID" 2>/dev/null; then
		echo "monado-service exited during startup, last lines of $LOG:"
		tail -n 20 "$LOG"
		exit 1
	fi
	sleep 0.2
done
if [ ! -S "$SOCKET" ]; then
	echo "monado-service didn't come up within 20 s, see $LOG"
	stop_service
	exit 1
fi
echo "Monado is ready. Running $APP $*"

CPU_PID=""
if [ "${JITTER_CHECK:-0}" = 1 ]; then
	python3 -I "$HERE/tools/cpusample.py" "$LOGDIR/monado-cpu-$STAMP.csv" "$(basename "$APP" | cut -c1-15)" monado-service \
		2>"$LOGDIR/monado-cpu-$STAMP-summary.txt" &
	CPU_PID=$!
fi
XR_RUNTIME_JSON="$MANIFEST" "$APP" "$@"
STATUS=$?
if [ -n "$CPU_PID" ]; then
	kill -INT "$CPU_PID" 2>/dev/null
	wait "$CPU_PID" 2>/dev/null
	echo "CPU per thread:"; cat "$LOGDIR/monado-cpu-$STAMP-summary.txt"
fi
echo "App exited ($STATUS)."
stop_service
exit "$STATUS"
