#!/usr/bin/env bash
# PS VR2 headset-camera controller tracking test run.
#
# Usage: ./run-test.sh [seconds]   (default 90)
#
# Starts monado-service from the psvr2-headset-constellation build with tracking debug logs,
# records everything to logs/<timestamp>.log, then prints a stage-by-stage summary.

set -u

DURATION="${1:-90}"
HERE="$(cd "$(dirname "$0")" && pwd)"
BUILD="${MONADO_BUILD:-$HERE/../trial/build}"
SERVICE="$BUILD/src/xrt/targets/service/monado-service"
LOGDIR="$HERE/logs"
LOG="$LOGDIR/$(date +%Y%m%d-%H%M%S).log"

mkdir -p "$LOGDIR"

echo "== Pre-flight"
if [ ! -x "$SERVICE" ]; then
	echo "monado-service not found at $SERVICE (set MONADO_BUILD to the build dir)"
	exit 1
fi

ok=1
lsusb | grep -qi '054c:0cde' && echo "  [ok] PS VR2 headset on USB (054c:0cde)" || { echo "  [!!] PS VR2 headset not found on USB (054c:0cde): check the PC adapter, USB and power"; ok=0; }
for pid in 0e45:left 0e46:right; do
	id="${pid%%:*}"; side="${pid##*:}"
	if lsusb | grep -qi "054c:$id" || ls /sys/bus/hid/devices/ 2>/dev/null | grep -qi "054C:${id^^}"; then
		echo "  [ok] Sense $side controller ($id)"
	else
		echo "  [!!] Sense $side controller ($id) not found: pair over Bluetooth or plug in USB"
		ok=0
	fi
done
if pgrep -x monado-service >/dev/null; then
	echo "  [!!] another monado-service is running, stop it first"
	exit 1
fi
if pgrep -f vrserver >/dev/null; then
	echo "  [!!] SteamVR (vrserver) is running and may hold the headset, close it first"
	ok=0
fi
if [ "$ok" = 0 ]; then
	read -r -p "Some checks failed. Run anyway? [y/N] " a
	[ "${a,,}" = y ] || exit 1
fi

echo
echo "== Running monado-service for ${DURATION}s"
echo "   Put the headset on a stable surface or wear it, and hold BOTH controllers in front of"
echo "   the headset cameras about 30-60cm away. Move them slowly after ~10s."
echo "   Log: $LOG"
echo

export XRT_LOG=debug
export PSVR2_LOG=debug
export PSSENSE_LOG=debug
export CONSTELLATION_TRACKER_LOG=debug
export T_LED_SYNC_REFINEMENT_LOG=debug
export T_CONSTELLATION_SENSOR_FUSION_LOG=${FUSION_LOG:-debug}
# Head pose: record raw IMU/SLAM for offline replay, and allow A/B against the old behaviour with
# POSE_FILTER=0 ./script.sh
export PSVR2_POSE_FILTER="${POSE_FILTER:-1}"
export PSVR2_RECORD_FILE="$LOGDIR/record-$(date +%Y%m%d-%H%M%S)-filter${PSVR2_POSE_FILTER}.csv"

timeout -s INT -k 10 "$DURATION" "$SERVICE" >"$LOG" 2>&1
echo "monado-service exited (code $?)"
echo

"$HERE/summarize.sh" "$LOG"
