#!/usr/bin/env bash
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
#
# SteamVR with our own development build (../trial/build), the way we've always tested: launch-steamvr.sh registers
# it for this session only and puts SteamVR's driver list back when SteamVR exits (the registered/compiled install,
# Sony's, whatever was there). Anything after the script name is passed on, e.g. ./launch-dev.sh with LOG_LEVEL=debug.
#
#   ./launch-dev.sh
#   ./launch-dev.sh --record                   also record the controller tracker's input (blobs, camera poses) with
#                                              the controllers' and headset's sensors, for offline replay (large)
#   ./launch-dev.sh --record-camera            also record the bottom camera pair (hand tracking's images) for offline
#                                              replay (logs/camera-*.pv2c, ~30 MB/s: keep the run short)
#   DEV_BUILD=/path/to/build ./launch-dev.sh   another build folder

set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
DEV_BUILD="${DEV_BUILD:-$HERE/../trial/build}"
[ -f "$DEV_BUILD/steamvr-monado-psvr2/bin/linux64/driver_monado_psvr2.so" ] ||
	{ echo "No SteamVR plugin in $DEV_BUILD (build it: ninja -C $DEV_BUILD)"; exit 1; }
echo "== Development build: $DEV_BUILD"
args=()
for a in "$@"; do
	if [ "$a" = "--record" ]; then
		export RECORD_DATASET=1 TRACKING=1
		echo "== Recording the controller tracker's input for offline replay (logs/dataset-*, sense-*, record-*)"
	elif [ "$a" = "--record-camera" ]; then
		export RECORD_CAMERA=1
		echo "== Recording the bottom camera pair for offline hand tracking replay (logs/camera-*.pv2c, ~30 MB/s)"
	else
		args+=("$a")
	fi
done
MONADO_BUILD="$DEV_BUILD" exec "$HERE/launch-steamvr.sh" ${args[@]+"${args[@]}"}
