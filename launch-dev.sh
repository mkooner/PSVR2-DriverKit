#!/usr/bin/env bash
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
#
# SteamVR with our own development build (../trial/build), the way we've always tested: launch-steamvr.sh registers
# it for this session only and puts SteamVR's driver list back when SteamVR exits (the registered/compiled install,
# Sony's, whatever was there). Anything after the script name is passed on, e.g. ./launch-dev.sh with LOG_LEVEL=debug.
#
#   ./launch-dev.sh
#   DEV_BUILD=/path/to/build ./launch-dev.sh   another build folder

set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
DEV_BUILD="${DEV_BUILD:-$HERE/../trial/build}"
[ -f "$DEV_BUILD/steamvr-monado-psvr2/bin/linux64/driver_monado_psvr2.so" ] ||
	{ echo "No SteamVR plugin in $DEV_BUILD (build it: ninja -C $DEV_BUILD)"; exit 1; }
echo "== Development build: $DEV_BUILD"
MONADO_BUILD="$DEV_BUILD" exec "$HERE/launch-steamvr.sh" "$@"
