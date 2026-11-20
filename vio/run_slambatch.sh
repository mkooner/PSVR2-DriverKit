#!/usr/bin/env bash
# Run monado-cli slambatch non-interactively (its "press enter to quit" thread reads stdin; give it one that stays
# open until the run ends). Usage: run_slambatch.sh dataset config.toml out_dir [timeout_s]
# Runs from this folder, so the config's relative paths (psvr2_basalt.toml) resolve here. MONADO_BUILD: the Monado
# build (default ../../trial/build); VIT_SYSTEM_LIBRARY_PATH: libbasalt.so (default ~/Documents/Claude/Monado/basalt-install).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$3"
DATASET="$(realpath "$1")" CONFIG="$(realpath "$2")" OUT="$(realpath "$3")"
BUILD="${MONADO_BUILD:-$HERE/../../trial/build}"
BASALT="${VIT_SYSTEM_LIBRARY_PATH:-$HOME/Documents/Claude/Monado/basalt-install/lib/libbasalt.so}"
cd "$HERE"
timeout "${4:-900}" env VIT_SYSTEM_LIBRARY_PATH="$BASALT" \
	"$BUILD/src/xrt/targets/cli/monado-cli" slambatch "$DATASET" "$CONFIG" "$OUT" \
	< <(sleep 100000) > "$OUT/log.txt" 2>&1
echo "exit $?"
