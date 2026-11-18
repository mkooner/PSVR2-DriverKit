#!/usr/bin/env bash
# Run monado-cli slambatch non-interactively (its "press enter to quit" thread reads stdin; give it one that stays
# open until the run ends). Usage: run_slambatch.sh dataset config.toml out_dir [timeout_s]
set -u
mkdir -p "$3"
timeout "${4:-900}" env VIT_SYSTEM_LIBRARY_PATH=/home/mehrab/Documents/Claude/Monado/basalt-install/lib/libbasalt.so \
	/home/mehrab/Documents/Claude/Monado/trial/build/src/xrt/targets/cli/monado-cli slambatch "$(realpath "$1")" "$(realpath "$2")" "$(realpath "$3")" \
	< <(sleep 100000) > "$3/log.txt" 2>&1
echo "exit $?"
