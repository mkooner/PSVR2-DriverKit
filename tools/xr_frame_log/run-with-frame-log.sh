#!/usr/bin/env bash
# Run an OpenXR app with XR_APILAYER_psvr2_frame_log (its frame timing as the runtime gives it, nothing changed), then
# print the analysis. Works on SteamVR (start SteamVR first) and Monado (as the app of launch-openxr.sh).
#   ./run-with-frame-log.sh ~/Documents/Claude/ZeldaWWHD-VR/build/linux/wwhd [args]
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
[ $# -ge 1 ] || { echo "usage: $0 /path/to/openxr-app [args]"; exit 1; }
LOGDIR="$HERE/../../logs"; mkdir -p "$LOGDIR"
OUT="$LOGDIR/xr-frames-$(date +%Y%m%d-%H%M%S)-$(basename "$1").csv"
[ -f "$HERE/libxr_frame_log.so" ] || gcc -O2 -fPIC -shared -o "$HERE/libxr_frame_log.so" "$HERE/xr_frame_log.c" -lpthread
echo "Frame timing log: $OUT"
cd "$(dirname "$1")" || exit 1
XR_API_LAYER_PATH="$HERE${XR_API_LAYER_PATH:+:$XR_API_LAYER_PATH}" \
XR_ENABLE_API_LAYERS="XR_APILAYER_psvr2_frame_log${XR_ENABLE_API_LAYERS:+:$XR_ENABLE_API_LAYERS}" \
XR_FRAME_LOG="$OUT" "$@"
STATUS=$?
python3 -I "$HERE/../../analysis/xr_frame_log.py" "$OUT"
exit $STATUS
