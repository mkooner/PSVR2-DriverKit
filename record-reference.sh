#!/usr/bin/env bash
# Record a reference session of ANY SteamVR headset driver (Sony's through the PS VR2 app / PSVR2Toolkit, or ours),
# for a field-by-field comparison. SteamVR must already be running with that driver. Nothing in SteamVR or the driver
# is changed; these are the read-only tools behind the earlier comparisons.
#
#   ./record-reference.sh sony ~/Documents/Claude/ZeldaWWHD-VR/build/linux/wwhd-vr-launcher
#   ./record-reference.sh sony            (no game: records for 60 s)
#
# Writes to logs/:
#   display-<tag>-<stamp>.txt     everything SteamVR knows about the display: all headset properties, projection,
#                                 eye-to-head, hidden-area mask, lens distortion (tools/vrdisplaydump;
#                                 analysis/display_diff.py compares two)
#   vrposelog-<tag>-<stamp>.csv   the head pose SteamVR hands a game, once per refresh (analysis/jitter/logjitter.py)
#   xr-frames-<stamp>-<game>.csv  the game's frame timing and the poses it draws with (with a game)
#   ref-<tag>-<stamp>-vrserver.txt / -vrcompositor.txt   SteamVR's own logs of the session
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
TAG="${1:-}"
[ -n "$TAG" ] || { echo "usage: $0 <tag, e.g. sony> [/path/to/game]"; exit 1; }
GAME="${2:-}"
LOGDIR="$HERE/logs"; mkdir -p "$LOGDIR"
STAMP="$(date +%Y%m%d-%H%M%S)"
STEAM_LOGS="${STEAM_LOGS:-$HOME/.local/share/Steam/logs}"

pgrep -x vrserver >/dev/null || { echo "SteamVR isn't running: start it with the driver to record, then run this again"; exit 1; }
for tool in vrdisplaydump vrposelog; do
	[ -x "$HERE/tools/$tool" ] || { echo "tools/$tool isn't built (see the Build: line in tools/$tool.cpp)"; exit 1; }
done

echo "== display description"
"$HERE/tools/vrdisplaydump" "$LOGDIR/display-$TAG-$STAMP.txt" && echo "  $LOGDIR/display-$TAG-$STAMP.txt"
grep -E '^prop (1000|1001|1005) ' "$LOGDIR/display-$TAG-$STAMP.txt" | sed 's/^/  /'

echo "== head pose per refresh"
"$HERE/tools/vrposelog" 36000 "$LOGDIR/vrposelog-$TAG-$STAMP.csv" 2>"$LOGDIR/vrposelog-$TAG-$STAMP.txt" &
POSELOG_PID=$!
echo "  $LOGDIR/vrposelog-$TAG-$STAMP.csv"

if [ -n "$GAME" ]; then
	echo "== game with the frame-timing layer: play for a minute or two (stand still, turn your head, move), then quit it"
	"$HERE/tools/xr_frame_log/run-with-frame-log.sh" "$GAME" >"$LOGDIR/ref-$TAG-$STAMP-game.txt" 2>&1
else
	echo "== no game given: recording 60 s (turn your head slowly and quickly)"
	sleep 60
fi

kill "$POSELOG_PID" 2>/dev/null; wait "$POSELOG_PID" 2>/dev/null
for f in vrserver vrcompositor; do
	[ -f "$STEAM_LOGS/$f.txt" ] && cp "$STEAM_LOGS/$f.txt" "$LOGDIR/ref-$TAG-$STAMP-$f.txt"
done
echo "== done: logs/*$STAMP* and logs/display-$TAG-$STAMP.txt"
