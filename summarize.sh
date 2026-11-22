#!/usr/bin/env bash
# Summarize a PS VR2 tracking test log stage by stage.
# Usage: ./summarize.sh logs/<file>.log

set -u
LOG="${1:?usage: summarize.sh <log>}"

count() { grep -c -E "$1" "$LOG" 2>/dev/null || true; }
first() { grep -m1 -E "$1" "$LOG" 2>/dev/null | cut -c1-200; }
stage() { # name, pattern, hint-if-missing
	local n; n=$(count "$2")
	if [ "$n" -gt 0 ]; then
		printf '  [ok] %-38s (%s lines)\n' "$1" "$n"
	else
		printf '  [--] %-38s  -> %s\n' "$1" "$3"
	fi
}

build=$(grep -m1 -oE "ovrn: driver build [^ ]+" "$LOG" | sed 's/ovrn: //')
echo "== ${build:-driver build not logged (driver older than the build line)}"
echo
echo "== Stages ($LOG)"
# Stages 1b-5 match the driver's info-level messages (the older patterns, kept as alternatives, are debug-level or
# gone: a normal run showed the calibration block, the tracker and LED data missing while they worked).
stage "1 Headset opened (Monado)"       "PS VR2 HMD|psvr2_hmd|\[PSVR2\]|d/psvr2"                   "headset not probed: USB/udev, or another runtime holds it"
stage "1b Head tracking (SLAM) arriving" "Re-centered SLAM space on the first pose"  "no head pose from the headset: see USB streams below"
stage "2 Calibration block parsed"      "psvr2_parse_gen_calibration_block\] Camera [0-9] (aligned|mount)|Camera [0-9] calibration: resolution|using the saved copy" "no calibration block from the headset: see USB streams below"
stage "3 LED tracker created"           "Creating the LED tracker|creating the LED tracker|added to the running LED tracker|camera [0-9] pose position" "tracker not created: no calibration block, or no controllers connected"
stage "4 LD (LED) data arriving"        "Headset LED detector now|LD stats:"    "no LED detector data from the headset: see USB streams below"
stage "5 LEDs seen by cameras"          "Count calibration done|LEDs per camera \[[^0]|LEDs per camera \[[0-9]+ [1-9]|LEDs per camera \[[0-9]+ [0-9]+ [1-9]|LEDs per camera \[[0-9]+ [0-9]+ [0-9]+ [1-9]" "cameras see no LEDs: controllers not in view, or LEDs not lit (LED sync)"
# Stages 6-10 also match the count calibration (search phases 9 -> 10) and the tracker's 5 s report, both info-level:
# the gray-code and "Found pose" lines are debug-level, so a normal run showed no poses while both controllers tracked.
stage "6 Controllers seen by LED sync"  "Device has been seen|Tracker report device [0-9]+: searches [0-9]+ found [1-9]|Tracker report device .*accepted [1-9]" "LEDs never identified: LED sync / timing"
stage "7 LED sync phase changes"        "Transitioning from search phase"       "LED sync stuck in its first phase"
stage "8 LED timing found"              "Accepting gray code|Count calibration done" "LED timing never found (count calibration / gray code)"
stage "9 LED timing held"               "Entering STABLE|Transitioning from search phase 9 -> 10" "LED timing never held (STABLE / count hold)"
stage "10 Controller poses solved"      "Found pose for device|succeeded with|Tracker report device .*accepted [1-9]" "no 6DoF poses solved: camera placement / LED model"

# Per controller, from the tracker's 5 s reports: accepted poses per second, and the share of reports with any.
awk '/Tracker report device [0-9]+:/ {d = $0; sub(/.*Tracker report device /, "", d); sub(/:.*/, "", d);
	a = $NF + 0; n[d]++; sum[d] += a; if (a > 0) with[d]++}
	END {for (d in n) printf "  device %s: %.0f poses/s accepted on average, poses in %d of %d reports (%.0f%%, 5 s each)\n",
		d, sum[d] / (5 * n[d]), with[d], n[d], 100 * with[d] / n[d]}' "$LOG" | sort

echo
echo "== Clock check (ts host-now should be a few ms negative)"
grep -E "LD stats:" "$LOG" | grep -o "ts host-now [+-][0-9.]*ms" | sort | uniq -c | sort -rn | head -5

echo
echo "== Room calibration (universe, captures, room dropped)"
grep -E "Room: |SLAM reset" "$LOG" | sed -E 's/.*(Room: |SLAM reset)/  \1/' | cut -c1-220 | tail -12
echo

echo "== Head pose (SLAM rate, how far gyro drifts from SLAM, filter on/off)"
grep -E "Pose stats:" "$LOG" | tail -4 | sed -E 's/.*Pose stats: //' | cut -c1-200

echo
echo "== Controllers (thread health, LED sync progress)"
for pat in "Skipping .* Bluetooth simple report" "Failed to send output report" "Output reports recovered" "Output reports failed .* stopping" "trying offset" "Device has been seen" "Accepting gray code" "Gray code jump of .* sent" "Gray code jump applied" "Entering STABLE" "Found pose for device" "holding position" "releasing held position" "using the optical pose" "agrees with the optical fix again" "Rejecting pose for device.*up direction" "Rejecting pose for device.*rotated" "taking the new one as the reference" "skipping confirmation" "LED clock map latched"; do
	printf '  %-40s %s\n' "$pat" "$(grep -c -E "$pat" "$LOG" 2>/dev/null || true)"
done

echo
echo "== Controller Bluetooth sends (last per controller)"
grep -E "Send stats" "$LOG" | tail -2 | sed -E 's/.*Send stats: /  /' | cut -c1-200
ts=$(basename "$LOG" | grep -oE '[0-9]{8}-[0-9]{6}' | head -1)
if [ -n "$ts" ]; then
	start="${ts:0:4}-${ts:4:2}-${ts:6:2} ${ts:9:2}:${ts:11:2}:${ts:13:2}"
	end=$(date -d "@$(stat -c %Y "$LOG")" '+%F %T')
	n=$(journalctl --since "$start" --until "$end" 2>/dev/null | grep -c "BT socket write error")
	echo "  bluetoothd write errors during the run: $n (should be 0)"
fi

echo
echo "== LED timing jitter (exposure timestamps; re-anchor moves should be small)"
grep -E "Exposure timestamp jitter" "$LOG" | tail -2 | sed -E 's/.*Exposure timestamp jitter: /  /' | cut -c1-160
grep -o "Clock model drifted -\?[0-9]*ns" "$LOG" | grep -o -- "-\?[0-9]*" | awk '{a=$1<0?-$1:$1; s+=a; n++; if(a>m)m=a} END{if(n) printf "  re-anchor moves: %d, mean %.0fus, max %.0fus (was mean 174us, max 1463us)\n", n, s/n/1000, m/1000}'
grep -E "Transitioning from search phase (4 -> 5|5 -> 6|. -> 6)" "$LOG" | sed -E 's/.*Transitioning/  reached:/' | sort | uniq -c
echo
echo "== Count calibration (LED sync by blob counts)"
for pat in "Count calibration starting" "Count calibration restarting" "searching again near the window" "Count calibration done" "accepting the window without confirmation" "time limit reached" "LEDs visible when always on" "not visible even always on" "Count hold probe" "moved (later|earlier)" "not in view" "Transitioning from search phase 10 -> 9"; do
	printf '  %-42s %s\n' "$pat" "$(grep -c -E "$pat" "$LOG" 2>/dev/null || true)"
done
grep -E "Count calibration (done|restarting)" "$LOG" | tail -4 | sed -E 's/.*Count calibration/  /' | cut -c1-200
echo "  LED drift (the hold following the clock; high drift = faster than normal controllers, 60 ppm+):"
for pat in "following the Bluetooth clock" "Bluetooth clock move (overshot|lost the LEDs)" "Count hold lost the LEDs" "high LED drift" "the LEDs drift .*trimming it"; do
	printf '    %-46s %s\n' "$pat" "$(grep -c -E "$pat" "$LOG" 2>/dev/null || true)"
done
grep -E "\[(left|right)\] Count hold: (high LED drift|the LEDs drift)" "$LOG" | tail -4 | sed -E 's/.*\[(left|right)\] Count hold: /    \1: /' | cut -c1-150
grep -E "Count drift from" "$LOG" | tail -3 | sed -E 's/.*Count drift from /  drift: /' | cut -c1-150
grep -E "Count hold probe" "$LOG" | tail -2 | sed -E 's/.*Count hold probe: /  hold: /' | cut -c1-160
grep -E "LED clock map" "$LOG" | tail -2 | sed -E 's/.*LED clock map: /  clock map: /' | cut -c1-140
grep -E "Count calibration state" "$LOG" | tail -3 | sed -E 's/.*Count calibration /  last step: /' | cut -c1-140
grep -E "LED command round trip" "$LOG" | tail -2 | sed -E 's/.*LED command round trip: /  LED command round trip: /' | cut -c1-120
grep -E "IMU clock: " "$LOG" | tail -2 | sed -E 's/.*IMU clock: /  IMU clock: /' | cut -c1-170
grep -E "\] Clock: " "$LOG" | tail -2 | sed -E 's/.*Clock: /  controller clock: /' | cut -c1-160

echo
echo "== LED sync BG phase (last summaries)"
grep -E "BG stats" "$LOG" | tail -4 | sed -E 's/.*BG stats/  BG/' | cut -c1-200

echo
echo "== What SteamVR got per controller (FINAL)"
for h in 0 1; do
	grep -E " FINAL [0-9]+ $h " "$LOG" | awk -v h=$h '{for(i=1;i<=NF;i++) if($i=="pos"){x=$(i+1);y=$(i+2);z=$(i+3)} f=$0; sub(/.*flags /,"",f); split(f,a," "); n++; d=sqrt(x*x+y*y+z*z); s+=d; if(d>3)far++; if(a[1]!="0")valid++} END{if(n) printf "  hand %d: %d samples, %d with flags, mean distance from origin %.2f m, >3 m: %d\n", h, n, valid, s/n, far}'
done

echo
echo "== Undecoded headset data (VD entries, possibly play area)"
grep -E "Unknown VD entry type" "$LOG" | sed -E 's/.*Unknown VD entry type/  type/' | sort | uniq -c | head -8

echo
echo "== Last LD stats lines"
grep -E "LD stats:" "$LOG" | tail -3 | cut -c1-220

echo
echo "== Camera poses"
grep -E "camera [0-9] pose position" "$LOG" | tail -4 | cut -c1-200

echo
echo "== Controller compatibility (what legacy games are shown; inputs created; inputs that never changed)"
if [ "$(count "controller compatibility:|controller inputs (after|at shutdown)|not created by SteamVR")" -gt 0 ]; then
	grep -E "controller compatibility:|controller inputs (after 2 minutes|at shutdown):|not created by SteamVR|legacy controller mode" "$LOG" |
		sed -E 's/^.*ovrn: /  /' | sort -u | cut -c1-240
else
	echo "  not logged (driver older than the compatibility lines)"
fi

echo
echo "== Controller tracker (every 5 s: blobs, searches, poses found, rejected, accepted per controller)"
if [ "$(count "Tracker report")" -gt 0 ]; then
	echo "  totals over the run:"
	grep -E "Tracker report device" "$LOG" | sed -E 's/.*Tracker report device ([0-9]+): searches ([0-9]+) found ([0-9]+), fast path ([0-9]+), refinement failed ([0-9]+), rejected by IMU up ([0-9]+) \/ gyro ([0-9]+), accepted ([0-9]+).*/\1 \2 \3 \4 \5 \6 \7 \8/' |
		awk '{s[$1]+=$2; f[$1]+=$3; p[$1]+=$4; r[$1]+=$5; u[$1]+=$6; g[$1]+=$7; a[$1]+=$8} END {for (d in s) printf "    device %s: searches %d found %d, fast path %d, refinement failed %d, rejected up %d / gyro %d, accepted %d\n", d, s[d], f[d], p[d], r[d], u[d], g[d], a[d]}' | sort
	echo "  last reports:"
	grep -E "Tracker report" "$LOG" | tail -6 | sed -E 's/^.*Tracker report/    Tracker report/' | cut -c1-200
else
	echo "  no tracker reports (driver older than the report, or the tracker never started)"
fi

echo
echo "== USB streams (a stopped stream sends nothing more this session; commands that failed)"
n=$(count "USB stream stopped|Failed to send report|Failed to get report|cannot cancel")
if [ "$n" -gt 0 ]; then
	grep -E "USB stream stopped|Failed to send report|Failed to get report|cannot cancel|failed to cancel transfer" "$LOG" | sed -E 's/^.*(USB stream stopped|Failed to send report|Failed to get report|USB shutdown: cannot cancel)/  \1/' | sort | uniq -c | cut -c1-200
else
	echo "  none stopped, no failed commands"
fi
old=$(count "failed to cancel transfer")
[ "$old" -gt 0 ] && echo "  $old x 'failed to cancel transfer' (older driver: streams that had already stopped, reason not logged)"
grep -E "had already stopped" "$LOG" | sed -E 's/^.*USB shutdown: /  at shutdown: /' | sort | uniq -c | cut -c1-160

echo
echo "== Errors/warnings (top 15 distinct)"
grep -E "\b(ERROR|WARN|error|warning)\b|\(E\)|\(W\)" "$LOG" | sed -E 's/[0-9]+(\.[0-9]+)?/N/g' | sort | uniq -c | sort -rn | head -15 | cut -c1-220

echo
echo "Send me this summary plus the log file: $LOG"
