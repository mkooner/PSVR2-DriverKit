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

echo "== Stages ($LOG)"
stage "1 Headset opened (Monado)"       "PS VR2 HMD|psvr2_hmd|\[PSVR2\]|d/psvr2"                   "headset not probed: USB/udev, or another runtime holds it"
stage "2 Calibration block parsed"      "Camera [0-9] calibration: resolution"  "no calibration block: controllers must be on when the service starts"
stage "3 Camera poses computed"         "camera [0-9] pose position"            "tracker not created"
stage "4 LD (LED) data arriving"        "LD stats:"                             "no LED payloads from the headset"
stage "5 LEDs seen by cameras"          "LEDs per camera \[[^0]|LEDs per camera \[[0-9]+ [1-9]|LEDs per camera \[[0-9]+ [0-9]+ [1-9]|LEDs per camera \[[0-9]+ [0-9]+ [0-9]+ [1-9]" "cameras see no LEDs: controllers not in view, or LEDs not lit (LED sync)"
stage "6 Controllers seen by LED sync"  "Device has been seen"                   "LEDs never identified: LED sync / timing"
stage "7 LED sync phase changes"        "Transitioning from search phase"       "LED sync stuck in its first phase"
stage "8 Gray code accepted"            "Accepting gray code"                   "gray code never locked"
stage "9 LED sync STABLE"               "Entering STABLE"                       "LED sync never reached STABLE"
stage "10 Controller poses solved"      "Found pose for device|succeeded with"  "no 6DoF poses solved: camera placement / LED model"

echo
echo "== Clock check (ts host-now should be a few ms negative)"
grep -E "LD stats:" "$LOG" | grep -o "ts host-now [+-][0-9.]*ms" | sort | uniq -c | sort -rn | head -5

echo
echo "== Head pose (SLAM rate, how far gyro drifts from SLAM, filter on/off)"
grep -E "Pose stats:" "$LOG" | tail -4 | sed -E 's/.*Pose stats: //' | cut -c1-200

echo
echo "== Controllers (thread health, LED sync progress)"
for pat in "Skipping .* Bluetooth simple report" "Failed to send output report" "Output reports recovered" "Output reports failed .* stopping" "trying offset" "Device has been seen" "Accepting gray code" "Gray code jump of .* sent" "Gray code jump applied" "Entering STABLE" "Found pose for device" "holding position" "releasing held position" "using the optical pose" "agrees with the optical fix again"; do
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
for pat in "Count calibration starting" "Count calibration restarting" "searching again near the window" "Count calibration done" "Count hold probe" "moved (later|earlier)" "not in view" "Transitioning from search phase 10 -> 9"; do
	printf '  %-42s %s\n' "$pat" "$(grep -c -E "$pat" "$LOG" 2>/dev/null || true)"
done
grep -E "Count calibration (done|restarting)" "$LOG" | tail -4 | sed -E 's/.*Count calibration/  /' | cut -c1-200
grep -E "Count hold probe" "$LOG" | tail -2 | sed -E 's/.*Count hold probe: /  hold: /' | cut -c1-160
grep -E "LED clock map" "$LOG" | tail -2 | sed -E 's/.*LED clock map: /  clock map: /' | cut -c1-140
grep -E "Count calibration state" "$LOG" | tail -3 | sed -E 's/.*Count calibration /  last step: /' | cut -c1-140
grep -E "LED command round trip" "$LOG" | tail -2 | sed -E 's/.*LED command round trip: /  LED command round trip: /' | cut -c1-120
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
echo "== Errors/warnings (top 15 distinct)"
grep -E "\b(ERROR|WARN|error|warning)\b|\(E\)|\(W\)" "$LOG" | sed -E 's/[0-9]+(\.[0-9]+)?/N/g' | sort | uniq -c | sort -rn | head -15 | cut -c1-220

echo
echo "Send me this summary plus the log file: $LOG"
