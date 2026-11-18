#!/usr/bin/env bash
# Basalt with the IMU moved by d (cm, IMU axes x right / y up / z back) from the camera midpoint; benchmark each.
set -u
for d in "0,0,0" "0,3,0" "0,-3,0" "0,0,3" "0,0,-3" "2,0,0" "-2,0,0" "-0.27,-1.83,0.13"; do
	python3 -I - "$d" <<'PY'
import json, sys
d=[float(x)/100 for x in sys.argv[1].split(',')]
c=json.load(open('psvr2_basalt_calib.json'))
for T in c['value0']['T_imu_cam']:
    T['px']-=d[0]; T['py']-=d[1]; T['pz']-=d[2]
json.dump(c,open('sweep_calib.json','w'),indent=1)
PY
	sed 's/psvr2_basalt_calib.json/sweep_calib.json/' psvr2_basalt.toml > sweep.toml
	rm -rf out-sweep; ./run_slambatch.sh euroc-20261116-202020 sweep.toml out-sweep 600 > /dev/null
	r=$(python3 -I benchmark.py euroc-20261116-202020 out-sweep ../logs/record-20261116-202020-filter1.csv 2>/dev/null | sed -n '/== Basalt/,/== headset/p')
	rot=$(echo "$r" | grep -o 'p99 [0-9.]* deg' | head -1)
	pos=$(echo "$r" | grep 'position wobble' | sed -E 's/.*mm: //; s/ Hz//g')
	jmp=$(echo "$r" | grep 'off a smooth path' | grep -o '60-120 deg/s: [0-9.]* / [0-9.]*%')
	echo "IMU at midpoint + ($d) cm: rotation err $rot | position wobble mm $pos | fast-turn jumps (p99 mm / >5mm) $jmp"
done
