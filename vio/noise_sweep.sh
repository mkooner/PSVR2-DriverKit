#!/usr/bin/env bash
# Basalt IMU noise densities (accel m/s2/sqrtHz, gyro rad/s/sqrtHz) and camera time offset (ns); benchmark each.
set -u
run() {
	python3 -I - "$1" "$2" "$3" <<'PY'
import json, sys
a,g,off=float(sys.argv[1]),float(sys.argv[2]),int(sys.argv[3])
c=json.load(open('psvr2_basalt_calib.json')); v=c['value0']
v['accel_noise_std']=[a]*3; v['gyro_noise_std']=[g]*3; v['cam_time_offset_ns']=off
json.dump(c,open('sweep_calib.json','w'),indent=1)
PY
	sed 's/psvr2_basalt_calib.json/sweep_calib.json/' psvr2_basalt.toml > sweep.toml
	rm -rf out-sweep; ./run_slambatch.sh euroc-20261116-202020 sweep.toml out-sweep 600 > /dev/null
	r=$(python3 -I benchmark.py euroc-20261116-202020 out-sweep ../logs/record-20261116-202020-filter1.csv 2>/dev/null | sed -n '/== Basalt/,/== headset/p')
	rot=$(echo "$r" | grep -o 'p99 [0-9.]* deg' | head -1)
	pos=$(echo "$r" | grep 'position wobble' | sed -E 's/.*mm: //; s/ Hz//g')
	jmp=$(echo "$r" | grep 'off a smooth path' | sed -E 's/.*:  0-10/0-10/')
	echo "$4: rotation err $rot | position wobble mm $pos"
	echo "      off smooth path (p99 mm / >5mm): $jmp"
}
run 0.00205 0.00017 2000000 "measured x5, camera +2 ms"
