#!/usr/bin/env bash
# Unwarp saved frame pairs at several virtual views and score Mercury's detector on each (see detect_probe.cpp).
# Usage: sweep.sh dump_dir out_dir "fov pitch" ...
set -e
cd "$(dirname "$0")"
DUMP=$1; OUT=$2; shift 2
for cfg in "$@"; do
	set -- $cfg; D=$OUT/v$1_$2; mkdir -p $D
	for v in 0 1; do
		s=$([ $v = 0 ] && echo L || echo R); args=()
		for f in $DUMP/f*-$s.pgm; do args+=("$f" "$D/$(basename $f)"); done
		./rectify_probe psvr2-bottom-stereo.json $v $1 $2 640 "${args[@]}"
	done
	./detect_probe ~/.local/share/monado/hand-tracking-models/grayscale_detection_160x160.onnx $D/*.pgm 2>/dev/null > $D.txt
done
