#!/usr/bin/env bash
# Build the pose simulator against the trial tree's sources and build libraries.
set -e
cd "$(dirname "$0")"
R="${MONADO_SRC:-$PWD/../../trial}"; B="$R/build"
INC="-I$R/src/xrt/include -I$R/src/xrt/auxiliary -I$R/src/xrt/drivers/psvr2 -I$B/src/xrt/include -I/usr/include/eigen3"
gcc -std=c11 -O2 $INC -c "$R/src/xrt/drivers/psvr2/psvr2_pose_filter.c" -o pose_filter.o
g++ -std=c++17 -O2 $INC pose_sim.cpp pose_filter.o -o pose_sim \
	"$B/src/xrt/auxiliary/tracking/libaux_tracking.a" "$B/src/xrt/auxiliary/math/libaux_math.a" \
	"$B/src/xrt/auxiliary/util/libaux_util.a" "$B/src/xrt/auxiliary/os/libaux_os.a" -lpthread -lm
echo built ./pose_sim
