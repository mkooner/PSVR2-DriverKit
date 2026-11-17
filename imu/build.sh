#!/usr/bin/env bash
# Build the IMU re-fusion harness against the trial tree's build.
set -e
cd "$(dirname "$0")"
R="${MONADO_SRC:-$PWD/../../trial}"; B="$R/build"; L="$B/src/xrt"
INC="-I$R/src/xrt/include -I$B/src/xrt/include -I$R/src/xrt/auxiliary -I$R/src/xrt/drivers"
gcc -std=gnu11 -O2 -D_GNU_SOURCE $INC -c refuse.c -o refuse.o
g++ refuse.o -o refuse -Wl,--start-group $L/auxiliary/math/libaux_math.a $L/auxiliary/util/libaux_util.a $L/auxiliary/os/libaux_os.a -Wl,--end-group -lpthread -lm -ldl
rm -f refuse.o
echo built ./refuse
