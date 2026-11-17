#!/usr/bin/env bash
# Build the LED sync calibration simulator against the trial tree (compiles the real t_led_sync_refinement.c).
set -e
cd "$(dirname "$0")"
R="${MONADO_SRC:-$PWD/../../trial}"; B="$R/build"; L="$B/src/xrt"
INC="-I$R/src/xrt/include -I$B/src/xrt/include -I$R/src/xrt/auxiliary -I$R/src/xrt/tracking -I$R/src/xrt/tracking/constellation -I$R/src/xrt/tracking/constellation/led_sync -I$R/src/external"
gcc -std=gnu11 -O2 -D_GNU_SOURCE $INC -c calib_sim.c -o calib_sim.o
g++ calib_sim.o -o calib_sim -Wl,--start-group $L/auxiliary/math/libaux_math.a $L/auxiliary/util/libaux_util.a $L/auxiliary/os/libaux_os.a -Wl,--end-group -lpthread -lm -ldl
rm -f calib_sim.o
echo built ./calib_sim
