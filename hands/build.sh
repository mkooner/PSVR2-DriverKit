#!/usr/bin/env bash
# Build the offline Mercury hand tracking prototype against the trial tree's build.
set -e
cd "$(dirname "$0")"
R="${MONADO_SRC:-$PWD/../../trial}"; B="$R/build"
INC="-I$R/src/xrt/include -I$B/src/xrt/include -I$R/src/xrt/auxiliary -I$R/src/xrt/tracking -I$R/src/xrt/tracking/hand/mercury -I$R/src/external/bcdec -I$R/src/external/cjson -I$R/src/external -I/usr/include/eigen3"
LIBS=$(for l in tracking/hand/mercury/libt_ht_mercury.a tracking/hand/mercury/libt_ht_mercury_model.a \
                tracking/hand/mercury/libt_ht_mercury_distorter.a tracking/hand/mercury/kine_lm/libt_ht_mercury_kine_lm.a \
                auxiliary/tracking/libaux_tracking.a auxiliary/onnx/libaux_onnx.a auxiliary/math/libaux_math.a \
                auxiliary/util/libaux_util.a auxiliary/os/libaux_os.a; do echo "$B/src/xrt/$l"; done)
gcc -O2 -c -I$R/src/external/bcdec bcdec_impl.c -o bcdec_impl.o
g++ -std=c++17 -O2 -D_GNU_SOURCE $INC hand_replay.cpp bcdec_impl.o -o hand_replay -Wl,--start-group $LIBS -Wl,--end-group \
	-lopencv_core -lopencv_imgproc -lonnxruntime -lpthread -lm -ldl
echo built ./hand_replay
