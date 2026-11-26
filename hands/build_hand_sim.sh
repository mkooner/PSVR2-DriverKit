#!/usr/bin/env bash
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
# Build hand_sim: the driver's hand tracking (as driver_ht_replay) + the SteamVR plugin's hand code from PLUGIN_DIR
# (default: the trial tree's; give an older version's directory to replay a session recorded with it).
# OUT names the binary (default ./hand_sim).
set -e
cd "$(dirname "$0")"
R="${MONADO_SRC:-$PWD/../../trial}"; B="$R/build"; L="$B/src/xrt"
PLUGIN_DIR="${PLUGIN_DIR:-$R/src/xrt/state_trackers/steamvr_psvr2}"
OUT="${OUT:-./hand_sim}"
INC="-I$R/src/xrt/include -I$B/src/xrt/include -I$R/src/xrt/auxiliary -I$B/src/xrt/auxiliary -I$R/src/xrt/tracking -I$B/src/xrt/tracking -I$R/src/xrt/tracking/hand/mercury -I$R/src/xrt/drivers/psvr2 -I$R/src/xrt/drivers -I$R/src/external -I$R/src/external/bcdec -I$R/src/external/cjson $(pkg-config --cflags libusb-1.0)"
DEFS="-D_GNU_SOURCE -DXRT_BUILD_DRIVER_PSVR2 -DXRT_BUILD_DRIVER_HANDTRACKING"
T=$(mktemp -d)
gcc -std=gnu11 -O2 $DEFS $INC -c sim_ht_side.c -o $T/sim_ht_side.o
gcc -std=gnu11 -O2 $DEFS $INC -c "$R/src/xrt/drivers/psvr2/psvr2_hand_tracking.c" -o $T/psvr2_hand_tracking.o
gcc -std=gnu11 -O2 $DEFS $INC -c "$R/src/xrt/drivers/psvr2/psvr2_constellation_tracking.c" -o $T/psvr2_constellation_tracking.o
gcc -std=gnu11 -O2 -DHAVE_AUX_TRACKING -c cam_pair_probe_stubs.c -o $T/stubs.o
gcc -O2 -c -I$R/src/external/bcdec bcdec_impl.c -o $T/bcdec_impl.o
g++ -std=c++17 -O2 $DEFS -I"$PLUGIN_DIR" -I$R/src/external/openvr_includes $INC -c hand_sim.cpp -o $T/hand_sim.o
LIBS=""
for l in tracking/hand/libhand_async.a tracking/hand/mercury/libt_ht_mercury.a tracking/hand/mercury/libt_ht_mercury_model.a \
         tracking/hand/mercury/libt_ht_mercury_distorter.a tracking/hand/mercury/kine_lm/libt_ht_mercury_kine_lm.a \
         auxiliary/tracking/libaux_tracking.a auxiliary/onnx/libaux_onnx.a auxiliary/math/libaux_math.a \
         auxiliary/util/libaux_util.a auxiliary/os/libaux_os.a; do
	[ -f "$L/$l" ] && LIBS="$LIBS $L/$l"
done
CJSON=$(find "$B" -name 'libxrt-external-cjson.a' -o -name 'libcjson*.a' 2>/dev/null | head -1)
g++ $T/hand_sim.o $T/sim_ht_side.o $T/psvr2_hand_tracking.o $T/psvr2_constellation_tracking.o $T/stubs.o $T/bcdec_impl.o \
	-o "$OUT" -Wl,--start-group $LIBS $CJSON -Wl,--end-group \
	-lopencv_calib -lopencv_core -lopencv_dnn -lopencv_features -lopencv_flann -lopencv_geometry -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_objdetect -lopencv_stereo -lopencv_video -lopencv_videoio -lonnxruntime -lpthread -lm -ldl
rm -rf $T
echo "built $OUT (plugin code: $PLUGIN_DIR)"
