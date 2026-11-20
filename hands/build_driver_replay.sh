#!/usr/bin/env bash
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
# Build driver_ht_replay: the driver's own hand tracking (psvr2_hand_tracking.c) and calibration parser
# (psvr2_constellation_tracking.c) against the trial tree's build, Mercury and its libraries.
set -e
cd "$(dirname "$0")"
R="${MONADO_SRC:-$PWD/../../trial}"; B="$R/build"; L="$B/src/xrt"
INC="-I$R/src/xrt/include -I$B/src/xrt/include -I$R/src/xrt/auxiliary -I$B/src/xrt/auxiliary -I$R/src/xrt/tracking -I$B/src/xrt/tracking -I$R/src/xrt/drivers/psvr2 -I$R/src/xrt/drivers -I$R/src/external -I$R/src/external/bcdec -I$R/src/external/cjson $(pkg-config --cflags libusb-1.0)"
DEFS="-D_GNU_SOURCE -DXRT_BUILD_DRIVER_PSVR2 -DXRT_BUILD_DRIVER_HANDTRACKING"
gcc -std=gnu11 -O2 $DEFS $INC -c driver_ht_replay.c -o driver_ht_replay.o
gcc -std=gnu11 -O2 $DEFS $INC -c "$R/src/xrt/drivers/psvr2/psvr2_hand_tracking.c" -o psvr2_hand_tracking.o
gcc -std=gnu11 -O2 $DEFS $INC -c "$R/src/xrt/drivers/psvr2/psvr2_constellation_tracking.c" -o psvr2_constellation_tracking.o
gcc -std=gnu11 -O2 -DHAVE_AUX_TRACKING -c cam_pair_probe_stubs.c -o cam_pair_probe_stubs.o
gcc -O2 -c -I$R/src/external/bcdec bcdec_impl.c -o bcdec_impl.o
LIBS=""
for l in tracking/hand/libhand_async.a tracking/hand/mercury/libt_ht_mercury.a tracking/hand/mercury/libt_ht_mercury_model.a \
         tracking/hand/mercury/libt_ht_mercury_distorter.a tracking/hand/mercury/kine_lm/libt_ht_mercury_kine_lm.a \
         auxiliary/tracking/libaux_tracking.a auxiliary/onnx/libaux_onnx.a auxiliary/math/libaux_math.a \
         auxiliary/util/libaux_util.a auxiliary/os/libaux_os.a; do
	[ -f "$L/$l" ] && LIBS="$LIBS $L/$l"
done
g++ driver_ht_replay.o psvr2_hand_tracking.o psvr2_constellation_tracking.o cam_pair_probe_stubs.o bcdec_impl.o \
	-o driver_ht_replay -Wl,--start-group $LIBS -Wl,--end-group \
	-lopencv_calib -lopencv_core -lopencv_dnn -lopencv_features -lopencv_flann -lopencv_geometry -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_objdetect -lopencv_stereo -lopencv_video -lopencv_videoio  -lonnxruntime -lpthread -lm -ldl
echo built ./driver_ht_replay
