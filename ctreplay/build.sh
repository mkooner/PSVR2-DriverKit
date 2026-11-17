#!/usr/bin/env bash
# Build the constellation tracker dataset player against the trial tree's build.
set -e
cd "$(dirname "$0")"
R="${MONADO_SRC:-$PWD/../../trial}"; B="$R/build"; L="$B/src/xrt"
INC="-I$R/src/xrt/include -I$B/src/xrt/include -I$R/src/xrt/auxiliary -I$R/src/xrt/tracking -I$R/src/xrt/tracking/constellation -I$R/src/xrt/drivers -I$R/src/external -I$R/src/external/cjson -I/usr/include/eigen3"
LIBS="$L/tracking/constellation/libconstellation.a $L/auxiliary/tracking/libaux_tracking.a $L/auxiliary/math/libaux_math.a $L/auxiliary/util/libaux_util.a $L/auxiliary/util/libaux_util_scribble.a $L/auxiliary/util/libaux_util_sink.a $L/auxiliary/os/libaux_os.a"
g++ -std=gnu++20 -O2 -D_GNU_SOURCE -DGFLAGS_IS_A_DLL=0 -DGLOG_USE_GFLAGS -DGLOG_USE_GLOG_EXPORT $INC ct_replay.cpp -o ct_replay -Wl,--start-group $LIBS -Wl,--end-group \
	-lceres -lglog -lgflags -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -ljpeg -lpthread -lm -ldl
echo built ./ct_replay
