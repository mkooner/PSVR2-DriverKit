#!/bin/sh
# Builds gesture_probe against the trial tree's SteamVR plugin gesture code (needs trial/build configured).
set -e
cd "$(dirname "$0")"
R=../../trial; L=$R/build/src/xrt
g++ -std=c++17 -O2 -Wall -o gesture_probe gesture_probe.cpp -I$R/src/xrt/state_trackers/steamvr_psvr2 \
	-I$R/src/external/openvr_includes -I$R/src/xrt/auxiliary -I$R/src/xrt/include -I$R/build/src/xrt/include \
	-I$R/src/external/cjson -I$R/src/external \
	$L/auxiliary/util/libaux_util.a $L/auxiliary/math/libaux_math.a $L/auxiliary/os/libaux_os.a -lpthread -lm
echo built ./gesture_probe
# The MediaPipe gesture state machine on HT_REPLAY_GESTURES dumps (header only).
g++ -std=c++20 -O2 -Wall -Wextra -o gesture_mp_probe gesture_mp_probe.cpp -I$R/src/xrt/state_trackers/steamvr_psvr2
echo built ./gesture_mp_probe
