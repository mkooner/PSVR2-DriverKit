#!/bin/sh
# NOTE: the old plugin (steamvr_drv, with ovrd_direct_mode.hpp) was removed on 2026-11-18; this prototype no longer
# builds against the trial tree. Kept for reference.
# Builds the direct mode harness against the trial tree's plugin header.
set -e
cd "$(dirname "$0")"
T=../../trial/src
g++ -std=c++17 -O2 -Wall -o dm_harness dm_harness.cpp -I$T/xrt/state_trackers/steamvr_drv -I$T/external/openvr_includes \
	-I$T/xrt/include -I$T/xrt/auxiliary -I../../trial/build/src/xrt/include \
	../../trial/build/src/xrt/auxiliary/math/libaux_math.a -lvulkan -lpthread
