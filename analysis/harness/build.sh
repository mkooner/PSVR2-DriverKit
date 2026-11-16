#!/usr/bin/env bash
# Builds the camera-chain harness against the trial worktree's Monado math library.
set -e
T=${MONADO:-$HOME/Documents/Claude/Monado/trial}
cd "$(dirname "$0")"
gcc -c chain.c -o chain.o -I"$T/src/xrt/include" -I"$T/src/xrt/auxiliary" -I"$T/build/src/xrt/include"
g++ chain.o -o chain "$T/build/src/xrt/auxiliary/math/libaux_math.a" "$T/build/src/xrt/auxiliary/util/libaux_util.a" -lm
echo built "$(pwd)/chain"
