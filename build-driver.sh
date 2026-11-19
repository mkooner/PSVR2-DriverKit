#!/usr/bin/env bash
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
#
# Compile the PS VR2 driver and install everything PSVR2-DriverKit needs into one folder, from our GitHub repository
# (https://github.com/mkooner/PSVR2-DriverKit, cloned or updated into <dest>/source; the source isn't selectable):
#
#   ./build-driver.sh [--dest <install folder>]
#
#   <dest>/source                        the repository (main branch): the Monado fork in monado/, DriverKit, the kit
#   <dest>/build                         the Monado build: the SteamVR plugin (build/steamvr-monado-psvr2, with the
#                                        libraries Steam's runtime lacks bundled next to it, so SteamVR can load it
#                                        inside Steam's container too), monado-service and the OpenXR runtime
#                                        (build/openxr_monado-dev.json)
#   <dest>/kit                           DriverKit (built from the repository), the launch scripts and tools
#   ~/.config/PSVR2-DriverKit/driver.env where it went: the launch scripts and DriverKit read it, so everything that
#                                        uses the driver finds this build
#
# Default dest: ~/.local/share/PSVR2-DriverKit. Progress: "STEP n/5" lines and ninja's "[done/total]" (DriverKit's
# progress bar reads both). The repository is private: git uses your GitHub login (git credential helper / gh).
# SteamVR must be closed for the install step: it would still have the old plugin loaded.

set -euo pipefail

REPO_URL="https://github.com/mkooner/PSVR2-DriverKit.git"
REPO_BRANCH="main"
DEST="${HOME}/.local/share/PSVR2-DriverKit"
while [ $# -gt 0 ]; do
	case "$1" in
	--dest) DEST="$2"; shift 2 ;;
	-h | --help) sed -n '4,22p' "$0"; exit 0 ;;
	*) echo "unknown option $1"; exit 2 ;;
	esac
done
mkdir -p "$DEST"
DEST="$(cd "$DEST" && pwd)"
REPO="$DEST/source"
SRC="$REPO/monado"
HERE="$REPO"
BUILD="$DEST/build"
KIT="$DEST/kit"
CONF_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/PSVR2-DriverKit"

echo "== PSVR2-DriverKit driver build"
echo "  source:  $REPO_URL ($REPO_BRANCH)"
echo "  install: $DEST"

for tool in git cmake ninja python3 g++ cargo; do
	command -v "$tool" >/dev/null || { echo "ERROR: $tool not installed (see the package list on DriverKit's Compile Driver tab)"; exit 1; }
done

echo "STEP 1/5 download the source"
export GIT_TERMINAL_PROMPT=0
# The repository is private: when the GitHub CLI is signed in, git gets the login from it for these commands only
# (your global git config is left as it is).
GIT_AUTH=()
if command -v gh >/dev/null && gh auth status >/dev/null 2>&1; then
	GIT_AUTH=(-c credential.helper= -c "credential.helper=!gh auth git-credential")
fi
NO_LOGIN="sign in to GitHub first: gh auth login (or set up a git credential helper)"
if [ -d "$REPO/.git" ]; then
	if ! git "${GIT_AUTH[@]}" -C "$REPO" fetch --depth 1 origin "$REPO_BRANCH" >"$DEST/download.log" 2>&1 ||
		! git -C "$REPO" reset --hard FETCH_HEAD >>"$DEST/download.log" 2>&1; then
		tail -3 "$DEST/download.log"
		echo "ERROR: couldn't update $REPO from GitHub: $NO_LOGIN"
		exit 1
	fi
else
	if [ -e "$REPO" ] && [ -n "$(ls -A "$REPO" 2>/dev/null)" ]; then
		echo "ERROR: $REPO exists and isn't our repository: move it away or pick another install location"
		exit 1
	fi
	rmdir "$REPO" 2>/dev/null || true
	if ! git "${GIT_AUTH[@]}" clone --depth 1 --branch "$REPO_BRANCH" "$REPO_URL" "$REPO" >"$DEST/download.log" 2>&1; then
		tail -3 "$DEST/download.log"
		echo "ERROR: couldn't download $REPO_URL: $NO_LOGIN"
		exit 1
	fi
fi
echo "  $(git -C "$REPO" log -1 --format='commit %h, %cd: %s' --date=format:'%Y-%m-%d %H:%M')"
[ -f "$SRC/CMakeLists.txt" ] || { echo "ERROR: no Monado source in $SRC"; exit 1; }

echo "STEP 2/5 configure"
# Only the PS VR2 drivers (headset, Sense controllers) plus the simulated ones the tests use; OpenCV, GStreamer, onnx,
# SDL2, the debug GUI and Basalt off, so the plugin's libraries are few enough to bundle (bundle_libs.py) and SteamVR
# can load it inside Steam's runtime container as well as on the host. monado-service stays on (OpenXR games).
OFF=(-DXRT_HAVE_OPENCV=OFF -DXRT_HAVE_GST=OFF -DXRT_HAVE_ONNXRUNTIME=OFF -DXRT_HAVE_SDL2=OFF
	-DXRT_FEATURE_DEBUG_GUI=OFF -DXRT_FEATURE_SLAM=OFF -DXRT_FEATURE_RENDERDOC=OFF
	-DXRT_HAVE_BLUETOOTH=OFF -DXRT_HAVE_HIDAPI=OFF)
cmake -S "$SRC" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
	-DXRT_FEATURE_STEAMVR_PLUGIN=ON -DXRT_FEATURE_SERVICE=ON -DXRT_FEATURE_OPENXR=ON \
	-DXRT_BUILD_DRIVER_PSVR2=ON -DXRT_BUILD_DRIVER_PSSENSE=ON -DXRT_BUILD_DRIVER_SIMULATED=ON \
	"${OFF[@]}" -DOVRN_BUNDLE_LIBS=ON -DOVRN_STEAM_RUNTIME=auto >"$DEST/configure.log" 2>&1 || {
	tail -30 "$DEST/configure.log"
	echo "ERROR: configure failed (log: $DEST/configure.log)"
	exit 1
}

echo "STEP 3/5 compile"
set +e
NINJA_STATUS="[%f/%t] " ninja -C "$BUILD" 2>&1 | tee "$DEST/compile.log" | while IFS= read -r line; do
	case "$line" in
	"["[0-9]*/[0-9]*"]"*) printf '%s\n' "${line:0:160}" ;;
	*error* | *FAILED*) printf '%s\n' "$line" ;;
	esac
done
rc=${PIPESTATUS[0]}
set -e
[ "$rc" = 0 ] || { echo "ERROR: compile failed (log: $DEST/compile.log)"; exit 1; }
PLUGIN="$BUILD/steamvr-monado-psvr2"
[ -f "$PLUGIN/bin/linux64/driver_monado_psvr2.so" ] || { echo "ERROR: the SteamVR plugin wasn't built"; exit 1; }
[ -x "$BUILD/src/xrt/targets/service/monado-service" ] || { echo "ERROR: monado-service wasn't built"; exit 1; }

echo "STEP 4/5 DriverKit, scripts and tools"
# Sony's Sense render models and PS VR2 icons, from the PlayStation VR2 App if it's installed in one of Steam's
# libraries: copied into the driver's resources (they're Sony's files, so they aren't in the repository). The driver
# uses them when they're there; without them SteamVR shows Oculus controllers and generic icons.
SONY_RES=""
STEAM_ROOT="${STEAM_ROOT:-$HOME/.local/share/Steam}"
LIBS=("$STEAM_ROOT" "$HOME/.steam/steam")
if [ -f "$STEAM_ROOT/steamapps/libraryfolders.vdf" ]; then
	while IFS= read -r lib; do LIBS+=("$lib"); done < <(sed -n 's/^[[:space:]]*"path"[[:space:]]*"\(.*\)"/\1/p' "$STEAM_ROOT/steamapps/libraryfolders.vdf")
fi
for lib in "${LIBS[@]}"; do
	r="$lib/steamapps/common/PlayStation VR2 App/SteamVR_Plug-In/resources"
	[ -d "$r/rendermodels/playstation_vr2_sense_left" ] && SONY_RES="$r" && break
done
if [ -n "$SONY_RES" ]; then
	mkdir -p "$PLUGIN/resources/rendermodels" "$PLUGIN/resources/icons"
	for side in left right; do
		rm -rf "$PLUGIN/resources/rendermodels/playstation_vr2_sense_$side"
		cp -r "$SONY_RES/rendermodels/playstation_vr2_sense_$side" "$PLUGIN/resources/rendermodels/"
	done
	rm -rf "$PLUGIN/resources/icons/playstation_vr2"
	cp -r "$SONY_RES/icons" "$PLUGIN/resources/icons/playstation_vr2"
	echo "  PS VR2 render models and icons copied from the PlayStation VR2 App"
else
	echo "  note: PlayStation VR2 App not found in Steam's libraries: SteamVR will show generic controller models and icons"
fi
mkdir -p "$KIT/tools" "$KIT/analysis" "$KIT/logs"
for f in launch-steamvr.sh launch-openxr.sh summarize.sh build-driver.sh README.md ROOM.md; do
	[ -f "$HERE/$f" ] && cp -f "$HERE/$f" "$KIT/"
done
for f in steamvr_frame_shift.py steamvr_probe.py steamvr_timeline.py; do
	[ -f "$HERE/analysis/$f" ] && cp -f "$HERE/analysis/$f" "$KIT/analysis/"
done
[ -f "$HERE/tools/cpusample.py" ] && cp -f "$HERE/tools/cpusample.py" "$KIT/tools/"
[ -d "$HERE/tools/xr_frame_log" ] && cp -rf "$HERE/tools/xr_frame_log" "$KIT/tools/"
# The OpenVR tools (background clients: room setup, probes, timing): built against SteamVR's own libopenvr_api.
STEAMVR_DIR="${STEAMVR_DIR:-$HOME/.local/share/Steam/steamapps/common/SteamVR}"
HDR="$SRC/src/external/openvr_includes"
for t in vrlaunch vrprobe vrframetiming vrposelog vrposeprobe vrdisplaydump; do
	[ -f "$HERE/tools/$t.cpp" ] || continue
	cp -f "$HERE/tools/$t.cpp" "$KIT/tools/"
	if [ -f "$STEAMVR_DIR/bin/linux64/libopenvr_api.so" ] &&
		g++ -O2 -o "$KIT/tools/$t" "$HERE/tools/$t.cpp" -I"$HDR" -L"$STEAMVR_DIR/bin/linux64" -lopenvr_api \
			-Wl,-rpath,"$STEAMVR_DIR/bin/linux64" -lpthread 2>"$KIT/tools/$t.build.log"; then
		rm -f "$KIT/tools/$t.build.log"
	elif [ -x "$HERE/tools/$t" ]; then
		cp -f "$HERE/tools/$t" "$KIT/tools/"
	else
		echo "  note: tools/$t not built (see $KIT/tools/$t.build.log)"
	fi
done
if [ -f "$REPO/launcher-gui/Cargo.toml" ]; then
	CARGO_TARGET_DIR="$DEST/cargo-target" cargo build --release --manifest-path "$REPO/launcher-gui/Cargo.toml" \
		>"$DEST/driverkit-build.log" 2>&1 || { tail -20 "$DEST/driverkit-build.log"; echo "ERROR: DriverKit didn't build (log: $DEST/driverkit-build.log)"; exit 1; }
	cp -f "$DEST/cargo-target/release/PSVR2-DriverKit" "$KIT/"
fi

echo "STEP 5/5 record the install location"
mkdir -p "$CONF_DIR"
cat >"$CONF_DIR/driver.env" <<EOF
# Written by build-driver.sh $(date '+%Y-%m-%d %H:%M'): where the PS VR2 driver was compiled to. Read by
# launch-steamvr.sh, launch-openxr.sh and PSVR2-DriverKit (unless MONADO_BUILD is given).
PSVR2_DRIVERKIT_HOME="$DEST"
MONADO_BUILD="$BUILD"
PSVR2_PLUGIN="$PLUGIN"
PSVR2_KIT="$KIT"
MONADO_SOURCE="$SRC"
EOF
echo "  $CONF_DIR/driver.env"
echo "DONE plugin: $PLUGIN"
echo "DONE monado-service: $BUILD/src/xrt/targets/service/monado-service"
echo "DONE OpenXR runtime: $BUILD/openxr_monado-dev.json"
echo "DONE kit: $KIT"
