#!/usr/bin/env bash
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
#
# Compile the PS VR2 driver and install everything PSVR2-DriverKit needs into one folder, from our GitHub repository
# (https://github.com/mkooner/PSVR2-DriverKit, cloned or updated into <dest>/source; the source isn't selectable):
#
#   ./build-driver.sh [--dest <install folder>] [--hand-tracking] [--ref <release>]
#
#   --ref <release>  build a release (a tag such as v0.2) instead of the latest on the main branch ("nightly", the
#                    default; --ref nightly is the same). The release's own scripts and DriverKit are installed with
#                    it; DriverKit's Compile Driver in that older version builds the latest again.
#   --hand-tracking  build with Monado's Mercury hand tracking (experimental): needs OpenCV and onnxruntime
#                    (install-deps.sh --hand-tracking), and downloads its two models into
#                    ~/.local/share/monado/hand-tracking-models
#
#   <dest>/source                        the repository (main branch): the Monado fork in monado/, DriverKit, the kit
#   <dest>/build                         the Monado build: the SteamVR plugin (build/steamvr-monado-psvr2, with the
#                                        libraries Steam's runtime lacks bundled next to it, so SteamVR can load it
#                                        inside Steam's container too), monado-service and the OpenXR runtime
#                                        (build/openxr_monado-dev.json)
#   <dest>/kit                           DriverKit (built from the repository), the launch scripts and tools
#   ~/.config/PSVR2-DriverKit/driver.env where it went: the launch scripts and DriverKit read it, so everything that
#                                        uses the driver finds this build; also which commit was built (SOURCE_COMMIT,
#                                        SOURCE_DATE: DriverKit's Check compares it with GitHub)
#
# Default dest: ~/.local/share/PSVR2-DriverKit. Progress: "STEP n/5" lines and ninja's "[done/total]" (DriverKit's
# progress bar reads both). The repository is public; a signed-in GitHub CLI (gh) is used if there is one.
# SteamVR must be closed for the install step: it would still have the old plugin loaded.

set -euo pipefail

REPO_URL="https://github.com/mkooner/PSVR2-DriverKit.git"
REPO_BRANCH="main"
REPO_REF="" # a release tag, or empty for nightly (the main branch)
DEST="${HOME}/.local/share/PSVR2-DriverKit"
HAND_TRACKING=0
ORIG_ARGS=("$@")
while [ $# -gt 0 ]; do
	case "$1" in
	--dest) DEST="$2"; shift 2 ;;
	--hand-tracking) HAND_TRACKING=1; shift ;;
	--ref) REPO_REF="$2"; shift 2 ;;
	-h | --help) sed -n '4,30p' "$0"; exit 0 ;;
	*) echo "unknown option $1"; exit 2 ;;
	esac
done
[ "$REPO_REF" = nightly ] && REPO_REF=""
# Release tags only (v0.2, v1.0-rc1): nothing that git could read as an option or a path.
if [ -n "$REPO_REF" ] && ! [[ "$REPO_REF" =~ ^v[0-9][0-9A-Za-z._-]*$ ]]; then
	echo "ERROR: --ref takes a release tag such as v0.2 (or nightly), not '$REPO_REF'"
	exit 2
fi
mkdir -p "$DEST"
DEST="$(cd "$DEST" && pwd)"
REPO="$DEST/source"
SRC="$REPO/monado"
HERE="$REPO"
BUILD="$DEST/build"
KIT="$DEST/kit"
CONF_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/PSVR2-DriverKit"

echo "== PSVR2-DriverKit driver build"
if [ -n "$REPO_REF" ]; then
	echo "  source:  $REPO_URL (release $REPO_REF)"
else
	echo "  source:  $REPO_URL (nightly: the latest on $REPO_BRANCH)"
fi
echo "  install: $DEST"
echo "  hand tracking: $([ "$HAND_TRACKING" = 1 ] && echo "yes (experimental)" || echo no)"

TOOLS=(git cmake ninja python3 g++ cargo)
[ "$HAND_TRACKING" = 1 ] && TOOLS+=(curl sha256sum)
for tool in "${TOOLS[@]}"; do
	command -v "$tool" >/dev/null || { echo "ERROR: $tool not installed: press Install dependencies on DriverKit's Compile Driver tab"; exit 1; }
done

echo "STEP 1/5 download the source"
export GIT_TERMINAL_PROMPT=0
# The repository is public (no login needed); when the GitHub CLI is signed in, git uses its login for these commands only
# (your global git config is left as it is).
GIT_AUTH=()
if command -v gh >/dev/null && gh auth status >/dev/null 2>&1; then
	GIT_AUTH=(-c credential.helper= -c "credential.helper=!gh auth git-credential")
fi
NO_LOGIN="couldn't reach GitHub: check your internet connection"
# What to fetch: the main branch, or the release's tag (the same shallow fetch and hard reset either way, so switching
# between nightly and a release in the same install folder just moves the source to it).
FETCH_REF="$REPO_BRANCH"
[ -n "$REPO_REF" ] && FETCH_REF="refs/tags/$REPO_REF"
if [ -n "$REPO_REF" ] && ! git "${GIT_AUTH[@]}" ls-remote --exit-code --tags "$REPO_URL" "$FETCH_REF" >/dev/null 2>&1; then
	echo "ERROR: no release $REPO_REF on GitHub (or $NO_LOGIN)"
	exit 1
fi
if [ -d "$REPO/.git" ]; then
	if ! git "${GIT_AUTH[@]}" -C "$REPO" fetch --depth 1 origin "$FETCH_REF" >"$DEST/download.log" 2>&1 ||
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
	if ! git "${GIT_AUTH[@]}" clone --depth 1 --branch "${REPO_REF:-$REPO_BRANCH}" "$REPO_URL" "$REPO" >"$DEST/download.log" 2>&1; then
		tail -3 "$DEST/download.log"
		echo "ERROR: couldn't download $REPO_URL: $NO_LOGIN"
		exit 1
	fi
fi
echo "  $(git -C "$REPO" log -1 --format='commit %h, %cd: %s' --date=format:'%Y-%m-%d %H:%M')"
[ -f "$SRC/CMakeLists.txt" ] || { echo "ERROR: no Monado source in $SRC"; exit 1; }

# DriverKit runs the copy of this script built into it, which can be older than the one just downloaded (a DriverKit
# built before a build fix kept failing with the fix already downloaded): hand over to the downloaded copy when it
# differs, so build fixes reach older DriverKits. Once only: PSVR2_DK_HANDED_OVER stops a loop. Not for a release:
# its own script is older than this one and doesn't know --ref (v0.1's not even --hand-tracking), so this one builds it.
if [ -z "$REPO_REF" ] && [ -z "${PSVR2_DK_HANDED_OVER:-}" ] && [ -f "$REPO/build-driver.sh" ] &&
	! cmp -s "$0" "$REPO/build-driver.sh"; then
	echo "  this DriverKit's build script is older than the downloaded one: using the downloaded one"
	# Without --ref (this is nightly anyway): a downloaded script older than --ref would stop at the unknown option.
	HANDOVER_ARGS=()
	skip=0
	for a in "${ORIG_ARGS[@]}"; do
		if [ "$skip" = 1 ]; then skip=0; continue; fi
		if [ "$a" = --ref ]; then skip=1; continue; fi
		HANDOVER_ARGS+=("$a")
	done
	PSVR2_DK_HANDED_OVER=1 exec bash "$REPO/build-driver.sh" "${HANDOVER_ARGS[@]}" --dest "$DEST"
fi

echo "STEP 2/5 configure"
# Only the PS VR2 drivers (headset, Sense controllers) plus the simulated ones the tests use; OpenCV, GStreamer, onnx,
# SDL2, the debug GUI and Basalt off, so the plugin's libraries are few enough to bundle (bundle_libs.py) and SteamVR
# can load it inside Steam's runtime container as well as on the host. monado-service stays on (OpenXR games).
# RealSense off too: the PS VR2 doesn't use it, and with a librealsense installed that lacks
# rs2::device::get_device_time_ms() (Monado's RealSense driver calls it) the build failed.
# With --hand-tracking, OpenCV and onnxruntime are on for Mercury (their libraries are bundled too, several hundred MB more with
# Arch's OpenCV); the EuRoC dataset driver OpenCV would also bring in stays off.
OFF=(-DXRT_HAVE_GST=OFF -DXRT_HAVE_SDL2=OFF
	-DXRT_FEATURE_DEBUG_GUI=OFF -DXRT_FEATURE_SLAM=OFF -DXRT_FEATURE_RENDERDOC=OFF
	-DXRT_HAVE_BLUETOOTH=OFF -DXRT_HAVE_HIDAPI=OFF -DXRT_HAVE_REALSENSE=OFF -DXRT_BUILD_DRIVER_EUROC=OFF)
if [ "$HAND_TRACKING" = 1 ]; then
	OFF+=(-DXRT_HAVE_OPENCV=ON -DXRT_HAVE_ONNXRUNTIME=ON)
else
	OFF+=(-DXRT_HAVE_OPENCV=OFF -DXRT_HAVE_ONNXRUNTIME=OFF)
fi
cmake -S "$SRC" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
	-DXRT_FEATURE_STEAMVR_PLUGIN=ON -DXRT_FEATURE_SERVICE=ON -DXRT_FEATURE_OPENXR=ON \
	-DXRT_BUILD_DRIVER_PSVR2=ON -DXRT_BUILD_DRIVER_PSSENSE=ON -DXRT_BUILD_DRIVER_SIMULATED=ON \
	"${OFF[@]}" -DOVRN_BUNDLE_LIBS=ON -DOVRN_STEAM_RUNTIME=auto >"$DEST/configure.log" 2>&1 || {
	tail -30 "$DEST/configure.log"
	echo "ERROR: configure failed (log: $DEST/configure.log)"
	exit 1
}
if [ "$HAND_TRACKING" = 1 ] && ! grep -q "^XRT_BUILD_DRIVER_HANDTRACKING:BOOL=ON" "$BUILD/CMakeCache.txt"; then
	grep -E "#    (OPENCV|ONNXRUNTIME|DRIVER_HANDTRACKING):" "$DEST/configure.log" || true
	echo "ERROR: hand tracking needs OpenCV and onnxruntime: tick Compile hand tracking, press Install dependencies, then compile again"
	exit 1
fi

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
for f in launch-steamvr.sh launch-openxr.sh summarize.sh build-driver.sh install-deps.sh README.md ROOM.md; do
	[ -f "$HERE/$f" ] && cp -f "$HERE/$f" "$KIT/"
done
# install-deps.sh installs our copy of the udev rules from udev/ next to itself.
if [ -f "$HERE/udev/70-xrhardware.rules" ]; then
	mkdir -p "$KIT/udev"
	cp -f "$HERE/udev/70-xrhardware.rules" "$KIT/udev/"
fi
for f in steamvr_frame_shift.py steamvr_probe.py steamvr_timeline.py; do
	[ -f "$HERE/analysis/$f" ] && cp -f "$HERE/analysis/$f" "$KIT/analysis/"
done
[ -f "$HERE/tools/cpusample.py" ] && cp -f "$HERE/tools/cpusample.py" "$KIT/tools/"
[ -f "$HERE/tools/gesture_prompts.py" ] && cp -f "$HERE/tools/gesture_prompts.py" "$KIT/tools/"
[ -d "$HERE/tools/xr_frame_log" ] && cp -rf "$HERE/tools/xr_frame_log" "$KIT/tools/"
# The OpenVR tools (background clients: room setup, probes, timing): built against SteamVR's own libopenvr_api.
STEAMVR_DIR="${STEAMVR_DIR:-$HOME/.local/share/Steam/steamapps/common/SteamVR}"
HDR="$SRC/src/external/openvr_includes"
for t in vrlaunch vrprobe vrframetiming vrposelog vrhandlog vrposeprobe vrdisplaydump; do
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

if [ "$HAND_TRACKING" = 1 ]; then
	# Mercury's two models (Monado's hand-tracking-models, master), where Monado looks for them; checked by hash.
	MODELS="${XDG_DATA_HOME:-$HOME/.local/share}/monado/hand-tracking-models"
	MODELS_URL="https://gitlab.freedesktop.org/monado/utilities/hand-tracking-models/-/raw/master"
	mkdir -p "$MODELS"
	for m in "grayscale_detection_160x160.onnx 1f1a039a266e13dc186bb884430ebd9c8216bdda680ab08a533d4c671f27ed36" \
		"grayscale_keypoint_jan18.onnx 40c0daa598cedb993b54fff17685231b7465d6db342656c401b01c2029efd1d5"; do
		f="${m% *}"
		sum="${m#* }"
		if [ -f "$MODELS/$f" ] && [ "$(sha256sum "$MODELS/$f" | cut -d' ' -f1)" = "$sum" ]; then
			continue
		fi
		if ! curl -sfL -o "$MODELS/$f.part" "$MODELS_URL/$f" ||
			[ "$(sha256sum "$MODELS/$f.part" | cut -d' ' -f1)" != "$sum" ]; then
			rm -f "$MODELS/$f.part"
			echo "ERROR: couldn't download the hand tracking model $f from $MODELS_URL"
			exit 1
		fi
		mv -f "$MODELS/$f.part" "$MODELS/$f"
		echo "  hand tracking model $f downloaded into $MODELS"
	done
	# MediaPipe's hand models (in our repository, models/mediapipe; see its NOTICE): the driver uses them when present.
	cp -f "$REPO"/models/mediapipe/*.onnx "$MODELS/" 2>/dev/null && echo "  MediaPipe hand models copied into $MODELS"
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
SOURCE_COMMIT="$(git -C "$REPO" rev-parse HEAD)"
SOURCE_DATE="$(git -C "$REPO" log -1 --format=%cI)"
SOURCE_REF="${REPO_REF:-nightly}"
HAND_TRACKING_BUILT="$HAND_TRACKING"
EOF
echo "  $CONF_DIR/driver.env"
echo "DONE plugin: $PLUGIN"
echo "DONE monado-service: $BUILD/src/xrt/targets/service/monado-service"
echo "DONE OpenXR runtime: $BUILD/openxr_monado-dev.json"
echo "DONE kit: $KIT"
