#!/usr/bin/env bash
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
#
# Builds PSVR2-DriverKit-x86_64.AppImage: the DriverKit app (it carries build-driver.sh, so it can compile and install
# the driver, Monado and a local DriverKit by itself) with AppRun, which starts the player's locally compiled DriverKit
# once there is one (see AppRun).
#
#   appimage/build-appimage.sh            -> dist/PSVR2-DriverKit-x86_64.AppImage
#
# appimagetool is downloaded once into appimage/.tools (official release, github.com/AppImage/appimagetool).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TOOLS="$HERE/.tools"
OUT="$ROOT/dist"
APPDIR="$(mktemp -d)/PSVR2-DriverKit.AppDir"
trap 'rm -rf "$(dirname "$APPDIR")"' EXIT

echo "== Building DriverKit"
(cd "$ROOT/launcher-gui" && cargo build --release)
BIN="$ROOT/launcher-gui/target/release/PSVR2-DriverKit"

echo "== Assembling the AppDir"
mkdir -p "$APPDIR/usr/bin" "$APPDIR/usr/share/applications" "$APPDIR/usr/share/icons/hicolor/256x256/apps"
install -m 755 "$BIN" "$APPDIR/usr/bin/PSVR2-DriverKit"
install -m 755 "$HERE/AppRun" "$APPDIR/AppRun"
install -m 644 "$HERE/PSVR2-DriverKit.desktop" "$APPDIR/PSVR2-DriverKit.desktop"
install -m 644 "$HERE/PSVR2-DriverKit.desktop" "$APPDIR/usr/share/applications/PSVR2-DriverKit.desktop"
install -m 644 "$ROOT/launcher-gui/assets/headset.png" "$APPDIR/psvr2-driverkit.png"
install -m 644 "$ROOT/launcher-gui/assets/headset.png" "$APPDIR/usr/share/icons/hicolor/256x256/apps/psvr2-driverkit.png"
install -m 644 "$ROOT/launcher-gui/assets/headset.png.license" "$APPDIR/psvr2-driverkit.png.license"

TOOL="$TOOLS/appimagetool-x86_64.AppImage"
if [ ! -x "$TOOL" ]; then
	echo "== Downloading appimagetool"
	mkdir -p "$TOOLS"
	curl -fL -o "$TOOL.part" "https://github.com/AppImage/appimagetool/releases/download/continuous/appimagetool-x86_64.AppImage"
	chmod +x "$TOOL.part" && mv "$TOOL.part" "$TOOL"
fi

echo "== Packing"
mkdir -p "$OUT"
# APPIMAGE_EXTRACT_AND_RUN: appimagetool runs without FUSE
ARCH=x86_64 APPIMAGE_EXTRACT_AND_RUN=1 "$TOOL" --no-appstream "$APPDIR" "$OUT/PSVR2-DriverKit-x86_64.AppImage"
echo "== $OUT/PSVR2-DriverKit-x86_64.AppImage"
