#!/usr/bin/env bash
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
#
# Install the packages build-driver.sh needs, with the distribution's package manager (Arch based: pacman, Debian
# based: apt, Fedora based: dnf). DriverKit's "Install dependencies" button runs it (through pkexec, which asks for
# your password).
#
#   ./install-deps.sh [--hand-tracking] [--list] [--dry-run]
#
#   --hand-tracking  also what hand tracking needs: OpenCV and onnxruntime (Monado's Mercury hand tracker)
#   --list           print the distribution found and the command, install nothing
#   --dry-run        the same, after checking which packages are missing (no root needed)
#
# It also installs the udev rules that let you (not only root) open the headset and the Sense controllers: our copy
# of Monado's xr-hardware rules (udev/70-xrhardware.rules) as /etc/udev/rules.d/70-xrhardware.rules, then reloads
# and triggers udev, as xr-hardware's own install line does but from this repository instead of a download. Skipped
# when that file is already the same as ours.
#
# Needs root to install: asks through sudo when run from a terminal.
set -euo pipefail
HT=0
LIST=0
DRY=0
for a in "$@"; do
	case "$a" in
	--hand-tracking) HT=1 ;;
	--list) LIST=1 ;;
	--dry-run) DRY=1 ;;
	-h | --help) sed -n '4,20p' "$0"; exit 0 ;;
	*) echo "unknown option $a"; exit 2 ;;
	esac
done

# udev rules: without them only root can open the headset's USB device and the Sense controllers' HID devices, and
# Monado (running as you) finds no controllers and logs nothing about it (a user's 2026-11-23 PC: the controllers
# connected over Bluetooth, no hidraw node was usable, left/right "<none>").
RULES_FILE=/etc/udev/rules.d/70-xrhardware.rules
# Our copy of Monado's xr-hardware rules (udev/70-xrhardware.rules in the repository, next to this script; DriverKit
# writes it beside its copy of this script). Taken unchanged from
# https://gitlab.freedesktop.org/monado/utilities/xr-hardware (commit 143de7a5, 2026-08-09, BSL-1.0): TAG+="uaccess"
# for XR devices, the PS VR2 and both Sense controllers among them; nothing is run.
RULES_SRC="$(cd "$(dirname "$0")" && pwd)/udev/70-xrhardware.rules"
RULES_NEEDED=1
RULES_HAVE=""
if [ -r "$RULES_FILE" ] && [ -r "$RULES_SRC" ] && cmp -s "$RULES_SRC" "$RULES_FILE"; then
	RULES_NEEDED=0
	RULES_HAVE="$RULES_FILE, the same as ours"
fi
install_udev_rules() {
	if [ ! -r "$RULES_SRC" ]; then
		echo "ERROR: the udev rules aren't next to this script ($RULES_SRC): get the whole PSVR2-DriverKit folder"
		return 1
	fi
	mkdir -p /etc/udev/rules.d
	install -m 644 "$RULES_SRC" "$RULES_FILE"
	if command -v udevadm >/dev/null; then
		udevadm control --reload-rules
		udevadm trigger
	fi
	echo "  udev rules installed: $RULES_FILE (reconnect the controllers, or turn them off and on, if they were on)"
}

NAME="your system"
if [ -r /etc/os-release ]; then
	# shellcheck disable=SC1091
	NAME="$(. /etc/os-release && echo "${PRETTY_NAME:-${NAME:-your system}}")"
fi

# The compilers, CMake/Ninja, the Monado build's libraries and Rust (DriverKit).
ARCH_BASE=(base-devel git cmake ninja python pkgconf eigen vulkan-headers vulkan-icd-loader glslang shaderc libusb
	systemd-libs libjpeg-turbo ceres-solver suitesparse google-glog gflags jsoncpp libx11 libxcb libxrandr wayland
	wayland-protocols libdrm zlib patchelf libcap rust)
DEB_BASE=(build-essential git cmake ninja-build python3 pkg-config libeigen3-dev libvulkan-dev glslang-tools glslc
	libusb-1.0-0-dev libudev-dev libjpeg-dev libceres-dev libsuitesparse-dev libgoogle-glog-dev libgflags-dev
	libjsoncpp-dev libx11-dev libx11-xcb-dev libxcb-randr0-dev libxrandr-dev libwayland-dev wayland-protocols libdrm-dev
	zlib1g-dev patchelf libcap2-bin cargo)
FED_BASE=(gcc-c++ git cmake ninja-build python3 pkgconf-pkg-config eigen3-devel vulkan-loader-devel vulkan-headers
	glslang glslc libusb1-devel systemd-devel libjpeg-turbo-devel ceres-solver-devel suitesparse-devel glog-devel
	gflags-devel jsoncpp-devel libX11-devel libxcb-devel libXrandr-devel wayland-devel wayland-protocols-devel
	libdrm-devel zlib-devel patchelf libcap cargo)
# Hand tracking. onnxruntime's development files: Arch extra, Debian 13 / Ubuntu 25.04 and newer, Fedora 43 and newer.
ARCH_HT=(opencv onnxruntime-cpu)
DEB_HT=(libopencv-dev libonnxruntime-dev)
FED_HT=(opencv-devel onnxruntime-devel)

if command -v pacman >/dev/null; then
	FAMILY="Arch based"
	PKGS=("${ARCH_BASE[@]}")
	[ "$HT" = 1 ] && PKGS+=("${ARCH_HT[@]}")
	# Only what isn't there yet, by what installed packages provide: rustup already gives cargo, onnxruntime-cuda or
	# -rocm already give onnxruntime (installing rust or onnxruntime-cpu next to them would conflict).
	MISSING=()
	for p in "${PKGS[@]}"; do
		want="$p"
		[ "$p" = onnxruntime-cpu ] && want=onnxruntime
		[ "$p" = rust ] && want=cargo
		pacman -T "$want" >/dev/null 2>&1 || MISSING+=("$p")
	done
	CMD=(pacman -S --needed --noconfirm "${MISSING[@]}")
elif command -v apt-get >/dev/null; then
	FAMILY="Debian based"
	PKGS=("${DEB_BASE[@]}")
	if [ "$HT" = 1 ]; then
		if ! apt-cache show libonnxruntime-dev >/dev/null 2>&1; then
			echo "ERROR: $NAME has no libonnxruntime-dev package: hand tracking needs Debian 13, Ubuntu 25.04 or newer."
			echo "       Untick Compile hand tracking to install what the driver needs without it."
			exit 1
		fi
		PKGS+=("${DEB_HT[@]}")
	fi
	MISSING=("${PKGS[@]}")
	CMD=(apt-get install -y "${PKGS[@]}")
elif command -v dnf >/dev/null; then
	FAMILY="Fedora based"
	PKGS=("${FED_BASE[@]}")
	if [ "$HT" = 1 ]; then
		if ! dnf -q info onnxruntime-devel >/dev/null 2>&1; then
			echo "ERROR: $NAME has no onnxruntime-devel package: hand tracking needs Fedora 43 or newer."
			echo "       Untick Compile hand tracking to install what the driver needs without it."
			exit 1
		fi
		PKGS+=("${FED_HT[@]}")
	fi
	MISSING=("${PKGS[@]}")
	CMD=(dnf install -y "${PKGS[@]}")
else
	echo "ERROR: no pacman, apt-get or dnf found on $NAME: install the packages by hand (DriverKit's Compile Driver tab"
	echo "       lists them for Arch, Debian/Ubuntu and Fedora)."
	exit 1
fi

echo "== $NAME ($FAMILY)$([ "$HT" = 1 ] && echo ", with hand tracking")"
PKGS_NEEDED=1
[ "$FAMILY" = "Arch based" ] && [ ${#MISSING[@]} -eq 0 ] && PKGS_NEEDED=0
if [ "$LIST" = 1 ] || [ "$DRY" = 1 ]; then
	if [ "$PKGS_NEEDED" = 0 ]; then
		echo "  packages: all installed already"
	else
		echo "  ${CMD[*]}"
	fi
	if [ "$RULES_NEEDED" = 1 ]; then
		echo "  udev rules: would install our copy ($RULES_SRC) as $RULES_FILE"
	else
		echo "  udev rules: installed ($RULES_HAVE)"
	fi
	exit 0
fi
if [ "$PKGS_NEEDED" = 0 ] && [ "$RULES_NEEDED" = 0 ]; then
	echo "  everything is installed (udev rules: $RULES_HAVE)"
	exit 0
fi
if [ "$(id -u)" != 0 ]; then
	echo "  needs root: asking through sudo"
	exec sudo bash "$0" "$@"
fi
if [ "$FAMILY" = "Debian based" ] && [ "$PKGS_NEEDED" = 1 ]; then
	echo "  apt-get update"
	apt-get update
	export DEBIAN_FRONTEND=noninteractive
fi
if [ "$PKGS_NEEDED" = 1 ]; then
	echo "  ${CMD[*]}"
	"${CMD[@]}"
else
	echo "  packages: all installed already"
fi
if [ "$RULES_NEEDED" = 1 ]; then
	install_udev_rules || exit 1
else
	echo "  udev rules: installed ($RULES_HAVE)"
fi
echo "DONE dependencies installed"
