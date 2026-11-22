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
	-h | --help) sed -n '4,16p' "$0"; exit 0 ;;
	*) echo "unknown option $a"; exit 2 ;;
	esac
done

NAME="your system"
if [ -r /etc/os-release ]; then
	# shellcheck disable=SC1091
	NAME="$(. /etc/os-release && echo "${PRETTY_NAME:-${NAME:-your system}}")"
fi

# The compilers, CMake/Ninja, the Monado build's libraries and Rust (DriverKit).
ARCH_BASE=(base-devel git cmake ninja python pkgconf eigen vulkan-headers vulkan-icd-loader glslang shaderc libusb
	systemd-libs libjpeg-turbo ceres-solver suitesparse google-glog gflags jsoncpp libx11 libxcb libxrandr wayland
	wayland-protocols libdrm zlib patchelf rust)
DEB_BASE=(build-essential git cmake ninja-build python3 pkg-config libeigen3-dev libvulkan-dev glslang-tools glslc
	libusb-1.0-0-dev libudev-dev libjpeg-dev libceres-dev libsuitesparse-dev libgoogle-glog-dev libgflags-dev
	libjsoncpp-dev libx11-dev libx11-xcb-dev libxcb-randr0-dev libxrandr-dev libwayland-dev wayland-protocols libdrm-dev
	zlib1g-dev patchelf cargo)
FED_BASE=(gcc-c++ git cmake ninja-build python3 pkgconf-pkg-config eigen3-devel vulkan-loader-devel vulkan-headers
	glslang glslc libusb1-devel systemd-devel libjpeg-turbo-devel ceres-solver-devel suitesparse-devel glog-devel
	gflags-devel jsoncpp-devel libX11-devel libxcb-devel libXrandr-devel wayland-devel wayland-protocols-devel
	libdrm-devel zlib-devel patchelf cargo)
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
if [ "$LIST" = 1 ] || [ "$DRY" = 1 ]; then
	if [ "$FAMILY" = "Arch based" ] && [ ${#MISSING[@]} -eq 0 ]; then
		echo "  everything is installed"
	else
		echo "  ${CMD[*]}"
	fi
	exit 0
fi
if [ "$FAMILY" = "Arch based" ] && [ ${#MISSING[@]} -eq 0 ]; then
	echo "  everything is installed"
	exit 0
fi
if [ "$(id -u)" != 0 ]; then
	echo "  needs root: asking through sudo"
	exec sudo bash "$0" "$@"
fi
if [ "$FAMILY" = "Debian based" ]; then
	echo "  apt-get update"
	apt-get update
	export DEBIAN_FRONTEND=noninteractive
fi
echo "  ${CMD[*]}"
"${CMD[@]}"
echo "DONE dependencies installed"
