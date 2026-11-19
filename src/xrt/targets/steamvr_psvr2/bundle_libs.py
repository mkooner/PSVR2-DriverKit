#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""
Bundle the SteamVR plugin's shared libraries that Steam's runtime doesn't provide, next to the plugin, so SteamVR can
load it inside its Steam Linux Runtime container (sniper). The plugin is linked with DT_RPATH=$ORIGIN, which the
dynamic loader also applies to the bundled libraries' own dependencies (they carry no RUNPATH of their own), so the
whole set resolves from the plugin's folder.

Without this, SteamVR started normally couldn't load the plugin (libgstapp-1.0.so.0 not found, 2026-11-19), the
launcher had to run SteamVR outside the container (VALVE_SKIP_RUNTIME_SAFETY), and Steam games, which always run
inside it, couldn't connect.

Usage: bundle_libs.py <plugin.so> <steam runtime files dir> [--dry-run]
  The runtime dir is e.g. ~/.local/share/Steam/steamapps/common/SteamLinuxRuntime_sniper/sniper_platform_*/files.
Never bundled: the C runtime (glibc, libstdc++, libgcc_s): pressure-vessel uses the newer of the host's and the
runtime's, and a second copy would clash with the one SteamVR itself loads.
"""

import os
import shutil
import subprocess
import sys

NEVER = (
    "linux-vdso.so", "ld-linux", "libc.so", "libm.so", "libmvec.so", "libdl.so", "libpthread.so", "librt.so",
    "libresolv.so", "libutil.so", "libstdc++.so", "libgcc_s.so",
    # graphics and system services: always the host's (pressure-vessel imports the host's graphics stack)
    "libvulkan.so", "libGL", "libEGL.so", "libGLX", "libGLdispatch.so", "libOpenGL.so", "libdrm.so", "libgbm.so",
    "libnvidia", "libudev.so", "libsystemd.so", "libdbus-1.so",
)


def runtime_libs(files_dir):
    names = set()
    lib_dir = os.path.join(files_dir, "lib", "x86_64-linux-gnu")
    if os.path.isdir(lib_dir):
        names.update(os.listdir(lib_dir))
    mtree = os.path.join(os.path.dirname(files_dir), "usr-mtree.txt.gz")
    if os.path.exists(mtree):
        import gzip
        with gzip.open(mtree, "rt", errors="replace") as f:
            for line in f:
                p = line.split(" ", 1)[0]
                if "/x86_64-linux-gnu/" in p and ".so" in p:
                    names.add(os.path.basename(p))
    return names


def dependencies(path):
    out = subprocess.run(["ldd", path], capture_output=True, text=True, check=True).stdout
    deps = {}
    for line in out.splitlines():
        parts = line.strip().split()
        if len(parts) >= 3 and parts[1] == "=>" and parts[2].startswith("/"):
            deps[parts[0]] = parts[2]
        elif parts and parts[0].startswith("/") and "ld-linux" not in parts[0]:
            deps[os.path.basename(parts[0])] = parts[0]
    return deps


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    plugin, files_dir = sys.argv[1], sys.argv[2]
    if files_dir == "auto":
        import glob
        found = sorted(glob.glob(os.path.expanduser(
            "~/.local/share/Steam/steamapps/common/SteamLinuxRuntime_sniper/sniper_platform_*/files")))
        if not found:
            print("no Steam sniper runtime found under ~/.local/share/Steam; set OVRN_STEAM_RUNTIME")
            return 1
        files_dir = found[-1]
    dry = "--dry-run" in sys.argv[3:]
    have = runtime_libs(files_dir)
    if not have:
        print(f"no runtime libraries found under {files_dir}")
        return 1
    out_dir = os.path.dirname(os.path.abspath(plugin))
    deps = dependencies(plugin)
    bundled, skipped = [], []
    for name, src in sorted(deps.items()):
        if any(name.startswith(n) for n in NEVER):
            skipped.append(name)
            continue
        if name in have:
            continue
        bundled.append(name)
        if not dry:
            dst = os.path.join(out_dir, name)
            if os.path.abspath(src) != os.path.abspath(dst):
                shutil.copy2(os.path.realpath(src), dst)
    print(f"{len(deps)} dependencies: {len(bundled)} bundled into {out_dir}, {len(deps) - len(bundled) - len(skipped)} "
          f"from Steam's runtime, {len(skipped)} from the system (C runtime, graphics, udev/dbus)")
    if bundled:
        print("bundled: " + " ".join(bundled))
    return 0


if __name__ == "__main__":
    sys.exit(main())
