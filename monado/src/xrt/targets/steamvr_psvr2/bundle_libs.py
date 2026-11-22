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
Bundled libraries with a DT_RUNPATH of their own (e.g. Arch's OpenCV and libjpeg: /usr/lib) don't take the plugin's
DT_RPATH for their dependencies, so inside the container those wouldn't be found next to the plugin (the hand tracking
build, 2026-11-22: 35 of OpenCV's libraries, Qt, GStreamer and FFmpeg among them). Their copies get that entry turned
into a DT_RPATH (the tag only, the path stays): the loader then searches it and, after it, the RPATHs of the libraries
that loaded it, up to the plugin's $ORIGIN. No tool needed (an earlier version used patchelf, which a user's build
then stopped on, for libjpeg).
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


def runpath_to_rpath(path):
    """Turn the library's DT_RUNPATH entries into DT_RPATH (ELF64 little-endian, via PT_DYNAMIC). Returns how many."""
    import struct
    DT_NULL, DT_RPATH, DT_RUNPATH, PT_DYNAMIC = 0, 15, 29, 2
    with open(path, "r+b") as f:
        head = f.read(64)
        if head[:4] != b"\x7fELF" or head[4] != 2 or head[5] != 1:
            return 0  # not ELF64 little-endian: left as it is
        e_phoff, = struct.unpack_from("<Q", head, 0x20)
        e_phentsize, e_phnum = struct.unpack_from("<HH", head, 0x36)
        changed = 0
        for i in range(e_phnum):
            f.seek(e_phoff + i * e_phentsize)
            ph = f.read(e_phentsize)
            p_type, = struct.unpack_from("<I", ph, 0)
            if p_type != PT_DYNAMIC:
                continue
            p_offset, = struct.unpack_from("<Q", ph, 0x08)
            p_filesz, = struct.unpack_from("<Q", ph, 0x20)
            for off in range(p_offset, p_offset + p_filesz, 16):
                f.seek(off)
                tag, = struct.unpack("<q", f.read(8))
                if tag == DT_NULL:
                    break
                if tag == DT_RUNPATH:
                    f.seek(off)
                    f.write(struct.pack("<q", DT_RPATH))
                    changed += 1
        return changed


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
    bundled, skipped, repathed = [], [], []
    for name, src in sorted(deps.items()):
        if any(name.startswith(n) for n in NEVER):
            skipped.append(name)
            continue
        if name in have:
            continue
        bundled.append(name)
    for name in bundled:
        src = deps[name]
        if not dry:
            dst = os.path.join(out_dir, name)
            if os.path.abspath(src) != os.path.abspath(dst):
                shutil.copy2(os.path.realpath(src), dst)
            if runpath_to_rpath(dst):
                repathed.append(name)
    print(f"{len(deps)} dependencies: {len(bundled)} bundled into {out_dir}, {len(deps) - len(bundled) - len(skipped)} "
          f"from Steam's runtime, {len(skipped)} from the system (C runtime, graphics, udev/dbus)")
    if bundled:
        print("bundled: " + " ".join(bundled))
    if repathed:
        print(f"{len(repathed)} had a RUNPATH, now an RPATH (they find the bundled libraries): " + " ".join(repathed))
    return 0


if __name__ == "__main__":
    sys.exit(main())
