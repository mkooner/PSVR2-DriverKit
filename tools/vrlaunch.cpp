// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Ask SteamVR to launch one of its apps, the way SteamVR's own menus do (IVRApplications::LaunchApplication), so it
// runs in SteamVR's environment. Started directly from a shell, Room Setup relaunches under another runtime and can
// crash (2026-11-18 19:34: SIGSEGV in its Mono runtime 3 s in).
// Build: g++ -O2 -o vrlaunch vrlaunch.cpp -L$STEAMVR/bin/linux64 -lopenvr_api -Wl,-rpath,$STEAMVR/bin/linux64
// Usage: vrlaunch [--no-home] [app key]   (default openvr.tool.steamvr_room_setup; SteamVR must be running)
#include <openvr/openvr.h>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
using namespace vr;
int main(int argc, char **argv) {
	// --no-home: switch SteamVR Home off first (the steamvr/enableHomeApp setting, the "SteamVR Home" toggle in
	// SteamVR's settings), when it keeps failing and restarting so nothing else can be launched.
	bool no_home = false;
	const char *key = "openvr.tool.steamvr_room_setup";
	for (int i = 1; i < argc; i++) {
		if (std::string(argv[i]) == "--no-home") {
			no_home = true;
		} else {
			key = argv[i];
		}
	}
	EVRInitError e = VRInitError_None;
	// Background: never starts SteamVR itself (a Utility connection did, 2026-11-18 19:35, and that stray server
	// collided with the user's session); fails instead when SteamVR isn't running.
	VR_Init(&e, VRApplication_Background);
	if (e != VRInitError_None) {
		fprintf(stderr, "SteamVR isn't running or refused the connection: %s\n", VR_GetVRInitErrorAsEnglishDescription(e));
		return 1;
	}
	if (no_home) {
		EVRSettingsError se = VRSettingsError_None;
		VRSettings()->SetBool(k_pch_SteamVR_Section, "enableHomeApp", false, &se);
		printf("SteamVR Home %s\n", se == VRSettingsError_None ? "switched off (SteamVR's settings: SteamVR Home)"
		                                                          : "could not be switched off");
		std::this_thread::sleep_for(std::chrono::seconds(2));
	}
	// SteamVR refuses while it is launching another app (LaunchInProgress): retry for up to 15 s. If it never
	// clears, something keeps launching (2026-11-18 20:08: SteamVR Home failing and restarting every 200 ms).
	EVRApplicationError r = VRApplicationError_None;
	for (int attempt = 0; attempt < 30; attempt++) {
		r = VRApplications()->LaunchApplication(key);
		if (r != VRApplicationError_LaunchInProgress) {
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	if (r != VRApplicationError_None) {
		fprintf(stderr, "SteamVR couldn't launch %s: %s\n", key, VRApplications()->GetApplicationsErrorNameFromEnum(r));
		if (r == VRApplicationError_LaunchInProgress) {
			fprintf(stderr, "SteamVR stayed busy launching another app for 15 s. If SteamVR Home keeps restarting in "
			                "vrserver.txt (\"Attempting to start home app\"), switch SteamVR Home off in SteamVR's "
			                "settings and try again.\n");
		}
		VR_Shutdown();
		return 1;
	}
	printf("SteamVR is launching %s\n", key);
	VR_Shutdown();
	return 0;
}
