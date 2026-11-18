// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Black-box test of SteamVR's own pose prediction (vrserver), separate from any app and from OpenXR: every ~1 ms asks
// for the headset pose at fixed prediction times (GetDeviceToAbsoluteTrackingPose, standing) and logs what comes back.
// analysis/steamvr_probe.py lines it up with the plugin's pose log (OVRD_POSE_LOG): for each answer, the moment of the
// head's real motion it matches, i.e. how far ahead SteamVR really predicted, and whether that jumps or drifts.
// Build: g++ -O2 -o vrposeprobe vrposeprobe.cpp -L$STEAMVR/bin/linux64 -lopenvr_api -Wl,-rpath,$STEAMVR/bin/linux64
// Usage: vrposeprobe seconds out.csv   (SteamVR running; Ctrl+C ends it)
#include <openvr/openvr.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <time.h>
static std::atomic<bool> g_stop{false};
static void on_signal(int) { g_stop = true; }
static long long mono_ns() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (long long)t.tv_sec * 1000000000LL + t.tv_nsec; }
static void quat(const float (*m)[4], float q[4]) {
	float tr = m[0][0] + m[1][1] + m[2][2];
	if (tr > 0) { float S = std::sqrt(tr + 1.0f) * 2; q[3] = 0.25f * S; q[0] = (m[2][1] - m[1][2]) / S; q[1] = (m[0][2] - m[2][0]) / S; q[2] = (m[1][0] - m[0][1]) / S; }
	else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) { float S = std::sqrt(1.0f + m[0][0] - m[1][1] - m[2][2]) * 2; q[3] = (m[2][1] - m[1][2]) / S; q[0] = 0.25f * S; q[1] = (m[0][1] + m[1][0]) / S; q[2] = (m[0][2] + m[2][0]) / S; }
	else if (m[1][1] > m[2][2]) { float S = std::sqrt(1.0f + m[1][1] - m[0][0] - m[2][2]) * 2; q[3] = (m[0][2] - m[2][0]) / S; q[0] = (m[0][1] + m[1][0]) / S; q[1] = 0.25f * S; q[2] = (m[1][2] + m[2][1]) / S; }
	else { float S = std::sqrt(1.0f + m[2][2] - m[0][0] - m[1][1]) * 2; q[3] = (m[1][0] - m[0][1]) / S; q[0] = (m[0][2] + m[2][0]) / S; q[1] = (m[1][2] + m[2][1]) / S; q[2] = 0.25f * S; }
}
int main(int argc, char **argv) {
	if (argc < 3) { fprintf(stderr, "usage: %s seconds out.csv\n", argv[0]); return 1; }
	std::signal(SIGINT, on_signal); std::signal(SIGTERM, on_signal);
	double secs = atof(argv[1]);
	vr::EVRInitError e = vr::VRInitError_None;
	for (int i = 0; i < 120 && !g_stop; i++) {
		vr::VR_Init(&e, vr::VRApplication_Background);
		if (e == vr::VRInitError_None) break;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	if (e != vr::VRInitError_None) { fprintf(stderr, "VR_Init failed: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(e)); return 1; }
	const float preds[] = {-0.020f, -0.010f, 0.0f, 0.010f, 0.020f, 0.030f};
	const int N = sizeof preds / sizeof preds[0];
	FILE *f = fopen(argv[2], "w");
	fprintf(f, "# SteamVR's headset pose for fixed prediction times (s), standing, CLOCK_MONOTONIC ns at the call\nmono_ns,pred_s,valid,qx,qy,qz,qw,px,py,pz\n");
	long long t0 = mono_ns(), rows = 0;
	while (!g_stop && (mono_ns() - t0) * 1e-9 < secs) {
		for (int k = 0; k < N; k++) {
			vr::TrackedDevicePose_t p;
			long long a = mono_ns();
			vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, preds[k], &p, 1);
			long long b = mono_ns();
			float q[4]; quat(p.mDeviceToAbsoluteTracking.m, q);
			const float(*m)[4] = p.mDeviceToAbsoluteTracking.m;
			fprintf(f, "%lld,%.3f,%d,%.7f,%.7f,%.7f,%.7f,%.6f,%.6f,%.6f\n", (a + b) / 2, preds[k], (int)p.bPoseIsValid, q[0], q[1], q[2], q[3], m[0][3], m[1][3], m[2][3]);
			rows++;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	fclose(f);
	fprintf(stderr, "%lld answers in %.0f s\n", rows, (mono_ns() - t0) * 1e-9);
	vr::VR_Shutdown();
	return 0;
}
