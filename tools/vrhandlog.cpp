// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//
// Black-box hand pose logger: what SteamVR gives an app for the headset and the tracked bare hands (our driver's
// playstation_vr2_hand devices), once per display refresh, so hand jumps can be measured in the room and set against
// head motion. Read-only (a background OpenVR app; SteamVR must already be running; it never starts SteamVR).
//
// Per refresh: the headset, then the left and right hand device (found by their controller type, re-checked every
// second as hands come and go), each predicted to photons the way a game asks (next vsync + vsync-to-photons).
// Output CSV: t_s, pred_s, then for hmd, left, right: present, valid, result, qx qy qz qw, px py pz, wx wy wz; then for
// left and right: the legacy controller state SteamVR reports (trigger axis, buttons pressed bitmask).
// <out>.events.csv: every SteamVR event (t_s, event type number and name, device index, device's controller type,
// button id): dashboard shown/hidden, button presses and releases, so a gesture's path through SteamVR can be read
// afterwards without anyone watching the desktop.
//
// Build: see tools/build.sh (or g++ -O2 -o vrhandlog vrhandlog.cpp -L$STEAMVR/bin/linux64 -lopenvr_api
//        -Wl,-rpath,$STEAMVR/bin/linux64)
// Usage: vrhandlog seconds out.csv
#include <openvr/openvr.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <thread>

using namespace vr;

static void
quat(const HmdMatrix34_t &m, double q[4])
{
	double tr = m.m[0][0] + m.m[1][1] + m.m[2][2];
	if (tr > 0) {
		double s = sqrt(tr + 1.0) * 2;
		q[3] = 0.25 * s, q[0] = (m.m[2][1] - m.m[1][2]) / s, q[1] = (m.m[0][2] - m.m[2][0]) / s,
		q[2] = (m.m[1][0] - m.m[0][1]) / s;
	} else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
		double s = sqrt(1.0 + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2;
		q[3] = (m.m[2][1] - m.m[1][2]) / s, q[0] = 0.25 * s, q[1] = (m.m[0][1] + m.m[1][0]) / s,
		q[2] = (m.m[0][2] + m.m[2][0]) / s;
	} else if (m.m[1][1] > m.m[2][2]) {
		double s = sqrt(1.0 + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2;
		q[3] = (m.m[0][2] - m.m[2][0]) / s, q[0] = (m.m[0][1] + m.m[1][0]) / s, q[1] = 0.25 * s,
		q[2] = (m.m[1][2] + m.m[2][1]) / s;
	} else {
		double s = sqrt(1.0 + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2;
		q[3] = (m.m[1][0] - m.m[0][1]) / s, q[0] = (m.m[0][2] + m.m[2][0]) / s, q[1] = (m.m[1][2] + m.m[2][1]) / s,
		q[2] = 0.25 * s;
	}
}

static void
dump(FILE *f, bool present, const TrackedDevicePose_t &p)
{
	if (!present) {
		fprintf(f, ",0,0,0,0,0,0,1,0,0,0,0,0,0");
		return;
	}
	double q[4];
	quat(p.mDeviceToAbsoluteTracking, q);
	const HmdMatrix34_t &m = p.mDeviceToAbsoluteTracking;
	fprintf(f, ",1,%d,%d,%.7f,%.7f,%.7f,%.7f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f", p.bPoseIsValid, (int)p.eTrackingResult,
	        q[0], q[1], q[2], q[3], m.m[0][3], m.m[1][3], m.m[2][3], p.vAngularVelocity.v[0], p.vAngularVelocity.v[1],
	        p.vAngularVelocity.v[2]);
}

//! The device index of the tracked bare hand on @p role's side (k_unTrackedDeviceIndexInvalid: none).
static TrackedDeviceIndex_t
find_hand(IVRSystem *sys, ETrackedControllerRole role)
{
	for (TrackedDeviceIndex_t i = 1; i < k_unMaxTrackedDeviceCount; i++) {
		if (sys->GetTrackedDeviceClass(i) != TrackedDeviceClass_Controller) {
			continue;
		}
		char type[128] = "";
		sys->GetStringTrackedDeviceProperty(i, Prop_ControllerType_String, type, sizeof(type));
		if (strcmp(type, "playstation_vr2_hand") != 0) {
			continue;
		}
		if (sys->GetControllerRoleForTrackedDeviceIndex(i) == role ||
		    sys->GetInt32TrackedDeviceProperty(i, Prop_ControllerRoleHint_Int32) == role) {
			return i;
		}
	}
	return k_unTrackedDeviceIndexInvalid;
}

int
main(int argc, char **argv)
{
	if (argc < 3) {
		fprintf(stderr, "usage: %s seconds out.csv\n", argv[0]);
		return 1;
	}
	const double seconds = atof(argv[1]);
	EVRInitError e = VRInitError_None;
	IVRSystem *sys = VR_Init(&e, VRApplication_Background);
	if (!sys) {
		fprintf(stderr, "VR_Init failed: %s (is SteamVR running?)\n", VR_GetVRInitErrorAsEnglishDescription(e));
		return 1;
	}
	float hz = sys->GetFloatTrackedDeviceProperty(k_unTrackedDeviceIndex_Hmd, Prop_DisplayFrequency_Float);
	const float v2p =
	    sys->GetFloatTrackedDeviceProperty(k_unTrackedDeviceIndex_Hmd, Prop_SecondsFromVsyncToPhotons_Float);
	if (!(hz > 1)) {
		hz = 90;
	}
	FILE *f = fopen(argv[2], "w");
	if (!f) {
		perror(argv[2]);
		return 1;
	}
	{
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC, &ts);
		fprintf(f, "# %.3f Hz, vsync_to_photons %.5f, t0_mono_ns %lld\n", hz, v2p,
		        (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec);
	}
	fprintf(f, "t_s,pred_s");
	for (const char *tag : {"hmd", "left", "right"}) {
		fprintf(f, ",%s_present,%s_valid,%s_result,%s_qx,%s_qy,%s_qz,%s_qw,%s_px,%s_py,%s_pz,%s_wx,%s_wy,%s_wz", tag,
		        tag, tag, tag, tag, tag, tag, tag, tag, tag, tag, tag, tag);
	}
	fprintf(f, ",left_trigger,left_buttons,right_trigger,right_buttons\n");
	const std::string events_path = std::string(argv[2]) + ".events.csv";
	FILE *ev = fopen(events_path.c_str(), "w");
	if (ev != nullptr) {
		fprintf(ev, "t_s,event,name,device,controller_type,button\n");
	}
	fprintf(stderr, "logging headset and tracked hands for %.0f s at %.2f Hz to %s\n", seconds, hz, argv[2]);

	const double frame = 1.0 / hz;
	const auto t0 = std::chrono::steady_clock::now();
	uint64_t last_frame = 0;
	long rows = 0;
	double last_scan = -1.0;
	TrackedDeviceIndex_t hand[2] = {k_unTrackedDeviceIndexInvalid, k_unTrackedDeviceIndexInvalid};
	for (;;) {
		float since = 0;
		uint64_t frame_counter = 0;
		sys->GetTimeSinceLastVsync(&since, &frame_counter);
		const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
		if (el > seconds) {
			break;
		}
		if (frame_counter == last_frame) {
			const double left = frame - since;
			std::this_thread::sleep_for(std::chrono::microseconds(left > 0.002 ? (long)((left - 0.0015) * 1e6) : 200));
			continue;
		}
		last_frame = frame_counter;
		if (el - last_scan > 1.0) {
			last_scan = el;
			hand[0] = find_hand(sys, TrackedControllerRole_LeftHand);
			hand[1] = find_hand(sys, TrackedControllerRole_RightHand);
		}
		const double pred = frame - since + v2p;
		TrackedDevicePose_t P[k_unMaxTrackedDeviceCount];
		sys->GetDeviceToAbsoluteTrackingPose(TrackingUniverseStanding, (float)pred, P, k_unMaxTrackedDeviceCount);
		fprintf(f, "%.6f,%.5f", el, pred);
		dump(f, true, P[k_unTrackedDeviceIndex_Hmd]);
		for (int h = 0; h < 2; h++) {
			const bool present = hand[h] != k_unTrackedDeviceIndexInvalid && sys->IsTrackedDeviceConnected(hand[h]);
			dump(f, present, present ? P[hand[h]] : P[0]);
		}
		for (int h = 0; h < 2; h++) {
			VRControllerState_t st = {};
			const bool got = hand[h] != k_unTrackedDeviceIndexInvalid &&
			                 sys->GetControllerState(hand[h], &st, sizeof(st));
			fprintf(f, ",%.3f,%llu", got ? st.rAxis[1].x : -1.0f, got ? (unsigned long long)st.ulButtonPressed : 0ULL);
		}
		fprintf(f, "\n");
		VREvent_t e;
		while (ev != nullptr && sys->PollNextEvent(&e, sizeof(e))) {
			char type[128] = "";
			if (e.trackedDeviceIndex < k_unMaxTrackedDeviceCount) {
				sys->GetStringTrackedDeviceProperty(e.trackedDeviceIndex, Prop_ControllerType_String, type, sizeof(type));
			}
			const bool button = e.eventType >= VREvent_ButtonPress && e.eventType <= VREvent_ButtonUntouch;
			fprintf(ev, "%.6f,%u,%s,%u,%s,%d\n", el, e.eventType,
			        sys->GetEventTypeNameFromEnum((EVREventType)e.eventType), e.trackedDeviceIndex, type,
			        button ? (int)e.data.controller.button : -1);
		}
		rows++;
	}
	fclose(f);
	if (ev != nullptr) {
		fclose(ev);
	}
	fprintf(stderr, "%ld refreshes written to %s\n", rows, argv[2]);
	VR_Shutdown();
	return 0;
}
