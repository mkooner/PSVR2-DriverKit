// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//
// Black-box head pose logger: what SteamVR gives an app, whichever driver is running (ours or Sony's), so the two can
// be compared with the same numbers. Read-only (a background OpenVR app; SteamVR must already be running).
//
// Once per display refresh it asks SteamVR for the headset pose twice: predicted to photons, the way a game asks
// (frame time + vsync-to-photons from the HMD properties, minus the time since the last vsync), and unpredicted (0 s).
// Output CSV: t_s, predicted_s, then for each: valid, result, qx qy qz qw, px py pz, vx vy vz, wx wy wz.
//
// Build: g++ -O2 -o vrposelog vrposelog.cpp -L$STEAMVR/bin/linux64 -lopenvr_api -Wl,-rpath,$STEAMVR/bin/linux64
// Usage: vrposelog seconds out.csv
#include <openvr/openvr.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
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
dump(FILE *f, const TrackedDevicePose_t &p)
{
	double q[4];
	quat(p.mDeviceToAbsoluteTracking, q);
	const HmdMatrix34_t &m = p.mDeviceToAbsoluteTracking;
	fprintf(f, ",%d,%d,%.7f,%.7f,%.7f,%.7f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f", p.bPoseIsValid,
	        (int)p.eTrackingResult, q[0], q[1], q[2], q[3], m.m[0][3], m.m[1][3], m.m[2][3], p.vVelocity.v[0],
	        p.vVelocity.v[1], p.vVelocity.v[2], p.vAngularVelocity.v[0], p.vAngularVelocity.v[1],
	        p.vAngularVelocity.v[2]);
}

int
main(int argc, char **argv)
{
	if (argc < 3) {
		fprintf(stderr, "usage: %s seconds out.csv\n", argv[0]);
		return 1;
	}
	double seconds = atof(argv[1]);
	EVRInitError e = VRInitError_None;
	IVRSystem *sys = VR_Init(&e, VRApplication_Background);
	if (!sys) {
		fprintf(stderr, "VR_Init failed: %s (is SteamVR running?)\n", VR_GetVRInitErrorAsEnglishDescription(e));
		return 1;
	}
	float hz = sys->GetFloatTrackedDeviceProperty(k_unTrackedDeviceIndex_Hmd, Prop_DisplayFrequency_Float);
	float v2p = sys->GetFloatTrackedDeviceProperty(k_unTrackedDeviceIndex_Hmd, Prop_SecondsFromVsyncToPhotons_Float);
	char model[256] = "", driver[256] = "";
	sys->GetStringTrackedDeviceProperty(k_unTrackedDeviceIndex_Hmd, Prop_ModelNumber_String, model, sizeof(model));
	sys->GetStringTrackedDeviceProperty(k_unTrackedDeviceIndex_Hmd, Prop_TrackingSystemName_String, driver,
	                                    sizeof(driver));
	if (!(hz > 1))
		hz = 90;
	fprintf(stderr, "HMD %s (%s), %.2f Hz, vsync to photons %.4f s; logging %.0f s\n", model, driver, hz, v2p,
	        seconds);
	FILE *f = fopen(argv[2], "w");
	if (!f) {
		perror(argv[2]);
		return 1;
	}
	{
		// t_s = 0 on CLOCK_MONOTONIC (in the one header line), to line the rows up with the driver's own recordings.
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC, &ts);
		fprintf(f, "# HMD %s (%s), %.3f Hz, vsync_to_photons %.5f, t0_mono_ns %lld\n", model, driver, hz, v2p,
		        (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec);
	}
	fprintf(f, "t_s,pred_s");
	for (const char *tag : {"P", "N"})
		fprintf(f, ",%s_valid,%s_result,%s_qx,%s_qy,%s_qz,%s_qw,%s_px,%s_py,%s_pz,%s_vx,%s_vy,%s_vz,%s_wx,%s_wy,%s_wz",
		        tag, tag, tag, tag, tag, tag, tag, tag, tag, tag, tag, tag, tag, tag, tag);
	fprintf(f, "\n");

	const double frame = 1.0 / hz;
	auto t0 = std::chrono::steady_clock::now();
	uint64_t last_frame = 0;
	long rows = 0;
	for (;;) {
		float since = 0;
		uint64_t frame_counter = 0;
		sys->GetTimeSinceLastVsync(&since, &frame_counter);
		double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
		if (el > seconds)
			break;
		if (frame_counter == last_frame) {
			// Wait for the next vsync, polling finely near it.
			double left = frame - since;
			std::this_thread::sleep_for(std::chrono::microseconds(left > 0.002 ? (long)((left - 0.0015) * 1e6) : 200));
			continue;
		}
		last_frame = frame_counter;
		// A game samples once per frame for the frame after this one: photons at the next vsync + vsync-to-photons.
		double pred = frame - since + v2p;
		TrackedDevicePose_t P[1], N[1];
		sys->GetDeviceToAbsoluteTrackingPose(TrackingUniverseStanding, (float)pred, P, 1);
		sys->GetDeviceToAbsoluteTrackingPose(TrackingUniverseStanding, 0.0f, N, 1);
		fprintf(f, "%.6f,%.5f", el, pred);
		dump(f, P[0]);
		dump(f, N[0]);
		fprintf(f, "\n");
		rows++;
	}
	fclose(f);
	fprintf(stderr, "%ld frames written to %s\n", rows, argv[2]);
	VR_Shutdown();
	return 0;
}
