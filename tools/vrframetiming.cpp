// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Logs SteamVR's per-frame compositor timing (IVRCompositor::GetFrameTimings) while a game runs, to tell whether
// late/reprojected frames come from the game's GPU time, its CPU frame interval, or the compositor itself.
// Build: g++ -O2 -o vrframetiming vrframetiming.cpp -L$STEAMVR/bin/linux64 -lopenvr_api -Wl,-rpath,$STEAMVR/bin/linux64
// Usage: vrframetiming seconds out.csv   (run while SteamVR and the game are running). Prints a summary every 30 s
// and at the end. Each row also has the headset pose the game rendered that frame with (m_HmdPose), and "mono_s",
// CLOCK_MONOTONIC when the row was read, so psvr2-test/analysis/steamvr_timeline.py can line frames up with the
// plugin's pose log (OVRD_POSE_LOG).
// Optional 3rd argument poses.csv: the headset pose as SteamVR has it (GetDeviceToAbsoluteTrackingPose, standing, no
// prediction) sampled 1000 times a second, in the plugin pose log's format, so the same analysis works with any driver
// (Sony's too): analysis/steamvr_timeline.py poses.csv frames.csv.
#include <openvr/openvr.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>
#include <time.h>
#include <csignal>
#include <atomic>
#include <mutex>
#include <cmath>
#include <algorithm>
// Ctrl+C / SIGTERM (launch-steamvr.sh JITTER_CHECK=1 stops it this way): finish the file and print the summary.
static std::atomic<bool> g_stop{false};
// Bits that mean the frame was re-warped: late on CPU / GPU, or motion smoothing. VRCompositor_ReprojectionAsync (0x04)
// only says async reprojection is on.
static const uint32_t kReprojected = vr::VRCompositor_ReprojectionReason_Cpu | vr::VRCompositor_ReprojectionReason_Gpu |
                                     vr::VRCompositor_ReprojectionMotion;
static void on_signal(int) { g_stop = true; }

int main(int argc, char **argv) {
	std::signal(SIGINT, on_signal);
	std::signal(SIGTERM, on_signal);
	if (argc < 3) { fprintf(stderr, "usage: %s seconds frames.csv [poses.csv]\n", argv[0]); return 1; }
	double secs = atof(argv[1]);
	vr::EVRInitError e = vr::VRInitError_None;
	// SteamVR may still be starting (launched together with it): keep trying for up to 60 s.
	for (int i = 0; i < 120 && !g_stop; i++) {
		vr::VR_Init(&e, vr::VRApplication_Background);
		if (e == vr::VRInitError_None && vr::VRCompositor() != nullptr) break;
		if (e == vr::VRInitError_None) vr::VR_Shutdown();
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	if (e != vr::VRInitError_None) { fprintf(stderr, "VR_Init failed: %s (is SteamVR running?)\n", vr::VR_GetVRInitErrorAsEnglishDescription(e)); return 1; }
	vr::IVRCompositor *comp = vr::VRCompositor();
	if (!comp) { fprintf(stderr, "no compositor\n"); return 1; }
	std::thread pose_thread;
	if (argc > 3) {
		FILE *pf = fopen(argv[3], "w");
		if (pf == nullptr) { fprintf(stderr, "can't write %s\n", argv[3]); return 1; }
		pose_thread = std::thread([pf] {
			fprintf(pf, "# headset pose from SteamVR (standing, 0 s prediction), CLOCK_MONOTONIC ns\n");
			fprintf(pf, "submit_ns,get_pose_us,valid,result,pos_tracked,qx,qy,qz,qw,px,py,pz,vx,vy,vz,wx,wy,wz\n");
			while (!g_stop) {
				vr::TrackedDevicePose_t p;
				timespec a, b;
				clock_gettime(CLOCK_MONOTONIC, &a);
				vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0f, &p, 1);
				clock_gettime(CLOCK_MONOTONIC, &b);
				const float(*m)[4] = p.mDeviceToAbsoluteTracking.m;
				// rotation matrix -> quaternion
				float tr = m[0][0] + m[1][1] + m[2][2], qw, qx, qy, qz;
				if (tr > 0) { float S = std::sqrt(tr + 1.0f) * 2; qw = 0.25f * S; qx = (m[2][1] - m[1][2]) / S; qy = (m[0][2] - m[2][0]) / S; qz = (m[1][0] - m[0][1]) / S; }
				else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) { float S = std::sqrt(1.0f + m[0][0] - m[1][1] - m[2][2]) * 2; qw = (m[2][1] - m[1][2]) / S; qx = 0.25f * S; qy = (m[0][1] + m[1][0]) / S; qz = (m[0][2] + m[2][0]) / S; }
				else if (m[1][1] > m[2][2]) { float S = std::sqrt(1.0f + m[1][1] - m[0][0] - m[2][2]) * 2; qw = (m[0][2] - m[2][0]) / S; qx = (m[0][1] + m[1][0]) / S; qy = 0.25f * S; qz = (m[1][2] + m[2][1]) / S; }
				else { float S = std::sqrt(1.0f + m[2][2] - m[0][0] - m[1][1]) * 2; qw = (m[1][0] - m[0][1]) / S; qx = (m[0][2] + m[2][0]) / S; qy = (m[1][2] + m[2][1]) / S; qz = 0.25f * S; }
				long long ns = (long long)a.tv_sec * 1000000000LL + a.tv_nsec;
				double us = ((b.tv_sec - a.tv_sec) * 1e9 + (b.tv_nsec - a.tv_nsec)) / 1000.0;
				fprintf(pf, "%lld,%.1f,%d,%d,1,%.7f,%.7f,%.7f,%.7f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", ns, us,
				        (int)p.bPoseIsValid, (int)p.eTrackingResult, qx, qy, qz, qw, m[0][3], m[1][3], m[2][3], p.vVelocity.v[0],
				        p.vVelocity.v[1], p.vVelocity.v[2], p.vAngularVelocity.v[0], p.vAngularVelocity.v[1], p.vAngularVelocity.v[2]);
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			fclose(pf);
		});
	}
	float hz = vr::VRSystem()->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float);
	double budget = 1000.0 / hz;
	FILE *f = fopen(argv[2], "w");
	fprintf(f, "# display %.3f Hz, frame budget %.2f ms\n", hz, budget);
	fprintf(f, "frame,t_s,presents,mispresented,dropped,reproj_flags,app_gpu_ms,total_gpu_ms,comp_gpu_ms,comp_cpu_ms,comp_idle_ms,client_interval_ms,submit_ms,vsyncs_ready,vsyncs_to_first_view,mono_s,pose_valid,m00,m01,m02,m03,m10,m11,m12,m13,m20,m21,m22,m23,wait_get_poses_ms,new_poses_ready_ms,new_frame_ready_ms,comp_update_start_ms\n");
	auto mono = [] { timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; };
	// 30 s window counters
	long w_rows = 0, w_reproj = 0, w_drop = 0, w_mis = 0, w_over = 0; std::vector<double> w_app;
	double w_start = mono(), first_mono = w_start;
	uint32_t last = 0; auto t0 = std::chrono::steady_clock::now(); long rows = 0;
	std::vector<double> app, interval; long reproj = 0, dropped = 0, mis = 0, late_gpu = 0, late_cpu = 0;
	while (!g_stop && std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < secs) {
		vr::Compositor_FrameTiming ft[64];
		for (auto &x : ft) x.m_nSize = sizeof(vr::Compositor_FrameTiming);
		uint32_t n = comp->GetFrameTimings(ft, 64);
		for (uint32_t i = 0; i < n; i++) {
			const auto &t = ft[i];
			if (t.m_nFrameIndex <= last) continue;
			last = t.m_nFrameIndex;
			const float(*m)[4] = t.m_HmdPose.mDeviceToAbsoluteTracking.m;
			fprintf(f, "%u,%.6f,%u,%u,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%.6f,%d", t.m_nFrameIndex, t.m_flSystemTimeInSeconds,
			        t.m_nNumFramePresents, t.m_nNumMisPresented, t.m_nNumDroppedFrames, t.m_nReprojectionFlags, t.m_flPreSubmitGpuMs,
			        t.m_flTotalRenderGpuMs, t.m_flCompositorRenderGpuMs, t.m_flCompositorRenderCpuMs, t.m_flCompositorIdleCpuMs,
			        t.m_flClientFrameIntervalMs, t.m_flSubmitFrameMs, t.m_nNumVSyncsReadyForUse, t.m_nNumVSyncsToFirstView, mono(),
			        (int)t.m_HmdPose.bPoseIsValid);
			for (int a = 0; a < 3; a++) for (int b = 0; b < 4; b++) fprintf(f, ",%.6f", m[a][b]);
			// When (ms from this frame's vsync) the app called WaitGetPoses, its poses were ready, it submitted, and the
			// compositor started: where a frame drawn one refresh off differs in timing.
			fprintf(f, ",%.3f,%.3f,%.3f,%.3f\n", t.m_flWaitGetPosesCalledMs, t.m_flNewPosesReadyMs, t.m_flNewFrameReadyMs,
			        t.m_flCompositorUpdateStartMs);
			w_rows++; w_reproj += (t.m_nReprojectionFlags & kReprojected) != 0; w_drop += t.m_nNumDroppedFrames > 0; w_mis += t.m_nNumMisPresented > 0;
			w_over += t.m_flTotalRenderGpuMs > budget; w_app.push_back(t.m_flPreSubmitGpuMs);
			rows++; app.push_back(t.m_flPreSubmitGpuMs); interval.push_back(t.m_flClientFrameIntervalMs);
			reproj += (t.m_nReprojectionFlags & kReprojected) != 0; dropped += t.m_nNumDroppedFrames > 0; mis += t.m_nNumMisPresented > 0;
			late_gpu += t.m_flTotalRenderGpuMs > budget; late_cpu += t.m_flClientFrameIntervalMs > budget * 1.5;
		}
		{
			// Exact clock link: SteamVR's last vsync in CLOCK_MONOTONIC with its frame counter (the frame rows' t_s
			// "aligns with the vsync" of their frame index).
			// The current frame's timing read right after: its t_s is the vsync it started on, so the pair links SteamVR's
			// clock to CLOCK_MONOTONIC (the analysis takes the most common offset, the frame normally being the last vsync's).
			float since = 0; uint64_t counter = 0;
			vr::Compositor_FrameTiming cur = {};
			cur.m_nSize = sizeof(cur);
			double now_mono = mono();
			if (vr::VRSystem()->GetTimeSinceLastVsync(&since, &counter) && comp->GetFrameTiming(&cur, 0)) {
				fprintf(f, "# link %.6f %.6f %llu %u\n", now_mono - since, cur.m_flSystemTimeInSeconds,
				        (unsigned long long)counter, cur.m_nFrameIndex);
			}
		}
		if (mono() - w_start >= 30.0 && w_rows > 0) {
			std::sort(w_app.begin(), w_app.end());
			fprintf(stderr, "[%4.0f-%4.0f s] %5ld frames: reprojected %5.1f%%, dropped %4.1f%%, mispresented %4.1f%%, game GPU median %.2f p99 %.2f ms, over budget %4.1f%%\n",
			        w_start - first_mono, mono() - first_mono, w_rows, 100.0 * w_reproj / w_rows, 100.0 * w_drop / w_rows, 100.0 * w_mis / w_rows,
			        w_app[w_app.size() / 2], w_app[(size_t)(0.99 * (w_app.size() - 1))], 100.0 * w_over / w_rows);
			w_rows = w_reproj = w_drop = w_mis = w_over = 0; w_app.clear(); w_start = mono();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
	fclose(f);
	auto pct = [](std::vector<double> v, double p) { if (v.empty()) return 0.0; std::sort(v.begin(), v.end()); return v[(size_t)(p * (v.size() - 1))]; };
	fprintf(stderr, "%ld frames (budget %.2f ms): reprojected %.1f%%, dropped %.1f%%, mispresented %.1f%%\n", rows, budget,
	        100.0 * reproj / std::max(rows, 1L), 100.0 * dropped / std::max(rows, 1L), 100.0 * mis / std::max(rows, 1L));
	fprintf(stderr, "game GPU time per frame median %.2f p90 %.2f p99 %.2f ms; total GPU over budget in %.1f%% of frames\n",
	        pct(app, 0.5), pct(app, 0.9), pct(app, 0.99), 100.0 * late_gpu / std::max(rows, 1L));
	fprintf(stderr, "game frame interval median %.2f p90 %.2f p99 %.2f ms; longer than 1.5 budgets in %.1f%% of frames\n",
	        pct(interval, 0.5), pct(interval, 0.9), pct(interval, 0.99), 100.0 * late_cpu / std::max(rows, 1L));
	g_stop = true;
	if (pose_thread.joinable()) pose_thread.join();
	vr::VR_Shutdown();
	return 0;
}
