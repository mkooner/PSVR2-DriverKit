// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Logs SteamVR's per-frame compositor timing (IVRCompositor::GetFrameTimings) while a game runs, to tell whether
// late/reprojected frames come from the game's GPU time, its CPU frame interval, or the compositor itself.
// Build: g++ -O2 -o vrframetiming vrframetiming.cpp -L$STEAMVR/bin/linux64 -lopenvr_api -Wl,-rpath,$STEAMVR/bin/linux64
// Usage: vrframetiming seconds out.csv   (run while SteamVR and the game are running), then summary on stderr.
#include <openvr/openvr.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>
#include <algorithm>
int main(int argc, char **argv) {
	if (argc < 3) { fprintf(stderr, "usage: %s seconds out.csv\n", argv[0]); return 1; }
	double secs = atof(argv[1]);
	vr::EVRInitError e = vr::VRInitError_None;
	vr::VR_Init(&e, vr::VRApplication_Background);
	if (e != vr::VRInitError_None) { fprintf(stderr, "VR_Init failed: %s (is SteamVR running?)\n", vr::VR_GetVRInitErrorAsEnglishDescription(e)); return 1; }
	vr::IVRCompositor *comp = vr::VRCompositor();
	if (!comp) { fprintf(stderr, "no compositor\n"); return 1; }
	float hz = vr::VRSystem()->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float);
	double budget = 1000.0 / hz;
	FILE *f = fopen(argv[2], "w");
	fprintf(f, "# display %.3f Hz, frame budget %.2f ms\n", hz, budget);
	fprintf(f, "frame,t_s,presents,mispresented,dropped,reproj_flags,app_gpu_ms,total_gpu_ms,comp_gpu_ms,comp_cpu_ms,comp_idle_ms,client_interval_ms,submit_ms,vsyncs_ready,vsyncs_to_first_view\n");
	uint32_t last = 0; auto t0 = std::chrono::steady_clock::now(); long rows = 0;
	std::vector<double> app, interval; long reproj = 0, dropped = 0, mis = 0, late_gpu = 0, late_cpu = 0;
	while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < secs) {
		vr::Compositor_FrameTiming ft[64];
		for (auto &x : ft) x.m_nSize = sizeof(vr::Compositor_FrameTiming);
		uint32_t n = comp->GetFrameTimings(ft, 64);
		for (uint32_t i = 0; i < n; i++) {
			const auto &t = ft[i];
			if (t.m_nFrameIndex <= last) continue;
			last = t.m_nFrameIndex;
			fprintf(f, "%u,%.6f,%u,%u,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u\n", t.m_nFrameIndex, t.m_flSystemTimeInSeconds,
			        t.m_nNumFramePresents, t.m_nNumMisPresented, t.m_nNumDroppedFrames, t.m_nReprojectionFlags, t.m_flPreSubmitGpuMs,
			        t.m_flTotalRenderGpuMs, t.m_flCompositorRenderGpuMs, t.m_flCompositorRenderCpuMs, t.m_flCompositorIdleCpuMs,
			        t.m_flClientFrameIntervalMs, t.m_flSubmitFrameMs, t.m_nNumVSyncsReadyForUse, t.m_nNumVSyncsToFirstView);
			rows++; app.push_back(t.m_flPreSubmitGpuMs); interval.push_back(t.m_flClientFrameIntervalMs);
			reproj += t.m_nReprojectionFlags != 0; dropped += t.m_nNumDroppedFrames > 0; mis += t.m_nNumMisPresented > 0;
			late_gpu += t.m_flTotalRenderGpuMs > budget; late_cpu += t.m_flClientFrameIntervalMs > budget * 1.5;
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
	vr::VR_Shutdown();
	return 0;
}
