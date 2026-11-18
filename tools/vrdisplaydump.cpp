// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Everything SteamVR knows about the headset's display, as an app sees it, for comparing drivers (ours vs Sony's):
// every HMD property that has a value, per-eye projection, eye-to-head transform, render size, hidden-area mesh sizes
// and the lens distortion SteamVR uses (IVRSystem::ComputeDistortion on a 33x33 grid per eye).
// analysis/display_diff.py compares two dumps.
// Build: g++ -O2 -o vrdisplaydump vrdisplaydump.cpp -L$STEAMVR/bin/linux64 -lopenvr_api -Wl,-rpath,$STEAMVR/bin/linux64
// Usage: vrdisplaydump out.txt   (SteamVR running with the driver to dump)
// With Sony's driver: vrdisplaydump ~/.config/monado/psvr2-hidden-area.txt and our plugin uses Sony's hidden-area mask.
#include <openvr/openvr.h>
#include <cstdio>
#include <cstring>
#include <thread>
#include <chrono>
using namespace vr;
int main(int argc, char **argv) {
	if (argc < 2) { fprintf(stderr, "usage: %s out.txt\n", argv[0]); return 1; }
	EVRInitError e = VRInitError_None;
	IVRSystem *sys = VR_Init(&e, VRApplication_Background);
	if (e != VRInitError_None) { fprintf(stderr, "VR_Init failed: %s\n", VR_GetVRInitErrorAsEnglishDescription(e)); return 1; }
	FILE *f = fopen(argv[1], "w");
	const TrackedDeviceIndex_t H = k_unTrackedDeviceIndex_Hmd;
	char s[4096];
	// Properties: try every id in the ranges openvr.h uses, with each getter; keep the ones that answer.
	int ranges[][2] = {{1000, 1200}, {2000, 2300}, {3000, 3100}, {4000, 4100}, {5000, 5200}, {6000, 6100}, {7000, 7100}};
	for (auto &r : ranges) {
		for (int id = r[0]; id < r[1]; id++) {
			ETrackedDeviceProperty p = (ETrackedDeviceProperty)id;
			ETrackedPropertyError pe;
			const char *name = sys->GetPropErrorNameFromEnum(TrackedProp_Success); (void)name;
			uint32_t n = sys->GetStringTrackedDeviceProperty(H, p, s, sizeof s, &pe);
			if (pe == TrackedProp_Success && n > 0) { fprintf(f, "prop %d string %s\n", id, s); continue; }
			float fl = sys->GetFloatTrackedDeviceProperty(H, p, &pe);
			if (pe == TrackedProp_Success) { fprintf(f, "prop %d float %.6g\n", id, fl); continue; }
			int32_t in = sys->GetInt32TrackedDeviceProperty(H, p, &pe);
			if (pe == TrackedProp_Success) { fprintf(f, "prop %d int32 %d\n", id, in); continue; }
			uint64_t u = sys->GetUint64TrackedDeviceProperty(H, p, &pe);
			if (pe == TrackedProp_Success) { fprintf(f, "prop %d uint64 %llu\n", id, (unsigned long long)u); continue; }
			bool b = sys->GetBoolTrackedDeviceProperty(H, p, &pe);
			if (pe == TrackedProp_Success) { fprintf(f, "prop %d bool %d\n", id, (int)b); continue; }
			HmdMatrix34_t m = sys->GetMatrix34TrackedDeviceProperty(H, p, &pe);
			if (pe == TrackedProp_Success) {
				fprintf(f, "prop %d matrix34", id);
				for (int a = 0; a < 3; a++) for (int c = 0; c < 4; c++) fprintf(f, " %.6f", m.m[a][c]);
				fprintf(f, "\n");
			}
		}
	}
	uint32_t w, h; sys->GetRecommendedRenderTargetSize(&w, &h);
	fprintf(f, "render_target %u %u\n", w, h);
	for (int eye = 0; eye < 2; eye++) {
		float l, r, t, b; sys->GetProjectionRaw((EVREye)eye, &l, &r, &t, &b);
		fprintf(f, "projection_raw %d %.6f %.6f %.6f %.6f\n", eye, l, r, t, b);
		HmdMatrix34_t m = sys->GetEyeToHeadTransform((EVREye)eye);
		fprintf(f, "eye_to_head %d", eye);
		for (int a = 0; a < 3; a++) for (int c = 0; c < 4; c++) fprintf(f, " %.6f", m.m[a][c]);
		fprintf(f, "\n");
		for (int type = 0; type < 3; type++) {
			HiddenAreaMesh_t hm = sys->GetHiddenAreaMesh((EVREye)eye, (EHiddenAreaMeshType)type);
			fprintf(f, "hidden_area %d type %d triangles %u\n", eye, type, hm.unTriangleCount);
			// The mask itself (render target UV), for the plugin's OVRD_HIDDEN_AREA_FILE: standard/inverse are triangles
			// (3 vertices each), the line loop is one vertex per "triangle".
			uint32_t nv = type == 2 ? hm.unTriangleCount : hm.unTriangleCount * 3;
			for (uint32_t i = 0; i < nv && hm.pVertexData != nullptr; i++) {
				fprintf(f, "ham_vert %d %d %.7f %.7f\n", eye, type, hm.pVertexData[i].v[0], hm.pVertexData[i].v[1]);
			}
		}
		const int N = 33;
		for (int y = 0; y < N; y++) for (int x = 0; x < N; x++) {
			DistortionCoordinates_t d;
			float u = (float)x / (N - 1), v = (float)y / (N - 1);
			bool ok = sys->ComputeDistortion((EVREye)eye, u, v, &d);
			fprintf(f, "distortion %d %.4f %.4f %d %.6f %.6f %.6f %.6f %.6f %.6f\n", eye, u, v, (int)ok, d.rfRed[0], d.rfRed[1],
			        d.rfGreen[0], d.rfGreen[1], d.rfBlue[0], d.rfBlue[1]);
		}
	}
	float since; uint64_t frame;
	sys->GetTimeSinceLastVsync(&since, &frame);
	fprintf(f, "display_frequency_now %.4f\n", sys->GetFloatTrackedDeviceProperty(H, Prop_DisplayFrequency_Float));
	fclose(f);
	VR_Shutdown();
	fprintf(stderr, "wrote %s\n", argv[1]);
	return 0;
}
