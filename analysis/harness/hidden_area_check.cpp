// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Runs the SteamVR plugin's hidden-area mesh builder (ovrd_hidden_area.hpp) with the PS VR2 driver's distortion
// (psvr2_compute_distortion_asymmetric) and the driver's default calibration, prints how much of each eye's render
// target it hides, and checks no sampled point falls inside a hidden triangle.
#include "ovrd_hidden_area.hpp"
#include <cstdio>
extern "C" {
#include "xrt/xrt_defines.h"
void psvr2_compute_distortion_asymmetric(float *calibration, struct xrt_uv_triplet *outCoords, int eEye, float fU, float fV);
}
static bool in_tri(ovrd_hidden_area::Vec2 p, ovrd_hidden_area::Vec2 a, ovrd_hidden_area::Vec2 b, ovrd_hidden_area::Vec2 c) {
	auto s = [](ovrd_hidden_area::Vec2 p1, ovrd_hidden_area::Vec2 p2, ovrd_hidden_area::Vec2 p3) { return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y); };
	float d1 = s(p, a, b), d2 = s(p, b, c), d3 = s(p, c, a);
	bool neg = d1 < 0 || d2 < 0 || d3 < 0, pos = d1 > 0 || d2 > 0 || d3 > 0;
	return !(neg && pos);
}
int main() {
	float calib[8] = {-0.09919293f, 0, 0.09919293f, 0, 1, 0, 1, 0}; // the driver's default (old calibration format), no rotation
	for (int eye = 0; eye < 2; eye++) {
		auto distort = [&](float u, float v, float out[6]) {
			xrt_uv_triplet d; psvr2_compute_distortion_asymmetric(calib, &d, eye, u, v);
			out[0] = d.r.x, out[1] = d.r.y, out[2] = d.g.x, out[3] = d.g.y, out[4] = d.b.x, out[5] = d.b.y;
		};
		ovrd_hidden_area::Mesh m;
		if (!ovrd_hidden_area::build(distort, m)) { printf("eye %d: no mesh\n", eye); continue; }
		// Check with a much finer grid that no sampled render-target point is hidden.
		int bad = 0, total = 0;
		for (int gy = 0; gy <= 400; gy++) for (int gx = 0; gx <= 400; gx++) {
			float d[6]; distort(gx / 400.f, gy / 400.f, d);
			for (int c = 0; c < 3; c++) {
				ovrd_hidden_area::Vec2 p{d[2 * c], d[2 * c + 1]};
				if (!(p.x >= 0 && p.x <= 1 && p.y >= 0 && p.y <= 1)) continue;
				total++;
				for (size_t t = 0; t + 2 < m.triangles.size(); t += 3)
					if (in_tri(p, m.triangles[t], m.triangles[t + 1], m.triangles[t + 2])) { bad++; break; }
			}
		}
		printf("eye %d: %zu triangles, %.1f%% of the render target hidden; sampled points inside the mask: %d of %d\n", eye,
		       m.triangles.size() / 3, m.hidden_fraction * 100, bad, total);
	}
}
