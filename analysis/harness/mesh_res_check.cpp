// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// How far SteamVR's distortion mesh (our ComputeDistortion sampled on an N x N grid, blended in between) is from the
// exact PS VR2 distortion, in render-target pixels, for SteamVR's default N=49, Monado's 64 and Sony's 80.
#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>
extern "C" {
#include "xrt/xrt_defines.h"
void psvr2_compute_distortion_asymmetric(float *calibration, struct xrt_uv_triplet *outCoords, int eEye, float fU, float fV);
}
int main() {
	float calib[8] = {-0.09919293f, 0, 0.09919293f, 0, 1, 0, 1, 0};
	const double W = 2800, H = 2856; // render target per eye at 140 %
	for (int N : {33, 49, 64, 80, 128}) {
		std::vector<xrt_uv_triplet> g(N * N);
		for (int y = 0; y < N; y++) for (int x = 0; x < N; x++)
			psvr2_compute_distortion_asymmetric(calib, &g[y * N + x], 0, (float)x / (N - 1), (float)y / (N - 1));
		std::vector<double> err, rings[3]; double mx = 0;
		const int F = 600;
		for (int y = 0; y <= F; y++) for (int x = 0; x <= F; x++) {
			float u = (float)x / F, v = (float)y / F;
			xrt_uv_triplet e; psvr2_compute_distortion_asymmetric(calib, &e, 0, u, v);
			if (e.g.x < 0 || e.g.x > 1 || e.g.y < 0 || e.g.y > 1) continue; // outside the picture: black anyway
			float gx = u * (N - 1), gy = v * (N - 1); int ix = std::min((int)gx, N - 2), iy = std::min((int)gy, N - 2);
			float fx = gx - ix, fy = gy - iy;
			auto at = [&](int a, int b) { return g[b * N + a].g; };
			double ix_ = (1 - fx) * (1 - fy) * at(ix, iy).x + fx * (1 - fy) * at(ix + 1, iy).x + (1 - fx) * fy * at(ix, iy + 1).x + fx * fy * at(ix + 1, iy + 1).x;
			double iy_ = (1 - fx) * (1 - fy) * at(ix, iy).y + fx * (1 - fy) * at(ix + 1, iy).y + (1 - fx) * fy * at(ix, iy + 1).y + fx * fy * at(ix + 1, iy + 1).y;
			double d = std::hypot((ix_ - e.g.x) * W, (iy_ - e.g.y) * H);
			double r = std::hypot(e.g.x - 0.5, e.g.y - 0.5) * 2; // 0 centre, 1 edge midpoint of the picture
			int ring = r < 0.5 ? 0 : r < 0.8 ? 1 : 2;
			rings[ring].push_back(d);
			err.push_back(d); mx = std::max(mx, d);
		}
		std::sort(err.begin(), err.end());
		printf("mesh %3d x %3d: error median %.3f px, p99 %.3f px, max %.3f px (render target 2800x2856)\n", N, N,
		       err[err.size() / 2], err[(size_t)(0.99 * err.size())], mx);
		const char *nm[3] = {"centre (r<0.5)", "middle (0.5-0.8)", "edge (>0.8)"};
		for (int k = 0; k < 3; k++) {
			auto &v = rings[k]; std::sort(v.begin(), v.end());
			if (!v.empty()) printf("    %-17s median %.3f px, p99 %.3f px\n", nm[k], v[v.size() / 2], v[(size_t)(0.99 * v.size())]);
		}
	}
}
