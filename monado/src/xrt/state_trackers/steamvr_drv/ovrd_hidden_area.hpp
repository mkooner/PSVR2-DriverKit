// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Hidden-area mesh from a distortion function: the parts of an eye's render target the distortion never samples.
 *
 * Sample the panel, map every point through the distortion (all three colours: red spreads furthest), and per
 * direction from the image centre find how far out the render target is sampled. Everything beyond that outline (plus
 * a 2% margin) is hidden: triangles between the outline and the target's border. The line loop is the outline. Shared
 * by the SteamVR plugin and offline checks (psvr2-test/analysis/harness/hidden_area_check.cpp).
 *
 * @ingroup st_ovrd
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace ovrd_hidden_area {

struct Vec2
{
	float x, y;
};

struct Mesh
{
	std::vector<Vec2> triangles; //!< hidden triangles, 3 vertices each, render target UV [0,1]
	std::vector<Vec2> loop;      //!< outline of the sampled area
	double hidden_fraction = 0;  //!< share of the render target hidden
};

//! @p distort(u, v, out) maps panel UV to render target UV: out = {r.u, r.v, g.u, g.v, b.u, b.v}.
inline bool
build(const std::function<void(float, float, float[6])> &distort, Mesh &mesh, int grid = 96, int angles = 128)
{
	float c6[6];
	distort(0.5f, 0.5f, c6);
	float cx = c6[2], cy = c6[3];
	if (!(cx > 0.f && cx < 1.f && cy > 0.f && cy < 1.f)) {
		return false;
	}
	std::vector<float> reach(angles, 0.f);
	for (int gy = 0; gy <= grid; gy++) {
		for (int gx = 0; gx <= grid; gx++) {
			float d[6];
			distort((float)gx / grid, (float)gy / grid, d);
			for (int c = 0; c < 3; c++) {
				float u = d[2 * c], v = d[2 * c + 1];
				if (!(u >= 0.f && u <= 1.f && v >= 0.f && v <= 1.f)) {
					continue; // outside the target: that panel pixel is black anyway
				}
				float dx = u - cx, dy = v - cy;
				int bin = (int)std::floor((std::atan2(dy, dx) + (float)M_PI) / (2.f * (float)M_PI) * angles) % angles;
				float rr = std::sqrt(dx * dx + dy * dy);
				for (int k = -1; k <= 1; k++) { // a sample covers its neighbours' directions (coarse grid at the edge)
					int b = (bin + k + angles) % angles;
					reach[b] = std::max(reach[b], rr);
				}
			}
		}
	}
	auto border = [&](float a, float *ox, float *oy) {
		float dx = std::cos(a), dy = std::sin(a), t = 1e9f;
		if (dx > 1e-6f) t = std::min(t, (1.f - cx) / dx);
		if (dx < -1e-6f) t = std::min(t, -cx / dx);
		if (dy > 1e-6f) t = std::min(t, (1.f - cy) / dy);
		if (dy < -1e-6f) t = std::min(t, -cy / dy);
		*ox = cx + dx * t;
		*oy = cy + dy * t;
		return t;
	};
	mesh = Mesh{};
	for (int i = 0; i < angles; i++) {
		float a0 = -(float)M_PI + 2.f * (float)M_PI * i / angles, a1 = a0 + 2.f * (float)M_PI / angles;
		float e0x, e0y, e1x, e1y;
		float t0 = border(a0, &e0x, &e0y), t1 = border(a1, &e1x, &e1y);
		float r0 = std::min(reach[i] * 1.02f, t0), r1 = std::min(reach[(i + 1) % angles] * 1.02f, t1);
		Vec2 p0{cx + std::cos(a0) * r0, cy + std::sin(a0) * r0}, p1{cx + std::cos(a1) * r1, cy + std::sin(a1) * r1};
		Vec2 q0{e0x, e0y}, q1{e1x, e1y};
		mesh.loop.push_back(p0);
		if (r0 >= t0 - 1e-4f && r1 >= t1 - 1e-4f) {
			continue; // sampled out to the border here
		}
		mesh.triangles.insert(mesh.triangles.end(), {p0, q0, q1, p0, q1, p1});
		Vec2 poly[4] = {p0, q0, q1, p1};
		double ar = 0;
		for (int k = 0; k < 4; k++) {
			ar += (double)poly[k].x * poly[(k + 1) % 4].y - (double)poly[(k + 1) % 4].x * poly[k].y;
		}
		mesh.hidden_fraction += std::fabs(ar) / 2;
	}
	return true;
}

} // namespace ovrd_hidden_area
