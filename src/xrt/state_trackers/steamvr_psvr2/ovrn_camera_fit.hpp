// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  SteamVR's camera lens model (Extended_FTheta) fitted to a camera's own (Monado's camera models).
 * @ingroup st_ovrn
 */

#pragma once

#include "tracking/t_camera_models.h"
#include "tracking/t_tracking.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace ovrn {

/*!
 * Fit Extended_FTheta (r_d = t + k0 t^3 + k1 t^5 + k2 t^7 + k3 t^9, least squares, as PSVR2Toolkit fits it to
 * Sony's model) to the camera's own lens model: sample directions up to the image's usable circle, project them
 * with Monado's model, compare radii. Returns the fit's rms error in pixels.
 */
inline float
fit_ftheta(const struct t_camera_calibration &cam, double out[8])
{
	struct t_camera_model_params mp;
	t_camera_model_params_from_t_camera_calibration(&cam, &mp);
	const double fx = cam.intrinsics[0][0], fy = cam.intrinsics[1][1];
	const double cx = cam.intrinsics[0][2], cy = cam.intrinsics[1][2];
	const double w = cam.image_size_pixels.w, h = cam.image_size_pixels.h;
	const double rmax = 0.5 * std::min(w, h); // the usable image circle
	std::vector<double> th, rd;
	for (int it = 1; it <= 90; it++) {
		const double t = it * (M_PI / 180.0);
		for (int ia = 0; ia < 16; ia++) {
			const double a = ia * (2.0 * M_PI / 16.0);
			float px, py;
			// OpenCV camera axes for Monado's models: z forward.
			if (!t_camera_models_project(&mp, (float)(sin(t) * cos(a)), (float)(sin(t) * sin(a)), (float)cos(t),
			                             &px, &py)) {
				continue;
			}
			if (!std::isfinite(px) || !std::isfinite(py) || hypot(px - cx, py - cy) > rmax) {
				continue;
			}
			th.push_back(t);
			rd.push_back(hypot((px - cx) / fx, (py - cy) / fy));
		}
	}
	for (int i = 0; i < 8; i++) {
		out[i] = 0.0;
	}
	if (th.size() < 8) {
		return INFINITY;
	}
	// Normal equations for (r_d - t) = sum_k c_k t^(2k+3), k = 0..3.
	double M[4][5] = {};
	for (size_t i = 0; i < th.size(); i++) {
		double p[4];
		for (int k = 0; k < 4; k++) {
			p[k] = pow(th[i], 2 * k + 3);
		}
		for (int r = 0; r < 4; r++) {
			for (int c = 0; c < 4; c++) {
				M[r][c] += p[r] * p[c];
			}
			M[r][4] += p[r] * (rd[i] - th[i]);
		}
	}
	for (int c = 0; c < 4; c++) { // Gauss-Jordan with partial pivoting
		int piv = c;
		for (int r = c + 1; r < 4; r++) {
			if (fabs(M[r][c]) > fabs(M[piv][c])) {
				piv = r;
			}
		}
		for (int k = 0; k < 5; k++) {
			std::swap(M[c][k], M[piv][k]);
		}
		if (fabs(M[c][c]) < 1e-15) {
			return INFINITY;
		}
		for (int r = 0; r < 4; r++) {
			if (r == c) {
				continue;
			}
			const double f = M[r][c] / M[c][c];
			for (int k = c; k < 5; k++) {
				M[r][k] -= f * M[c][k];
			}
		}
	}
	for (int k = 0; k < 4; k++) {
		out[k] = M[k][4] / M[k][k];
	}
	double sum = 0;
	for (size_t i = 0; i < th.size(); i++) {
		double r = th[i];
		for (int k = 0; k < 4; k++) {
			r += out[k] * pow(th[i], 2 * k + 3);
		}
		const double e = (r - rd[i]) * 0.5 * (fx + fy);
		sum += e * e;
	}
	return (float)sqrt(sum / th.size());
}

} // namespace ovrn
