// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Recorded headset poses played into SteamVR instead of the headset's (OVRD_POSE_REPLAY=pose-log.csv), so every
 *         pose-delivery experiment gets exactly the same input: diagnosis only.
 *
 * Reads an OVRD_POSE_LOG file (the head's motion: the "now" pose nqx.. when present, else the submitted one) and plays
 * it in a loop from when the replay starts. Velocities come from the recorded motion itself (centred differences over
 * 10 ms), the same for every mode, so modes differ only in how the pose is handed over.
 *
 * @ingroup st_ovrd
 */

#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

class OvrdPoseReplay
{
public:
	struct Sample
	{
		float q[4];  //!< x y z w
		float p[3];  //!< driver space, m
		float v[3];  //!< world, m/s
		float wl[3]; //!< head-local angular velocity, rad/s (as the plugin sends it)
	};

	bool
	load(const char *path)
	{
		FILE *f = fopen(path, "r");
		if (f == nullptr) {
			return false;
		}
		char line[4096];
		std::vector<std::string> cols;
		while (fgets(line, sizeof line, f) != nullptr) {
			if (line[0] == '#') {
				continue;
			}
			if (cols.empty()) {
				char *tok = strtok(line, ",\n");
				while (tok != nullptr) {
					cols.push_back(tok);
					tok = strtok(nullptr, ",\n");
				}
				continue;
			}
			std::vector<double> v;
			char *tok = strtok(line, ",\n");
			while (tok != nullptr) {
				v.push_back(atof(tok));
				tok = strtok(nullptr, ",\n");
			}
			if (v.size() != cols.size()) {
				continue; // a cut-off line
			}
			auto col = [&](const char *name) {
				for (size_t i = 0; i < cols.size(); i++) {
					if (cols[i] == name) {
						return (int)i;
					}
				}
				return -1;
			};
			bool now_pose = col("nqx") >= 0;
			int iq = col(now_pose ? "nqx" : "qx"), ip = col(now_pose ? "npx" : "px"), it = col("submit_ns"),
			    iv = col("valid");
			if (iq < 0 || ip < 0 || it < 0 || (iv >= 0 && v[iv] < 0.5)) {
				continue;
			}
			m_t.push_back((int64_t)v[it]);
			Sample s = {};
			for (int k = 0; k < 4; k++) {
				s.q[k] = (float)v[iq + k];
			}
			for (int k = 0; k < 3; k++) {
				s.p[k] = (float)v[ip + k];
			}
			if (!m_s.empty() && dot4(s.q, m_s.back().q) < 0) {
				for (float &x : s.q) {
					x = -x;
				}
			}
			m_s.push_back(s);
		}
		fclose(f);
		if (m_s.size() < 100) {
			return false;
		}
		derive_velocities();
		return true;
	}

	size_t
	size() const
	{
		return m_s.size();
	}

	double
	duration_s() const
	{
		return m_s.empty() ? 0 : (m_t.back() - m_t.front()) * 1e-9;
	}

	//! The recorded pose @p rel_ns after the recording's start, looped.
	Sample
	at(int64_t rel_ns) const
	{
		int64_t span = m_t.back() - m_t.front();
		rel_ns %= span;
		if (rel_ns < 0) {
			rel_ns += span;
		}
		int64_t t = m_t.front() + rel_ns;
		size_t lo = 0, hi = m_t.size() - 1;
		while (hi - lo > 1) {
			size_t mid = (lo + hi) / 2;
			(m_t[mid] <= t ? lo : hi) = mid;
		}
		float a = m_t[hi] > m_t[lo] ? (float)(t - m_t[lo]) / (float)(m_t[hi] - m_t[lo]) : 0.0f;
		const Sample &A = m_s[lo], &B = m_s[hi];
		Sample o;
		float n = 0;
		for (int k = 0; k < 4; k++) {
			o.q[k] = A.q[k] + (B.q[k] - A.q[k]) * a;
			n += o.q[k] * o.q[k];
		}
		n = std::sqrt(n);
		for (float &x : o.q) {
			x /= n;
		}
		for (int k = 0; k < 3; k++) {
			o.p[k] = A.p[k] + (B.p[k] - A.p[k]) * a;
			o.v[k] = A.v[k] + (B.v[k] - A.v[k]) * a;
			o.wl[k] = A.wl[k] + (B.wl[k] - A.wl[k]) * a;
		}
		return o;
	}

private:
	static float
	dot4(const float *a, const float *b)
	{
		return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
	}

	//! Head-local rate from q0 to q1 over dt: 2 * vec(q0^-1 q1) / dt (small angles).
	static void
	local_rate(const float *q0, const float *q1, float dt, float *out)
	{
		// conj(q0) * q1, quaternions x y z w
		float x0 = -q0[0], y0 = -q0[1], z0 = -q0[2], w0 = q0[3];
		float x1 = q1[0], y1 = q1[1], z1 = q1[2], w1 = q1[3];
		float x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1;
		float y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1;
		float z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1;
		float w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1;
		float sgn = w < 0 ? -2.0f : 2.0f;
		out[0] = sgn * x / dt;
		out[1] = sgn * y / dt;
		out[2] = sgn * z / dt;
	}

	void
	derive_velocities()
	{
		const int64_t half = 5000000; // +-5 ms
		size_t lo = 0, hi = 0;
		for (size_t i = 0; i < m_s.size(); i++) {
			while (lo < i && m_t[i] - m_t[lo] > half) {
				lo++;
			}
			while (hi + 1 < m_s.size() && m_t[hi + 1] - m_t[i] <= half) {
				hi++;
			}
			float dt = (float)((m_t[hi] - m_t[lo]) * 1e-9);
			if (dt <= 0) {
				continue;
			}
			for (int k = 0; k < 3; k++) {
				m_s[i].v[k] = (m_s[hi].p[k] - m_s[lo].p[k]) / dt;
			}
			local_rate(m_s[lo].q, m_s[hi].q, dt, m_s[i].wl);
		}
	}

	std::vector<int64_t> m_t;
	std::vector<Sample> m_s;
};
