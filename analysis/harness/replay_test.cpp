// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Offline test of the SteamVR plugin's pose replay (ovrd_pose_replay.hpp): plays a pose log back and compares it with
// the log itself (pose and derived velocities), and checks the loop.
#include "ovrd_pose_replay.hpp"
#include <cstdio>
#include <cmath>
int main(int argc, char **argv) {
	OvrdPoseReplay r;
	if (!r.load(argv[1])) { printf("load failed\n"); return 1; }
	printf("%zu poses, %.2f s\n", r.size(), r.duration_s());
	// head yaw of the synthetic log: 0.6 sin(2 pi 0.25 t) -> yaw rate 0.6*2pi*0.25 cos(...) (rad/s) about local y (pitch tilts it)
	double max_q_err = 0, max_w_err = 0;
	for (int i = 100; i < 60000; i += 37) {
		double t = i * 1e-3;
		OvrdPoseReplay::Sample s = r.at((int64_t)(t * 1e9));
		double yaw = 0.6 * sin(2 * M_PI * 0.25 * t), yr = 0.6 * 2 * M_PI * 0.25 * cos(2 * M_PI * 0.25 * t);
		double pitch = 0.2 * sin(2 * M_PI * 0.4 * t + 1), pr = 0.2 * 2 * M_PI * 0.4 * cos(2 * M_PI * 0.4 * t + 1);
		// local angular velocity of R = Ry(yaw) Rx(pitch): Rx^T (0, yr, 0) + (pr, 0, 0)
		double wx = pr, wy = yr * cos(pitch), wz = -yr * sin(pitch);
		double e = std::sqrt(pow(s.wl[0] - wx, 2) + pow(s.wl[1] - wy, 2) + pow(s.wl[2] - wz, 2));
		if (std::fabs(t - 20.0) > 0.03) max_w_err = std::max(max_w_err, e); // skip the planted jump at 20 s
		double n = std::sqrt(s.q[0] * s.q[0] + s.q[1] * s.q[1] + s.q[2] * s.q[2] + s.q[3] * s.q[3]);
		max_q_err = std::max(max_q_err, std::fabs(n - 1));
	}
	OvrdPoseReplay::Sample a = r.at(1000000000LL), b = r.at(1000000000LL + (int64_t)(r.duration_s() * 1e9));
	printf("derived angular velocity error max %.4f rad/s (%.2f deg/s); quaternion norm error %.1e; loop: %s\n", max_w_err,
	       max_w_err * 57.2958, max_q_err, std::fabs(a.q[0] - b.q[0]) < 1e-4 ? "seamless" : "MISMATCH");
	return 0;
}
