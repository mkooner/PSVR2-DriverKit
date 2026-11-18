// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Tests for the PS VR2 head pose filter's smoothly ramped SLAM corrections (corr_smooth_tau_s): the reported
 *         velocity has to match how the output position moves (SteamVR predicts every pose with it), without making
 *         the pose itself jerkier.
 */

#include "psvr2_pose_filter.h"
#include "math/m_vec3.h"

#include "catch_amalgamated.hpp"

#include <cmath>
#include <random>
#include <vector>

namespace {

constexpr int64_t kImuStep = 500000;    // 2 kHz
constexpr int64_t kSlamStep = 16666667; // 60 Hz
constexpr double kAmp = 0.05, kHz = 0.5; // head moving 5 cm back and forth along x

double
true_x(double t)
{
	return kAmp * std::sin(2 * M_PI * kHz * t);
}

double
true_ax(double t)
{
	return -kAmp * std::pow(2 * M_PI * kHz, 2) * std::sin(2 * M_PI * kHz * t);
}

struct Result
{
	double vel_mismatch_rms; //!< reported velocity vs the output position's own motion (m/s)
	double pose_jitter_rms;  //!< second difference of the output position at 90 Hz (m)
};

Result
run(float corr_smooth_tau_s)
{
	psvr2_pose_filter_params params;
	psvr2_pose_filter_default_params(&params);
	params.accel_enable = true;
	params.pos_blend_ns = 100000000; // 100 ms, as the driver default
	params.corr_smooth_tau_s = corr_smooth_tau_s;
	psvr2_pose_filter f;
	psvr2_pose_filter_init(&f, &params);

	std::mt19937 rng(3);
	std::normal_distribution<double> slam_noise(0.0, 0.0005); // 0.5 mm
	const xrt_vec3 zero = {0, 0, 0};
	const double accel_bias = 0.05; // m/s^2 along x: the accelerometer drifts, SLAM keeps correcting it

	int64_t next_slam = 0;
	std::vector<double> ts, px, vx;
	for (int64_t t = 0; t < 12LL * 1000000000; t += kImuStep) {
		double ts_s = t * 1e-9;
		psvr2_pose_filter_push_gyro(&f, t, &zero);
		xrt_vec3 a = {(float)(true_ax(ts_s) + accel_bias), 9.81f, 0.0f}; // identity orientation: specific force
		psvr2_pose_filter_push_accel(&f, t, &a);
		if (t >= next_slam) {
			xrt_pose p = {{0, 0, 0, 1}, {(float)(true_x(ts_s) + slam_noise(rng)), 1.6f, 0.0f}};
			psvr2_pose_filter_push_slam(&f, t, &p, nullptr);
			next_slam += kSlamStep;
		}
		// Sample the output at 90 Hz, after a 4 s settle (gravity estimate, velocity).
		if (ts_s > 4.0 && (t % 11111111) < kImuStep) {
			xrt_space_relation r0, rm, rp;
			psvr2_pose_filter_get(&f, t, &r0);
			psvr2_pose_filter_get(&f, t - 2000000, &rm);
			psvr2_pose_filter_get(&f, t + 2000000, &rp);
			ts.push_back(ts_s);
			px.push_back(r0.pose.position.x);
			vx.push_back(r0.linear_velocity.x);
		}
	}
	// The output position's own motion between consecutive 90 Hz samples vs the velocity reported at their middle.
	double sum_v = 0, sum_j = 0;
	int nv = 0, nj = 0;
	for (size_t i = 1; i + 1 < px.size(); i++) {
		double v_motion = (px[i + 1] - px[i - 1]) / (ts[i + 1] - ts[i - 1]);
		sum_v += std::pow(vx[i] - v_motion, 2);
		nv++;
		double j = px[i + 1] - 2 * px[i] + px[i - 1];
		sum_j += j * j;
		nj++;
	}
	return {std::sqrt(sum_v / nv), std::sqrt(sum_j / nj)};
}

} // namespace

TEST_CASE("psvr2_pose_filter: smoothly ramped corrections are on by default")
{
	psvr2_pose_filter_params params;
	psvr2_pose_filter_default_params(&params);
	CHECK(params.corr_smooth_tau_s == Catch::Approx(0.03f));
}

TEST_CASE("psvr2_pose_filter: the velocity matches the pose's motion with smoothly ramped corrections")
{
	Result off = run(0.0f);
	Result on = run(0.03f);
	INFO("velocity mismatch rms off " << off.vel_mismatch_rms * 1000 << " mm/s, on " << on.vel_mismatch_rms * 1000
	                                  << " mm/s; pose jitter rms off " << off.pose_jitter_rms * 1e6 << " um, on "
	                                  << on.pose_jitter_rms * 1e6 << " um");
	// The velocity follows the output much better ...
	CHECK(on.vel_mismatch_rms < 0.5 * off.vel_mismatch_rms);
	// ... and the pose itself is no jerkier.
	CHECK(on.pose_jitter_rms <= 1.1 * off.pose_jitter_rms);
}

TEST_CASE("psvr2_pose_filter: no velocity while the position is held without SLAM")
{
	// A still head with a correction still being blended in, then SLAM stops (the headset lost its tracking): the
	// filter holds the position, and the reported velocity has to be zero, not the correction's rate.
	psvr2_pose_filter_params params;
	psvr2_pose_filter_default_params(&params);
	params.accel_enable = true;
	params.pos_blend_ns = 100000000;
	psvr2_pose_filter f;
	psvr2_pose_filter_init(&f, &params);
	const xrt_vec3 zero = {0, 0, 0};
	int64_t t = 0;
	for (; t < 3LL * 1000000000; t += kImuStep) {
		psvr2_pose_filter_push_gyro(&f, t, &zero);
		xrt_vec3 a = {0.0f, 9.81f, 0.0f};
		psvr2_pose_filter_push_accel(&f, t, &a);
		if (t % kSlamStep < kImuStep) {
			// The last SLAM poses step 2 cm, so a correction is pending when SLAM stops.
			float x = t > 2900000000LL ? 0.02f : 0.0f;
			xrt_pose p = {{0, 0, 0, 1}, {x, 1.6f, 0.0f}};
			psvr2_pose_filter_push_slam(&f, t, &p, nullptr);
		}
	}
	for (; t < 4LL * 1000000000; t += kImuStep) { // 1 s without SLAM
		psvr2_pose_filter_push_gyro(&f, t, &zero);
		xrt_vec3 a = {0.0f, 9.81f, 0.0f};
		psvr2_pose_filter_push_accel(&f, t, &a);
	}
	xrt_space_relation r;
	REQUIRE(psvr2_pose_filter_get(&f, t, &r));
	CHECK(m_vec3_len(r.linear_velocity) < 0.001f);
}
