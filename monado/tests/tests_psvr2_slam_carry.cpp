// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Tests for carrying the PS VR2 head position across the headset's SLAM resets (psvr2_slam_carry_push).
 */

#include "psvr2_pose_filter.h"
#include "math/m_vec3.h"

#include "catch_amalgamated.hpp"

#include <cmath>
#include <vector>

using Catch::Approx;

namespace {

constexpr int64_t kStep = 16666667; // 60 Hz

// Walk along x at 0.5 m/s from t = 0; the headset re-originates at the given times (position jumps to ~0, plus a
// few mm of motion, as while walking).
struct Run
{
	std::vector<double> reset_times;
	psvr2_slam_carry carry;
	double max_step = 0;
	std::vector<psvr2_slam_carry_event> events;

	xrt_vec3 start = {0, 0, 0}; // where the session's tracking starts (Sony's driver can leave it far from 0)

	explicit Run(std::vector<double> resets, bool enabled = true) : reset_times(resets)
	{
		psvr2_slam_carry_init(&carry, enabled);
	}

	void
	go(double seconds)
	{
		double origin = 0; // where the headset's current SLAM origin is in the true path
		size_t next = 0;
		xrt_vec3 prev{};
		bool have = false;
		for (int64_t t = kStep; t < (int64_t)(seconds * 1e9); t += kStep) {
			double x = 0.5 * t * 1e-9;
			if (next < reset_times.size() && t * 1e-9 >= reset_times[next]) {
				origin = x - 0.004; // re-originated a few mm ago
				next++;
			}
			xrt_vec3 raw = {(float)(x - origin) + start.x, start.y, start.z}; // resets land back at the start
			psvr2_slam_carry_event ev;
			xrt_vec3 out = psvr2_slam_carry_push(&carry, t, &raw, &ev, nullptr);
			if (ev != PSVR2_SLAM_CARRY_NONE) {
				events.push_back(ev);
			}
			if (have) {
				max_step = std::fmax(max_step, m_vec3_len(m_vec3_sub(out, prev)));
			}
			prev = out;
			have = true;
		}
	}
};

} // namespace

TEST_CASE("psvr2_slam_carry: no resets, the position passes through")
{
	Run r({});
	r.go(30);
	CHECK(r.events.empty());
	CHECK(r.max_step == Approx(0.5 / 60).margin(1e-4));
	CHECK(m_vec3_len(r.carry.rebase) == 0.0f);
}

TEST_CASE("psvr2_slam_carry: a mid-session reset is carried over")
{
	Run r({25.0});
	r.go(30);
	REQUIRE(r.events.size() == 1);
	CHECK(r.events[0] == PSVR2_SLAM_CARRY_CARRIED);
	CHECK(r.carry.world_changes == 0);
	// The step at the reset is no bigger than normal walking (8.3 mm per pose); without carrying it is 12.5 m.
	CHECK(r.max_step < 0.0084 + 1e-3);
}

TEST_CASE("psvr2_slam_carry: resets during the first 20 s are carried too, loops are carried but count")
{
	Run r({5.0, 25.0, 25.5});
	r.go(30);
	REQUIRE(r.events.size() == 3);
	CHECK(r.events[0] == PSVR2_SLAM_CARRY_CARRIED);
	CHECK(r.events[1] == PSVR2_SLAM_CARRY_CARRIED);
	CHECK(r.events[2] == PSVR2_SLAM_CARRY_CARRIED_LOOP);
	CHECK(r.carry.world_changes == 1);
	CHECK(r.carry.carried == 3);
	CHECK(r.max_step < 0.0084 + 1e-3); // no step at any reset, the startup one included
}

TEST_CASE("psvr2_slam_carry: a startup burst like the 2026-11-18 17:24 run stays continuous")
{
	// 4 resets in the first 8 s (2.9, 5.8, 7.7, 8.0 s), the last two under 1 s apart (a loop).
	Run r({2.9, 5.8, 7.7, 8.0});
	r.go(12);
	REQUIRE(r.events.size() == 4);
	CHECK(r.carry.carried == 4);
	CHECK(r.carry.world_changes == 1);
	CHECK(r.max_step < 0.0084 + 1e-3);
}

TEST_CASE("psvr2_slam_carry: a session starting away from the origin (after Sony's driver)")
{
	Run r({25.0});
	r.start = {-25.047f, -6.371f, -35.229f};
	r.go(30);
	REQUIRE(r.events.size() == 1);
	CHECK(r.events[0] == PSVR2_SLAM_CARRY_CARRIED);
	CHECK(r.max_step < 0.0084 + 1e-3);
}

TEST_CASE("psvr2_slam_carry: disabled only counts")
{
	Run r({25.0}, false);
	r.go(30);
	REQUIRE(r.events.size() == 1);
	CHECK(r.events[0] == PSVR2_SLAM_CARRY_NOT_CARRIED_DISABLED);
	CHECK(r.carry.world_changes == 1);
	CHECK(r.max_step > 1.0);
}

TEST_CASE("psvr2_slam_carry: a fast real move near the origin isn't a reset")
{
	// Head passes through the origin at 2 m/s: steps are 3.3 cm, under the 5 cm reset step.
	psvr2_slam_carry c;
	psvr2_slam_carry_init(&c, true);
	for (int64_t t = kStep; t < 30LL * 1000000000; t += kStep) {
		double x = 2.0 * (t * 1e-9 - 25.0);
		xrt_vec3 raw = {(float)x, 0, 0};
		psvr2_slam_carry_push(&c, t, &raw, nullptr, nullptr);
	}
	CHECK(c.world_changes + c.carried == 0);
}

TEST_CASE("psvr2_pose_filter: a position correction is blended in, not stepped")
{
	for (int blend_ms : {0, 17}) {
		psvr2_pose_filter_params params;
		psvr2_pose_filter_default_params(&params);
		params.pivot_offset = {0, 0, 0};
		params.pos_blend_ns = (time_duration_ns)blend_ms * 1000000;
		psvr2_pose_filter f;
		psvr2_pose_filter_init(&f, &params);
		xrt_vec3 zero = {0, 0, 0};
		xrt_pose p = {{0, 0, 0, 1}, {0, 0, 0}};
		int64_t t = 0;
		// Still head, gyro at 2 kHz, SLAM at 60 Hz; at 1 s SLAM says 2 cm to the side (under the 5 cm snap).
		double max_step = 0;
		xrt_vec3 prev{};
		bool have = false;
		for (int k = 0; k < 4000; k++) {
			t = (int64_t)k * 500000;
			psvr2_pose_filter_push_gyro(&f, t, &zero);
			if (k % 33 == 0) {
				p.position.x = t >= 1000000000 ? 0.02f : 0.0f;
				psvr2_pose_filter_push_slam(&f, t, &p, nullptr);
			}
			xrt_space_relation r;
			if (psvr2_pose_filter_get(&f, t, &r)) {
				if (have && t > 1000000000 && t < 1100000000)
					max_step = std::fmax(max_step, std::fabs(r.pose.position.x - prev.x));
				prev = r.pose.position;
				have = true;
			}
		}
		INFO("blend " << blend_ms << " ms");
		CHECK(prev.x == Approx(0.02).margin(5e-4)); // converges to the SLAM position either way
		if (blend_ms == 0)
			CHECK(max_step > 0.004); // a step of alpha * 2 cm at the update
		else
			CHECK(max_step < 0.0006); // spread over 17 ms (34 queries)
	}
}