// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The SteamVR plugin's controller hand skeleton (ovrn_hand_skeleton.hpp): every pose a game can get has 31
 *         valid bones (finite, unit quaternions, hand-sized offsets), for both hands and both motion ranges.
 */

#include "ovrn_hand_skeleton.hpp"

#include "catch_amalgamated.hpp"

#include <cmath>

using ovrn::skel::HandInput;
using ovrn::skel::HandSkeleton;
using ovrn::skel::kBoneCount;

namespace {

//! Why the pose isn't valid (empty: valid).
std::string
problem(const vr::VRBoneTransform_t bones[kBoneCount])
{
	for (uint32_t i = 0; i < kBoneCount; i++) {
		const vr::VRBoneTransform_t &b = bones[i];
		const float q[4] = {b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z};
		float n = 0;
		for (float v : q) {
			if (!std::isfinite(v)) {
				return "bone " + std::to_string(i) + ": non-finite rotation";
			}
			n += v * v;
		}
		if (std::fabs(std::sqrt(n) - 1.0f) > 1e-3f) {
			return "bone " + std::to_string(i) + ": rotation not unit length (" + std::to_string(std::sqrt(n)) + ")";
		}
		for (int k = 0; k < 3; k++) {
			if (!std::isfinite(b.position.v[k]) || std::fabs(b.position.v[k]) > 0.25f) {
				return "bone " + std::to_string(i) + ": position out of a hand's range";
			}
		}
		if (b.position.v[3] != 1.0f) {
			return "bone " + std::to_string(i) + ": position w is not 1";
		}
	}
	return {};
}

} // namespace

TEST_CASE("ovrn_skeleton: valid before any input")
{
	for (bool left : {true, false}) {
		HandSkeleton s(left);
		vr::VRBoneTransform_t bones[kBoneCount];
		for (bool with : {true, false}) {
			s.pose(with, bones);
			INFO((left ? "left" : "right") << (with ? " with" : " without") << " controller");
			CHECK(problem(bones).empty());
		}
	}
}

TEST_CASE("ovrn_skeleton: valid for every input combination and through the touch animations")
{
	int poses = 0;
	for (bool left : {true, false}) {
		HandSkeleton s(left);
		for (int trig = 0; trig <= 10; trig++) {
			for (int grip : {0, 1}) {
				for (int touches = 0; touches < 4; touches++) {
					HandInput in;
					in.trigger = trig / 10.0f;
					in.grip = (float)grip;
					in.trigger_touch = (touches & 1) != 0 || trig > 0;
					in.thumb_touch = (touches & 2) != 0;
					for (int frame = 0; frame < HandSkeleton::kAnimationFrames + 2; frame++) {
						s.update(in);
						vr::VRBoneTransform_t bones[kBoneCount];
						for (bool with : {true, false}) {
							s.pose(with, bones);
							const std::string why = problem(bones);
							INFO((left ? "left" : "right") << " trigger " << in.trigger << " grip " << grip
							                              << " touches " << touches << " frame " << frame);
							REQUIRE(why.empty());
							poses++;
						}
					}
				}
			}
		}
	}
	CHECK(poses > 1000);
}

TEST_CASE("ovrn_skeleton: bad values from the backend still give a valid hand")
{
	HandSkeleton s(true);
	HandInput in;
	in.trigger = NAN;
	in.grip = 7.0f;
	s.update(in);
	vr::VRBoneTransform_t bones[kBoneCount];
	s.pose(true, bones);
	CHECK(problem(bones).empty());
}

TEST_CASE("ovrn_skeleton: the trigger curls the index finger, the grip closes the others")
{
	HandSkeleton s(false);
	vr::VRBoneTransform_t open[kBoneCount], pulled[kBoneCount], gripped[kBoneCount];
	s.update(HandInput{});
	s.pose(true, open);
	HandInput in;
	in.trigger = 1.0f;
	in.trigger_touch = true;
	s.update(in);
	s.pose(true, pulled);
	in = HandInput{};
	in.grip = 1.0f;
	s.update(in);
	s.pose(true, gripped);
	auto differs = [](const vr::VRBoneTransform_t &a, const vr::VRBoneTransform_t &b) {
		return std::fabs(a.orientation.w - b.orientation.w) + std::fabs(a.orientation.x - b.orientation.x) +
		           std::fabs(a.orientation.y - b.orientation.y) + std::fabs(a.orientation.z - b.orientation.z) >
		       0.05f;
	};
	CHECK(differs(open[7], pulled[7]));   // index proximal
	CHECK(differs(open[12], gripped[12])); // middle proximal
	CHECK(differs(open[22], gripped[22])); // pinky proximal
}
