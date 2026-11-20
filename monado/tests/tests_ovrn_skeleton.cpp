// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The SteamVR plugin's Sense hand skeleton (ovrn_hand_skeleton.hpp): every pose a game can get has 31 valid
 *         bones, for both hands and both motion ranges; the hand fitted to the Sense touches its controls and never
 *         sinks into the handle; the sensors move the fingers the right way. Run with SteamVR's reference hand (when
 *         SteamVR is installed) and with the built-in hand.
 */

#include "ovrn_hand_skeleton.hpp"

#include "catch_amalgamated.hpp"

#include <sys/stat.h>

#include <cmath>
#include <cstdlib>
#include <string>

using ovrn::skel::HandInput;
using ovrn::skel::HandSkeleton;
using ovrn::skel::kBoneCount;
using ovrn::skel::Thumb;

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

//! SteamVR's folder if it's installed in the usual place (the tests then also run with Valve's reference hand).
std::string
installed_steamvr()
{
	const char *home = getenv("HOME");
	if (home == nullptr) {
		return {};
	}
	for (const char *rel : {"/.local/share/Steam/steamapps/common/SteamVR", "/.steam/steam/steamapps/common/SteamVR"}) {
		const std::string d = std::string(home) + rel;
		struct stat st;
		if (stat((d + "/resources/skeletons/vr_glove_left_skeleton.glb").c_str(), &st) == 0) {
			return d;
		}
	}
	return {};
}

//! The hands to test: "builtin" always, "steamvr" when SteamVR is installed.
std::vector<std::string>
sources()
{
	std::vector<std::string> s = {"builtin"};
	if (!installed_steamvr().empty()) {
		s.push_back("steamvr");
	}
	return s;
}

void
use(const std::string &source)
{
	setenv("OVRN_STEAMVR_DIR", source == "steamvr" ? installed_steamvr().c_str() : "/nonexistent", 1);
}

//! Every bone's position in the grip pose (forward kinematics of what SteamVR gets).
void
world(const vr::VRBoneTransform_t b[kBoneCount], ovrn::skel::detail::V out[kBoneCount])
{
	using namespace ovrn::skel::detail;
	V pos[kBoneCount];
	Q rot[kBoneCount], wr[kBoneCount];
	for (uint32_t i = 0; i < kBoneCount; i++) {
		pos[i] = {b[i].position.v[0], b[i].position.v[1], b[i].position.v[2]};
		rot[i] = {b[i].orientation.w, b[i].orientation.x, b[i].orientation.y, b[i].orientation.z};
	}
	forward(pos, rot, out, wr);
}

//! The pose after the smoothing has settled on @p in.
void
settled(HandSkeleton &s, const HandInput &in, bool with, vr::VRBoneTransform_t out[kBoneCount])
{
	for (int i = 0; i < 40; i++) {
		s.update(in);
		struct timespec ts = {0, 5000000};
		nanosleep(&ts, nullptr);
	}
	s.pose(with, out);
}

} // namespace

TEST_CASE("ovrn_skeleton: valid before any input")
{
	for (const std::string &src : sources()) {
		use(src);
		for (bool left : {true, false}) {
			HandSkeleton s(left);
			INFO(src << " " << (left ? "left" : "right") << ": " << s.describe());
			CHECK(s.fromSteamVR() == (src == "steamvr"));
			vr::VRBoneTransform_t bones[kBoneCount];
			for (bool with : {true, false}) {
				s.pose(with, bones);
				CHECK(problem(bones).empty());
			}
		}
	}
}

TEST_CASE("ovrn_skeleton: valid for every input combination")
{
	int poses = 0;
	for (const std::string &src : sources()) {
		use(src);
		for (bool left : {true, false}) {
			HandSkeleton s(left);
			for (int trig = 0; trig <= 4; trig++) {
				for (int near = 0; near <= 2; near++) {
					for (int grip = 0; grip <= 2; grip++) {
						for (Thumb t : {Thumb::None, Thumb::Stick, Thumb::ButtonA, Thumb::ButtonB}) {
							HandInput in;
							in.trigger = trig / 4.0f;
							in.trigger_proximity = near / 2.0f;
							in.trigger_touch = near == 2 || trig > 0;
							in.grip_proximity = grip / 2.0f;
							in.grip_touch = grip >= 1;
							in.grip = grip == 2 ? 1.0f : 0.0f;
							in.thumb = t;
							for (int frame = 0; frame < 3; frame++) {
								s.update(in);
								vr::VRBoneTransform_t bones[kBoneCount];
								for (bool with : {true, false}) {
									s.pose(with, bones);
									const std::string why = problem(bones);
									INFO(src << " " << (left ? "left" : "right") << " trigger " << in.trigger
									         << " near " << near << " grip " << grip << " thumb " << (int)t);
									REQUIRE(why.empty());
									poses++;
								}
							}
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
	use("builtin");
	HandSkeleton s(true);
	HandInput in;
	in.trigger = NAN;
	in.grip = 7.0f;
	in.trigger_proximity = -INFINITY;
	in.grip_proximity = NAN;
	s.update(in);
	vr::VRBoneTransform_t bones[kBoneCount];
	s.pose(true, bones);
	CHECK(problem(bones).empty());
	s.pose(false, bones);
	CHECK(problem(bones).empty());
}

TEST_CASE("ovrn_skeleton: fitted to the Sense - index on the trigger, thumb on the controls, nothing inside the handle")
{
	using namespace ovrn::skel::detail;
	for (const std::string &src : sources()) {
		use(src);
		for (bool left : {true, false}) {
			HandSkeleton s(left);
			INFO(src << " " << (left ? "left" : "right") << ": " << s.describe());
			const Fit &f = s.fit();
			if (src == "steamvr") { // the built-in hand is a plain stand-in: valid and moving, not fitted this closely
				CHECK(f.index_error_m < 0.008f);
				CHECK(f.thumb_error_m[1] < 0.010f); // stick
				CHECK(f.thumb_error_m[2] < 0.015f); // buttons
				CHECK(f.thumb_error_m[3] < 0.015f);
			}
			// holding: index on the trigger, middle finger on the grip, thumb on the stick
			HandInput in;
			in.trigger_touch = true;
			in.grip_touch = true;
			in.thumb = Thumb::Stick;
			vr::VRBoneTransform_t b[kBoneCount];
			settled(s, in, true, b);
			V w[kBoneCount];
			world(b, w);
			// no finger joint inside the handle (with a few millimetres for the finger's own thickness)
			float deepest = 1;
			for (int finger = 1; finger < 5; finger++) {
				for (int j = kFingerFirst[finger] + 2; j <= kTip[finger]; j++) {
					if (w[j].z > -0.06f && w[j].z < 0.05f) {
						deepest = std::fmin(deepest, from_axis(w[j]));
					}
				}
			}
			INFO("nearest finger joint to the handle axis: " << deepest * 1000 << " mm");
			CHECK(deepest > kHandleRadius + kFingerHalf * 0.6f);
			// the middle fingertip lands at the grip sensor's place along the handle
			CHECK(std::fabs(w[kTip[2]].z - kGripSensor[2]) < 0.004f);
			// the aux bones are the fingers' last joints, from the root
			for (int k = 0; k < 5; k++) {
				CHECK(len(w[26 + k] - w[kLastJoint[k]]) < 1e-4f);
				const V aux{b[26 + k].position.v[0], b[26 + k].position.v[1], b[26 + k].position.v[2]};
				CHECK(len(aux - w[kLastJoint[k]]) < 1e-4f);
			}
		}
	}
}

TEST_CASE("ovrn_skeleton: the right hand mirrors the left")
{
	using namespace ovrn::skel::detail;
	for (const std::string &src : sources()) {
		use(src);
		HandSkeleton l(true), r(false);
		for (int state = 0; state < 3; state++) {
			HandInput in;
			in.trigger_touch = state >= 1;
			in.trigger = state == 2 ? 0.7f : 0.0f;
			in.grip_touch = state >= 1;
			in.thumb = state == 1 ? Thumb::ButtonB : Thumb::None;
			vr::VRBoneTransform_t bl[kBoneCount], br[kBoneCount];
			settled(l, in, true, bl);
			settled(r, in, true, br);
			V wl[kBoneCount], wr[kBoneCount];
			world(bl, wl);
			world(br, wr);
			float worst = 0;
			for (uint32_t i = 1; i < kBoneCount; i++) {
				worst = std::fmax(worst, len(V{-wl[i].x, wl[i].y, wl[i].z} - wr[i]));
			}
			INFO(src << " state " << state << ": worst bone mismatch " << worst * 1000 << " mm");
			CHECK(worst < 0.003f);
		}
	}
}

TEST_CASE("ovrn_skeleton: the sensors move the fingers the right way")
{
	using namespace ovrn::skel::detail;
	for (const std::string &src : sources()) {
		use(src);
		HandSkeleton s(false);
		auto bend = [&](const HandInput &in, bool with, int bone) {
			vr::VRBoneTransform_t b[kBoneCount];
			settled(s, in, with, b);
			const float w = std::fmin(1.0f, std::fabs(b[bone].orientation.w));
			return 2.0f * std::acos(w); // angle of the joint's rotation in its parent
		};
		auto tip_to_axis = [&](const HandInput &in, int finger) {
			vr::VRBoneTransform_t b[kBoneCount];
			settled(s, in, true, b);
			V w[kBoneCount];
			world(b, w);
			return from_axis(w[kTip[finger]]);
		};
		INFO(src);
		HandInput open;
		HandInput near = open;
		near.trigger_proximity = 0.6f;
		near.grip_proximity = 0.6f;
		HandInput touch = open;
		touch.trigger_touch = true;
		touch.grip_touch = true;
		HandInput pressed = touch;
		pressed.trigger = 1.0f;
		pressed.grip = 1.0f;
		// with the controller: the fingers come in toward the handle as the hand closes on it
		for (int finger : {2, 3, 4}) {
			const float o = tip_to_axis(open, finger), n = tip_to_axis(near, finger), t = tip_to_axis(touch, finger);
			INFO("finger " << finger << " tip from axis open/near/touch mm " << o * 1000 << " " << n * 1000 << " "
			               << t * 1000);
			CHECK(o > n);
			CHECK(n > t);
		}
		// the trigger pull bends the index's middle joint further than resting on it
		CHECK(bend(pressed, true, kFingerFirst[1] + 2) > bend(touch, true, kFingerFirst[1] + 2));
		// without the controller: the same sensors close a fist
		for (int bone : {kFingerFirst[1] + 2, kFingerFirst[2] + 2, kFingerFirst[4] + 2}) {
			const float o = bend(open, false, bone), t = bend(touch, false, bone), p = bend(pressed, false, bone);
			INFO("bone " << bone << " bend open/touch/pressed deg " << o * 57.3f << " " << t * 57.3f << " "
			             << p * 57.3f);
			CHECK(o < t);
			CHECK(t < p);
		}
		// the thumb goes somewhere different for each control
		auto thumb_tip = [&](Thumb t) {
			HandInput in = touch;
			in.thumb = t;
			vr::VRBoneTransform_t b[kBoneCount];
			settled(s, in, true, b);
			V w[kBoneCount];
			world(b, w);
			return (w[kLastJoint[0]] + w[kTip[0]]) * 0.5f; // the thumb's pad
		};
		const V none = thumb_tip(Thumb::None), stick = thumb_tip(Thumb::Stick), a = thumb_tip(Thumb::ButtonA),
		        b = thumb_tip(Thumb::ButtonB);
		CHECK(len(none - stick) > 0.01f);
		CHECK(len(a - stick) > 0.008f);
		CHECK(len(b - a) > 0.005f);
	}
}

#include "ovrn_backend.hpp"

TEST_CASE("ovrn_controller_root: the reported root is the grip with the grip's offset undone, velocities included")
{
	using ovrn::Pose;
	using ovrn::Quat;
	using ovrn::TrackedPose;
	using ovrn::Vec3;
	// Monado's Sense grip in the SteamVR root (left): 20.6 deg about x, (0.007, -0.0018, 0.1019) m
	const double a = 20.6 * 3.14159265358979 / 180.0;
	Pose g;
	g.orientation = {std::sin(a / 2), 0, 0, std::cos(a / 2)};
	g.position = {0.007, -0.00182941, 0.1019482};
	TrackedPose grip;
	const double b = 0.7;
	grip.pose.orientation = {0, std::sin(b / 2), 0, std::cos(b / 2)};
	grip.pose.position = {0.3, 1.2, -0.4};
	grip.linear_velocity = {0.1, -0.2, 0.05};
	grip.angular_velocity = {0.5, 1.5, -0.8};
	const TrackedPose root = ovrn::root_from_grip(grip, g);
	// root * grip_in_root must give the grip back
	const Vec3 p = ovrn::quat_rotate(root.pose.orientation, g.position);
	CHECK(std::fabs(root.pose.position.x + p.x - grip.pose.position.x) < 1e-9);
	CHECK(std::fabs(root.pose.position.y + p.y - grip.pose.position.y) < 1e-9);
	CHECK(std::fabs(root.pose.position.z + p.z - grip.pose.position.z) < 1e-9);
	const Quat q = ovrn::quat_mul(root.pose.orientation, g.orientation);
	CHECK(std::fabs(std::fabs(q.x * grip.pose.orientation.x + q.y * grip.pose.orientation.y +
	                          q.z * grip.pose.orientation.z + q.w * grip.pose.orientation.w) -
	                1.0) < 1e-9);
	// the root's velocity: move the grip on by dt with its velocities and compare the root's positions
	const double dt = 1e-5;
	TrackedPose later = grip;
	const Vec3 &w = grip.angular_velocity;
	const double wn = std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
	const Quat dq{w.x / wn * std::sin(wn * dt / 2), w.y / wn * std::sin(wn * dt / 2), w.z / wn * std::sin(wn * dt / 2),
	              std::cos(wn * dt / 2)};
	later.pose.orientation = ovrn::quat_mul(dq, grip.pose.orientation); // world-frame angular velocity
	later.pose.position = {grip.pose.position.x + grip.linear_velocity.x * dt,
	                       grip.pose.position.y + grip.linear_velocity.y * dt,
	                       grip.pose.position.z + grip.linear_velocity.z * dt};
	const TrackedPose root2 = ovrn::root_from_grip(later, g);
	CHECK(std::fabs((root2.pose.position.x - root.pose.position.x) / dt - root.linear_velocity.x) < 1e-4);
	CHECK(std::fabs((root2.pose.position.y - root.pose.position.y) / dt - root.linear_velocity.y) < 1e-4);
	CHECK(std::fabs((root2.pose.position.z - root.pose.position.z) / dt - root.linear_velocity.z) < 1e-4);
	CHECK(root.angular_velocity.x == grip.angular_velocity.x);
	// identity offset: unchanged
	const TrackedPose same = ovrn::root_from_grip(grip, Pose());
	CHECK(std::fabs(same.pose.position.x - grip.pose.position.x) < 1e-12);
}

TEST_CASE("ovrn_skeleton: with the grip's offset in the root, the wrist and aux bones move by it, the fingers don't")
{
	using namespace ovrn::skel::detail;
	use("builtin");
	HandSkeleton plain(true), offset(true);
	const float a = 20.6f * 3.14159265f / 180.0f;
	const V gp{0.007f, -0.0018f, 0.1019f};
	const Q gq{std::cos(a / 2), std::sin(a / 2), 0, 0};
	offset.setGripInRoot(gp, gq);
	vr::VRBoneTransform_t b0[kBoneCount], b1[kBoneCount];
	plain.pose(true, b0);
	offset.pose(true, b1);
	CHECK(problem(b1).empty());
	for (uint32_t i : {1u, 26u, 27u, 28u, 29u, 30u}) {
		const V p0{b0[i].position.v[0], b0[i].position.v[1], b0[i].position.v[2]};
		const V want = gp + rotate(gq, p0);
		CHECK(std::fabs(b1[i].position.v[0] - want.x) < 1e-5f);
		CHECK(std::fabs(b1[i].position.v[1] - want.y) < 1e-5f);
		CHECK(std::fabs(b1[i].position.v[2] - want.z) < 1e-5f);
	}
	for (uint32_t i = 2; i < 26; i++) { // the fingers are relative to their parents: unchanged
		CHECK(std::fabs(b1[i].orientation.w - b0[i].orientation.w) < 1e-6f);
		CHECK(std::fabs(b1[i].position.v[0] - b0[i].position.v[0]) < 1e-6f);
	}
}
