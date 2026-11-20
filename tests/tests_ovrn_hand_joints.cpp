// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Tests for the tracked hand joints -> SteamVR skeleton conversion (ovrn_hand_joints.hpp), with the built-in
 *         hand and, when SteamVR is installed, Valve's reference hands.
 */

#include "ovrn_hand_joints.hpp"

#include "catch_amalgamated.hpp"

#include <sys/stat.h>

#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

using namespace ovrn::skel;
using namespace ovrn::skel::detail;

namespace {

std::string
installed_steamvr()
{
	const char *home = getenv("HOME");
	if (home == nullptr) {
		return {};
	}
	const std::string d = std::string(home) + "/.local/share/Steam/steamapps/common/SteamVR";
	struct stat st;
	return stat((d + "/resources/skeletons/vr_glove_left_skeleton.glb").c_str(), &st) == 0 ? d : std::string();
}

//! The hands to test with: the built-in ones, and Valve's when installed.
std::vector<std::pair<std::string, Rest>>
hands(bool left)
{
	std::vector<std::pair<std::string, Rest>> out;
	out.emplace_back("built-in", builtin_rest(left));
	const std::string d = installed_steamvr();
	if (!d.empty()) {
		Rest r;
		std::string why;
		if (load_glb(d + "/resources/skeletons/vr_glove_" + (left ? "left" : "right") + "_skeleton.glb", left, r,
		             &why)) {
			out.emplace_back("Valve's", r);
		}
	}
	return out;
}

V
bone_pos(const vr::VRBoneTransform_t &b)
{
	return {b.position.v[0], b.position.v[1], b.position.v[2]};
}

Q
bone_rot(const vr::VRBoneTransform_t &b)
{
	return {b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z};
}

//! The angle between two rotations (radians).
float
angle(Q a, Q b)
{
	const float d = std::fabs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z);
	return 2.0f * std::acos(d > 1.0f ? 1.0f : d);
}

//! Every bone in the root from the 31 parent-relative transforms (as SteamVR composes them).
void
compose(const vr::VRBoneTransform_t bones[kBoneCount], V wp[kBoneCount], Q wr[kBoneCount])
{
	V pos[kBoneCount];
	Q rot[kBoneCount];
	for (uint32_t i = 0; i < kBoneCount; i++) {
		pos[i] = bone_pos(bones[i]);
		rot[i] = bone_rot(bones[i]);
	}
	forward(pos, rot, wp, wr);
}

Joints
transformed(const Joints &j, Q r, V t)
{
	Joints o = j;
	for (int i = 0; i < kJointCount; i++) {
		o.pos[i] = t + rotate(r, j.pos[i]);
		o.rot[i] = normalized(r * j.rot[i]);
	}
	return o;
}

} // namespace

TEST_CASE("hand joints: a hand in the reference rest pose gives back the reference skeleton")
{
	for (bool left : {true, false}) {
		for (const auto &h : hands(left)) {
			INFO((left ? "left, " : "right, ") << h.first);
			JointSkeleton js(h.second, left);
			vr::VRBoneTransform_t out[kBoneCount];
			REQUIRE(js.convert(js.openxr_frames_at_rest(), out));
			for (int i = 2; i <= 25; i++) {
				INFO("bone " << i);
				CHECK(len(bone_pos(out[i]) - h.second.pos[i]) < 1e-4f);
				CHECK(angle(bone_rot(out[i]), h.second.rot[i]) < 1e-3f);
			}
			// the root is the identity, the wrist where the reference has it
			CHECK(angle(bone_rot(out[0]), Q{}) < 1e-6f);
			CHECK(len(bone_pos(out[1]) - h.second.pos[1]) < 1e-4f);
			CHECK(angle(bone_rot(out[1]), h.second.rot[1]) < 1e-3f);
		}
	}
}

TEST_CASE("hand joints: the reference hand's bending axis is its +Z, so OpenXR's +Y is the back of the fingers")
{
	for (bool left : {true, false}) {
		for (const auto &h : hands(left)) {
			INFO((left ? "left, " : "right, ") << h.first);
			JointSkeleton js(h.second, left);
			const Joints j = js.openxr_frames_at_rest();
			V pos[kBoneCount], wp[kBoneCount];
			Q rot[kBoneCount], wr[kBoneCount];
			for (uint32_t i = 0; i < kBoneCount; i++) {
				pos[i] = h.second.pos[i];
				rot[i] = h.second.rot[i];
			}
			pos[0] = {};
			rot[0] = Q{};
			forward(pos, rot, wp, wr);
			for (int f = 1; f < 5; f++) {
				const int b = kFingerFirst[f] + 1; // the proximal bone
				INFO("finger " << f);
				// Valve's +Z is across the bone (it is a bending axis)
				const V along = unit(wp[b + 1] - wp[b]);
				CHECK(std::fabs(dot(rotate(wr[b], V{0, 0, 1}), along)) < 0.2f);
				// Bending the finger the way Valve's hand closes (+Z) moves its tip toward OpenXR's -Y (the palm).
				Q rot2[kBoneCount];
				for (uint32_t i = 0; i < kBoneCount; i++) {
					rot2[i] = rot[i];
				}
				rot2[b] = rot[b] * about({0, 0, 1}, deg(30));
				V wp2[kBoneCount];
				Q wr2[kBoneCount];
				forward(pos, rot2, wp2, wr2);
				const V moved = wp2[kTip[f]] - wp[kTip[f]];
				CHECK(dot(moved, rotate(j.rot[b], V{0, 1, 0})) < -0.005f);
			}
		}
	}
}

TEST_CASE("hand joints: every tracked joint lands where it was tracked, for any hand pose and size")
{
	for (bool left : {true, false}) {
		for (const auto &h : hands(left)) {
			INFO((left ? "left, " : "right, ") << h.first);
			JointSkeleton js(h.second, left);
			Joints j = js.openxr_frames_at_rest();
			// a bigger hand, joints moved a little each (a tracker's own bone lengths), turned and moved as a whole
			for (int i = 1; i <= 25; i++) {
				j.pos[i] = j.pos[i] * 1.12f + V{0.002f * std::sin((float)i), 0.0015f * std::cos(2.0f * i), 0.001f};
				j.rot[i] = normalized(j.rot[i] * about({1, 0.3f, 0}, deg(4.0f * std::sin(3.0f * i))));
			}
			const Q turn = about({0.3f, 1.0f, -0.2f}, deg(70));
			const V shift{0.1f, -0.05f, 0.3f};
			const Joints moved = transformed(j, turn, shift);
			vr::VRBoneTransform_t out[kBoneCount];
			REQUIRE(js.convert(moved, out));
			V wp[kBoneCount];
			Q wr[kBoneCount];
			compose(out, wp, wr);
			for (int i = 1; i <= 25; i++) {
				INFO("joint " << i);
				CHECK(len(wp[i] - moved.pos[i]) < 1e-4f);
			}
			// the aux bones are each finger's distal joint
			for (int f = 0; f < 5; f++) {
				CHECK(len(bone_pos(out[26 + f]) - moved.pos[kLastJoint[f]]) < 1e-4f);
			}
			// turning the whole hand turns only the wrist: the fingers' parent-relative rotations stay
			vr::VRBoneTransform_t still[kBoneCount];
			REQUIRE(js.convert(j, still));
			CHECK(angle(normalized(turn * bone_rot(still[1])), bone_rot(out[1])) < 1e-3f);
			for (int i = 2; i <= 25; i++) {
				CHECK(angle(bone_rot(still[i]), bone_rot(out[i])) < 1e-3f);
			}
		}
	}
}

TEST_CASE("hand joints: a finger flexed in OpenXR's terms bends the SteamVR finger the same way")
{
	for (bool left : {true, false}) {
		for (const auto &h : hands(left)) {
			INFO((left ? "left, " : "right, ") << h.first);
			JointSkeleton js(h.second, left);
			Joints j = js.openxr_frames_at_rest();
			// flex the index finger 50 degrees at its knuckle (proximal joint 7): OpenXR's flexion turns about the
			// joint's -X (the tip goes toward -Y, the palm), carrying the joints after it
			const int b = 7;
			const Q flex = about(rotate(j.rot[b], V{1, 0, 0}), deg(-50));
			const V pivot = j.pos[b];
			for (int i = b; i <= 10; i++) {
				if (i > b) {
					j.pos[i] = pivot + rotate(flex, j.pos[i] - pivot);
				}
				j.rot[i] = normalized(flex * j.rot[i]);
			}
			vr::VRBoneTransform_t out[kBoneCount];
			REQUIRE(js.convert(j, out));
			// SteamVR's bone 7 is its rest rotation bent 50 degrees about its +Z; the bones after it unchanged
			CHECK(angle(bone_rot(out[b]), normalized(h.second.rot[b] * about({0, 0, 1}, deg(50)))) < deg(0.5f));
			for (int i = b + 1; i <= 10; i++) {
				CHECK(angle(bone_rot(out[i]), h.second.rot[i]) < deg(0.5f));
			}
		}
	}
}

TEST_CASE("hand joints: invalid joints take their rest pose, an invalid wrist gives no skeleton")
{
	for (bool left : {true, false}) {
		const Rest rest = builtin_rest(left);
		JointSkeleton js(rest, left);
		Joints j = js.openxr_frames_at_rest();
		// the middle finger's intermediate joint lost: it and its distal joint and tip take the rest pose there
		for (int i = 1; i <= 25; i++) {
			j.pos[i] = j.pos[i] + V{0, 0.01f, 0};
		}
		j.valid[13] = false;
		j.pos[14] = j.pos[14] + V{0.05f, 0, 0}; // a far-off joint after the lost one must not be used
		vr::VRBoneTransform_t out[kBoneCount];
		REQUIRE(js.convert(j, out));
		for (int i = 13; i <= 15; i++) {
			CHECK(len(bone_pos(out[i]) - rest.pos[i]) < 1e-4f);
			CHECK(angle(bone_rot(out[i]), rest.rot[i]) < 1e-3f);
		}
		for (uint32_t i = 0; i < kBoneCount; i++) {
			for (int k = 0; k < 3; k++) {
				CHECK(std::isfinite(out[i].position.v[k]));
			}
		}
		j.valid[1] = false;
		CHECK_FALSE(js.convert(j, out));
	}
}

#include "ovrn_hand_gestures.hpp"

TEST_CASE("hand gestures: open hand, fist and pinch from the joints")
{
	for (bool left : {true, false}) {
		JointSkeleton js(builtin_rest(left), left);
		const Joints open = js.openxr_frames_at_rest();
		const float radius[kJointCount] = {};
		const Gestures g_open = gestures(open, radius);
		CHECK(g_open.pinch < 0.1f);
		CHECK(g_open.grab < 0.1f);

		// A fist: every finger's three joints flexed 75 degrees the OpenXR way (about each joint's -X).
		Joints fist = open;
		for (int f = 1; f < 5; f++) {
			for (int b = kFingerFirst[f] + 1; b < kTip[f]; b++) {
				const Q flex = about(rotate(fist.rot[b], V{1, 0, 0}), deg(-75));
				for (int i = b + 1; i <= kTip[f]; i++) {
					fist.pos[i] = fist.pos[b] + rotate(flex, fist.pos[i] - fist.pos[b]);
				}
				for (int i = b; i <= kTip[f]; i++) {
					fist.rot[i] = normalized(flex * fist.rot[i]);
				}
			}
		}
		const Gestures g_fist = gestures(fist, radius);
		CHECK(g_fist.grab > 0.9f);
		for (int f = 1; f < 5; f++) {
			CHECK(g_fist.curl[f] > 0.8f);
		}

		// A pinch: the thumb tip on the index tip.
		Joints pinch = open;
		pinch.pos[5] = pinch.pos[10];
		const Gestures g_pinch = gestures(pinch, radius);
		CHECK(g_pinch.pinch > 0.99f);
		CHECK(g_pinch.grab < 0.1f);
		// clicks with hysteresis
		CHECK(click(0.9f, false));
		CHECK_FALSE(click(0.8f, false));
		CHECK(click(0.7f, true));
		CHECK_FALSE(click(0.6f, true));
	}
}
