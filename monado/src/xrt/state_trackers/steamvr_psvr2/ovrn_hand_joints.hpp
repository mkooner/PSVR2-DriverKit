// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Tracked hand joints (OpenXR's 26, XR_EXT_hand_tracking) to SteamVR's 31-bone hand skeleton.
 *
 * One hand representation feeds both APIs: Monado's xrt_hand_joint_set (OpenXR's joints, what XR_EXT_hand_tracking
 * returns as is) becomes SteamVR's skeleton here.
 *
 * The mapping: OpenXR's joints 1-25 (wrist; thumb metacarpal, proximal, distal, tip; then per finger metacarpal,
 * proximal, intermediate, distal, tip) are SteamVR's bones 1-25 in the same order (wrist; thumb_0, thumb_1, thumb_2,
 * thumb_end; per finger meta, 0, 1, 2, end), so no joints are made up. OpenXR's palm (0) has no bone. SteamVR's root
 * (0) is the device pose, and its five aux bones (26-30) are each finger's distal joint in the root.
 *
 * The two use different axes in each bone. OpenXR: -Z from a joint toward the next one (along the bone), +Y out of
 * the back of the hand or finger, X = Y x Z. SteamVR's bones are Valve's reference hand's (read from SteamVR, see
 * ovrn_hand_skeleton.hpp): each bone has its own fixed frame. So for each bone a fixed rotation C is found once from
 * the reference hand at rest, between the OpenXR frame that bone would have there and Valve's frame:
 *   valve_rotation = openxr_rotation * C.
 * The OpenXR frame of a reference bone is built from the reference hand itself: along the bone from its position to its
 * child's; its X axis is the bone's bending axis. Valve's fingers bend about their +Z axis (ovrn_hand_skeleton.hpp's
 * articulation, the convention its Sense fit closes the fingers with), OpenXR's flex toward the palm turning negatively
 * about +X, so OpenXR's X is Valve's -Z; this is checked against the reference hand in the tests (Y then points out of
 * the back of the fingers).
 *
 * Positions are the tracked joints' own, each in its parent bone: the bones get the tracked hand's lengths and every
 * joint lands where it was tracked (fingertips meet in a pinch). A joint the tracker marks invalid takes its rest pose
 * in its parent (as do the joints after it, from there), so a bad joint never stretches or snaps the hand.
 * @ingroup st_ovrn
 */

#pragma once

#include "ovrn_hand_skeleton.hpp"

namespace ovrn {
namespace skel {

constexpr int kJointCount = 26; //!< OpenXR's hand joints (XR_HAND_JOINT_COUNT_EXT)

//! One hand's tracked joints (OpenXR's order and axes), all in the same frame.
struct Joints
{
	detail::V pos[kJointCount];
	detail::Q rot[kJointCount];
	bool valid[kJointCount] = {};
};

/*!
 * The converter for one hand, made once from the reference hand.
 */
class JointSkeleton
{
public:
	JointSkeleton(const detail::Rest &rest, bool left) : m_rest(rest), m_left(left)
	{
		using namespace detail;
		V pos[kBoneCount];
		Q rot[kBoneCount];
		for (uint32_t i = 0; i < kBoneCount; i++) {
			pos[i] = rest.pos[i];
			rot[i] = rest.rot[i];
		}
		pos[0] = {};
		rot[0] = Q{};
		forward(pos, rot, m_restPos, m_restRot);
		Joints rest_joints = openxr_frames_at_rest();
		for (int i = 1; i <= 25; i++) {
			m_offset[i] = normalized(conj(rest_joints.rot[i]) * m_restRot[i]);
		}
	}

	/*!
	 * The OpenXR joints the reference hand at rest has (in the skeleton's root): what the tracker would report for a
	 * hand exactly in Valve's rest pose. For the tests and the offsets.
	 */
	Joints
	openxr_frames_at_rest() const
	{
		using namespace detail;
		Joints j;
		for (int i = 1; i <= 25; i++) {
			j.pos[i] = m_restPos[i];
			j.valid[i] = true;
		}
		// Fingers: along each bone to the next joint; X: Valve's -Z (the bending axis); tips as their parent.
		for (int f = 0; f < 5; f++) {
			for (int b = kFingerFirst[f]; b < kTip[f]; b++) {
				const V along = unit(m_restPos[b + 1] - m_restPos[b]);
				const V bend = rotate(m_restRot[b], V{0, 0, 1}) * -1.0f;
				j.rot[b] = frame(along, bend);
			}
			j.rot[kTip[f]] = j.rot[kTip[f] - 1];
		}
		// Wrist: toward the middle finger's knuckle, its back as the fingers' backs (index..ring proximal bones).
		{
			const V along = unit(m_restPos[12] - m_restPos[1]);
			V back{};
			for (int b : {7, 12, 17}) {
				back = back + rotate(j.rot[b], V{0, 1, 0});
			}
			const V x = cross(back, along * -1.0f); // X = Y x Z with Z = -along
			j.rot[1] = frame(along, x);
		}
		j.pos[0] = m_restPos[1];
		j.rot[0] = j.rot[1];
		return j;
	}

	/*!
	 * All 31 bones from the tracked joints, @p in given in the skeleton's root (the device pose SteamVR gets).
	 * The wrist must be valid (else false and @p out untouched).
	 */
	bool
	convert(const Joints &in, vr::VRBoneTransform_t out[kBoneCount]) const
	{
		using namespace detail;
		if (!in.valid[1]) {
			return false;
		}
		V wp[kBoneCount];
		Q wr[kBoneCount];
		wp[0] = {};
		wr[0] = Q{};
		for (int i = 1; i <= 25; i++) {
			const int p = parent_of(i);
			if (in.valid[i] && (i == 1 || valid_chain(in, i))) {
				wp[i] = in.pos[i];
				wr[i] = normalized(in.rot[i] * m_offset[i]);
			} else {
				// the rest pose in its parent from here on
				wp[i] = wp[p] + rotate(wr[p], m_rest.pos[i]);
				wr[i] = normalized(wr[p] * m_rest.rot[i]);
			}
		}
		for (int f = 0; f < 5; f++) {
			wp[26 + f] = wp[kLastJoint[f]];
			wr[26 + f] = wr[kLastJoint[f]];
		}
		for (uint32_t i = 0; i < kBoneCount; i++) {
			V p;
			Q q;
			if (i == 0) {
				q = Q{};
			} else if (i >= 26) { // the aux bones are in the root
				p = wp[i];
				q = wr[i];
			} else {
				const int par = parent_of((int)i);
				const Q inv = conj(wr[par]);
				p = rotate(inv, wp[i] - wp[par]);
				q = normalized(inv * wr[i]);
			}
			out[i].position = {{p.x, p.y, p.z, 1.0f}};
			out[i].orientation = {q.w, q.x, q.y, q.z};
		}
		return true;
	}

	bool
	left() const
	{
		return m_left;
	}

private:
	static detail::Q
	conj(detail::Q q)
	{
		return {q.w, -q.x, -q.y, -q.z};
	}

	//! OpenXR's frame for a bone along @p along with X near @p x_hint: Z = -along, Y = Z x X... made orthonormal.
	static detail::Q
	frame(detail::V along, detail::V x_hint)
	{
		using namespace detail;
		const V z = unit(along * -1.0f);
		V x = x_hint - z * dot(x_hint, z);
		x = unit(x);
		const V y = cross(z, x);
		return from_columns(x, y, z);
	}

	//! Every joint from the finger's first one up to @p i is valid (a finger is used up to its first bad joint).
	static bool
	valid_chain(const Joints &in, int i)
	{
		for (int b = detail::parent_of(i); b > 1; b = detail::parent_of(b)) {
			if (!in.valid[b]) {
				return false;
			}
		}
		return true;
	}

	detail::Rest m_rest;
	bool m_left;
	detail::V m_restPos[kBoneCount]; //!< the reference hand at rest, in the root
	detail::Q m_restRot[kBoneCount];
	detail::Q m_offset[kJointCount]; //!< valve = openxr * offset, per bone 1-25
};

} // namespace skel
} // namespace ovrn
