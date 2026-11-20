// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Gestures from tracked hand joints (OpenXR's 26), kept apart from the skeleton: the skeleton gets the joints
 *         unchanged, these only derive button-like values for bindings.
 * @ingroup st_ovrn
 */

#pragma once

#include "ovrn_hand_joints.hpp"

#include <cmath>

namespace ovrn {
namespace skel {

//! Button-like values from one hand (0..1).
struct Gestures
{
	float pinch = 0.0f;    //!< thumb tip on the index fingertip
	float finger_pinch[4] = {}; //!< thumb tip on the index, middle, ring, little fingertip (pinch = finger_pinch[0])
	float grab = 0.0f;     //!< middle, ring and little fingers closed
	float curl[5] = {};    //!< each finger's bend: 0 straight, 1 fully closed (thumb..little)
	float pinch_distance_m = 1.0f; //!< between the thumb and index fingertips' surfaces
};

namespace detail {

//! How far a finger bends: the turn between successive bones from its metacarpal to its tip (radians).
inline float
finger_bend(const Joints &j, int first, int tip)
{
	float sum = 0.0f;
	V prev = unit(j.pos[first + 1] - j.pos[first]);
	for (int b = first + 1; b < tip; b++) {
		const V d = unit(j.pos[b + 1] - j.pos[b]);
		float c = dot(prev, d);
		c = c > 1.0f ? 1.0f : (c < -1.0f ? -1.0f : c);
		sum += std::acos(c);
		prev = d;
	}
	return sum;
}

} // namespace detail

/*!
 * @p radius: each joint's radius (metres, the tracker's; 0 if unknown). The values are smooth ramps: pinch is 1 with
 * the fingertips' surfaces within 5 mm and 0 from 4 cm apart; a finger's curl runs from 0.5 to 4 rad of total bend
 * (an open hand bends ~0.3 rad, a fist ~4.5); grab follows the middle, ring and little fingers' mean curl from 0.3
 * to 0.8.
 */
inline Gestures
gestures(const Joints &j, const float radius[kJointCount])
{
	using namespace detail;
	Gestures g;
	for (int f = 1; f < 5; f++) {
		const int tip = kTip[f];
		const float r = radius != nullptr ? radius[5] + radius[tip] : 0.0f;
		const float d = len(j.pos[5] - j.pos[tip]) - r;
		g.finger_pinch[f - 1] = 1.0f - clamp01((d - 0.005f) / 0.035f);
		if (f == 1) {
			g.pinch_distance_m = d;
		}
	}
	g.pinch = g.finger_pinch[0];
	for (int f = 0; f < 5; f++) {
		g.curl[f] = clamp01((finger_bend(j, kFingerFirst[f], kTip[f]) - 0.5f) / 3.5f);
	}
	const float closed = (g.curl[2] + g.curl[3] + g.curl[4]) / 3.0f;
	g.grab = clamp01((closed - 0.3f) / 0.5f);
	return g;
}

/*!
 * Whether the palm faces @p eye (the head): its inner side (OpenXR's palm joint -Y) within ~50 degrees of the
 * direction to it. @p palm_pos / @p palm_rot: the palm joint, @p eye in the same frame.
 */
inline bool
palm_faces(detail::V palm_pos, detail::Q palm_rot, detail::V eye)
{
	using namespace detail;
	const V inner = rotate(palm_rot, V{0, -1, 0});
	return dot(inner, unit(eye - palm_pos)) > 0.65f;
}

//! A click from a value, with hysteresis (on above 0.85, off below 0.65).
inline bool
click(float value, bool was)
{
	return was ? value > 0.65f : value > 0.85f;
}

} // namespace skel
} // namespace ovrn
