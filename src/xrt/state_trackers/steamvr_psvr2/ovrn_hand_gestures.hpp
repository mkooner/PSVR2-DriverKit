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

//! Distance from @p p to the segment @p a - @p b.
inline float
segment_distance(const V &p, const V &a, const V &b)
{
	const V ab = b - a;
	const float l2 = dot(ab, ab);
	float t = l2 > 1e-12f ? dot(p - a, ab) / l2 : 0.0f;
	t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
	return len(p - (a + ab * t));
}

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
 * The hand-size reference for the pinch distances: an index finger (proximal + intermediate + distal bone) this long
 * gets the distances as measured; a longer or shorter one has them scaled to this size first (MRTK3 scales pinch by
 * index-finger length, Ultraleap by metacarpal length). Mercury's hand size differs between sessions (the 2026-11-23
 * recordings: 93 mm in one, 72-78 mm in the other for the same hand), which shifted every pinch distance; scaled,
 * real pinches clicked 72% -> 77% of their duration and an open claw-shaped hand 23% -> 21% (labelled moments,
 * psvr2-test/hands gesture analysis).
 */
constexpr float kPinchRefIndexLength = 0.075f;

/*!
 * The pinch gap: from each fingertip to the closest point of the other finger's last bone (thumb distal joint to tip,
 * finger distal joint to tip), the smaller of the two, minus both tips' radii; as Meta's Interaction SDK (fingertip
 * to the thumb's bone segments) and Ultraleap (closest distance between the last phalanges). Tip to tip needed
 * Mercury to put both tips in the same place, which it rarely does: on the 2026-11-24 15:12 session only 35 of 155
 * close approaches got the tips within 10 mm, worst with the palm toward the cameras. The pinch value is 1 at a gap of
 * kPinchFullGap and 0 at kPinchFullGap + kPinchSpan, so the usual click levels (0.85 on, 0.65 off) are a gap of 8 mm
 * on, 15 mm off: on the labelled moments real pinches clicked 99-100% of the time (tip to tip at 10/17 mm: 81-84%),
 * an open claw-shaped hand 35% (28%), a relaxed hand 0% (0%).
 */
constexpr float kPinchFullGap = 0.00275f;
constexpr float kPinchSpan = 0.035f;

/*!
 * @p radius: each joint's radius (metres, the tracker's; 0 if unknown). The values are smooth ramps: pinch from the
 * gap between the last bones of thumb and finger (see kPinchFullGap; after the hand-size scaling, kPinchRefIndexLength);
 * a finger's curl runs from 0.5 to 4 rad of total bend (an open hand bends ~0.3 rad, a fist ~4.5); grab follows the
 * middle, ring and little fingers' mean curl from 0.3 to 0.8.
 */
inline Gestures
gestures(const Joints &j, const float radius[kJointCount])
{
	using namespace detail;
	Gestures g;
	// Index finger length: proximal, intermediate and distal bones (joints 7-10).
	const float index_len = len(j.pos[8] - j.pos[7]) + len(j.pos[9] - j.pos[8]) + len(j.pos[10] - j.pos[9]);
	float scale = index_len > 0.001f ? kPinchRefIndexLength / index_len : 1.0f;
	scale = scale < 0.6f ? 0.6f : (scale > 1.6f ? 1.6f : scale);
	for (int f = 1; f < 5; f++) {
		const int tip = kTip[f];
		const float r = radius != nullptr ? radius[5] + radius[tip] : 0.0f;
		// Joint 4 is the thumb's distal joint, tip - 1 the finger's.
		const float gap = std::fmin(segment_distance(j.pos[tip], j.pos[4], j.pos[5]),
		                            segment_distance(j.pos[5], j.pos[tip - 1], j.pos[tip]));
		const float d = (gap - r) * scale;
		g.finger_pinch[f - 1] = 1.0f - clamp01((d - kPinchFullGap) / kPinchSpan);
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

/*!
 * The grip's click from grab (on above 0.78, off below 0.70). On the 2026-11-23 recordings fists read 0.81-1.0 and
 * relaxed hands 0.54-0.71 (labelled from the camera images): 0.85 sat inside the fists' range (a fist's lowest
 * readings flickered the grip off), 0.78 is between the two with the release above the relaxed hands' p90.
 * Both labelled fists clicked 100% of the time, relaxed and claw-shaped hands 0%. (Meta's palm grab: start 0.9,
 * release 0.85 on its own scale; Ultraleap: 0.8 / 0.6.)
 */
inline bool
click_grab(float value, bool was)
{
	return was ? value > 0.70f : value > 0.78f;
}

} // namespace skel
} // namespace ovrn
