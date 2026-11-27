// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Hand geometry for gestures on MediaPipe's 21 hand landmarks (header only, no dependencies, unit tested).
 *
 * Landmark layout (MediaPipe Hands; Mercury's 21 keypoints use the same order): 0 wrist; thumb 1-4 (CMC, MCP, IP,
 * tip); index 5-8, middle 9-12, ring 13-16, little 17-20 (MCP, PIP, DIP, tip). All points of one calculation must be
 * in one coordinate space; the results are ratios and angles, so any uniform scale works (MediaPipe's hand-centred
 * "world" landmarks are used, which are metric but not camera or room space).
 *
 * Invalid input (a non-finite coordinate, zero-length vectors, palm width ~0) gives an invalid result, never NaN/Inf.
 *
 * @ingroup aux_tracking
 */
#pragma once

#include <cmath>
#include <cstdint>

namespace xrt::tracking::hand::mercury::gesture {

struct Vec3
{
	float x = 0, y = 0, z = 0;
};

constexpr int kLandmarkCount = 21;
constexpr float kEpsilon = 1e-6f;

//! Landmark indices (MediaPipe Hands).
enum Landmark : int
{
	WRIST = 0,
	THUMB_CMC = 1,
	THUMB_MCP = 2,
	THUMB_IP = 3,
	THUMB_TIP = 4,
	INDEX_MCP = 5,
	INDEX_PIP = 6,
	INDEX_DIP = 7,
	INDEX_TIP = 8,
	MIDDLE_MCP = 9,
	MIDDLE_TIP = 12,
	RING_MCP = 13,
	RING_TIP = 16,
	LITTLE_MCP = 17,
	LITTLE_TIP = 20,
};

//! A measurement that may be missing.
struct Value
{
	bool valid = false;
	float value = 0;
};

inline bool
finite(const Vec3 &v)
{
	return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline Vec3
sub(const Vec3 &a, const Vec3 &b)
{
	return {a.x - b.x, a.y - b.y, a.z - b.z};
}

//! length(a - b); invalid if a coordinate isn't finite.
inline Value
distance(const Vec3 &a, const Vec3 &b)
{
	if (!finite(a) || !finite(b)) {
		return {};
	}
	const Vec3 d = sub(a, b);
	const float l = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
	if (!std::isfinite(l)) {
		return {};
	}
	return {true, l};
}

//! The angle at @p b between (a - b) and (c - b), degrees 0..180; invalid for a zero-length side or bad input.
inline Value
angleDeg(const Vec3 &a, const Vec3 &b, const Vec3 &c)
{
	if (!finite(a) || !finite(b) || !finite(c)) {
		return {};
	}
	const Vec3 u = sub(a, b), v = sub(c, b);
	const float lu = std::sqrt(u.x * u.x + u.y * u.y + u.z * u.z);
	const float lv = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	if (!(lu > kEpsilon) || !(lv > kEpsilon)) {
		return {};
	}
	float cosine = (u.x * v.x + u.y * v.y + u.z * v.z) / (lu * lv);
	cosine = std::fmax(-1.0f, std::fmin(1.0f, cosine));
	return {true, std::acos(cosine) * 57.29577951308232f};
}

//! One hand's 21 landmarks.
struct Landmarks
{
	Vec3 p[kLandmarkCount];
	bool valid = false; //!< all 21 present
};

//! Palm width: index knuckle (5) to little finger knuckle (17).
inline Value
palmWidth(const Landmarks &lm)
{
	if (!lm.valid) {
		return {};
	}
	return distance(lm.p[INDEX_MCP], lm.p[LITTLE_MCP]);
}

//! distance(lm[a], lm[b]) / palm width; invalid when the palm width is ~0 or input is bad.
inline Value
normalizedDistance(const Landmarks &lm, int a, int b)
{
	if (!lm.valid || a < 0 || b < 0 || a >= kLandmarkCount || b >= kLandmarkCount) {
		return {};
	}
	const Value w = palmWidth(lm);
	const Value d = distance(lm.p[a], lm.p[b]);
	if (!w.valid || !d.valid || !(w.value > kEpsilon)) {
		return {};
	}
	return {true, d.value / w.value};
}

//! Tunable limits of the finger extension test.
struct ExtensionLimits
{
	float min_joint_angle_deg = 150.0f; //!< both middle joints of the finger at least this straight
	float min_tip_reach = 1.15f;        //!< tip-to-wrist at least this x the finger's base-to-wrist distance
	float thumb_min_angle_deg = 140.0f; //!< thumb: MCP and IP joints at least this straight
	float thumb_min_reach = 0.9f;       //!< thumb: tip to index knuckle at least this x palm width
};

/*!
 * Whether finger @p f (0 thumb ... 4 little) is extended: its middle joints nearly straight and its tip well away from
 * the wrist (the thumb: away from the index knuckle). Invalid with bad input.
 */
inline Value
fingerExtended(const Landmarks &lm, int f, const ExtensionLimits &lim = {})
{
	if (!lm.valid || f < 0 || f > 4) {
		return {};
	}
	if (f == 0) {
		const Value a1 = angleDeg(lm.p[THUMB_CMC], lm.p[THUMB_MCP], lm.p[THUMB_IP]);
		const Value a2 = angleDeg(lm.p[THUMB_MCP], lm.p[THUMB_IP], lm.p[THUMB_TIP]);
		const Value reach = normalizedDistance(lm, THUMB_TIP, INDEX_MCP);
		if (!a1.valid || !a2.valid || !reach.valid) {
			return {};
		}
		const bool ext = a1.value >= lim.thumb_min_angle_deg && a2.value >= lim.thumb_min_angle_deg &&
		                 reach.value >= lim.thumb_min_reach;
		return {true, ext ? 1.0f : 0.0f};
	}
	const int mcp = 1 + 4 * f, pip = mcp + 1, dip = mcp + 2, tip = mcp + 3;
	const Value a1 = angleDeg(lm.p[mcp], lm.p[pip], lm.p[dip]);
	const Value a2 = angleDeg(lm.p[pip], lm.p[dip], lm.p[tip]);
	const Value tip_reach = distance(lm.p[tip], lm.p[WRIST]);
	const Value base_reach = distance(lm.p[mcp], lm.p[WRIST]);
	if (!a1.valid || !a2.valid || !tip_reach.valid || !base_reach.valid || !(base_reach.value > kEpsilon)) {
		return {};
	}
	const bool ext = a1.value >= lim.min_joint_angle_deg && a2.value >= lim.min_joint_angle_deg &&
	                 tip_reach.value >= lim.min_tip_reach * base_reach.value;
	return {true, ext ? 1.0f : 0.0f};
}

//! What gestures need from one hand's landmarks.
struct HandGeometry
{
	bool valid = false;
	float pinch_distance_normalized = 0; //!< thumb tip (4) to index tip (8) / palm width
	uint8_t extended_mask = 0;           //!< bit f: finger f extended
};

inline HandGeometry
measure(const Landmarks &lm, const ExtensionLimits &lim = {})
{
	HandGeometry g;
	const Value pinch = normalizedDistance(lm, THUMB_TIP, INDEX_TIP);
	if (!pinch.valid) {
		return g;
	}
	for (int f = 0; f < 5; f++) {
		const Value e = fingerExtended(lm, f, lim);
		if (!e.valid) {
			return HandGeometry{};
		}
		g.extended_mask |= e.value > 0.5f ? (uint8_t)(1u << f) : 0;
	}
	g.pinch_distance_normalized = pinch.value;
	g.valid = true;
	return g;
}

/*!
 * The joint features calibrated gesture rules use (psvr2-test hands/gesture_fit.py computes the same): per finger
 * (index, middle, ring, little) the angle at the middle joint (PIP) and the end joint (DIP) in degrees (180 straight),
 * the reach (tip-to-wrist / knuckle-to-wrist: about 1.8 straight, under 1 curled), the thumb tip to that fingertip /
 * palm width; the thumb's IP angle and its tip to the index knuckle / palm width (small: tucked in).
 */
enum Feature : int
{
	F_INDEX_PIP = 0,
	F_INDEX_DIP,
	F_MIDDLE_PIP,
	F_MIDDLE_DIP,
	F_RING_PIP,
	F_RING_DIP,
	F_LITTLE_PIP,
	F_LITTLE_DIP,
	F_INDEX_REACH,
	F_MIDDLE_REACH,
	F_RING_REACH,
	F_LITTLE_REACH,
	F_THUMB_INDEX,
	F_THUMB_MIDDLE,
	F_THUMB_RING,
	F_THUMB_LITTLE,
	F_THUMB_IP,
	F_THUMB_OUT,
	//! Thumb tip to the index finger's middle section (the PIP-DIP segment, 6-7) / palm width (thumb tap).
	F_THUMB_INDEX_SIDE,
	/*!
	 * The thumb tip in the index finger's own frame, from the index knuckle (5), / palm width (thumb swipe): x along
	 * the finger (5 -> 8, toward the tip), y across it toward the thumb side (the knuckle line 17 -> 5 without its
	 * part along the finger; the same side on both hands, no handedness flip), z = x cross y.
	 */
	F_THUMB_LOCAL_X,
	F_THUMB_LOCAL_Y,
	F_THUMB_LOCAL_Z,
	kFeatureCount,
};

//! The features' names, as the gesture config and the analysis tools write them.
inline const char *
featureName(int f)
{
	static const char *const names[kFeatureCount] = {
	    "index_pip",    "index_dip",    "middle_pip",  "middle_dip",   "ring_pip",   "ring_dip",
	    "little_pip",   "little_dip",   "index_reach", "middle_reach", "ring_reach", "little_reach",
	    "thumb_index",  "thumb_middle", "thumb_ring",  "thumb_little", "thumb_ip",   "thumb_out",
	    "thumb_index_side", "thumb_local_x", "thumb_local_y", "thumb_local_z",
	};
	return f >= 0 && f < kFeatureCount ? names[f] : "";
}

//! The feature with this name, or -1.
inline int
featureIndex(const char *name)
{
	for (int f = 0; f < kFeatureCount && name != nullptr; f++) {
		const char *n = featureName(f);
		int i = 0;
		while (n[i] != '\0' && n[i] == name[i]) {
			i++;
		}
		if (n[i] == '\0' && name[i] == '\0') {
			return f;
		}
	}
	return -1;
}

inline Vec3
add(const Vec3 &a, const Vec3 &b)
{
	return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3
scale(const Vec3 &a, float s)
{
	return {a.x * s, a.y * s, a.z * s};
}

inline float
dot(const Vec3 &a, const Vec3 &b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3
cross(const Vec3 &a, const Vec3 &b)
{
	return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

//! @p a scaled to length 1; invalid for a ~zero or non-finite vector.
inline bool
normalized(const Vec3 &a, Vec3 *out)
{
	const float l = std::sqrt(dot(a, a));
	if (!std::isfinite(l) || !(l > kEpsilon)) {
		return false;
	}
	*out = scale(a, 1.0f / l);
	return true;
}

//! Distance from @p p to the segment @p a - @p b (the closest point clamped to the segment).
inline Value
pointSegmentDistance(const Vec3 &p, const Vec3 &a, const Vec3 &b)
{
	if (!finite(p) || !finite(a) || !finite(b)) {
		return {};
	}
	const Vec3 seg = sub(b, a);
	const float l2 = dot(seg, seg);
	float u = l2 > kEpsilon * kEpsilon ? dot(sub(p, a), seg) / l2 : 0.0f;
	u = std::fmax(0.0f, std::fmin(1.0f, u));
	return distance(p, add(a, scale(seg, u)));
}

/*!
 * The index finger's own frame (thumb swipes): x from the index knuckle (5) to its tip (8), y the knuckle line from the
 * little finger (17) to the index (5) without its part along x (toward the thumb side on both hands), z = x cross y.
 */
inline bool
indexFrame(const Landmarks &lm, Vec3 *x, Vec3 *y, Vec3 *z)
{
	if (!lm.valid || !normalized(sub(lm.p[INDEX_TIP], lm.p[INDEX_MCP]), x)) {
		return false;
	}
	const Vec3 across = sub(lm.p[INDEX_MCP], lm.p[LITTLE_MCP]);
	if (!normalized(sub(across, scale(*x, dot(across, *x))), y)) {
		return false;
	}
	*z = cross(*x, *y);
	return true;
}

//! All joint features of one hand; false (and @p out untouched) when any can't be measured.
inline bool
jointFeatures(const Landmarks &lm, float out[kFeatureCount])
{
	if (!lm.valid) {
		return false;
	}
	float f[kFeatureCount];
	for (int finger = 0; finger < 4; finger++) {
		const int mcp = INDEX_MCP + 4 * finger, pip = mcp + 1, dip = mcp + 2, tip = mcp + 3;
		const Value a1 = angleDeg(lm.p[mcp], lm.p[pip], lm.p[dip]);
		const Value a2 = angleDeg(lm.p[pip], lm.p[dip], lm.p[tip]);
		const Value tip_reach = distance(lm.p[tip], lm.p[WRIST]);
		const Value base_reach = distance(lm.p[mcp], lm.p[WRIST]);
		const Value thumb = normalizedDistance(lm, THUMB_TIP, tip);
		if (!a1.valid || !a2.valid || !tip_reach.valid || !base_reach.valid || !(base_reach.value > kEpsilon) ||
		    !thumb.valid) {
			return false;
		}
		f[F_INDEX_PIP + 2 * finger] = a1.value;
		f[F_INDEX_DIP + 2 * finger] = a2.value;
		f[F_INDEX_REACH + finger] = tip_reach.value / base_reach.value;
		f[F_THUMB_INDEX + finger] = thumb.value;
	}
	const Value ip = angleDeg(lm.p[THUMB_MCP], lm.p[THUMB_IP], lm.p[THUMB_TIP]);
	const Value out_v = normalizedDistance(lm, THUMB_TIP, INDEX_MCP);
	if (!ip.valid || !out_v.valid) {
		return false;
	}
	f[F_THUMB_IP] = ip.value;
	f[F_THUMB_OUT] = out_v.value;
	const Value palm = palmWidth(lm);
	const Value side = pointSegmentDistance(lm.p[THUMB_TIP], lm.p[INDEX_PIP], lm.p[INDEX_DIP]);
	Vec3 ax, ay, az;
	if (!palm.valid || !(palm.value > kEpsilon) || !side.valid || !indexFrame(lm, &ax, &ay, &az)) {
		return false;
	}
	const Vec3 rel = sub(lm.p[THUMB_TIP], lm.p[INDEX_MCP]);
	f[F_THUMB_INDEX_SIDE] = side.value / palm.value;
	f[F_THUMB_LOCAL_X] = dot(rel, ax) / palm.value;
	f[F_THUMB_LOCAL_Y] = dot(rel, ay) / palm.value;
	f[F_THUMB_LOCAL_Z] = dot(rel, az) / palm.value;
	for (int i = 0; i < kFeatureCount; i++) {
		out[i] = f[i];
	}
	return true;
}

} // namespace xrt::tracking::hand::mercury::gesture
