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

} // namespace xrt::tracking::hand::mercury::gesture
