// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Formula gestures on top of the tracked hand (header only, unit tested): the palm facing the headset (for the
 *         palm pinch menu gesture), middle / ring / little pinch strength, the index fingertip poke pose, and the thumb
 *         tap and thumb swipe microgestures on the side of the index finger.
 *
 * Inputs: Mercury's fitted joints (room frame, as the head pose) for the palm and the fingertip pose; MediaPipe's joint
 * features (Mercury's gesture::jointFeatures, hand-centred, / palm width) for the pinch strengths, tap and swipe. All
 * thresholds come from Params (the gesture config's "params", per hand; defaults below). psvr2-test
 * hands/microgestures.py runs the same tap / swipe rules on recordings.
 *
 * @ingroup st_ovrn
 */

#pragma once

#include "hg_gesture_geometry.hpp"

#include <cmath>
#include <cstdint>
#include <map>
#include <string>

namespace ovrn::gest::micro {

namespace geo = xrt::tracking::hand::mercury::gesture;
using geo::Vec3;

//! The formula gestures' tunables (one hand), from the gesture config's "params".
struct Params
{
	// Palm facing the headset (palm pinch menu gesture): the angle between the palm normal and the direction to the
	// headset, below this.
	float palm_angle_deg = 40.0f;
	// Pinch strength per finger (middle, ring, little): thumb tip to fingertip / palm width; 0 at "open", 1 at "closed",
	// smoothed (EMA weight of the new value).
	float pinch_open[3] = {1.10f, 1.25f, 1.45f};
	float pinch_closed[3] = {0.40f, 0.45f, 0.55f};
	float pinch_alpha = 0.5f;
	// Index fingertip poke: direction weights of the last two index segments; "poke ready" when the index is extended
	// (reach at or above index_extended) and the middle finger isn't (reach below middle_curled).
	float poke_distal_weight = 0.7f;
	float poke_proximal_weight = 0.3f;
	float index_extended = 1.6f;
	float middle_curled = 1.5f;
	// Thumb tap: thumb tip to the index's middle section / palm width; contact below tap_contact, release above
	// tap_release; a tap is contact then release within [tap_min_ms, tap_max_ms] after the thumb was clear for
	// tap_arm_ms; tap_cooldown_ms after a tap; two taps within double_ms are a double tap.
	float alpha = 0.5f;
	float tap_contact = 0.25f;
	float tap_release = 0.45f;
	float tap_min_ms = 50.0f;
	float tap_max_ms = 300.0f;
	float tap_arm_ms = 150.0f;
	float tap_cooldown_ms = 250.0f;
	float double_ms = 400.0f;
	// Thumb swipe: inside swipe_region (thumb to the index's side), a move of swipe_min palm widths along the finger
	// (forward / back) or across it (toward the thumb side / away) within [swipe_min_ms, swipe_max_ms], the dominant
	// axis swipe_ratio times the other; swipe_cooldown_ms after one. swipe_thumbward_is_left: whether a swipe toward
	// the thumb side is "left" (on the right hand, palm down, the thumb side is the left).
	float swipe_region = 0.45f;
	float swipe_min = 0.30f;
	float swipe_min_ms = 100.0f;
	float swipe_max_ms = 500.0f;
	float swipe_ratio = 2.0f;
	float swipe_cooldown_ms = 400.0f;
	float swipe_thumbward_is_left = 1.0f;
	// Measurements missing longer than this: everything starts over.
	float grace_ms = 100.0f;

	//! Defaults for @p hand (0 left, 1 right), then any of @p values (config "params") by name.
	static Params
	fromMap(int hand, const std::map<std::string, float> &values)
	{
		Params p;
		p.swipe_thumbward_is_left = hand == 1 ? 1.0f : 0.0f;
		const auto set = [&](const char *k, float *v) {
			auto it = values.find(k);
			if (it != values.end() && std::isfinite(it->second)) {
				*v = it->second;
			}
		};
		set("palm_angle_deg", &p.palm_angle_deg);
		static const char *const fingers[3] = {"middle", "ring", "little"};
		for (int i = 0; i < 3; i++) {
			set((std::string("pinch_open_") + fingers[i]).c_str(), &p.pinch_open[i]);
			set((std::string("pinch_closed_") + fingers[i]).c_str(), &p.pinch_closed[i]);
		}
		set("pinch_alpha", &p.pinch_alpha);
		set("poke_distal_weight", &p.poke_distal_weight);
		set("poke_proximal_weight", &p.poke_proximal_weight);
		set("index_extended", &p.index_extended);
		set("middle_curled", &p.middle_curled);
		set("alpha", &p.alpha);
		set("tap_contact", &p.tap_contact);
		set("tap_release", &p.tap_release);
		set("tap_min_ms", &p.tap_min_ms);
		set("tap_max_ms", &p.tap_max_ms);
		set("tap_arm_ms", &p.tap_arm_ms);
		set("tap_cooldown_ms", &p.tap_cooldown_ms);
		set("double_ms", &p.double_ms);
		set("swipe_region", &p.swipe_region);
		set("swipe_min", &p.swipe_min);
		set("swipe_min_ms", &p.swipe_min_ms);
		set("swipe_max_ms", &p.swipe_max_ms);
		set("swipe_ratio", &p.swipe_ratio);
		set("swipe_cooldown_ms", &p.swipe_cooldown_ms);
		set("swipe_thumbward_is_left", &p.swipe_thumbward_is_left);
		set("grace_ms", &p.grace_ms);
		return p;
	}
};

// ------------------------------------------------------------------------------------------------------ palm

/*!
 * The palm normal from the wrist (@p wrist), index knuckle (@p index_mcp) and little finger knuckle (@p little_mcp):
 * normalize(cross(index - wrist, little - wrist)), turned to the side of @p inner_hint (the palm side of the hand: the
 * cross product's sign follows handedness, so it's oriented by the tracked palm's own inner axis). False if degenerate.
 */
inline bool
palmNormal(const Vec3 &wrist, const Vec3 &index_mcp, const Vec3 &little_mcp, const Vec3 &inner_hint, Vec3 *out)
{
	Vec3 n;
	if (!geo::normalized(geo::cross(geo::sub(index_mcp, wrist), geo::sub(little_mcp, wrist)), &n)) {
		return false;
	}
	*out = geo::dot(n, inner_hint) < 0.0f ? geo::scale(n, -1.0f) : n;
	return true;
}

//! The angle (degrees) between @p palm_normal and the direction from @p palm_center to @p headset; false if degenerate.
inline bool
palmAngleDeg(const Vec3 &palm_normal, const Vec3 &palm_center, const Vec3 &headset, float *out)
{
	Vec3 to_head;
	if (!geo::normalized(geo::sub(headset, palm_center), &to_head)) {
		return false;
	}
	const float d = std::fmax(-1.0f, std::fmin(1.0f, geo::dot(palm_normal, to_head)));
	*out = std::acos(d) * 57.29577951308232f;
	return true;
}

// ------------------------------------------------------------------------------------------------- pinch strength

//! 0 (open) .. 1 (closed) from a thumb-to-fingertip distance / palm width.
inline float
pinchStrength(float normalized_distance, float open, float closed)
{
	if (!std::isfinite(normalized_distance) || !(open > closed)) {
		return 0.0f;
	}
	return std::fmax(0.0f, std::fmin(1.0f, (open - normalized_distance) / (open - closed)));
}

// ------------------------------------------------------------------------------------------------------ poke

/*!
 * The index fingertip's pointing direction: the distal segment (DIP -> tip) weighted with the one before it
 * (PIP -> DIP), normalised. False if degenerate.
 */
inline bool
fingertipDirection(const Vec3 &pip, const Vec3 &dip, const Vec3 &tip, float proximal_w, float distal_w, Vec3 *out)
{
	Vec3 a, b;
	if (!geo::normalized(geo::sub(dip, pip), &a) || !geo::normalized(geo::sub(tip, dip), &b)) {
		return false;
	}
	return geo::normalized(geo::add(geo::scale(a, proximal_w), geo::scale(b, distal_w)), out);
}

//! Whether the index is extended for a poke (reach at or above the limit) and the middle finger curled.
inline bool
pokeReady(const float *features, const Params &p)
{
	return features[geo::F_INDEX_REACH] >= p.index_extended && features[geo::F_MIDDLE_REACH] < p.middle_curled;
}

// --------------------------------------------------------------------------------------- thumb tap and swipe

enum Event : uint32_t
{
	E_TAP = 1u << 0,
	E_DOUBLE_TAP = 1u << 1,
	E_SWIPE_FORWARD = 1u << 2, //!< along the index finger, toward its tip
	E_SWIPE_BACK = 1u << 3,    //!< along the index finger, toward the knuckle
	E_SWIPE_LEFT = 1u << 4,
	E_SWIPE_RIGHT = 1u << 5,
};

//! Thumb tap and swipe on the side of the index finger, from the joint features (psvr2-test hands/microgestures.py).
class Micro
{
public:
	explicit Micro(const Params &p = {}) : m_p(p) {}

	//! The params (e.g. after the config was loaded).
	void
	setParams(const Params &p)
	{
		m_p = p;
	}

	/*!
	 * One camera frame at @p t_ns: @p features (valid or not); @p blocked while a pinch or fist is on (the thumb is at
	 * the index finger then for another reason). Returns the events (Event bits) of this frame.
	 */
	uint32_t
	update(int64_t t_ns, const float *features, bool valid, bool blocked)
	{
		if (t_ns == m_lastT) {
			return 0; // the same camera frame again
		}
		m_lastT = t_ns;
		if (m_lastValid != 0 && t_ns - m_lastValid > ms(m_p.grace_ms)) {
			reset();
		}
		if (!valid || blocked) {
			const int64_t keep = valid ? t_ns : m_lastValid;
			reset();
			m_lastValid = keep;
			return 0;
		}
		m_lastValid = t_ns;
		const float side = features[geo::F_THUMB_INDEX_SIDE];
		const float lx = features[geo::F_THUMB_LOCAL_X];
		const float ly = features[geo::F_THUMB_LOCAL_Y];
		if (!std::isfinite(side) || !std::isfinite(lx) || !std::isfinite(ly)) {
			return 0;
		}
		if (!m_have) {
			m_side = side, m_x = lx, m_y = ly, m_have = true;
		} else {
			const float a = m_p.alpha;
			m_side = a * side + (1 - a) * m_side;
			m_x = a * lx + (1 - a) * m_x;
			m_y = a * ly + (1 - a) * m_y;
		}
		return tap(t_ns) | swipe(t_ns);
	}

	void
	reset()
	{
		m_have = false;
		m_tap = Tap::Idle;
		m_armedSince = 0;
		m_contactT = 0;
		m_coolUntil = 0;
		m_lastTap = 0;
		m_swStart = 0;
		m_swDone = false;
		m_swCoolUntil = 0;
		m_lastValid = 0;
	}

	//! The smoothed thumb-to-index-side distance (debugging).
	float
	side() const
	{
		return m_have ? m_side : NAN;
	}

private:
	enum class Tap
	{
		Idle,
		Armed,
		Contact,
		Held,
	};

	static int64_t
	ms(float v)
	{
		return (int64_t)(v * 1e6);
	}

	uint32_t
	tap(int64_t t)
	{
		uint32_t ev = 0;
		switch (m_tap) {
		case Tap::Idle:
			if (m_side > m_p.tap_release) {
				m_armedSince = m_armedSince != 0 ? m_armedSince : t;
				if (t - m_armedSince >= ms(m_p.tap_arm_ms) && t >= m_coolUntil) {
					m_tap = Tap::Armed;
				}
			} else {
				m_armedSince = 0;
			}
			break;
		case Tap::Armed:
			if (m_side < m_p.tap_contact) {
				m_tap = Tap::Contact;
				m_contactT = t;
			}
			break;
		case Tap::Contact: {
			const int64_t dt = t - m_contactT;
			if (m_side > m_p.tap_release) {
				if (dt >= ms(m_p.tap_min_ms) && dt <= ms(m_p.tap_max_ms)) {
					ev |= E_TAP;
					if (m_lastTap != 0 && t - m_lastTap <= ms(m_p.double_ms)) {
						ev |= E_DOUBLE_TAP;
						m_lastTap = 0;
					} else {
						m_lastTap = t;
					}
					m_coolUntil = t + ms(m_p.tap_cooldown_ms);
				}
				m_tap = Tap::Idle;
				m_armedSince = 0;
			} else if (dt > ms(m_p.tap_max_ms)) {
				m_tap = Tap::Held; // a held contact isn't a tap
			}
			break;
		}
		case Tap::Held:
			if (m_side > m_p.tap_release) {
				m_tap = Tap::Idle;
				m_armedSince = 0;
			}
			break;
		}
		return ev;
	}

	uint32_t
	swipe(int64_t t)
	{
		if (!(m_side < m_p.swipe_region) || t < m_swCoolUntil) {
			m_swStart = 0;
			return 0;
		}
		if (m_swStart == 0) {
			m_swStart = t, m_x0 = m_x, m_y0 = m_y, m_swDone = false;
			return 0;
		}
		if (m_swDone) {
			return 0;
		}
		const int64_t dt = t - m_swStart;
		const float dx = m_x - m_x0, dy = m_y - m_y0;
		if (dt > ms(m_p.swipe_max_ms)) {
			m_swStart = t, m_x0 = m_x, m_y0 = m_y; // too slow: start over from here
			return 0;
		}
		const float ax = std::fabs(dx), ay = std::fabs(dy);
		if (dt < ms(m_p.swipe_min_ms) || std::fmax(ax, ay) < m_p.swipe_min) {
			return 0;
		}
		const float big = std::fmax(ax, ay), small = std::fmin(ax, ay);
		if (big < m_p.swipe_ratio * small) {
			return 0; // diagonal: not a swipe
		}
		m_swDone = true;
		m_swCoolUntil = t + ms(m_p.swipe_cooldown_ms);
		if (ax >= ay) {
			return dx > 0 ? E_SWIPE_FORWARD : E_SWIPE_BACK;
		}
		const bool thumbward = dy > 0;
		const bool left = thumbward == (m_p.swipe_thumbward_is_left > 0.5f);
		return left ? E_SWIPE_LEFT : E_SWIPE_RIGHT;
	}

	Params m_p;
	bool m_have = false;
	float m_side = 0, m_x = 0, m_y = 0;
	Tap m_tap = Tap::Idle;
	int64_t m_armedSince = 0, m_contactT = 0, m_coolUntil = 0, m_lastTap = 0;
	int64_t m_swStart = 0, m_swCoolUntil = 0;
	float m_x0 = 0, m_y0 = 0;
	bool m_swDone = false;
	int64_t m_lastValid = 0, m_lastT = 0;
};

} // namespace ovrn::gest::micro
