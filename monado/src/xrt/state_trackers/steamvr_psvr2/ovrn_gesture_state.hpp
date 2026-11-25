// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Per-hand gesture state from MediaPipe's gesture measurements (header only, no SteamVR, unit tested).
 *
 * The hand tracker measures each camera frame (backend: HandGestures; Mercury with MediaPipe's models): the thumb to
 * index tip distance over the palm width, which fingers are extended, and the Gesture Recognizer's category. This
 * turns those per-frame measurements into steady gestures:
 *  - pinch: hysteresis (on below pinch_on, off above pinch_off) and a few frames' confirmation each way;
 *  - deliberate pinch (for the system gesture): a pinch with the other fingers open (deliberate_min_other_fingers of
 *    middle, ring, little straight): a fist or a pointing hand brings the thumb to the index finger with them curled;
 *  - fist: the Gesture Recognizer's Closed_Fist with enough score, confirmed; while a fist is held, no pinch (a fist
 *    brings the thumb to the index finger);
 *  - open palm, victory, I-love-you: the Gesture Recognizer's categories, confirmed; point: geometry (index extended,
 *    the other three fingers not), as the recognizer's Pointing_Up needs the finger up in the image;
 *  - everything lets go at once when the hand is lost, the measurement is invalid, old, of low confidence, or its time
 *    goes backwards (a new association); a press is reported once per activation.
 * Only a new camera frame (a later timestamp) advances the confirmations, however often update() is called.
 *
 * @ingroup st_ovrn
 */

#pragma once

#include <cstdint>
#include <cmath>

namespace ovrn::gest {

//! The Gesture Recognizer's categories (the order of its output; t_hand_gesture_category).
enum Category : int
{
	None = 0,
	ClosedFist = 1,
	OpenPalm = 2,
	PointingUp = 3,
	ThumbDown = 4,
	ThumbUp = 5,
	Victory = 6,
	ILoveYou = 7,
};

//! One camera frame's measurements of one hand (as the backend reports them).
struct Sample
{
	bool valid = false;
	int64_t timestamp_ns = 0;
	float confidence = 0.0f;
	float pinch_distance_normalized = 0.0f;
	uint8_t extended_mask = 0; //!< bit f: finger f extended (0 thumb ... 4 little finger)
	int category = None;
	float category_score = 0.0f;
};

//! Tuning (heuristics; see the defaults' notes).
struct Config
{
	/*!
	 * Pinch thresholds on thumb-to-index-tip / palm width. The usual starting values, 0.25 / 0.35, almost never fire
	 * on PS VR2 camera images: on the 2026-11-24 labelled recording (Mercury's MediaPipe crops) the right hand's pinch
	 * frames had median 0.61 (10% below 0.29) and open hands median 1.29 (10% below 0.83). 0.45 / 0.60 caught 6 of 6
	 * right-hand pinch prompts with 2 of 13 open-hand prompts clicking (0.25 / 0.35: 2 of 6).
	 * 0.50 / 0.65 since the sweep over the three labelled 2026-11-24 sessions (16:18, 20:50, 21:47; on 0.35-0.60,
	 * release gap 0.10-0.25, 1-3 confirmation frames, both hands per prompt): pinch prompts 31/35 caught vs 26/35
	 * at 0.45 / 0.60, false clicks on open/claw/point/fist prompts 31/112 vs 27/112 (one confirmation frame: 30/35
	 * and 41/112). Chosen on two sessions, it held on the third.
	 */
	float pinch_on = 0.50f;
	float pinch_off = 0.65f;
	float min_confidence = 0.5f; //!< the landmark model's hand presence needed
	/*!
	 * How many of the middle, ring and little fingers must be straight for a pinch (pinch_min_other_fingers) or a
	 * deliberate pinch (deliberate_min_other_fingers) to start; it lets go when fewer than one less are. 2026-11-24
	 * labelled recording, frames with thumb and index close: real pinches had 2 or more straight in 177 of 205,
	 * fists in 0 of 94, pointing in 0 of 20, open hands in 3 of 20. Per prompt, requiring it for the trigger kept the
	 * right hand's pinches (6/6) but cut the left's from 3/5 to 1/5 (MediaPipe reads the left hand's fingers as bent
	 * more often), so the trigger doesn't require it; the system gesture, where a false press opens the dashboard
	 * and takes the input away, does. 0: no requirement.
	 */
	int pinch_min_other_fingers = 0;
	int deliberate_min_other_fingers = 2;
	/*!
	 * The Gesture Recognizer's score needed for its categories (the top of 8 at 0.5 is a clear winner). 0.6 missed a
	 * held fist whose Closed_Fist score sat at 0.51 throughout (same recording); Closed_Fist never came up outside the
	 * fist prompts there.
	 */
	float min_category_score = 0.5f;
	int confirm_frames = 2;          //!< consecutive camera frames needed to switch a gesture on or off
	int64_t max_age_ns = 200'000'000; //!< a measurement older than this (vs now) counts as lost
};

//! The steady gestures of one hand.
struct State
{
	bool valid = false;
	int64_t timestamp_ns = 0;
	float confidence = 0.0f;

	bool pinch = false;
	bool deliberatePinch = false; //!< a pinch with the other fingers open (the system gesture's)
	bool fist = false;
	bool openPalm = false;
	bool point = false;
	bool victory = false;
	bool iLoveYou = false;

	float pinchDistanceNormalized = 0.0f;
};

//! What changed in one update (presses reported once per activation).
struct Events
{
	bool pinch_pressed = false, pinch_released = false;
	bool fist_pressed = false, fist_released = false;
	const char *reason = nullptr; //!< why a gesture changed or everything was let go (nullptr: nothing happened)
};

//! Counters for the diagnostics (since the last take).
struct Stats
{
	uint32_t frames = 0, rejected_invalid = 0, rejected_stale = 0, rejected_low_confidence = 0, reassociated = 0;
	uint32_t category[8] = {};
	float min_pinch = 1e9f;
};

namespace detail {

//! A boolean that changes only after @p n consecutive frames asking for the change.
struct Debounced
{
	bool on = false;
	int toward = 0;

	//! @p want_on: this frame asks for on; @p want_off: asks for off (both false: hold, between thresholds).
	bool
	step(bool want_on, bool want_off, int n)
	{
		if (!on && want_on) {
			toward++;
		} else if (on && want_off) {
			toward++;
		} else {
			toward = 0;
		}
		if (toward >= n) {
			on = !on;
			toward = 0;
			return true;
		}
		return false;
	}
	void
	reset()
	{
		on = false;
		toward = 0;
	}
};

} // namespace detail

class Machine
{
public:
	explicit Machine(const Config &c = {}) : m_cfg(c) {}

	const State &
	state() const
	{
		return m_state;
	}

	const Config &
	config() const
	{
		return m_cfg;
	}

	//! Takes the counters (and clears them).
	Stats
	takeStats()
	{
		Stats s = m_stats;
		m_stats = Stats{};
		return s;
	}

	//! Lets go of everything (lost hand, shutdown).
	Events
	reset(const char *reason)
	{
		Events e;
		if (m_state.pinch) {
			e.pinch_released = true;
		}
		if (m_state.fist) {
			e.fist_released = true;
		}
		if (e.pinch_released || e.fist_released) {
			e.reason = reason;
		}
		m_state = State{};
		m_pinch.reset();
		m_deliberate.reset();
		m_fist.reset();
		m_open.reset();
		m_point.reset();
		m_victory.reset();
		m_ily.reset();
		m_lastTs = 0;
		return e;
	}

	//! One look at the latest measurement (any rate); @p now_ns on the same clock as the measurements.
	Events
	update(const Sample &s, int64_t now_ns)
	{
		if (!s.valid || !finite(s)) {
			m_stats.rejected_invalid++;
			return reset("hand lost or measurement invalid");
		}
		if (now_ns - s.timestamp_ns > m_cfg.max_age_ns) {
			m_stats.rejected_stale++;
			return reset("measurement too old");
		}
		if (s.confidence < m_cfg.min_confidence) {
			m_stats.rejected_low_confidence++;
			return reset("low confidence");
		}
		if (m_lastTs != 0 && s.timestamp_ns < m_lastTs) {
			m_stats.reassociated++;
			return reset("measurement time went backwards");
		}
		if (s.timestamp_ns == m_lastTs) {
			return Events{}; // the same camera frame again
		}
		Events e;
		if (m_lastTs != 0 && s.timestamp_ns - m_lastTs > m_cfg.max_age_ns) {
			// A gap since the last frame: a re-found hand starts fresh.
			m_stats.reassociated++;
			e = reset("hand found again after a gap");
		}
		m_lastTs = s.timestamp_ns;
		m_stats.frames++;
		m_stats.category[(s.category >= 0 && s.category < 8) ? s.category : 0]++;
		m_stats.min_pinch = std::fmin(m_stats.min_pinch, s.pinch_distance_normalized);

		const int n = m_cfg.confirm_frames;
		const bool cat_ok = s.category_score >= m_cfg.min_category_score;
		const auto is = [&](int c) { return s.category == c && cat_ok; };

		if (m_fist.step(is(ClosedFist), !is(ClosedFist), n)) {
			(m_fist.on ? e.fist_pressed : e.fist_released) = true;
			e.reason = m_fist.on ? "fist recognised" : "fist ended";
		}
		const float d = s.pinch_distance_normalized;
		const int others = ((s.extended_mask >> 2) & 1) + ((s.extended_mask >> 3) & 1) + ((s.extended_mask >> 4) & 1);
		const bool fingers_open = others >= m_cfg.pinch_min_other_fingers;
		const bool fingers_closed = others + 1 < m_cfg.pinch_min_other_fingers;
		const bool deliberate_open = others >= m_cfg.deliberate_min_other_fingers;
		const bool deliberate_closed = others + 1 < m_cfg.deliberate_min_other_fingers;
		if (m_fist.on) {
			m_deliberate.reset();
		} else {
			m_deliberate.step(d < m_cfg.pinch_on && deliberate_open, d > m_cfg.pinch_off || deliberate_closed, n);
		}
		if (m_fist.on) {
			if (m_pinch.on) {
				e.pinch_released = true;
				e.reason = "fist: no pinch";
			}
			m_pinch.reset();
		} else if (m_pinch.step(d < m_cfg.pinch_on && fingers_open, d > m_cfg.pinch_off || fingers_closed, n)) {
			(m_pinch.on ? e.pinch_pressed : e.pinch_released) = true;
			e.reason = m_pinch.on ? "pinch distance below pinch_on, other fingers open"
			                      : (d > m_cfg.pinch_off ? "pinch distance above pinch_off" : "other fingers curled");
		}
		m_open.step(is(OpenPalm), !is(OpenPalm), n);
		m_victory.step(is(Victory), !is(Victory), n);
		m_ily.step(is(ILoveYou), !is(ILoveYou), n);
		const bool pointing = (s.extended_mask & 0x02) != 0 && (s.extended_mask & 0x1c) == 0;
		m_point.step(pointing, !pointing, n);

		m_state.valid = true;
		m_state.timestamp_ns = s.timestamp_ns;
		m_state.confidence = s.confidence;
		m_state.pinch = m_pinch.on;
		m_state.deliberatePinch = m_deliberate.on;
		m_state.fist = m_fist.on;
		m_state.openPalm = m_open.on;
		m_state.point = m_point.on;
		m_state.victory = m_victory.on;
		m_state.iLoveYou = m_ily.on;
		m_state.pinchDistanceNormalized = d;
		return e;
	}

	/*!
	 * A trigger-like value 0..1 from the pinch distance: 1 at pinch_on and closer, 0 at twice pinch_off and farther
	 * (only while valid; 0 while a fist is held).
	 */
	float
	pinchValue() const
	{
		if (!m_state.valid || m_state.fist) {
			return 0.0f;
		}
		const float open = 2.0f * m_cfg.pinch_off;
		const float v = (open - m_state.pinchDistanceNormalized) / (open - m_cfg.pinch_on);
		return std::fmax(0.0f, std::fmin(1.0f, v));
	}

private:
	static bool
	finite(const Sample &s)
	{
		return std::isfinite(s.confidence) && std::isfinite(s.pinch_distance_normalized) &&
		       std::isfinite(s.category_score);
	}

	Config m_cfg;
	State m_state;
	Stats m_stats;
	detail::Debounced m_pinch, m_deliberate, m_fist, m_open, m_point, m_victory, m_ily;
	int64_t m_lastTs = 0;
};

} // namespace ovrn::gest
