// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Per-hand gesture state from MediaPipe's gesture measurements (header only, no SteamVR, unit tested).
 *
 * The hand tracker measures each camera frame (backend: HandGestures; Mercury with MediaPipe's models): the thumb to
 * index tip distance over the palm width, which fingers are extended, and the Gesture Recognizer's category. This
 * turns those per-frame measurements into steady gestures:
 *  - pinch (the trigger): hysteresis (on below pinch_on, off above pinch_off) and a few frames' confirmation each way;
 *  - fist (the grip): the Gesture Recognizer's Closed_Fist with enough score, confirmed; while a fist is held, no
 *    pinch (a fist brings the thumb to the index finger);
 *  - open palm (the dashboard gesture's first half, SystemGesture): its Open_Palm, confirmed;
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
	/*
	 * No "other fingers straight" rule for the pinch: on the 2026-11-24 labelled recording it kept the right hand's
	 * pinches (6/6) but cut the left's from 3/5 to 1/5 (MediaPipe reads the left hand's fingers as bent more often).
	 */
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
	bool fist = false;
	bool openPalm = false;

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
		m_fist.reset();
		m_open.reset();
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
		if (m_fist.on) {
			if (m_pinch.on) {
				e.pinch_released = true;
				e.reason = "fist: no pinch";
			}
			m_pinch.reset();
		} else if (m_pinch.step(d < m_cfg.pinch_on, d > m_cfg.pinch_off, n)) {
			(m_pinch.on ? e.pinch_pressed : e.pinch_released) = true;
			e.reason = m_pinch.on ? "pinch distance below pinch_on" : "pinch distance above pinch_off";
		}
		m_open.step(is(OpenPalm), !is(OpenPalm), n);

		m_state.valid = true;
		m_state.timestamp_ns = s.timestamp_ns;
		m_state.confidence = s.confidence;
		m_state.pinch = m_pinch.on;
		m_state.fist = m_fist.on;
		m_state.openPalm = m_open.on;
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
	detail::Debounced m_pinch, m_fist, m_open;
	int64_t m_lastTs = 0;
};

/*!
 * The dashboard (system button) gesture: an open palm toward the face, then a fist.
 *
 * Why not "palm toward the face, pinch and hold" (until 2026-11-24): on both labelled sessions (20:50, 21:47) it
 * caught 0 of 8 dashboard attempts. With the hand raised toward the face MediaPipe sees a clear open palm (presence
 * 0.99, Open_Palm) and the thumb-index gap stays about 1.2 palm widths: the pinch isn't seen there. Open palm and fist
 * are the two shapes it recognises best (fist: every grip prompt in every session).
 *
 * Arming: the open palm with the pose (palm toward the face, raised) for palm_ns, with gaps up to gap_ns (invalid
 * frames while the hand turns). Then a fist within fist_within_ns presses the system button; it stays down while the
 * fist is held and goes up when the fist ends. A fist on its own (no open palm toward the face first) does nothing.
 * The open palm must be held still a moment (palm_ns) and the fist must follow quickly (fist_within_ns): looking at
 * the open hand and then grabbing something is an everyday move too (see Config).
 */
class SystemGesture
{
public:
	struct Config
	{
		/*
		 * 0.6 s and 0.5 s since the 2026-11-24 23:37 session: 0.3 s and 1 s let a look at the open hand followed
		 * by a grab toggle the dashboard (replay: 9 presses in 40 s with the -0.25 m height, 2 now). The open palm
		 * on the 8 deliberate attempts of the 20:50 and 21:47 sessions lasted 0.6 s on 5 (not asked to hold it).
		 */
		int64_t palm_ns = 600'000'000;
		int64_t gap_ns = 150'000'000;
		int64_t fist_within_ns = 500'000'000;
	};

	SystemGesture() = default;
	explicit SystemGesture(const Config &cfg) : m_cfg(cfg) {}

	/*!
	 * One frame. pose_ok: palm toward the face and raised; open_palm, fist: the steady gestures (Machine::state()).
	 * Returns whether the system button is down.
	 */
	bool
	update(int64_t now_ns, bool pose_ok, bool open_palm, bool fist)
	{
		if (m_down) {
			m_down = fist;
			return m_down;
		}
		if (pose_ok && open_palm) {
			if (!m_palm || now_ns - m_lastOkNs > m_cfg.gap_ns) {
				m_sinceNs = now_ns;
			}
			m_palm = true;
			m_lastOkNs = now_ns;
			if (now_ns - m_sinceNs >= m_cfg.palm_ns) {
				m_armed = true;
				m_armedUntilNs = now_ns + m_cfg.fist_within_ns;
			}
		}
		if (fist && armed(now_ns)) {
			m_down = true;
			m_armed = m_palm = false;
		}
		return m_down;
	}

	//! Whether a fist now would press the button (the open palm was seen, the time to close it hasn't run out).
	bool
	armed(int64_t now_ns) const
	{
		return m_armed && now_ns <= m_armedUntilNs;
	}

	bool
	down() const
	{
		return m_down;
	}

	//! The hand is lost: let go and forget the open palm.
	void
	reset()
	{
		m_down = m_armed = m_palm = false;
	}

private:
	Config m_cfg;
	bool m_down = false;
	bool m_palm = false;  //!< the open palm toward the face seen (since m_sinceNs, last at m_lastOkNs)
	bool m_armed = false; //!< the open palm lasted palm_ns: a fist until m_armedUntilNs presses
	int64_t m_sinceNs = 0, m_lastOkNs = 0, m_armedUntilNs = 0;
};

} // namespace ovrn::gest
