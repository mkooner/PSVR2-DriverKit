// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Per-hand gesture state from MediaPipe's gesture measurements (header only, no SteamVR, unit tested).
 *
 * The hand tracker measures each camera frame (backend: HandGestures; Mercury with MediaPipe's models): the thumb to
 * index tip distance over the palm width, which fingers are extended, and the Gesture Recognizer's category. This
 * turns those per-frame measurements into steady gestures:
 *  - pinch (the trigger): hysteresis (on below pinch_on, off above pinch_off), each change held a minimum time
 *    (pinch_press_ns, pinch_release_ns);
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
#include <deque>

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

//! The custom gesture mapping's gestures (t_hand_custom_gesture).
enum Custom : int
{
	CustomOpen = 0,
	CustomPinch = 1,
	CustomFist = 2,
	CustomPoint = 3,
	CustomClaw = 4,
	CustomCount = 5,
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
	//! The driver's custom gesture mapping (fitted on PS VR2 camera images): used for pinch and fist when valid.
	bool custom_valid = false;
	int custom = CustomOpen;
	float custom_scores[CustomCount] = {};
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
	/*!
	 * The pinch (trigger) switches on only after the distance has stayed below pinch_on this long, and off after it has
	 * stayed above pinch_off this long (camera frame times), instead of confirm_frames. 2026-11-25: with two frames
	 * (~33 ms) a third to half of the trigger presses lasted under 0.15 s, in the 13:35 headset session (25-27%) and in
	 * hand_sim on the benchmark (33-48%): MediaPipe's thumb-index distance jumping for a frame or two, and the thumb
	 * passing the index finger while closing into a fist. On the benchmark's readings 60 ms took those from 12 to 3 and
	 * re-presses from 3 to 0, presses in non-pinch prompts 16 -> 10, pinch prompts 10 -> 9 of 12; longer lost pinches.
	 */
	int64_t pinch_press_ns = 60'000'000;
	int64_t pinch_release_ns = 60'000'000;
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
	bool customValid = false;      //!< the driver's custom mapping decided pinch and fist this frame
	float customPinchScore = 0.0f; //!< its pinch probability
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
//! A boolean that changes only after the asked-for state has lasted a given time (camera frame times).
struct TimedDebounced
{
	bool on = false;
	int64_t since = 0; //!< when the asked-for change began (0: not asked)

	//! @p want_on / @p want_off as Debounced::step; @p on_ns / @p off_ns: how long each change must be asked for.
	bool
	step(bool want_on, bool want_off, int64_t t, int64_t on_ns, int64_t off_ns)
	{
		const bool asking = on ? want_off : want_on;
		if (!asking) {
			since = 0;
			return false;
		}
		if (since == 0) {
			since = t;
		}
		if (t - since >= (on ? off_ns : on_ns)) {
			on = !on;
			since = 0;
			return true;
		}
		return false;
	}
	void
	reset()
	{
		on = false;
		since = 0;
	}
};

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

		/*
		 * The custom mapping, when the driver has it: its gesture decides fist and pinch (open, point and claw:
		 * neither), each still held a minimum time. MediaPipe's own categories and pinch distance read the same for a
		 * pinch and an open hand on these cameras (2026-11-25 benchmark: thumb-index / palm width 0.89 vs 0.77) and
		 * almost never saw a fist (0-4% of fist readings); the mapping caught 5 of 6 pinch and 2 of 2 fist prompts per
		 * hand on the round it wasn't fitted on.
		 */
		const bool custom = s.custom_valid;
		const bool fist_on = custom ? s.custom == CustomFist : is(ClosedFist);
		if (m_fist.step(fist_on, !fist_on, n)) {
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
		} else if (custom) {
			const bool pinch_on = s.custom == CustomPinch;
			if (m_pinch.step(pinch_on, !pinch_on, s.timestamp_ns, m_cfg.pinch_press_ns, m_cfg.pinch_release_ns)) {
				(m_pinch.on ? e.pinch_pressed : e.pinch_released) = true;
				e.reason = m_pinch.on ? "custom mapping: pinch" : "custom mapping: no pinch";
			}
		} else if (m_pinch.step(d < m_cfg.pinch_on, d > m_cfg.pinch_off, s.timestamp_ns, m_cfg.pinch_press_ns,
		                        m_cfg.pinch_release_ns)) {
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
		m_state.customValid = custom;
		m_state.customPinchScore = custom ? s.custom_scores[CustomPinch] : 0.0f;
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
		if (m_state.customValid) {
			// The custom mapping's pinch probability, held at 1 while its pinch is on (the click).
			return m_state.pinch ? 1.0f : std::fmax(0.0f, std::fmin(1.0f, m_state.customPinchScore));
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
		       std::isfinite(s.category_score) && std::isfinite(s.custom_scores[CustomPinch]);
	}

	Config m_cfg;
	State m_state;
	Stats m_stats;
	detail::TimedDebounced m_pinch;
	detail::Debounced m_fist, m_open;
	int64_t m_lastTs = 0;
};

/*!
 * The dashboard (system button) gesture: an open palm toward the face, raised, held for a moment.
 *
 * Built only from what MediaPipe reads reliably with the palm facing the headset's cameras (2026-11-25 benchmark,
 * camera-20261125-002624, the four dashboard prompts and all 12 pinch prompts):
 *  - the palm's direction and height (the tracked 3D hand): matched every prompted palm orientation;
 *  - MediaPipe's Open_Palm category, judged over a window: on a palm facing the cameras it alternates with "none"
 *    (right-hand prompt: open on 92 frames, none on 312), so requiring it on every frame made the result depend on
 *    where the flickers fell (hand_sim, same recording: 1 of 4 prompts in one run, 4 of 4 in another).
 * Not used, because MediaPipe misreads them in this pose: a fist (Closed_Fist in 0 of the 4 prompts after the open
 * palm), finger straightness (an open palm read as 0-1 straight fingers) and a held pinch (under the threshold on 5-81
 * of ~270 readings in palm-toward pinch prompts). The earlier designs depended on those and never opened (palm + held
 * pinch with straight fingers: 0 of 8; open palm then fist: 0 of 4).
 *
 * Over the last window_ns: the pose (palm toward the face, raised) on at least pose_min of the frames, MediaPipe's
 * Open_Palm on at least open_min of its valid readings and a fist or pinch on at most bad_max. Then the system button
 * goes down and stays down until the pose drops below rearm_below of the window, so one hold is one press (SteamVR
 * toggles the dashboard per press). hand_sim on the benchmark (three runs): 4 of 4 dashboard prompts every run; presses
 * elsewhere 0, 1, 1, each in an "open hand" prompt where the open palm was held up toward the face (the same pose).
 */
class SystemGesture
{
public:
	struct Config
	{
		int64_t window_ns = 1'000'000'000;
		float pose_min = 0.95f;
		float open_min = 0.40f;
		float bad_max = 0.10f;
		float rearm_below = 0.50f;
	};

	SystemGesture() = default;
	explicit SystemGesture(const Config &cfg) : m_cfg(cfg) {}

	/*!
	 * One frame. pose_ok: palm toward the face and raised; gestures_valid: MediaPipe measured the hand this frame;
	 * open_palm: its Open_Palm (enough score); fist_or_pinch: its Closed_Fist (enough score) or thumb and index together.
	 * Returns whether the system button is down.
	 */
	bool
	update(int64_t now_ns, bool pose_ok, bool gestures_valid, bool open_palm, bool fist_or_pinch)
	{
		m_frames.push_back({now_ns, pose_ok, gestures_valid, gestures_valid && open_palm,
		                    gestures_valid && fist_or_pinch});
		while (!m_frames.empty() && m_frames.front().t < now_ns - m_cfg.window_ns) {
			m_frames.pop_front();
		}
		int pose = 0, valid = 0, open = 0, bad = 0;
		for (const Frame &f : m_frames) {
			pose += f.pose ? 1 : 0;
			valid += f.valid ? 1 : 0;
			open += f.open ? 1 : 0;
			bad += f.bad ? 1 : 0;
		}
		const float n = (float)m_frames.size();
		const float pose_share = pose / n;
		const bool full = m_frames.back().t - m_frames.front().t >= m_cfg.window_ns * 95 / 100;
		if (m_down) {
			m_down = pose_share >= m_cfg.rearm_below;
			return m_down;
		}
		if (full && pose_share >= m_cfg.pose_min && valid > 0 && (float)open / valid >= m_cfg.open_min &&
		    (float)bad / valid <= m_cfg.bad_max) {
			m_down = true;
		}
		return m_down;
	}

	bool
	down() const
	{
		return m_down;
	}

	//! Whether the pose is on in the latest frame (for diagnostics).
	bool
	posing() const
	{
		return !m_frames.empty() && m_frames.back().pose;
	}

	//! The hand is lost: let go and forget the window.
	void
	reset()
	{
		m_frames.clear();
		m_down = false;
	}

private:
	struct Frame
	{
		int64_t t;
		bool pose, valid, open, bad;
	};
	Config m_cfg;
	std::deque<Frame> m_frames;
	bool m_down = false;
};

} // namespace ovrn::gest
