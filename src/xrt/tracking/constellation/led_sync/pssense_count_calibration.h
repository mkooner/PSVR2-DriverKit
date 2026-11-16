// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief PS Sense LED sync calibration by counting the blobs the cameras see.
 *
 * Follows PSVR2Toolkit's custom-controller-sync latency calibration (libpad_hooks.cpp, MIT): blink all LEDs every
 * frame and binary search, by whether the blob count rises when they're on, the earliest and latest blink offsets that
 * the exposure still sees, confirm both edges, and centre the blink between them.
 *
 * The count is of everything all cameras see, so lamps and reflections are in it and change as the head moves (on
 * hardware 0 to 70 blobs between runs, 70 -> 10 within one calibration). So unlike the Toolkit's one baseline at the
 * start, every step measures the LEDs at the test offset and then off right after, and only the difference counts. Unlike the edge searches, no step needs a solved pose, which on PS VR2 is often
 * missing even with the LEDs in view.
 *
 * Only one controller calibrates at a time, the others keep their LEDs off until their turn, so the counts are of
 * one controller.
 *
 * @ingroup aux_tracking
 */

#pragma once

#include "led_sync_internal.h"

#include <stdatomic.h>


//! Frames of blob counts averaged per decision, once the LED change settled (counts are clean: 0 off, 15-30 on).
#define COUNT_FRAMES_PER_STEP 2
//! Frames of blob counts for the LEDs-off baseline.
#define COUNT_BASELINE_FRAMES 10
//! Blobs above the baseline that mean the LEDs are visible, PSVR2Toolkit uses 3.
#define COUNT_THRESHOLD_MARGIN 3.0f
//! Edge search resolution, as in PSVR2Toolkit.
#define COUNT_EDGE_RESOLUTION_NS (U_TIME_1US_IN_NS * 25)
/*!
 * Edge confirmation distance. PSVR2Toolkit uses 50us, but on hardware the window drifted more than that between the
 * search and the confirmation (15-30us/s); the 1.5ms blink has ~500us of room in the ~2.5ms window anyway.
 */
#define COUNT_CONFIRM_NS (U_TIME_1US_IN_NS * 200)
//! Hold: how often to probe both window edges, and sooner when the controller isn't seen.
#define COUNT_HOLD_PROBE_NS (U_TIME_1S_IN_NS * 3)
#define COUNT_HOLD_PROBE_UNSEEN_NS (U_TIME_1S_IN_NS * 1)
//! Hold: how far inside each window edge the probes go, and the correction step.
#define COUNT_HOLD_PROBE_MARGIN_NS (U_TIME_1US_IN_NS * 100)
#define COUNT_HOLD_STEP_NS (U_TIME_1US_IN_NS * 100)
//! Hold: how often the drift estimate moves the centre.
#define COUNT_HOLD_APPLY_NS (U_TIME_1MS_IN_NS * 500)
//! Exposure to assume when the source doesn't report one.
#define COUNT_DEFAULT_EXPOSURE_NS (U_TIME_1US_IN_NS * 1000)

enum count_calib_state
{
	COUNT_CALIB_WAIT_TOKEN = 0,
	COUNT_CALIB_BASELINE,
	COUNT_CALIB_FIND_ON,
	COUNT_CALIB_LEFT_EDGE,
	COUNT_CALIB_RIGHT_EDGE,
	COUNT_CALIB_CONFIRM_LEFT_OUTER,
	COUNT_CALIB_CONFIRM_LEFT_INNER,
	COUNT_CALIB_CONFIRM_RIGHT_OUTER,
	COUNT_CALIB_CONFIRM_RIGHT_INNER,
};

//! The refinement currently calibrating, only one may at a time.
static _Atomic(struct t_led_sync_refinement *) count_calib_owner = NULL;
/*!
 * A holding refinement that needs to probe while another calibrates. The calibration yields between steps: on
 * hardware the second controller's calibration held the token for 40s+, the first's hold couldn't probe, and its lock
 * drifted out.
 */
static _Atomic(struct t_led_sync_refinement *) count_probe_wanted = NULL;

static bool
count_calib_take_token(struct t_led_sync_refinement *refinement)
{
	if (refinement->count_calib.owns_token) {
		return true;
	}
	struct t_led_sync_refinement *expected = NULL;
	if (atomic_compare_exchange_strong(&count_calib_owner, &expected, refinement)) {
		refinement->count_calib.owns_token = true;
	}
	return refinement->count_calib.owns_token;
}

static void
count_calib_release_token(struct t_led_sync_refinement *refinement)
{
	if (!refinement->count_calib.owns_token) {
		return;
	}
	struct t_led_sync_refinement *expected = refinement;
	atomic_compare_exchange_strong(&count_calib_owner, &expected, NULL);
	refinement->count_calib.owns_token = false;
}

//! The controller's cycle period carries the measured drift trim (see @ref count_calib.skew).
static void
count_calib_apply_trim(struct t_led_sync_refinement *refinement)
{
	refinement->delta_sync.base_period_ns = frame_period_ns(refinement);
	refinement->delta_sync.skew = refinement->count_calib.skew;
}

//! All LEDs off, for the baseline and while waiting for the token. A delta of 0, so the cycle keeps its place.
static void
count_calib_set_leds_off(struct t_led_sync_refinement *refinement)
{
	struct t_led_sync_sample_pssense_data data = {
	    .base_time_offset_ns = 0,
	    .leds = {0xFF, 0xFF, 0xFF, 0xFF},
	};
	if (refinement->count_calib.anchored) {
		count_calib_apply_trim(refinement);
	}
	set_offset_duration_pssense(refinement, 0, refinement->current_blink_fudge_ns,
	                            refinement->current_blink_duration_ns, T_LED_SYNC_SAMPLE_PSSENSE_MODE_LED_ALL_OFF,
	                            data);
}

/*!
 * All LEDs blinking every frame, @p offset_ns after the exposure start. The first call places the cycle with an
 * absolute position (PRESCAN, through the driver's clock tracking); every later one moves it by the difference from
 * where it is (STABLE, a relative delta), so it no longer depends on the clock tracking.
 */
static void
count_calib_set_offset(struct t_led_sync_refinement *refinement, time_duration_ns offset_ns)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	cc->offset_ns = offset_ns;
	cc->frames = 0;
	cc->sum = 0;
	cc->off_phase = false;

	count_calib_apply_trim(refinement);

	if (!cc->anchored) {
		cc->anchored = true;
		cc->applied_ns = offset_ns;
		set_offset_duration(refinement, 0, offset_ns, refinement->current_blink_duration_ns);
		return;
	}

	struct t_led_sync_sample_pssense_data data = {
	    .base_time_offset_ns = (int32_t)(offset_ns - cc->applied_ns),
	    .leds = {0xFF, 0xFF, 0xFF, 0xFF},
	};
	cc->applied_ns = offset_ns;
	set_offset_duration_pssense(refinement, 0, offset_ns, refinement->current_blink_duration_ns,
	                            T_LED_SYNC_SAMPLE_PSSENSE_MODE_STABLE, data);
}

/*!
 * The window moved @p moved_ns in @p dt_ns since the last measurement: fold that into the cycle period trim. The
 * position is in the anchored cycle's coordinates, so the remaining motion is what the trim hasn't cancelled yet.
 */
static void
count_calib_learn_drift(struct t_led_sync_refinement *refinement,
                        time_duration_ns moved_ns,
                        time_duration_ns dt_ns,
                        double gain,
                        const char *what)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	if (dt_ns < U_TIME_1S_IN_NS / 2) {
		return;
	}
	double rate = (double)moved_ns / (double)dt_ns; // fraction, ns per ns
	cc->skew += gain * rate;
	const double max_skew = 500e-6; // 500ppm
	cc->skew = CLAMP(cc->skew, -max_skew, max_skew);
	LOG_DEBUG(refinement, "Count drift from %s: moved %+" PRId64 "us in %.1fs (%+.1fppm), cycle trim now %+.1fppm",
	          what, (int64_t)(moved_ns / U_TIME_1US_IN_NS), (double)dt_ns / U_TIME_1S_IN_NS, rate * 1e6,
	          cc->skew * 1e6);
	count_calib_apply_trim(refinement);
}

static void
count_calib_set_offset(struct t_led_sync_refinement *refinement, time_duration_ns offset_ns);

static void
count_calib_restart(struct t_led_sync_refinement *refinement, const char *why);

/*!
 * A confirmation failed: the window was found moments ago, so search again just before it instead of sweeping the
 * whole frame and handing over the token. Falls back to a full restart after a few tries.
 */
static void
count_calib_research(struct t_led_sync_refinement *refinement, const char *why)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	if (++cc->local_retries > 3) {
		count_calib_restart(refinement, why);
		return;
	}
	LOG_DEBUG(refinement, "Count calibration searching again near the window (%s), retry %u", why,
	          cc->local_retries);
	time_duration_ns start_ns = cc->left_ns - cc->full_window_ns;
	if (start_ns < 0) {
		start_ns = 0;
	}
	cc->state = COUNT_CALIB_FIND_ON;
	cc->swept_ns = 0;
	count_calib_set_offset(refinement, start_ns);
}

static void
count_calib_restart(struct t_led_sync_refinement *refinement, const char *why)
{
	refinement->count_calib.local_retries = 0;
	refinement->count_calib.paused = false;
	LOG_DEBUG(refinement, "Count calibration restarting (%s), attempt %u", why, refinement->count_calib.attempts);
	refinement->count_calib.attempts++;
	// Give the other controller a turn, and place the cycle afresh next time.
	count_calib_release_token(refinement);
	refinement->count_calib.anchored = false;
	refinement->count_calib.have_prev_left = false;
	refinement->count_calib.state = COUNT_CALIB_WAIT_TOKEN;
	refinement->count_calib.frames = 0;
	refinement->count_calib.sum = 0;
	count_calib_set_leds_off(refinement);
}

//! Called when entering @ref T_LED_SYNC_PHASE_COUNT_CALIBRATE.
static void
count_calib_enter(struct t_led_sync_refinement *refinement)
{
	time_duration_ns exposure_ns =
	    refinement->has_exposure_time ? refinement->exposure_time_ns : COUNT_DEFAULT_EXPOSURE_NS;
	refinement->count_calib.full_window_ns = refinement->current_blink_duration_ns + exposure_ns;
	refinement->count_calib.state = COUNT_CALIB_WAIT_TOKEN;
	refinement->count_calib.paused = false;
	refinement->count_calib.anchored = false;
	refinement->count_calib.have_prev_left = false;
	refinement->count_calib.frames = 0;
	refinement->count_calib.sum = 0;
	count_calib_set_leds_off(refinement);
}

//! Called with every timing event, before the settle checks, so counts are always recorded.
static void
count_calib_push_count(struct t_led_sync_refinement *refinement,
                       const struct t_timing_event_camera_exposure_start *event)
{
	if (!event->has_blob_count) {
		return;
	}
	refinement->count_calib.has_count = true;
	refinement->count_calib.last_count = event->blob_count;
	refinement->count_calib.count_sequence_id = event->sequence_id;
}

/*!
 * Adds this frame's count, true once @p needed frames were added since the last LED change settled. Only called
 * from the tick, which runs once the change applied and settled.
 */
static bool
count_calib_accumulate(struct t_led_sync_refinement *refinement, uint32_t needed, float *out_mean)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;

	if (!cc->has_count || cc->count_sequence_id != refinement->last_exposure_sequence_id ||
	    cc->count_sequence_id == cc->last_counted_sequence_id) {
		return false;
	}
	cc->last_counted_sequence_id = cc->count_sequence_id;
	cc->sum += cc->last_count;
	cc->frames++;

	if (cc->frames < needed) {
		return false;
	}
	*out_mean = (float)cc->sum / (float)cc->frames;
	return true;
}

static void
count_calib_finish(struct t_led_sync_refinement *refinement)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;

	time_duration_ns centre_ns = cc->left_ns + (cc->right_ns - cc->left_ns) / 2;
	time_duration_ns window_ns = cc->right_ns - cc->left_ns;
	LOG_INFO(refinement,
	         "Count calibration done: visible from %" PRId64 "us to %" PRId64 "us (window %" PRId64
	         "us, blink %" PRId64 "us, so exposure ~%" PRId64 "us), holding at %" PRId64 "us",
	         (int64_t)(cc->left_ns / U_TIME_1US_IN_NS), (int64_t)(cc->right_ns / U_TIME_1US_IN_NS),
	         (int64_t)(window_ns / U_TIME_1US_IN_NS),
	         (int64_t)(refinement->current_blink_duration_ns / U_TIME_1US_IN_NS),
	         (int64_t)((window_ns - refinement->current_blink_duration_ns) / U_TIME_1US_IN_NS),
	         (int64_t)(centre_ns / U_TIME_1US_IN_NS));

	cc->local_retries = 0;
	cc->have_known_left = true;
	cc->known_left_ns = cc->left_ns;
	cc->hold_centre_ns = centre_ns;
	cc->hold_window_ns = window_ns;
	cc->hold_sub = 0;
	cc->hold_last_probe_ns = os_monotonic_get_ns();
	cc->hold_last_apply_ns = cc->hold_last_probe_ns;
	cc->hold_drift_ns_per_s = 0;
	cc->hold_corrections = 0;
	cc->hold_in_view = true;
	count_calib_set_offset(refinement, centre_ns);
	refinement->frames_since_last_visually_seen = 0;
	set_phase_locked(refinement, T_LED_SYNC_PHASE_COUNT_HOLD);
}

static void
handle_count_calibrate(struct t_led_sync_refinement *refinement)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	const time_duration_ns fw = cc->full_window_ns;

	if (cc->paused) {
		if (!count_calib_take_token(refinement)) {
			return;
		}
		cc->paused = false;
		// Measure this step again, the other controller's probe ran in between.
		count_calib_set_offset(refinement, cc->offset_ns);
		return;
	}
	struct t_led_sync_refinement *wanted = atomic_load(&count_probe_wanted);
	if (cc->owns_token && wanted != NULL && wanted != refinement && cc->state >= COUNT_CALIB_FIND_ON &&
	    cc->frames == 0 && !cc->off_phase) {
		LOG_DEBUG(refinement, "Count calibration pausing for the other controller's hold probe");
		count_calib_release_token(refinement);
		cc->paused = true;
		return;
	}

	if (cc->state == COUNT_CALIB_WAIT_TOKEN) {
		if (os_monotonic_get_ns() < cc->retry_after_ns || !count_calib_take_token(refinement)) {
			return;
		}
		// Start just before where the window was last time, it's usually found within a step or two.
		time_duration_ns start_ns = cc->have_known_left ? cc->known_left_ns - cc->full_window_ns : 0;
		LOG_DEBUG(refinement, "Count calibration starting, sweeping from %" PRId64 "us",
		          (int64_t)(start_ns / U_TIME_1US_IN_NS));
		cc->state = COUNT_CALIB_FIND_ON;
		cc->swept_ns = 0;
		count_calib_set_offset(refinement, start_ns);
		return;
	}

	float mean = 0;
	if (!count_calib_accumulate(refinement, COUNT_FRAMES_PER_STEP, &mean)) {
		return;
	}

	if (!cc->off_phase) {
		// Measured the LEDs at the test offset, now the same with them off.
		cc->on_mean = mean;
		cc->off_phase = true;
		cc->frames = 0;
		cc->sum = 0;
		count_calib_set_leds_off(refinement);
		return;
	}

	cc->baseline = mean;
	cc->threshold = mean + COUNT_THRESHOLD_MARGIN;
	bool on = cc->on_mean > cc->threshold;

	LOG_DEBUG(refinement,
	          "Count calibration state %d offset %" PRId64 "us: %.1f blobs with LEDs, %.1f without -> %s", cc->state,
	          (int64_t)(cc->offset_ns / U_TIME_1US_IN_NS), cc->on_mean, mean, on ? "on" : "off");

	switch (cc->state) {

	case COUNT_CALIB_FIND_ON:
		if (on) {
			cc->upper_ns = cc->offset_ns;
			cc->lower_ns = cc->offset_ns - fw;
			cc->state = COUNT_CALIB_LEFT_EDGE;
			count_calib_set_offset(refinement, (cc->lower_ns + cc->upper_ns) / 2);
		} else if ((cc->swept_ns += fw / 2) >= refinement->exposure_interval_ns) {
			// Out of view most likely: don't keep flashing across the headset's own exposures.
			cc->retry_after_ns = os_monotonic_get_ns() + U_TIME_1S_IN_NS * 3;
			count_calib_restart(refinement, "LEDs not seen at any offset, controller out of view?");
		} else {
			// Half window steps overlap, so a little clock drift during the sweep can't skip the window.
			count_calib_set_offset(refinement, cc->offset_ns + fw / 2);
		}
		break;

	case COUNT_CALIB_LEFT_EDGE:
		if (on) {
			cc->upper_ns = cc->offset_ns;
		} else {
			cc->lower_ns = cc->offset_ns;
		}
		if (cc->upper_ns - cc->lower_ns < COUNT_EDGE_RESOLUTION_NS) {
			cc->left_ns = cc->upper_ns;
			timepoint_ns now_left_ns = os_monotonic_get_ns();
			if (cc->have_prev_left && now_left_ns - cc->prev_left_time_ns < U_TIME_1S_IN_NS * 30) {
				count_calib_learn_drift(refinement, cc->left_ns - cc->prev_left_ns,
				                        now_left_ns - cc->prev_left_time_ns, 1.0, "re-search");
			}
			cc->have_prev_left = true;
			cc->prev_left_ns = cc->left_ns;
			cc->prev_left_time_ns = now_left_ns;
			cc->lower_ns = cc->left_ns + (fw * 9) / 10;
			cc->upper_ns = cc->left_ns + (fw * 11) / 10;
			cc->state = COUNT_CALIB_RIGHT_EDGE;
		}
		count_calib_set_offset(refinement, (cc->lower_ns + cc->upper_ns) / 2);
		break;

	case COUNT_CALIB_RIGHT_EDGE:
		if (on) {
			cc->lower_ns = cc->offset_ns;
		} else {
			cc->upper_ns = cc->offset_ns;
		}
		if (cc->upper_ns - cc->lower_ns < COUNT_EDGE_RESOLUTION_NS) {
			cc->right_ns = cc->lower_ns;
			cc->state = COUNT_CALIB_CONFIRM_LEFT_OUTER;
			count_calib_set_offset(refinement, cc->left_ns - COUNT_CONFIRM_NS);
		} else {
			count_calib_set_offset(refinement, (cc->lower_ns + cc->upper_ns) / 2);
		}
		break;

	case COUNT_CALIB_CONFIRM_LEFT_OUTER:
		if (on) {
			count_calib_research(refinement, "left edge confirmation: on just before it");
			break;
		}
		cc->state = COUNT_CALIB_CONFIRM_LEFT_INNER;
		count_calib_set_offset(refinement, cc->left_ns + COUNT_CONFIRM_NS);
		break;

	case COUNT_CALIB_CONFIRM_LEFT_INNER:
		if (!on) {
			count_calib_research(refinement, "left edge confirmation: off just after it");
			break;
		}
		cc->state = COUNT_CALIB_CONFIRM_RIGHT_OUTER;
		count_calib_set_offset(refinement, cc->right_ns + COUNT_CONFIRM_NS);
		break;

	case COUNT_CALIB_CONFIRM_RIGHT_OUTER:
		if (on) {
			count_calib_research(refinement, "right edge confirmation: on just after it");
			break;
		}
		cc->state = COUNT_CALIB_CONFIRM_RIGHT_INNER;
		count_calib_set_offset(refinement, cc->right_ns - COUNT_CONFIRM_NS);
		break;

	case COUNT_CALIB_CONFIRM_RIGHT_INNER:
		if (!on) {
			count_calib_research(refinement, "right edge confirmation: off just before it");
			break;
		}
		count_calib_finish(refinement);
		break;

	default: count_calib_restart(refinement, "bad state"); break;
	}
}

/*!
 * Keeps the blink centred in the window. Nothing measured the drift before (15-30us/s on hardware, from the
 * controller clock estimate's rate error), so the lock would slide out within a minute. Every few seconds the
 * blink's count is measured at the centre and then just inside each window edge for a few frames; if one side drops
 * well below the centre the centre is moved away from it, and the corrections build a slow drift rate that moves the
 * centre between probes while the controller is in view.
 *
 * Probing takes the calibration token, so it never overlaps the other controller calibrating or probing, and sides are
 * judged against the centre count rather than the calibration's threshold: the other controller's lit LEDs are in
 * every count (on hardware that made sides read as on, or an unrelated toggle read as drift, and a drift rate built
 * from those, -65us/s against a real ~10us/s, walked the lock out of the window).
 */
static void
handle_count_hold(struct t_led_sync_refinement *refinement)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	timepoint_ns now_ns = os_monotonic_get_ns();

	time_duration_ns probe_ns = (cc->hold_window_ns - refinement->current_blink_duration_ns) / 2 -
	                            COUNT_HOLD_PROBE_MARGIN_NS;
	if (probe_ns < COUNT_HOLD_PROBE_MARGIN_NS) {
		probe_ns = COUNT_HOLD_PROBE_MARGIN_NS;
	}

	if (cc->hold_sub == 0) {
		// The drift is trimmed out of the free-running cycle's period, the centre only moves on corrections.
		bool unseen = refinement->frames_since_last_visually_seen > 60;
		time_duration_ns since_probe_ns = now_ns - cc->hold_last_probe_ns;
		if (since_probe_ns >= COUNT_HOLD_PROBE_NS || (unseen && since_probe_ns >= COUNT_HOLD_PROBE_UNSEEN_NS)) {
			if (!count_calib_take_token(refinement)) {
				// The other controller is calibrating or probing, ask it to yield between steps.
				atomic_store(&count_probe_wanted, refinement);
				return;
			}
			struct t_led_sync_refinement *self = refinement;
			atomic_compare_exchange_strong(&count_probe_wanted, &self, NULL);
			// First the count at the centre, where the blink already is.
			cc->hold_sub = 1;
			cc->frames = 0;
			cc->sum = 0;
		}
		return;
	}

	float mean = 0;
	if (!count_calib_accumulate(refinement, COUNT_FRAMES_PER_STEP, &mean)) {
		return;
	}

	if (cc->hold_sub == 1) {
		cc->hold_centre_mean = mean;
		cc->hold_sub = 2;
		count_calib_set_offset(refinement, cc->hold_centre_ns - probe_ns);
		return;
	}
	if (cc->hold_sub == 2) {
		cc->hold_left_mean = mean;
		cc->hold_sub = 3;
		count_calib_set_offset(refinement, cc->hold_centre_ns + probe_ns);
		return;
	}

	const float c = cc->hold_centre_mean;
	bool in_view = c > cc->threshold;
	// A side whose count drops below 3/4 of the centre's has an exposure edge in it.
	bool left_on = cc->hold_left_mean >= 0.75f * c;
	bool right_on = mean >= 0.75f * c;
	double dt_s = (double)(now_ns - cc->hold_last_probe_ns) / U_TIME_1S_IN_NS;
	int e = 0;
	if (in_view && !left_on && right_on) {
		e = 1; // The window moved later.
	} else if (in_view && left_on && !right_on) {
		e = -1; // The window moved earlier.
	}
	if (e != 0) {
		cc->hold_centre_ns += e * COUNT_HOLD_STEP_NS;
		cc->hold_corrections++;
		if (dt_s > 1.0) {
			// Each correction is drift the trim didn't cancel; fold part of it in.
			count_calib_learn_drift(refinement, e * COUNT_HOLD_STEP_NS,
			                        (time_duration_ns)(dt_s * U_TIME_1S_IN_NS), 0.5, "hold probe");
		}
		cc->hold_drift_ns_per_s = cc->skew * 1e9;
	}
	cc->hold_in_view = in_view;
	LOG_DEBUG(refinement,
	          "Count hold probe: centre %.1f left %.1f right %.1f blobs -> %s, centre %" PRId64
	          "us, cycle trim %+.1fppm, %u corrections",
	          c, cc->hold_left_mean, mean,
	          !in_view ? "not in view"
	                   : (e > 0 ? "moved later" : (e < 0 ? "moved earlier" : "centred")),
	          (int64_t)(cc->hold_centre_ns / U_TIME_1US_IN_NS), cc->skew * 1e6,
	          cc->hold_corrections);

	count_calib_release_token(refinement);
	cc->hold_last_probe_ns = now_ns;
	cc->hold_last_apply_ns = now_ns;
	cc->hold_sub = 0;
	count_calib_set_offset(refinement, cc->hold_centre_ns);
}
