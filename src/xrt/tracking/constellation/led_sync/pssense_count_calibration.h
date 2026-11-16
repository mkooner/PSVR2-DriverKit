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
//! Longest probe interval once the blink stays centred.
#define COUNT_HOLD_PROBE_MAX_NS (U_TIME_1S_IN_NS * 30)
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
	//! Visibility check with the LEDs lit regardless of timing, after sweeps that saw nothing.
	COUNT_CALIB_DIAG_ON,
};

//! Give up confirming and use the best window found after this long, the hold's probes centre it from there.
#define COUNT_CALIB_TIME_LIMIT_NS (U_TIME_1S_IN_NS * 20)
//! Empty sweeps in a row before the LEDs-always-on visibility check.
#define COUNT_CALIB_EMPTY_SWEEPS_DIAG 2

/*!
 * Visibility check before sweeping: one reading with the LEDs always on (visible at any timing) against off. On
 * hardware the first controller's first sweep found nothing in both 01:45 and 01:47 starts (15 steps, 2.7s, then a 3s
 * wait), with the controller not yet in view. While the check sees nothing it is repeated every
 * COUNT_CALIB_PRECHECK_RETRY_NS instead of sweeping blind. The always-on reading hasn't been seen succeeding on
 * hardware yet, so after COUNT_CALIB_PRECHECK_MAX_FAILS misses it sweeps anyway, and if that sweep finds the LEDs the
 * check is skipped for the rest of the session: a broken check costs one calibration ~2s once.
 */
#define COUNT_CALIB_PRECHECK_RETRY_NS (U_TIME_1MS_IN_NS * 500)
#define COUNT_CALIB_PRECHECK_MAX_FAILS 3

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
	count_calib_apply_trim(refinement);
	set_offset_duration_pssense(refinement, 0, refinement->current_blink_fudge_ns,
	                            refinement->current_blink_duration_ns, T_LED_SYNC_SAMPLE_PSSENSE_MODE_LED_ALL_OFF,
	                            data);
	// Keep the driver's mapping advancing at the measured drift while the LEDs are off.
	refinement->sample_for_driver.clock_skew = refinement->count_calib.skew;
}

/*!
 * All LEDs blinking every frame, @p offset_ns after the exposure start, as an absolute (PRESCAN) position. The driver
 * maps it with a host -> controller offset that is latched once and only advanced by the drift rate measured here
 * (clock_skew), so the Bluetooth clock estimate doesn't move the LEDs. (Placing once and moving by relative deltas in
 * STABLE mode was tried: the cameras then never saw the LEDs.)
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
	set_offset_duration(refinement, 0, offset_ns, refinement->current_blink_duration_ns);
	refinement->sample_for_driver.clock_skew = cc->skew;
}


static void
count_calib_set_offset(struct t_led_sync_refinement *refinement, time_duration_ns offset_ns);

static void
count_calib_restart(struct t_led_sync_refinement *refinement, const char *why);

static void
count_calib_finish(struct t_led_sync_refinement *refinement);

/*!
 * A confirmation failed: the window was found moments ago, so search again just before it instead of sweeping the
 * whole frame and handing over the token. Falls back to a full restart after a few tries.
 */
static void
count_calib_research(struct t_led_sync_refinement *refinement, const char *why)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	/*
	 * The edges were found and the window is as wide as it should be: rather than chase the confirmations while it
	 * drifts, accept it after one retry. The hold's probes correct the centre within seconds.
	 */
	if (cc->pass_plausible && cc->local_retries >= 1) {
		LOG_INFO(refinement, "Count calibration accepting the window without confirmation (%s)", why);
		count_calib_finish(refinement);
		return;
	}
	if (++cc->local_retries > 3) {
		count_calib_restart(refinement, why);
		return;
	}
	cc->pass_plausible = false;
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
	refinement->count_calib.started_ns = 0;
	refinement->count_calib.have_plausible = false;
	refinement->count_calib.pass_plausible = false;
	refinement->count_calib.empty_sweeps = 0;
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
	cc->quick = false;
	cc->started_ns = 0;
	cc->have_plausible = false;
	cc->pass_plausible = false;
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
	cc->hold_start_ns = cc->hold_last_probe_ns;
	cc->hold_trim_last_ns = cc->hold_last_probe_ns;
	cc->hold_corrections_sum_ns = 0;
	cc->hold_trim_integral_ns = 0;
	cc->hold_unseen_probes = 0;
	cc->hold_interval_ns = COUNT_HOLD_PROBE_NS;
	cc->hold_centred_streak = 0;
	count_calib_set_offset(refinement, centre_ns);
	refinement->frames_since_last_visually_seen = 0;
	set_phase_locked(refinement, T_LED_SYNC_PHASE_COUNT_HOLD);
}

static void
count_calib_start_sweep(struct t_led_sync_refinement *refinement)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	// Start just before where the window was last time, it's usually found within a step or two.
	time_duration_ns start_ns = cc->have_known_left ? cc->known_left_ns - cc->full_window_ns : 0;
	LOG_DEBUG(refinement, "Count calibration starting, sweeping from %" PRId64 "us",
	          (int64_t)(start_ns / U_TIME_1US_IN_NS));
	cc->state = COUNT_CALIB_FIND_ON;
	cc->swept_ns = 0;
	cc->pass_plausible = false;
	count_calib_set_offset(refinement, start_ns);
}

//! One reading with the LEDs always on, then off, see COUNT_CALIB_PRECHECK_RETRY_NS.
static void
count_calib_start_diag_on(struct t_led_sync_refinement *refinement, bool precheck)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	cc->precheck = precheck;
	cc->state = COUNT_CALIB_DIAG_ON;
	cc->frames = 0;
	cc->sum = 0;
	cc->off_phase = false;
	struct t_led_sync_sample_pssense_data data = {.leds = {0xFF, 0xFF, 0xFF, 0xFF}};
	set_offset_duration_pssense(refinement, 0, cc->offset_ns, refinement->current_blink_duration_ns,
	                            T_LED_SYNC_SAMPLE_PSSENSE_MODE_LED_ALL_ON, data);
}

static void
count_calib_start_precheck(struct t_led_sync_refinement *refinement)
{
	LOG_DEBUG(refinement, "Count calibration checking the LEDs are in view before sweeping");
	count_calib_start_diag_on(refinement, true);
}

/*!
 * A confirmation step read wrong. One reading can be a frame where a hand or the controller itself hid the LEDs
 * (01:47: the window 4680-6800us was found, then one confirmation read 0 blobs and ~30 more steps followed), so read
 * it once more, one step, before searching again. A real timing error reads wrong again.
 */
static void
count_calib_confirm_failed(struct t_led_sync_refinement *refinement, const char *why)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	if (!cc->confirm_reread) {
		cc->confirm_reread = true;
		LOG_DEBUG(refinement, "Count calibration reading the confirmation again (%s)", why);
		count_calib_set_offset(refinement, cc->offset_ns);
		return;
	}
	cc->confirm_reread = false;
	count_calib_research(refinement, why);
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
	    cc->state != COUNT_CALIB_DIAG_ON && cc->frames == 0 && !cc->off_phase) {
		LOG_DEBUG(refinement, "Count calibration pausing for the other controller's hold probe");
		count_calib_release_token(refinement);
		cc->paused = true;
		return;
	}

	if (cc->state == COUNT_CALIB_WAIT_TOKEN) {
		if (os_monotonic_get_ns() < cc->retry_after_ns || !count_calib_take_token(refinement)) {
			return;
		}
		if (cc->started_ns == 0) {
			cc->started_ns = os_monotonic_get_ns();
		}
		cc->confirm_reread = false;
		if (!cc->precheck_untrusted && cc->precheck_fails < COUNT_CALIB_PRECHECK_MAX_FAILS) {
			count_calib_start_precheck(refinement);
		} else {
			cc->sweep_despite_precheck = !cc->precheck_untrusted;
			count_calib_start_sweep(refinement);
		}
		return;
	}

	// Time limit: finish with the best window seen rather than keep the controller untracked.
	if (cc->have_plausible && os_monotonic_get_ns() - cc->started_ns > COUNT_CALIB_TIME_LIMIT_NS &&
	    cc->frames == 0 && !cc->off_phase) {
		LOG_INFO(refinement, "Count calibration time limit reached, using the best window found");
		cc->left_ns = cc->plausible_left_ns;
		cc->right_ns = cc->plausible_right_ns;
		count_calib_finish(refinement);
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
			cc->empty_sweeps = 0;
			cc->precheck_fails = 0;
			if (cc->sweep_despite_precheck) {
				cc->sweep_despite_precheck = false;
				cc->precheck_untrusted = true;
				LOG_INFO(refinement, "Count calibration: the sweep found LEDs the always-on check missed, "
				                     "skipping the check from now on");
			}
			cc->upper_ns = cc->offset_ns;
			cc->lower_ns = cc->offset_ns - fw;
			cc->state = COUNT_CALIB_LEFT_EDGE;
			count_calib_set_offset(refinement, (cc->lower_ns + cc->upper_ns) / 2);
		} else if ((cc->swept_ns += fw / 2) >= refinement->exposure_interval_ns) {
			if (++cc->empty_sweeps >= COUNT_CALIB_EMPTY_SWEEPS_DIAG) {
				// Are the LEDs visible at all? Light them regardless of timing and compare with off.
				cc->empty_sweeps = 0;
				count_calib_start_diag_on(refinement, false);
				break;
			}
			// Out of view most likely: don't keep flashing across the headset's own exposures.
			// The checks that saw nothing were right, so check again next time instead of sweeping blind.
			cc->sweep_despite_precheck = false;
			cc->precheck_fails = 0;
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
			/*
			 * No drift is learned here: two searches can land a whole sweep step (blink + exposure) apart, which
			 * on hardware read as +2340us in 3.5s (662ppm, against a real few ppm) and pushed the trim to its
			 * limit, walking that controller's LEDs out of the window. Drift comes only from the hold.
			 */
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
			time_duration_ns width_ns = cc->right_ns - cc->left_ns;
			cc->pass_plausible = width_ns > (fw * 7) / 10 && width_ns < (fw * 3) / 2;
			if (cc->pass_plausible) {
				cc->have_plausible = true;
				cc->plausible_left_ns = cc->left_ns;
				cc->plausible_right_ns = cc->right_ns;
			}
			if (cc->quick && cc->pass_plausible) {
				/*
				 * Recalibrating after a lost hold: the hold's edge probes centre the window within seconds, so
				 * skip the four confirmation steps (and the re-searches a failed one starts).
				 */
				LOG_INFO(refinement, "Count calibration after a lost hold: window plausible, skipping confirmation");
				count_calib_finish(refinement);
				break;
			}
			cc->state = COUNT_CALIB_CONFIRM_LEFT_OUTER;
			count_calib_set_offset(refinement, cc->left_ns - COUNT_CONFIRM_NS);
		} else {
			count_calib_set_offset(refinement, (cc->lower_ns + cc->upper_ns) / 2);
		}
		break;

	case COUNT_CALIB_CONFIRM_LEFT_OUTER:
		if (on) {
			count_calib_confirm_failed(refinement, "left edge confirmation: on just before it");
			break;
		}
		cc->confirm_reread = false;
		cc->state = COUNT_CALIB_CONFIRM_LEFT_INNER;
		count_calib_set_offset(refinement, cc->left_ns + COUNT_CONFIRM_NS);
		break;

	case COUNT_CALIB_CONFIRM_LEFT_INNER:
		if (!on) {
			count_calib_confirm_failed(refinement, "left edge confirmation: off just after it");
			break;
		}
		cc->confirm_reread = false;
		cc->state = COUNT_CALIB_CONFIRM_RIGHT_OUTER;
		count_calib_set_offset(refinement, cc->right_ns + COUNT_CONFIRM_NS);
		break;

	case COUNT_CALIB_CONFIRM_RIGHT_OUTER:
		if (on) {
			count_calib_confirm_failed(refinement, "right edge confirmation: on just after it");
			break;
		}
		cc->confirm_reread = false;
		cc->state = COUNT_CALIB_CONFIRM_RIGHT_INNER;
		count_calib_set_offset(refinement, cc->right_ns - COUNT_CONFIRM_NS);
		break;

	case COUNT_CALIB_CONFIRM_RIGHT_INNER:
		if (!on) {
			count_calib_confirm_failed(refinement, "right edge confirmation: off just before it");
			break;
		}
		cc->confirm_reread = false;
		count_calib_finish(refinement);
		break;

	case COUNT_CALIB_DIAG_ON:
		if (cc->precheck) {
			cc->precheck = false;
			if (on) {
				LOG_DEBUG(refinement, "Count calibration: LEDs in view (%.1f vs %.1f blobs always on), sweeping",
				          cc->on_mean, cc->baseline);
				cc->precheck_fails = 0;
				count_calib_start_sweep(refinement);
			} else {
				cc->precheck_fails++;
				LOG_DEBUG(refinement,
				          "Count calibration: LEDs not in view (%.1f vs %.1f blobs always on), check %u, "
				          "checking again in %ums",
				          cc->on_mean, cc->baseline, cc->precheck_fails,
				          (unsigned)(COUNT_CALIB_PRECHECK_RETRY_NS / U_TIME_1MS_IN_NS));
				// Let the other controller take its turn meanwhile; nothing measured to keep.
				count_calib_release_token(refinement);
				cc->state = COUNT_CALIB_WAIT_TOKEN;
				cc->retry_after_ns = os_monotonic_get_ns() + COUNT_CALIB_PRECHECK_RETRY_NS;
				count_calib_set_leds_off(refinement);
			}
			break;
		}
		if (on) {
			// Visible, so the timing is what's wrong: re-latch the driver's mapping and sweep again.
			LOG_INFO(refinement,
			         "Count calibration: LEDs visible when always on (%.1f vs %.1f blobs) but not at any "
			         "timing, re-latching the clock mapping",
			         cc->on_mean, cc->baseline);
			cc->have_known_left = false;
			count_calib_restart(refinement, "re-latching the clock mapping");
			// Windows found so far were in the old mapping.
			cc->have_plausible = false;
			cc->started_ns = 0;
			refinement->sample_for_driver.reset_clock_map = true;
		} else {
			LOG_INFO(refinement,
			         "Count calibration: LEDs not visible even always on (%.1f vs %.1f blobs), controller out "
			         "of the cameras' view or covered",
			         cc->on_mean, cc->baseline);
			cc->retry_after_ns = os_monotonic_get_ns() + U_TIME_1S_IN_NS * 3;
			count_calib_restart(refinement, "controller not visible");
		}
		break;

	default: count_calib_restart(refinement, "bad state"); break;
	}
}

/*!
 * Keeps the blink centred in the window, measuring only this controller's LEDs.
 *
 * Geometry, blink B (1500us) and exposure E (~1060us measured): with o the blink start relative to the exposure start,
 * the LEDs show for o in (-B, E), the window W = B + E (2559us measured), and they're fully exposed only for o within
 * +-(B - E)/2 (~220us) of the centre. A probe tests the visibility at centre +- (W/2 - margin), just inside each
 * measured edge: while the centre is right within the margin both sides are visible, and once it's off by more, the
 * side it moved away from goes dark. (Probing at centre +- ((W - B)/2 - 100us), as before, sat where ~80% of the
 * exposure is still lit, so it only noticed ~0.5-0.85ms of error, close to losing the LEDs at W/2 ~1280us.)
 *
 * Each probe reads the count at the centre, at both sides, and with this controller's LEDs off, and judges each by its
 * difference from the off reading, the same "more than 3 blobs" the calibration's edges were found with. The probe holds
 * the calibration token, so the other controller's LEDs don't change meanwhile and cancel out with lamps and
 * reflections. (Comparing raw counts let the other controller's lit LEDs make a controller whose LEDs had drifted out
 * look centred.)
 *
 * Drift: if the window moves at rate r against the untrimmed mapping, the phase applied over the hold, the sum of the
 * corrections plus the integral of the trim, is P(t) ~= r t, so r = P / T with an error under step / T. It is only
 * used once T >= 10s (error <= 10ppm) and only if |r| <= 50ppm: the controller clocks measure -3 to +47ppm against the
 * host, so anything beyond is a misreading and the trim is left alone.
 */
#define COUNT_HOLD_EDGE_MARGIN_NS (U_TIME_1US_IN_NS * 150)
#define COUNT_HOLD_DRIFT_MIN_T_NS (U_TIME_1S_IN_NS * 10)
#define COUNT_HOLD_DRIFT_MAX (50e-6)
//! Probes in a row with this controller's LEDs not seen at the centre before calibrating again.
#define COUNT_HOLD_UNSEEN_PROBES 3

static void
count_hold_integrate_trim(struct t_led_sync_refinement *refinement, timepoint_ns now_ns)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	cc->hold_trim_integral_ns += cc->skew * (double)(now_ns - cc->hold_trim_last_ns);
	cc->hold_trim_last_ns = now_ns;
}

//! The result of a hold probe, whichever kind: correction, drift estimate, probe interval, loss.
static void
count_hold_apply(struct t_led_sync_refinement *refinement,
                 timepoint_ns now_ns,
                 bool in_view,
                 int e,
                 const char *verdict,
                 const char *detail)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;

	count_hold_integrate_trim(refinement, now_ns);
	if (e != 0) {
		cc->hold_centre_ns += e * COUNT_HOLD_STEP_NS;
		cc->hold_corrections_sum_ns += e * (double)COUNT_HOLD_STEP_NS;
		cc->hold_corrections++;
	}

	time_duration_ns t_ns = now_ns - cc->hold_start_ns;
	if (in_view && t_ns >= COUNT_HOLD_DRIFT_MIN_T_NS) {
		double rate = (cc->hold_corrections_sum_ns + cc->hold_trim_integral_ns) / (double)t_ns;
		if (rate >= -COUNT_HOLD_DRIFT_MAX && rate <= COUNT_HOLD_DRIFT_MAX) {
			cc->skew = rate;
		} else {
			LOG_DEBUG(refinement, "Count hold drift estimate %+.1fppm is implausible, trim left at %+.1fppm",
			          rate * 1e6, cc->skew * 1e6);
		}
	}

	if (in_view) {
		cc->hold_unseen_probes = 0;
	} else {
		cc->hold_unseen_probes++;
	}

	/*
	 * Each probe dims and briefly turns off this controller's LEDs, and on hardware (01:28 run) 12 of the 22
	 * outages over 0.3s had a probe in them: the tracker needs 0.4-1.2s to find the controller again. While the
	 * probes keep finding the blink centred, double the interval up to 30s; any correction or loss goes back to 3s.
	 * Simulated with the measured numbers (analysis/hold_sim.py): 19.8 -> 3.8-6.2 probes/min, never lost at +-47ppm
	 * drift or after a sudden 300/600us shift, max error 255-457us against the 1280us loss point.
	 */
	if (in_view && e == 0) {
		if (++cc->hold_centred_streak >= 3) {
			cc->hold_interval_ns = MIN(cc->hold_interval_ns * 2, COUNT_HOLD_PROBE_MAX_NS);
		}
	} else {
		cc->hold_centred_streak = 0;
		cc->hold_interval_ns = COUNT_HOLD_PROBE_NS;
	}
	cc->hold_in_view = in_view;

	LOG_DEBUG(refinement,
	          "Count hold probe: %s -> %s, centre %" PRId64
	          "us, cycle trim %+.1fppm, %u corrections in %.0fs, next in %.0fs",
	          detail, verdict, (int64_t)(cc->hold_centre_ns / U_TIME_1US_IN_NS), cc->skew * 1e6,
	          cc->hold_corrections, (double)t_ns / U_TIME_1S_IN_NS,
	          (double)cc->hold_interval_ns / U_TIME_1S_IN_NS);

	count_calib_release_token(refinement);
	cc->hold_last_probe_ns = now_ns;
	cc->hold_last_apply_ns = now_ns;
	cc->hold_sub = 0;

	if (cc->hold_unseen_probes >= COUNT_HOLD_UNSEEN_PROBES) {
		// This controller's own LEDs gone at the centre for several probes: find the window again.
		LOG_INFO(refinement, "Count hold lost the LEDs for %u probes, calibrating again", cc->hold_unseen_probes);
		set_phase_locked(refinement, T_LED_SYNC_PHASE_FIND_INITIAL_OFFSET);
		cc->quick = true;
		return;
	}
	count_calib_set_offset(refinement, cc->hold_centre_ns);
}

static void
handle_count_hold_edges(struct t_led_sync_refinement *refinement)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	timepoint_ns now_ns = os_monotonic_get_ns();

	time_duration_ns probe_ns = cc->hold_window_ns / 2 - COUNT_HOLD_EDGE_MARGIN_NS;
	if (probe_ns < COUNT_HOLD_EDGE_MARGIN_NS) {
		probe_ns = COUNT_HOLD_EDGE_MARGIN_NS;
	}

	if (cc->hold_sub == 0) {
		bool unseen = refinement->frames_since_last_visually_seen > 60;
		time_duration_ns since_probe_ns = now_ns - cc->hold_last_probe_ns;
		if (since_probe_ns >= cc->hold_interval_ns || (unseen && since_probe_ns >= COUNT_HOLD_PROBE_UNSEEN_NS)) {
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

	switch (cc->hold_sub) {
	case 1:
		cc->hold_centre_mean = mean;
		cc->hold_sub = 2;
		count_calib_set_offset(refinement, cc->hold_centre_ns - probe_ns);
		return;
	case 2:
		cc->hold_left_mean = mean;
		cc->hold_sub = 3;
		count_calib_set_offset(refinement, cc->hold_centre_ns + probe_ns);
		return;
	case 3:
		cc->hold_right_mean_tmp = mean;
		cc->hold_sub = 4;
		count_calib_set_leds_off(refinement);
		cc->frames = 0;
		cc->sum = 0;
		return;
	default: break;
	}

	// Sub 4: this controller's LEDs off.
	cc->hold_off_mean = mean;
	const float off = mean;
	const float c = cc->hold_centre_mean - off;
	const float l = cc->hold_left_mean - off;
	const float r = cc->hold_right_mean_tmp - off;
	const bool in_view = c > COUNT_THRESHOLD_MARGIN;
	const bool left_on = l > COUNT_THRESHOLD_MARGIN;
	const bool right_on = r > COUNT_THRESHOLD_MARGIN;

	int e = 0;
	const char *verdict = "not in view";
	if (in_view) {
		if (left_on && right_on) {
			verdict = "centred";
		} else if (!left_on && right_on) {
			e = 1; // The left edge passed the left probe: the window moved later.
			verdict = "moved later";
		} else if (left_on && !right_on) {
			e = -1; // The window moved earlier.
			verdict = "moved earlier";
		} else {
			verdict = "both edges dark"; // Narrower than measured (partly covered?), no correction.
		}
	}

	char detail[128];
	snprintf(detail, sizeof(detail), "centre %+.1f left %+.1f right %+.1f blobs over %.1f off", c, l, r, off);
	count_hold_apply(refinement, now_ns, in_view, e, verdict, detail);
}

/*!
 * Nudge check: move the blink +-COUNT_HOLD_NUDGE_NS around the centre, one latch each, and compare the brightness of
 * this controller's matched LEDs (from the tracker's samples) at the two sides. While centred, the exposure stays
 * fully or mostly lit (83% at +-300us), so the tracker keeps the controller; once the blink sits off centre, the side
 * it moved toward dims. Uses only this device's LEDs, so no LEDs-off reference and no token.
 *
 * Simulated (analysis/dither_sim.py: brightness with 15% per-frame noise and slow +-30% changes from motion, +-47ppm
 * drift, +-400us starting offsets, 120 runs): never lost, worst error 666us of the 1280us loss point; checks leaving the
 * LEDs < 50% exposed 1.1% (the edge check: all of them, plus the LEDs-off frames); 7.7% of corrections in the wrong
 * direction, one 100us step, undone at the next check.
 */
#define COUNT_HOLD_NUDGE_NS (U_TIME_1US_IN_NS * 300)
//! One side is dimmer if under this fraction of the other.
#define COUNT_HOLD_NUDGE_RATIO 0.80f
//! Samples per side, or how long to collect them (about one latch).
#define COUNT_HOLD_NUDGE_SAMPLES 6
#define COUNT_HOLD_NUDGE_WINDOW_NS (U_TIME_1MS_IN_NS * 250)

static void
count_hold_push_sample(struct t_led_sync_refinement *refinement, const struct t_constellation_tracker_sample *sample)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	if (!cc->hold_measuring || sample->sequence_id < cc->hold_meas_from_seq) {
		return;
	}
	cc->hold_bright_sum += sample->average_brightness;
	cc->hold_bright_n++;
}

static void
handle_count_hold_nudge(struct t_led_sync_refinement *refinement)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	timepoint_ns now_ns = os_monotonic_get_ns();

	if (cc->hold_sub == 0) {
		bool unseen = refinement->frames_since_last_visually_seen > 60;
		time_duration_ns since_probe_ns = now_ns - cc->hold_last_probe_ns;
		if (since_probe_ns >= cc->hold_interval_ns || (unseen && since_probe_ns >= COUNT_HOLD_PROBE_UNSEEN_NS)) {
			cc->hold_sub = 1;
			cc->hold_measuring = false;
			count_calib_set_offset(refinement, cc->hold_centre_ns - COUNT_HOLD_NUDGE_NS);
		}
		return;
	}

	// The tick runs once the nudge applied and settled: start collecting from this frame.
	if (!cc->hold_measuring) {
		cc->hold_measuring = true;
		cc->hold_meas_from_seq = refinement->last_exposure_sequence_id;
		cc->hold_meas_start_ns = now_ns;
		cc->hold_bright_sum = 0;
		cc->hold_bright_n = 0;
		return;
	}
	if (cc->hold_bright_n < COUNT_HOLD_NUDGE_SAMPLES && now_ns - cc->hold_meas_start_ns < COUNT_HOLD_NUDGE_WINDOW_NS) {
		return;
	}
	float mean = cc->hold_bright_n > 0 ? (float)(cc->hold_bright_sum / cc->hold_bright_n) : 0.0f;
	uint32_t n = cc->hold_bright_n;
	cc->hold_measuring = false;

	if (cc->hold_sub == 1) {
		cc->hold_minus_mean = mean;
		cc->hold_minus_n = n;
		cc->hold_sub = 2;
		count_calib_set_offset(refinement, cc->hold_centre_ns + COUNT_HOLD_NUDGE_NS);
		return;
	}

	const float bm = cc->hold_minus_n >= 2 ? cc->hold_minus_mean : 0.0f;
	const float bp = n >= 2 ? mean : 0.0f;
	const bool in_view = bm > 0.0f || bp > 0.0f;
	int e = 0;
	const char *verdict = "not in view";
	if (in_view) {
		verdict = "centred";
		if (bm < COUNT_HOLD_NUDGE_RATIO * bp) {
			e = 1; // The early side is dimmer: the blink sits early, move it later.
			verdict = "moved later";
		} else if (bp < COUNT_HOLD_NUDGE_RATIO * bm) {
			e = -1;
			verdict = "moved earlier";
		}
	}
	char detail[128];
	snprintf(detail, sizeof(detail), "brightness -%dus %.3f (%u) +%dus %.3f (%u)",
	         (int)(COUNT_HOLD_NUDGE_NS / U_TIME_1US_IN_NS), bm, cc->hold_minus_n,
	         (int)(COUNT_HOLD_NUDGE_NS / U_TIME_1US_IN_NS), bp, n);
	count_hold_apply(refinement, now_ns, in_view, e, verdict, detail);
}

DEBUG_GET_ONCE_OPTION(pssense_hold_check, "PSSENSE_HOLD_CHECK", "nudge")

//! PSSENSE_HOLD_CHECK=edges selects the previous edge-and-LEDs-off check.
static void
handle_count_hold(struct t_led_sync_refinement *refinement)
{
	const char *mode = debug_get_option_pssense_hold_check();
	if (mode != NULL && strcmp(mode, "edges") == 0) {
		handle_count_hold_edges(refinement);
	} else {
		handle_count_hold_nudge(refinement);
	}
}
