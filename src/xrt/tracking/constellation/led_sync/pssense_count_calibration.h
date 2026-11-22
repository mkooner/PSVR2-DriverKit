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
#ifndef COUNT_FRAMES_PER_STEP
#define COUNT_FRAMES_PER_STEP 2
#endif
//! Frames of blob counts for the LEDs-off baseline.
#define COUNT_BASELINE_FRAMES 10
//! Blobs above the baseline that mean the LEDs are visible, PSVR2Toolkit uses 3. The least threshold, see below.
#define COUNT_THRESHOLD_MARGIN 3.0f
/*!
 * Noise-aware decisions. A step reads the LEDs on, then off, and the difference d of the two means is the LEDs plus
 * noise. In a calm room the noise is under a blob and d is either ~0 or >= 10 (11:45: 61% within 3, 35% >= 10, 4%
 * between); in a bright one the background changes from frame to frame (12:59: sd 5.9 blobs, 44% of steps between 3
 * and 10, 8% at <= -10, which LEDs can't cause), and a fixed 3 blob threshold made those steps coin flips: wrong edges,
 * wrong windows accepted, holds lost.
 *
 * The noise is measured from what can't contain the LEDs: the spread between frames of one phase (the LEDs don't
 * change within it) and negative differences. "On" needs d above COUNT_NOISE_SIGMAS noise standard deviations (and the
 * old 3 blobs); a d in between, or impossibly negative, repeats the step and averages, up to COUNT_MAX_REPEATS times,
 * which shrinks the noise. A calm room keeps its threshold of 3 and its speed.
 */
#define COUNT_NOISE_SIGMAS 3.0f
/*!
 * The tracker's view as evidence: a step is "on" when the tracker matched at least this many of this controller's
 * blobs in one of the frames with the LEDs at the test offset. Its matches are blobs on this controller's LED model
 * (with the IMU checks), which a sunlit floor's spots don't make, unlike the blob count: in direct sun the count
 * swung 30-80 blobs between a step's two halves while one controller held its window and tracked (12:59, 14:31). The
 * count still decides when the tracker has nothing to say (before it first sees the controller, in a calm room).
 */
#define COUNT_TRACKER_MIN_LEDS 4
/*!
 * Frames each phase counts: enough that the difference of the two phase means varies by about COUNT_TARGET_SD blobs
 * from frame noise, 2 in a calm room (frame sd ~1.5) up to COUNT_MAX_FRAMES_PER_PHASE in a bright one (12:59: ~6-8).
 * A frame costs 17ms, repeating a step ~180ms (two LED changes and settles), so averaging within the phase is cheaper.
 */
#define COUNT_TARGET_SD 3.0f
#define COUNT_MAX_FRAMES_PER_PHASE 10
#define COUNT_MAX_REPEATS 3
//! Noise variance (blobs^2) assumed before any is measured, and how fast the estimate follows.
#define COUNT_NOISE_INITIAL_VAR 1.0f
#define COUNT_NOISE_GAIN 0.2f
/*!
 * Edge search resolution. PSVR2Toolkit uses 25us; the window has 300-500us of room on each side of the 1.5ms blink and
 * the hold centres it in 100us steps within seconds. The two edge searches were ~85% of a calibration (~1 s each at
 * 100us, calib_sim SIM_BREAKDOWN=1), and the second controller waits for the first's. 100 -> 300us (calib_sim, 100
 * starts per case, time until both controllers hold): normal 4.8 -> 3.6 s, exposure 290us 4.8 -> 3.6 s (worst 8.5 ->
 * 4.2), exposure 1150us 4.8 -> 3.6 s, flickering background blobs 6.6 -> 4.8 s, wandering 5.0 -> 3.7 s; none failed;
 * held centre error mean 13-30 -> 28-52us, worst about the same. 250us gives the same; 400us 3.2 s, but at 500us the
 * 290 and 1150us exposures took 7.6 and 9.7 s (searches failing and starting again), so 300 keeps the margin.
 * One frame per step instead of two (COUNT_FRAMES_PER_STEP) locked onto wrong windows with flickering blobs.
 */
#ifndef COUNT_EDGE_RESOLUTION_NS
#define COUNT_EDGE_RESOLUTION_NS (U_TIME_1US_IN_NS * 300)
#endif
/*!
 * Where the window's right edge can be: the blink plus the camera's exposure, which the PS VR2 doesn't report (it sends a
 * fixed setting) and which moved 606-1000us between and within sessions. The searches used to assume the setting
 * (0.9-1.1x of blink + 840us): a 2106us window sat on that limit, and in simulation an exposure of 1150us never
 * finished (psvr2-test/ledsync/calib_sim).
 */
#define COUNT_MAX_EXPOSURE_NS (U_TIME_1US_IN_NS * 1400)
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
 * Pause after a sweep that saw no LEDs, before sweeping again. Was 3s: at startup the controller often isn't in view
 * yet when the first sweep runs (02:54, 01:45: the first sweep saw nothing), and the wait then delayed the first
 * controller's calibration by the whole 3s. A sweep takes ~2.7s, so retrying sooner costs nothing but some LED
 * flashing while the controller is out of view.
 */
#define COUNT_CALIB_EMPTY_RETRY_NS (U_TIME_1S_IN_NS)

/*!
 * The quick recalibration (after a lost hold) skips the confirmation only with a strong signal: an on - off blob count
 * difference of at least this much somewhere in the pass. On hardware (11:50) one was accepted on 4 vs 0.5 blobs, just
 * over the 3 blob threshold, and lost again.
 */
#define COUNT_QUICK_MIN_DELTA 8.0f


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
	cc->sum_sq = 0;
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
	/*
	 * Offsets are positions in a periodic blink cycle and may be negative (the left edge search goes below 0 whenever
	 * the sweep found the window at its first step). Clamping this to 0 started the sweep after the window instead of
	 * before it, so it went round the whole frame: 14 steps in the 10:58 run (left edge at -1773us).
	 */
	time_duration_ns start_ns = cc->left_ns - cc->full_window_ns;
	cc->state = COUNT_CALIB_FIND_ON;
	cc->swept_ns = 0;
	cc->pass_max_delta = 0;
	cc->repeats = 0;
	cc->repeat_sum = 0.0f;
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
	refinement->count_calib.sum_sq = 0;
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
	refinement->count_calib.sum_sq = 0;
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
	cc->sum_sq += (uint64_t)cc->last_count * cc->last_count;
	cc->frames++;

	// The frames the tracker's matches count for (the LEDs are at the test offset in the on phase).
	if (!cc->off_phase) {
		if (cc->frames == 1) {
			cc->on_from_seq = cc->count_sequence_id;
		}
		cc->on_to_seq = cc->count_sequence_id;
	}

	if (cc->frames < needed) {
		return false;
	}
	*out_mean = (float)cc->sum / (float)cc->frames;
	return true;
}

//! A tracker sample for this controller while calibrating, see COUNT_TRACKER_MIN_LEDS.
static void
count_calib_push_sample(struct t_led_sync_refinement *refinement, const struct t_constellation_tracker_sample *sample)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	if (!sample->has_world_pose || cc->on_from_seq == 0 || sample->sequence_id < cc->on_from_seq ||
	    sample->sequence_id > cc->on_to_seq) {
		return;
	}
	if (sample->metrics.matched_blob_count > cc->trk_leds_max) {
		cc->trk_leds_max = sample->metrics.matched_blob_count;
	}
}

static void
count_calib_finish(struct t_led_sync_refinement *refinement)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;

	time_duration_ns centre_ns = cc->left_ns + (cc->right_ns - cc->left_ns) / 2;
	time_duration_ns window_ns = cc->right_ns - cc->left_ns;
	LOG_INFO(refinement,
	         "Count calibration done: visible from %" PRId64 "us to %" PRId64 "us (window %" PRId64
	         "us, blink %" PRId64 "us, so exposure ~%" PRId64 "us), holding at %" PRId64 "us, took %.1fs",
	         (int64_t)(cc->left_ns / U_TIME_1US_IN_NS), (int64_t)(cc->right_ns / U_TIME_1US_IN_NS),
	         (int64_t)(window_ns / U_TIME_1US_IN_NS),
	         (int64_t)(refinement->current_blink_duration_ns / U_TIME_1US_IN_NS),
	         (int64_t)((window_ns - refinement->current_blink_duration_ns) / U_TIME_1US_IN_NS),
	         (int64_t)(centre_ns / U_TIME_1US_IN_NS),
	         cc->started_ns != 0 ? (double)(os_monotonic_get_ns() - cc->started_ns) / 1e9 : 0.0);

	if (!cc->startup_marked) {
		cc->startup_marked = true;
		u_startup_mark("[%s] controller: LED timing found", LED_SYNC_NAME(refinement));
	}
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
	cc->hold_moves_sum_ns = 0;
	cc->hold_seg_start_ns = 0;
	cc->hold_have_seg_rate = false;
	cc->hold_unseen_probes = 0;
	cc->hold_seen_since_calib = false;
	cc->hold_interval_ns = COUNT_HOLD_PROBE_NS;
	cc->hold_centred_streak = 0;
	// The window was just measured with the current clock offset: that's the reference to follow it from.
	cc->bt_have_ref = cc->bt_valid;
	cc->bt_ref_ns = cc->bt_smooth_ns;
	cc->bt_centre_ref_ns = centre_ns;
	cc->bt_trim_ref_ns = 0; // hold_trim_integral_ns starts again too
	cc->bt_jump_pending = false;
	cc->hold_have_fast_rate = false;
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
	cc->pass_max_delta = 0;
	cc->repeats = 0;
	cc->repeat_sum = 0.0f;
	cc->trk_leds_max = 0;
	cc->on_from_seq = 0;
	count_calib_set_offset(refinement, start_ns);
}

//! One reading with the LEDs always on, then off.
static void
count_calib_start_diag_on(struct t_led_sync_refinement *refinement)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	cc->state = COUNT_CALIB_DIAG_ON;
	cc->frames = 0;
	cc->sum = 0;
	cc->sum_sq = 0;
	cc->off_phase = false;
	struct t_led_sync_sample_pssense_data data = {.leds = {0xFF, 0xFF, 0xFF, 0xFF}};
	set_offset_duration_pssense(refinement, 0, cc->offset_ns, refinement->current_blink_duration_ns,
	                            T_LED_SYNC_SAMPLE_PSSENSE_MODE_LED_ALL_ON, data);
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
		/*
		 * No always-on visibility check first: on hardware (02:03) the LEDs read 0 blobs always on while a sweep
		 * found them moments later (16.5 blobs), so the check only cost ~2s.
		 */
		count_calib_start_sweep(refinement);
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

	if (cc->phase_frames < COUNT_FRAMES_PER_STEP) {
		cc->phase_frames = COUNT_FRAMES_PER_STEP;
	}
	float mean = 0;
	if (!count_calib_accumulate(refinement, cc->phase_frames, &mean)) {
		return;
	}

	// Variance between this phase's frames (no LED change within a phase).
	float phase_var = 0.0f;
	if (cc->frames >= 2) {
		double n = cc->frames;
		phase_var = (float)(((double)cc->sum_sq - (double)cc->sum * (double)cc->sum / n) / (n - 1.0));
		if (phase_var < 0.0f) {
			phase_var = 0.0f;
		}
	}
	cc->sum_sq = 0;

	if (!cc->off_phase) {
		/*
		 * Measured the LEDs at the test offset, now the same with them off. Every step: the background moved
		 * 25 -> 125 blobs within seconds on hardware (12:52), so an earlier off count can't stand in for it.
		 */
		cc->on_mean = mean;
		cc->on_var = phase_var;
		cc->off_phase = true;
		cc->frames = 0;
		cc->sum = 0;
		cc->sum_sq = 0;
		count_calib_set_leds_off(refinement);
		return;
	}

	if (cc->noise_var <= 0.0f) {
		cc->noise_var = COUNT_NOISE_INITIAL_VAR;
	}
	if (cc->frame_var <= 0.0f) {
		cc->frame_var = COUNT_NOISE_INITIAL_VAR;
	}
	cc->frame_var += COUNT_NOISE_GAIN * ((cc->on_var + phase_var) / 2.0f - cc->frame_var);
	/*
	 * The difference of two means of n frames each varies by (var_on + var_off) / n from frame noise alone, and a
	 * negative difference is all noise (frame noise and the background changing between the phases).
	 */
	float n_frames = (float)cc->phase_frames;
	float d = cc->on_mean - mean;
	float sample_var = (cc->on_var + phase_var) / n_frames;
	if (d < 0.0f) {
		sample_var = fmaxf(sample_var, d * d);
	}
	cc->noise_var += COUNT_NOISE_GAIN * (sample_var - cc->noise_var);

	// Frames for the next phases: 2 * frame_var / n <= COUNT_TARGET_SD^2.
	{
		float want = ceilf(2.0f * cc->frame_var / (COUNT_TARGET_SD * COUNT_TARGET_SD));
		if (want < COUNT_FRAMES_PER_STEP) {
			want = COUNT_FRAMES_PER_STEP;
		}
		if (want > COUNT_MAX_FRAMES_PER_PHASE) {
			want = COUNT_MAX_FRAMES_PER_PHASE;
		}
		cc->phase_frames = (uint32_t)want;
	}

	cc->repeats++;
	cc->repeat_sum += d;
	float mean_d = cc->repeat_sum / (float)cc->repeats;
	float sigma = sqrtf(cc->noise_var / (float)cc->repeats);
	float threshold = fmaxf(COUNT_THRESHOLD_MARGIN, COUNT_NOISE_SIGMAS * sigma);
	bool tracker_on = cc->trk_leds_max >= COUNT_TRACKER_MIN_LEDS;
	bool unclear = mean_d > COUNT_THRESHOLD_MARGIN && mean_d <= threshold;
	bool impossible = mean_d < -threshold;
	if (!tracker_on && (unclear || impossible) && cc->repeats < COUNT_MAX_REPEATS) {
		LOG_DEBUG(refinement,
		          "Count calibration state %d offset %" PRId64
		          "us: %.1f blobs with LEDs, %.1f without, difference %.1f vs threshold %.1f, measuring again",
		          cc->state, (int64_t)(cc->offset_ns / U_TIME_1US_IN_NS), cc->on_mean, mean, mean_d, threshold);
		count_calib_set_offset(refinement, cc->offset_ns);
		return;
	}
	cc->repeats = 0;
	cc->repeat_sum = 0.0f;
	uint32_t trk_leds = cc->trk_leds_max;
	cc->trk_leds_max = 0;
	cc->on_from_seq = 0;

	cc->baseline = mean;
	cc->threshold = mean + threshold;
	bool on = tracker_on || mean_d > threshold;
	// The strength check (skipping confirmation) compares against the noise as well; the tracker's view is strong.
	if (mean_d - (threshold - COUNT_THRESHOLD_MARGIN) > cc->pass_max_delta) {
		cc->pass_max_delta = mean_d - (threshold - COUNT_THRESHOLD_MARGIN);
	}
	if (tracker_on && cc->pass_max_delta < COUNT_QUICK_MIN_DELTA) {
		cc->pass_max_delta = COUNT_QUICK_MIN_DELTA;
	}

	LOG_DEBUG(refinement,
	          "Count calibration state %d offset %" PRId64
	          "us: %.1f blobs with LEDs, %.1f without (difference %.1f, threshold %.1f), tracker matched %u -> %s",
	          cc->state, (int64_t)(cc->offset_ns / U_TIME_1US_IN_NS), cc->on_mean, mean, mean_d, threshold, trk_leds,
	          on ? "on" : "off");

	switch (cc->state) {

	case COUNT_CALIB_FIND_ON:
		if (on) {
			cc->empty_sweeps = 0;
			cc->upper_ns = cc->offset_ns;
			cc->lower_ns = cc->offset_ns - (refinement->current_blink_duration_ns + COUNT_MAX_EXPOSURE_NS);
			cc->edge_seen_off = false;
			cc->state = COUNT_CALIB_LEFT_EDGE;
			count_calib_set_offset(refinement, (cc->lower_ns + cc->upper_ns) / 2);
		} else if ((cc->swept_ns += fw / 2) >= refinement->exposure_interval_ns) {
			if (++cc->empty_sweeps >= COUNT_CALIB_EMPTY_SWEEPS_DIAG) {
				// Are the LEDs visible at all? Light them regardless of timing and compare with off.
				cc->empty_sweeps = 0;
				count_calib_start_diag_on(refinement);
				break;
			}
			// Out of view most likely: pause before sweeping again, see COUNT_CALIB_EMPTY_RETRY_NS.
			cc->retry_after_ns = os_monotonic_get_ns() + COUNT_CALIB_EMPTY_RETRY_NS;
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
			cc->edge_seen_off = true;
		}
		if (cc->upper_ns - cc->lower_ns < COUNT_EDGE_RESOLUTION_NS) {
			cc->left_ns = cc->upper_ns;
			cc->left_seen_off = cc->edge_seen_off;
			cc->edge_seen_off = false;
			/*
			 * No drift is learned here: two searches can land a whole sweep step (blink + exposure) apart, which
			 * on hardware read as +2340us in 3.5s (662ppm, against a real few ppm) and pushed the trim to its
			 * limit, walking that controller's LEDs out of the window. Drift comes only from the hold.
			 */
			/*
			 * From inside any real window: it is the blink plus the exposure, so 3/4 of the blink after the left
			 * edge is always still lit. Starting at blink + 300us (as before) missed short exposures: with Sony's
			 * LED detector settings the window measured 1868us (exposure ~370us), the search began 70us before
			 * its right edge where the LEDs are barely exposed, and the left controller's calibration failed for
			 * the whole 15:38 session ("right edge search never saw the LEDs"; calib_sim at 368us: 90/100 never
			 * finished, now 0).
			 */
			cc->lower_ns = cc->left_ns + (refinement->current_blink_duration_ns * 3) / 4;
			cc->upper_ns = cc->left_ns + refinement->current_blink_duration_ns + COUNT_MAX_EXPOSURE_NS;
			cc->edge_seen_on = false;
			cc->state = COUNT_CALIB_RIGHT_EDGE;
		}
		count_calib_set_offset(refinement, (cc->lower_ns + cc->upper_ns) / 2);
		break;

	case COUNT_CALIB_RIGHT_EDGE:
		if (on) {
			cc->lower_ns = cc->offset_ns;
			cc->edge_seen_on = true;
		} else {
			cc->upper_ns = cc->offset_ns;
			cc->edge_seen_off = true;
		}
		if (cc->upper_ns - cc->lower_ns < COUNT_EDGE_RESOLUTION_NS) {
			cc->right_ns = cc->lower_ns;
			time_duration_ns width_ns = cc->right_ns - cc->left_ns;
			/*
			 * Both edges must have been seen: a search that only ever reads "on" ends at its limit, which is the
			 * expected width, so the width alone looked plausible. On hardware (02:03) such a window
			 * (12577-15136us, the right edge never seen) was accepted unconfirmed and lost within 5s; the real
			 * one was 16087-18632us.
			 */
			/*
			 * The right search starts inside any real window, so it must read "on" too: all "off" means the
			 * controller left the view, and its edge is just the search's lower limit (11:45: 237-2343us, every
			 * right probe 1-2 blobs).
			 */
			bool edges_seen = cc->left_seen_off && cc->edge_seen_off && cc->edge_seen_on;
			cc->pass_plausible =
			    edges_seen && width_ns > (refinement->current_blink_duration_ns * 4) / 5 &&
			    width_ns < refinement->current_blink_duration_ns + COUNT_MAX_EXPOSURE_NS;
			if (!edges_seen) {
				count_calib_research(refinement, !cc->left_seen_off ? "left edge not seen"
				                                 : !cc->edge_seen_on ? "right edge search never saw the LEDs"
				                                                     : "right edge not seen");
				break;
			}
			if (cc->pass_plausible) {
				cc->have_plausible = true;
				cc->plausible_left_ns = cc->left_ns;
				cc->plausible_right_ns = cc->right_ns;
			}
			if (cc->pass_plausible && cc->pass_max_delta >= COUNT_QUICK_MIN_DELTA) {
				/*
				 * Both edges seen, a plausible width and a strong on/off difference: the hold's edge probes centre
				 * the window within seconds, so skip the four confirmation steps (and the re-searches a failed one
				 * starts). Across the logs, 3 of 5 failed confirmations were false (final window within 0-256us of
				 * the rejected one), and the 2 real ones had 0 blobs throughout, which the strength check rejects.
				 * In psvr2-test/ledsync/calib_sim confirming every calibration was slower (5.0 vs 3.7s in view,
				 * 10-12 vs 3.7s with the controller out of view a fifth of the time) and no more accurate, also with
				 * a wandering background (SIM_AMBIENT_WALK): a failed confirmation retries once, then accepts.
				 */
				LOG_INFO(refinement, "Count calibration: window plausible with a strong signal, skipping confirmation");
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
			cc->retry_after_ns = os_monotonic_get_ns() + COUNT_CALIB_EMPTY_RETRY_NS;
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
 * used once T >= 10s (error <= 10ppm). Up to 50ppm it is used at once: the controller clocks measure -3 to +47ppm
 * against the host. Faster drift happens too, from a clock all the controllers share (the host's or the headset's):
 * on two users' PCs (2026-11-21/22) both controllers' windows moved together at ~75-100ppm, the trim stayed at its
 * limit and the hold lost the LEDs. So a rate up to COUNT_HOLD_DRIFT_FAST_MAX (the most Linux's time sync slews the
 * clock by) is used too, but only when an estimate at least COUNT_HOLD_DRIFT_FAST_AGREE_NS older agreed (same sign,
 * within COUNT_HOLD_DRIFT_FAST_TOL or a quarter): a single misreading, as the 662ppm read from two calibration
 * searches 3.5s apart, isn't taken.
 */
#define COUNT_HOLD_EDGE_MARGIN_NS (U_TIME_1US_IN_NS * 150)
#define COUNT_HOLD_DRIFT_MIN_T_NS (U_TIME_1S_IN_NS * 10)
#define COUNT_HOLD_DRIFT_MAX (50e-6)
#ifndef COUNT_HOLD_DRIFT_FAST_MAX
#define COUNT_HOLD_DRIFT_FAST_MAX (500e-6)
#endif
#define COUNT_HOLD_DRIFT_FAST_AGREE_NS (U_TIME_1S_IN_NS * 3)
#define COUNT_HOLD_DRIFT_FAST_TOL (20e-6)
/*!
 * High drift is detected when the hold's applied phase moved over COUNT_HOLD_FAST_DETECT in two COUNT_HOLD_FAST_SEG_NS
 * stretches in a row, the same way. Normal drift (-3 to +47ppm measured) never gets there; a one-off shift falls in
 * one stretch. Until then the hold runs as it did before 2026-11-22 (calib_sim: identical results at 0-47ppm).
 */
#ifndef COUNT_HOLD_FAST_SEG_NS
#define COUNT_HOLD_FAST_SEG_NS (U_TIME_1S_IN_NS * 10)
#endif
#ifndef COUNT_HOLD_FAST_DETECT
#define COUNT_HOLD_FAST_DETECT (60e-6)
#endif
/*!
 * Probes in a row with this controller's LEDs not seen at the centre before calibrating again (unconfirmed window).
 * The nudge probe measures the LEDs the tracker matched, so until the tracker has found the controller once every
 * probe reads "not in view", right window or not. At 3 (~3 s at the 1 s unseen interval) a controller the tracker
 * was slow to find was calibrated again and again (6-8 s each, LEDs mostly at wrong offsets): a user's cluttered
 * room (2026-11-21, 50-150 background blobs) took 5 calibrations and 60 s, all finding the same window.
 * calib_sim SIM_ACQUIRE_S (tracker needs 1-20 s on average, 120 starts each), 3 -> 8: start 4.9/11.5/21.0/40.9 s ->
 * 4.9/9.4/16.7/32.4 s, worst 15/56/96/204 s -> 9/30/86/156 s, most calibrations 2/6/11/22 -> 1/2/5/9; the same or
 * better with wandering or flickering background blobs except one worst case (81 vs 58 s). A wrong window is held
 * ~8 s instead of ~3 s before it's calibrated again.
 */
#ifndef COUNT_HOLD_UNSEEN_PROBES
#define COUNT_HOLD_UNSEEN_PROBES 8
#endif
/*!
 * Once the hold has seen the controller since its calibration, how long it keeps the timing while the controller is
 * unseen. Unseen almost always means out of view: the timing can only walk out of the window by drift, which at the
 * <= 30ppm measured takes >= 40s to cross the 1280us loss point. Calibrating again after 3 probes (~5s) instead
 * threw away a good window: on hardware (11:50) both controllers left every camera's view for ~17s, the windows found
 * afterwards were within 550-650us of the held ones, and the recalibrations, with the controllers out of view or half
 * visible, looped (calibrate, lose, calibrate) so tracking only fully returned ~15s after they were back in view.
 */
#define COUNT_HOLD_UNSEEN_KEEP_NS (U_TIME_1S_IN_NS * 20)

/*!
 * Bluetooth clock follow. The driver holds its host -> controller LED clock mapping fixed (only trimmed by the drift
 * rate the hold measures), because following the Bluetooth clock estimate moved the LEDs out of the window: its errors
 * were 0-330us/s of drift or ~0.5ms steps. But the controller's clock relation can also really change: in a user's
 * session (2026-11-20 23:26, firmware 06000102) the Bluetooth offset of each controller moved ~1.2ms within ~40s, one
 * after the other, after minutes of < 2ppm, and each controller's window then moved the same way (+0.7 / +0.4ms by the
 * end, still moving), which the hold chased in 100us steps at up to 30s probe intervals.
 *
 * So the hold follows a Bluetooth change only when its own optical probe confirms it: once the smoothed offset has
 * moved COUNT_HOLD_BT_TRIGGER_NS from its value at the last centred probe, it probes at once (not up to 30s later),
 * and if that probe sees the blink off centre in the same direction, it moves the centre by the whole Bluetooth change
 * instead of one 100us step. The probe after (COUNT_HOLD_BT_VERIFY_NS later) has to see the LEDs again, or the move is
 * undone. A Bluetooth error with the LEDs still centred moves nothing.
 */
#define COUNT_HOLD_BT_TRIGGER_NS (U_TIME_1US_IN_NS * 300)
//! Compile-time switch for the simulator's before/after comparison (psvr2-test/ledsync/calib_sim).
#ifndef COUNT_HOLD_BT_FOLLOW
#define COUNT_HOLD_BT_FOLLOW 1
#endif
//! Largest move taken from the Bluetooth change, beyond that it's a clock reset, not a shift.
#define COUNT_HOLD_BT_MAX_NS (U_TIME_1MS_IN_NS * 3)
#define COUNT_HOLD_BT_VERIFY_NS (U_TIME_1S_IN_NS * 1)
//! Smoothing of the offset per report (~250 reports/s: ~0.4s).
#define COUNT_HOLD_BT_ALPHA 0.01

static void
count_hold_push_clock_offset(struct t_led_sync_refinement *refinement, time_duration_ns offset_ns)
{
	__typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	if (!cc->bt_valid || fabs((double)offset_ns - cc->bt_smooth_ns) > 50e6) {
		// First value, or a clock reset (reconnect): start over, and drop a reference from the old clock.
		cc->bt_smooth_ns = (double)offset_ns;
		cc->bt_have_ref = false;
		cc->bt_valid = true;
		return;
	}
	cc->bt_smooth_ns += COUNT_HOLD_BT_ALPHA * ((double)offset_ns - cc->bt_smooth_ns);
}

//! The Bluetooth offset's change since the last centred probe, 0 without a reference.
static double
count_hold_bt_change_ns(const struct t_led_sync_refinement *refinement)
{
	const __typeof__(refinement->count_calib) *cc = &refinement->count_calib;
	return COUNT_HOLD_BT_FOLLOW && cc->bt_have_ref ? cc->bt_smooth_ns - cc->bt_ref_ns : 0.0;
}

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
	/*
	 * Normal drift runs the hold as it was before 2026-11-22. Only once high drift is detected (see
	 * COUNT_HOLD_FAST_*) does it also count the Bluetooth moves in the drift estimate, measure each Bluetooth move from
	 * the last confirmed one, take the trim already applied off it, and use drift rates over COUNT_HOLD_DRIFT_MAX.
	 */
	const bool fast = cc->high_drift;
	const time_duration_ns centre_before_ns = cc->hold_centre_ns;
	bool bt_moved = false;
	if (cc->bt_jump_pending) {
		cc->bt_jump_pending = false;
		const int dir = cc->hold_centre_ns > cc->bt_pre_jump_centre_ns ? 1 : -1;
		/*
		 * LEDs gone, or the blink now off centre the other way: the move overshot, so the Bluetooth change wasn't
		 * (all) the LEDs'. Go back to where the hold was, plus the one step its probe asked for. In simulation a
		 * Bluetooth-only error otherwise made ~1 wrong move per run whenever the normal drift corrections happened
		 * to point the same way.
		 */
		if (!in_view || e == -dir) {
			LOG_INFO(refinement,
			         "Count hold: the Bluetooth clock move %s, back to %" PRId64 "us",
			         in_view ? "overshot" : "lost the LEDs",
			         (int64_t)((cc->bt_pre_jump_centre_ns + dir * COUNT_HOLD_STEP_NS) / U_TIME_1US_IN_NS));
			cc->hold_centre_ns = cc->bt_pre_jump_centre_ns + dir * COUNT_HOLD_STEP_NS;
			cc->bt_jumps_undone++;
			// Don't take the same change again: the next reference is set from a centred probe.
			cc->bt_have_ref = false;
			e = 0;
		} else if (fast && cc->bt_have_ref) {
			/*
			 * High drift, confirmed: the next move is measured from this one. Measured from the last centred probe,
			 * a steady drift (a user's 2026-11-22 session: ~100 ppm on both controllers, the probes never centred
			 * again) added up to COUNT_HOLD_BT_MAX_NS within ~30 s, the hold stopped following and lost the LEDs.
			 */
			cc->bt_ref_ns = cc->bt_jump_smooth_ns;
			cc->bt_centre_ref_ns = cc->hold_centre_ns;
			cc->bt_trim_ref_ns = cc->bt_jump_trim_ns;
		}
	}
	if (in_view && e != 0 && cc->bt_have_ref) {
		// High drift: the trim applied since the reference already moved the LEDs that far with the clock.
		const double trimmed_ns = fast ? cc->hold_trim_integral_ns - cc->bt_trim_ref_ns : 0.0;
		const double bt_ns = count_hold_bt_change_ns(refinement) - trimmed_ns;
		const double done_ns = (double)(cc->hold_centre_ns - cc->bt_centre_ref_ns);
		// The blink sits off centre the way the clock moved (the window moves later as the offset grows).
		if ((bt_ns > 0) == (e > 0) && fabs(bt_ns) >= (double)COUNT_HOLD_BT_TRIGGER_NS &&
		    fabs(bt_ns) <= (double)COUNT_HOLD_BT_MAX_NS && fabs(bt_ns - done_ns) >= (double)COUNT_HOLD_STEP_NS) {
			cc->bt_pre_jump_centre_ns = cc->hold_centre_ns;
			cc->hold_centre_ns = cc->bt_centre_ref_ns + (time_duration_ns)bt_ns;
			cc->bt_jump_smooth_ns = cc->bt_smooth_ns;
			cc->bt_jump_trim_ns = cc->hold_trim_integral_ns;
			cc->bt_jump_pending = true;
			cc->bt_jumps++;
			bt_moved = true;
			LOG_INFO(refinement,
			         "Count hold: following the Bluetooth clock, it moved %+.0fus and the probe agrees, centre %" PRId64
			         "us -> %" PRId64 "us",
			         bt_ns / 1e3, (int64_t)(cc->bt_pre_jump_centre_ns / U_TIME_1US_IN_NS),
			         (int64_t)(cc->hold_centre_ns / U_TIME_1US_IN_NS));
		}
	}
	if (e != 0 && !bt_moved) {
		cc->hold_centre_ns += e * COUNT_HOLD_STEP_NS;
		cc->hold_corrections_sum_ns += e * (double)COUNT_HOLD_STEP_NS;
		cc->hold_corrections++;
	}
	// Every move of the centre (step, Bluetooth move, undo): what high drift is detected and estimated from.
	cc->hold_moves_sum_ns += (double)(cc->hold_centre_ns - centre_before_ns);
	if (in_view && e == 0 && !cc->bt_jump_pending && cc->bt_valid) {
		// Centred: the clock offset now belongs to this centre.
		cc->bt_have_ref = true;
		cc->bt_ref_ns = cc->bt_smooth_ns;
		cc->bt_centre_ref_ns = cc->hold_centre_ns;
		cc->bt_trim_ref_ns = cc->hold_trim_integral_ns;
	}

	time_duration_ns t_ns = now_ns - cc->hold_start_ns;
	if (in_view && t_ns >= COUNT_HOLD_DRIFT_MIN_T_NS && !fast) {
		double rate = (cc->hold_corrections_sum_ns + cc->hold_trim_integral_ns) / (double)t_ns;
		if (rate >= -COUNT_HOLD_DRIFT_MAX && rate <= COUNT_HOLD_DRIFT_MAX) {
			cc->skew = rate;
		} else {
			LOG_DEBUG(refinement, "Count hold drift estimate %+.1fppm is implausible, trim left at %+.1fppm",
			          rate * 1e6, cc->skew * 1e6);
		}
	} else if (in_view && t_ns >= COUNT_HOLD_DRIFT_MIN_T_NS) {
		/*
		 * High drift: before 2026-11-22 only the 100us steps were counted, not the Bluetooth moves that carry most
		 * of a fast drift, so the estimate stayed near 0 while the hold chased the drift move after move.
		 */
		double rate = (cc->hold_moves_sum_ns + cc->hold_trim_integral_ns) / (double)t_ns;
		const double fast_tol = fmax(COUNT_HOLD_DRIFT_FAST_TOL, 0.25 * fabs(rate));
		if (rate >= -COUNT_HOLD_DRIFT_MAX && rate <= COUNT_HOLD_DRIFT_MAX) {
			cc->skew = rate;
			cc->hold_have_fast_rate = false;
		} else if (fabs(rate) <= COUNT_HOLD_DRIFT_FAST_MAX && cc->hold_have_fast_rate &&
		           now_ns - cc->hold_fast_rate_ns >= COUNT_HOLD_DRIFT_FAST_AGREE_NS &&
		           (rate > 0) == (cc->hold_fast_rate > 0) && fabs(rate - cc->hold_fast_rate) <= fast_tol) {
			if (fabs(cc->skew) <= COUNT_HOLD_DRIFT_MAX) {
				LOG_INFO(refinement, "Count hold: the LEDs drift %+.0fppm (measured over %.0fs), trimming it",
				         rate * 1e6, (double)t_ns / U_TIME_1S_IN_NS);
			}
			cc->skew = rate;
		} else {
			// The first fast estimate, or one that disagreed: a later one has to agree with this one.
			if (fabs(rate) <= COUNT_HOLD_DRIFT_FAST_MAX &&
			    (!cc->hold_have_fast_rate || now_ns - cc->hold_fast_rate_ns >= COUNT_HOLD_DRIFT_FAST_AGREE_NS)) {
				cc->hold_fast_rate = rate;
				cc->hold_fast_rate_ns = now_ns;
				cc->hold_have_fast_rate = true;
			}
			LOG_DEBUG(refinement, "Count hold drift estimate %+.1fppm not used yet, trim left at %+.1fppm",
			          rate * 1e6, cc->skew * 1e6);
		}
	}

	/*
	 * High-drift detection: the phase applied (every move plus the trim) per 10 s stretch. Two stretches in a row
	 * over COUNT_HOLD_FAST_DETECT in the same direction is a drift, not a one-off shift (that lands in one stretch).
	 */
	const double phase_ns = cc->hold_moves_sum_ns + cc->hold_trim_integral_ns;
	if (cc->hold_seg_start_ns == 0) {
		cc->hold_seg_start_ns = now_ns;
		cc->hold_seg_start_phase_ns = phase_ns;
	} else if (now_ns - cc->hold_seg_start_ns >= COUNT_HOLD_FAST_SEG_NS) {
		const double seg_rate = (phase_ns - cc->hold_seg_start_phase_ns) / (double)(now_ns - cc->hold_seg_start_ns);
		if (!cc->high_drift && cc->hold_have_seg_rate && fabs(seg_rate) > COUNT_HOLD_FAST_DETECT &&
		    fabs(cc->hold_seg_prev_rate) > COUNT_HOLD_FAST_DETECT && (seg_rate > 0) == (cc->hold_seg_prev_rate > 0)) {
			cc->high_drift = true;
			LOG_INFO(refinement,
			         "Count hold: high LED drift (%+.0f then %+.0fppm over 10 s), following it from now on",
			         cc->hold_seg_prev_rate * 1e6, seg_rate * 1e6);
		}
		cc->hold_seg_prev_rate = seg_rate;
		cc->hold_have_seg_rate = true;
		cc->hold_seg_start_ns = now_ns;
		cc->hold_seg_start_phase_ns = phase_ns;
	}

	if (in_view) {
		cc->hold_unseen_probes = 0;
		cc->hold_seen_since_calib = true;
	} else {
		if (cc->hold_unseen_probes == 0) {
			cc->hold_unseen_since_ns = now_ns;
		}
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
	if (cc->bt_jump_pending) {
		cc->hold_interval_ns = COUNT_HOLD_BT_VERIFY_NS;
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

	bool lost = cc->hold_seen_since_calib
	                ? (cc->hold_unseen_probes > 0 && now_ns - cc->hold_unseen_since_ns >= COUNT_HOLD_UNSEEN_KEEP_NS)
	                : cc->hold_unseen_probes >= COUNT_HOLD_UNSEEN_PROBES;
	if (lost) {
		// This controller's own LEDs gone at the centre for too long: find the window again.
		LOG_INFO(refinement, "Count hold lost the LEDs for %u probes (%.0fs, window %s), calibrating again",
		         cc->hold_unseen_probes, (double)(now_ns - cc->hold_unseen_since_ns) / U_TIME_1S_IN_NS,
		         cc->hold_seen_since_calib ? "confirmed" : "never confirmed");
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
		// The clock moved by Bluetooth: check now whether the LEDs moved with it.
		bool bt_trigger = fabs(count_hold_bt_change_ns(refinement)) >= (double)COUNT_HOLD_BT_TRIGGER_NS;
		if (since_probe_ns >= cc->hold_interval_ns || (unseen && since_probe_ns >= COUNT_HOLD_PROBE_UNSEEN_NS) ||
		    (bt_trigger && since_probe_ns >= COUNT_HOLD_BT_VERIFY_NS)) {
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
			cc->sum_sq = 0;
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
		cc->sum_sq = 0;
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
		// The clock moved by Bluetooth: check now whether the LEDs moved with it.
		bool bt_trigger = fabs(count_hold_bt_change_ns(refinement)) >= (double)COUNT_HOLD_BT_TRIGGER_NS;
		if (since_probe_ns >= cc->hold_interval_ns || (unseen && since_probe_ns >= COUNT_HOLD_PROBE_UNSEEN_NS) ||
		    (bt_trigger && since_probe_ns >= COUNT_HOLD_BT_VERIFY_NS)) {
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
