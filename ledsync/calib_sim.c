// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*
 * Drives the real LED sync refinement (t_led_sync_refinement.c with the PS Sense count calibration) for two
 * controllers against a simulated headset camera, on a simulated clock, to measure the startup calibration.
 *
 * Model (from the PS VR2 logs): 60 Hz frames, frame events processed ~7.5ms after the exposure start, the camera's
 * blob count = ambient + each controller's visible LEDs. A controller's LEDs show when the calibration offset lies in
 * its window [L, L + blink + exposure], ramping over ~150us at each edge; LEDs-all-off shows nothing. LED commands are
 * applied after a Bluetooth delay (send wait + round trip) and take effect from the next exposure after that. The
 * window drifts slowly; the controller can leave the view for stretches.
 *
 *   calib_sim [runs] [exposure_us] [dropout_share] [seed]
 *
 * SIM_DROP_AT=<s> SIM_DROP_FOR=<s>: the left controller's link drops at that time and comes back after that long, as
 * pssense_reconnect does it (t_led_sync_refinement_restart at the drop and at the reconnect). Its hold time is then
 * counted from the reconnect; the run passes when both hold.
 *
 * SIM_ACQUIRE_S=<s> (with SIM_TRACKER=1): the tracker needs on average that long, with the LEDs lit in the pictures,
 * to find a controller it doesn't have (each frame a 1 / (60 * s) chance), and loses it after 0.5 s without them;
 * reports how long after the start each controller is tracked (found, LEDs lit, 15 frames in a row) and how many
 * calibrations that took. In a user's cluttered room (2026-11-21, 50-150 background blobs) the left controller
 * wasn't found for 60 s, over 5 calibrations that each found the same window.
 *
 * SIM_SHIFT_US=<us> [SIM_SHIFT_S=<s>] (with SIM_TRACKER=1): 40 s after both hold (probe intervals grown), the left
 * controller's clock relation shifts by that much, as a ramp over SIM_SHIFT_S (0: a step); its LED window moves with
 * it and so does the Bluetooth clock offset the driver reports (with noise). SIM_BTERR_US=<us> [SIM_BTERR_S=<s>]:
 * instead (or as well) a Bluetooth-only error of that size, ramped over SIM_BTERR_S, held 10 s, then gone, while the
 * LEDs don't move. Reported for the left controller over the 90 s from the event: frames tracked (>= 4 LEDs in the
 * pictures), recalibrations, held centre error, and Bluetooth follow moves (made, undone).
 *
 * SIM_OUTAGE_FOR=<s> (with SIM_TRACKER=1): once both hold, the left controller holds 5 s more, then leaves every
 * camera's view for that long and comes back (its link stays up). Reported: how long after it is back in view until it
 * is tracked again (holding, its LEDs in the pictures for 15 frames running), and how many recalibrations that took.
 */
#include "os/os_time.h"

static int64_t g_now_ns = 1000000000LL;
static inline int64_t
sim_now_ns(void)
{
	return g_now_ns;
}
#define os_monotonic_get_ns sim_now_ns

#include "t_led_sync_refinement.c"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define FRAME_NS 16683000LL
//! LED change takes effect within this many frames after the echo (SIM_LED_LATENCY_FRAMES).
static double g_led_latency_frames = 1.0;
static bool g_ambient_walk = false;
static bool g_tracker = false;
static double g_drift = 15e-6;
static double g_ambient = 30.0;
static double g_flicker = 0.0;
#define PROC_DELAY_NS 7500000LL

static uint64_t rng_state = 88172645463325252ULL;
static double
urand(void)
{
	rng_state ^= rng_state << 13;
	rng_state ^= rng_state >> 7;
	rng_state ^= rng_state << 17;
	return (double)(rng_state >> 11) / 9007199254740992.0;
}
static int
poisson(double lambda)
{
	double L = exp(-lambda), p = 1.0;
	int k = 0;
	do {
		k++;
		p *= urand();
	} while (p > L);
	return k - 1;
}

struct sim_ctrl
{
	struct t_led_sync_refinement r;
	double window_left_ns; // true left edge, in calibration offset units
	int leds;              // LEDs seen when fully lit
	// Applied LED state
	bool on_mode;
	double offset_ns;
	// Pending command
	bool pending;
	int64_t apply_at_ns;
	struct t_led_sync_sample pending_sample;
	int64_t effective_from_ns;
	bool visible;
	int64_t vis_change_ns;
	// SIM_DROP_AT/SIM_DROP_FOR: this controller's Bluetooth link drops (the driver restarts its LED sync, no events
	// reach it, its LEDs aren't driven), then it reconnects (restart again) and must calibrate anew.
	bool dropped;
	int64_t reconnect_ns;
	int64_t hold_at_ns;
	// Simulated tracker: this frame's match, reported one frame later (SIM_TRACKER=1).
	int pending_match;
	uint64_t pending_seq;
	double pending_bright;
	double bt_ns; // the driver's host -> controller offset estimate (true part)
	bool acquired;  // SIM_ACQUIRE_S: the tracker has found this controller
	int dark_frames; // frames in a row without enough LEDs in the pictures
	int match_streak;
	int64_t tracked_at_ns;
	int calibrations;
	int prev_phase;
	double hold_centre_err_us;
	uint32_t steps;
};

/*!
 * The LEDs' brightness in a picture, 0..1: how much of the exposure the blink covers (the hold's nudge check compares
 * it at two offsets). Rises over the first exposure's worth of the window, flat while the blink covers the whole
 * exposure, falls over the last.
 */
static double
led_brightness(struct sim_ctrl *c, double exposure_ns, int64_t exposure_start_ns)
{
	if (!c->visible || !c->on_mode || exposure_start_ns < c->effective_from_ns)
		return 0;
	double blink = (double)c->r.current_blink_duration_ns;
	double L = c->window_left_ns, R = L + blink + exposure_ns;
	double off = fmod(c->offset_ns - L, (double)FRAME_NS);
	if (off < 0)
		off += FRAME_NS;
	off += L;
	double b = fmin(fmin(off - L, R - off), exposure_ns) / exposure_ns;
	return b < 0 ? 0 : (b > 1 ? 1 : b);
}

static int
visible_leds(struct sim_ctrl *c, double exposure_ns, int64_t exposure_start_ns)
{
	if (!c->visible || !c->on_mode || exposure_start_ns < c->effective_from_ns)
		return 0;
	double blink = (double)c->r.current_blink_duration_ns;
	double L = c->window_left_ns, R = L + blink + exposure_ns;
	// The offset is periodic in the frame.
	double off = fmod(c->offset_ns - L, (double)FRAME_NS);
	if (off < 0)
		off += FRAME_NS;
	off += L;
	double inside = fmin(off - L, R - off);
	double f = inside / 150000.0;
	if (f <= 0)
		return 0;
	if (f > 1)
		f = 1;
	return (int)lround(c->leds * f);
}

int
main(int argc, char **argv)
{
	int runs = argc > 1 ? atoi(argv[1]) : 50;
	double exposure_us = argc > 2 ? atof(argv[2]) : 840;
	double dropout = argc > 3 ? atof(argv[3]) : 0.0;
	rng_state ^= (uint64_t)(argc > 4 ? atoll(argv[4]) : 1) * 0x9E3779B97F4A7C15ULL;
	double exposure_ns = exposure_us * 1000.0;
	g_ambient_walk = getenv("SIM_AMBIENT_WALK") != NULL;
	g_tracker = getenv("SIM_TRACKER") != NULL;
	if (getenv("SIM_DRIFT_PPM"))
		g_drift = atof(getenv("SIM_DRIFT_PPM")) * 1e-6;
	g_flicker = getenv("SIM_AMBIENT_FLICKER") ? atof(getenv("SIM_AMBIENT_FLICKER")) : 0.0;
	if (getenv("SIM_LED_LATENCY_FRAMES"))
		g_led_latency_frames = atof(getenv("SIM_LED_LATENCY_FRAMES"));

	const double drop_at = getenv("SIM_DROP_AT") ? atof(getenv("SIM_DROP_AT")) : -1;
	const double drop_for = getenv("SIM_DROP_FOR") ? atof(getenv("SIM_DROP_FOR")) : 3;
	const double outage_for = getenv("SIM_OUTAGE_FOR") ? atof(getenv("SIM_OUTAGE_FOR")) : 0;
	const double shift_us = getenv("SIM_SHIFT_US") ? atof(getenv("SIM_SHIFT_US")) : 0;
	const double shift_s = getenv("SIM_SHIFT_S") ? atof(getenv("SIM_SHIFT_S")) : 0;
	const double bterr_us = getenv("SIM_BTERR_US") ? atof(getenv("SIM_BTERR_US")) : 0;
	const double bterr_s = getenv("SIM_BTERR_S") ? atof(getenv("SIM_BTERR_S")) : 0;
	const bool shift_mode = shift_us != 0 || bterr_us != 0 || getenv("SIM_SHIFT_BASE") != NULL;
	const double acquire_s = getenv("SIM_ACQUIRE_S") ? atof(getenv("SIM_ACQUIRE_S")) : 0;
	double acq_sum = 0, acq_worst = 0, acq_cal_sum = 0;
	int acq_n = 0, acq_never = 0, acq_cal_max = 0;
	double sh_tracked = 0, sh_worst_tracked = 1, sh_err_worst = 0, sh_err_sum = 0;
	int sh_runs = 0, sh_recal = 0, sh_recal_max = 0, sh_jumps = 0, sh_undone = 0;
	double sum_rec = 0, worst_rec = 0, sum_recal = 0;
	int n_rec = 0, n_never = 0, max_recal = 0;
	double sum_other = 0, worst_other = 0, sum_back = 0, worst_back = 0;
	double sum_both = 0, worst_both = 0, sum_err = 0, worst_err = 0;
	int failed = 0, n_ok = 0;
	double times[1000];

	for (int run = 0; run < runs; run++) {
		struct sim_ctrl c[2];
		memset(c, 0, sizeof(c));
		struct t_led_sync_refinement_options opts = {
		    .flags = T_LED_SYNC_REFINEMENT_FLAGS_OPTICAL_DRIVEN_OFFSET | T_LED_SYNC_REFINEMENT_FLAGS_HAS_LATENCY_CAP |
		             T_LED_SYNC_REFINEMENT_FLAGS_PS_SENSE | T_LED_SYNC_REFINEMENT_FLAGS_COUNT_CALIBRATION,
		    .initial_blink_duration_ns = 1500000,
		    .min_blink_duration_ns = 50000,
		    .max_blink_duration_ns = 1500000,
		    .time_to_resync_ns = T_LED_SYNC_DEFAULT_RESYNC_TIME,
		    .settle_frames = getenv("SIM_SETTLE") ? (uint32_t)atoi(getenv("SIM_SETTLE")) : 3,
		    .latency_cap_ns = 8000000,
		};
		for (int i = 0; i < 2; i++) {
			t_led_sync_refinement_init(&c[i].r, &opts);
			// Windows seen on hardware sit 0-2.5ms into the frame, a few hundred us to 2ms apart.
			c[i].window_left_ns = (urand() * 2.5e6) - 1.2e6;
			c[i].leds = 15 + (int)(urand() * 18);
			c[i].visible = true;
			c[i].vis_change_ns = g_now_ns + (int64_t)(1e9 * (2 + urand() * 4));
			c[i].hold_at_ns = -1;
		}
		int64_t start_ns = g_now_ns;
		uint64_t seq = 1000;
		int64_t exposure_start = g_now_ns;
		int64_t out_start = -1, out_end = -1, recovered = -1;
		int recal = 0, streak = 0, prev_phase = -1;
		int64_t ev_start = -1;
		double applied_shift = 0, bt_walk = 0, run_err_worst = 0;
		long ev_frames = 0, ev_tracked = 0;
		for (int i = 0; i < 2; i++)
			c[i].bt_ns = 3e6 + urand() * 1e6;
		const int64_t run_limit = (outage_for > 0 || shift_mode || acquire_s > 0 ? 220LL : 60LL) * 1000000000LL;
		while (g_now_ns - start_ns < run_limit) {
			// Advance to this frame's processing time, applying LED commands due before it.
			exposure_start += FRAME_NS;
			int64_t proc = exposure_start + PROC_DELAY_NS;
			if (outage_for > 0) {
				if (out_start < 0 && c[0].hold_at_ns >= 0 && c[1].hold_at_ns >= 0) {
					out_start = proc + 5LL * 1000000000LL;
				}
				if (out_start >= 0 && proc >= out_start && out_end < 0) {
					c[0].visible = proc >= out_start + (int64_t)(outage_for * 1e9);
					if (c[0].visible) {
						out_end = proc;
					}
				}
			}
			if (shift_mode) {
				if (ev_start < 0 && c[0].hold_at_ns >= 0 && c[1].hold_at_ns >= 0) {
					ev_start = proc + 40LL * 1000000000LL;
				}
				double t = ev_start >= 0 ? (proc - ev_start) / 1e9 : -1;
				// Real shift: LED window and Bluetooth offset together.
				double want = t < 0 ? 0 : (shift_s <= 0 ? shift_us : shift_us * fmin(1.0, t / shift_s));
				double step = (want - applied_shift) * 1000.0;
				applied_shift = want;
				c[0].window_left_ns += step;
				c[0].bt_ns += step;
				// Bluetooth-only error: ramp in, hold 10 s, gone.
				double err = 0;
				if (t >= 0 && bterr_us != 0) {
					double up = bterr_s <= 0 ? 1.0 : fmin(1.0, t / bterr_s);
					err = t < bterr_s + 10 ? bterr_us * up : 0;
				}
				// Noise: 30 us per report and a slow random walk (logs: +-30 us per 5 s while calm).
				bt_walk += 1.0 * (urand() - 0.5) * 3.46;
				for (int i = 0; i < 2; i++) {
					if (c[i].dropped)
						continue;
					double u1 = fmax(urand(), 1e-12), u2 = urand();
					double white = 30.0 * sqrt(-2 * log(u1)) * cos(2 * M_PI * u2);
					double v = c[i].bt_ns + (white + bt_walk + (i == 0 ? err : 0)) * 1000.0;
					t_led_sync_push_host_device_clock_offset(&c[i].r, (time_duration_ns)v);
				}
			}
			if (drop_at >= 0) {
				struct sim_ctrl *d = &c[0];
				if (!d->dropped && d->reconnect_ns == 0 && proc - start_ns >= (int64_t)(drop_at * 1e9)) {
					d->dropped = true; // the read thread ends: pssense_run_thread restarts the LED sync
					d->pending = false;
					d->on_mode = false;
					d->hold_at_ns = -1;
					t_led_sync_refinement_restart(&d->r);
				} else if (d->dropped && proc - start_ns >= (int64_t)((drop_at + drop_for) * 1e9)) {
					d->dropped = false; // pssense_reconnect: restart again, a fresh link
					d->reconnect_ns = proc;
					d->on_mode = true;
					t_led_sync_refinement_restart(&d->r);
				}
			}
			for (int i = 0; i < 2; i++) {
				struct sim_ctrl *s = &c[i];
				if (s->pending && s->apply_at_ns <= proc) {
					s->pending = false;
					s->on_mode = s->pending_sample.pssense_mode != T_LED_SYNC_SAMPLE_PSSENSE_MODE_LED_ALL_OFF;
					s->offset_ns = (double)s->pending_sample.fudge_offset_ns;
					// The controller blinks with the new settings from its next cycle.
					s->effective_from_ns = s->apply_at_ns + (int64_t)(urand() * g_led_latency_frames * FRAME_NS);
					g_now_ns = s->apply_at_ns;
					t_led_sync_mark_latest_sample_applied(&s->r, s->apply_at_ns);
				}
				// Window drift 15us/s, as measured.
				s->window_left_ns += g_drift * FRAME_NS; // SIM_DRIFT_PPM, 15 as measured
				s->bt_ns += g_drift * FRAME_NS;          // a real drift shows in the Bluetooth clock too
				if (dropout > 0 && proc >= s->vis_change_ns) {
					s->visible = !s->visible;
					double mean = s->visible ? 4.0 * (1 - dropout) / fmax(dropout, 0.01) : 4.0;
					if (mean > 30)
						mean = 30;
					s->vis_change_ns = proc + (int64_t)(1e9 * (0.3 + urand() * mean));
				}
			}
			g_now_ns = proc;
			// Background blobs (lamps, reflections): steady 2, or with SIM_AMBIENT_WALK a level wandering 0-120 as in
			// the 12:52 run (LEDs-off counts 25 -> 125 within seconds).
			if (g_ambient_walk) {
				g_ambient += (urand() - 0.5) * 2.0;
				if (urand() < 0.01)
					g_ambient += (urand() - 0.5) * 80.0;
				g_ambient = fmin(120.0, fmax(0.0, g_ambient));
			}
			// Background blobs are fixed lamps and reflections: their number follows the level, with a little noise.
			int vis[2];
			int count = g_ambient_walk ? (int)lround(g_ambient) + poisson(2.0) : poisson(2.0);
			// SIM_AMBIENT_FLICKER: reflections near the detection threshold coming and going from frame to frame
			// (12:59: total count sd 5.9 between consecutive frames).
			if (g_flicker > 0) {
				double u1 = fmax(urand(), 1e-12), u2 = urand();
				count += (int)lround(g_flicker * sqrt(-2 * log(u1)) * cos(2 * M_PI * u2));
				if (count < 0)
					count = 0;
			}
			for (int i = 0; i < 2; i++) {
				vis[i] = c[i].dropped ? 0 : visible_leds(&c[i], exposure_ns, exposure_start);
				count += vis[i];
			}

			struct t_timing_event_camera_exposure_start ev = {
			    .sequence_id = seq++,
			    .timestamp_ns = exposure_start,
			    .frame_period_ns = FRAME_NS,
			    .exposure_time_ns = 840000, // what the headset reports (a setting), whatever the real exposure
			    .blob_count = (uint32_t)count,
			    .has_blob_count = true,
			};
			struct t_timing_event te = {.type = T_TIMING_EVENT_TYPE_CAMERA_EXPOSURE_START};
			te.camera_exposure_start = ev;
			for (int i = 0; i < 2; i++) {
				struct sim_ctrl *s = &c[i];
				if (s->dropped) {
					continue; // no driver thread for it
				}
				t_led_sync_push_timing_event(&s->r, &te.camera_exposure_start);
				if (g_tracker) {
					// Last frame's tracker result arrives now.
					if (s->pending_match > 0) {
						struct t_constellation_tracker_sample smp2 = {0};
						smp2.sequence_id = s->pending_seq;
						smp2.timestamp_ns = exposure_start - FRAME_NS;
						smp2.has_world_pose = true;
						smp2.metrics.matched_blob_count = (uint32_t)s->pending_match;
						// 15% noise per picture (motion, angle), as analysis/dither_sim.py assumed
						smp2.average_brightness = (float)(s->pending_bright * (0.85 + 0.3 * urand()));
						t_led_sync_push_constellation_sample(&s->r, &smp2);
					}
					if (acquire_s > 0) {
						if (vis[i] >= 4) {
							s->dark_frames = 0;
							if (!s->acquired && urand() < 1.0 / (acquire_s * 60.0))
								s->acquired = true;
						} else if (++s->dark_frames > 30) {
							s->acquired = false;
						}
						s->pending_match = (s->acquired && vis[i] >= 4 && urand() < 0.7) ? vis[i] : 0;
						s->match_streak = (s->acquired && vis[i] >= 4) ? s->match_streak + 1 : 0;
						if (s->tracked_at_ns == 0 && s->match_streak >= 15)
							s->tracked_at_ns = proc - start_ns;
						const int ph = (int)s->r.phase;
						if (ph == T_LED_SYNC_PHASE_COUNT_CALIBRATE && s->prev_phase != ph)
							s->calibrations++;
						s->prev_phase = ph;
					} else {
						s->pending_match = (vis[i] >= 4 && urand() < 0.7) ? vis[i] : 0;
					}
					s->pending_seq = ev.sequence_id;
					s->pending_bright = led_brightness(s, exposure_ns, exposure_start);
				}
				struct t_led_sync_sample smp;
				if (t_led_sync_get_sample(&s->r, &smp)) {
					s->pending = true;
					s->pending_sample = smp;
					s->steps++;
					// Driver send wait (report cadence, haptics) + round trip ~10-30ms.
					s->apply_at_ns = proc + (int64_t)(4e6 + urand() * 8e6) + (int64_t)(8e6 + urand() * 20e6);
				}
				if (s->hold_at_ns < 0 && s->r.phase == T_LED_SYNC_PHASE_COUNT_HOLD &&
				    !(drop_at >= 0 && i == 0 && s->reconnect_ns == 0)) {
					s->hold_at_ns = proc - (s->reconnect_ns ? s->reconnect_ns : start_ns);
					double blink = (double)s->r.current_blink_duration_ns;
					double true_centre = s->window_left_ns + (blink + exposure_ns) / 2;
					double held = (double)s->r.count_calib.hold_centre_ns;
					double d = fmod(held - true_centre, (double)FRAME_NS);
					if (d > FRAME_NS / 2)
						d -= FRAME_NS;
					if (d < -FRAME_NS / 2)
						d += FRAME_NS;
					s->hold_centre_err_us = d / 1000.0;
				}
			}
			if (acquire_s > 0 && !shift_mode && outage_for <= 0) {
				if (c[0].tracked_at_ns > 0 && c[1].tracked_at_ns > 0)
					break;
				continue;
			}
			if (shift_mode && ev_start >= 0 && proc >= ev_start) {
				const int ph = (int)c[0].r.phase;
				if (ph == T_LED_SYNC_PHASE_COUNT_CALIBRATE && prev_phase != T_LED_SYNC_PHASE_COUNT_CALIBRATE &&
				    prev_phase >= 0)
					recal++;
				prev_phase = ph;
				ev_frames++;
				ev_tracked += vis[0] >= 4;
				if (ph == T_LED_SYNC_PHASE_COUNT_HOLD) {
					double blink = (double)c[0].r.current_blink_duration_ns;
					double tc = c[0].window_left_ns + (blink + exposure_ns) / 2;
					double d = fmod((double)c[0].r.count_calib.hold_centre_ns - tc, (double)FRAME_NS);
					if (d > FRAME_NS / 2)
						d -= FRAME_NS;
					if (d < -FRAME_NS / 2)
						d += FRAME_NS;
					run_err_worst = fmax(run_err_worst, fabs(d) / 1000.0);
				}
				if (proc - ev_start > 90LL * 1000000000LL)
					break;
				continue;
			}
			if (shift_mode) {
				prev_phase = (int)c[0].r.phase;
				continue;
			}
			if (outage_for > 0 && out_start >= 0 && proc >= out_start) {
				const int ph = (int)c[0].r.phase;
				if (ph == T_LED_SYNC_PHASE_COUNT_CALIBRATE && prev_phase != T_LED_SYNC_PHASE_COUNT_CALIBRATE &&
				    prev_phase >= 0) {
					recal++;
				}
				prev_phase = ph;
				if (out_end >= 0 && recovered < 0) {
					streak = (ph == T_LED_SYNC_PHASE_COUNT_HOLD && vis[0] >= 4) ? streak + 1 : 0;
					if (streak >= 15) {
						recovered = proc - 14 * FRAME_NS;
					}
				}
				if (recovered >= 0 || (out_end >= 0 && proc - out_end > 60LL * 1000000000LL)) {
					break;
				}
				continue;
			}
			if (outage_for > 0 && out_start >= 0) {
				prev_phase = (int)c[0].r.phase;
				continue;
			}
			if (outage_for <= 0 && c[0].hold_at_ns >= 0 && c[1].hold_at_ns >= 0)
				break;
			if (drop_at >= 0 && c[0].reconnect_ns == 0 && proc - start_ns > (int64_t)((drop_at + drop_for + 1) * 1e9))
				break; // can't happen: guard
		}
		if (acquire_s > 0 && !shift_mode && outage_for <= 0) {
			for (int i = 0; i < 2; i++) {
				if (c[i].tracked_at_ns > 0) {
					double t = c[i].tracked_at_ns / 1e9;
					acq_sum += t;
					acq_worst = fmax(acq_worst, t);
					acq_n++;
				} else {
					acq_never++;
				}
				acq_cal_sum += c[i].calibrations;
				acq_cal_max = c[i].calibrations > acq_cal_max ? c[i].calibrations : acq_cal_max;
			}
		}
		if (shift_mode && ev_frames > 0) {
			double f = (double)ev_tracked / ev_frames;
			sh_tracked += f;
			sh_worst_tracked = fmin(sh_worst_tracked, f);
			sh_err_worst = fmax(sh_err_worst, run_err_worst);
			sh_err_sum += run_err_worst;
			sh_recal += recal;
			sh_recal_max = recal > sh_recal_max ? recal : sh_recal_max;
			sh_jumps += c[0].r.count_calib.bt_jumps;
			sh_undone += c[0].r.count_calib.bt_jumps_undone;
			sh_runs++;
		}
		if (outage_for > 0 && out_end >= 0) {
			sum_recal += recal;
			max_recal = recal > max_recal ? recal : max_recal;
			if (recovered >= 0) {
				const double r = (recovered - out_end) / 1e9;
				sum_rec += r;
				worst_rec = fmax(worst_rec, r);
				n_rec++;
			} else {
				n_never++;
			}
		}
		if (c[0].hold_at_ns < 0 || c[1].hold_at_ns < 0) {
			failed++;
		} else {
			if (drop_at >= 0) {
				double other = c[1].hold_at_ns / 1e9, back = c[0].hold_at_ns / 1e9;
				sum_other += other;
				worst_other = fmax(worst_other, other);
				sum_back += back;
				worst_back = fmax(worst_back, back);
			}
			double both = fmax(c[0].hold_at_ns, c[1].hold_at_ns) / 1e9;
			times[n_ok++] = both;
			sum_both += both;
			worst_both = fmax(worst_both, both);
			for (int i = 0; i < 2; i++) {
				double e = fabs(c[i].hold_centre_err_us);
				sum_err += e;
				worst_err = fmax(worst_err, e);
			}
		}
		for (int i = 0; i < 2; i++) {
			count_calib_release_token(&c[i].r);
			t_led_sync_refinement_destroy(&c[i].r);
		}
		atomic_store(&count_calib_owner, NULL);
		atomic_store(&count_probe_wanted, NULL);
		g_now_ns += 10LL * 1000000000LL;
	}
	// median
	for (int i = 0; i < n_ok; i++)
		for (int j = i + 1; j < n_ok; j++)
			if (times[j] < times[i]) {
				double t = times[i];
				times[i] = times[j];
				times[j] = t;
			}
	printf("exposure %4.0fus dropout %.2f: both holding after median %.1fs, mean %.1fs, worst %.1fs; "
	       "held centre error mean %.0fus worst %.0fus (half room %.0fus); %d/%d never finished\n",
	       exposure_us, dropout, n_ok ? times[n_ok / 2] : 0, n_ok ? sum_both / n_ok : 0, worst_both,
	       n_ok ? sum_err / (2 * n_ok) : 0, worst_err, exposure_us / 2, failed, runs);
	if (acquire_s > 0 && (acq_n + acq_never) > 0) {
		printf("acquire mean %.0fs: controller tracked after mean %.1fs worst %.1fs, never within 220s %d of %d; "
		       "calibrations mean %.1f max %d\n",
		       acquire_s, acq_n ? acq_sum / acq_n : 0, acq_worst, acq_never, acq_n + acq_never,
		       acq_cal_sum / (acq_n + acq_never), acq_cal_max);
	}
	if (shift_mode && sh_runs > 0) {
		printf("shift %+.0fus over %.0fs, BT error %+.0fus over %.0fs: tracked %.1f%% (worst run %.1f%%), "
		       "recalibrations %.2f/run (max %d), worst centre error mean %.0fus max %.0fus, BT moves %.2f/run "
		       "(%d undone), %d runs\n",
		       shift_us, shift_s, bterr_us, bterr_s, 100 * sh_tracked / sh_runs, 100 * sh_worst_tracked,
		       (double)sh_recal / sh_runs, sh_recal_max, sh_err_sum / sh_runs, sh_err_worst,
		       (double)sh_jumps / sh_runs, sh_undone, sh_runs);
	}
	if (outage_for > 0) {
		printf("left out of view %.0fs after holding: tracked again after mean %.2fs worst %.2fs (%d runs), "
		       "never within 60s %d; recalibrations mean %.2f max %d\n",
		       outage_for, n_rec ? sum_rec / n_rec : 0, worst_rec, n_rec, n_never,
		       (n_rec + n_never) ? sum_recal / (n_rec + n_never) : 0, max_recal);
	}
	if (drop_at >= 0 && n_ok > 0) {
		printf("left drops at %.1fs for %.1fs: right holding after mean %.1fs worst %.1fs; left holding again "
		       "after reconnecting mean %.1fs worst %.1fs\n",
		       drop_at, drop_for, sum_other / n_ok, worst_other, sum_back / n_ok, worst_back);
	}
	return 0;
}
