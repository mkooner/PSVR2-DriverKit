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
	double hold_centre_err_us;
	uint32_t steps;
};

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
	g_flicker = getenv("SIM_AMBIENT_FLICKER") ? atof(getenv("SIM_AMBIENT_FLICKER")) : 0.0;
	if (getenv("SIM_LED_LATENCY_FRAMES"))
		g_led_latency_frames = atof(getenv("SIM_LED_LATENCY_FRAMES"));

	const double drop_at = getenv("SIM_DROP_AT") ? atof(getenv("SIM_DROP_AT")) : -1;
	const double drop_for = getenv("SIM_DROP_FOR") ? atof(getenv("SIM_DROP_FOR")) : 3;
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
		while (g_now_ns - start_ns < 60LL * 1000000000LL) {
			// Advance to this frame's processing time, applying LED commands due before it.
			exposure_start += FRAME_NS;
			int64_t proc = exposure_start + PROC_DELAY_NS;
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
				s->window_left_ns += 15e-6 * FRAME_NS;
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
						t_led_sync_push_constellation_sample(&s->r, &smp2);
					}
					s->pending_match = (vis[i] >= 4 && urand() < 0.7) ? vis[i] : 0;
					s->pending_seq = ev.sequence_id;
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
			if (c[0].hold_at_ns >= 0 && c[1].hold_at_ns >= 0)
				break;
			if (drop_at >= 0 && c[0].reconnect_ns == 0 && proc - start_ns > (int64_t)((drop_at + drop_for + 1) * 1e9))
				break; // can't happen: guard
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
	if (drop_at >= 0 && n_ok > 0) {
		printf("left drops at %.1fs for %.1fs: right holding after mean %.1fs worst %.1fs; left holding again "
		       "after reconnecting mean %.1fs worst %.1fs\n",
		       drop_at, drop_for, sum_other / n_ok, worst_other, sum_back / n_ok, worst_back);
	}
	return 0;
}
