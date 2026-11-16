// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Host <-> PS Sense controller clock tracking over Bluetooth.
 *
 * Every input report carries the controller's clock, received after a Bluetooth delay that is never negative but
 * often large (connection intervals, retransmissions, queueing). So the observed offset (host time - controller time)
 * is the true offset plus that delay, and its lower envelope is the true offset.
 *
 * This replaces m_clock_windowed_skew_tracker for PS Sense. That tracker wiped its whole window whenever one report
 * arrived over 100ms later than the previous one, then started over from the next report, delay included; on hardware
 * its estimate moved by 13-43ms within seconds, shifting the LED timing and the IMU timestamps used for fusion.
 *
 * Here late reports are simply never the minimum: the minimum offset is kept per 250ms bucket over the last 8s, a line
 * fitted through the bucket minima gives the clock skew and is shifted down to the lowest minimum, and the output moves
 * toward it slew limited, so it stays continuous. Only an offset far below the estimate (impossible unless the
 * controller's clock was reset) held for 20 reports resets it.
 *
 * @ingroup drv_pssense
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "util/u_time.h"

#include <stdbool.h>
#include <string.h>


#define PSSENSE_CLOCK_BUCKET_NS (U_TIME_1MS_IN_NS * 250)
#define PSSENSE_CLOCK_NUM_BUCKETS 32
//! An offset this far below the estimate means the controller clock jumped (reports can't arrive early).
#define PSSENSE_CLOCK_RESET_BELOW_NS (U_TIME_1MS_IN_NS * 20)
#define PSSENSE_CLOCK_RESET_COUNT 20
//! Output slew per report (~133Hz, so ~650us/s), above real clock skew (~90ppm measured), below delay noise.
#define PSSENSE_CLOCK_SLEW_NS (U_TIME_1US_IN_NS * 5)
//! Reports before the estimate is used.
#define PSSENSE_CLOCK_MIN_SAMPLES 20

struct pssense_clock_bucket
{
	bool used;
	timepoint_ns local_ns;       //!< Host time of the minimum.
	time_duration_ns min_offset; //!< Minimum (host - controller) in the bucket.
};

struct pssense_clock
{
	struct pssense_clock_bucket buckets[PSSENSE_CLOCK_NUM_BUCKETS];
	int cur;
	timepoint_ns cur_start_ns;
	uint32_t samples;

	//! Fitted lower envelope: offset(local) = a + b * (local - ref), b in ns per second.
	double a, b;
	timepoint_ns ref_ns;
	//! Recompute the slope, set when a bucket completes.
	bool refit_slope;

	//! What callers get, slew limited toward the fit.
	bool have_output;
	time_duration_ns output_offset;

	uint32_t below_count;

	//! Logged every few seconds.
	struct
	{
		uint32_t resets;
		time_duration_ns max_delay_ns;
		time_duration_ns output_at_log;
		timepoint_ns last_log_ns;
	} stats;
};

static inline void
pssense_clock_reset(struct pssense_clock *c)
{
	uint32_t resets = c->stats.resets + 1;
	memset(c, 0, sizeof(*c));
	c->stats.resets = resets;
}

static inline void
pssense_clock_fit(struct pssense_clock *c, timepoint_ns now_ns)
{
	double sx = 0, sy = 0, sxx = 0, sxy = 0;
	int n = 0;
	c->ref_ns = now_ns;
	for (int i = 0; i < PSSENSE_CLOCK_NUM_BUCKETS; i++) {
		const struct pssense_clock_bucket *bk = &c->buckets[i];
		if (!bk->used) {
			continue;
		}
		double x = (double)(bk->local_ns - c->ref_ns) / U_TIME_1S_IN_NS;
		double y = (double)bk->min_offset;
		sx += x;
		sy += y;
		sxx += x * x;
		sxy += x * y;
		n++;
	}
	if (n == 0) {
		return;
	}

	(void)sxx;
	(void)sxy;

	/*
	 * Skew by Theil-Sen (median of the pairwise slopes between bucket minima): a least squares fit was thrown
	 * off by buckets that only held delayed reports (Bluetooth stalls of seconds on hardware), got clamped,
	 * and left the skew at 0 while the controller ran ~90ppm off the host clock.
	 */
	double b = c->b;
	if (n >= 8 && c->refit_slope) {
		double slopes[PSSENSE_CLOCK_NUM_BUCKETS * PSSENSE_CLOCK_NUM_BUCKETS / 2];
		int ns = 0;
		for (int i = 0; i < PSSENSE_CLOCK_NUM_BUCKETS; i++) {
			const struct pssense_clock_bucket *bi = &c->buckets[i];
			if (!bi->used) {
				continue;
			}
			for (int j = i + 1; j < PSSENSE_CLOCK_NUM_BUCKETS; j++) {
				const struct pssense_clock_bucket *bj = &c->buckets[j];
				if (!bj->used) {
					continue;
				}
				double dx = (double)(bj->local_ns - bi->local_ns) / U_TIME_1S_IN_NS;
				// Only pairs at least a second apart, closer ones are dominated by delay noise.
				if (dx < 1.0 && dx > -1.0) {
					continue;
				}
				slopes[ns++] = (double)(bj->min_offset - bi->min_offset) / dx;
			}
		}
		if (ns >= 3) {
			// Partial selection sort up to the median, ns is at most a few hundred.
			int mid = ns / 2;
			for (int i = 0; i <= mid; i++) {
				int m = i;
				for (int k = i + 1; k < ns; k++) {
					if (slopes[k] < slopes[m]) {
						m = k;
					}
				}
				double t = slopes[i];
				slopes[i] = slopes[m];
				slopes[m] = t;
			}
			b = slopes[mid];
			if (b > 1000000.0 || b < -1000000.0) { // 1000ppm, not a crystal
				b = 0;
			}
		}
		c->refit_slope = false;
	}
	double a = (sy - b * sx) / n;

	// Shift the line down to the lowest bucket minimum: the true offset is at or below every observation.
	double lowest = 0;
	bool first = true;
	for (int i = 0; i < PSSENSE_CLOCK_NUM_BUCKETS; i++) {
		const struct pssense_clock_bucket *bk = &c->buckets[i];
		if (!bk->used) {
			continue;
		}
		double x = (double)(bk->local_ns - c->ref_ns) / U_TIME_1S_IN_NS;
		double r = (double)bk->min_offset - (a + b * x);
		if (first || r < lowest) {
			lowest = r;
			first = false;
		}
	}
	c->a = a + lowest;
	c->b = b;
}

static inline void
pssense_clock_push(struct pssense_clock *c, timepoint_ns local_ns, timepoint_ns remote_ns)
{
	time_duration_ns obs = local_ns - remote_ns;

	if (c->have_output) {
		if (obs < c->output_offset - PSSENSE_CLOCK_RESET_BELOW_NS) {
			if (++c->below_count >= PSSENSE_CLOCK_RESET_COUNT) {
				pssense_clock_reset(c);
			}
			return;
		}
		c->below_count = 0;

		time_duration_ns delay = obs - c->output_offset;
		if (delay > c->stats.max_delay_ns) {
			c->stats.max_delay_ns = delay;
		}
	}

	if (c->samples == 0 || local_ns - c->cur_start_ns >= PSSENSE_CLOCK_BUCKET_NS) {
		if (c->samples != 0) {
			c->cur = (c->cur + 1) % PSSENSE_CLOCK_NUM_BUCKETS;
			c->refit_slope = true;
		}
		c->buckets[c->cur].used = false;
		c->cur_start_ns = local_ns;
	}

	struct pssense_clock_bucket *bk = &c->buckets[c->cur];
	if (!bk->used || obs < bk->min_offset) {
		bk->used = true;
		bk->min_offset = obs;
		bk->local_ns = local_ns;
	}
	c->samples++;

	pssense_clock_fit(c, local_ns);
	time_duration_ns target = (time_duration_ns)c->a; // at ref = now

	if (!c->have_output) {
		c->output_offset = target;
		c->have_output = true;
		c->stats.output_at_log = target;
	} else {
		time_duration_ns d = target - c->output_offset;
		if (d > PSSENSE_CLOCK_SLEW_NS) {
			d = PSSENSE_CLOCK_SLEW_NS;
		} else if (d < -PSSENSE_CLOCK_SLEW_NS) {
			d = -PSSENSE_CLOCK_SLEW_NS;
		}
		c->output_offset += d;
	}
}

static inline bool
pssense_clock_ready(const struct pssense_clock *c)
{
	return c->have_output && c->samples >= PSSENSE_CLOCK_MIN_SAMPLES;
}

//! Host -> controller.
static inline bool
pssense_clock_to_remote(const struct pssense_clock *c, timepoint_ns local_ns, timepoint_ns *out_remote_ns)
{
	if (!pssense_clock_ready(c)) {
		return false;
	}
	*out_remote_ns = local_ns - c->output_offset;
	return true;
}

//! Controller -> host.
static inline bool
pssense_clock_to_local(const struct pssense_clock *c, timepoint_ns remote_ns, timepoint_ns *out_local_ns)
{
	if (!pssense_clock_ready(c)) {
		return false;
	}
	*out_local_ns = remote_ns + c->output_offset;
	return true;
}

//! The host - controller offset, same sign as m_clock_windowed_skew_tracker_get_skew.
static inline bool
pssense_clock_get_offset(const struct pssense_clock *c, time_duration_ns *out_offset_ns)
{
	if (!pssense_clock_ready(c)) {
		return false;
	}
	*out_offset_ns = c->output_offset;
	return true;
}
