// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Long-window host <-> PS Sense controller clock estimate (PSSENSE_CLOCK_LONG), see pssense_clock.h.
 *
 * The same 250 ms bucket minima of (host - controller) as pssense_clock.h, but kept for 60 s: the skew is a
 * Theil-Sen fit over pairs of buckets at least 2 s apart, the level the 10th percentile of the minima around that line
 * (the delay floor, robust to a few lucky reports), both refitted once a second and extrapolated in between, and the
 * output slew limited like pssense_clock.h. pssense_clock.h's 8 s median moves with every few seconds of delay noise:
 * on hardware its output wandered 10-90 us/s with alternating sign after settling (19:34 run), which walks the LED
 * blink out of the ~+-500 us of room in the camera's visible window within seconds. On synthetic Bluetooth timing
 * with the measured delays (psvr2-test/clock/clock_sim.c), its output wandered 16/37/43 us/s (p50/p95/max) and this
 * 3/6/8, settling in 11 s.
 *
 * @ingroup drv_pssense
 */

#pragma once

#include "util/u_time.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define PSSENSE_CLOCK_LONG_BUCKETS 240 // x 250 ms = 60 s
#define PSSENSE_CLOCK_LONG_BUCKET_NS (U_TIME_1MS_IN_NS * 250)
//! Buckets of history before the fit is trusted (4 s).
#define PSSENSE_CLOCK_LONG_MIN_BUCKETS 16
//! Pairs closer than this are left out of the skew fit, their slope is mostly delay noise.
#define PSSENSE_CLOCK_LONG_MIN_PAIR_S 2.0
#define PSSENSE_CLOCK_LONG_MAX_PAIRS (PSSENSE_CLOCK_LONG_BUCKETS * 8)
//! Output slew per report (as pssense_clock.h).
#define PSSENSE_CLOCK_LONG_SLEW_NS (U_TIME_1US_IN_NS * 5)
//! An observation this far below the output means the controller clock jumped; this many in a row reset.
#define PSSENSE_CLOCK_LONG_RESET_BELOW_NS (U_TIME_1MS_IN_NS * 20)
#define PSSENSE_CLOCK_LONG_RESET_COUNT 20

struct pssense_clock_long
{
	struct
	{
		bool used;
		int64_t local_ns;
		int64_t min_offset;
	} b[PSSENSE_CLOCK_LONG_BUCKETS];
	int cur;
	int64_t cur_start_ns;
	uint32_t samples;
	double skew;  //!< ns per s
	double level; //!< (host - controller) floor at ref_ns
	int64_t ref_ns;
	bool have_fit, have_out;
	int64_t out;
	int refit_countdown;
	uint32_t below;
};

static inline int
pssense_clock_long_cmp_double(const void *a, const void *b)
{
	double x = *(const double *)a, y = *(const double *)b;
	return x < y ? -1 : x > y;
}

static inline void
pssense_clock_long_fit(struct pssense_clock_long *c, int64_t now)
{
	double xs[PSSENSE_CLOCK_LONG_BUCKETS], ys[PSSENSE_CLOCK_LONG_BUCKETS];
	int n = 0;
	for (int i = 0; i < PSSENSE_CLOCK_LONG_BUCKETS; i++) {
		if (c->b[i].used) {
			xs[n] = (double)(c->b[i].local_ns - now) / U_TIME_1S_IN_NS;
			ys[n] = (double)c->b[i].min_offset;
			n++;
		}
	}
	if (n == 0) {
		return;
	}
	if (n == 1) {
		c->level = ys[0];
		c->ref_ns = now;
		return;
	}
	// Theil-Sen on a strided subset of the pairs (bounded work).
	double slopes[PSSENSE_CLOCK_LONG_MAX_PAIRS]; // 15 KB of stack; the controllers' threads each have their own
	int ns = 0;
	for (int i = 0; i < n && ns < PSSENSE_CLOCK_LONG_MAX_PAIRS; i++) {
		for (int j = i + 1; j < n && ns < PSSENSE_CLOCK_LONG_MAX_PAIRS; j += 1 + n / 16) {
			double dx = xs[j] - xs[i];
			if (fabs(dx) < PSSENSE_CLOCK_LONG_MIN_PAIR_S) {
				continue;
			}
			slopes[ns++] = (ys[j] - ys[i]) / dx;
		}
	}
	double skew = c->skew;
	if (ns >= 8) {
		qsort(slopes, ns, sizeof(double), pssense_clock_long_cmp_double);
		skew = slopes[ns / 2];
		if (fabs(skew) > 1e6) { // 1000 ppm, not a crystal
			skew = 0;
		}
	}
	double r[PSSENSE_CLOCK_LONG_BUCKETS];
	for (int i = 0; i < n; i++) {
		r[i] = ys[i] - skew * xs[i];
	}
	qsort(r, n, sizeof(double), pssense_clock_long_cmp_double);
	c->skew = skew;
	c->level = r[n / 10];
	c->ref_ns = now;
	c->have_fit = n >= PSSENSE_CLOCK_LONG_MIN_BUCKETS;
}

static inline void
pssense_clock_long_push(struct pssense_clock_long *c, int64_t local_ns, int64_t remote_ns)
{
	int64_t obs = local_ns - remote_ns;
	if (c->have_out && obs < c->out - PSSENSE_CLOCK_LONG_RESET_BELOW_NS) {
		if (++c->below >= PSSENSE_CLOCK_LONG_RESET_COUNT) {
			memset(c, 0, sizeof(*c));
		} else {
			return;
		}
	} else {
		c->below = 0;
	}
	if (c->samples == 0 || local_ns - c->cur_start_ns >= PSSENSE_CLOCK_LONG_BUCKET_NS) {
		if (c->samples != 0) {
			c->cur = (c->cur + 1) % PSSENSE_CLOCK_LONG_BUCKETS;
			if (--c->refit_countdown <= 0) {
				pssense_clock_long_fit(c, local_ns);
				c->refit_countdown = 4; // once a second
			}
		}
		c->b[c->cur].used = false;
		c->cur_start_ns = local_ns;
	}
	if (!c->b[c->cur].used || obs < c->b[c->cur].min_offset) {
		c->b[c->cur].used = true;
		c->b[c->cur].min_offset = obs;
		c->b[c->cur].local_ns = local_ns;
	}
	c->samples++;
	if (!c->have_fit) {
		pssense_clock_long_fit(c, local_ns);
	}
	double target = c->level + c->skew * (double)(local_ns - c->ref_ns) / U_TIME_1S_IN_NS;
	if (!c->have_out) {
		c->out = (int64_t)target;
		c->have_out = true;
	} else {
		double d = target - (double)c->out;
		if (d > PSSENSE_CLOCK_LONG_SLEW_NS) {
			d = PSSENSE_CLOCK_LONG_SLEW_NS;
		} else if (d < -PSSENSE_CLOCK_LONG_SLEW_NS) {
			d = -PSSENSE_CLOCK_LONG_SLEW_NS;
		}
		c->out += (int64_t)d;
	}
}
