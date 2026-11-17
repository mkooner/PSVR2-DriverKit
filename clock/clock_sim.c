// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//
// Host <-> PS Sense controller clock estimator simulation: the driver's pssense_clock.h against candidates, on
// synthetic Bluetooth timing with the characteristics measured on hardware (19:34 run logs): ~133 reports/s, a true
// clock skew of tens of ppm, one-way delay of a few ms floor plus connection-interval jitter, retransmissions and
// stalls up to ~27 ms, and (optionally) a slowly drifting delay floor from the report / connection-event phase.
//
// Score per estimator, after a settle period: how fast the output offset (host - controller, as LED sync uses it)
// wanders against the truth minus its average bias (us/s over 5 s windows, p50/p95/max), peak-to-peak error over the
// run, time to settle within 100 us of the final bias, and behaviour across a controller clock jump.
//
// Build: gcc -O2 -I<trial>/src/xrt/include -I<trial>/build/src/xrt/include -I<trial>/src/xrt/auxiliary
//        -I<trial>/src/xrt/drivers/pssense clock_sim.c -lm -o clock_sim

#include "pssense_clock.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Candidate "long": the driver's pssense_clock_long.h (same code as PSSENSE_CLOCK_LONG).
#include "pssense_clock_long.h"
#define clock_long pssense_clock_long
#define long_push pssense_clock_long_push

static int
cmp_double(const void *a, const void *b)
{
	double x = *(const double *)a, y = *(const double *)b;
	return x < y ? -1 : x > y;
}

/*
 * Candidate "decay": PSVR2 custom-controller-sync style. The estimate is the highest (controller - host) seen,
 * i.e. the lowest delay, decaying by a fixed rate so it can follow skew, output slew limited.
 */
struct clock_decay
{
	bool have;
	double env; // lowest (host - controller) envelope
	int64_t last;
	double out;
};

static void
decay_push(struct clock_decay *c, int64_t local, int64_t remote)
{
	double obs = (double)(local - remote);
	if (!c->have) {
		c->env = obs;
		c->out = obs;
		c->have = true;
		c->last = local;
		return;
	}
	double dt = (double)(local - c->last);
	c->last = local;
	c->env += dt * 5e-5; // envelope rises 50 ppm until a lower-delay report pulls it down
	if (obs < c->env)
		c->env = obs;
	double d = c->env - c->out;
	if (d > 2500)
		d = 2500;
	if (d < -2500)
		d = -2500;
	c->out += d;
}

/*
 * Simulation.
 */

static double
urand(unsigned *s)
{
	*s = *s * 1103515245u + 12345u;
	return ((*s >> 8) & 0xffffff) / (double)0x1000000;
}

struct scenario
{
	const char *name;
	double skew_ppm;
	double floor_ms;
	double floor_wander_ms;    // amplitude of the slow floor wander
	double floor_wander_s;     // its period
	double retransmit_p;       // probability of +1..3 connection intervals
	double stall_every_s;      // a stall (all reports held) of stall_ms every so often, 0 off
	double stall_ms;
	double clock_jump_at_s;    // controller clock jumps by -100 ms at this time, 0 off
};

#define DURATION_S 120.0
#define SETTLE_S 15.0

struct metrics
{
	double wander_p50, wander_p95, wander_max, p2p, settle_s;
};

static int
cmp_d(const void *a, const void *b)
{
	return cmp_double(a, b);
}

static void
score(const double *t, const double *err, int n, struct metrics *m)
{
	// bias: mean error after settle
	double sum = 0;
	int k = 0;
	for (int i = 0; i < n; i++)
		if (t[i] > SETTLE_S) {
			sum += err[i];
			k++;
		}
	double bias = k ? sum / k : 0;
	double mn = 1e18, mx = -1e18;
	for (int i = 0; i < n; i++)
		if (t[i] > SETTLE_S) {
			mn = fmin(mn, err[i] - bias);
			mx = fmax(mx, err[i] - bias);
		}
	m->p2p = (mx - mn) / 1000.0;
	// settle: last time |err - bias| > 100 us
	m->settle_s = 0;
	for (int i = 0; i < n; i++)
		if (fabs(err[i] - bias) > 100000.0)
			m->settle_s = t[i];
	// wander: |err(t+5)-err(t)|/5 over windows after settle
	static double w[4096];
	int nw = 0, j = 0;
	for (int i = 0; i < n && nw < 4096; i++) {
		if (t[i] < SETTLE_S)
			continue;
		while (j < n && t[j] < t[i] + 5.0)
			j++;
		if (j >= n)
			break;
		w[nw++] = fabs(err[j] - err[i]) / 5.0 / 1000.0;
		i += 600; // ~ every 4.5 s
	}
	qsort(w, nw, sizeof(double), cmp_d);
	m->wander_p50 = nw ? w[nw / 2] : 0;
	m->wander_p95 = nw ? w[(int)(nw * 0.95)] : 0;
	m->wander_max = nw ? w[nw - 1] : 0;
}

//! Real data: '#C,host_rx_ns,controller_ns' lines of a PSSENSE_RECORD_FILE recording. No truth there, but the true
//! offset is a straight line (constant skew), so each estimate's distance from its own straight-line fit is its wander.
static int
replay_file(const char *path)
{
	FILE *f = fopen(path, "r");
	if (!f) {
		perror(path);
		return 1;
	}
	static struct pssense_clock cur;
	static struct pssense_clock_long lng;
	memset(&cur, 0, sizeof(cur));
	memset(&lng, 0, sizeof(lng));
	int cap = 1 << 20, n = 0;
	double *t = malloc(sizeof(double) * cap), *e0 = malloc(sizeof(double) * cap), *e1 = malloc(sizeof(double) * cap);
	char line[256];
	long long t0 = 0;
	while (fgets(line, sizeof(line), f) && n < cap) {
		long long local, remote;
		if (sscanf(line, "#C,%lld,%lld", &local, &remote) != 2)
			continue;
		pssense_clock_push(&cur, local, remote);
		pssense_clock_long_push(&lng, local, remote);
		if (!pssense_clock_ready(&cur) || !lng.have_out)
			continue;
		if (!t0)
			t0 = local;
		t[n] = (local - t0) / 1e9;
		e0[n] = (double)cur.output_offset;
		e1[n] = (double)lng.out;
		n++;
	}
	fclose(f);
	if (getenv("SIM_MAP")) {
		// LED mapping advanced at a smoothed long-window skew vs held still, against the session's straight-line
		// drift of the bucket floor (the true rate over the run).
		FILE *g = fopen(path, "r");
		static struct pssense_clock_long l2;
		memset(&l2, 0, sizeof(l2));
		double ema = 0, map_ema = 0, t_prev = -1, t_start = -1;
		int have = 0;
		static double ts[1 << 20], me[1 << 20], fl[1 << 20];
		int m = 0;
		long long t00 = 0;
		while (fgets(line, sizeof(line), g) && m < (1 << 20)) {
			long long local, remote;
			if (sscanf(line, "#C,%lld,%lld", &local, &remote) != 2)
				continue;
			pssense_clock_long_push(&l2, local, remote);
			if (!t00)
				t00 = local;
			double tt = (local - t00) / 1e9;
			if (!l2.have_fit)
				continue;
			if (!have) {
				ema = l2.skew;
				have = 1;
				t_start = tt;
				t_prev = tt;
			}
			double dt = tt - t_prev;
			t_prev = tt;
			ema += (l2.skew - ema) * fmin(1.0, dt / 60.0);
			map_ema += ema * dt; // ns of (host - controller) the mapping has followed
			ts[m] = tt;
			me[m] = map_ema;
			fl[m] = (double)(local - remote);
			m++;
		}
		fclose(g);
		// true drift: straight line through the floor (1st percentile per 5 s)
		double sx = 0, sy = 0, sxx = 0, sxy = 0;
		int k = 0;
		for (double w = t_start; w < ts[m - 1] - 5; w += 5) {
			double mn = 1e30;
			for (int i = 0; i < m; i++)
				if (ts[i] >= w && ts[i] < w + 5 && fl[i] < mn)
					mn = fl[i];
			sx += w;
			sy += mn;
			sxx += w * w;
			sxy += w * mn;
			k++;
		}
		double slope = (k * sxy - sx * sy) / (k * sxx - sx * sx);
		double max_ema = 0, max_zero = 0;
		for (int i = 0; i < m; i++) {
			double truth = slope * (ts[i] - t_start);
			max_ema = fmax(max_ema, fabs(me[i] - truth));
			max_zero = fmax(max_zero, fabs(0 - truth));
		}
		printf("true drift %.1f ppm over %.0f s; LED mapping error vs it: held still max %.0f us, smoothed long skew max "
		       "%.0f us\n",
		       slope / 1000.0, ts[m - 1] - t_start, max_zero / 1000.0, max_ema / 1000.0);
		return 0;
	}
	if (n < 100) {
		fprintf(stderr, "%s: only %d clock samples\n", path, n);
		return 1;
	}
	const char *names[2] = {"current", "long"};
	double *e[2] = {e0, e1};
	for (int k = 0; k < 2; k++) {
		// straight line through the settled part, then score the residual like the simulation
		double sx = 0, sy = 0, sxx = 0, sxy = 0;
		int m = 0;
		for (int i = 0; i < n; i++)
			if (t[i] > SETTLE_S) {
				sx += t[i];
				sy += e[k][i];
				sxx += t[i] * t[i];
				sxy += t[i] * e[k][i];
				m++;
			}
		double slope = (m * sxy - sx * sy) / (m * sxx - sx * sx), icpt = (sy - slope * sx) / m;
		for (int i = 0; i < n; i++)
			e[k][i] -= icpt + slope * t[i];
		struct metrics mt;
		score(t, e[k], n, &mt);
		printf("%-8s skew %+.1f ppm; wander us/s p50/p95/max %.1f/%.1f/%.1f, p2p %.0f us\n", names[k],
		       slope / 1000.0, mt.wander_p50, mt.wander_p95, mt.wander_max, mt.p2p);
	}
	return 0;
}

int
main(int argc, char **argv)
{
	if (argc > 1)
		return replay_file(argv[1]);

	const struct scenario scen[] = {
	    {"plain: 30 ppm, 3 ms floor, jitter", 30, 3, 0, 1, 0.05, 0, 0, 0},
	    {"floor wander 3 ms / 60 s", 30, 3, 3, 60, 0.05, 0, 0, 0},
	    {"floor wander 1 ms / 15 s", -40, 3, 1, 15, 0.05, 0, 0, 0},
	    {"stalls 25 ms every 3 s", 30, 3, 0.5, 40, 0.08, 3, 25, 0},
	    {"clock jump at 60 s", 30, 3, 0.5, 40, 0.05, 0, 0, 60},
	};
	printf("%-36s %-8s %25s %9s %9s\n", "scenario", "est", "wander us/s p50/p95/max", "p2p us", "settle s");
	for (size_t si = 0; si < sizeof(scen) / sizeof(scen[0]); si++) {
		const struct scenario *sc = &scen[si];
		static struct pssense_clock cur;
		static struct clock_long lng;
		static struct clock_decay dec;
		memset(&cur, 0, sizeof(cur));
		memset(&lng, 0, sizeof(lng));
		memset(&dec, 0, sizeof(dec));
		const int n_max = (int)(DURATION_S * 140);
		double *t = malloc(sizeof(double) * n_max), *e[3];
		for (int k = 0; k < 3; k++)
			e[k] = malloc(sizeof(double) * n_max);
		int n = 0;
		unsigned seed = 1234 + (unsigned)si;
		const int64_t host0 = 1000000000000LL;
		const double report_ns = 7.5e6, conn_ns = 7.5e6;
		double stall_until = -1, last_arrive = 0;
		for (double ct = 0; ct < DURATION_S * 1e9; ct += report_ns) {
			// controller time ct; host time of generation = ct*(1+skew) + host0 (true offset grows with skew)
			double true_off = host0 + ct * sc->skew_ppm * 1e-6;
			if (sc->clock_jump_at_s > 0 && ct > sc->clock_jump_at_s * 1e9)
				true_off += 100e6; // controller clock went back 100 ms: host - controller jumps up
			double gen_host = ct + true_off;
			double floor =
			    sc->floor_ms * 1e6 + sc->floor_wander_ms * 1e6 * 0.5 *
			                             (1 + sin(2 * M_PI * gen_host / 1e9 / sc->floor_wander_s));
			double delay = floor + urand(&seed) * conn_ns;
			if (urand(&seed) < sc->retransmit_p)
				delay += conn_ns * (1 + (int)(urand(&seed) * 3));
			double secs = ct / 1e9;
			if (sc->stall_every_s > 0 && fmod(secs, sc->stall_every_s) < report_ns / 1e9)
				stall_until = gen_host + sc->stall_ms * 1e6;
			double arrive = gen_host + delay;
			if (arrive < stall_until)
				arrive = stall_until + urand(&seed) * 1e6;
			// Bluetooth delivers in order.
			if (arrive < last_arrive)
				arrive = last_arrive + 1000;
			last_arrive = arrive;
			int64_t local = (int64_t)arrive, remote = (int64_t)ct;
			pssense_clock_push(&cur, local, remote);
			long_push(&lng, local, remote);
			decay_push(&dec, local, remote);
			if (getenv("SIM_DEBUG") && si == 0 && ((int)(ct / report_ns) % 4000) == 0)
				printf("t %.1f: long skew %.0f ns/s (truth %.0f), level-at-now - truth %.0f us, out - truth %.0f us, "
				       "have_fit %d\n",
				       ct / 1e9, lng.skew, sc->skew_ppm * 1e3,
				       (lng.level + lng.skew * (double)(local - lng.ref_ns) / 1e9 - true_off) / 1e3,
				       ((double)lng.out - true_off) / 1e3, lng.have_fit);
			if (!pssense_clock_ready(&cur) || !lng.have_out)
				continue;
			// error of each estimate of (host - controller) at this host time, vs truth
			double truth = true_off;
			t[n] = secs;
			e[0][n] = (double)cur.output_offset - truth;
			e[1][n] = (double)lng.out - truth;
			e[2][n] = dec.out - truth;
			n++;
		}
		const char *names[3] = {"current", "long", "decay"};
		for (int k = 0; k < 3; k++) {
			struct metrics m;
			if (sc->clock_jump_at_s > 0) {
				// score only after the jump settled region separately: report the max error in the 10 s after
				double worst = 0;
				int i0 = -1;
				for (int i = 0; i < n; i++)
					if (t[i] > sc->clock_jump_at_s && i0 < 0)
						i0 = i;
				// settle time after the jump to within 1 ms of the new truth
				double settle = 0;
				for (int i = i0 >= 0 ? i0 : n; i < n; i++)
					if (fabs(e[k][i]) > 10e6)
						settle = t[i] - sc->clock_jump_at_s;
				for (int i = i0 >= 0 ? i0 : n; i < n && t[i] < sc->clock_jump_at_s + 10; i++)
					worst = fmax(worst, fabs(e[k][i]));
				score(t, e[k], i0 > 0 ? i0 : n, &m);
				printf("%-36s %-8s %8.1f %6.1f %8.1f %9.0f %9.1f   after jump: back within 10 ms in %.1f s\n",
				       si == 0 || k ? (k ? "" : sc->name) : sc->name, names[k], m.wander_p50, m.wander_p95,
				       m.wander_max, m.p2p, m.settle_s, settle);
			} else {
				score(t, e[k], n, &m);
				printf("%-36s %-8s %8.1f %6.1f %8.1f %9.0f %9.1f\n", k ? "" : sc->name, names[k],
				       m.wander_p50, m.wander_p95, m.wander_max, m.p2p, m.settle_s);
			}
		}
		free(t);
		for (int k = 0; k < 3; k++)
			free(e[k]);
	}
	return 0;
}
