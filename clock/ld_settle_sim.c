// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*
 * Replays a PS VR2 record file (PSVR2_RECORD) through the headset driver's camera clock path: the IMU records feed
 * m_clock_windowed_skew_tracker exactly as psvr2.c does, and at every LED detector frame (L lines, in file order) the
 * offset/settle logic of led_xfer_cb runs. Prints how the tracker's target offset moves and when each settle rule
 * would let camera frames through.
 *
 *   ld_settle_sim <record.csv>
 */
#include "math/m_clock_tracking.h"
#include "util/u_time.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

int
main(int argc, char **argv)
{
	FILE *f = fopen(argv[1], "r");
	if (!f) {
		perror("open");
		return 1;
	}
	struct m_clock_windowed_skew_tracker *clk = m_clock_windowed_skew_tracker_alloc(4096);
	char line[1024];
	int64_t last_vts = 0, last_host = 0, first_ld_host = -1;
	bool valid = false;
	int64_t offset = 0, valid_since = 0, steady_since = 0;
	long frames = 0, relatches = 0;
	int64_t settled_rule_ns = -1;
	// Track the target offset per LD frame for the "final" reference.
	static int64_t tgt[200000], hostv[200000];
	long n = 0;
	// LD_SIM_SKIP_MS: don't feed the clock tracker the first ms of the IMU stream (startup backlog).
	int64_t skip_ns = getenv("LD_SIM_SKIP_MS") ? (int64_t)(atof(getenv("LD_SIM_SKIP_MS")) * 1e6) : 0;
	// LD_SIM_STEADY_MS: how long the offset must go without a re-latch (driver: 1000).
	int64_t steady_ns = getenv("LD_SIM_STEADY_MS") ? (int64_t)(atof(getenv("LD_SIM_STEADY_MS")) * 1e6) : 1000000000LL;
	int64_t first_imu_host = -1;
	while (fgets(line, sizeof(line), f)) {
		if (line[0] == 'I' && line[1] == ',') {
			long long host, vts;
			if (sscanf(line + 2, "%lld,%lld", &host, &vts) == 2) {
				if (first_imu_host < 0)
					first_imu_host = host;
				if (host - first_imu_host >= skip_ns)
					m_clock_windowed_skew_tracker_push(clk, (timepoint_ns)host, (timepoint_ns)vts);
				last_vts = vts;
				last_host = host;
			}
		} else if (line[0] == 'L' && line[1] == ',' && last_vts != 0) {
			timepoint_ns tracked;
			if (!m_clock_windowed_skew_tracker_to_local(clk, last_vts, &tracked))
				continue;
			int64_t now = last_host; // host time of this frame ~ latest IMU arrival
			if (first_ld_host < 0)
				first_ld_host = now;
			int64_t target = tracked - last_vts;
			int64_t d = target - offset;
			if (!valid || llabs(d) > 300000) {
				offset = target;
				if (!valid)
					valid_since = now;
				valid = true;
				steady_since = now;
				relatches++;
				if (getenv("LD_SIM_VERBOSE"))
					printf("  re-latch at %.3f s after valid: target moved %.3f ms from the slewed offset\n",
					       (now - valid_since) / 1e9, d / 1e6);
			} else {
				offset += d > 10000 ? 10000 : d < -10000 ? -10000 : d;
			}
			if (settled_rule_ns < 0 && (now - steady_since >= steady_ns || now - valid_since >= 8000000000LL))
				settled_rule_ns = now - valid_since;
			if (n < 200000) {
				tgt[n] = target;
				hostv[n] = now;
				n++;
			}
			frames++;
		}
	}
	int64_t final = tgt[n - 1];
	printf("%ld LD frames, %ld re-latches; current rule settles %.2f s after valid%s\n", frames, relatches,
	       settled_rule_ns / 1e9, settled_rule_ns >= 8000000000LL ? " (time limit)" : "");
	printf("target offset minus its final value (ms), and its change over the previous 1 s (us), per 0.5 s:\n");
	long j0 = 0;
	for (double t = 0; t < 20; t += 0.5) {
		long j = j0;
		while (j < n && hostv[j] - hostv[0] < (int64_t)(t * 1e9))
			j++;
		if (j >= n)
			break;
		long k = j;
		while (k > 0 && hostv[j] - hostv[k] < 1000000000LL)
			k--;
		printf("  %4.1fs: %8.3f ms, moved %7.0f us in 1 s\n", t, (tgt[j] - final) / 1e6, (double)(tgt[j] - tgt[k]) / 1e3);
		j0 = j;
	}
	return 0;
}
