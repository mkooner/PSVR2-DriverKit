// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Replays recorded SLAM positions (S rows) through the driver's psvr2_slam_carry_push and reports resets, carried
// ones, world changes, and the largest position step after the first 10 s before/after.
// Usage: carry_replay record.csv [...]
#include "math/m_api.h"
#include "math/m_vec3.h"
#include "psvr2_pose_filter.h"
#include <cmath>
#include <cstdio>
#include <vector>

int
main(int argc, char **argv)
{
	for (int a = 1; a < argc; a++) {
		FILE *f = fopen(argv[a], "r");
		if (!f)
			continue;
		std::vector<std::pair<long long, xrt_vec3>> s;
		char line[512];
		while (fgets(line, sizeof(line), f)) {
			long long host, vts;
			float x, y, z;
			if (line[0] == 'S' && sscanf(line, "S,%lld,%lld,%f,%f,%f", &host, &vts, &x, &y, &z) == 5)
				s.push_back({vts, xrt_vec3{x, y, z}});
		}
		fclose(f);
		if (s.size() < 100)
			continue;
		psvr2_slam_carry c;
		psvr2_slam_carry_init(&c, true);
		double max_raw = 0, max_out = 0;
		int big_raw = 0, big_out = 0;
		xrt_vec3 pr{}, po{};
		for (size_t i = 0; i < s.size(); i++) {
			enum psvr2_slam_carry_event ev;
			xrt_vec3 out = psvr2_slam_carry_push(&c, s[i].first, &s[i].second, &ev, nullptr);
			if (i > 0 && s[i].first - s[0].first > 10000000000LL) {
				double dr = m_vec3_len(m_vec3_sub(s[i].second, pr)), dout = m_vec3_len(m_vec3_sub(out, po));
				max_raw = fmax(max_raw, dr), max_out = fmax(max_out, dout);
				big_raw += dr > 0.05, big_out += dout > 0.05;
			}
			pr = s[i].second;
			po = out;
		}
		const char *n = argv[a];
		for (const char *p = argv[a]; *p; p++)
			if (*p == '/')
				n = p + 1;
		printf("%s: resets %u, carried %u, world changes %u; max step after 10 s %.1f -> %.1f cm, steps > 5 cm %d -> "
		       "%d; final offset %.1f cm\n",
		       n, c.carried + c.world_changes, c.carried, c.world_changes, max_raw * 100, max_out * 100, big_raw,
		       big_out, m_vec3_len(c.rebase) * 100);
	}
	return 0;
}
