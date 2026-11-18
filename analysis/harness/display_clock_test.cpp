// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Offline test of the SteamVR plugin's vsync clock (ovrd_display_clock.hpp) against a simulated compositor: known clock
// offset, 89.91 Hz vsync, a frame entry appearing 0-0.5 ms after its vsync, reads every ~1.06 ms with up to 0.3 ms
// scheduling jitter and the odd 3 ms stall. Checks the offset it finds and that display_time_ns picks the right refresh.
#include "ovrd_display_clock.hpp"
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
int main() {
	std::mt19937 rng(7);
	std::uniform_real_distribution<double> U(0, 1);
	const double true_off = 45870.9664; // mono = steamvr + off
	const int64_t period = 11122233;
	OvrdDisplayClock c(period);
	int64_t t = (int64_t)(46000.0 * 1e9);
	int64_t vsync0 = t - 3000000; // a vsync 3 ms before the start
	std::vector<double> off_err, tgt_err;
	int wrong_refresh = 0, checks = 0, near_cut_flips = 0;
	for (int i = 0; i < 60000; i++) { // 60 s
		t += 1060000 + (int64_t)(U(rng) * 300000) + (U(rng) < 0.002 ? 3000000 : 0);
		int64_t k = (t - vsync0) / period;          // vsyncs so far
		int64_t vk = vsync0 + k * period;            // newest vsync
		int64_t appear = vk + (int64_t)(U(rng) * 500000);
		if (appear > t) { vk -= period; k--; }       // its entry isn't there yet: the previous one is the newest
		double t_s = vk * 1e-9 - true_off;
		c.observe(t, t_s, (uint32_t)k);
		if (!c.ready()) continue;
		off_err.push_back((c.offset_s() - true_off) * 1e3);
		// target with a cut of half a refresh, 6.5 ms after vsync: must be the first vsync after now + cut (truth)
		int64_t cut = period / 2, after = 6500000;
		int64_t got = c.display_time_ns(t, cut, after);
		int64_t kk = (t + cut - vsync0) / period + 1;
		int64_t want = vsync0 + kk * period + after;
		tgt_err.push_back((got - want) * 1e-6);
		checks++;
		// Within 0.5 ms of the cut either refresh is a fair answer (the clock is good to ~0.1 ms); elsewhere it must be right.
		int64_t from_boundary = (t + cut - vsync0) % period;
		bool near_cut = from_boundary < 500000 || from_boundary > period - 500000;
		if (std::llabs(got - want) > period / 2) { if (near_cut) near_cut_flips++; else wrong_refresh++; }
	}
	auto pct = [](std::vector<double> v, double p) { std::sort(v.begin(), v.end()); return v[(size_t)(p * (v.size() - 1))]; };
	printf("clock offset error: median %+.3f ms, p1 %+.3f, p99 %+.3f ms\n", pct(off_err, 0.5), pct(off_err, 0.01), pct(off_err, 0.99));
	printf("display time error: median %+.3f ms, p1 %+.3f, p99 %+.3f ms; wrong refresh in %d of %d\n", pct(tgt_err, 0.5),
	       pct(tgt_err, 0.01), pct(tgt_err, 0.99), wrong_refresh, checks);
	printf("(other refresh within 0.5 ms of the cut, where both are right: %d)\n", near_cut_flips);
	return wrong_refresh == 0 ? 0 : 1;
}
