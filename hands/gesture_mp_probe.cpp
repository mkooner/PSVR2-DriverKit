// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//
// Offline check of the SteamVR plugin's MediaPipe gesture state machine (ovrn_gesture_state.hpp, unchanged) on the
// measurements driver_ht_replay dumps (HT_REPLAY_GESTURES=<csv>, with MERCURY_MEDIAPIPE=1
// MERCURY_MEDIAPIPE_GESTURES=1). Each query is fed as the plugin feeds it (now = the newest frame seen, so the age
// check passes; the driver already drops old measurements).
// Usage: gesture_mp_probe gestures.csv [pinch_on pinch_off confirm_frames category_score] > out.csv
// Output per query: t,hand,valid,pinch,fist,open_palm,pinch_norm,trigger_value
#include "ovrn_gesture_state.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

int
main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "usage: %s gestures.csv [pinch_on pinch_off confirm_frames category_score]\n", argv[0]);
		return 1;
	}
	ovrn::gest::Config c;
	if (argc > 2) c.pinch_on = (float)atof(argv[2]);
	if (argc > 3) c.pinch_off = (float)atof(argv[3]);
	if (argc > 4) c.confirm_frames = atoi(argv[4]);
	if (argc > 5) c.min_category_score = (float)atof(argv[5]);
	FILE *f = fopen(argv[1], "r");
	if (f == nullptr) {
		perror(argv[1]);
		return 1;
	}
	ovrn::gest::Machine m[2] = {ovrn::gest::Machine(c), ovrn::gest::Machine(c)};
	char line[512];
	fgets(line, sizeof(line), f); // header
	printf("t,hand,valid,pinch,fist,open_palm,pinch_norm,trigger_value\n");
	while (fgets(line, sizeof(line), f) != nullptr) {
		double t;
		int hand, have, valid, mask, cat;
		long long ts;
		float conf, pinch, score;
		if (sscanf(line, "%lf,%d,%d,%d,%lld,%f,%f,%d,%d,%f", &t, &hand, &have, &valid, &ts, &conf, &pinch, &mask,
		           &cat, &score) != 10 || hand < 0 || hand > 1) {
			continue;
		}
		ovrn::gest::Sample s;
		s.valid = have && valid;
		s.timestamp_ns = ts;
		s.confidence = conf;
		s.pinch_distance_normalized = pinch;
		s.extended_mask = (uint8_t)mask;
		s.category = cat;
		s.category_score = score;
		m[hand].update(s, ts);
		const ovrn::gest::State &st = m[hand].state();
		printf("%.4f,%d,%d,%d,%d,%d,%.4f,%.3f\n", t, hand, st.valid, st.pinch, st.fist, st.openPalm,
		       st.pinchDistanceNormalized, m[hand].pinchValue());
	}
	return 0;
}
