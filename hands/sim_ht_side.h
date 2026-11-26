// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// hand_sim: the C interface between the tracker side (sim_ht_side.c) and the SteamVR plugin side (hand_sim.cpp).
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! The head pose at a time (the replay's clock): out = px, py, pz, qx, qy, qz, qw.
typedef void (*sim_head_fn)(int64_t at_ns, double out[7]);

struct sim_joints
{
	int active;
	int64_t timestamp_ns;
	double pose[26][7]; //!< px, py, pz, qx, qy, qz, qw (tracking space)
	int valid[26];
	float radius[26];
};

struct sim_gestures
{
	int valid;
	int64_t timestamp_ns;
	float confidence;
	float pinch_distance_normalized;
	uint8_t extended_mask;
	int category;
	float category_score;
	int pinch_image_valid;
	float pinch_image;
	int features_valid;
	float features[18];
	int custom_valid;
	int custom;
	float custom_scores[5];
};

int
sim_ht_init(const char *calibration_path, sim_head_fn head_fn);
void
sim_ht_push(const uint8_t *buf, uint32_t len, int64_t now_ns);
int
sim_ht_joints(int hand, int64_t at_ns, struct sim_joints *out);
int
sim_ht_gestures(int hand, struct sim_gestures *out);
void
sim_ht_destroy(void);

#ifdef __cplusplus
}
#endif
