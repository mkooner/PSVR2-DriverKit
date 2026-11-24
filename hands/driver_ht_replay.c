// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//
// Drives the driver's own hand tracking module (psvr2_hand_tracking.c) with a PSVR2_RECORD_CAMERA recording in real
// time, as the USB thread would, with a stand-in head device at identity, and queries the hands as SteamVR would
// (every 11 ms). Reports how often each hand is active and the right wrist positions.
// The cameras come from the headset's calibration block, through the driver's own parser (the controller tracker's
// camera chain, exactly what the driver gives the hand tracker), not from a separate stereo file.
// Usage: driver_ht_replay recording.pv2c ~/.config/monado/psvr2-calibration-<serial>.bin
// Also reports each hand's active share in HT_REPLAY_HANDS=<from>-<to> s (default 2-35, the hands-over-the-desk part
// of camera-20261120-105225) and in HT_REPLAY_EMPTY=<from>-<to> s (default 37-43, no hands: false detections), and
// the CPU used (cores on average over the replay).
#include "xrt/xrt_device.h"
#include "os/os_time.h"
#include "math/m_api.h"
#include "util/u_device.h"
#include "util/u_json.h"
#include "util/u_time.h"
#include "tracking/t_tracking.h"
#include "psvr2_hand_tracking.h"
#include "psvr2.h"
#include "psvr2_constellation_tracking.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/resource.h>

static xrt_result_t
head_pose(struct xrt_device *xdev, enum xrt_input_name name, int64_t at, struct xrt_space_relation *out)
{
	*out = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
	out->pose.orientation.w = 1;
	out->relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                       XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                                                       XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                                                       XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
	return XRT_SUCCESS;
}

static struct t_camera_calibration g_cams[2];
static struct xrt_pose g_poses[2];
static int g_have_cams;

//! The parser hands the bottom pair here (as psvr2.c's version passes it to the hand tracker).
void
psvr2_hmd_offer_hand_tracking_cameras(struct psvr2_hmd *hmd, const struct t_camera_calibration cams[2],
                                      const struct xrt_pose poses[2])
{
	(void)hmd;
	memcpy(g_cams, cams, sizeof(g_cams));
	memcpy(g_poses, poses, sizeof(g_poses));
	g_have_cams = 1;
}

//! The hand tracker asks the headset for the camera images; here the recording is the headset.
void
psvr2_request_bottom_camera_images(struct xrt_device *xdev, bool on)
{
	(void)xdev;
	(void)on;
}

int
main(int argc, char **argv)
{
	if (argc < 3) {
		fprintf(stderr, "usage: %s recording.pv2c psvr2-calibration-<serial>.bin\n", argv[0]);
		return 1;
	}
	FILE *cf = fopen(argv[2], "rb");
	if (cf == NULL) {
		fprintf(stderr, "can't open %s\n", argv[2]);
		return 1;
	}
	static uint8_t block[1 << 20];
	const uint32_t block_size = (uint32_t)fread(block, 1, sizeof(block), cf);
	fclose(cf);
	struct psvr2_hmd *hmd = calloc(1, sizeof(*hmd));
	hmd->log_level = U_LOGGING_WARN;
	psvr2_parse_gen_calibration_block(hmd, block, block_size);
	if (!g_have_cams) {
		fprintf(stderr, "no bottom camera pair in %s\n", argv[2]);
		return 1;
	}
	struct t_camera_calibration *cams = g_cams;
	struct xrt_pose *poses = g_poses;

	struct xrt_device *head = U_DEVICE_ALLOCATE(struct xrt_device, U_DEVICE_ALLOC_TRACKING_NONE, 1, 0);
	head->get_tracked_pose = head_pose;
	struct xrt_device *ht = psvr2_hand_tracking_create(head);
	psvr2_hand_tracking_set_cameras(ht, cams, poses);
	psvr2_hand_tracking_set_active(ht, true);

	FILE *in = fopen(argv[1], "rb");
	char magic[8];
	if (fread(magic, 1, 8, in) != 8 || memcmp(magic, "PV2C", 4)) {
		fprintf(stderr, "not a PSVR2_RECORD_CAMERA file\n");
		return 1;
	}
	// Mercury loads on the worker when the first frame is offered; give it the first frame until it runs.
	static uint8_t buf[1040640];
	int64_t first_rec = 0, start = 0, next_query = 0;
	int queries = 0, act[2] = {0, 0}, pushed = 0;
	double h_lo = 2, h_hi = 35, e_lo = 37, e_hi = 43;
	if (getenv("HT_REPLAY_HANDS"))
		sscanf(getenv("HT_REPLAY_HANDS"), "%lf-%lf", &h_lo, &h_hi);
	if (getenv("HT_REPLAY_EMPTY"))
		sscanf(getenv("HT_REPLAY_EMPTY"), "%lf-%lf", &e_lo, &e_hi);
	int q_hands = 0, q_empty = 0, act_hands[2] = {0, 0}, act_empty[2] = {0, 0};
	double gap_start[2] = {-1, -1}, last_active[2] = {-1, -1}, gap_time[2][5] = {{0}};
	int gap_count[2][5] = {{0}}, held_hands[2][2] = {{0}};
	struct rusage ru0;
	getrusage(RUSAGE_SELF, &ru0);
	double sx = 0, sy = 0, sz = 0;
	for (;;) {
		uint64_t ns;
		uint32_t len, dropped;
		if (fread(&ns, 8, 1, in) != 1 || fread(&len, 4, 1, in) != 1 || fread(&dropped, 4, 1, in) != 1 ||
		    len > sizeof(buf) || fread(buf, 1, len, in) != len)
			break;
		if (!first_rec) {
			first_rec = (int64_t)ns;
			// Wait for Mercury to start (the worker starts it on set_cameras).
			for (int w = 0; w < 300; w++) {
				struct xrt_hand_joint_set s;
				int64_t t;
				xrt_device_get_hand_tracking(ht, XRT_INPUT_HT_UNOBSTRUCTED_RIGHT, os_monotonic_get_ns(), &s, &t);
				struct timespec d = {0, 20 * 1000 * 1000};
				nanosleep(&d, NULL);
			}
			start = os_monotonic_get_ns();
			next_query = start;
		}
		int64_t due = start + ((int64_t)ns - first_rec);
		// Query hands every 11 ms (90 Hz) until this frame is due, then offer it.
		for (;;) {
			int64_t now = os_monotonic_get_ns();
			if (now >= next_query) {
				for (int h = 0; h < 2; h++) {
					struct xrt_hand_joint_set s;
					int64_t t;
					xrt_device_get_hand_tracking(ht,
					                             h ? XRT_INPUT_HT_UNOBSTRUCTED_RIGHT
					                               : XRT_INPUT_HT_UNOBSTRUCTED_LEFT,
					                             now, &s, &t);
					const double ts = (now - start) / 1e9;
					if (s.is_active && ts >= h_lo && ts <= h_hi)
						act_hands[h]++;
					// Gaps while hands are in view: how much lost time comes from short dropouts, and what holding
					// the last pose for 0.15 / 0.3 s after a loss would give.
					if (ts >= h_lo && ts <= h_hi) {
						if (s.is_active) {
							if (gap_start[h] >= 0) {
								const double g = ts - gap_start[h];
								const int b = g < 0.1 ? 0 : g < 0.25 ? 1 : g < 0.5 ? 2 : g < 1.0 ? 3 : 4;
								gap_time[h][b] += g;
								if (g >= 1.0 && getenv("HT_REPLAY_GAPS"))
									printf("GAP %s hand lost %.2f-%.2f s (%.2f s)\n", h ? "right" : "left",
									       gap_start[h], ts, g);
								gap_count[h][b]++;
								gap_start[h] = -1;
							}
							last_active[h] = ts;
						} else if (gap_start[h] < 0 && last_active[h] >= 0) {
							gap_start[h] = ts;
						}
						for (int k = 0; k < 2; k++) {
							const double hold = k ? 0.3 : 0.15;
							if (s.is_active || (last_active[h] >= 0 && ts - last_active[h] <= hold))
								held_hands[h][k]++;
						}
					}
					if (s.is_active && ts >= e_lo && ts <= e_hi)
						act_empty[h]++;
					if (s.is_active) {
						act[h]++;
						if (h == 1) {
							struct xrt_pose w;
							math_pose_transform(
							    &s.hand_pose.pose,
							    &s.values.hand_joint_set_default[XRT_HAND_JOINT_WRIST].relation.pose,
							    &w);
							sx += w.position.x;
							sy += w.position.y;
							sz += w.position.z;
						}
					}
				}
				queries++;
				{
					const double ts = (now - start) / 1e9;
					q_hands += ts >= h_lo && ts <= h_hi;
					q_empty += ts >= e_lo && ts <= e_hi;
				}
				next_query += 11 * U_TIME_1MS_IN_NS;
			}
			if (now >= due)
				break;
			struct timespec d = {0, 500 * 1000};
			nanosleep(&d, NULL);
		}
		psvr2_hand_tracking_push_transfer(ht, buf, len, os_monotonic_get_ns());
		pushed++;
	}
	static const char *const bucket[5] = {"<0.1 s", "0.1-0.25 s", "0.25-0.5 s", "0.5-1 s", ">1 s"};
	for (int h = 0; h < 2; h++) {
		printf("%s hand gaps in view (count, lost s):", h ? "right" : "left");
		for (int b = 0; b < 5; b++)
			printf(" %s %d/%.1f", bucket[b], gap_count[h][b], gap_time[h][b]);
		printf("; with a hold of 0.15 s %.0f%%, 0.3 s %.0f%%\n", q_hands ? 100.0 * held_hands[h][0] / q_hands : 0,
		       q_hands ? 100.0 * held_hands[h][1] / q_hands : 0);
	}
	printf("%d frames offered over %.1f s, %d queries: left active %.0f%%, right active %.0f%%",
	       pushed, (os_monotonic_get_ns() - start) / 1e9, queries, 100.0 * act[0] / queries,
	       100.0 * act[1] / queries);
	if (act[1])
		printf("; right wrist mean (head frame) %.3f %.3f %.3f m", sx / act[1], sy / act[1], sz / act[1]);
	printf("\n");
	struct rusage ru1;
	getrusage(RUSAGE_SELF, &ru1);
	const double cpu = (ru1.ru_utime.tv_sec - ru0.ru_utime.tv_sec) + (ru1.ru_utime.tv_usec - ru0.ru_utime.tv_usec) / 1e6 +
	                   (ru1.ru_stime.tv_sec - ru0.ru_stime.tv_sec) + (ru1.ru_stime.tv_usec - ru0.ru_stime.tv_usec) / 1e6;
	printf("hands in view %.0f-%.0f s: left %.0f%%, right %.0f%%; no hands %.0f-%.0f s: false left %.0f%%, right %.0f%%; "
	       "CPU %.2f cores on average\n",
	       h_lo, h_hi, q_hands ? 100.0 * act_hands[0] / q_hands : 0, q_hands ? 100.0 * act_hands[1] / q_hands : 0, e_lo,
	       e_hi, q_empty ? 100.0 * act_empty[0] / q_empty : 0, q_empty ? 100.0 * act_empty[1] / q_empty : 0,
	       cpu / ((os_monotonic_get_ns() - start) / 1e9));
	xrt_device_destroy(&ht);
	return 0;
}
