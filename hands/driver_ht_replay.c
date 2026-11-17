// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//
// Drives the driver's own hand tracking module (psvr2_hand_tracking.c) with a PSVR2_RECORD_CAMERA recording in real
// time, as the USB thread would, with a stand-in head device at identity, and queries the hands as SteamVR would
// (every 11 ms). Reports how often each hand is active and the right wrist positions.
// Usage: driver_ht_replay recording.pv2c stereo.json
#include "xrt/xrt_device.h"
#include "os/os_time.h"
#include "math/m_api.h"
#include "util/u_device.h"
#include "util/u_json.h"
#include "util/u_time.h"
#include "tracking/t_tracking.h"
#include "psvr2_hand_tracking.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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

static double
jnum(cJSON *a, int i)
{
	return cJSON_GetArrayItem(a, i)->valuedouble;
}

int
main(int argc, char **argv)
{
	if (argc < 3) {
		fprintf(stderr, "usage: %s recording.pv2c stereo.json\n", argv[0]);
		return 1;
	}
	FILE *jf = fopen(argv[2], "rb");
	static char js[1 << 16];
	size_t jn = fread(js, 1, sizeof(js) - 1, jf);
	js[jn] = 0;
	fclose(jf);
	cJSON *j = cJSON_Parse(js);
	struct t_camera_calibration cams[2] = {0};
	for (int v = 0; v < 2; v++) {
		cJSON *view = cJSON_GetArrayItem(cJSON_GetObjectItem(j, "views"), v);
		cams[v].image_size_pixels.w = cams[v].image_size_pixels.h = 1016;
		cams[v].distortion_model = T_DISTORTION_PSVR2;
		cJSON *K = cJSON_GetObjectItem(view, "K");
		for (int r = 0; r < 3; r++)
			for (int k = 0; k < 3; k++)
				cams[v].intrinsics[r][k] = jnum(cJSON_GetArrayItem(K, r), k);
		for (int k = 0; k < 20; k++)
			cams[v].psvr2.params[k] = jnum(cJSON_GetObjectItem(view, "dist"), k);
	}
	// x_cam1 = R x_cam0 + T (OpenCV) -> cam1 in cam0, OpenXR camera axes. Camera 0 at the head origin.
	cJSON *R = cJSON_GetObjectItem(j, "R"), *T = cJSON_GetObjectItem(j, "T");
	const float F[3] = {1, -1, -1};
	struct xrt_matrix_3x3 m;
	for (int a = 0; a < 3; a++)
		for (int b = 0; b < 3; b++)
			m.v[a * 3 + b] = F[a] * jnum(cJSON_GetArrayItem(R, a), b) * F[b];
	struct xrt_pose c1_c0;
	math_quat_from_matrix_3x3(&m, &c1_c0.orientation);
	c1_c0.position = (struct xrt_vec3){F[0] * jnum(T, 0), F[1] * jnum(T, 1), F[2] * jnum(T, 2)};
	struct xrt_pose poses[2];
	poses[0] = (struct xrt_pose)XRT_POSE_IDENTITY;
	math_pose_invert(&c1_c0, &poses[1]);

	struct xrt_device *head = U_DEVICE_ALLOCATE(struct xrt_device, U_DEVICE_ALLOC_TRACKING_NONE, 1, 0);
	head->get_tracked_pose = head_pose;
	struct xrt_device *ht = psvr2_hand_tracking_create(head);
	psvr2_hand_tracking_set_cameras(ht, cams, poses);

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
	printf("%d frames offered over %.1f s, %d queries: left active %.0f%%, right active %.0f%%",
	       pushed, (os_monotonic_get_ns() - start) / 1e9, queries, 100.0 * act[0] / queries,
	       100.0 * act[1] / queries);
	if (act[1])
		printf("; right wrist mean (head frame) %.3f %.3f %.3f m", sx / act[1], sy / act[1], sz / act[1]);
	printf("\n");
	xrt_device_destroy(&ht);
	return 0;
}
