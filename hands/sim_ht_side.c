// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// hand_sim's tracker side: the driver's own hand tracking (psvr2_hand_tracking.c, Mercury + MediaPipe) on recorded
// camera transfers, as driver_ht_replay runs it, behind a small C interface for the C++ side (the SteamVR plugin's
// hand code). The head pose the tracker asks for comes from the C++ side (the session's own head poses).
#include "xrt/xrt_device.h"
#include "os/os_time.h"
#include "math/m_api.h"
#include "util/u_device.h"
#include "tracking/t_tracking.h"
#include "tracking/t_hand_tracking.h"
#include "psvr2_hand_tracking.h"
#include "psvr2_interface.h"
#include "psvr2.h"
#include "psvr2_constellation_tracking.h"

#include "sim_ht_side.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static sim_head_fn g_head_fn;

static xrt_result_t
head_pose(struct xrt_device *xdev, enum xrt_input_name name, int64_t at, struct xrt_space_relation *out)
{
	(void)xdev;
	(void)name;
	*out = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
	double p[7] = {0, 0, 0, 0, 0, 0, 1};
	if (g_head_fn != NULL) {
		g_head_fn(at, p);
	}
	out->pose.position = (struct xrt_vec3){(float)p[0], (float)p[1], (float)p[2]};
	out->pose.orientation = (struct xrt_quat){(float)p[3], (float)p[4], (float)p[5], (float)p[6]};
	out->relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                       XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                                                       XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                                                       XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
	return XRT_SUCCESS;
}

static struct t_camera_calibration g_cams[2];
static struct xrt_pose g_poses[2];
static int g_have_cams;

void
psvr2_hmd_offer_hand_tracking_cameras(struct psvr2_hmd *hmd, const struct t_camera_calibration cams[2],
                                      const struct xrt_pose poses[2])
{
	(void)hmd;
	memcpy(g_cams, cams, sizeof(g_cams));
	memcpy(g_poses, poses, sizeof(g_poses));
	g_have_cams = 1;
}

void
psvr2_request_bottom_camera_images(struct xrt_device *xdev, bool on)
{
	(void)xdev;
	(void)on;
}

static struct xrt_device *g_ht;

int
sim_ht_init(const char *calibration_path, sim_head_fn head_fn)
{
	g_head_fn = head_fn;
	FILE *cf = fopen(calibration_path, "rb");
	if (cf == NULL) {
		fprintf(stderr, "can't open %s\n", calibration_path);
		return 0;
	}
	static uint8_t block[1 << 20];
	const uint32_t block_size = (uint32_t)fread(block, 1, sizeof(block), cf);
	fclose(cf);
	struct psvr2_hmd *hmd = calloc(1, sizeof(*hmd));
	hmd->log_level = U_LOGGING_WARN;
	psvr2_parse_gen_calibration_block(hmd, block, block_size);
	if (!g_have_cams) {
		fprintf(stderr, "no bottom camera pair in %s\n", calibration_path);
		return 0;
	}
	struct xrt_device *head = U_DEVICE_ALLOCATE(struct xrt_device, U_DEVICE_ALLOC_TRACKING_NONE, 1, 0);
	head->get_tracked_pose = head_pose;
	g_ht = psvr2_hand_tracking_create(head);
	psvr2_hand_tracking_set_cameras(g_ht, g_cams, g_poses);
	psvr2_hand_tracking_set_active(g_ht, true);
	return 1;
}

void
sim_ht_push(const uint8_t *buf, uint32_t len, int64_t now_ns)
{
	psvr2_hand_tracking_push_transfer(g_ht, (uint8_t *)buf, len, now_ns);
}

int
sim_ht_joints(int hand, int64_t at_ns, struct sim_joints *out)
{
	memset(out, 0, sizeof(*out));
	struct xrt_hand_joint_set set;
	memset(&set, 0, sizeof(set));
	int64_t ts = 0;
	if (xrt_device_get_hand_tracking(g_ht, hand ? XRT_INPUT_HT_UNOBSTRUCTED_RIGHT : XRT_INPUT_HT_UNOBSTRUCTED_LEFT,
	                                 at_ns, &set, &ts) != XRT_SUCCESS) {
		return 0;
	}
	const int need = XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
	if (!set.is_active || (set.hand_pose.relation_flags & need) != need) {
		return 0;
	}
	// As the plugin's Monado backend (handJoints): hand pose times each joint, the tracking origin's offset is
	// identity here as on the headset.
	out->active = 1;
	out->timestamp_ns = ts;
	for (int i = 0; i < 26; i++) {
		const struct xrt_hand_joint_value *v = &set.values.hand_joint_set_default[i];
		struct xrt_pose p;
		math_pose_transform(&set.hand_pose.pose, &v->relation.pose, &p);
		out->pose[i][0] = p.position.x;
		out->pose[i][1] = p.position.y;
		out->pose[i][2] = p.position.z;
		out->pose[i][3] = p.orientation.x;
		out->pose[i][4] = p.orientation.y;
		out->pose[i][5] = p.orientation.z;
		out->pose[i][6] = p.orientation.w;
		out->valid[i] = (v->relation.relation_flags & need) == need;
		out->radius[i] = v->radius;
	}
	return 1;
}

int
sim_ht_gestures(int hand, struct sim_gestures *out)
{
	memset(out, 0, sizeof(*out));
	struct t_hand_gestures g;
	memset(&g, 0, sizeof(g));
	if (!psvr2_hand_tracking_get_gestures(g_ht, hand, &g)) {
		return 0;
	}
	out->valid = g.valid;
	out->timestamp_ns = g.timestamp_ns;
	out->confidence = g.confidence;
	out->pinch_distance_normalized = g.pinch_distance_normalized;
	out->extended_mask = g.extended_mask;
	out->category = (int)g.category;
	out->category_score = g.category_score;
	out->custom_valid = g.custom_valid;
	out->custom = (int)g.custom;
	for (int c = 0; c < 5; c++) {
		out->custom_scores[c] = g.custom_scores[c];
	}
	return 1;
}

void
sim_ht_destroy(void)
{
	xrt_device_destroy(&g_ht);
}
