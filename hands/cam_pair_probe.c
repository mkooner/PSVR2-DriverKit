// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//
// Offline check of the bottom camera pair the hand tracker gets: runs the driver's calibration block parser
// (psvr2_parse_gen_calibration_block, the controller tracker's camera chain, what psvr2_hmd_offer_hand_tracking_cameras
// receives) and the cameras' true placement (psvr2_bottom_cameras_in_head, what the passthrough uses) on a saved
// calibration block, and compares the two: each camera's rotation, and the pair's relation (stereo for Mercury).
// Usage: cam_pair_probe ~/.config/monado/psvr2-calibration-<serial>.bin
#include "psvr2.h"
#include "psvr2_cam_mount.h"
#include "psvr2_constellation_tracking.h"
#include "math/m_api.h"
#include "math/m_space.h"

#include <stdio.h>
#include <stdlib.h>

static struct xrt_pose g_offered[2];
static int g_have;

void
psvr2_hmd_offer_hand_tracking_cameras(struct psvr2_hmd *hmd, const struct t_camera_calibration cams[2],
                                      const struct xrt_pose poses[2])
{
	g_offered[0] = poses[0];
	g_offered[1] = poses[1];
	g_have = 1;
}
// The tracker hooks the parse doesn't reach without controllers are in cam_pair_probe_stubs.c.

static float
deg_between(struct xrt_quat a, struct xrt_quat b)
{
	float d = fabsf(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z);
	return 2.0f * acosf(d > 1 ? 1 : d) * 57.29578f;
}

static void
relation(const struct xrt_pose p[2], struct xrt_pose *out)
{
	struct xrt_pose inv;
	math_pose_invert(&p[0], &inv);
	math_pose_transform(&inv, &p[1], out); // camera 1 in camera 0
}

int
main(int argc, char **argv)
{
	FILE *f = fopen(argv[1], "rb");
	if (f == NULL) {
		return 1;
	}
	static uint8_t data[1 << 20];
	const uint32_t size = (uint32_t)fread(data, 1, sizeof(data), f);
	fclose(f);
	struct psvr2_hmd *hmd = calloc(1, sizeof(*hmd));
	hmd->log_level = U_LOGGING_WARN;
	psvr2_parse_gen_calibration_block(hmd, data, size);
	struct xrt_pose truth[2];
	if (!g_have || !psvr2_bottom_cameras_in_head(data, size, truth)) {
		printf("parse failed (offered %d)\n", g_have);
		return 1;
	}
	for (int i = 0; i < 2; i++) {
		printf("camera %d: rotation differs by %.2f deg; position chain (%.1f %.1f %.1f) cm, true (%.1f %.1f %.1f) cm\n",
		       i, deg_between(g_offered[i].orientation, truth[i].orientation), g_offered[i].position.x * 100,
		       g_offered[i].position.y * 100, g_offered[i].position.z * 100, truth[i].position.x * 100,
		       truth[i].position.y * 100, truth[i].position.z * 100);
	}
	struct xrt_pose rc, rt;
	relation(g_offered, &rc);
	relation(truth, &rt);
	printf("pair (camera 1 in camera 0): chain turn %.2f deg, true turn %.2f deg, the two differ by %.2f deg\n",
	       deg_between(rc.orientation, (struct xrt_quat)XRT_QUAT_IDENTITY),
	       deg_between(rt.orientation, (struct xrt_quat)XRT_QUAT_IDENTITY), deg_between(rc.orientation, rt.orientation));
	printf("pair offset: chain (%.2f %.2f %.2f) cm, true (%.2f %.2f %.2f) cm\n", rc.position.x * 100,
	       rc.position.y * 100, rc.position.z * 100, rt.position.x * 100, rt.position.y * 100, rt.position.z * 100);
	return 0;
}
