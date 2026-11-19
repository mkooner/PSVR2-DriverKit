// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The Basalt referee's stand-ins for builds without Monado's SLAM module (XRT_FEATURE_SLAM off, e.g. the
 *         SteamVR plugin's minimal build that must load inside Steam's runtime): no referee, PSVR2_BASALT does nothing.
 * @ingroup drv_psvr2
 */

#include "psvr2_basalt.h"

#include "util/u_logging.h"

struct psvr2_basalt *
psvr2_basalt_create(struct xrt_frame_context *xfctx, const uint8_t *calib_block, uint32_t calib_size)
{
	(void)xfctx;
	(void)calib_block;
	(void)calib_size;
	U_LOG_W("PSVR2_BASALT: this build has no SLAM module (XRT_FEATURE_SLAM off), the Basalt referee is off");
	return NULL;
}

void
psvr2_basalt_push_imu(struct psvr2_basalt *b, timepoint_ns ts, const struct xrt_vec3 *accel, const struct xrt_vec3 *gyro)
{
	(void)b;
	(void)ts;
	(void)accel;
	(void)gyro;
}

void
psvr2_basalt_push_transfer(struct psvr2_basalt *b, timepoint_ns ts, const uint8_t *data, size_t length)
{
	(void)b;
	(void)ts;
	(void)data;
	(void)length;
}

bool
psvr2_basalt_ref_pose(struct psvr2_basalt *b, timepoint_ns ts, const struct xrt_pose *slam_pose, struct xrt_pose *out_pose)
{
	(void)b;
	(void)ts;
	(void)slam_pose;
	(void)out_pose;
	return false;
}

void
psvr2_basalt_destroy(struct psvr2_basalt **b_ptr)
{
	if (b_ptr != NULL) {
		*b_ptr = NULL;
	}
}
