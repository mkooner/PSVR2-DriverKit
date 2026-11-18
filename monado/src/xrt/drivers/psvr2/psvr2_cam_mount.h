// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS VR2 camera 0 mount from the headset's calibration block, shared by the driver and offline checks.
 *
 * The block holds camera 0's camera-to-head rotation at PSVR2_CALIB_CAM0_TO_HEAD_OFFSET (9 floats, row-major as
 * stored); Sony's driver tilts it by 15 degrees about X. The result is the camera's orientation in the head frame in
 * OpenXR camera axes (x right, y up, -z forward).
 *
 * @ingroup drv_psvr2
 */

#pragma once

#include "math/m_api.h"
#include "xrt/xrt_byte_order.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define PSVR2_CALIB_CAM0_TO_HEAD_OFFSET 0x6f0

/*!
 * Camera 0 mount from angles (degrees) in OpenXR camera axes: yaw about +Y (positive: left), pitch about +X (negative:
 * down), roll about +Z, applied as yaw * pitch * roll. The bottom left camera measured from motion (camera rotation vs
 * gyro, 2026-11-17): pitch -28.2, yaw +16.1.
 */
static inline void
psvr2_cam0_mount_from_angles(float pitch_deg, float yaw_deg, float roll_deg, struct xrt_quat *out)
{
	struct xrt_quat q_yaw, q_pitch, q_roll;
	math_quat_from_angle_vector((float)(yaw_deg * M_PI / 180.0), &(struct xrt_vec3){0, 1, 0}, &q_yaw);
	math_quat_from_angle_vector((float)(pitch_deg * M_PI / 180.0), &(struct xrt_vec3){1, 0, 0}, &q_pitch);
	math_quat_from_angle_vector((float)(roll_deg * M_PI / 180.0), &(struct xrt_vec3){0, 0, 1}, &q_roll);
	math_quat_rotate(&q_yaw, &q_pitch, out);
	math_quat_rotate(out, &q_roll, out);
}

//! Where camera 0 looks (degrees): its -Z axis in the head frame (pitch positive up, yaw positive left).
static inline void
psvr2_cam_mount_view_angles(const struct xrt_quat *mount, float *out_pitch_deg, float *out_yaw_deg)
{
	struct xrt_vec3 fwd;
	math_quat_rotate_vec3(mount, &(struct xrt_vec3){0, 0, -1}, &fwd);
	*out_pitch_deg = (float)(asin(fwd.y) * 180.0 / M_PI);
	*out_yaw_deg = (float)(atan2(-fwd.x, -fwd.z) * 180.0 / M_PI);
}

/*!
 * Camera 0 mount: Rx(@p tilt_deg) * R, R the stored matrix (transposed into xrt_matrix_3x3 order if @p transpose).
 * False if the block is too small or the matrix isn't a rotation.
 */
static inline bool
psvr2_cam0_mount_from_block(const uint8_t *data, uint32_t size, float tilt_deg, bool transpose, struct xrt_quat *out)
{
	if (size < PSVR2_CALIB_CAM0_TO_HEAD_OFFSET + 9 * sizeof(float)) {
		return false;
	}
	struct xrt_matrix_3x3 r = {0};
	for (int j = 0; j < 9; j++) {
		uint32_t bits;
		memcpy(&bits, data + PSVR2_CALIB_CAM0_TO_HEAD_OFFSET + j * 4, sizeof(bits));
		bits = __le32_to_cpu(bits);
		memcpy(&r.v[j], &bits, sizeof(float));
	}
	if (transpose) {
		math_matrix_3x3_transpose(&r, &r);
	}
	struct xrt_quat q_r;
	math_quat_from_matrix_3x3(&r, &q_r);
	float n = sqrtf(q_r.x * q_r.x + q_r.y * q_r.y + q_r.z * q_r.z + q_r.w * q_r.w);
	if (!(n > 0.9f && n < 1.1f)) {
		return false;
	}
	struct xrt_quat q_tilt;
	math_quat_from_angle_vector((float)(tilt_deg * M_PI / 180.0), &(struct xrt_vec3){1, 0, 0}, &q_tilt);
	math_quat_normalize(&q_r);
	math_quat_rotate(&q_tilt, &q_r, out);
	return true;
}
