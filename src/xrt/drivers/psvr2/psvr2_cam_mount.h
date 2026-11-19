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
	const struct xrt_vec3 y_axis = {0, 1, 0}, x_axis = {1, 0, 0}, z_axis = {0, 0, 1};
	math_quat_from_angle_vector((float)(yaw_deg * M_PI / 180.0), &y_axis, &q_yaw);
	math_quat_from_angle_vector((float)(pitch_deg * M_PI / 180.0), &x_axis, &q_pitch);
	math_quat_from_angle_vector((float)(roll_deg * M_PI / 180.0), &z_axis, &q_roll);
	math_quat_rotate(&q_yaw, &q_pitch, out);
	math_quat_rotate(out, &q_roll, out);
}

//! Where camera 0 looks (degrees): its -Z axis in the head frame (pitch positive up, yaw positive left).
static inline void
psvr2_cam_mount_view_angles(const struct xrt_quat *mount, float *out_pitch_deg, float *out_yaw_deg)
{
	struct xrt_vec3 fwd;
	const struct xrt_vec3 minus_z = {0, 0, -1};
	math_quat_rotate_vec3(mount, &minus_z, &fwd);
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
	struct xrt_matrix_3x3 r;
	memset(&r, 0, sizeof(r));
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
	const struct xrt_vec3 tilt_axis = {1, 0, 0};
	math_quat_from_angle_vector((float)(tilt_deg * M_PI / 180.0), &tilt_axis, &q_tilt);
	math_quat_normalize(&q_r);
	math_quat_rotate(&q_tilt, &q_r, out);
	return true;
}

/*!
 * Both bottom cameras' poses in the head frame, OpenXR camera axes, from the calibration block: camera 0's corrected
 * mount (Rx(-15) * R as stored, 1.5 deg from the motion measurement), camera 1 from the block's 1 -> 0 relation as
 * Basalt uses it (R_cam0_cam1 = stored^T, camera 1 at -stored^T t in camera 0, OpenCV axes), the head origin taken
 * midway between the cameras (as Basalt's IMU). For the passthrough view, independent of the controller tracker's
 * camera chain (PSVR2_CAM0_MOUNT_LEGACY there). False if the block lacks the mount or the relation.
 */
static inline bool
psvr2_bottom_cameras_in_head(const uint8_t *data, uint32_t size, struct xrt_pose out[2])
{
	struct xrt_quat q_mount;
	if (!psvr2_cam0_mount_from_block(data, size, -15.0f, false, &q_mount)) {
		return false;
	}
	// The relation "to camera 0, from camera 1" (layout as psvr2_basalt.c's parse_block).
	if (size < 24 || memcmp(data, "SIE\x01", 4) != 0) {
		return false;
	}
	const uint8_t ndist = data[15], nrel = data[16];
	if (ndist < 2 || size < 24u + ndist * 228u + nrel * 52u) {
		return false;
	}
	float S[9], t[3];
	bool found = false;
	for (int i = 0; i < nrel && !found; i++) {
		const uint8_t *r = data + 24 + ndist * 228 + i * 52;
		if (r[0] == 0 && r[1] == 1) {
			for (int j = 0; j < 12; j++) {
				uint32_t bits;
				memcpy(&bits, r + 4 + j * 4, sizeof(bits));
				bits = __le32_to_cpu(bits);
				memcpy(j < 9 ? &S[j] : &t[j - 9], &bits, sizeof(float));
			}
			found = true;
		}
	}
	if (!found) {
		return false;
	}
	// Camera 0 in OpenCV axes: mount * Rx(180).
	struct xrt_quat flip, q0cv, q1cv, q01;
	const struct xrt_vec3 flip_axis = {1, 0, 0};
	math_quat_from_angle_vector((float)M_PI, &flip_axis, &flip);
	math_quat_rotate(&q_mount, &flip, &q0cv);
	struct xrt_matrix_3x3 R01;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			R01.v[i * 3 + j] = S[j * 3 + i];
		}
	}
	math_quat_from_matrix_3x3(&R01, &q01);
	math_quat_normalize(&q01);
	math_quat_rotate(&q0cv, &q01, &q1cv);
	struct xrt_vec3 p1_in_0;
	p1_in_0.x = -(S[0] * t[0] + S[3] * t[1] + S[6] * t[2]);
	p1_in_0.y = -(S[1] * t[0] + S[4] * t[1] + S[7] * t[2]);
	p1_in_0.z = -(S[2] * t[0] + S[5] * t[1] + S[8] * t[2]);
	struct xrt_vec3 d;
	math_quat_rotate_vec3(&q0cv, &p1_in_0, &d);
	// Back to OpenXR camera axes (Rx(180) undoes itself).
	math_quat_rotate(&q0cv, &flip, &out[0].orientation);
	math_quat_rotate(&q1cv, &flip, &out[1].orientation);
	out[0].position.x = -0.5f * d.x;
	out[0].position.y = -0.5f * d.y;
	out[0].position.z = -0.5f * d.z;
	out[1].position.x = 0.5f * d.x;
	out[1].position.y = 0.5f * d.y;
	out[1].position.z = 0.5f * d.z;
	return true;
}
