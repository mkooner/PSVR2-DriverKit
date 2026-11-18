// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Basalt visual-inertial tracking of the PS VR2 headset (bottom camera pair + IMU), used as a referee for the
 *         headset's own SLAM positions in the pose filter (psvr2_pose_filter ref_jump_m), never as the output.
 *
 * Offline-validated pipeline (psvr2-test/vio): camera mode 0x10 BC4 frames, Basalt (Monado fork) through Monado's
 * SLAM tracker, kb4 fit to Sony's camera model, camera placement from psvr2_cam0_mount_from_block (as stored, -15 deg,
 * matches the mount measured from motion within 1.5 deg) - its own, independent of the controller camera chain - and
 * IMU noise measured on this headset (x5). Basalt starts only once the headset has been still for a moment (started
 * while moving it diverged: 2026-11-17 19:12 recording) and is reset when it disagrees with SLAM by a lot.
 *
 * @ingroup drv_psvr2
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "xrt/xrt_frame.h"
#include "util/u_time.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct psvr2_basalt;

/*!
 * Create from the headset's calibration block (GEN_DATA_CALIBRATION_BLOCK). NULL if Basalt can't be loaded or the
 * block doesn't hold what's needed. Times are on one clock (the driver: monotonic; offline harness: anything).
 */
struct psvr2_basalt *
psvr2_basalt_create(struct xrt_frame_context *xfctx, const uint8_t *calib_block, uint32_t calib_size);

//! IMU sample (head axes, m/s^2 and rad/s), at 2 kHz.
void
psvr2_basalt_push_imu(struct psvr2_basalt *b, timepoint_ns ts, const struct xrt_vec3 *accel, const struct xrt_vec3 *gyro);

//! A camera mode 0x10 transfer (256 byte header + two BC4 images), @p ts its exposure time on the IMU's clock.
void
psvr2_basalt_push_transfer(struct psvr2_basalt *b, timepoint_ns ts, const uint8_t *data, size_t length);

/*!
 * Basalt's head pose for time @p ts, rotated into the SLAM world (estimated from @p slam_pose, the headset's SLAM pose
 * of the same time), for use as the pose filter's referee. False when Basalt has no pose for that time (not started,
 * not there yet, diverged).
 */
bool
psvr2_basalt_ref_pose(struct psvr2_basalt *b, timepoint_ns ts, const struct xrt_pose *slam_pose, struct xrt_pose *out_pose);

void
psvr2_basalt_destroy(struct psvr2_basalt **b_ptr);

#ifdef __cplusplus
}
#endif
