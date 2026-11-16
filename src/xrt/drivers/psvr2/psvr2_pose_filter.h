// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS VR2 headset pose filter: blends 2kHz gyro integration with the headset's SLAM poses.
 *
 * The headset reports SLAM poses at a low rate and raw IMU at 2kHz. Snapping the prediction to every new SLAM pose,
 * and extrapolating position from a two-sample velocity, makes head motion judder at the SLAM rate. This filter
 * instead:
 *
 * - integrates the gyro continuously, and pulls the integrated orientation towards each SLAM pose by a fraction of
 *   the error measured at the SLAM pose's own timestamp (complementary filter), snapping only on large errors;
 * - tracks position and velocity with an alpha-beta filter on the SLAM positions, and extrapolates over the full
 *   horizon from the last SLAM pose.
 *
 * Self contained (only aux_math), so the same code can run in offline tests.
 *
 * @ingroup drv_psvr2
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "util/u_time.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PSVR2_POSE_FILTER_HISTORY 1024

struct psvr2_pose_filter_params
{
	/*!
	 * Time constant for pulling the gyro-integrated orientation onto SLAM. The per-update gain is
	 * 1 - exp(-dt / tau), so behaviour doesn't depend on the (unknown) SLAM rate.
	 */
	float orient_tau_s;
	//! Orientation errors above this snap straight to SLAM (relocalization, startup).
	float orient_snap_rad;
	/*!
	 * Time constant of the position alpha-beta filter: alpha = 1 - exp(-dt / tau), with the optimal
	 * (Benedict-Bordner) beta = alpha^2 / (2 - alpha). Smaller follows faster, larger smooths more.
	 */
	float pos_tau_s;
	//! Position residuals above this reset to the SLAM position.
	float pos_snap_m;
	//! A gap between SLAM poses longer than this resets position and velocity.
	time_duration_ns pos_reset_gap_ns;
	//! Never extrapolate further than this past the latest data.
	time_duration_ns max_extrapolation_ns;
};

//! What one SLAM update did, for logging/tuning.
struct psvr2_pose_filter_update
{
	bool orientation_snapped;
	bool position_reset;
	//! Orientation error between the filter and SLAM at the SLAM timestamp, before correcting.
	float orient_error_rad;
	//! Position residual against the filter's prediction, before correcting.
	float pos_residual_m;
};

struct psvr2_pose_filter
{
	struct psvr2_pose_filter_params params;

	bool have_orientation;
	bool have_position;

	//! Orientation integrated up to @ref orient_ts.
	struct xrt_quat orientation;
	timepoint_ns orient_ts;
	struct xrt_vec3 last_gyro;
	timepoint_ns last_slam_ts;

	//! Recent integrated orientations, to compare against SLAM at its own timestamp.
	struct
	{
		timepoint_ns ts;
		struct xrt_quat orientation;
	} history[PSVR2_POSE_FILTER_HISTORY];
	uint32_t history_next;
	uint32_t history_count;

	//! Position/velocity at @ref pos_ts.
	struct xrt_vec3 position;
	struct xrt_vec3 velocity;
	timepoint_ns pos_ts;
};

void
psvr2_pose_filter_default_params(struct psvr2_pose_filter_params *out_params);

void
psvr2_pose_filter_init(struct psvr2_pose_filter *f, const struct psvr2_pose_filter_params *params);

void
psvr2_pose_filter_reset(struct psvr2_pose_filter *f);

//! Push a gyro sample (body frame, rad/s), timestamps in one consistent clock.
void
psvr2_pose_filter_push_gyro(struct psvr2_pose_filter *f, timepoint_ns ts, const struct xrt_vec3 *gyro);

//! Push a SLAM pose taken at @p ts. @p out_update may be NULL.
void
psvr2_pose_filter_push_slam(struct psvr2_pose_filter *f,
                            timepoint_ns ts,
                            const struct xrt_pose *pose,
                            struct psvr2_pose_filter_update *out_update);

//! Get the filtered pose at @p at_ts. Returns false until both a SLAM pose and gyro data have arrived.
bool
psvr2_pose_filter_get(const struct psvr2_pose_filter *f, timepoint_ns at_ts, struct xrt_space_relation *out_relation);

#ifdef __cplusplus
}
#endif
