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
	/*!
	 * Offset from a pivot (the neck) to the tracked head point, in the head frame (metres). Position is filtered as
	 * the pivot (SLAM position - R * pivot_offset), which moves slowly, and R(t) * pivot_offset is added back from
	 * the 2 kHz orientation, so head turns move the position at the gyro rate instead of in 60 Hz SLAM steps. Zero
	 * disables it.
	 */
	struct xrt_vec3 pivot_offset;
	/*!
	 * A SLAM position further than pos_snap_m from the prediction is only taken if the next one agrees with it
	 * (within 3 cm, allowing for motion); a lone jump is skipped (the prediction carries on). Off: snap at once.
	 */
	bool pos_confirm_jumps;
	/*!
	 * Blend each position correction in over this long instead of stepping at the SLAM update (0: step). The
	 * output stays continuous when a SLAM position arrives; the correction is fully in after this time.
	 */
	time_duration_ns pos_blend_ns;
	/*!
	 * Accelerometer-aided position (psvr2_pose_filter_push_accel): between SLAM poses, velocity follows the 2 kHz
	 * accelerometer (gravity and bias removed by a slow average, accel_gravity_tau_s) and SLAM only corrects drift,
	 * with alpha-beta gains from pos_tau_s. Off: the velocity comes from SLAM positions alone and steps at the SLAM
	 * rate, which SteamVR's prediction turns into position wobble while the head moves.
	 */
	bool accel_enable;
	float accel_gravity_tau_s;
	/*!
	 * Gyro bias estimation: the part of the SLAM orientation error that keeps pointing the same way is a gyro offset;
	 * it is learned with this time constant (seconds, the integral term of a Mahony-style complementary filter) and
	 * subtracted from every gyro sample. 0: off, the raw gyro is integrated and SLAM corrections undo its drift in
	 * steps at the SLAM rate.
	 */
	float gyro_bias_tau_s;
	/*!
	 * Accelerometer-aided position: don't update the gravity/bias average while turning faster than this (rad/s, 0:
	 * always update). Fast head motion disturbs the average: in replay of three 2026-11-17 recordings pausing above
	 * 30 deg/s cut the 250 ms prediction drift in fast turns from 4.8 to 2.7 mm, slow motion unchanged (0.70 mm).
	 */
	float accel_gravity_pause_rad_s;
	//! Turning speed reported with each SLAM update: max gyro rate over this long before the pose (diagnostics).
	time_duration_ns pos_rate_window_ns;
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
	//! A lone position jump was skipped, waiting for confirmation (pos_confirm_jumps).
	bool position_skipped;
	//! Turning speed around the SLAM pose's timestamp (rad/s, max over pos_rate_window_ns).
	float rate_rad_s;
};

struct psvr2_pose_filter
{
	struct psvr2_pose_filter_params params;

	bool have_orientation;
	bool have_position;

	//! pos_blend_ns: what the output was above the new estimate when the correction started, fading to 0.
	struct xrt_vec3 blend_offset;
	timepoint_ns blend_start;

	//! A position jump waiting for the next SLAM position to confirm it (pos_confirm_jumps).
	bool have_pending;
	struct xrt_vec3 pending;
	timepoint_ns pending_ts;
	uint32_t jumps_skipped;

	//! Orientation integrated up to @ref orient_ts.
	struct xrt_quat orientation;
	timepoint_ns orient_ts;
	struct xrt_vec3 last_gyro;
	timepoint_ns last_slam_ts;
	//! Estimated gyro bias (rad/s, IMU frame), subtracted from each sample (gyro_bias_tau_s).
	struct xrt_vec3 gyro_bias;

	//! Recent integrated orientations, to compare against SLAM at its own timestamp.
	struct
	{
		timepoint_ns ts;
		struct xrt_quat orientation;
		//! Gyro rate magnitude (rad/s) of the sample that brought this state, for the SLAM noise model.
		float rate;
	} history[PSVR2_POSE_FILTER_HISTORY];
	uint32_t history_next;
	uint32_t history_count;

	//! Position/velocity at @ref pos_ts.
	struct xrt_vec3 position;
	struct xrt_vec3 velocity;
	timepoint_ns pos_ts;

	//! Accelerometer-aided position (accel_enable): the IMU point's velocity, gravity estimate, pending correction.
	bool have_accel;
	struct xrt_vec3 v_imu;
	struct xrt_vec3 gravity;
	struct xrt_vec3 pending_correction;
	timepoint_ns last_slam_pos_ts;
	//! Recent pivot positions at IMU rate, to compare against SLAM at its own timestamp.
	struct
	{
		timepoint_ns ts;
		struct xrt_vec3 position;
	} pos_history[PSVR2_POSE_FILTER_HISTORY];
	uint32_t pos_history_next;
	uint32_t pos_history_count;
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

/*!
 * Feed the accelerometer sample (m/s^2, IMU/body frame, the same frame as the gyro) for the gyro sample just pushed
 * with the same @p ts. Only used with params.accel_enable.
 */
void
psvr2_pose_filter_push_accel(struct psvr2_pose_filter *f, timepoint_ns ts, const struct xrt_vec3 *accel);

//! Push a SLAM pose taken at @p ts. @p out_update may be NULL.
void
psvr2_pose_filter_push_slam(struct psvr2_pose_filter *f,
                            timepoint_ns ts,
                            const struct xrt_pose *pose,
                            struct psvr2_pose_filter_update *out_update);

//! Get the filtered pose at @p at_ts. Returns false until both a SLAM pose and gyro data have arrived.
bool
psvr2_pose_filter_get(const struct psvr2_pose_filter *f, timepoint_ns at_ts, struct xrt_space_relation *out_relation);

/*!
 * Carries the head position across the headset's SLAM resets (see psvr2_slam_carry_push).
 */
struct psvr2_slam_carry
{
	bool enabled;
	//! Added to the raw SLAM position so it continues across carried-over resets.
	struct xrt_vec3 rebase;
	struct xrt_vec3 raw_prev;
	//! Where this session's tracking started: the headset re-originates here.
	struct xrt_vec3 start;
	bool have_raw;
	timepoint_ns first_ts;
	timepoint_ns last_reset_ts;
	//! The last two output positions and times, newest first.
	struct xrt_vec3 pos[2];
	timepoint_ns ts[2];
	uint32_t count;
	//! Resets that replaced the tracking world (not carried), and carried ones.
	uint32_t world_changes;
	uint32_t carried;
};

enum psvr2_slam_carry_event
{
	PSVR2_SLAM_CARRY_NONE,
	PSVR2_SLAM_CARRY_CARRIED,
	//! Carried, but less than 1 s after the previous reset: counted as a world change too.
	PSVR2_SLAM_CARRY_CARRIED_LOOP,
	PSVR2_SLAM_CARRY_NOT_CARRIED_DISABLED,
	PSVR2_SLAM_CARRY_NOT_CARRIED_SETTLING,
};

void
psvr2_slam_carry_init(struct psvr2_slam_carry *c, bool enabled);

/*!
 * One SLAM position (headset clock @p ts). A reset is a step of at least 5 cm landing within 1 cm of where this
 * session's tracking started (the headset re-originates there; orientation carries on). After the first 20 s it is
 * carried over: the output continues from where the position was heading (last two outputs, speed clamped to 2 m/s).
 * Resets during the first 20 s, and loops (under 1 s after the previous reset, carried anyway), count as world changes.
 * Returns the position to use, @p out_event says what happened.
 */
struct xrt_vec3
psvr2_slam_carry_push(struct psvr2_slam_carry *c,
                      timepoint_ns ts,
                      const struct xrt_vec3 *raw,
                      enum psvr2_slam_carry_event *out_event,
                      float *out_jump_m);

#ifdef __cplusplus
}
#endif
