// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS VR2 headset pose filter, see @ref psvr2_pose_filter.h.
 * @ingroup drv_psvr2
 */

#include "psvr2_pose_filter.h"

#include "math/m_api.h"
#include "math/m_vec3.h"
#include "math/m_mathinclude.h"

#include <string.h>

void
psvr2_pose_filter_default_params(struct psvr2_pose_filter_params *out_params)
{
	*out_params = (struct psvr2_pose_filter_params){
	    .orient_tau_s = 0.15f,
	    .orient_snap_rad = (float)(10.0 * M_PI / 180.0),
	    .pos_tau_s = 0.03f,
	    .pos_snap_m = 0.05f,
	    .pos_reset_gap_ns = U_TIME_1MS_IN_NS * 200,
	    .max_extrapolation_ns = U_TIME_1MS_IN_NS * 100,
	};
}

void
psvr2_pose_filter_init(struct psvr2_pose_filter *f, const struct psvr2_pose_filter_params *params)
{
	memset(f, 0, sizeof(*f));
	f->params = *params;
	psvr2_pose_filter_reset(f);
}

void
psvr2_pose_filter_reset(struct psvr2_pose_filter *f)
{
	struct psvr2_pose_filter_params params = f->params;
	memset(f, 0, sizeof(*f));
	f->params = params;
	f->orientation = (struct xrt_quat)XRT_QUAT_IDENTITY;
}

static void
quat_normalize_positive_w(struct xrt_quat *q)
{
	math_quat_normalize(q);
	if (q->w < 0.0f) {
		q->x = -q->x;
		q->y = -q->y;
		q->z = -q->z;
		q->w = -q->w;
	}
}

//! orientation = orientation * exp(gyro * dt), gyro in the body frame.
static void
integrate_gyro(struct xrt_quat *orientation, const struct xrt_vec3 *gyro, double dt_s)
{
	struct xrt_vec3 scaled = m_vec3_mul_scalar(*gyro, (float)dt_s);
	struct xrt_quat delta;
	math_quat_exp_so3(&scaled, &delta);
	math_quat_rotate(orientation, &delta, orientation);
	math_quat_normalize(orientation);
}

static void
history_push(struct psvr2_pose_filter *f, timepoint_ns ts, const struct xrt_quat *orientation)
{
	f->history[f->history_next].ts = ts;
	f->history[f->history_next].orientation = *orientation;
	f->history_next = (f->history_next + 1) % PSVR2_POSE_FILTER_HISTORY;
	if (f->history_count < PSVR2_POSE_FILTER_HISTORY) {
		f->history_count++;
	}
}

//! Index 0 is the newest entry.
static uint32_t
history_index(const struct psvr2_pose_filter *f, uint32_t age)
{
	return (f->history_next + PSVR2_POSE_FILTER_HISTORY - 1 - age) % PSVR2_POSE_FILTER_HISTORY;
}

/*!
 * The integrated orientation at @p ts: interpolated from history, extrapolated with the last gyro sample when newer
 * than the history, or the oldest entry when older.
 */
static void
orientation_at(const struct psvr2_pose_filter *f, timepoint_ns ts, struct xrt_quat *out)
{
	if (f->history_count == 0 || ts >= f->orient_ts) {
		*out = f->orientation;
		time_duration_ns ahead = ts - f->orient_ts;
		if (ahead > 0) {
			if (ahead > f->params.max_extrapolation_ns) {
				ahead = f->params.max_extrapolation_ns;
			}
			integrate_gyro(out, &f->last_gyro, time_ns_to_s(ahead));
		}
		return;
	}

	for (uint32_t age = 0; age < f->history_count; age++) {
		uint32_t idx = history_index(f, age);
		if (f->history[idx].ts <= ts) {
			if (age == 0) {
				*out = f->history[idx].orientation;
				return;
			}
			uint32_t newer = history_index(f, age - 1);
			timepoint_ns t0 = f->history[idx].ts;
			timepoint_ns t1 = f->history[newer].ts;
			float t = t1 > t0 ? (float)(ts - t0) / (float)(t1 - t0) : 0.0f;
			math_quat_slerp(&f->history[idx].orientation, &f->history[newer].orientation, t, out);
			return;
		}
	}

	*out = f->history[history_index(f, f->history_count - 1)].orientation;
}

void
psvr2_pose_filter_push_gyro(struct psvr2_pose_filter *f, timepoint_ns ts, const struct xrt_vec3 *gyro)
{
	if (!f->have_orientation) {
		// Nothing to integrate onto yet, but remember the rate for extrapolation.
		f->last_gyro = *gyro;
		return;
	}

	if (ts <= f->orient_ts) {
		return; // Older than the state (e.g. right after a SLAM snap), already covered.
	}

	// Same zero-order hold as t_apply_dead_reckoning: each sample covers the interval before it.
	integrate_gyro(&f->orientation, gyro, time_ns_to_s(ts - f->orient_ts));
	f->orient_ts = ts;
	f->last_gyro = *gyro;
	history_push(f, ts, &f->orientation);
}

static void
apply_world_correction(struct psvr2_pose_filter *f, const struct xrt_quat *correction)
{
	math_quat_rotate(correction, &f->orientation, &f->orientation);
	math_quat_normalize(&f->orientation);

	// Keep history consistent, so the next SLAM comparison doesn't see this correction as error again.
	for (uint32_t i = 0; i < f->history_count; i++) {
		uint32_t idx = history_index(f, i);
		math_quat_rotate(correction, &f->history[idx].orientation, &f->history[idx].orientation);
		math_quat_normalize(&f->history[idx].orientation);
	}
}

static void
update_orientation(struct psvr2_pose_filter *f,
                   timepoint_ns ts,
                   const struct xrt_quat *slam,
                   struct psvr2_pose_filter_update *u)
{
	if (!f->have_orientation) {
		f->orientation = *slam;
		math_quat_normalize(&f->orientation);
		f->orient_ts = ts;
		f->have_orientation = true;
		f->history_count = 0;
		f->history_next = 0;
		history_push(f, ts, &f->orientation);
		u->orientation_snapped = true;
		return;
	}

	struct xrt_quat filtered;
	orientation_at(f, ts, &filtered);

	// World-frame error: slam = error * filtered
	struct xrt_quat filtered_inv, error;
	math_quat_invert(&filtered, &filtered_inv);
	math_quat_rotate(slam, &filtered_inv, &error);
	quat_normalize_positive_w(&error);

	struct xrt_vec3 error_aa;
	math_quat_ln_so3(&error, &error_aa);
	float angle = m_vec3_len(error_aa);
	u->orient_error_rad = angle;

	float gain = 1.0f;
	time_duration_ns dt_ns = ts - f->last_slam_ts;
	if (f->last_slam_ts != 0 && dt_ns > 0 && f->params.orient_tau_s > 0.0f) {
		gain = 1.0f - expf(-(float)time_ns_to_s(dt_ns) / f->params.orient_tau_s);
	}
	f->last_slam_ts = ts;
	if (angle > f->params.orient_snap_rad) {
		gain = 1.0f;
		u->orientation_snapped = true;
	}

	struct xrt_vec3 scaled = m_vec3_mul_scalar(error_aa, gain);
	struct xrt_quat correction;
	math_quat_exp_so3(&scaled, &correction);
	apply_world_correction(f, &correction);

	if (ts > f->orient_ts) {
		// SLAM is newer than any gyro sample, move the state up to it.
		f->orient_ts = ts;
		history_push(f, ts, &f->orientation);
	}
}

static void
update_position(struct psvr2_pose_filter *f,
                timepoint_ns ts,
                const struct xrt_vec3 *slam,
                struct psvr2_pose_filter_update *u)
{
	time_duration_ns dt_ns = ts - f->pos_ts;

	if (!f->have_position || dt_ns > f->params.pos_reset_gap_ns) {
		f->position = *slam;
		f->velocity = (struct xrt_vec3)XRT_VEC3_ZERO;
		f->pos_ts = ts;
		f->have_position = true;
		u->position_reset = true;
		return;
	}

	if (dt_ns <= 0) {
		return; // Out of order or duplicate.
	}

	float dt = (float)time_ns_to_s(dt_ns);
	struct xrt_vec3 predicted = m_vec3_add(f->position, m_vec3_mul_scalar(f->velocity, dt));
	struct xrt_vec3 residual = m_vec3_sub(*slam, predicted);
	u->pos_residual_m = m_vec3_len(residual);

	if (u->pos_residual_m > f->params.pos_snap_m) {
		f->position = *slam;
		f->velocity = (struct xrt_vec3)XRT_VEC3_ZERO;
		u->position_reset = true;
	} else {
		float alpha = f->params.pos_tau_s > 0.0f ? 1.0f - expf(-dt / f->params.pos_tau_s) : 1.0f;
		float beta = alpha * alpha / (2.0f - alpha);
		f->position = m_vec3_add(predicted, m_vec3_mul_scalar(residual, alpha));
		f->velocity = m_vec3_add(f->velocity, m_vec3_mul_scalar(residual, beta / dt));
	}
	f->pos_ts = ts;
}

void
psvr2_pose_filter_push_slam(struct psvr2_pose_filter *f,
                            timepoint_ns ts,
                            const struct xrt_pose *pose,
                            struct psvr2_pose_filter_update *out_update)
{
	struct psvr2_pose_filter_update u = {0};
	update_orientation(f, ts, &pose->orientation, &u);
	update_position(f, ts, &pose->position, &u);
	if (out_update != NULL) {
		*out_update = u;
	}
}

bool
psvr2_pose_filter_get(const struct psvr2_pose_filter *f, timepoint_ns at_ts, struct xrt_space_relation *out_relation)
{
	if (!f->have_orientation || !f->have_position) {
		return false;
	}

	struct xrt_quat orientation;
	orientation_at(f, at_ts, &orientation);

	time_duration_ns pos_dt_ns = at_ts - f->pos_ts;
	if (pos_dt_ns > f->params.max_extrapolation_ns) {
		pos_dt_ns = f->params.max_extrapolation_ns;
	}
	struct xrt_vec3 position =
	    m_vec3_add(f->position, m_vec3_mul_scalar(f->velocity, (float)time_ns_to_s(pos_dt_ns)));

	// Angular velocity in the base (world) frame.
	struct xrt_vec3 angular_velocity;
	math_quat_rotate_vec3(&orientation, &f->last_gyro, &angular_velocity);

	*out_relation = (struct xrt_space_relation){
	    .pose = {.orientation = orientation, .position = position},
	    .linear_velocity = f->velocity,
	    .angular_velocity = angular_velocity,
	    .relation_flags = (enum xrt_space_relation_flags)(
	        XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	        XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
	        XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT),
	};
	return true;
}
