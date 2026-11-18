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
	    // 20 deg: in replay of 4 recordings 10 / 20 / 45 deg made no difference except the 18:11 run's turning p99
	    // (0.45 -> 0.35 deg at 20); SLAM orientation spikes there were 15-22 deg while the gyro was right.
	    .orient_snap_rad = (float)(20.0 * M_PI / 180.0),
	    .pos_tau_s = 0.03f,
	    .pos_snap_m = 0.05f,
	    .pos_reset_gap_ns = U_TIME_1MS_IN_NS * 200,
	    .max_extrapolation_ns = U_TIME_1MS_IN_NS * 100,
	    // 10 cm in front of the pivot: the consistent part of a lever arm fitted on 7 recordings (forward 13-19 cm;
	    // sideways and vertical varied, so they're left out). In replay of 4 recordings it cut the position error
	    // against SLAM 19-28% and turning jitter 24-31%, best of 0 / 10 / 15 / 20 cm (psvr2-test/analysis/jitter).
	    .pivot_offset = {0.0f, 0.0f, -0.10f},
	    // One SLAM interval: each correction spreads over the time until the next one. In replay of 3 recordings
	    // it cut position jitter while turning 3.5x (p50 0.108 -> 0.031 mm/frame) for ~15% more error against SLAM;
	    // 33 ms smoothed 6x for +37%, 50 ms no smoother than 33 (psvr2-test/analysis/jitter).
	    .pos_blend_ns = U_TIME_1MS_IN_NS * 17,
	    .corr_smooth_tau_s = 0.03f,
	    .accel_enable = false,
	    .accel_gravity_tau_s = 1.0f,
	    .gyro_bias_tau_s = 0.0f,
	    .accel_gravity_pause_rad_s = 0.0f,
	    .pos_rate_window_ns = U_TIME_1MS_IN_NS * 50,
	    .ref_jump_m = 0.0f,
	    .ref_jump_per_rad_s = 0.0005f,
	    .ref_offset_tau_s = 1.0f,
	    .ref_wait_ns = U_TIME_1MS_IN_NS * 40,
	    .ref_imu_decides = true,
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
	f->history[f->history_next].rate = m_vec3_len(f->last_gyro);
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

//! Highest gyro rate (rad/s) in the pos_rate_window_ns before @p ts (and a few ms after, the SLAM pose's exposure).
static float
rate_at(const struct psvr2_pose_filter *f, timepoint_ns ts)
{
	float best = m_vec3_len(f->last_gyro);
	bool any = false;
	for (uint32_t age = 0; age < f->history_count; age++) {
		uint32_t idx = history_index(f, age);
		timepoint_ns t = f->history[idx].ts;
		if (t > ts + U_TIME_1MS_IN_NS * 5) {
			continue;
		}
		if (t < ts - f->params.pos_rate_window_ns) {
			break;
		}
		if (!any || f->history[idx].rate > best) {
			best = f->history[idx].rate;
		}
		any = true;
	}
	return best;
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
psvr2_pose_filter_push_gyro(struct psvr2_pose_filter *f, timepoint_ns ts, const struct xrt_vec3 *gyro_raw)
{
	struct xrt_vec3 corrected = m_vec3_sub(*gyro_raw, f->gyro_bias);
	const struct xrt_vec3 *gyro = &corrected;
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

	/*
	 * Gyro bias (integral term): a gyro offset b makes the integrated orientation run ahead by about b * tau between
	 * corrections, so the error SLAM sees settles at -R b * tau. Learn b from it slowly; jumps (snaps) and big errors
	 * are SLAM's, not the gyro's.
	 */
	if (f->params.gyro_bias_tau_s > 0.0f && !u->orientation_snapped && dt_ns > 0 && dt_ns < U_TIME_1MS_IN_NS * 100 &&
	    angle < 0.035f) {
		struct xrt_quat inv;
		math_quat_invert(&filtered, &inv);
		struct xrt_vec3 error_body;
		math_quat_rotate_vec3(&inv, &error_aa, &error_body);
		float dt = (float)time_ns_to_s(dt_ns);
		float tau = f->params.orient_tau_s > 0.0f ? f->params.orient_tau_s : 0.15f;
		// error ~ -b * tau at steady state: step b towards -error / tau with time constant gyro_bias_tau_s.
		struct xrt_vec3 target_step = m_vec3_mul_scalar(error_body, -dt / (tau * f->params.gyro_bias_tau_s));
		f->gyro_bias = m_vec3_add(f->gyro_bias, target_step);
		float len = m_vec3_len(f->gyro_bias);
		const float max_bias = 0.1f; // 5.7 deg/s, far above the ~1 deg/s seen
		if (len > max_bias) {
			f->gyro_bias = m_vec3_mul_scalar(f->gyro_bias, max_bias / len);
		}
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

/*!
 * The filtered (pivot) position at @p at_ts: the estimate extrapolated with its velocity (capped), plus what's left
 * of a blended correction (pos_blend_ns).
 */
static bool
pos_history_at(const struct psvr2_pose_filter *f, timepoint_ns ts, struct xrt_vec3 *out);

static struct xrt_vec3
position_output(const struct psvr2_pose_filter *f, timepoint_ns at_ts)
{
	// Accelerometer-aided: a time in the past (the tracker's camera times) comes from the history.
	if (f->params.accel_enable && f->have_accel && at_ts < f->pos_ts) {
		struct xrt_vec3 past;
		if (pos_history_at(f, at_ts, &past)) {
			return past;
		}
	}
	time_duration_ns pos_dt_ns = at_ts - f->pos_ts;
	if (pos_dt_ns > f->params.max_extrapolation_ns) {
		pos_dt_ns = f->params.max_extrapolation_ns;
	}
	struct xrt_vec3 vel = f->velocity;
	if (f->params.corr_smooth_tau_s > 0.0f && f->params.accel_enable && f->have_accel) {
		vel = m_vec3_add(vel, f->corr_rate); // the correction moves the output too (corr_smooth_tau_s)
	}
	struct xrt_vec3 position = m_vec3_add(f->position, m_vec3_mul_scalar(vel, (float)time_ns_to_s(pos_dt_ns)));
	if (f->params.pos_blend_ns > 0) {
		time_duration_ns since = at_ts - f->blend_start;
		float left = since <= 0 ? 1.0f : 1.0f - (float)since / (float)f->params.pos_blend_ns;
		if (left > 0.0f) {
			position = m_vec3_add(position, m_vec3_mul_scalar(f->blend_offset, left));
		}
	}
	return position;
}

static uint32_t
pos_history_index(const struct psvr2_pose_filter *f, uint32_t age)
{
	return (f->pos_history_next + PSVR2_POSE_FILTER_HISTORY - 1 - age) % PSVR2_POSE_FILTER_HISTORY;
}

//! The accel-propagated pivot position at @p ts (from history), false when older than the history.
static bool
pos_history_at(const struct psvr2_pose_filter *f, timepoint_ns ts, struct xrt_vec3 *out)
{
	for (uint32_t age = 0; age < f->pos_history_count; age++) {
		uint32_t idx = pos_history_index(f, age);
		if (f->pos_history[idx].ts <= ts) {
			if (age == 0) {
				*out = f->pos_history[idx].position;
				return true;
			}
			uint32_t newer = pos_history_index(f, age - 1);
			timepoint_ns t0 = f->pos_history[idx].ts, t1 = f->pos_history[newer].ts;
			float t = t1 > t0 ? (float)(ts - t0) / (float)(t1 - t0) : 0.0f;
			*out = m_vec3_lerp(f->pos_history[idx].position, f->pos_history[newer].position, t);
			return true;
		}
	}
	return false;
}

static void
pos_history_shift(struct psvr2_pose_filter *f, struct xrt_vec3 d)
{
	// The referee's previous IMU position and those of updates waiting for a verdict are history points too.
	f->ref_prev_imu = m_vec3_add(f->ref_prev_imu, d);
	for (uint32_t k = 0; k < f->pend_count; k++) {
		f->pend[k].at_ts = m_vec3_add(f->pend[k].at_ts, d);
	}
	for (uint32_t age = 0; age < f->pos_history_count; age++) {
		uint32_t idx = pos_history_index(f, age);
		f->pos_history[idx].position = m_vec3_add(f->pos_history[idx].position, d);
	}
}

void
psvr2_pose_filter_push_accel(struct psvr2_pose_filter *f, timepoint_ns ts, const struct xrt_vec3 *accel)
{
	if (!f->params.accel_enable || !f->have_orientation || !f->have_position) {
		return;
	}
	// The world-frame specific force at the IMU, with the orientation the gyro sample of this ts brought.
	struct xrt_vec3 a_w;
	math_quat_rotate_vec3(&f->orientation, accel, &a_w);
	if (!f->have_accel) {
		f->have_accel = true;
		f->gravity = a_w;
		f->v_imu = f->velocity;
		f->pos_ts = ts;
		f->pending_correction = (struct xrt_vec3)XRT_VEC3_ZERO;
		f->corr_rate = (struct xrt_vec3)XRT_VEC3_ZERO;
		f->pos_history_count = 0;
		return;
	}
	time_duration_ns dt_ns = ts - f->pos_ts;
	if (dt_ns <= 0) {
		return;
	}
	if (dt_ns > U_TIME_1MS_IN_NS * 20) {
		dt_ns = U_TIME_1MS_IN_NS * 20; // a gap: don't integrate across it
	}
	float dt = (float)time_ns_to_s(dt_ns);
	/*
	 * Without SLAM the accelerometer alone drifts: in a 250 ms SLAM dropout (15:48, looking down at -80 deg pitch
	 * and shaking the head at ~140 deg/s) it built up ~1 m/s while the head stayed in place. Coast on it only as
	 * long as the SLAM-only path extrapolates (max_extrapolation_ns), then hold the position until SLAM is back.
	 */
	if (f->last_slam_pos_ts != 0 && ts - f->last_slam_pos_ts > f->params.max_extrapolation_ns) {
		f->v_imu = (struct xrt_vec3)XRT_VEC3_ZERO;
		f->velocity = (struct xrt_vec3)XRT_VEC3_ZERO;
		f->pos_ts = ts;
		f->pos_history[f->pos_history_next].ts = ts;
		f->pos_history[f->pos_history_next].position = f->position;
		f->pos_history_next = (f->pos_history_next + 1) % PSVR2_POSE_FILTER_HISTORY;
		if (f->pos_history_count < PSVR2_POSE_FILTER_HISTORY) {
			f->pos_history_count++;
		}
		return;
	}
	/*
	 * Gravity, plus the accelerometer's bias and scale error, as a slow average: a head doesn't keep accelerating
	 * one way for a second, so what stays is the offset.
	 */
	float k = dt / (f->params.accel_gravity_tau_s + dt);
	if (f->params.accel_gravity_pause_rad_s > 0.0f && m_vec3_len(f->last_gyro) > f->params.accel_gravity_pause_rad_s) {
		k = 0.0f; // fast head motion: keep the estimate from before it
	}
	f->gravity = m_vec3_add(f->gravity, m_vec3_mul_scalar(m_vec3_sub(a_w, f->gravity), k));
	struct xrt_vec3 a_lin = m_vec3_sub(a_w, f->gravity);
	f->v_imu = m_vec3_add(f->v_imu, m_vec3_mul_scalar(a_lin, dt));

	// The pivot moves with the IMU point minus the turn's lever arm velocity w x (R r).
	struct xrt_vec3 w_w, lever, lever_v;
	math_quat_rotate_vec3(&f->orientation, &f->last_gyro, &w_w);
	math_quat_rotate_vec3(&f->orientation, &f->params.pivot_offset, &lever);
	math_vec3_cross(&w_w, &lever, &lever_v);
	f->velocity = m_vec3_sub(f->v_imu, lever_v);
	f->position = m_vec3_add(f->position, m_vec3_mul_scalar(f->velocity, dt));

	// Bring the last SLAM correction in over pos_blend_ns instead of stepping.
	if (f->params.pos_blend_ns > 0 && f->params.corr_smooth_tau_s > 0.0f) {
		// The rate that would bring the rest in over pos_blend_ns, approached smoothly: no step in the rate (and so in
		// the reported velocity) when a SLAM update adds to what's pending.
		float blend_s = (float)time_ns_to_s(f->params.pos_blend_ns);
		struct xrt_vec3 target = m_vec3_mul_scalar(f->pending_correction, 1.0f / blend_s);
		float k = dt / (f->params.corr_smooth_tau_s + dt);
		f->corr_rate = m_vec3_add(f->corr_rate, m_vec3_mul_scalar(m_vec3_sub(target, f->corr_rate), k));
		struct xrt_vec3 step = m_vec3_mul_scalar(f->corr_rate, dt);
		f->position = m_vec3_add(f->position, step);
		f->pending_correction = m_vec3_sub(f->pending_correction, step);
	} else if (f->params.pos_blend_ns > 0) {
		float frac = (float)dt_ns / (float)f->params.pos_blend_ns;
		if (frac > 1.0f) {
			frac = 1.0f;
		}
		struct xrt_vec3 step = m_vec3_mul_scalar(f->pending_correction, frac);
		f->position = m_vec3_add(f->position, step);
		f->pending_correction = m_vec3_sub(f->pending_correction, step);
	} else {
		// No blending: take the correction at once (it used to be left pending forever, so position drifted).
		f->position = m_vec3_add(f->position, f->pending_correction);
		f->pending_correction = (struct xrt_vec3)XRT_VEC3_ZERO;
		f->corr_rate = (struct xrt_vec3)XRT_VEC3_ZERO;
	}
	f->pos_ts = ts;

	f->pos_history[f->pos_history_next].ts = ts;
	f->pos_history[f->pos_history_next].position = f->position;
	f->pos_history_next = (f->pos_history_next + 1) % PSVR2_POSE_FILTER_HISTORY;
	if (f->pos_history_count < PSVR2_POSE_FILTER_HISTORY) {
		f->pos_history_count++;
	}
}

//! The referee pivot at @p ts (interpolated), false if not covered.
static bool
ref_at(const struct psvr2_pose_filter *f, timepoint_ns ts, struct xrt_vec3 *out)
{
	for (uint32_t age = 0; age + 1 < f->ref_count; age++) {
		uint32_t i1 = (f->ref_next + 64 - 1 - age) % 64, i0 = (f->ref_next + 64 - 2 - age) % 64;
		if (f->ref[i0].ts <= ts && ts <= f->ref[i1].ts) {
			float u = f->ref[i1].ts > f->ref[i0].ts ? (float)(ts - f->ref[i0].ts) / (float)(f->ref[i1].ts - f->ref[i0].ts) : 0.0f;
			*out = m_vec3_lerp(f->ref[i0].pivot, f->ref[i1].pivot, u);
			return true;
		}
	}
	return false;
}


/*!
 * Referee verdicts for applied SLAM updates whose referee pose has now arrived (see ref_jump_m). A jump: the referee's
 * step replaces SLAM's (offset), and the correction the jumped update applied is undone.
 */
static void
ref_verdicts(struct psvr2_pose_filter *f, struct psvr2_pose_filter_update *u)
{
	timepoint_ns newest_ref = f->ref_count > 0 ? f->ref[(f->ref_next + 63) % 64].ts : 0;
	while (f->pend_count > 0) {
		timepoint_ns ts = f->pend[0].ts;
		bool too_old = f->orient_ts - ts > f->params.ref_wait_ns;
		if (newest_ref < ts && !too_old) {
			break;
		}
		struct xrt_vec3 r_now, r_prev;
		bool have = !too_old && f->have_ref_prev && ts > f->ref_prev_ts && ts - f->ref_prev_ts < U_TIME_1MS_IN_NS * 100 &&
		            ref_at(f, ts, &r_now) && ref_at(f, f->ref_prev_ts, &r_prev);
		if (have) {
			float dt_s = (float)time_ns_to_s(ts - f->ref_prev_ts);
			float decay = f->params.ref_offset_tau_s > 0.0f ? expf(-dt_s / f->params.ref_offset_tau_s) : 0.0f;
			struct xrt_vec3 d_slam = m_vec3_sub(f->pend[0].slam, f->ref_prev_slam);
			struct xrt_vec3 d_ref = m_vec3_sub(r_now, r_prev);
			struct xrt_vec3 d_imu = m_vec3_sub(f->pend[0].at_ts, f->ref_prev_imu);
			struct xrt_vec3 dis = m_vec3_sub(d_slam, d_ref);
			float disagree = m_vec3_len(dis), vs_imu = m_vec3_len(m_vec3_sub(d_ref, d_imu));
			float thr = f->params.ref_jump_m + f->params.ref_jump_per_rad_s * f->pend[0].rate;
			if (u != NULL) {
				u->ref_disagree_m = disagree;
				u->ref_vs_imu_m = vs_imu;
			}
			f->ref_offset = m_vec3_mul_scalar(f->ref_offset, decay);
			struct xrt_vec3 slam_vs_imu = m_vec3_sub(d_slam, d_imu);
			bool jump = f->params.ref_imu_decides ? (m_vec3_len(slam_vs_imu) > thr && vs_imu < 0.5f * thr)
			                                      : (disagree > thr && vs_imu < thr);
			if (jump) {
				f->ref_offset = m_vec3_add(f->ref_offset, f->params.ref_imu_decides ? slam_vs_imu : dis);
				dis = f->params.ref_imu_decides ? slam_vs_imu : dis;
				f->ref_replacements++;
				if (u != NULL) {
					u->ref_replaced = true;
				}
				// Undo what the jump did: this update, and any later one already applied with the old offset.
				for (uint32_t k = 0; k < f->pend_count; k++) {
					if (f->pend[k].alpha <= 0.0f) {
						continue;
					}
					struct xrt_vec3 corr = m_vec3_mul_scalar(dis, -f->pend[k].alpha);
					f->pending_correction = m_vec3_add(f->pending_correction, corr);
					pos_history_shift(f, corr);
					f->v_imu = m_vec3_add(f->v_imu, m_vec3_mul_scalar(dis, -f->pend[k].beta / f->pend[k].dt));
				}
			}
		} else if (!f->have_ref_prev || ts - f->ref_prev_ts >= U_TIME_1MS_IN_NS * 100) {
			f->ref_offset = (struct xrt_vec3)XRT_VEC3_ZERO; // a gap: nothing to compare against
		}
		f->have_ref_prev = true;
		f->ref_prev_ts = ts;
		f->ref_prev_slam = f->pend[0].slam;
		f->ref_prev_imu = f->pend[0].at_ts;
		memmove(&f->pend[0], &f->pend[1], sizeof(f->pend[0]) * (f->pend_count - 1));
		f->pend_count--;
	}
}

//! A SLAM pivot position with accelerometer-aided propagation (accel_enable).
static void
update_position_accel(struct psvr2_pose_filter *f,
                      timepoint_ns ts,
                      const struct xrt_vec3 *slam,
                      struct psvr2_pose_filter_update *u)
{
	struct xrt_vec3 at_ts;
	if (!pos_history_at(f, ts, &at_ts)) {
		return; // older than the history (a stall): the next one compares
	}
	struct xrt_vec3 slam_checked;
	int pend_idx = -1;
	if (f->params.ref_jump_m > 0.0f) {
		// Remember this update for the referee's verdict (it comes when the referee pose of this time arrives).
		if (f->pend_count == 8) {
			memmove(&f->pend[0], &f->pend[1], sizeof(f->pend[0]) * 7);
			f->pend_count--;
		}
		pend_idx = (int)f->pend_count++;
		f->pend[pend_idx].ts = ts;
		f->pend[pend_idx].slam = *slam;
		f->pend[pend_idx].at_ts = at_ts;
		f->pend[pend_idx].alpha = 0.0f;
		f->pend[pend_idx].rate = u->rate_rad_s;
		slam_checked = m_vec3_sub(*slam, f->ref_offset);
		slam = &slam_checked;
	}

	// The correction still being blended in counts as already applied.
	struct xrt_vec3 residual = m_vec3_sub(m_vec3_sub(*slam, at_ts), f->pending_correction);
	u->pos_residual_m = m_vec3_len(residual);
	float dt = f->last_slam_pos_ts != 0 ? (float)time_ns_to_s(ts - f->last_slam_pos_ts) : 0.0f;
	f->last_slam_pos_ts = ts;

	if (u->pos_residual_m > f->params.pos_snap_m && f->params.pos_confirm_jumps && dt > 0.0f) {
		// As without the accelerometer: a lone jump waits for the next SLAM position to confirm it.
		bool confirmed = f->have_pending && ts > f->pending_ts &&
		                 m_vec3_len(m_vec3_sub(*slam, f->pending)) <
		                     0.03f + m_vec3_len(f->velocity) * (float)time_ns_to_s(ts - f->pending_ts);
		if (!confirmed) {
			f->have_pending = true;
			f->pending = *slam;
			f->pending_ts = ts;
			f->jumps_skipped++;
			u->position_skipped = true;
			return;
		}
	}
	f->have_pending = false;
	if (u->pos_residual_m > f->params.pos_snap_m || dt <= 0.0f || dt > time_ns_to_s(f->params.pos_reset_gap_ns)) {
		// Jump, start or a long gap: take SLAM (shifted to now) and restart the velocity.
		struct xrt_vec3 d = m_vec3_add(residual, f->pending_correction);
		f->position = m_vec3_add(f->position, d);
		pos_history_shift(f, d);
		f->pending_correction = (struct xrt_vec3)XRT_VEC3_ZERO;
		f->corr_rate = (struct xrt_vec3)XRT_VEC3_ZERO;
		f->v_imu = (struct xrt_vec3)XRT_VEC3_ZERO;
		u->position_reset = true;
		return;
	}
	float alpha = f->params.pos_tau_s > 0.0f ? 1.0f - expf(-dt / f->params.pos_tau_s) : 1.0f;
	float beta = alpha * alpha / (2.0f - alpha);
	struct xrt_vec3 corr = m_vec3_mul_scalar(residual, alpha);
	if (pend_idx >= 0) {
		f->pend[pend_idx].alpha = alpha;
		f->pend[pend_idx].beta = beta;
		f->pend[pend_idx].dt = dt;
	}
	f->pending_correction = m_vec3_add(f->pending_correction, corr);
	// History as if already corrected, so the next comparison doesn't see this again.
	pos_history_shift(f, corr);
	f->v_imu = m_vec3_add(f->v_imu, m_vec3_mul_scalar(residual, beta / dt));
}

static void
update_position(struct psvr2_pose_filter *f,
                timepoint_ns ts,
                const struct xrt_vec3 *slam,
                struct psvr2_pose_filter_update *u)
{
	if (f->params.accel_enable && f->have_accel) {
		update_position_accel(f, ts, slam, u);
		return;
	}

	time_duration_ns dt_ns = ts - f->pos_ts;

	if (!f->have_position || dt_ns > f->params.pos_reset_gap_ns) {
		f->position = *slam;
		f->velocity = (struct xrt_vec3)XRT_VEC3_ZERO;
		f->pos_ts = ts;
		f->have_position = true;
		f->blend_offset = (struct xrt_vec3)XRT_VEC3_ZERO;
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

	if (u->pos_residual_m > f->params.pos_snap_m && f->params.pos_confirm_jumps) {
		// Take a jump only once the next SLAM position confirms it (it lands near where the first one pointed).
		bool confirmed = f->have_pending && ts > f->pending_ts &&
		                 m_vec3_len(m_vec3_sub(*slam, f->pending)) <
		                     0.03f + m_vec3_len(f->velocity) * (float)time_ns_to_s(ts - f->pending_ts);
		if (!confirmed) {
			f->have_pending = true;
			f->pending = *slam;
			f->pending_ts = ts;
			f->jumps_skipped++;
			u->position_skipped = true;
			return; // keep the prediction (pos_ts unchanged)
		}
		f->have_pending = false;
		f->position = *slam;
		f->velocity = (struct xrt_vec3)XRT_VEC3_ZERO;
		u->position_reset = true;
	} else if (u->pos_residual_m > f->params.pos_snap_m) {
		f->position = *slam;
		f->velocity = (struct xrt_vec3)XRT_VEC3_ZERO;
		u->position_reset = true;
	} else {
		f->have_pending = false;
		float alpha = f->params.pos_tau_s > 0.0f ? 1.0f - expf(-dt / f->params.pos_tau_s) : 1.0f;
		float beta = alpha * alpha / (2.0f - alpha);

		// The output "now" (the newest gyro sample, about when this SLAM pose arrived; SLAM poses come ~8 ms
		// after their own timestamp) before the correction, to fade from, old blend included so back-to-back
		// corrections stay continuous.
		timepoint_ns now = f->orient_ts > ts ? f->orient_ts : ts;
		struct xrt_vec3 before = position_output(f, now);

		f->position = m_vec3_add(predicted, m_vec3_mul_scalar(residual, alpha));
		f->velocity = m_vec3_add(f->velocity, m_vec3_mul_scalar(residual, beta / dt));
		f->pos_ts = ts;

		if (f->params.pos_blend_ns > 0) {
			f->blend_offset = (struct xrt_vec3)XRT_VEC3_ZERO;
			f->blend_start = now;
			f->blend_offset = m_vec3_sub(before, position_output(f, now));
		}
		return;
	}
	f->blend_offset = (struct xrt_vec3)XRT_VEC3_ZERO; // snaps and resets jump on purpose
	f->pos_ts = ts;
}


void
psvr2_pose_filter_push_ref(struct psvr2_pose_filter *f, timepoint_ns ts, const struct xrt_pose *pose)
{
	struct xrt_vec3 lever;
	math_quat_rotate_vec3(&pose->orientation, &f->params.pivot_offset, &lever);
	f->ref[f->ref_next].ts = ts;
	f->ref[f->ref_next].pivot = m_vec3_sub(pose->position, lever);
	f->ref_next = (f->ref_next + 1) % 64;
	if (f->ref_count < 64) {
		f->ref_count++;
	}
	if (f->params.ref_jump_m > 0.0f && f->params.accel_enable) {
		ref_verdicts(f, NULL);
	}
}

void
psvr2_pose_filter_push_slam(struct psvr2_pose_filter *f,
                            timepoint_ns ts,
                            const struct xrt_pose *pose,
                            struct psvr2_pose_filter_update *out_update)
{
	struct psvr2_pose_filter_update u = {0};
	u.rate_rad_s = rate_at(f, ts);
	update_orientation(f, ts, &pose->orientation, &u);
	// Filter the pivot: the head point minus the turn's lever arm (at this SLAM pose's own orientation).
	struct xrt_vec3 lever;
	math_quat_rotate_vec3(&pose->orientation, &f->params.pivot_offset, &lever);
	struct xrt_vec3 pivot = m_vec3_sub(pose->position, lever);
	update_position(f, ts, &pivot, &u);
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

	struct xrt_vec3 position = position_output(f, at_ts);

	// Angular velocity in the base (world) frame.
	struct xrt_vec3 angular_velocity;
	math_quat_rotate_vec3(&orientation, &f->last_gyro, &angular_velocity);

	// Back from the pivot to the head point: the lever arm at the current orientation, and its velocity w x (R r).
	struct xrt_vec3 lever, lever_velocity;
	math_quat_rotate_vec3(&orientation, &f->params.pivot_offset, &lever);
	math_vec3_cross(&angular_velocity, &lever, &lever_velocity);
	position = m_vec3_add(position, lever);

	*out_relation = (struct xrt_space_relation){
	    .pose = {.orientation = orientation, .position = position},
	    .linear_velocity = m_vec3_add(f->params.corr_smooth_tau_s > 0.0f && f->params.accel_enable && f->have_accel
	                                      ? m_vec3_add(f->velocity, f->corr_rate)
	                                      : f->velocity,
	                                  lever_velocity),
	    .angular_velocity = angular_velocity,
	    .relation_flags = (enum xrt_space_relation_flags)(
	        XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	        XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
	        XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT),
	};
	return true;
}

void
psvr2_slam_carry_init(struct psvr2_slam_carry *c, bool enabled)
{
	*c = (struct psvr2_slam_carry){.enabled = enabled};
}

struct xrt_vec3
psvr2_slam_carry_push(struct psvr2_slam_carry *c,
                      timepoint_ns ts,
                      const struct xrt_vec3 *raw,
                      enum psvr2_slam_carry_event *out_event,
                      float *out_jump_m)
{
	enum psvr2_slam_carry_event event = PSVR2_SLAM_CARRY_NONE;
	if (!c->have_raw) {
		// The headset re-originates at where this session's tracking started, which isn't always (0, 0, 0): after
		// Sony's driver ran it was (-25.0, -6.4, -35.2) m (18:11 and 18:33 runs).
		c->start = *raw;
	}
	float jump = c->have_raw ? m_vec3_len(m_vec3_sub(*raw, c->raw_prev)) : 0.0f;
	bool reset = c->have_raw && m_vec3_len(m_vec3_sub(*raw, c->start)) < 0.01f && jump > 0.05f;
	c->raw_prev = *raw;
	c->have_raw = true;
	if (c->first_ts == 0) {
		c->first_ts = ts;
	}

	if (reset) {
		bool settled = ts - c->first_ts >= 20 * U_TIME_1S_IN_NS;
		bool spaced = c->last_reset_ts == 0 || ts - c->last_reset_ts >= U_TIME_1S_IN_NS;
		if (c->enabled && settled && c->count >= 2) {
			float dt_prev = (float)time_ns_to_s(c->ts[0] - c->ts[1]);
			struct xrt_vec3 v = {0, 0, 0};
			if (dt_prev > 0.0f) {
				v = m_vec3_mul_scalar(m_vec3_sub(c->pos[0], c->pos[1]), 1.0f / dt_prev);
			}
			float speed = m_vec3_len(v);
			if (speed > 2.0f) {
				v = m_vec3_mul_scalar(v, 2.0f / speed);
			}
			struct xrt_vec3 predicted =
			    m_vec3_add(c->pos[0], m_vec3_mul_scalar(v, (float)time_ns_to_s(ts - c->ts[0])));
			c->rebase = m_vec3_sub(predicted, *raw);
			c->carried++;
			event = PSVR2_SLAM_CARRY_CARRIED;
			if (!spaced) {
				// A reset loop: carried too (in replay of the 18:33 run, jumps over 5 cm 10 -> 3 instead of 5),
				// but the position may have drifted, so the world counts as replaced for the room calibration.
				c->world_changes++;
				event = PSVR2_SLAM_CARRY_CARRIED_LOOP;
			}
		} else {
			c->world_changes++;
			event = !c->enabled ? PSVR2_SLAM_CARRY_NOT_CARRIED_DISABLED : PSVR2_SLAM_CARRY_NOT_CARRIED_SETTLING;
		}
		c->last_reset_ts = ts;
	}

	struct xrt_vec3 out = m_vec3_add(*raw, c->rebase);
	c->pos[1] = c->pos[0];
	c->ts[1] = c->ts[0];
	c->pos[0] = out;
	c->ts[0] = ts;
	if (c->count < 2) {
		c->count++;
	}
	if (out_event != NULL) {
		*out_event = event;
	}
	if (out_jump_m != NULL) {
		*out_jump_m = jump;
	}
	return out;
}
