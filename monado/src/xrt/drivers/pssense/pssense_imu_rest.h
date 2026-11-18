// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Sense controller rest detection: gyro bias estimate and initial leveling of the 3DoF fusion.
 *
 * The factory calibration leaves each controller's gyro with a fixed bias: measured at rest in six recordings
 * (2026-11-16/17), left [-0.08 0.12 -0.27] deg/s (0.3 deg/s) and right [0.22 -0.64 0.67] deg/s (0.95 deg/s), the same
 * in every session. m_imu_3dof only subtracts a bias when asked to (gyro_bias.manually_fire), which nothing did, and
 * corrects tilt only while the controller is still; in play it is never still, so the bias turned the right
 * controller's orientation 3 -> 66 deg in a minute (sense-20261117-000628).
 *
 * The fusion also starts at identity and levels at 3 deg/s while still, 36 deg off for the first ~10 s.
 *
 * Rest is judged from 0.5 s windows by the spread of the samples, which a gyro bias does not affect: accel standard
 * deviation under 0.05 m/s^2 and gyro under 0.3 deg/s on every axis, mean accel within 0.3 m/s^2 of gravity. A rest
 * window's mean gyro is the bias. The first rest window also levels the fusion (tilt only, yaw kept).
 *
 * @ingroup drv_pssense
 */

#pragma once

#include "math/m_api.h"
#include "math/m_imu_3dof.h"
#include "math/m_vec3.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define PSSENSE_REST_WINDOW_NS (500 * 1000 * 1000LL)
#define PSSENSE_REST_MIN_SAMPLES 50
#define PSSENSE_REST_ACCEL_STD 0.05f
#define PSSENSE_REST_GYRO_STD (0.3f * (float)M_PI / 180.0f)
#define PSSENSE_REST_GRAVITY_TOL 0.3f
//! Larger than any bias seen (1 deg/s) by far; a mean above this is slow turning, not rest.
#define PSSENSE_REST_MAX_BIAS (5.0f * (float)M_PI / 180.0f)
//! How far each later rest window moves the bias (follows slow temperature drift).
#define PSSENSE_REST_BIAS_GAIN 0.3f

struct pssense_imu_rest
{
	int64_t window_start_ns;
	uint32_t n;
	double sum_g[3], sum_g2[3], sum_a[3], sum_a2[3];

	bool bias_valid;
	struct xrt_vec3 bias;
	bool leveled;
	uint32_t rest_windows;
};

static inline void
pssense_imu_rest_window_reset(struct pssense_imu_rest *r, int64_t ts_ns)
{
	r->window_start_ns = ts_ns;
	r->n = 0;
	for (int i = 0; i < 3; i++) {
		r->sum_g[i] = r->sum_g2[i] = r->sum_a[i] = r->sum_a2[i] = 0.0;
	}
}

/*!
 * Rotate @p f's orientation by the smallest rotation that brings the measured gravity direction to world up.
 * @p accel_mean is the body-frame accelerometer mean at rest (points up).
 */
static inline void
pssense_imu_rest_level(struct m_imu_3dof *f, const struct xrt_vec3 *accel_mean)
{
	struct xrt_vec3 a_world;
	math_quat_rotate_vec3(&f->rot, accel_mean, &a_world);
	a_world = m_vec3_normalize(a_world);
	struct xrt_vec3 up = {0.0f, 1.0f, 0.0f};

	struct xrt_quat corr;
	math_quat_from_vec_a_to_vec_b(&a_world, &up, &corr);

	struct xrt_quat out;
	math_quat_rotate(&corr, &f->rot, &out);
	math_quat_normalize(&out);
	f->rot = out;
	// Anything the gravity correction still had queued is measured against the old orientation.
	f->grav.error_angle = 0.0f;
}

/*!
 * Feed one calibrated sample (gyro rad/s, accel m/s^2, both before any bias removal).
 * Updates @p f's gyro bias when a rest window closes, and levels @p f on the first one.
 *
 * @return true when a rest window closed on this sample.
 */
static inline bool
pssense_imu_rest_push(struct pssense_imu_rest *r,
                      struct m_imu_3dof *f,
                      int64_t ts_ns,
                      const struct xrt_vec3 *gyro,
                      const struct xrt_vec3 *accel)
{
	if (r->n == 0 || ts_ns < r->window_start_ns) {
		pssense_imu_rest_window_reset(r, ts_ns);
	}

	const float g[3] = {gyro->x, gyro->y, gyro->z};
	const float a[3] = {accel->x, accel->y, accel->z};
	for (int i = 0; i < 3; i++) {
		r->sum_g[i] += g[i];
		r->sum_g2[i] += (double)g[i] * g[i];
		r->sum_a[i] += a[i];
		r->sum_a2[i] += (double)a[i] * a[i];
	}
	r->n++;

	if (ts_ns - r->window_start_ns < PSSENSE_REST_WINDOW_NS) {
		return false;
	}

	bool rest = r->n >= PSSENSE_REST_MIN_SAMPLES;
	double mg[3], ma[3];
	for (int i = 0; i < 3 && rest; i++) {
		mg[i] = r->sum_g[i] / r->n;
		ma[i] = r->sum_a[i] / r->n;
		double vg = r->sum_g2[i] / r->n - mg[i] * mg[i];
		double va = r->sum_a2[i] / r->n - ma[i] * ma[i];
		if (va > (double)PSSENSE_REST_ACCEL_STD * PSSENSE_REST_ACCEL_STD ||
		    vg > (double)PSSENSE_REST_GYRO_STD * PSSENSE_REST_GYRO_STD ||
		    fabs(mg[i]) > (double)PSSENSE_REST_MAX_BIAS) {
			rest = false;
		}
	}
	if (rest) {
		double alen = sqrt(ma[0] * ma[0] + ma[1] * ma[1] + ma[2] * ma[2]);
		if (fabs(alen - MATH_GRAVITY_M_S2) > PSSENSE_REST_GRAVITY_TOL) {
			rest = false;
		}
	}
	pssense_imu_rest_window_reset(r, ts_ns);
	if (!rest) {
		return false;
	}

	struct xrt_vec3 mean_g = {(float)mg[0], (float)mg[1], (float)mg[2]};
	if (!r->bias_valid) {
		r->bias = mean_g;
		r->bias_valid = true;
	} else {
		r->bias.x += PSSENSE_REST_BIAS_GAIN * (mean_g.x - r->bias.x);
		r->bias.y += PSSENSE_REST_BIAS_GAIN * (mean_g.y - r->bias.y);
		r->bias.z += PSSENSE_REST_BIAS_GAIN * (mean_g.z - r->bias.z);
	}
	f->gyro_bias.value = r->bias;
	r->rest_windows++;

	if (!r->leveled) {
		struct xrt_vec3 mean_a = {(float)ma[0], (float)ma[1], (float)ma[2]};
		pssense_imu_rest_level(f, &mean_a);
		r->leveled = true;
	}
	return true;
}
