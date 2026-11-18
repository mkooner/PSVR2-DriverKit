// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Basalt visual-inertial tracking of the PS VR2 headset, see psvr2_basalt.h.
 * @ingroup drv_psvr2
 */

#include "psvr2_basalt.h"
#include "xrt/xrt_config_build.h"
#include "psvr2_cam_mount.h"

#include "math/m_api.h"
#include "math/m_vec3.h"
#include "os/os_threading.h"
#include "tracking/t_camera_models.h"
#include "tracking/t_tracking.h"
#include "util/u_frame.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "xrt/xrt_byte_order.h"
#include "xrt/xrt_tracking.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#define B_INFO(...) U_LOG_I("psvr2_basalt: " __VA_ARGS__)
#define B_WARN(...) U_LOG_W("psvr2_basalt: " __VA_ARGS__)

#define IMG_W 1016
#define IMG_STRIDE 1024
#define IMG_BYTES (IMG_STRIDE * IMG_W / 2)
#define XFER_BYTES (256 + 2 * IMG_BYTES)

//! Basalt pipeline settings: the Monado fork's msdmi config (its own headsets' datasets), optical flow safe radius 490
//! for the 1016 px fisheye and matching depth 1 m. Validated offline (psvr2-test/vio/psvr2_basalt_config.json).
static const char *const BASALT_CONFIG_JSON =
	"{\n"
	" \"value0\": {\n"
	"  \"config.optical_flow_type\": \"frame_to_frame\",\n"
	"  \"config.optical_flow_detection_grid_size\": 50,\n"
	"  \"config.optical_flow_detection_num_points_cell\": 1,\n"
	"  \"config.optical_flow_detection_min_threshold\": 5,\n"
	"  \"config.optical_flow_detection_max_threshold\": 40,\n"
	"  \"config.optical_flow_detection_nonoverlap\": true,\n"
	"  \"config.optical_flow_max_recovered_dist2\": 0.04,\n"
	"  \"config.optical_flow_pattern\": 51,\n"
	"  \"config.optical_flow_max_iterations\": 5,\n"
	"  \"config.optical_flow_epipolar_error\": 0.005,\n"
	"  \"config.optical_flow_levels\": 3,\n"
	"  \"config.optical_flow_skip_frames\": 1,\n"
	"  \"config.optical_flow_matching_guess_type\": \"REPROJ_AVG_DEPTH\",\n"
	"  \"config.optical_flow_matching_default_depth\": 1.0,\n"
	"  \"config.optical_flow_image_safe_radius\": 490.0,\n"
	"  \"config.optical_flow_recall_enable\": false,\n"
	"  \"config.optical_flow_recall_all_cams\": false,\n"
	"  \"config.optical_flow_recall_num_points_cell\": true,\n"
	"  \"config.optical_flow_recall_over_tracking\": false,\n"
	"  \"config.optical_flow_recall_update_patch_viewpoint\": false,\n"
	"  \"config.optical_flow_recall_max_patch_dist\": 3,\n"
	"  \"config.optical_flow_recall_max_patch_norms\": [\n"
	"   1.74,\n"
	"   0.96,\n"
	"   0.99,\n"
	"   0.44\n"
	"  ],\n"
	"  \"config.vio_linearization_type\": \"ABS_QR\",\n"
	"  \"config.vio_sqrt_marg\": true,\n"
	"  \"config.vio_max_states\": 3,\n"
	"  \"config.vio_max_kfs\": 7,\n"
	"  \"config.vio_min_frames_after_kf\": 5,\n"
	"  \"config.vio_new_kf_keypoints_thresh\": 0.7,\n"
	"  \"config.vio_debug\": false,\n"
	"  \"config.vio_extended_logging\": false,\n"
	"  \"config.vio_obs_std_dev\": 0.5,\n"
	"  \"config.vio_obs_huber_thresh\": 1.0,\n"
	"  \"config.vio_min_triangulation_dist\": 0.05,\n"
	"  \"config.vio_outlier_threshold\": 3.0,\n"
	"  \"config.vio_filter_iteration\": 4,\n"
	"  \"config.vio_max_iterations\": 7,\n"
	"  \"config.vio_enforce_realtime\": false,\n"
	"  \"config.vio_use_lm\": true,\n"
	"  \"config.vio_lm_lambda_initial\": 0.0001,\n"
	"  \"config.vio_lm_lambda_min\": 1e-06,\n"
	"  \"config.vio_lm_lambda_max\": 100.0,\n"
	"  \"config.vio_lm_landmark_damping_variant\": 1,\n"
	"  \"config.vio_lm_pose_damping_variant\": 1,\n"
	"  \"config.vio_scale_jacobian\": false,\n"
	"  \"config.vio_init_pose_weight\": 100000000.0,\n"
	"  \"config.vio_init_ba_weight\": 10.0,\n"
	"  \"config.vio_init_bg_weight\": 100.0,\n"
	"  \"config.vio_marg_lost_landmarks\": true,\n"
	"  \"config.vio_fix_long_term_keyframes\": false,\n"
	"  \"config.vio_kf_marg_feature_ratio\": 0.1,\n"
	"  \"config.vio_kf_marg_criteria\": \"KF_MARG_DEFAULT\",\n"
	"  \"config.mapper_obs_std_dev\": 0.25,\n"
	"  \"config.mapper_obs_huber_thresh\": 1.5,\n"
	"  \"config.mapper_detection_num_points\": 800,\n"
	"  \"config.mapper_num_frames_to_match\": 30,\n"
	"  \"config.mapper_frames_to_match_threshold\": 0.04,\n"
	"  \"config.mapper_min_matches\": 20,\n"
	"  \"config.mapper_ransac_threshold\": 5e-05,\n"
	"  \"config.mapper_min_track_length\": 5,\n"
	"  \"config.mapper_max_hamming_distance\": 70,\n"
	"  \"config.mapper_second_best_test_ratio\": 1.2,\n"
	"  \"config.mapper_bow_num_bits\": 16,\n"
	"  \"config.mapper_min_triangulation_dist\": 0.07,\n"
	"  \"config.mapper_no_factor_weights\": false,\n"
	"  \"config.mapper_use_factors\": true,\n"
	"  \"config.mapper_use_lm\": true,\n"
	"  \"config.mapper_lm_lambda_min\": 1e-32,\n"
	"  \"config.mapper_lm_lambda_max\": 1000.0\n"
	" }\n"
	"}\n";

#define Q_IMU 8192   // ~4 s of IMU
#define Q_FRAMES 4    // camera transfers waiting for the worker
#define STILL_NS (U_TIME_1S_IN_NS * 1)
#define STILL_GYRO_RAD_S 0.09f // ~5 deg/s
#define STILL_ACC_DEV 0.4f     // m/s^2 from the running mean
#define GUARD_NS (U_TIME_1S_IN_NS * 1)
#define GUARD_MAX_M 0.25f
#define IMU_AHEAD_NS (U_TIME_1MS_IN_NS * 100)
#define ALIGN_POSES 60 // ~1 s of SLAM poses

struct q_item
{
	timepoint_ns ts;
	int frame_slot; //!< -1: IMU sample
	struct xrt_vec3 accel, gyro;
};

/*!
 * The driver's threads only enqueue (never wait on Basalt); a worker thread feeds Basalt in arrival order. If Basalt
 * falls behind and the queue fills, it counts as lost and is restarted (a new instance: Basalt's own reset followed by
 * a gap in the data diverged in replay, 95 m in a second).
 */
struct psvr2_basalt
{
	char toml[512];

	// Queue (lock).
	struct os_mutex lock;
	struct os_cond cond;
	struct q_item q[Q_IMU];
	uint32_t q_head, q_count;
	uint8_t *frames[Q_FRAMES];
	bool frame_used[Q_FRAMES];
	bool running;
	bool restart_requested;
	uint32_t dropped;
	uint32_t frames_skipped;
	struct os_thread_helper worker;

	// Tracker (tracker_lock: created/destroyed by the worker, read by psvr2_basalt_ref_pose).
	struct os_mutex tracker_lock;
	struct xrt_frame_context xfctx;
	struct xrt_tracked_slam *slam;
	struct xrt_tracking_sinks *sinks;
	bool started; //!< the current instance is being fed (gate passed)

	// Worker-only state.
	timepoint_ns still_since;
	timepoint_ns start_ts;
	struct xrt_vec3 acc_mean;
	bool have_acc_mean;
	timepoint_ns last_frame_ts;
	time_duration_ns frame_interval_ns;
	uint8_t *full_img; //!< full size decode for g_scale 2
	timepoint_ns newest_frame_ts; //!< newest camera transfer time enqueued (lock)
	uint8_t *decode_tmp;

	// Referee state (lock).
	struct xrt_quat w_sum;
	uint32_t w_n;
	bool have_guard;
	timepoint_ns guard_ts;
	struct xrt_vec3 guard_b, guard_s;
	uint32_t resets;
};


/*
 *
 * Calibration: kb4 fit to Sony's camera model, camera placement, files for Basalt.
 *
 */

struct calib_cam
{
	uint32_t id;
	uint16_t w, h;
	float K[9];
	double p[20];
};

static bool
parse_block(const uint8_t *data, uint32_t size, struct calib_cam cams[2], float rel_R[9], float rel_t[3])
{
	if (size < 24 || memcmp(data, "SIE\x01", 4) != 0) {
		return false;
	}
	uint8_t ndist = data[15], nrel = data[16];
	if (ndist < 2 || size < 24u + ndist * 228u + nrel * 52u) {
		return false;
	}
	for (int i = 0; i < 2; i++) {
		const uint8_t *c = data + 24 + i * 228;
		memcpy(&cams[i].id, c, 4);
		memcpy(&cams[i].w, c + 4, 2);
		memcpy(&cams[i].h, c + 6, 2);
		memcpy(cams[i].K, c + 8, 36);
		memcpy(cams[i].p, c + 44, 160);
	}
	for (int i = 0; i < nrel; i++) {
		const uint8_t *r = data + 24 + ndist * 228 + i * 52;
		if (r[0] == 0 && r[1] == 1) { // to camera 0, from camera 1
			memcpy(rel_R, r + 4, 36);
			memcpy(rel_t, r + 40, 12);
			return true;
		}
	}
	return false;
}

static void
kb4_project(const double q[8], double X, double Y, double Z, double *u, double *v)
{
	double r = sqrt(X * X + Y * Y), th = atan2(r, Z), t2 = th * th;
	double d = th * (1 + t2 * (q[4] + t2 * (q[5] + t2 * (q[6] + t2 * q[7]))));
	double s = r > 1e-12 ? d / r : 1.0;
	*u = q[0] * s * X + q[2];
	*v = q[1] * s * Y + q[3];
}

//! Solve A x = b (n x n, Gaussian elimination with partial pivoting); false if singular.
static bool
solve(int n, double *A, double *b, double *x)
{
	for (int c = 0; c < n; c++) {
		int piv = c;
		for (int r = c + 1; r < n; r++) {
			if (fabs(A[r * n + c]) > fabs(A[piv * n + c])) {
				piv = r;
			}
		}
		if (fabs(A[piv * n + c]) < 1e-18) {
			return false;
		}
		for (int k = 0; k < n; k++) {
			double t = A[c * n + k];
			A[c * n + k] = A[piv * n + k];
			A[piv * n + k] = t;
		}
		double t = b[c];
		b[c] = b[piv];
		b[piv] = t;
		for (int r = c + 1; r < n; r++) {
			double f = A[r * n + c] / A[c * n + c];
			for (int k = c; k < n; k++) {
				A[r * n + k] -= f * A[c * n + k];
			}
			b[r] -= f * b[c];
		}
	}
	for (int r = n - 1; r >= 0; r--) {
		double s = b[r];
		for (int k = r + 1; k < n; k++) {
			s -= A[r * n + k] * x[k];
		}
		x[r] = s / A[r * n + r];
	}
	return true;
}

/*!
 * Fit kb4 (fx fy cx cy k1..k4) to Sony's model: unproject a pixel grid with Monado's psvr2 camera model, then
 * Levenberg-Marquardt on the reprojection error (psvr2-test/vio/kb4_fit.py: bottom pair 0.13 px mean, 0.38 max).
 */
static bool
fit_kb4(const struct calib_cam *c, double q[8], double *out_max_err_px)
{
	struct t_camera_calibration cc = {0};
	cc.image_size_pixels.w = c->w;
	cc.image_size_pixels.h = c->h;
	cc.distortion_model = T_DISTORTION_PSVR2;
	for (int i = 0; i < 9; i++) {
		cc.intrinsics[i / 3][i % 3] = c->K[i];
	}
	for (int i = 0; i < 20; i++) {
		cc.psvr2.params[i] = c->p[i];
	}
	struct t_camera_model_params mp;
	t_camera_model_params_from_t_camera_calibration(&cc, &mp);

	enum { G = 60 };
	static double P[G * G][5]; // X Y Z u v
	int n = 0;
	double cx = c->K[2], cy = c->K[5], rmax = 0.49 * (c->w < c->h ? c->w : c->h);
	for (int gy = 0; gy < G; gy++) {
		for (int gx = 0; gx < G; gx++) {
			float u = 4.0f + (c->w - 9.0f) * gx / (G - 1), v = 4.0f + (c->h - 9.0f) * gy / (G - 1);
			if (hypot(u - cx, v - cy) > rmax) {
				continue;
			}
			float X, Y, Z;
			if (!t_camera_models_unproject(&mp, u, v, &X, &Y, &Z) || !(Z > 0.05f) || !isfinite(X)) {
				continue;
			}
			// Check the inverse converged: project back with Sony's model.
			float pu, pv;
			if (!t_camera_models_project(&mp, X, Y, Z, &pu, &pv) || hypotf(pu - u, pv - v) > 0.01f) {
				continue;
			}
			P[n][0] = X, P[n][1] = Y, P[n][2] = Z, P[n][3] = u, P[n][4] = v;
			n++;
		}
	}
	if (n < 200) {
		return false;
	}
	double init[8] = {c->K[0], c->K[4], c->K[2], c->K[5], 0, 0, 0, 0};
	memcpy(q, init, sizeof(init));
	double lam = 1e-3;
	double cost = 0;
	for (int i = 0; i < n; i++) {
		double u, v;
		kb4_project(q, P[i][0], P[i][1], P[i][2], &u, &v);
		cost += (u - P[i][3]) * (u - P[i][3]) + (v - P[i][4]) * (v - P[i][4]);
	}
	for (int it = 0; it < 100; it++) {
		double A[64] = {0}, g[8] = {0};
		for (int i = 0; i < n; i++) {
			double u0, v0;
			kb4_project(q, P[i][0], P[i][1], P[i][2], &u0, &v0);
			double ru = u0 - P[i][3], rv = v0 - P[i][4], Ju[8], Jv[8];
			for (int k = 0; k < 8; k++) {
				double qq[8];
				memcpy(qq, q, sizeof(qq));
				double h = 1e-6 * (fabs(q[k]) > 1 ? fabs(q[k]) : 1);
				qq[k] += h;
				double u1, v1;
				kb4_project(qq, P[i][0], P[i][1], P[i][2], &u1, &v1);
				Ju[k] = (u1 - u0) / h;
				Jv[k] = (v1 - v0) / h;
			}
			for (int a = 0; a < 8; a++) {
				g[a] += Ju[a] * ru + Jv[a] * rv;
				for (int bb = 0; bb < 8; bb++) {
					A[a * 8 + bb] += Ju[a] * Ju[bb] + Jv[a] * Jv[bb];
				}
			}
		}
		double M[64], rhs[8], step[8], qn[8];
		memcpy(M, A, sizeof(M));
		for (int a = 0; a < 8; a++) {
			M[a * 8 + a] += lam * A[a * 8 + a];
			rhs[a] = -g[a];
		}
		if (!solve(8, M, rhs, step)) {
			break;
		}
		for (int k = 0; k < 8; k++) {
			qn[k] = q[k] + step[k];
		}
		double cn = 0;
		for (int i = 0; i < n; i++) {
			double u, v;
			kb4_project(qn, P[i][0], P[i][1], P[i][2], &u, &v);
			cn += (u - P[i][3]) * (u - P[i][3]) + (v - P[i][4]) * (v - P[i][4]);
		}
		if (cn < cost) {
			memcpy(q, qn, sizeof(qn));
			double rel = (cost - cn) / cost;
			cost = cn;
			lam *= 0.3;
			if (rel < 1e-12) {
				break;
			}
		} else {
			lam *= 10;
		}
	}
	double mx = 0;
	for (int i = 0; i < n; i++) {
		double u, v;
		kb4_project(q, P[i][0], P[i][1], P[i][2], &u, &v);
		double e = hypot(u - P[i][3], v - P[i][4]);
		mx = e > mx ? e : mx;
	}
	*out_max_err_px = mx;
	return mx < 2.0;
}

/*!
 * Image scale for Basalt (PSVR2_BASALT_SCALE, 1 or 2): 2 averages 2x2 pixels (508x508). Basalt's per-frame time grew
 * in fast turns at full size (pose delay median 44 ms above 120 deg/s, half past the referee's 40 ms window).
 */
static int g_scale = 1;

static void
fprint_pose(FILE *f, struct xrt_vec3 p, struct xrt_quat q)
{
	fprintf(f, "{\"px\": %.9f, \"py\": %.9f, \"pz\": %.9f, \"qx\": %.9f, \"qy\": %.9f, \"qz\": %.9f, \"qw\": %.9f}",
	        p.x, p.y, p.z, q.x, q.y, q.z, q.w);
}

/*!
 * Write Basalt's calibration and config for this headset into @p dir; true on success. Camera placement: camera 0
 * from psvr2_cam0_mount_from_block (OpenXR camera axes in the head = IMU frame), turned into Basalt's OpenCV camera
 * axes; camera 1 from the block's 1 -> 0 relation, used as validated offline (R_cam0_cam1 = stored^T, camera 1 at
 * -stored^T t in camera 0); the IMU between the two cameras (its exact position changed Basalt's results little).
 */
static bool
write_files(const char *dir, const uint8_t *data, uint32_t size, char *out_toml, size_t toml_len)
{
	struct calib_cam cams[2];
	float relR[9], relt[3];
	if (!parse_block(data, size, cams, relR, relt)) {
		B_WARN("calibration block lacks the bottom cameras or their relation");
		return false;
	}
	struct xrt_quat q_mount;
	if (!psvr2_cam0_mount_from_block(data, size, -15.0f, false, &q_mount)) {
		B_WARN("no camera 0 mount in the calibration block");
		return false;
	}
	double kb[2][8], err[2];
	for (int i = 0; i < 2; i++) {
		if (!fit_kb4(&cams[i], kb[i], &err[i])) {
			B_WARN("kb4 fit failed for camera %d", i);
			return false;
		}
	}
	B_INFO("kb4 fit to the headset's camera model: max error %.2f / %.2f px", err[0], err[1]);

	// Camera 0 in the IMU frame, OpenCV axes: mount * Rx(180).
	struct xrt_quat flip, q0;
	math_quat_from_angle_vector((float)M_PI, &(struct xrt_vec3){1, 0, 0}, &flip);
	math_quat_rotate(&q_mount, &flip, &q0);
	// Relation: R01 = stored^T (row-major stored S: R01[i][j] = S[j][i]); camera 1 origin in camera 0: -S^T t.
	struct xrt_matrix_3x3 R01;
	struct xrt_vec3 p1_in_0 = {0, 0, 0};
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			R01.v[i * 3 + j] = relR[j * 3 + i];
		}
	}
	p1_in_0.x = -(relR[0] * relt[0] + relR[3] * relt[1] + relR[6] * relt[2]);
	p1_in_0.y = -(relR[1] * relt[0] + relR[4] * relt[1] + relR[7] * relt[2]);
	p1_in_0.z = -(relR[2] * relt[0] + relR[5] * relt[1] + relR[8] * relt[2]);
	struct xrt_quat q01, q1;
	math_quat_from_matrix_3x3(&R01, &q01);
	math_quat_rotate(&q0, &q01, &q1);
	struct xrt_vec3 d;
	math_quat_rotate_vec3(&q0, &p1_in_0, &d);
	struct xrt_vec3 c0 = m_vec3_mul_scalar(d, -0.5f), c1 = m_vec3_mul_scalar(d, 0.5f);
	{
		float vp, vy;
		psvr2_cam_mount_view_angles(&q_mount, &vp, &vy);
		B_INFO("camera 0 (Basalt's own placement) looks %.1f deg down, %.1f deg %s; camera 1 %.1f mm away", -vp,
		       fabsf(vy), vy > 0 ? "left" : "right", m_vec3_len(d) * 1000.0f);
	}

	mkdir(dir, 0700);
	char path[512];
	snprintf(path, sizeof(path), "%s/psvr2_basalt_calib.json", dir);
	FILE *f = fopen(path, "w");
	if (f == NULL) {
		return false;
	}
	fprintf(f, "{\"value0\": {\"T_imu_cam\": [");
	fprint_pose(f, c0, q0);
	fprintf(f, ", ");
	fprint_pose(f, c1, q1);
	fprintf(f, "], \"intrinsics\": [");
	for (int i = 0; i < 2; i++) {
		// Scaled image: pixel centres map as (u + 0.5) / s - 0.5; focal lengths divide by s.
		double sc = (double)g_scale, off = 0.5 / sc - 0.5;
		fprintf(f,
		        "%s{\"camera_type\": \"kb4\", \"intrinsics\": {\"fx\": %.9f, \"fy\": %.9f, \"cx\": %.9f, \"cy\": %.9f, "
		        "\"k1\": %.12f, \"k2\": %.12f, \"k3\": %.12f, \"k4\": %.12f}}",
		        i ? ", " : "", kb[i][0] / sc, kb[i][1] / sc, kb[i][2] / sc + off, kb[i][3] / sc + off, kb[i][4], kb[i][5],
		        kb[i][6], kb[i][7]);
	}
	// IMU noise densities measured on this headset resting (gyro 3.4e-5 rad/s/sqrtHz, accel 4.1e-4 m/s^2/sqrtHz), x5.
	fprintf(f,
	        "], \"resolution\": [[%d, %d], [%d, %d]], \"vignette\": [], "
	        "\"calib_accel_bias\": [0,0,0,0,0,0,0,0,0], \"calib_gyro_bias\": [0,0,0,0,0,0,0,0,0,0,0,0], "
	        "\"imu_update_rate\": 2000.0, \"accel_noise_std\": [0.00205, 0.00205, 0.00205], "
	        "\"gyro_noise_std\": [0.00017, 0.00017, 0.00017], \"accel_bias_std\": [0.001, 0.001, 0.001], "
	        "\"gyro_bias_std\": [0.0001, 0.0001, 0.0001], "
	        "\"T_mocap_world\": {\"px\": 0, \"py\": 0, \"pz\": 0, \"qx\": 0, \"qy\": 0, \"qz\": 0, \"qw\": 1}, "
	        "\"T_imu_marker\": {\"px\": 0, \"py\": 0, \"pz\": 0, \"qx\": 0, \"qy\": 0, \"qz\": 0, \"qw\": 1}, "
	        "\"mocap_time_offset_ns\": 0, \"mocap_to_imu_offset_ns\": 0, \"cam_time_offset_ns\": 0}}\n",
	        cams[0].w / g_scale, cams[0].h / g_scale, cams[1].w / g_scale, cams[1].h / g_scale);
	fclose(f);
	snprintf(path, sizeof(path), "%s/psvr2_basalt_config.json", dir);
	f = fopen(path, "w");
	if (f == NULL) {
		return false;
	}
	if (g_scale == 1) {
		fputs(BASALT_CONFIG_JSON, f);
	} else {
		// The optical flow safe radius is in pixels: scale it with the image.
		const char *key = "\"config.optical_flow_image_safe_radius\": ";
		const char *at = strstr(BASALT_CONFIG_JSON, key);
		if (at == NULL) {
			fputs(BASALT_CONFIG_JSON, f);
		} else {
			fwrite(BASALT_CONFIG_JSON, 1, (size_t)(at - BASALT_CONFIG_JSON) + strlen(key), f);
			double radius = atof(at + strlen(key));
			fprintf(f, "%.1f", radius / g_scale);
			const char *rest = at + strlen(key);
			while (*rest != ',' && *rest != '\n' && *rest != '}' && *rest != '\0') {
				rest++;
			}
			fputs(rest, f);
		}
	}
	fclose(f);
	snprintf(out_toml, toml_len, "%s/psvr2_basalt.toml", dir);
	f = fopen(out_toml, "w");
	if (f == NULL) {
		return false;
	}
	// Reference values (Monado fork's configs): deterministic=0 (1 deadlocks with a single feeding thread);
	// 4 threads keep up with 60 frames/s (77/s offline, 1 thread 55/s).
	fprintf(f,
	        "show-gui=0\ncam-calib=\"%s/psvr2_basalt_calib.json\"\nconfig-path=\"%s/psvr2_basalt_config.json\"\n"
	        "marg-data=\"\"\nprint-queue=0\nuse-double=0\ndeterministic=0\nnum-threads=4\n",
	        dir, dir);
	fclose(f);
	return true;
}


/*
 *
 * Tracker instances and the worker.
 *
 */

static bool
tracker_create(struct psvr2_basalt *b)
{
#ifdef XRT_FEATURE_SLAM
	struct t_slam_tracker_config config;
	t_slam_fill_default_config(&config);
	config.cam_count = 2;
	config.slam_config = b->toml;
	config.submit_from_start = true;
	config.prediction = SLAM_PRED_NONE;
	config.write_csvs = false;
	memset(&b->xfctx, 0, sizeof(b->xfctx));
	if (t_slam_create(&b->xfctx, &config, &b->slam, &b->sinks) != 0 || t_slam_start(b->slam) != 0) {
		xrt_frame_context_destroy_nodes(&b->xfctx);
		b->slam = NULL;
		b->sinks = NULL;
		return false;
	}
	return true;
#else
	(void)b;
	return false;
#endif
}

//! Stop and free the current instance (its threads are joined by the frame context).
static void
tracker_destroy(struct psvr2_basalt *b)
{
	if (b->slam == NULL) {
		return;
	}
	xrt_frame_context_destroy_nodes(&b->xfctx);
	b->slam = NULL;
	b->sinks = NULL;
}

static void
bc4_decode(const uint8_t *src, uint8_t *dst, int dst_stride)
{
	for (int by = 0; by < IMG_W / 4; by++) {
		for (int bx = 0; bx < IMG_STRIDE / 4; bx++) {
			const uint8_t *blk = src + (by * (IMG_STRIDE / 4) + bx) * 8;
			int a0 = blk[0], a1 = blk[1], pal[8] = {a0, a1};
			if (a0 > a1) {
				for (int i = 1; i < 7; i++) {
					pal[1 + i] = ((7 - i) * a0 + i * a1) / 7;
				}
			} else {
				for (int i = 1; i < 5; i++) {
					pal[1 + i] = ((5 - i) * a0 + i * a1) / 5;
				}
				pal[6] = 0;
				pal[7] = 255;
			}
			uint64_t bits = 0;
			for (int i = 0; i < 6; i++) {
				bits |= (uint64_t)blk[2 + i] << (8 * i);
			}
			for (int p = 0; p < 16; p++) {
				int x = bx * 4 + (p & 3), y = by * 4 + (p >> 2);
				if (x < IMG_W) {
					dst[y * dst_stride + x] = (uint8_t)pal[(bits >> (3 * p)) & 7];
				}
			}
		}
	}
}

//! Worker: restart the instance and wait for stillness again.
static void
worker_restart(struct psvr2_basalt *b)
{
	os_mutex_lock(&b->tracker_lock);
	tracker_destroy(b);
	b->started = false;
	bool ok = tracker_create(b);
	os_mutex_unlock(&b->tracker_lock);
	b->still_since = 0;
	b->have_acc_mean = false;
	b->last_frame_ts = 0;
	os_mutex_lock(&b->lock);
	b->have_guard = false;
	b->w_n = 0;
	b->w_sum = (struct xrt_quat){0, 0, 0, 0};
	b->resets++;
	os_mutex_unlock(&b->lock);
	B_INFO("Basalt restarted (%s); waiting for the headset to be still", ok ? "new instance" : "FAILED");
}

static void
worker_imu(struct psvr2_basalt *b, const struct q_item *it)
{
	if (!b->started) {
		// Stillness gate: gyro slow and the accelerometer near its running mean, for STILL_NS.
		if (!b->have_acc_mean) {
			b->acc_mean = it->accel;
			b->have_acc_mean = true;
		}
		b->acc_mean = m_vec3_add(b->acc_mean, m_vec3_mul_scalar(m_vec3_sub(it->accel, b->acc_mean), 0.01f));
		bool still = m_vec3_len(it->gyro) < STILL_GYRO_RAD_S &&
		             m_vec3_len(m_vec3_sub(it->accel, b->acc_mean)) < STILL_ACC_DEV;
		if (!still) {
			b->still_since = 0;
		} else if (b->still_since == 0) {
			b->still_since = it->ts;
		} else if (it->ts - b->still_since > STILL_NS && b->slam != NULL) {
			os_mutex_lock(&b->tracker_lock);
			b->started = true;
			os_mutex_unlock(&b->tracker_lock);
			b->start_ts = it->ts;
			B_INFO("headset still: starting Basalt");
		}
		if (!b->started) {
			return;
		}
	}
	struct xrt_imu_sample s = {
	    .timestamp_ns = it->ts,
	    .accel_m_s2 = {it->accel.x, it->accel.y, it->accel.z},
	    .gyro_rad_secs = {it->gyro.x, it->gyro.y, it->gyro.z},
	};
	xrt_sink_push_imu(b->sinks->imus[0], &s);
}

static void
worker_frame(struct psvr2_basalt *b, const struct q_item *it, const uint8_t *data)
{
	// Frames from 20 ms after the IMU started (Basalt needs IMU before each frame; seconds of IMU before the first
	// frame fill its bounded queues and deadlock it), in order.
	if (!b->started || it->ts <= b->start_ts + U_TIME_1MS_IN_NS * 20 || it->ts <= b->last_frame_ts) {
		return;
	}
	// Camera rate for Basalt (PSVR2_BASALT_HZ, default 60: every frame; Monado's Rift S driver uses 30, which diverged
	// three times in the 19:12 replay and made Basalt's poses 25 ms late instead of 10). Busy: frames are skipped.
	if (b->last_frame_ts != 0 && it->ts - b->last_frame_ts < b->frame_interval_ns - U_TIME_1MS_IN_NS * 4) {
		return;
	}
	b->last_frame_ts = it->ts;
	for (int c = 0; c < 2; c++) {
		struct xrt_frame *xf = NULL;
		int w = IMG_W / g_scale;
		u_frame_create_one_off(XRT_FORMAT_L8, w, w, &xf);
		if (xf == NULL) {
			return;
		}
		if (g_scale == 1) {
			bc4_decode(data + 256 + c * IMG_BYTES, xf->data, (int)xf->stride);
		} else {
			bc4_decode(data + 256 + c * IMG_BYTES, b->full_img, IMG_W);
			for (int y = 0; y < w; y++) {
				const uint8_t *r0 = b->full_img + (size_t)(2 * y) * IMG_W, *r1 = r0 + IMG_W;
				uint8_t *o = xf->data + (size_t)y * xf->stride;
				for (int x = 0; x < w; x++) {
					o[x] = (uint8_t)((r0[2 * x] + r0[2 * x + 1] + r1[2 * x] + r1[2 * x + 1] + 2) / 4);
				}
			}
		}
		xf->timestamp = it->ts;
		xf->source_timestamp = it->ts;
		xf->source_sequence = (uint64_t)(it->ts / 1000);
		xrt_sink_push_frame(b->sinks->cams[c], xf);
		xrt_frame_reference(&xf, NULL);
	}
}

static void *
worker_run(void *ptr)
{
	struct psvr2_basalt *b = ptr;
	for (;;) {
		os_mutex_lock(&b->lock);
		while (b->running && b->q_count == 0 && !b->restart_requested) {
			os_cond_wait(&b->cond, &b->lock);
		}
		if (!b->running) {
			os_mutex_unlock(&b->lock);
			break;
		}
		if (b->restart_requested) {
			b->restart_requested = false;
			// Drop what's queued: it belongs to the old instance's stretch.
			for (uint32_t i = 0; i < Q_FRAMES; i++) {
				b->frame_used[i] = false;
			}
			b->q_count = 0;
			os_mutex_unlock(&b->lock);
			worker_restart(b);
			continue;
		}
		struct q_item it = b->q[b->q_head];
		/*
		 * Don't run the IMU far ahead of the frames: Basalt only consumes IMU as it processes frames, and its IMU
		 * queue is bounded, so IMU pushed while frames are missing fills it and blocks this thread for good (60 Hz
		 * replay hung). Hold IMU more than IMU_AHEAD_NS past the newest frame time seen until a frame comes; a
		 * camera stall that fills our queue restarts Basalt.
		 */
		if (it.frame_slot < 0 && b->started && b->newest_frame_ts != 0 && it.ts > b->newest_frame_ts + IMU_AHEAD_NS) {
			bool frame_waiting = false;
			for (uint32_t k = 1; k < b->q_count; k++) {
				if (b->q[(b->q_head + k) % Q_IMU].frame_slot >= 0) {
					frame_waiting = true;
					break;
				}
			}
			if (!frame_waiting) {
				os_cond_wait(&b->cond, &b->lock); // until a frame (or restart/stop) is enqueued
				os_mutex_unlock(&b->lock);
				continue;
			}
		}
		b->q_head = (b->q_head + 1) % Q_IMU;
		b->q_count--;
		uint8_t *frame = NULL;
		if (it.frame_slot >= 0) {
			memcpy(b->decode_tmp, b->frames[it.frame_slot], XFER_BYTES);
			b->frame_used[it.frame_slot] = false;
			frame = b->decode_tmp;
		}
		os_mutex_unlock(&b->lock);
		if (it.frame_slot < 0) {
			worker_imu(b, &it);
		} else {
			worker_frame(b, &it, frame);
		}
	}
	return NULL;
}

//! Enqueue (driver threads; never blocks). A full queue means Basalt fell behind: drop it all and restart.
static void
enqueue(struct psvr2_basalt *b, const struct q_item *it, const uint8_t *frame)
{
	os_mutex_lock(&b->lock);
	struct q_item item = *it;
	item.frame_slot = -1;
	bool full = b->q_count >= Q_IMU;
	if (!full && frame != NULL) {
		int slot = -1;
		for (int i = 0; i < Q_FRAMES; i++) {
			if (!b->frame_used[i]) {
				slot = i;
				break;
			}
		}
		if (slot < 0) {
			// Basalt is busy with earlier frames: skip this one (it copes with gaps; a stuck IMU queue restarts it).
			b->frames_skipped++;
			os_mutex_unlock(&b->lock);
			return;
		} else {
			memcpy(b->frames[slot], frame, XFER_BYTES);
			b->frame_used[slot] = true;
			item.frame_slot = slot;
			if (item.ts > b->newest_frame_ts) {
				b->newest_frame_ts = item.ts;
			}
		}
	}
	if (full) {
		if (!b->restart_requested) {
			B_WARN("Basalt fell behind (queue full): restarting it");
		}
		b->dropped++;
		b->restart_requested = true;
	} else if (!b->restart_requested) {
		b->q[(b->q_head + b->q_count) % Q_IMU] = item;
		b->q_count++;
	}
	os_cond_signal(&b->cond);
	os_mutex_unlock(&b->lock);
}


/*
 *
 * Exported functions.
 *
 */

struct psvr2_basalt *
psvr2_basalt_create(struct xrt_frame_context *xfctx, const uint8_t *calib_block, uint32_t calib_size)
{
	(void)xfctx; // Basalt gets its own frame contexts, so it can be restarted on its own.
#ifndef XRT_FEATURE_SLAM
	(void)calib_block;
	(void)calib_size;
	B_WARN("built without SLAM support");
	return NULL;
#else
	char dir[400];
	const char *rt = getenv("XDG_RUNTIME_DIR");
	const char *scale = getenv("PSVR2_BASALT_SCALE");
	g_scale = scale != NULL && atoi(scale) == 1 ? 1 : 2; // default 2: on time in fast turns (full size 44 ms late above 120 deg/s)
	snprintf(dir, sizeof(dir), "%s/psvr2-basalt", rt != NULL ? rt : "/tmp");
	struct psvr2_basalt *b = U_TYPED_CALLOC(struct psvr2_basalt);
	if (!write_files(dir, calib_block, calib_size, b->toml, sizeof(b->toml))) {
		free(b);
		return NULL;
	}
	os_mutex_init(&b->lock);
	os_cond_init(&b->cond);
	os_mutex_init(&b->tracker_lock);
	for (int i = 0; i < Q_FRAMES; i++) {
		b->frames[i] = malloc(XFER_BYTES);
	}
	b->decode_tmp = malloc(XFER_BYTES);
	b->full_img = malloc((size_t)IMG_W * IMG_W);
	{
		const char *hz = getenv("PSVR2_BASALT_HZ");
		double f = hz != NULL ? atof(hz) : 60.0; // 60: no divergence restarts in replay (30: 3), ~10 ms latency
		b->frame_interval_ns = f > 0.0 ? (time_duration_ns)(1e9 / f) : 0;
	}
	if (!tracker_create(b)) {
		B_WARN("could not start Basalt (VIT_SYSTEM_LIBRARY_PATH set to libbasalt.so?)");
		for (int i = 0; i < Q_FRAMES; i++) {
			free(b->frames[i]);
		}
		free(b->decode_tmp);
		os_mutex_destroy(&b->tracker_lock);
		os_cond_destroy(&b->cond);
		os_mutex_destroy(&b->lock);
		free(b);
		return NULL;
	}
	b->running = true;
	os_thread_helper_init(&b->worker);
	os_thread_helper_start(&b->worker, worker_run, b);
	B_INFO("Basalt running, config %s; waiting for the headset to be still", b->toml);
	return b;
#endif
}

void
psvr2_basalt_push_imu(struct psvr2_basalt *b, timepoint_ns ts, const struct xrt_vec3 *accel, const struct xrt_vec3 *gyro)
{
	struct q_item it = {.ts = ts, .frame_slot = -1, .accel = *accel, .gyro = *gyro};
	enqueue(b, &it, NULL);
}

void
psvr2_basalt_push_transfer(struct psvr2_basalt *b, timepoint_ns ts, const uint8_t *data, size_t length)
{
	if (length < XFER_BYTES) {
		return;
	}
	struct q_item it = {.ts = ts};
	enqueue(b, &it, data);
}

bool
psvr2_basalt_ref_pose(struct psvr2_basalt *b, timepoint_ns ts, const struct xrt_pose *slam_pose, struct xrt_pose *out_pose)
{
	struct xrt_pose bp;
	os_mutex_lock(&b->tracker_lock);
	bool have = b->started && b->slam != NULL && t_slam_get_pose_at(b->slam, ts, &bp);
	os_mutex_unlock(&b->tracker_lock);
	if (!have) {
		return false;
	}
	os_mutex_lock(&b->lock);
	/*
	 * World rotation W = q_slam * q_basalt^-1: averaged over the first ALIGN_POSES poses of an instance, then fixed.
	 * No referee pose until then: an alignment still being refined rotates every position a little at each update,
	 * which shows up as false steps (replay: 88-116 steps off by > 1 mm in the 5 s after a start, 1-8 later).
	 */
	struct xrt_quat inv, w;
	math_quat_invert(&bp.orientation, &inv);
	math_quat_rotate(&slam_pose->orientation, &inv, &w);
	if (b->w_n > 0 && (w.x * b->w_sum.x + w.y * b->w_sum.y + w.z * b->w_sum.z + w.w * b->w_sum.w) < 0) {
		w.x = -w.x, w.y = -w.y, w.z = -w.z, w.w = -w.w;
	}
	if (b->w_n < ALIGN_POSES) {
		b->w_sum.x += w.x, b->w_sum.y += w.y, b->w_sum.z += w.z, b->w_sum.w += w.w;
		b->w_n++;
		os_mutex_unlock(&b->lock);
		return false;
	}
	struct xrt_quat W = b->w_sum;
	math_quat_normalize(&W);
	struct xrt_pose out;
	math_quat_rotate(&W, &bp.orientation, &out.orientation);
	math_quat_rotate_vec3(&W, &bp.position, &out.position);

	// Divergence guard: over each second Basalt must have moved about as far as SLAM.
	bool diverged = false;
	if (!b->have_guard) {
		b->have_guard = true;
		b->guard_ts = ts, b->guard_b = out.position, b->guard_s = slam_pose->position;
	} else if (ts - b->guard_ts > GUARD_NS) {
		float db = m_vec3_len(m_vec3_sub(out.position, b->guard_b));
		float ds = m_vec3_len(m_vec3_sub(slam_pose->position, b->guard_s));
		if (fabsf(db - ds) > GUARD_MAX_M) {
			diverged = true;
			B_WARN("Basalt moved %.2f m vs SLAM %.2f m in a second: restarting it", db, ds);
		}
		b->guard_ts = ts, b->guard_b = out.position, b->guard_s = slam_pose->position;
	}
	if (diverged) {
		b->restart_requested = true;
		os_cond_signal(&b->cond);
	}
	os_mutex_unlock(&b->lock);
	if (diverged) {
		return false;
	}
	*out_pose = out;
	return true;
}

void
psvr2_basalt_destroy(struct psvr2_basalt **b_ptr)
{
	struct psvr2_basalt *b = *b_ptr;
	if (b == NULL) {
		return;
	}
	os_mutex_lock(&b->lock);
	b->running = false;
	os_cond_signal(&b->cond);
	os_mutex_unlock(&b->lock);
	os_thread_helper_stop_and_wait(&b->worker);
	os_thread_helper_destroy(&b->worker);
	os_mutex_lock(&b->tracker_lock);
	tracker_destroy(b);
	os_mutex_unlock(&b->tracker_lock);
	for (int i = 0; i < Q_FRAMES; i++) {
		free(b->frames[i]);
	}
	free(b->decode_tmp);
	free(b->full_img);
	os_mutex_destroy(&b->tracker_lock);
	os_cond_destroy(&b->cond);
	os_mutex_destroy(&b->lock);
	free(b);
	*b_ptr = NULL;
}
