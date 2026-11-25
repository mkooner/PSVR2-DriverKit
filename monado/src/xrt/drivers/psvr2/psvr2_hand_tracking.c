// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Optical hand tracking from the PS VR2 bottom camera pair, see psvr2_hand_tracking.h.
 * @ingroup drv_psvr2
 */

#include "psvr2_hand_tracking.h"
#include "psvr2_interface.h"

#include "xrt/xrt_frame.h"
#include "os/os_threading.h"
#include "os/os_time.h"
#include "math/m_api.h"
#include "math/m_mathinclude.h"
#include "math/m_vec3.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_file.h"
#include "util/u_frame.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_sink.h"
#include "util/u_time.h"
#include "util/u_trace_marker.h"
#include "tracking/t_camera_models.h"
#include "tracking/t_hand_tracking.h"
#include "hand/mercury/hg_interface.h"

// The implementation (u_sink_converter.c) is built with the precise BC4 decoder, so declare the same signature.
#define BCDEC_BC4BC5_PRECISE
#include "bcdec.h"

#include <stdlib.h>
#include <string.h>

DEBUG_GET_ONCE_LOG_OPTION(psvr2_ht_log, "PSVR2_HT_LOG", U_LOGGING_INFO)
/*!
 * Field of view of the unwarped pinhole views (deg). 120: on the 2026-11-20 recording (hands over a desk, Sony's
 * shorter LED exposure) the left hand was found 17% of the time and the right 49%, against 3% and 30% at 80 (best of
 * 60-120 on the 2026-11-16 recordings) and 5% / 49% at 100: hands take less of the picture, which Mercury's
 * detector copes with better here.
 */
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_ht_fov, "PSVR2_HT_FOV", 120.0f)
/*
 * Experiment (off by default): give Mercury the cameras' own fisheye images with their lens calibration instead of
 * the unwarped pinhole views, as Monado's Valve Index setup (target_builder_lighthouse.c) and psvr2ht do: Mercury then
 * undistorts each hand around itself (hg_image_distorter), where one 120 deg pinhole view stretches hands away from
 * the centre (1.7x at 60 deg). PSVR2_HT_RAW_RADIUS: the round image boundary, a fraction of the width (the Index's
 * 0.55).
 */
DEBUG_GET_ONCE_BOOL_OPTION(psvr2_ht_raw, "PSVR2_HT_RAW", false)
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_ht_raw_radius, "PSVR2_HT_RAW_RADIUS", 0.55f)
//! Brightening, out = 255 * (in/255)^gamma. 0.5 was best of 0.35-1 on the recordings (frames average ~25/255).
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_ht_gamma, "PSVR2_HT_GAMMA", 0.5f)
/*!
 * Auto levels per camera before the brightening: each view's tones between its LEVELS_LO and LEVELS_HI percentiles
 * (smoothed over ~0.3 s) are stretched to the full range, so a bright monitor or lamp in view no longer squeezes the
 * hands into a narrow band of grey. A dark room lit by a monitor (2026-11-23 recording, the user's second headset):
 * hands were faint grey on grey, the left camera's darker than the right's. PSVR2_HT_LEVELS=0: the fixed curve only.
 */
DEBUG_GET_ONCE_BOOL_OPTION(psvr2_ht_levels, "PSVR2_HT_LEVELS", true)
/*!
 * Hold a hand's last pose this long (ms) when Mercury loses it, so brief dropouts don't flash the hand away. A
 * smoothing step, not better detection: on the 2026-11-23 recordings the right hand's losses were mostly 0.1-0.25 s
 * gaps (10 in 32 s); a 0.3 s hold took it 82% -> 89% and the left 63% -> 71% (its losses are mostly longer than 1 s,
 * which the hold doesn't hide). 0: off.
 */
DEBUG_GET_ONCE_NUM_OPTION(psvr2_ht_hold_ms, "PSVR2_HT_HOLD_MS", 300)
/*!
 * Smoothing of the tracked hands (One Euro filter, Casiez et al. 2012): Mercury's joints come out a little different
 * every frame. Measured on the 2026-11-23 recordings (driver_ht_replay + psvr2-test/hands analysis): on still hands
 * the palm's orientation wobbled 1-2 deg RMS and the index fingertip 2.2-3.0 mm (median), 6-10 mm (p95), 90 times a
 * second: the shaky hands. The filter smooths strongly when the hand is slow and less when it moves fast. Simulated on
 * the same data with these defaults: fingertip wobble 0.8-1.2 mm (median), 2.6-3.4 mm (p95), ~18 ms behind while
 * moving. It runs on the palm's pose in the room (head turns are not hand motion) and on every joint relative to the
 * palm. PSVR2_HT_FILTER=0: off. MIN_CUTOFF: Hz when still; BETA: how fast the cutoff rises with speed (per m/s for
 * positions, per rad/s for rotations); the fingers use twice MIN_CUTOFF.
 */
DEBUG_GET_ONCE_BOOL_OPTION(psvr2_ht_filter, "PSVR2_HT_FILTER", true)
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_ht_filter_min_cutoff, "PSVR2_HT_FILTER_MIN_CUTOFF", 1.5f)
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_ht_filter_beta_pos, "PSVR2_HT_FILTER_BETA_POS", 2.0f)
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_ht_filter_beta_rot, "PSVR2_HT_FILTER_BETA_ROT", 1.0f)
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_ht_levels_lo, "PSVR2_HT_LEVELS_LO", 0.01f)
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_ht_levels_hi, "PSVR2_HT_LEVELS_HI", 0.95f)
//! Unwarped image size (square).
DEBUG_GET_ONCE_NUM_OPTION(psvr2_ht_size, "PSVR2_HT_SIZE", 640)
//! Offline diagnosis: a folder to save every 30th pair of images Mercury gets (raw and unwarped, PGM). Off when unset.
DEBUG_GET_ONCE_OPTION(psvr2_ht_dump, "PSVR2_HT_DUMP", NULL)
/*
 * Training data for the gesture classifier (offline): a folder to write, for every camera frame, a CROP_SIZE x
 * CROP_SIZE grayscale crop around each tracked hand in each unwarped view (crops.bin; records of CropRecord then the
 * pixels). The hand is placed from its latest joints (Mercury's, as the SteamVR plugin gets them) projected into the
 * view. Off when unset.
 */
DEBUG_GET_ONCE_OPTION(psvr2_ht_crops, "PSVR2_HT_CROPS", NULL)
/*
 * PSVR2_HT_CROPS_UPRIGHT=1: the crop is turned so the wrist-to-middle-finger direction points up and sized from that
 * length (MediaPipe's hand landmark model expects hands so: its palm-to-hand box, 2.6x the palm), PSVR2_HT_CROP_SIZE
 * pixels square (default CROP_SIZE); every PSVR2_HT_CROP_EVERY-th frame.
 */
DEBUG_GET_ONCE_BOOL_OPTION(psvr2_ht_crops_upright, "PSVR2_HT_CROPS_UPRIGHT", false)
DEBUG_GET_ONCE_NUM_OPTION(psvr2_ht_crop_size, "PSVR2_HT_CROP_SIZE", 48)
DEBUG_GET_ONCE_NUM_OPTION(psvr2_ht_crop_every, "PSVR2_HT_CROP_EVERY", 1)


#define HT_TRACE(h, ...) U_LOG_IFL_T(h->log_level, __VA_ARGS__)
#define HT_DEBUG(h, ...) U_LOG_IFL_D(h->log_level, __VA_ARGS__)
#define HT_INFO(h, ...) U_LOG_IFL_I(h->log_level, __VA_ARGS__)
#define HT_WARN(h, ...) U_LOG_IFL_W(h->log_level, __VA_ARGS__)
#define HT_ERROR(h, ...) U_LOG_IFL_E(h->log_level, __VA_ARGS__)

// Camera mode 0x10 transfer: 256 byte header, then two BC4 images of 1024 (stride) x 1016 rows, back to back:
// (1040640 - 256) / 2 = 520192 = 1024 * 1016 / 2. 1016x1016 is the calibrated image size.
#define CAM_HEADER 256
#define CAM_W 1016
#define CAM_H 1016
#define CAM_STRIDE 1024
#define CAM_IMAGE_BYTES (CAM_STRIDE * CAM_H / 2)
#define CAM_TRANSFER_BYTES (CAM_HEADER + 2 * CAM_IMAGE_BYTES)

/*
 *
 * Smoothing (One Euro filter).
 *
 */

//! One value's filter state: the filtered value and its filtered speed.
struct ht_oe
{
	bool init;
	struct xrt_vec3 pos;
	struct xrt_quat rot;
	float speed_pos, speed_rot; //!< filtered speeds, m/s and rad/s
};

struct ht_hand_filter
{
	int64_t last_ns; //!< timestamp of the last filtered result (0: no state)
	struct ht_oe palm;
	struct ht_oe joint[XRT_HAND_JOINT_COUNT];
};

//! The smoothing factor for cutoff @p cutoff (Hz) over @p dt (s).
static inline float
ht_oe_alpha(float cutoff, float dt)
{
	const float r = 2.0f * (float)M_PI * cutoff * dt;
	return r / (r + 1.0f);
}

//! The speed's own filter has a fixed 1 Hz cutoff, as in the paper.
static void
ht_oe_step(struct ht_oe *f, const struct xrt_pose *in, float dt, float min_cutoff, float beta_pos, float beta_rot)
{
	if (!f->init) {
		f->init = true;
		f->pos = in->position;
		f->rot = in->orientation;
		f->speed_pos = f->speed_rot = 0.0f;
		return;
	}
	// Position.
	struct xrt_vec3 d = m_vec3_sub(in->position, f->pos);
	const float lin_speed = m_vec3_len(d) / dt;
	// Rotation: the angle between the filtered and the new orientation.
	struct xrt_quat q = in->orientation;
	float dot = f->rot.x * q.x + f->rot.y * q.y + f->rot.z * q.z + f->rot.w * q.w;
	if (dot < 0.0f) {
		q.x = -q.x, q.y = -q.y, q.z = -q.z, q.w = -q.w;
		dot = -dot;
	}
	const float ang_speed = 2.0f * acosf(fminf(1.0f, dot)) / dt;
	// Position and rotation are filtered separately, each with its own speed and beta.
	f->speed_pos += ht_oe_alpha(1.0f, dt) * (lin_speed - f->speed_pos);
	f->speed_rot += ht_oe_alpha(1.0f, dt) * (ang_speed - f->speed_rot);
	const float a_pos = ht_oe_alpha(min_cutoff + beta_pos * f->speed_pos, dt);
	const float a_rot = ht_oe_alpha(min_cutoff + beta_rot * f->speed_rot, dt);
	f->pos = m_vec3_add(f->pos, m_vec3_mul_scalar(d, a_pos));
	struct xrt_quat out;
	math_quat_slerp(&f->rot, &q, a_rot, &out);
	math_quat_normalize(&out);
	f->rot = out;
}

//! Side of a gesture crop (pixels).
#define CROP_SIZE 48

//! One gesture crop in crops.bin (PSVR2_HT_CROPS), followed by CROP_SIZE * CROP_SIZE bytes.
struct crop_record
{
	int64_t frame_ns;  //!< the camera frame's timestamp
	int64_t joints_ns; //!< the joints' timestamp the crop was placed with
	uint8_t hand;      //!< 0 left, 1 right
	uint8_t view;      //!< 0, 1: the camera
	uint16_t box;      //!< the square box's side in the unwarped view (pixels), before the resize
	int16_t x0, y0;    //!< its top left corner (upright crops: its centre)
	uint16_t size;     //!< the crop's side (pixels)
	float angle;       //!< upright crops: the turn applied (radians), else 0
} __attribute__((packed));

struct psvr2_hand_tracking
{
	struct xrt_device base;
	struct xrt_device *head;
	enum u_logging_level log_level;

	struct os_thread_helper worker;

	// Under the worker lock.
	bool have_transfer;
	int64_t transfer_ns;
	uint8_t *transfer;
	bool cameras_pending;
	struct t_camera_calibration pending_cams[2];
	struct xrt_pose pending_poses[2];
	uint64_t offered, taken;

	// Worker thread only, until running is set.
	struct xrt_frame_context xfctx;
	struct t_hand_tracking_async *async; // NULL until Mercury is running
	int out_size;
	float *map_x[2], *map_y[2]; // per unwarped pixel, the source position, -1 outside
	uint8_t lut[256];           // the fixed brightening curve (also the first frame's, before auto levels)
	uint8_t view_lut[2][256];   // each view's curve: auto levels then the brightening
	uint32_t view_hist[2][256]; // each view's tones in the current frame (every 4th pixel), for the next frame's levels
	float level_lo[2], level_hi[2];
	bool levels_init[2];
	float gamma;
	uint8_t *decoded[2];        // CAM_W x CAM_H each
	uint64_t processed;
	//! Worker thread: brightness of the images Mercury gets (unwarped view 0, before the brightening), per 5 s.
	uint32_t bright_hist[256];
	uint64_t bright_frames;
	int64_t bright_since_ns;
	int64_t last_report_ns;

	// Status report every 5 s from get_hand_tracking (under pose_lock): how often each hand is found and where.
	int64_t stats_since_ns;
	uint32_t stats_queries[2], stats_active[2], stats_held[2];
	//! Under pose_lock: each hand's last found joints (room space) and when, for the short hold (PSVR2_HT_HOLD_MS).
	struct xrt_hand_joint_set last_found[2];
	int64_t last_found_ns[2];
	//! Gestures (psvr2_hand_tracking_get_gestures): Mercury (NULL until running), whether the latest query found
	//! each hand (not held), and the newest camera frame's timestamp given to Mercury (all under pose_lock).
	struct t_hand_tracking_sync *sync;
	bool found_now[2];
	int64_t newest_frame_ns;
	struct xrt_vec3 stats_wrist_head[2], stats_wrist_world[2]; // last seen
	//! Under pose_lock: each hand's smoothing state (PSVR2_HT_FILTER).
	struct ht_hand_filter filter[2];

	// Read by get_hand_tracking.
	struct os_mutex pose_lock;
	struct xrt_pose cam0_in_head;
	struct xrt_pose cam1_in_head;
	volatile bool running;
	//! Worker thread: PSVR2_HT_CROPS output (NULL: off) and the unwarped views' focal length (pixels).
	FILE *crops;
	float view_f;
	/*!
	 * Whether hands are wanted now (psvr2_hand_tracking_set_active): only then are the bottom cameras asked for and
	 * their frames processed. Under the worker's lock.
	 */
	bool active;
};

static inline struct psvr2_hand_tracking *
psvr2_ht(struct xrt_device *xdev)
{
	return (struct psvr2_hand_tracking *)xdev;
}


/*
 *
 * Worker.
 *
 */

static void
build_unwarp(struct psvr2_hand_tracking *h, int view, const struct t_camera_calibration *cam, float fov_deg, int n)
{
	struct t_camera_model_params mp;
	t_camera_model_params_from_t_camera_calibration(cam, &mp);
	h->map_x[view] = U_TYPED_ARRAY_CALLOC(float, (size_t)n * n);
	h->map_y[view] = U_TYPED_ARRAY_CALLOC(float, (size_t)n * n);
	float f = (n / 2.0f) / tanf(fov_deg * (float)M_PI / 360.0f);
	for (int y = 0; y < n; y++) {
		for (int x = 0; x < n; x++) {
			size_t i = (size_t)y * n + x;
			float sx = -1, sy = -1;
			// Pinhole ray in OpenCV camera axes (x right, y down, z forward), same axes as the camera.
			if (!t_camera_models_project(&mp, (x - n / 2.0f) / f, (y - n / 2.0f) / f, 1.0f, &sx, &sy) ||
			    sx < 0 || sy < 0 || sx >= CAM_W - 1 || sy >= CAM_H - 1) {
				sx = sy = -1;
			}
			h->map_x[view][i] = sx;
			h->map_y[view][i] = sy;
		}
	}
}

//! Start Mercury for the given cameras (worker thread: loading the models takes a while).
static void
start_mercury(struct psvr2_hand_tracking *h, const struct t_camera_calibration cams[2], const struct xrt_pose poses[2])
{
	float fov = debug_get_float_option_psvr2_ht_fov();
	float gamma = debug_get_float_option_psvr2_ht_gamma();
	const bool raw = debug_get_bool_option_psvr2_ht_raw();
	int n = raw ? CAM_W : (int)debug_get_num_option_psvr2_ht_size();
	h->out_size = n;
	h->gamma = gamma;
	for (int i = 0; i < 256; i++) {
		h->lut[i] = (uint8_t)lroundf(255.0f * powf(i / 255.0f, gamma));
	}
	memcpy(h->view_lut[0], h->lut, sizeof(h->lut));
	memcpy(h->view_lut[1], h->lut, sizeof(h->lut));
	for (int v = 0; v < 2; v++) {
		if (raw) {
			// One to one: the camera's own pixels (the last row and column stay out, the sampling reads x+1, y+1).
			h->map_x[v] = U_TYPED_ARRAY_CALLOC(float, (size_t)n * n);
			h->map_y[v] = U_TYPED_ARRAY_CALLOC(float, (size_t)n * n);
			for (int y = 0; y < n; y++) {
				for (int x = 0; x < n; x++) {
					const bool in = x < CAM_W - 1 && y < CAM_H - 1;
					h->map_x[v][(size_t)y * n + x] = in ? (float)x : -1.0f;
					h->map_y[v][(size_t)y * n + x] = in ? (float)y : -1.0f;
				}
			}
		} else {
			build_unwarp(h, v, &cams[v], fov, n);
		}
		h->decoded[v] = U_TYPED_ARRAY_CALLOC(uint8_t, (size_t)CAM_W * CAM_H);
	}

	// Mercury sees two distortion-free pinhole cameras (RT8 with zero coefficients) with the physical cameras' axes,
	// so the stereo extrinsics are the physical ones: x_cam1 = R x_cam0 + T in OpenCV camera axes.
	struct t_stereo_camera_calibration *stereo = NULL;
	t_stereo_camera_calibration_alloc(&stereo, raw ? cams[0].distortion_model : T_DISTORTION_OPENCV_RADTAN_8);
	float f = (n / 2.0f) / tanf(fov * (float)M_PI / 360.0f);
	for (int v = 0; v < 2; v++) {
		struct t_camera_calibration *cc = &stereo->view[v];
		if (raw) {
			*cc = cams[v]; // the camera's own intrinsics and lens model, on its own 1016x1016 image
			continue;
		}
		cc->image_size_pixels.w = (uint32_t)n;
		cc->image_size_pixels.h = (uint32_t)n;
		cc->distortion_model = T_DISTORTION_OPENCV_RADTAN_8;
		memset(cc->distortion_parameters_as_array, 0, sizeof(cc->distortion_parameters_as_array));
		memset(cc->intrinsics, 0, sizeof(cc->intrinsics));
		cc->intrinsics[0][0] = cc->intrinsics[1][1] = f;
		cc->intrinsics[0][2] = cc->intrinsics[1][2] = n / 2.0;
		cc->intrinsics[2][2] = 1;
	}
	// cam1_from_cam0 in OpenXR axes, then into OpenCV axes on both sides (F = diag(1, -1, -1)).
	struct xrt_pose inv1, c1_c0;
	math_pose_invert(&poses[1], &inv1);
	math_pose_transform(&inv1, &poses[0], &c1_c0);
	struct xrt_matrix_3x3 r;
	math_matrix_3x3_from_quat(&c1_c0.orientation, &r);
	const double F[3] = {1, -1, -1};
	for (int a = 0; a < 3; a++) {
		for (int b = 0; b < 3; b++) {
			stereo->camera_rotation[a][b] = F[a] * r.v[a * 3 + b] * F[b];
		}
	}
	stereo->camera_translation[0] = F[0] * c1_c0.position.x;
	stereo->camera_translation[1] = F[1] * c1_c0.position.y;
	stereo->camera_translation[2] = F[2] * c1_c0.position.z;
	HT_INFO(h, "Hand tracking cameras: baseline %.1f mm; unwarped %dx%d, %.0f deg, gamma %.2f",
	        m_vec3_len(c1_c0.position) * 1000.0f, n, n, fov, gamma);
	HT_INFO(h, "Hand tracking stereo (OpenCV axes, x_cam1 = R x_cam0 + T): R [%.5f %.5f %.5f; %.5f %.5f %.5f; %.5f %.5f %.5f] "
	        "T [%.4f %.4f %.4f] m",
	        stereo->camera_rotation[0][0], stereo->camera_rotation[0][1], stereo->camera_rotation[0][2],
	        stereo->camera_rotation[1][0], stereo->camera_rotation[1][1], stereo->camera_rotation[1][2],
	        stereo->camera_rotation[2][0], stereo->camera_rotation[2][1], stereo->camera_rotation[2][2],
	        stereo->camera_translation[0], stereo->camera_translation[1], stereo->camera_translation[2]);

	char models[1024] = {0};
	if (u_file_get_hand_tracking_models_dir(models, sizeof(models)) < 0) {
		HT_ERROR(h, "No hand tracking models directory (scripts/get-ht-models.sh); hand tracking off");
		t_stereo_camera_calibration_reference(&stereo, NULL);
		return;
	}

	struct t_hand_tracking_create_info info = {0};
	for (int v = 0; v < 2; v++) {
		info.cams_info.views[v].boundary_type = HT_IMAGE_BOUNDARY_NONE; // unwarped views are filled edge to edge
		info.cams_info.views[v].camera_orientation = CAMERA_ORIENTATION_0;
		if (raw) {
			info.cams_info.views[v].boundary_type = HT_IMAGE_BOUNDARY_CIRCLE;
			info.cams_info.views[v].boundary.circle.normalized_center.x = 0.5f;
			info.cams_info.views[v].boundary.circle.normalized_center.y = 0.5f;
			info.cams_info.views[v].boundary.circle.normalized_radius = debug_get_float_option_psvr2_ht_raw_radius();
		}
	}
	if (raw) {
		HT_INFO(h, "Hand tracking: the cameras' own fisheye images (PSVR2_HT_RAW), boundary radius %.2f",
		        debug_get_float_option_psvr2_ht_raw_radius());
	}
	/*
	 * Start hands from one confident camera: with the mean of both views (Mercury's default), the left camera's
	 * near-zero scores cancelled the right camera's 0.99 detections and the right hand was found in 2-10% of its
	 * frames on the recordings, 81% with this. Mercury reads the option once at creation; an explicit setting wins.
	 */
	setenv("MERCURY_DETECTION_ANY_VIEW", "1", 0);
	/*
	 * Left hand: Mercury's detector finds right hands far more readily, so also detect on the mirrored image (its
	 * right hand is our left), and start hands at 0.7 confidence instead of 0.92. On the recordings: left hand found
	 * 62% -> 73% of the hands-in-view time (live run) and 0% -> 9% (earlier run, where it was never found), right
	 * hand 85% -> 97% (earlier run); no detections away from hands, no two hands on one. Overlays put the new left
	 * hand pickups on the left hand. Costs one more detector run per view (Mercury ~4.9 -> 6.7 ms per frame).
	 */
	setenv("MERCURY_DETECTION_MIRROR_LEFT", "1", 0);
	/*
	 * 0.55 since 2026-11-23 (with the auto levels): small hands far out in front, side by side near a bright monitor
	 * (the user's second headset, dark room) were seen but scored below 0.7, so the left hand was rarely picked up.
	 * Replays: left 57% -> 85% and 19% -> 78% on the two new recordings, right unchanged (70-87%); old desk recording
	 * left 68-79%, right 81%; no false hands in any empty stretch. 0.4 started a phantom left hand (6%) there.
	 */
	setenv("MERCURY_ACQUIRE_CONFIDENCE", "0.55", 0);
	/*
	 * Keep a view only while the keypoint model is 98% sure it's a hand (Mercury: 97%). A right hand raised quickly
	 * out of the top of the view left its joints on dark furniture for half a second at 0.93-0.98 (real hands: median
	 * 0.9992, 95% above 0.986); 0.98 dropped it at once and cost 0-7 points of tracked time on the recordings.
	 * (2026-11-20, scored against frames labelled by hand: 0.92 tracked exactly the same, so it stays.)
	 */
	setenv("MERCURY_KEYPOINT_HAND_CONFIDENCE", "0.98", 0);
	/*
	 * Mercury's zoomed-in detection tiles: while a hand isn't found, the detector also looks at a 3x3 grid of
	 * overlapping crops, all nine every frame, run in parallel (hg_model.cpp run_detection_tile). On the 2026-11-20
	 * recording, replayed through this module (psvr2-test/hands/driver_ht_replay: share of SteamVR queries a hand is
	 * active, hands over the desk 2-35 s / none 37-43 s): off left 22% right 63%, one tile a frame 52% / 65%, all nine
	 * in parallel 77% / 68%, no false hands in any; the tiles took 3.7 ms per searching frame (one after another
	 * ~17 ms, too slow for the 60 Hz stream), CPU 0.55 -> 1.0 cores. MERCURY_DETECTION_TILES=0 turns them off.
	 */
	setenv("MERCURY_DETECTION_TILES", "3", 0);
	setenv("MERCURY_DETECTION_TILES_PER_FRAME", "9", 0);
	/*
	 * MediaPipe's hand landmark model reads the fingers (and its Gesture Recognizer the gestures) when its model is
	 * installed next to Mercury's: on the 2026-11-24 headset sessions pinch went from almost never registering to
	 * 3-4 of 6 prompts per hand, fist to 2 of 2, and with the cameras well lit both hands tracked 90%+. Without
	 * hand_landmarks_detector.onnx Mercury's own model as before (MediaPipe on without it would stop Mercury from
	 * starting). MERCURY_MEDIAPIPE=0 / MERCURY_MEDIAPIPE_GESTURES=0 turn them off.
	 */
	{
		char mp[1100];
		snprintf(mp, sizeof(mp), "%s/hand_landmarks_detector.onnx", models);
		FILE *probe = fopen(mp, "rb");
		if (probe != NULL) {
			fclose(probe);
			setenv("MERCURY_MEDIAPIPE", "1", 0);
			setenv("MERCURY_MEDIAPIPE_GESTURES", "1", 0);
		} else if (getenv("MERCURY_MEDIAPIPE") == NULL) {
			HT_INFO(h, "MediaPipe's hand landmark model isn't installed (%s): Mercury's own finger model", mp);
		}
	}
	struct t_hand_tracking_sync *sync = t_hand_tracking_sync_mercury_create(stereo, info, models);
	t_stereo_camera_calibration_reference(&stereo, NULL);
	if (sync == NULL) {
		HT_ERROR(h, "Mercury failed to start; hand tracking off");
		return;
	}
	h->async = t_hand_tracking_async_default_create(&h->xfctx, sync);
	os_mutex_lock(&h->pose_lock);
	h->sync = sync;
	os_mutex_unlock(&h->pose_lock);

	os_mutex_lock(&h->pose_lock);
	h->cam0_in_head = poses[0];
	h->cam1_in_head = poses[1];
	os_mutex_unlock(&h->pose_lock);
	h->view_f = f;
	const char *crops = debug_get_option_psvr2_ht_crops();
	if (crops != NULL && !raw) {
		char path[1024];
		snprintf(path, sizeof(path), "%s/crops.bin", crops);
		h->crops = fopen(path, "wb");
		HT_INFO(h, "Gesture crops (%dx%d per hand and view) to %s%s", CROP_SIZE, CROP_SIZE, path,
		        h->crops != NULL ? "" : ": can't write");
	}
	h->running = true;
	HT_INFO(h, "Hand tracking running");
}

/*!
 * Every 5 s: how bright the camera images are that Mercury looks for hands in, before and after the brightening
 * (mean, median, 95th and 99th percentile of the unwarped view 0, 0-255). A dark picture is the first suspect when
 * hands aren't found (the cameras expose for the controllers' LEDs).
 */
static void
log_brightness(struct psvr2_hand_tracking *h, int64_t ts)
{
	h->bright_frames++;
	if (h->bright_since_ns == 0) {
		h->bright_since_ns = ts;
		return;
	}
	if (ts - h->bright_since_ns < 5 * U_TIME_1S_IN_NS) {
		return;
	}
	uint64_t total = 0, sum = 0, sum_out = 0;
	for (int i = 0; i < 256; i++) {
		total += h->bright_hist[i];
		sum += (uint64_t)i * h->bright_hist[i];
		sum_out += (uint64_t)h->view_lut[0][i] * h->bright_hist[i];
	}
	int pct[3] = {0, 0, 0};
	const double want[3] = {0.5, 0.95, 0.99};
	uint64_t run = 0;
	int k = 0;
	for (int i = 0; i < 256 && k < 3; i++) {
		run += h->bright_hist[i];
		while (k < 3 && total > 0 && (double)run >= want[k] * (double)total) {
			pct[k++] = i;
		}
	}
	if (total > 0) {
		HT_INFO(h,
		        "Camera brightness (5 s, %llu frames): mean %.1f, median %d, 95%% %d, 99%% %d of 255; after the "
		        "brightening mean %.1f, 95%% %d, 99%% %d",
		        (unsigned long long)h->bright_frames, (double)sum / (double)total, pct[0], pct[1], pct[2],
		        (double)sum_out / (double)total, h->view_lut[0][pct[1]], h->view_lut[0][pct[2]]);
	}
	memset(h->bright_hist, 0, sizeof(h->bright_hist));
	h->bright_frames = 0;
	h->bright_since_ns = ts;
}

/*!
 * The next frame's curve for view @p v from this frame's tones: the LEVELS_LO..LEVELS_HI percentile range (smoothed,
 * at least 24 levels wide) stretched to 0..255, then the brightening (gamma).
 */
static void
update_levels(struct psvr2_hand_tracking *h, int v)
{
	uint32_t *hist = h->view_hist[v];
	uint64_t total = 0;
	for (int i = 0; i < 256; i++) {
		total += hist[i];
	}
	if (total < 1000 || !debug_get_bool_option_psvr2_ht_levels()) {
		memset(hist, 0, sizeof(h->view_hist[v]));
		return;
	}
	const double want_lo = debug_get_float_option_psvr2_ht_levels_lo() * (double)total;
	const double want_hi = debug_get_float_option_psvr2_ht_levels_hi() * (double)total;
	int lo = 0, hi = 255;
	uint64_t run = 0;
	bool have_lo = false;
	for (int i = 0; i < 256; i++) {
		run += hist[i];
		if (!have_lo && (double)run >= want_lo) {
			lo = i;
			have_lo = true;
		}
		if ((double)run >= want_hi) {
			hi = i;
			break;
		}
	}
	memset(hist, 0, sizeof(h->view_hist[v]));
	// ~0.3 s smoothing at 60 frames a second, so the picture doesn't pump when a hand or the monitor moves in view.
	const float a = 0.05f;
	if (!h->levels_init[v]) {
		h->level_lo[v] = (float)lo;
		h->level_hi[v] = (float)hi;
		h->levels_init[v] = true;
	} else {
		h->level_lo[v] += a * ((float)lo - h->level_lo[v]);
		h->level_hi[v] += a * ((float)hi - h->level_hi[v]);
	}
	float l = h->level_lo[v], top = h->level_hi[v];
	if (top - l < 24.0f) {
		top = l + 24.0f;
	}
	for (int i = 0; i < 256; i++) {
		float t = ((float)i - l) / (top - l);
		t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
		h->view_lut[v][i] = (uint8_t)lroundf(255.0f * powf(t, h->gamma));
	}
}

/*!
 * PSVR2_HT_CROPS: a crop around each tracked hand in each unwarped view (worker thread). The hand is placed from its
 * latest joints (room space, as get_hand_tracking hands them out), brought into each camera with the head pose at
 * their time and projected with the views' pinhole model; the square box around the joints grows by a third and is
 * resized to CROP_SIZE.
 */
static void
write_crops(struct psvr2_hand_tracking *h, struct xrt_frame *const views[2], int64_t frame_ns)
{
	if (h->crops == NULL || (h->processed % (uint64_t)debug_get_num_option_psvr2_ht_crop_every()) != 0) {
		return;
	}
	for (int hand = 0; hand < 2; hand++) {
		struct xrt_hand_joint_set set;
		int64_t joints_ns;
		struct xrt_pose cam_in_head[2];
		os_mutex_lock(&h->pose_lock);
		set = h->last_found[hand];
		joints_ns = h->last_found_ns[hand];
		cam_in_head[0] = h->cam0_in_head;
		cam_in_head[1] = h->cam1_in_head;
		os_mutex_unlock(&h->pose_lock);
		// Only joints from within ~2 frames of this one (a found hand, not a stale or held one).
		if (joints_ns == 0 || !set.is_active || llabs(frame_ns - joints_ns) > 40 * U_TIME_1MS_IN_NS) {
			continue;
		}
		struct xrt_space_relation head = XRT_SPACE_RELATION_ZERO;
		if (xrt_device_get_tracked_pose(h->head, XRT_INPUT_GENERIC_HEAD_POSE, joints_ns, &head) != XRT_SUCCESS) {
			continue;
		}
		struct xrt_pose head_inv;
		math_pose_invert(&head.pose, &head_inv);
		for (int v = 0; v < 2; v++) {
			struct xrt_frame *img = views[v];
			const int n = (int)img->width;
			struct xrt_pose cam_inv;
			math_pose_invert(&cam_in_head[v], &cam_inv);
			float minx = 1e9f, miny = 1e9f, maxx = -1e9f, maxy = -1e9f;
			float proj[XRT_HAND_JOINT_COUNT][2] = {{0}};
			int behind = 0;
			for (int j = 0; j < XRT_HAND_JOINT_COUNT; j++) {
				struct xrt_pose room, in_head, in_cam;
				math_pose_transform(&set.hand_pose.pose, &set.values.hand_joint_set_default[j].relation.pose, &room);
				math_pose_transform(&head_inv, &room, &in_head);
				math_pose_transform(&cam_inv, &in_head, &in_cam);
				const struct xrt_vec3 p = in_cam.position; // OpenXR camera axes: x right, y up, -z forward
				if (p.z > -0.05f) {
					behind++;
					continue;
				}
				const float u = n / 2.0f + h->view_f * p.x / -p.z;
				const float w = n / 2.0f + h->view_f * p.y / p.z;
				minx = fminf(minx, u), maxx = fmaxf(maxx, u), miny = fminf(miny, w), maxy = fmaxf(maxy, w);
				proj[j][0] = u, proj[j][1] = w;
			}
			if (behind > 0) {
				continue;
			}
			const int size = (int)debug_get_num_option_psvr2_ht_crop_size();
			const bool upright = debug_get_bool_option_psvr2_ht_crops_upright();
			float cx = (minx + maxx) / 2.0f, cy = (miny + maxy) / 2.0f;
			float side = fmaxf(maxx - minx, maxy - miny) * 1.33f;
			float angle = 0.0f;
			if (upright) {
				// wrist (1) to the middle finger's proximal joint (12): up in the crop; box 2.6x that length (MediaPipe).
				const float dx = proj[12][0] - proj[1][0], dy = proj[12][1] - proj[1][1];
				const float len = sqrtf(dx * dx + dy * dy);
				angle = atan2f(dx, -dy); // 0 when the hand points up in the view
				side = len * 2.6f;
				cx = (proj[1][0] + proj[12][0]) / 2.0f - 0.25f * dx; // MediaPipe shifts the box toward the fingers
				cy = (proj[1][1] + proj[12][1]) / 2.0f - 0.25f * dy;
				cx += 0.5f * dx, cy += 0.5f * dy;
			}
			const int box = (int)lroundf(side);
			if (box < 16 || cx - side / 2 < 0 || cy - side / 2 < 0 || cx + side / 2 >= n - 1 || cy + side / 2 >= n - 1) {
				continue; // too small or partly out of the view
			}
			uint8_t *pix = (uint8_t *)malloc((size_t)size * size);
			const float ca = cosf(angle), sa = sinf(angle);
			for (int y = 0; y < size; y++) {
				for (int x = 0; x < size; x++) {
					const float u = ((x + 0.5f) / size - 0.5f) * side, w = ((y + 0.5f) / size - 0.5f) * side;
					const float sx = cx + ca * u - sa * w, sy = cy + sa * u + ca * w;
					const int ix = (int)sx, iy = (int)sy;
					const float fx = sx - ix, fy = sy - iy;
					const uint8_t *r0 = img->data + (size_t)iy * img->stride + ix;
					const uint8_t *r1 = r0 + img->stride;
					const float top = r0[0] + (r0[1] - r0[0]) * fx, bot = r1[0] + (r1[1] - r1[0]) * fx;
					pix[y * size + x] = (uint8_t)(top + (bot - top) * fy + 0.5f);
				}
			}
			struct crop_record rec = {frame_ns, joints_ns, (uint8_t)hand, (uint8_t)v, (uint16_t)box,
			                          (int16_t)lroundf(upright ? cx : cx - side / 2), (int16_t)lroundf(upright ? cy : cy - side / 2),
			                          (uint16_t)size, angle};
			fwrite(&rec, sizeof(rec), 1, h->crops);
			fwrite(pix, 1, (size_t)size * size, h->crops);
			free(pix);
		}
	}
}

static void
decode_and_push(struct psvr2_hand_tracking *h, const uint8_t *transfer, int64_t ts)
{
	const size_t row_bytes = (CAM_STRIDE / 4) * BCDEC_BC4_BLOCK_SIZE;
	struct xrt_frame *out[2] = {NULL, NULL};
	for (int v = 0; v < 2; v++) {
		const uint8_t *src = transfer + CAM_HEADER + (size_t)v * CAM_IMAGE_BYTES;
		uint8_t *dst = h->decoded[v];
		for (int by = 0; by < CAM_H / 4; by++) {
			for (int bx = 0; bx < CAM_W / 4; bx++) {
				bcdec_bc4(src + by * row_bytes + bx * BCDEC_BC4_BLOCK_SIZE, dst + (by * 4) * CAM_W + bx * 4,
				          CAM_W, 0);
			}
		}

		int n = h->out_size;
		u_frame_create_one_off(XRT_FORMAT_L8, n, n, &out[v]);
		out[v]->timestamp = ts;
		const float *mx = h->map_x[v], *my = h->map_y[v];
		const uint8_t *lut = h->view_lut[v];
		uint32_t *vh = h->view_hist[v];
		for (int y = 0; y < n; y++) {
			uint8_t *row = out[v]->data + (size_t)y * out[v]->stride;
			for (int x = 0; x < n; x++) {
				size_t i = (size_t)y * n + x;
				uint8_t val = 0;
				if (mx[i] >= 0) {
					int x0 = (int)mx[i], y0 = (int)my[i];
					float fx = mx[i] - x0, fy = my[i] - y0;
					const uint8_t *p = dst + (size_t)y0 * CAM_W + x0;
					float top = p[0] + (p[1] - p[0]) * fx;
					float bot = p[CAM_W] + (p[CAM_W + 1] - p[CAM_W]) * fx;
					val = (uint8_t)(top + (bot - top) * fy + 0.5f);
				}
				row[x] = lut[val];
				if (((x | y) & 3) == 0 && mx[i] >= 0) {
					vh[val]++;
					if (v == 0) {
						h->bright_hist[val]++;
					}
				}
			}
		}
		update_levels(h, v);
	}
	log_brightness(h, ts);
	const char *dump = debug_get_option_psvr2_ht_dump();
	if (dump != NULL && h->processed % 30 == 0) {
		for (int v = 0; v < 2; v++) {
			char path[1024];
			snprintf(path, sizeof(path), "%s/f%05llu_view%d_raw.pgm", dump, (unsigned long long)h->processed, v);
			FILE *f = fopen(path, "wb");
			if (f != NULL) {
				fprintf(f, "P5\n%d %d\n255\n", CAM_W, CAM_H);
				fwrite(h->decoded[v], 1, (size_t)CAM_W * CAM_H, f);
				fclose(f);
			}
			snprintf(path, sizeof(path), "%s/f%05llu_view%d_unwarped.pgm", dump, (unsigned long long)h->processed, v);
			f = fopen(path, "wb");
			if (f != NULL) {
				fprintf(f, "P5\n%d %d\n255\n", out[v]->width, out[v]->height);
				for (uint32_t y = 0; y < out[v]->height; y++) {
					fwrite(out[v]->data + (size_t)y * out[v]->stride, 1, out[v]->width, f);
				}
				fclose(f);
			}
		}
	}
	write_crops(h, out, ts);
	os_mutex_lock(&h->pose_lock);
	h->newest_frame_ns = ts;
	os_mutex_unlock(&h->pose_lock);
	// The async helper runs Mercury on its own thread once it has both views (same timestamp).
	xrt_sink_push_frame(&h->async->left, out[0]);
	xrt_sink_push_frame(&h->async->right, out[1]);
	xrt_frame_reference(&out[0], NULL);
	xrt_frame_reference(&out[1], NULL);
	h->processed++;
}

static void *
worker_thread(void *ptr)
{
	struct psvr2_hand_tracking *h = (struct psvr2_hand_tracking *)ptr;
	U_TRACE_SET_THREAD_NAME("PS VR2 hand tracking");
	uint8_t *local = U_TYPED_ARRAY_CALLOC(uint8_t, CAM_TRANSFER_BYTES);

	os_thread_helper_lock(&h->worker);
	while (os_thread_helper_is_running_locked(&h->worker)) {
		if (!h->have_transfer && !h->cameras_pending) {
			os_thread_helper_wait_locked(&h->worker);
			continue;
		}
		bool start = h->cameras_pending && h->async == NULL;
		struct t_camera_calibration cams[2];
		struct xrt_pose poses[2];
		if (start) {
			memcpy(cams, h->pending_cams, sizeof(cams));
			memcpy(poses, h->pending_poses, sizeof(poses));
		}
		h->cameras_pending = false;
		bool have = h->have_transfer;
		int64_t ts = h->transfer_ns;
		if (have) {
			memcpy(local, h->transfer, CAM_TRANSFER_BYTES);
			h->have_transfer = false;
		}
		os_thread_helper_unlock(&h->worker);

		if (start) {
			start_mercury(h, cams, poses);
		}
		if (have && h->async != NULL) {
			decode_and_push(h, local, ts);
		}
		int64_t now = os_monotonic_get_ns();
		if (now - h->last_report_ns > 10 * U_TIME_1S_IN_NS) {
			HT_DEBUG(h, "Hand tracking: %llu frames processed", (unsigned long long)h->processed);
			h->last_report_ns = now;
		}

		os_thread_helper_lock(&h->worker);
	}
	os_thread_helper_unlock(&h->worker);
	free(local);
	return NULL;
}


/*
 *
 * Device.
 *
 */

/*!
 * Smooths one found hand in place (under pose_lock): the palm's pose in the room, then each joint relative to the
 * palm; the result has the filtered palm as the hand pose and the joints relative to it. A query within one display
 * refresh of the last step (or for an older time) gets that result again (several threads ask for each hand, the
 * SteamVR plugin's pose thread every 1 ms; the filter steps forward once per refresh). After a loss up to the hold time the filter carries on from where it was, with its step capped at one
 * 30 Hz frame, so the hand slides to its new place instead of snapping there; after a longer loss it starts afresh.
 */
static void
smooth_hand(struct psvr2_hand_tracking *h, int hand, struct xrt_hand_joint_set *set, int64_t ts)
{
	struct ht_hand_filter *f = &h->filter[hand];
	const int64_t hold_ns = (int64_t)debug_get_num_option_psvr2_ht_hold_ms() * U_TIME_1MS_IN_NS;
	/*
	 * One step per display refresh at most. The SteamVR plugin asks for each hand every 1 ms (its pose thread) while
	 * Mercury delivers ~60 results a second: stepped on every query, each new result arrived as a jump within 1 ms,
	 * the speed estimate saw that as a very fast hand and opened the filter fully. Measured on the same data (replay
	 * of camera-20261123-204915 queried every 1 ms): palm orientation wobble 0.82 deg raw, 0.78 stepped per query,
	 * 0.48 stepped per refresh; index tip 2.9 / 1.7 / 1.0 mm (medians). Queries in between get the last result.
	 */
	if (f->last_ns != 0 && ts - f->last_ns < 11 * U_TIME_1MS_IN_NS) {
		*set = h->last_found[hand];
		return;
	}
	if (f->last_ns != 0 && ts - f->last_ns > hold_ns + 100 * U_TIME_1MS_IN_NS) {
		memset(f, 0, sizeof(*f));
	}
	float dt = f->last_ns != 0 ? (float)(ts - f->last_ns) / 1e9f : 1.0f / 90.0f;
	dt = fminf(fmaxf(dt, 0.001f), 1.0f / 30.0f);
	f->last_ns = ts;

	const float min_cutoff = debug_get_float_option_psvr2_ht_filter_min_cutoff();
	const float beta_pos = debug_get_float_option_psvr2_ht_filter_beta_pos();
	const float beta_rot = debug_get_float_option_psvr2_ht_filter_beta_rot();

	// Every joint in the room, then relative to the raw palm.
	struct xrt_pose world[XRT_HAND_JOINT_COUNT];
	for (int j = 0; j < XRT_HAND_JOINT_COUNT; j++) {
		math_pose_transform(&set->hand_pose.pose, &set->values.hand_joint_set_default[j].relation.pose, &world[j]);
	}
	struct xrt_pose palm_inv;
	math_pose_invert(&world[XRT_HAND_JOINT_PALM], &palm_inv);

	ht_oe_step(&f->palm, &world[XRT_HAND_JOINT_PALM], dt, min_cutoff, beta_pos, beta_rot);
	const struct xrt_pose palm = {f->palm.rot, f->palm.pos};
	for (int j = 0; j < XRT_HAND_JOINT_COUNT; j++) {
		struct xrt_pose rel;
		math_pose_transform(&palm_inv, &world[j], &rel);
		ht_oe_step(&f->joint[j], &rel, dt, 2.0f * min_cutoff, beta_pos, beta_rot);
		set->values.hand_joint_set_default[j].relation.pose.position = f->joint[j].pos;
		set->values.hand_joint_set_default[j].relation.pose.orientation = f->joint[j].rot;
	}
	set->hand_pose.pose = palm;
}

static xrt_result_t
psvr2_ht_get_hand_tracking(struct xrt_device *xdev,
                           enum xrt_input_name name,
                           int64_t at_timestamp_ns,
                           struct xrt_hand_joint_set *out_value,
                           int64_t *out_timestamp_ns)
{
	struct psvr2_hand_tracking *h = psvr2_ht(xdev);
	if (name != XRT_INPUT_HT_UNOBSTRUCTED_LEFT && name != XRT_INPUT_HT_UNOBSTRUCTED_RIGHT) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&h->base, h->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}
	{
		int64_t now = os_monotonic_get_ns();
		os_mutex_lock(&h->pose_lock);
		if (h->stats_since_ns == 0) {
			h->stats_since_ns = now;
		} else if (now - h->stats_since_ns > 5 * U_TIME_1S_IN_NS) {
			HT_INFO(h,
			        "Hands (5 s): left found %u/%u (+%u held), right found %u/%u (+%u held) queries (%llu frames "
			        "processed); last "
			        "right wrist: head frame %.2f %.2f %.2f m, room %.2f %.2f %.2f m; last left wrist: head "
			        "frame %.2f %.2f %.2f m",
			        h->stats_active[0], h->stats_queries[0], h->stats_held[0], h->stats_active[1],
			        h->stats_queries[1], h->stats_held[1], (unsigned long long)h->processed, h->stats_wrist_head[1].x, h->stats_wrist_head[1].y,
			        h->stats_wrist_head[1].z, h->stats_wrist_world[1].x, h->stats_wrist_world[1].y,
			        h->stats_wrist_world[1].z, h->stats_wrist_head[0].x, h->stats_wrist_head[0].y,
			        h->stats_wrist_head[0].z);
			memset(h->stats_queries, 0, sizeof(h->stats_queries));
			memset(h->stats_active, 0, sizeof(h->stats_active));
			memset(h->stats_held, 0, sizeof(h->stats_held));
			h->stats_since_ns = now;
		}
		os_mutex_unlock(&h->pose_lock);
	}
	os_thread_helper_lock(&h->worker);
	const bool active = h->active;
	os_thread_helper_unlock(&h->worker);
	if (!h->running || !active) {
		memset(out_value, 0, sizeof(*out_value));
		out_value->is_active = false;
		*out_timestamp_ns = at_timestamp_ns;
		return XRT_SUCCESS;
	}

	int hand = name == XRT_INPUT_HT_UNOBSTRUCTED_RIGHT ? 1 : 0;
	h->async->get_hand(h->async, name, at_timestamp_ns, out_value, out_timestamp_ns);
	os_mutex_lock(&h->pose_lock);
	h->found_now[hand] = out_value->is_active;
	os_mutex_unlock(&h->pose_lock);
	if (!out_value->is_active) {
		const int64_t hold_ns = (int64_t)debug_get_num_option_psvr2_ht_hold_ms() * U_TIME_1MS_IN_NS;
		os_mutex_lock(&h->pose_lock);
		h->stats_queries[hand]++;
		const bool hold = hold_ns > 0 && h->last_found_ns[hand] != 0 &&
		                  at_timestamp_ns - h->last_found_ns[hand] <= hold_ns &&
		                  at_timestamp_ns >= h->last_found_ns[hand];
		if (hold) {
			// The last found hand, where it was in the room, stands in for a moment (no velocity: it stays put).
			*out_value = h->last_found[hand];
			*out_timestamp_ns = at_timestamp_ns;
			h->stats_held[hand]++;
		}
		os_mutex_unlock(&h->pose_lock);
		return XRT_SUCCESS;
	}

	// Mercury's joints are in camera 0 (OpenXR camera axes); place camera 0 with the head pose at that time.
	struct xrt_space_relation head = XRT_SPACE_RELATION_ZERO;
	xrt_result_t xret = xrt_device_get_tracked_pose(h->head, XRT_INPUT_GENERIC_HEAD_POSE, *out_timestamp_ns, &head);
	if (xret != XRT_SUCCESS || (head.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		out_value->is_active = false;
		return XRT_SUCCESS;
	}
	struct xrt_pose cam0_in_head;
	os_mutex_lock(&h->pose_lock);
	cam0_in_head = h->cam0_in_head;
	os_mutex_unlock(&h->pose_lock);

	struct xrt_pose cam_in_world, hand_in_world;
	math_pose_transform(&head.pose, &cam0_in_head, &cam_in_world);
	math_pose_transform(&cam_in_world, &out_value->hand_pose.pose, &hand_in_world);
	out_value->hand_pose.pose = hand_in_world;

	{
		// Status: the last wrist position in the room and in the head frame, and the found ratio per hand.
		const struct xrt_pose *wrist = &out_value->values.hand_joint_set_default[XRT_HAND_JOINT_WRIST].relation.pose;
		struct xrt_pose in_world, head_inv, in_head;
		math_pose_transform(&hand_in_world, wrist, &in_world);
		math_pose_invert(&head.pose, &head_inv);
		math_pose_transform(&head_inv, &in_world, &in_head);
		os_mutex_lock(&h->pose_lock);
		h->stats_queries[hand]++;
		h->stats_active[hand]++;
		h->stats_wrist_head[hand] = in_head.position;
		h->stats_wrist_world[hand] = in_world.position;
		os_mutex_unlock(&h->pose_lock);
	}
	out_value->hand_pose.linear_velocity = (struct xrt_vec3)XRT_VEC3_ZERO;
	out_value->hand_pose.angular_velocity = (struct xrt_vec3)XRT_VEC3_ZERO;
	out_value->hand_pose.relation_flags =
	    (enum xrt_space_relation_flags)(out_value->hand_pose.relation_flags & head.relation_flags &
	                                    (XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                     XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                                     XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                                     XRT_SPACE_RELATION_POSITION_TRACKED_BIT));
	os_mutex_lock(&h->pose_lock);
	if (debug_get_bool_option_psvr2_ht_filter()) {
		smooth_hand(h, hand, out_value, *out_timestamp_ns);
	}
	h->last_found[hand] = *out_value;
	h->last_found_ns[hand] = *out_timestamp_ns;
	os_mutex_unlock(&h->pose_lock);
	return XRT_SUCCESS;
}

bool
psvr2_hand_tracking_get_gestures(struct xrt_device *ht, int hand, struct t_hand_gestures *out)
{
	memset(out, 0, sizeof(*out));
	if (ht == NULL || hand < 0 || hand > 1) {
		return false;
	}
	struct psvr2_hand_tracking *h = psvr2_ht(ht);
	// Mercury is read under pose_lock: destroy clears h->sync under it before Mercury goes away.
	os_mutex_lock(&h->pose_lock);
	const bool have = h->sync != NULL && t_hand_tracking_sync_mercury_get_gestures(h->sync, hand, out);
	const bool found = h->found_now[hand];
	const int64_t newest = h->newest_frame_ns;
	os_mutex_unlock(&h->pose_lock);
	if (!have) {
		return false;
	}
	os_thread_helper_lock(&h->worker);
	const bool active = h->active && h->running;
	os_thread_helper_unlock(&h->worker);
	// Only for a hand found now (not held or lost), measured on a recent frame: Mercury runs a frame or two behind
	// the newest camera frame; older than 100 ms means it fell behind or the hand's result is old.
	const bool fresh = newest != 0 && out->timestamp_ns != 0 && newest - out->timestamp_ns <= 100 * U_TIME_1MS_IN_NS;
	out->valid = out->valid && active && found && fresh;
	return true;
}

static void
psvr2_ht_destroy(struct xrt_device *xdev)
{
	struct psvr2_hand_tracking *h = psvr2_ht(xdev);

	os_thread_helper_destroy(&h->worker); // stops and joins
	h->running = false;
	os_mutex_lock(&h->pose_lock);
	h->sync = NULL; // no gesture reads from here on
	os_mutex_unlock(&h->pose_lock);
	xrt_frame_context_destroy_nodes(&h->xfctx); // the async helper and Mercury
	if (h->crops != NULL) {
		fclose(h->crops);
		h->crops = NULL;
	}
	os_mutex_destroy(&h->pose_lock);
	HT_INFO(h, "Hand tracking stopped: %llu of %llu offered frames processed", (unsigned long long)h->processed,
	        (unsigned long long)h->offered);
	for (int v = 0; v < 2; v++) {
		free(h->map_x[v]);
		free(h->map_y[v]);
		free(h->decoded[v]);
	}
	free(h->transfer);
	u_device_free(&h->base);
}

struct xrt_device *
psvr2_hand_tracking_create(struct xrt_device *head)
{
	struct psvr2_hand_tracking *h =
	    U_DEVICE_ALLOCATE(struct psvr2_hand_tracking, U_DEVICE_ALLOC_NO_FLAGS, 2, 0);
	if (h == NULL) {
		return NULL;
	}
	h->log_level = debug_get_log_option_psvr2_ht_log();
	/*
	 * Pinch thresholds for the emulated controllers (ht_ctrl_emu reads them once, after this). Its 20/40 mm press/
	 * release fired once in a 40 s run with several pinches: Mercury's fingertips on these cameras close to 19-35 mm
	 * (right) and 38-47 mm (left) in a pinch, 60-90 mm relaxed. 40/55 mm caught 6 right and 2 left pinches on the
	 * recording. An explicit setting wins.
	 */
	setenv("CEMU_PINCH_ON_MM", "40", 0);
	setenv("CEMU_PINCH_OFF_MM", "55", 0);
	h->head = head;
	// Same space as the head: hands are placed with the head pose.
	h->base.tracking_origin = head->tracking_origin;

	h->base.update_inputs = u_device_noop_update_inputs;
	h->base.get_hand_tracking = psvr2_ht_get_hand_tracking;
	h->base.destroy = psvr2_ht_destroy;
	snprintf(h->base.str, XRT_DEVICE_NAME_LEN, "PS VR2 Hand Tracking");
	snprintf(h->base.serial, XRT_DEVICE_NAME_LEN, "PS VR2 Hand Tracking");
	h->base.inputs[0].name = XRT_INPUT_HT_UNOBSTRUCTED_LEFT;
	h->base.inputs[1].name = XRT_INPUT_HT_UNOBSTRUCTED_RIGHT;
	h->base.name = XRT_DEVICE_HAND_TRACKER;
	h->base.device_type = XRT_DEVICE_TYPE_HAND_TRACKER;
	h->base.supported.orientation_tracking = true;
	h->base.supported.position_tracking = true;
	h->base.supported.hand_tracking = true;

	h->transfer = U_TYPED_ARRAY_CALLOC(uint8_t, CAM_TRANSFER_BYTES);
	if (h->transfer == NULL || os_mutex_init(&h->pose_lock) != 0 || os_thread_helper_init(&h->worker) != 0) {
		free(h->transfer);
		u_device_free(&h->base);
		return NULL;
	}
	if (os_thread_helper_start(&h->worker, worker_thread, h) != 0) {
		os_thread_helper_destroy(&h->worker);
		os_mutex_destroy(&h->pose_lock);
		free(h->transfer);
		u_device_free(&h->base);
		return NULL;
	}
	HT_INFO(h, "Hand tracking device created; waiting for the camera calibration");
	return &h->base;
}

void
psvr2_hand_tracking_set_cameras(struct xrt_device *xdev,
                                const struct t_camera_calibration cams[2],
                                const struct xrt_pose cam_in_head[2])
{
	struct psvr2_hand_tracking *h = psvr2_ht(xdev);
	os_thread_helper_lock(&h->worker);
	memcpy(h->pending_cams, cams, sizeof(h->pending_cams));
	memcpy(h->pending_poses, cam_in_head, sizeof(h->pending_poses));
	h->cameras_pending = true;
	os_thread_helper_signal_locked(&h->worker);
	os_thread_helper_unlock(&h->worker);
}

void
psvr2_hand_tracking_push_transfer(struct xrt_device *xdev, const uint8_t *data, size_t length, int64_t timestamp_ns)
{
	struct psvr2_hand_tracking *h = psvr2_ht(xdev);
	if (length != CAM_TRANSFER_BYTES || !h->running) {
		return;
	}
	os_thread_helper_lock(&h->worker);
	if (!h->active) {
		// The frames are someone else's (Room View): no hands wanted now, no work.
		os_thread_helper_unlock(&h->worker);
		return;
	}
	h->offered++;
	if (!h->have_transfer) {
		// One copy under the lock (~1 MB); the worker copies it out before decoding, so this never waits long.
		memcpy(h->transfer, data, CAM_TRANSFER_BYTES);
		h->transfer_ns = timestamp_ns;
		h->have_transfer = true;
		h->taken++;
		os_thread_helper_signal_locked(&h->worker);
	}
	os_thread_helper_unlock(&h->worker);
}

void
psvr2_hand_tracking_set_active(struct xrt_device *xdev, bool active)
{
	struct psvr2_hand_tracking *h = psvr2_ht(xdev);
	os_thread_helper_lock(&h->worker);
	const bool changed = h->active != active;
	h->active = active;
	os_thread_helper_unlock(&h->worker);
	if (!changed) {
		return;
	}
	// The bottom cameras' images (camera mode 0x10) while hands are wanted, shared with Room View's requests.
	psvr2_request_bottom_camera_images(h->head, active);
	HT_INFO(h, "Hand tracking %s", active ? "on (a controller is off): the bottom cameras stream to Mercury"
	                                       : "off (both controllers are on)");
}
