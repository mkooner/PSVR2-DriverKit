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
//! Brightening, out = 255 * (in/255)^gamma. 0.5 was best of 0.35-1 on the recordings (frames average ~25/255).
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_ht_gamma, "PSVR2_HT_GAMMA", 0.5f)
/*!
 * Auto levels per camera before the brightening: each view's tones between its LEVELS_LO and LEVELS_HI percentiles
 * (smoothed over ~0.3 s) are stretched to the full range, so a bright monitor or lamp in view no longer squeezes the
 * hands into a narrow band of grey. A dark room lit by a monitor (2026-11-23 recording, the user's second headset):
 * hands were faint grey on grey, the left camera's darker than the right's. PSVR2_HT_LEVELS=0: the fixed curve only.
 */
DEBUG_GET_ONCE_BOOL_OPTION(psvr2_ht_levels, "PSVR2_HT_LEVELS", true)
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_ht_levels_lo, "PSVR2_HT_LEVELS_LO", 0.01f)
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_ht_levels_hi, "PSVR2_HT_LEVELS_HI", 0.95f)
//! Unwarped image size (square).
DEBUG_GET_ONCE_NUM_OPTION(psvr2_ht_size, "PSVR2_HT_SIZE", 640)
//! Offline diagnosis: a folder to save every 30th pair of images Mercury gets (raw and unwarped, PGM). Off when unset.
DEBUG_GET_ONCE_OPTION(psvr2_ht_dump, "PSVR2_HT_DUMP", NULL)


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
	uint32_t stats_queries[2], stats_active[2];
	struct xrt_vec3 stats_wrist_head[2], stats_wrist_world[2]; // last seen

	// Read by get_hand_tracking.
	struct os_mutex pose_lock;
	struct xrt_pose cam0_in_head;
	volatile bool running;
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
	int n = (int)debug_get_num_option_psvr2_ht_size();
	h->out_size = n;
	h->gamma = gamma;
	for (int i = 0; i < 256; i++) {
		h->lut[i] = (uint8_t)lroundf(255.0f * powf(i / 255.0f, gamma));
	}
	memcpy(h->view_lut[0], h->lut, sizeof(h->lut));
	memcpy(h->view_lut[1], h->lut, sizeof(h->lut));
	for (int v = 0; v < 2; v++) {
		build_unwarp(h, v, &cams[v], fov, n);
		h->decoded[v] = U_TYPED_ARRAY_CALLOC(uint8_t, (size_t)CAM_W * CAM_H);
	}

	// Mercury sees two distortion-free pinhole cameras (RT8 with zero coefficients) with the physical cameras' axes,
	// so the stereo extrinsics are the physical ones: x_cam1 = R x_cam0 + T in OpenCV camera axes.
	struct t_stereo_camera_calibration *stereo = NULL;
	t_stereo_camera_calibration_alloc(&stereo, T_DISTORTION_OPENCV_RADTAN_8);
	float f = (n / 2.0f) / tanf(fov * (float)M_PI / 360.0f);
	for (int v = 0; v < 2; v++) {
		struct t_camera_calibration *cc = &stereo->view[v];
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
	setenv("MERCURY_ACQUIRE_CONFIDENCE", "0.7", 0);
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
	struct t_hand_tracking_sync *sync = t_hand_tracking_sync_mercury_create(stereo, info, models);
	t_stereo_camera_calibration_reference(&stereo, NULL);
	if (sync == NULL) {
		HT_ERROR(h, "Mercury failed to start; hand tracking off");
		return;
	}
	h->async = t_hand_tracking_async_default_create(&h->xfctx, sync);

	os_mutex_lock(&h->pose_lock);
	h->cam0_in_head = poses[0];
	os_mutex_unlock(&h->pose_lock);
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
			        "Hands (5 s): left found %u/%u, right found %u/%u queries (%llu frames processed); last "
			        "right wrist: head frame %.2f %.2f %.2f m, room %.2f %.2f %.2f m; last left wrist: head "
			        "frame %.2f %.2f %.2f m",
			        h->stats_active[0], h->stats_queries[0], h->stats_active[1], h->stats_queries[1],
			        (unsigned long long)h->processed, h->stats_wrist_head[1].x, h->stats_wrist_head[1].y,
			        h->stats_wrist_head[1].z, h->stats_wrist_world[1].x, h->stats_wrist_world[1].y,
			        h->stats_wrist_world[1].z, h->stats_wrist_head[0].x, h->stats_wrist_head[0].y,
			        h->stats_wrist_head[0].z);
			memset(h->stats_queries, 0, sizeof(h->stats_queries));
			memset(h->stats_active, 0, sizeof(h->stats_active));
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
	if (!out_value->is_active) {
		os_mutex_lock(&h->pose_lock);
		h->stats_queries[hand]++;
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
	return XRT_SUCCESS;
}

static void
psvr2_ht_destroy(struct xrt_device *xdev)
{
	struct psvr2_hand_tracking *h = psvr2_ht(xdev);

	os_thread_helper_destroy(&h->worker); // stops and joins
	h->running = false;
	xrt_frame_context_destroy_nodes(&h->xfctx); // the async helper and Mercury
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
