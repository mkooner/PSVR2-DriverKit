// Copyright 2020-2021, Collabora, Ltd.
// Copyright 2023, Jan Schmidt
// Copyright 2024, Joel Valenciano
// Copyright 2025, Beyley Cardellio
// Copyright 2026, Conner Bondurant
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PSVR2 HMD device
 *
 * @author Jan Schmidt <jan@centricular.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Joel Valenciano <joelv1907@gmail.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @author Conner Bondurant <conner.bondurant@triptych.me>
 * @ingroup drv_psvr2
 */

#include "tracking/t_time_sync.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_tracking.h"
#include "xrt/xrt_defines.h"

#include "os/os_threading.h"
#include "os/os_time.h"

#include "math/m_api.h"
#include "math/m_clock_tracking.h"
#include "math/m_mathinclude.h"
#include "math/m_relation_history.h"
#include "math/m_vec3.h"
#include "math/m_space.h"

#include "tracking/t_dead_reckoning.h"
#include "tracking/t_constellation.h"

#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_frame.h"
#include "util/u_logging.h"
#include "util/u_sink.h"
#include "util/u_time.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"
#include "util/u_debug.h"

#include "b_timing_source.h"

#include <stdio.h>
#include <ctype.h>
#include <sys/stat.h>
#include <assert.h>
#include <inttypes.h>
#include <libusb.h>

// The implementation (u_sink_converter.c) is built with the precise BC4 decoder, so declare the same signature.
#define BCDEC_BC4BC5_PRECISE
#include "bcdec.h"

#include "psvr2_imu_time.h"
#include "psvr2_basalt.h"
#include "psvr2.h"
#include "psvr2_camera_recorder.h"
#include "psvr2_interface.h"
#include "xrt/xrt_config_drivers.h"
#ifdef XRT_BUILD_DRIVER_HANDTRACKING
#include "psvr2_hand_tracking.h"
#endif
#include "psvr2_constellation_tracking.h"


/*
 *
 * Structs and defines.
 *
 */

// @todo Once clang format is updated in CI, remove this
// clang-format off
#define SLAM_POSE_CORRECTION {.orientation = {.x = 0, .y = 0, .z = sqrt(2) / 2, .w = sqrt(2) / 2}}
// clang-format on

DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_default_brightness, "PSVR2_DEFAULT_BRIGHTNESS", 1.0f)
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_refresh_hz, "PSVR2_REFRESH_HZ", 89.91f)
//! How long to wait for the headset's calibration block before using the saved copy (take_calibration_block).
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_calib_wait_s, "PSVR2_CALIB_WAIT_S", 5.0f)

DEBUG_GET_ONCE_LOG_OPTION(psvr2_log, "PSVR2_LOG", U_LOGGING_WARN)

// Head pose filter: set PSVR2_POSE_FILTER=0 for the previous SLAM + gyro dead reckoning behaviour (A/B testing).
DEBUG_GET_ONCE_BOOL_OPTION(psvr2_pose_filter, "PSVR2_POSE_FILTER", true)
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_pose_orient_tau, "PSVR2_POSE_ORIENT_TAU", 0.15f)
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_pose_pos_tau, "PSVR2_POSE_POS_TAU", 0.03f)
/*
 * The headset's SLAM origin is where the head was at startup, not the floor. Raise the tracking origin by this much
 * so poses are floor-relative: about your standing head height, or seated head height if you start seated.
 */
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_floor_height, "PSVR2_FLOOR_HEIGHT", 1.6f)
DEBUG_GET_ONCE_BOOL_OPTION(psvr2_recenter, "PSVR2_RECENTER", true)
/*!
 * Accelerometer-aided head position in the pose filter (accel_enable): velocity from the 2 kHz accelerometer between
 * SLAM poses. Off until tested on the headset; steamvr_sim SIM_ACCEL=1 replays it.
 */
DEBUG_GET_ONCE_BOOL_OPTION(psvr2_accel_fusion, "PSVR2_ACCEL_FUSION", false)
/*!
 * Gyro bias estimation time constant in seconds (pose filter gyro_bias_tau_s), 0: off. This headset's gyro reads
 * about (0.8, -0.2, 0.05) deg/s at rest (same in four recordings, 2026-11-17); steamvr_sim SIM_GYRO_BIAS_TAU replays.
 */
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_gyro_bias_tau, "PSVR2_GYRO_BIAS_TAU", 0.0f)
/*!
 * With PSVR2_ACCEL_FUSION: pause the gravity/bias average while turning faster than this (deg/s, 0: never). Replay of
 * three 2026-11-17 recordings: 250 ms prediction drift in fast turns 4.8 -> 2.7 mm, slow motion unchanged.
 */
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_gravity_pause_deg, "PSVR2_GRAVITY_PAUSE_DEG", 30.0f)
/*!
 * Basalt VIO (bottom camera pair + IMU, camera mode 0x10) as the referee of the headset's SLAM steps in the pose filter
 * (psvr2_basalt.h): a SLAM step the IMU and Basalt both reject is replaced by the IMU step. Needs libbasalt.so
 * (VIT_SYSTEM_LIBRARY_PATH) and turns accelerometer fusion on. Replay of the 2026-11-17 19:12 recording through this
 * module in real time: fast-move position jitter worst-10% 0.71 -> 0.34 mm, worst-1% 1.65 -> 0.79 mm, still unchanged.
 */
DEBUG_GET_ONCE_BOOL_OPTION(psvr2_basalt, "PSVR2_BASALT", false)
//! Carry the head position across mid-session SLAM resets (PSVR2_SLAM_CARRY=0 for the previous behaviour).
DEBUG_GET_ONCE_BOOL_OPTION(psvr2_slam_carry, "PSVR2_SLAM_CARRY", true)
/*!
 * Head position correction blend time (ms, 0: take each correction at once), see pos_blend_ns. 100 ms: replay of two
 * 2026-11-17 recordings, worst-1% position jitter still 1.63 -> 0.71 mm, turning 0.61 -> 0.40, fast moves worst-10%
 * 1.03 -> 0.71 (fusion on; fusion off similar); the output follows SLAM ~0.1-0.5 mm more loosely (was 17).
 */
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_pose_blend_ms, "PSVR2_POSE_BLEND_MS", 100.0f)
/*!
 * PSVR2_SMOOTH_CORRECTION_MS > 0 (with accelerometer fusion): SLAM corrections are brought in at a rate that ramps
 * smoothly with this time constant, and the velocity includes it (psvr2_pose_filter_params::corr_smooth_tau_s), so the
 * reported velocity matches the pose's motion. Affects SteamVR (which predicts with the
 * velocity) and the OpenXR path's position extrapolation alike; the pose itself stays as before. Default 30 ms,
 * 0 turns it off (the earlier exponential blend, velocity without the correction's motion).
 */
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_smooth_correction_ms, "PSVR2_SMOOTH_CORRECTION_MS", 30.0f)
//! Head point distance in front of the neck pivot for the pose filter (metres, 0 disables), see pivot_offset.
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_head_pivot_m, "PSVR2_HEAD_PIVOT_M", 0.10f)
/*
 * Tracking camera exposure time reported to LED sync, which centres the LED blink on exposure/2 (BG included).
 * Measured on hardware from the LED visibility edges found by the basic LED sync: 841us. Earlier, 0 centred the
 * blink on the exposure start and a 500us guess put it too early, so the LEDs left the exposure during BG. The LD
 * header has a 1000 constant (float at byte 36, u16 at byte 48) that may be the nominal value. 0 = unknown.
 */
/*!
 * Send the LED detector settings Sony's driver sends at start-up (reports 0x14 and 0x09, from a USB capture of the
 * PlayStation VR2 App on 2026-11-17). Without them the headset keeps its defaults: the detector's settings, which it
 * echoes in every LED detector frame header (bytes 24-63), read 128 / 1000 and 0x78 under our driver in every
 * recording, and 85 / 500 and 0xb4 under Sony's. 500 is likely the exposure in us (half ours) and 0xb4 a higher
 * detection threshold: short exposures are how constellation trackers keep room light out (Meta, Microsoft), and in
 * bright rooms the defaults let 270-430 background blobs per frame through, more than LED sync and tracking can cope with.
 */
DEBUG_GET_ONCE_BOOL_OPTION(psvr2_sony_ld_config, "PSVR2_SONY_LD_CONFIG", true)
//! Exposure reported to LED sync (us); 500 with Sony's LED detector settings (above), 840 measured without.
DEBUG_GET_ONCE_NUM_OPTION(psvr2_camera_exposure_us, "PSVR2_CAMERA_EXPOSURE_US", 0)
// Path to record raw IMU and SLAM samples to (CSV), for replaying through the pose simulator.
DEBUG_GET_ONCE_OPTION(psvr2_record_file, "PSVR2_RECORD_FILE", NULL)
/*!
 * Directory to save a pair of bottom camera images to every 2 s (PGM, 1000x1000), to judge whether they could
 * drive a room relocalizer (are they exposed for the room, or dark like the LED frames?). Off when unset.
 */
DEBUG_GET_ONCE_OPTION(psvr2_record_images, "PSVR2_RECORD_IMAGES", NULL)
/*!
 * Record every raw SLAM transfer (512 byte records, PV2C format of psvr2_camera_recorder.h) to this file, to find
 * the headset's tracking state among the record's unknown bytes. Off when unset.
 */
DEBUG_GET_ONCE_OPTION(psvr2_record_slam_raw, "PSVR2_RECORD_SLAM_RAW", NULL)
//! Optical hand tracking from the bottom camera pair (Mercury), see psvr2_hand_tracking.h. Off when unset.
DEBUG_GET_ONCE_BOOL_OPTION(psvr2_hand_tracking, "PSVR2_HAND_TRACKING", false)
/*!
 * Record the bottom camera pair's image transfers (camera mode 10) to this file, continuously, for offline work on the
 * images (hand tracking prototype). Format in psvr2_camera_recorder.h. Off when unset.
 */
DEBUG_GET_ONCE_OPTION(psvr2_record_camera, "PSVR2_RECORD_CAMERA", NULL)
//! At most this many transfers per second in PSVR2_RECORD_CAMERA (0: all).
DEBUG_GET_ONCE_FLOAT_OPTION(psvr2_record_camera_hz, "PSVR2_RECORD_CAMERA_HZ", 30.0f)

static void
psvr2_usb_stop(struct psvr2_hmd *hmd);

static void
psvr2_usb_destroy(struct psvr2_hmd *hmd);

static void
psvr2_hmd_destroy(struct xrt_device *xdev)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);

	psvr2_free_et_data(hmd);

	os_thread_helper_lock(&hmd->usb_thread);
	hmd->usb_complete = 1;
	os_thread_helper_unlock(&hmd->usb_thread);
	os_thread_helper_destroy(&hmd->usb_thread);
	psvr2_basalt_destroy(&hmd->basalt); // after the USB thread: nothing feeds it any more

	psvr2_usb_destroy(hmd);
	psvr2_camera_recorder_destroy(&hmd->camera_recorder);
	psvr2_camera_recorder_destroy(&hmd->slam_recorder);

	// @note We appear to be hitting a bug in libusb, so this is commented out
	//       see: https://github.com/libusb/libusb/issues/1605
	// if (hmd->dev != NULL) {
	// 	libusb_close(hmd->dev);
	// }

	if (hmd->ctx != NULL) {
		libusb_exit(hmd->ctx);
	}

	// Remove the variable tracking.
	u_var_remove_root(hmd);

	m_ff_vec3_f32_free(&hmd->ff_gyro);
	m_relation_history_destroy(&hmd->slam_relation_history);
	if (hmd->ld_clock != NULL) {
		m_clock_windowed_skew_tracker_destroy(hmd->ld_clock);
		hmd->ld_clock = NULL;
	}
	if (hmd->record_file != NULL) {
		fclose(hmd->record_file);
		hmd->record_file = NULL;
	}
	/*
	 * @todo The LED tracker lives on hmd->xfctx, which is never torn down, and its worker thread can still call
	 * mark_blob_device on the blobwatch. Until the tracker is destroyed here first, the blobwatch and origin
	 * source are deliberately left alive with it rather than freed under it.
	 */
	os_mutex_destroy(&hmd->data_lock);
	os_mutex_destroy(&hmd->calib_lock);
	free(hmd->calib_block);
	u_device_free(&hmd->base);
}

static xrt_result_t
psvr2_compute_distortion(struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *result)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);

	psvr2_compute_distortion_asymmetric(hmd->distortion_calibration, result, view, u, v);

	return XRT_SUCCESS;
}

static xrt_result_t
psvr2_hmd_update_inputs(struct xrt_device *xdev)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);

	timepoint_ns now = os_monotonic_get_ns();

	int32_t proximity = hmd->proximity_sensor;
	hmd->base.inputs[PSVR2_HMD_INPUT_HEAD_DETECT].value.boolean = proximity;
	hmd->base.inputs[PSVR2_HMD_INPUT_HEAD_DETECT].timestamp = now;

	os_mutex_lock(&hmd->data_lock);
	hmd->base.inputs[PSVR2_HMD_INPUT_FUNCTION_BUTTON].value.boolean = hmd->function_button;
	hmd->base.inputs[PSVR2_HMD_INPUT_FUNCTION_BUTTON].timestamp = now;
	os_mutex_unlock(&hmd->data_lock);

	return XRT_SUCCESS;
}

static void
hmd_get_raw_tracker_pose(struct psvr2_hmd *hmd, timepoint_ns at_timestamp_ns, struct xrt_space_relation *out_relation)
{
	struct xrt_space_relation latest_relation;
	timepoint_ns latest_relation_ts;
	if (hmd->use_pose_filter && psvr2_pose_filter_get(&hmd->pose_filter, at_timestamp_ns, out_relation)) {
		return;
	}

	if (!m_relation_history_get_latest(hmd->slam_relation_history, &latest_relation_ts, &latest_relation)) {
		*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
		return;
	}

	if (at_timestamp_ns <= latest_relation_ts) {
		m_relation_history_get(hmd->slam_relation_history, at_timestamp_ns, out_relation);
		return;
	}

	// Predict forward using dead reckoning
	if (!t_apply_dead_reckoning( //
	        hmd->ff_gyro,        //
	        NULL,                //
	        NULL,                //
	        at_timestamp_ns,     //
	        &latest_relation,    //
	        latest_relation_ts,  //
	        out_relation)) {
		// If dead reckoning fails, return the latest SLAM pose
		*out_relation = latest_relation;
	}
}

static xrt_result_t
psvr2_hmd_get_tracked_pose(struct xrt_device *xdev,
                           enum xrt_input_name name,
                           int64_t at_timestamp_ns,
                           struct xrt_space_relation *out_relation)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);

	switch (name) {
	case XRT_INPUT_GENERIC_HEAD_POSE:
	case XRT_INPUT_GENERIC_EYE_GAZE_POSE: break;
	default: PSVR2_ERROR(hmd, "unknown input name"); return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	os_mutex_lock(&hmd->data_lock);

	if (hmd->timestamp_samples < TIMESTAMP_SAMPLES) {
		os_mutex_unlock(&hmd->data_lock);
		*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
		return XRT_SUCCESS;
	}

	timepoint_ns prediction_ns_hw = at_timestamp_ns - hmd->hw2mono_vts;

	struct xrt_relation_chain chain = {0};

	// Push the eye pose before the head pose if required, so that we're returning the gaze relative to the head
	if (name == XRT_INPUT_GENERIC_EYE_GAZE_POSE) {
		m_relation_history_get(hmd->et_data.gaze_relation_history, prediction_ns_hw,
		                       m_relation_chain_reserve(&chain));
	}

	// Push the SLAM->head offset
	m_relation_chain_push_pose(&chain, &hmd->T_imu_head);

	// Push the normal head pose
	hmd_get_raw_tracker_pose(hmd, prediction_ns_hw, m_relation_chain_reserve(&chain));

	os_mutex_unlock(&hmd->data_lock);

	// Resolve the final relation
	m_relation_chain_resolve(&chain, out_relation);

	return XRT_SUCCESS;
}

static xrt_result_t
psvr2_hmd_get_view_poses(struct xrt_device *xdev,
                         const struct xrt_vec3 *default_eye_relation,
                         int64_t at_timestamp_ns,
                         enum xrt_view_type view_type,
                         uint32_t view_count,
                         struct xrt_space_relation *out_head_relation,
                         struct xrt_fov *out_fovs,
                         struct xrt_pose *out_poses)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);

	os_mutex_lock(&hmd->data_lock);
	if (hmd->ipd_updated) {
		hmd->info.lens_horizontal_separation_meters = hmd->ipd_mm / 1000.0;
		PSVR2_DEBUG(hmd, "IPD updated to %u mm", hmd->ipd_mm);
		hmd->ipd_updated = false;
	}
	os_mutex_unlock(&hmd->data_lock);

	struct xrt_vec3 eye_relation = *default_eye_relation;
	eye_relation.x = hmd->info.lens_horizontal_separation_meters;

	return u_device_get_view_poses(xdev, &eye_relation, at_timestamp_ns, view_type, view_count, out_head_relation,
	                               out_fovs, out_poses);
}

void
process_imu_record(struct psvr2_hmd *hmd, size_t index, struct imu_usb_record *in, timepoint_ns estimated_sample_time)
{
	struct imu_record imu_data;

	imu_data.vts_us = __le32_to_cpu(in->vts_us);
	for (int i = 0; i < 3; i++) {
		imu_data.accel[i] = __le16_to_cpu(in->accel[i]);
		imu_data.gyro[i] = __le16_to_cpu(in->gyro[i]);
	}
	imu_data.dp_frame_cnt = __le16_to_cpu(in->dp_frame_cnt);
	imu_data.dp_line_cnt = __le16_to_cpu(in->dp_line_cnt);
	imu_data.imu_ts_us = __le16_to_cpu(in->imu_ts_us);
	imu_data.status = __le16_to_cpu(in->status);

	PSVR2_TRACE(hmd,
	            "Record #%d: TS %u vts %u "
	            "accel { %d, %d, %d } gyro { %d, %d, %d } "
	            "dp_frame_cnt %u dp_line_cnt %u status %u",
	            (int)index, imu_data.imu_ts_us, imu_data.vts_us, imu_data.accel[0], imu_data.accel[1],
	            imu_data.accel[2], imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2], imu_data.dp_frame_cnt,
	            imu_data.dp_line_cnt, imu_data.status);

	/*
	 * The headset's own display frame counter and scan line, carried by every IMU sample: what the panel is showing.
	 * Recorded once per displayed frame ("D,host_ns,vts_us,frame,line"), so a recording shows whether the panel shows
	 * every frame once and on time (analysis/panel_frames.py), independent of SteamVR's or Monado's own timing.
	 */
	if (hmd->record_file != NULL && (!hmd->have_dp_frame_cnt || imu_data.dp_frame_cnt != hmd->last_dp_frame_cnt)) {
		fprintf(hmd->record_file, "D,%" PRId64 ",%u,%u,%u\n", (int64_t)estimated_sample_time, imu_data.vts_us,
		        imu_data.dp_frame_cnt, imu_data.dp_line_cnt);
	}
	hmd->last_dp_frame_cnt = imu_data.dp_frame_cnt;
	hmd->have_dp_frame_cnt = true;

	uint32_t last_imu_vts_us = hmd->last_imu_vts_us;
	uint32_t last_imu_ts = hmd->last_imu_ts;

	hmd->last_imu_vts_us = imu_data.vts_us; /* Last VTS timestamp */
	hmd->last_imu_ts = imu_data.imu_ts_us;

	hmd->last_gyro.x = -DEG_TO_RAD(imu_data.gyro[1] * GYRO_SCALE);
	hmd->last_gyro.y = DEG_TO_RAD(imu_data.gyro[2] * GYRO_SCALE);
	hmd->last_gyro.z = -DEG_TO_RAD(imu_data.gyro[0] * GYRO_SCALE);

	hmd->last_accel.x = -imu_data.accel[1] * ACCEL_SCALE;
	hmd->last_accel.y = imu_data.accel[2] * ACCEL_SCALE;
	hmd->last_accel.z = -imu_data.accel[0] * ACCEL_SCALE;

	// @note Overflow expected and fine, since this is an unsigned subtraction
	uint32_t imu_vts_delta_us = imu_data.vts_us - last_imu_vts_us;
	uint16_t imu_delta_us = imu_data.imu_ts_us - last_imu_ts;

	hmd->last_imu_vts_ns += (timepoint_ns)imu_vts_delta_us * U_TIME_1US_IN_NS;
	hmd->last_imu_ns += (timepoint_ns)imu_delta_us * U_TIME_1US_IN_NS;

	const timepoint_ns now_vts = hmd->last_imu_vts_ns;
	const timepoint_ns now_imu = hmd->last_imu_ns;

	m_clock_offset_a2b(IMU_FREQ, now_vts, estimated_sample_time, &hmd->hw2mono_vts);
	/*
	 * The first quarter second of the stream is a USB backlog whose sample times are estimated up to 6ms early
	 * (11:45) or 650ms late (10:58), against +-0.1ms afterwards. The camera clock tracker kept such a minimum for its
	 * whole 2s window and then crept toward the real offset, so camera frames waited 3-8s for it to settle; skip
	 * the first 0.5s. The camera stream starts ~2.2s after the IMU, so this costs nothing.
	 */
	if (hmd->imu_stream_start_ns == 0) {
		hmd->imu_stream_start_ns = estimated_sample_time;
	}
	if (hmd->ld_clock != NULL &&
	    estimated_sample_time - hmd->imu_stream_start_ns >= (timepoint_ns)U_TIME_1MS_IN_NS * 500) {
		m_clock_windowed_skew_tracker_push(hmd->ld_clock, estimated_sample_time, now_vts);
	}
	m_clock_offset_a2b(IMU_FREQ, now_imu, estimated_sample_time, &hmd->hw2mono_imu);

	if (hmd->timestamp_samples < TIMESTAMP_SAMPLES) {
		hmd->timestamp_samples++;
	}

	// Two samples share each VTS millisecond, see psvr2_imu_time.h.
	struct xrt_imu_sample sample = {
	    .timestamp_ns = psvr2_imu_time_fix(&hmd->imu_time, hmd->last_imu_vts_ns),
	    .accel_m_s2 = {hmd->last_accel.x, hmd->last_accel.y, hmd->last_accel.z},
	    .gyro_rad_secs = {hmd->last_gyro.x, hmd->last_gyro.y, hmd->last_gyro.z},
	};

	m_ff_vec3_f32_push(hmd->ff_gyro, &hmd->last_gyro, sample.timestamp_ns);
	psvr2_pose_filter_push_gyro(&hmd->pose_filter, sample.timestamp_ns, &hmd->last_gyro);
	psvr2_pose_filter_push_accel(&hmd->pose_filter, sample.timestamp_ns, &hmd->last_accel);

	if (hmd->basalt != NULL) {
		// Basalt runs on the headset's clock (VTS), like the filter: IMU here, camera transfers by their header time.
		psvr2_basalt_push_imu(hmd->basalt, sample.timestamp_ns, &hmd->last_accel, &hmd->last_gyro);
		// Referee: SLAM poses whose Basalt pose is there now (oldest first), dropped after 150 ms.
		while (hmd->ref_pend_count > 0) {
			struct xrt_pose ref;
			timepoint_ns rts = hmd->ref_pend[0].ts;
			if (psvr2_basalt_ref_pose(hmd->basalt, rts, &hmd->ref_pend[0].pose, &ref)) {
				psvr2_pose_filter_push_ref(&hmd->pose_filter, rts, &ref);
			} else if (sample.timestamp_ns - rts < U_TIME_1MS_IN_NS * 150) {
				break; // not there yet
			}
			memmove(&hmd->ref_pend[0], &hmd->ref_pend[1], sizeof(hmd->ref_pend[0]) * (hmd->ref_pend_count - 1));
			hmd->ref_pend_count--;
		}
	}

	if (hmd->record_file != NULL) {
		fprintf(hmd->record_file, "I,%" PRId64 ",%" PRId64 ",%.7f,%.7f,%.7f,%.5f,%.5f,%.5f\n",
		        (int64_t)estimated_sample_time, (int64_t)sample.timestamp_ns, hmd->last_gyro.x, hmd->last_gyro.y,
		        hmd->last_gyro.z, hmd->last_accel.x, hmd->last_accel.y, hmd->last_accel.z);
	}
}

static void
process_status_report(struct psvr2_hmd *hmd, uint8_t *buf, int bytes_read, timepoint_ns received_ns)
{
	struct status_record_hdr *hdr = (struct status_record_hdr *)buf;

	hmd->dprx_status = hdr->dprx_status;
	hmd->proximity_sensor = hdr->prox_sensor_flag;
	hmd->function_button = hdr->function_button;

	hmd->ipd_updated |= (hmd->ipd_mm != hdr->ipd_dial_mm);
	hmd->ipd_mm = hdr->ipd_dial_mm;

	size_t i = 0;
	uint8_t *cur = buf + sizeof(struct status_record_hdr);
	uint8_t *end = buf + bytes_read;
	size_t num_imu_samples = (size_t)(end - cur) / sizeof(struct imu_usb_record);
	while (cur < end && i < num_imu_samples) {
		struct imu_usb_record imu;
		memcpy(&imu, cur, sizeof(struct imu_usb_record));

		process_imu_record(hmd, i, &imu, received_ns - (num_imu_samples - 1 - i) * IMU_PERIOD_NS);

		cur += sizeof(struct imu_usb_record);
		i++;
	}
}

bool
psvr2_usb_xfer_continue(struct libusb_transfer *xfer, const char *type)
{
	struct psvr2_hmd *hmd = xfer->user_data;

	switch (xfer->status) {
	case LIBUSB_TRANSFER_OVERFLOW:
		PSVR2_ERROR(hmd, "%s xfer returned overflow!", type);
		/* Fall through */
	case LIBUSB_TRANSFER_ERROR:
	case LIBUSB_TRANSFER_TIMED_OUT:
	case LIBUSB_TRANSFER_CANCELLED:
	case LIBUSB_TRANSFER_STALL:
	case LIBUSB_TRANSFER_NO_DEVICE:
		os_thread_helper_lock(&hmd->usb_thread);
		hmd->usb_active_xfers--;
		os_thread_helper_signal_locked(&hmd->usb_thread);
		os_thread_helper_unlock(&hmd->usb_thread);
		PSVR2_TRACE(hmd, "%s xfer is aborting with status %d", type, xfer->status);
		return false;

	case LIBUSB_TRANSFER_COMPLETED: break;
	}

	return true;
}

static void LIBUSB_CALL
status_xfer_cb(struct libusb_transfer *xfer)
{
	DRV_TRACE_MARKER();

	if (!psvr2_usb_xfer_continue(xfer, "Status")) {
		return;
	}

	timepoint_ns received_ns = os_monotonic_get_ns();

	/* handle status packet */
	struct psvr2_hmd *hmd = xfer->user_data;
	os_mutex_lock(&hmd->data_lock);
	if ((size_t)xfer->actual_length >= sizeof(struct status_record_hdr)) {
		PSVR2_TRACE(hmd, "Status - %d bytes", xfer->actual_length);
		PSVR2_TRACE_HEX(hmd, xfer->buffer, xfer->actual_length);

		process_status_report(hmd, xfer->buffer, xfer->actual_length, received_ns);
	}

	libusb_submit_transfer(xfer);
	os_mutex_unlock(&hmd->data_lock);
}

/*!
 * PSVR2_RECORD_IMAGES: decode the two BC4 images of a mode 10 transfer and write them as PGM, every 2 s. Opt-in
 * diagnostics only: the write (2 x 1 MB) happens on the USB thread.
 */
static void
psvr2_record_images(struct psvr2_hmd *hmd, const uint8_t *src, size_t img_size, int64_t now)
{
	const char *dir = debug_get_option_psvr2_record_images();
	if (dir == NULL || now - hmd->last_image_record_ns < 2 * U_TIME_1S_IN_NS) {
		return;
	}
	hmd->last_image_record_ns = now;

	static uint8_t pixels[USB_CAM_MODE10_WIDTH * USB_CAM_MODE10_HEIGHT];
	const size_t row_blocks = USB_CAM_MODE10_PIXEL_STRIDE / 4;
	for (int i = 0; i < 2; i++) {
		for (uint32_t by = 0; by < USB_CAM_MODE10_HEIGHT / 4; by++) {
			for (uint32_t bx = 0; bx < USB_CAM_MODE10_WIDTH / 4; bx++) {
				const uint8_t *block = src + (by * row_blocks + bx) * BCDEC_BC4_BLOCK_SIZE;
				bcdec_bc4(block, pixels + (by * 4) * USB_CAM_MODE10_WIDTH + bx * 4, USB_CAM_MODE10_WIDTH, 0);
			}
		}
		char path[512];
		snprintf(path, sizeof(path), "%s/cam%d-%" PRId64 ".pgm", dir, i, now);
		FILE *f = fopen(path, "wb");
		if (f == NULL) {
			PSVR2_WARN(hmd, "PSVR2_RECORD_IMAGES: can't write %s", path);
			return;
		}
		fprintf(f, "P5\n%d %d\n255\n", USB_CAM_MODE10_WIDTH, USB_CAM_MODE10_HEIGHT);
		fwrite(pixels, 1, sizeof(pixels), f);
		fclose(f);
		src += img_size + (BCDEC_BC4_BLOCK_SIZE * row_blocks * (24 / 4));
	}
}

static void LIBUSB_CALL
img_xfer_cb(struct libusb_transfer *xfer)
{
	DRV_TRACE_MARKER();

	{
		/*
		 * Camera stream report every 5 s (info level): whether image transfers arrive at all, their sizes, and
		 * how the others ended. Two PSVR2_RECORD_CAMERA runs recorded nothing; failures were only traced.
		 */
		struct psvr2_hmd *h = xfer->user_data;
		int st = xfer->status >= 0 && xfer->status < 8 ? (int)xfer->status : 7;
		h->cam_stats.status[st]++;
		if (xfer->status == LIBUSB_TRANSFER_COMPLETED) {
			if (xfer->actual_length == 0) {
				h->cam_stats.empty++;
			} else {
				h->cam_stats.last_length = xfer->actual_length;
				h->cam_stats.with_data++;
			}
		}
		int64_t now = os_monotonic_get_ns();
		if (h->cam_stats.since_ns == 0) {
			h->cam_stats.since_ns = now;
		} else if (now - h->cam_stats.since_ns > 5 * U_TIME_1S_IN_NS) {
			PSVR2_INFO(h,
			           "Camera stream: %u with data (last %d bytes), %u empty; ended: completed %u, error %u, "
			           "timed out %u, cancelled %u, stall %u, no device %u, overflow %u (camera %s, mode 0x%x)",
			           h->cam_stats.with_data, h->cam_stats.last_length, h->cam_stats.empty, h->cam_stats.status[0],
			           h->cam_stats.status[1], h->cam_stats.status[2], h->cam_stats.status[3],
			           h->cam_stats.status[4], h->cam_stats.status[5], h->cam_stats.status[6],
			           h->camera_enable ? "on" : "off", (unsigned)h->camera_mode);
			memset(&h->cam_stats, 0, sizeof(h->cam_stats));
			h->cam_stats.since_ns = now;
		}
	}

	if (!psvr2_usb_xfer_continue(xfer, "Camera frame")) {
		return;
	}

	struct psvr2_hmd *hmd = xfer->user_data;
	if (xfer->actual_length > 0) {
		// Whatever mode the headset sends: with mode 10 requested it streamed 819456 byte (mode 1) transfers at
		// 120/s (19:58 run), which the mode 10 only hook never saw.
		int64_t arrival_ns = os_monotonic_get_ns();
		psvr2_camera_recorder_push(hmd->camera_recorder, arrival_ns, xfer->buffer, (size_t)xfer->actual_length);
#ifdef XRT_BUILD_DRIVER_HANDTRACKING
		// Copies the mode 0x10 transfer for the hand tracking worker if it's free (ignores other sizes).
		struct xrt_device *ht = hmd->hand_tracker;
		if (ht != NULL) {
			psvr2_hand_tracking_push_transfer(ht, xfer->buffer, (size_t)xfer->actual_length, arrival_ns);
		}
#endif
		PSVR2_TRACE(hmd, "Camera frame - %d bytes", xfer->actual_length);
		PSVR2_TRACE_HEX(hmd, xfer->buffer, MIN(256, xfer->actual_length));

		if (xfer->actual_length == USB_CAM_MODE10_XFER_SIZE) {
			// A client of the bottom camera images (the SteamVR plugin's passthrough): exposure time on the host
			// clock (VTS as below, plus the VTS -> host offset).
			os_mutex_lock(&hmd->data_lock);
			psvr2_camera_image_cb cb = hmd->camera_image_cb;
			void *user = hmd->camera_image_user;
			const time_duration_ns hw2mono = hmd->hw2mono_vts;
			const int64_t imu_us_now = hmd->last_imu_vts_ns / 1000;
			os_mutex_unlock(&hmd->data_lock);
			if (cb != NULL) {
				uint32_t vts32;
				memcpy(&vts32, xfer->buffer + 8, sizeof(vts32));
				vts32 = __le32_to_cpu(vts32);
				int64_t us = (imu_us_now & ~(int64_t)0xffffffff) | (int64_t)vts32;
				if (us - imu_us_now > ((int64_t)1 << 31)) {
					us -= (int64_t)1 << 32;
				} else if (imu_us_now - us > ((int64_t)1 << 31)) {
					us += (int64_t)1 << 32;
				}
				cb(user, xfer->buffer + 256, (size_t)xfer->actual_length - 256, us * 1000 + hw2mono);
			}
		}

		if (xfer->actual_length == USB_CAM_MODE10_XFER_SIZE && hmd->basalt != NULL) {
			// Header u32 at 8: exposure VTS in us; extended to 64 bits next to the IMU's VTS (wraps every 71 min).
			uint32_t vts_us;
			memcpy(&vts_us, xfer->buffer + 8, sizeof(vts_us));
			vts_us = __le32_to_cpu(vts_us);
			int64_t imu_us = hmd->last_imu_vts_ns / 1000;
			int64_t full_us = (imu_us & ~(int64_t)0xffffffff) | (int64_t)vts_us;
			if (full_us - imu_us > ((int64_t)1 << 31)) {
				full_us -= (int64_t)1 << 32;
			} else if (imu_us - full_us > ((int64_t)1 << 31)) {
				full_us += (int64_t)1 << 32;
			}
			psvr2_basalt_push_transfer(hmd->basalt, (timepoint_ns)full_us * 1000, xfer->buffer,
			                           (size_t)xfer->actual_length);
		}

		if (xfer->actual_length == USB_CAM_MODE10_XFER_SIZE) {
			struct xrt_frame *frames[2] = {0};

			uint8_t *src = xfer->buffer + 256;

			const size_t img_size =
			    BCDEC_BC4_COMPRESSED_SIZE(USB_CAM_MODE10_PIXEL_STRIDE, USB_CAM_MODE10_HEIGHT);

			int64_t now = os_monotonic_get_ns();
			psvr2_record_images(hmd, src, img_size, now);
			for (int i = 0; i < 2; i++) {
				bool debug_active = u_sink_debug_is_active(&hmd->debug_sinks[i]);

				if (debug_active) {
					u_frame_create_one_off(XRT_FORMAT_BC4, USB_CAM_MODE10_WIDTH,
					                       USB_CAM_MODE10_HEIGHT, &frames[i]);

					struct xrt_frame *xf = frames[i];

					// chop off the rightmost pixels, which are corrupt/control data[?]
					uint8_t *copy_src = src;
					uint8_t *copy_dst = xf->data;
					for (uint32_t y = 0; y < USB_CAM_MODE10_HEIGHT; y += 4) {
						const size_t src_stride =
						    BCDEC_BC4_BLOCK_SIZE * (USB_CAM_MODE10_PIXEL_STRIDE / 4);
						const size_t width = BCDEC_BC4_BLOCK_SIZE * (USB_CAM_MODE10_WIDTH / 4);
						memcpy(copy_dst, copy_src, width);

						copy_src += src_stride;
						copy_dst += xf->stride;
					}

					xf->timestamp = now;


					u_sink_debug_push_frame(&hmd->debug_sinks[i], xf);


					xrt_frame_reference(&xf, NULL);
				}

				// move them padding bytes outta here
				src += img_size + (BCDEC_BC4_BLOCK_SIZE * (USB_CAM_MODE10_PIXEL_STRIDE / 4) * (24 / 4));
			}
		} else if (xfer->actual_length == USB_CAM_MODE1_XFER_SIZE) {
			if (u_sink_debug_is_active(&hmd->debug_sinks[3])) {

				struct xrt_frame *xf = NULL;
				u_frame_create_one_off(XRT_FORMAT_L8, 1280, 640, &xf);

				uint8_t *src = xfer->buffer + 256;
				uint8_t *dest = xf->data;
				memcpy(dest, src, 640 * 1280);
				xf->timestamp = os_monotonic_get_ns();
				u_sink_debug_push_frame(&hmd->debug_sinks[3], xf);
				xrt_frame_reference(&xf, NULL);
			}
		} else if (xfer->actual_length == USB_CAM_MODE4_XFER_SIZE) {
			bool debug_sink_active = u_sink_debug_is_active(&hmd->debug_sinks[2]);


			if (debug_sink_active) {
				struct xrt_frame *xf = NULL;
				u_frame_create_one_off(XRT_FORMAT_L8, 512, 508 * 2, &xf);

				uint8_t *src = xfer->buffer + 256;
				uint8_t *dest = xf->data;

				memcpy(dest, src, 512 * 508 * 2);
				xf->timestamp = os_monotonic_get_ns();

				u_sink_debug_push_frame(&hmd->debug_sinks[2], xf);

				xrt_frame_reference(&xf, NULL);
			}
		} else {
			// wa
			PSVR2_ERROR(hmd, "WAA");
		}
	}

	os_mutex_lock(&hmd->data_lock);
	libusb_submit_transfer(xfer);
	os_mutex_unlock(&hmd->data_lock);
}

//! Logs SLAM rate and how much the pose filter corrects, every 2 seconds at debug level.
static void
psvr2_pose_stats_update_locked(struct psvr2_hmd *hmd, timepoint_ns vts_ns, const struct psvr2_pose_filter_update *u)
{
	if (hmd->pose_stats.window_start_ns == 0) {
		hmd->pose_stats.window_start_ns = vts_ns;
	}

	hmd->pose_stats.slam_count++;
	hmd->pose_stats.snaps += (u->orientation_snapped || u->position_reset) ? 1 : 0;
	hmd->pose_stats.orient_error_sum_rad += u->orient_error_rad;
	hmd->pose_stats.pos_residual_sum_m += u->pos_residual_m;
	if (u->orient_error_rad > hmd->pose_stats.orient_error_max_rad) {
		hmd->pose_stats.orient_error_max_rad = u->orient_error_rad;
	}

	time_duration_ns window_ns = vts_ns - hmd->pose_stats.window_start_ns;
	if (window_ns < U_TIME_1S_IN_NS * 2) {
		return;
	}

	uint32_t n = hmd->pose_stats.slam_count;
	PSVR2_DEBUG(hmd,
	            "Pose stats: SLAM %.1f Hz, gyro-vs-SLAM error mean %.3f max %.3f deg, position residual mean %.2f "
	            "mm, %u snaps/resets, filter %s",
	            (double)n / time_ns_to_s(window_ns), hmd->pose_stats.orient_error_sum_rad / n * 180.0 / M_PI,
	            hmd->pose_stats.orient_error_max_rad * 180.0 / M_PI, hmd->pose_stats.pos_residual_sum_m / n * 1000.0,
	            hmd->pose_stats.snaps, hmd->use_pose_filter ? "on" : "off");

	memset(&hmd->pose_stats, 0, sizeof(hmd->pose_stats));
	hmd->pose_stats.window_start_ns = vts_ns;
}

static void
process_slam_record(struct psvr2_hmd *hmd, uint8_t *buf, int bytes_read)
{
	struct slam_usb_record slam;
	assert(bytes_read >= (int)sizeof(struct slam_usb_record));
	memcpy(&slam, buf, sizeof(struct slam_usb_record));

	if (slam.unknown1 != 3) {
		PSVR2_TRACE(hmd, "SLAM - unknown1 field was not 3, it was %d", slam.unknown1);
	}
	// assert(usb_data->unknown1 == 3 || usb_data->unknown1 == 0);

	os_mutex_lock(&hmd->data_lock);

	const struct xrt_quat old_pose_orientation = hmd->last_slam_pose.orientation;

	uint32_t last_slam_vts_us = hmd->last_slam_vts_us;
	hmd->last_slam_vts_us = __le32_to_cpu(slam.vts_ts_us);
	timepoint_ns vts_ns = hmd->last_slam_vts_ns +=
	    (timepoint_ns)(hmd->last_slam_vts_us - last_slam_vts_us) * U_TIME_1US_IN_NS;

	//@todo: Manual axis correction should come from calibration somewhere I think
	hmd->last_slam_pose.position.x = __lef32_to_cpu(slam.pos[2]);
	hmd->last_slam_pose.position.y = __lef32_to_cpu(slam.pos[1]);
	hmd->last_slam_pose.position.z = -__lef32_to_cpu(slam.pos[0]);
	hmd->last_slam_pose.orientation.w = __lef32_to_cpu(slam.orient[0]);
	hmd->last_slam_pose.orientation.x = -__lef32_to_cpu(slam.orient[2]);
	hmd->last_slam_pose.orientation.y = -__lef32_to_cpu(slam.orient[1]);
	hmd->last_slam_pose.orientation.z = __lef32_to_cpu(slam.orient[3]);

	/*
	 * SLAM reset: the headset's SLAM re-originates its position, jumping back to (nearly) where this session's
	 * tracking started; the orientation carries on. Detected as a step of at least 5 cm landing within 1 cm of
	 * the origin (while walking the first sample after it has already moved a few mm, 18:11 run).
	 *
	 * After the first 20 s (SLAM settles with bursts of resets, carrying those over left offsets of up to 1.7 m in
	 * replay) and at least 1 s after the previous reset (reset loops are drift, carrying them builds it up), the
	 * position is carried over: an offset makes it continue from where the head was heading, so games and the
	 * controllers (placed from the head pose) don't see the jump. Simulated on 8 recordings
	 * (psvr2-test/analysis/slam_rebase_sim.py): the 18:11 run's largest jump 36 -> 6 cm, 03:02's 18.5 -> 1 cm,
	 * no change where there were no mid-session resets. Resets not carried over replace the tracking world, which the
	 * SteamVR room calibration has to know (psvr2_get_slam_reset_count).
	 */
	{
		enum psvr2_slam_carry_event event;
		float jump_m;
		hmd->last_slam_pose.position =
		    psvr2_slam_carry_push(&hmd->slam_carry, vts_ns, &hmd->last_slam_pose.position, &event, &jump_m);
		if (event != PSVR2_SLAM_CARRY_NONE) {
			static const char *what[] = {
			    "", "position carried over", "position carried over (reset loop, counted as a world change)",
			    "not carried (PSVR2_SLAM_CARRY=0), tracking world replaced",
			    "not carried (SLAM still settling), tracking world replaced"}; // (no longer happens: carried)
			PSVR2_INFO(hmd, "SLAM reset (%.3f m jump, %.1f s in): %s", jump_m,
			           time_ns_to_s(vts_ns - hmd->slam_carry.first_ts), what[event]);
		}
	}

	// Always choose nearest quaternion to last slam pose. This prevents the motion estimation from thinking the
	// device has rotated nearly 360 degrees when the SLAM tracker gives us the 2nd poled quaternion for the
	// orientation, since it does not appear to do any kind of continuity checking itself.
	if (math_quat_dot(&old_pose_orientation, &hmd->last_slam_pose.orientation) < 0.0f) {
		hmd->last_slam_pose.orientation.w = -hmd->last_slam_pose.orientation.w;
		hmd->last_slam_pose.orientation.x = -hmd->last_slam_pose.orientation.x;
		hmd->last_slam_pose.orientation.y = -hmd->last_slam_pose.orientation.y;
		hmd->last_slam_pose.orientation.z = -hmd->last_slam_pose.orientation.z;
	}

	struct xrt_pose tmp = hmd->slam_correction_pose;
	math_quat_normalize(&tmp.orientation);
	math_quat_rotate(&tmp.orientation, &hmd->last_slam_pose.orientation, &hmd->pose.orientation);
	hmd->pose.position = hmd->last_slam_pose.position;
	math_vec3_accum(&tmp.position, &hmd->pose.position);

	if (hmd->recenter_enabled) {
		if (!hmd->recenter_done) {
			// Head at the origin, facing -Z, keeping the headset's gravity alignment (yaw only).
			struct xrt_vec3 fwd = {0, 0, -1};
			math_quat_rotate_vec3(&hmd->pose.orientation, &fwd, &fwd);
			float yaw = atan2f(-fwd.x, -fwd.z);
			struct xrt_quat yaw_q;
			math_quat_from_angle_vector(yaw, &(struct xrt_vec3){0, 1, 0}, &yaw_q);
			struct xrt_pose start = {.orientation = yaw_q, .position = hmd->pose.position};
			math_pose_invert(&start, &hmd->recenter_pose);
			hmd->recenter_done = true;
			PSVR2_INFO(hmd,
			           "Re-centered SLAM space on the first pose: head was at (%.3f, %.3f, %.3f), yaw %.1f deg",
			           start.position.x, start.position.y, start.position.z, yaw * 180.0f / (float)M_PI);
		}
		math_pose_transform(&hmd->recenter_pose, &hmd->pose, &hmd->pose);
	}

	// Copied under the lock for the constellation tracker's camera placement below.
	const struct xrt_pose head_pose_for_tracker = hmd->pose;

	struct psvr2_pose_filter_update filter_update;
	psvr2_pose_filter_push_slam(&hmd->pose_filter, vts_ns, &hmd->pose, &filter_update);
	if (hmd->basalt != NULL) {
		if (hmd->ref_pend_count == ARRAY_SIZE(hmd->ref_pend)) {
			memmove(&hmd->ref_pend[0], &hmd->ref_pend[1], sizeof(hmd->ref_pend[0]) * (ARRAY_SIZE(hmd->ref_pend) - 1));
			hmd->ref_pend_count--;
		}
		hmd->ref_pend[hmd->ref_pend_count].ts = vts_ns;
		hmd->ref_pend[hmd->ref_pend_count].pose = hmd->pose;
		hmd->ref_pend_count++;
	}
	psvr2_pose_stats_update_locked(hmd, vts_ns, &filter_update);

	if (hmd->record_file != NULL) {
		fprintf(hmd->record_file, "S,%" PRId64 ",%" PRId64 ",%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f\n",
		        (int64_t)os_monotonic_get_ns(), (int64_t)vts_ns, hmd->pose.position.x, hmd->pose.position.y,
		        hmd->pose.position.z, hmd->pose.orientation.x, hmd->pose.orientation.y,
		        hmd->pose.orientation.z, hmd->pose.orientation.w);
	}
	os_mutex_unlock(&hmd->data_lock);

	PSVR2_TRACE(hmd, "SLAM - %d leftover bytes", (int)sizeof(slam.remainder));
	PSVR2_TRACE_HEX(hmd, slam.remainder, sizeof(slam.remainder));

	struct xrt_pose_sample pose_sample = {
	    .timestamp_ns = vts_ns,
	    .pose = hmd->pose,
	};

	struct xrt_space_relation relation = {
	    .pose = pose_sample.pose,
	    .relation_flags = (enum xrt_space_relation_flags)(
	        XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	        XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT),
	};

	struct xrt_space_relation cam0_relation = {
	    .pose = {.orientation = {.x = __lef32_to_cpu(slam.cam0orient[0]),
	                             .y = __lef32_to_cpu(slam.cam0orient[1]),
	                             .z = __lef32_to_cpu(slam.cam0orient[2]),
	                             .w = __lef32_to_cpu(slam.cam0orient[3])},
	             .position =
	                 {
	                     .x = __lef32_to_cpu(slam.cam0offset[0]),
	                     .y = __lef32_to_cpu(slam.cam0offset[1]),
	                     .z = __lef32_to_cpu(slam.cam0offset[2]),
	                 }},
	    .relation_flags = (enum xrt_space_relation_flags)(
	        XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	        XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT),
	};

	/*
	 * The constellation tracker's cameras are placed relative to this origin. The record's cam0 fields are always
	 * identity on hardware (recorded over a whole session), which made the tracker treat the cameras as fixed in
	 * the room: controllers swung away or flew off whenever the head moved. Use the tracked head pose (same space
	 * as the head pose we report, so controllers and head agree); the camera mount is applied per camera.
	 */
	// In the head frame (like the camera-to-head calibration and the pose we report), not the IMU/SLAM body.
	struct xrt_space_relation head_relation_for_tracker = {
	    .relation_flags = cam0_relation.relation_flags,
	};
	math_pose_transform(&head_pose_for_tracker, &hmd->T_imu_head, &head_relation_for_tracker.pose);
	psvr2_constellation_tracking_source_push(hmd->constellation_origin_source, &head_relation_for_tracker, vts_ns);

	if (hmd->record_file != NULL) {
		// Raw camera 0 fields next to the head pose, to work out which frame they are in (offline).
		os_mutex_lock(&hmd->data_lock);
		fprintf(hmd->record_file, "C,%" PRId64 ",%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f\n", (int64_t)vts_ns,
		        cam0_relation.pose.position.x, cam0_relation.pose.position.y, cam0_relation.pose.position.z,
		        cam0_relation.pose.orientation.x, cam0_relation.pose.orientation.y,
		        cam0_relation.pose.orientation.z, cam0_relation.pose.orientation.w);
		os_mutex_unlock(&hmd->data_lock);
	}

	m_relation_history_push_with_motion_estimation(hmd->slam_relation_history, &relation, pose_sample.timestamp_ns);
}

static void LIBUSB_CALL
slam_xfer_cb(struct libusb_transfer *xfer)
{
	DRV_TRACE_MARKER();

	if (!psvr2_usb_xfer_continue(xfer, "SLAM frame")) {
		return;
	}

	struct psvr2_hmd *hmd = xfer->user_data;
	if (xfer->actual_length > 0) {
		psvr2_camera_recorder_push(hmd->slam_recorder, os_monotonic_get_ns(), xfer->buffer,
		                           (size_t)xfer->actual_length);
	}
	if (xfer->actual_length == sizeof(struct slam_usb_record)) {
		process_slam_record(hmd, xfer->buffer, xfer->actual_length);
	}

	os_mutex_lock(&hmd->data_lock);
	libusb_submit_transfer(xfer);
	os_mutex_unlock(&hmd->data_lock);
}


static void LIBUSB_CALL
led_xfer_cb(struct libusb_transfer *xfer)
{
	DRV_TRACE_MARKER();

	if (!psvr2_usb_xfer_continue(xfer, "LED Detector")) {
		return;
	}

	struct psvr2_hmd *hmd = xfer->user_data;

	uint8_t *buffer = xfer->buffer;

	if (xfer->actual_length == 0) {
		goto led_xfer_end;
	}

	if (xfer->actual_length != 36944) {
		PSVR2_WARN(hmd, "Got LD data of unexpected size %d", xfer->actual_length);
		goto led_xfer_end;
	}

	if (buffer[0] != 'L' || buffer[1] != 'D') {
		PSVR2_WARN(hmd, "Got LD data of unexpected header %d %d", buffer[0], buffer[1]);
		goto led_xfer_end;
	}

	// PSVR2_INFO_HEX(hmd, xfer->buffer, xfer->actual_length);

#if 0
	FILE *out = fopen("LED_DETECTOR_DATA.bin", "wb");
	size_t written = 0;
	while (written < (size_t)xfer->actual_length) {
		written += fwrite(buffer + written, 1, xfer->actual_length - written, out);
	}
	fflush(out);
	fclose(out);
#endif

	// @todo this is invalid on some arches, pointer may not be aligned.
	struct ld_payload *payload = (struct ld_payload *)buffer;

	// Once, a second in: did the headset take the LED detector settings? It echoes them in header bytes 24-63.
	if (!hmd->ld_config_checked && debug_get_bool_option_psvr2_sony_ld_config() && hmd->ld_stats_frames >= 60) {
		hmd->ld_config_checked = true;
		const uint8_t *h = (const uint8_t *)&payload->header;
		static const uint8_t want_09[4] = {0xb4, 0x20, 0x40, 0x20};
		static const uint8_t want_14[8] = {0x00, 0x00, 0xaa, 0x42, 0x00, 0x00, 0xfa, 0x43}; // 85.0f, 500.0f
		bool ok = memcmp(h + 24, want_09, 4) == 0 && memcmp(h + 32, want_14, 8) == 0;
		PSVR2_INFO(hmd, "LED detector settings %s (header %02x %02x %02x %02x ... %02x %02x %02x %02x %02x %02x %02x %02x)",
		           ok ? "applied" : "NOT applied", h[24], h[25], h[26], h[27], h[32], h[33], h[34], h[35], h[36], h[37],
		           h[38], h[39]);
	}

	PSVR2_TRACE(hmd, "timestamp: %u, frame_counter: %u, unk3: %u", payload->header.timestamp,
	            payload->header.frame_counter, payload->header.unk3);

	/*
	 * The LD timestamp is a 32-bit microsecond counter that appears to share the VTS clock of the IMU records
	 * (same width and units; PSVR2Toolkit also maps camera timestamps through the IMU timestamp offset). Unwrap
	 * it against the last IMU VTS sample and move it into host monotonic time.
	 */
	os_mutex_lock(&hmd->data_lock);
	int32_t ld_vts_delta_us = (int32_t)(payload->header.timestamp - hmd->last_imu_vts_us);
	timepoint_ns ld_vts_ns = hmd->last_imu_vts_ns + (timepoint_ns)ld_vts_delta_us * U_TIME_1US_IN_NS;
	/*
	 * The offset follows the minimum-delay tracker, slew limited so the camera's hardware cadence passes through.
	 * It used to be latched from hw2mono_vts on the first frame (often tens of ms off before the tracker was ready)
	 * and slewed at 2us/frame, which is just the headset's real VTS -> host skew (~120ppm, 16683.4us VTS vs 16681.4us
	 * host frames), so it never caught up: camera timestamps sat 23-80ms in the future and drifted ~120us/s, which
	 * moved the controller LED window and misaligned the optical poses with the IMUs. Now LD frames wait for the
	 * tracker, a far-off offset re-latches at once, and the slew (10us/frame) covers the skew with room to spare.
	 */
	timepoint_ns ld_tracked_ns;
	bool have_tracked = hmd->ld_clock != NULL &&
	                    m_clock_windowed_skew_tracker_to_local(hmd->ld_clock, ld_vts_ns, &ld_tracked_ns);
	if (have_tracked) {
		time_duration_ns target_offset_ns = ld_tracked_ns - ld_vts_ns;
		const time_duration_ns max_slew_ns = U_TIME_1US_IN_NS * 10;
		time_duration_ns d = target_offset_ns - hmd->ld_offset_ns;
		timepoint_ns now_ns = (timepoint_ns)os_monotonic_get_ns();
		/*
		 * Re-latch above 0.3ms (was 2ms). At startup the offset was 1.5-3.5ms off its target (02:54) and slewed
		 * there at the 10us/frame limit, ~600us/s for 3-6s; LED calibration started meanwhile and chased a window
		 * moving ~550us/s (left edge 5557 -> 7312 -> 8793us in successive passes), so the first controller took
		 * 13s. Settled, the timestamps stay within +-70us of the camera's steady cadence (02:03, 02:54), well
		 * inside 0.3ms, so the slew still carries the cadence and the real skew.
		 */
		if (!hmd->ld_offset_valid || d > U_TIME_1US_IN_NS * 300 || d < -U_TIME_1US_IN_NS * 300) {
			hmd->ld_offset_ns = target_offset_ns;
			if (!hmd->ld_offset_valid) {
				hmd->ld_offset_valid_since_ns = now_ns;
			}
			hmd->ld_offset_valid = true;
			hmd->ld_offset_steady_since_ns = now_ns;
		} else {
			hmd->ld_offset_ns += CLAMP(d, -max_slew_ns, max_slew_ns);
		}
		/*
		 * Camera frames go to the tracker and LED sync only once the offset hasn't had to re-latch (more than 0.3ms
		 * off) for 0.5s (was 1s; with the startup backlog skipped it moves <0.1ms/s by then): the tracker's own estimate also moves while it fills (02:03: 31ms off at start, 11.5ms at
		 * 2s, 3ms at 6s). Requiring it within 0.1ms of the tracker instead never held (12:06: hit the 8s limit), the
		 * tracker's frame-to-frame noise is larger than that, and delayed tracking 8s. Once only, so a later hiccup
		 * can't stop tracking; at most 8s after the offset became valid.
		 */
		if (!hmd->ld_clock_settled && hmd->ld_offset_valid &&
		    (now_ns - hmd->ld_offset_steady_since_ns >= (timepoint_ns)U_TIME_1MS_IN_NS * 500 ||
		     now_ns - hmd->ld_offset_valid_since_ns >= (timepoint_ns)U_TIME_1S_IN_NS * 8)) {
			hmd->ld_clock_settled = true;
			PSVR2_INFO(hmd, "Camera clock settled %.1fs after it became valid%s",
			           (double)(now_ns - hmd->ld_offset_valid_since_ns) / 1e9,
			           now_ns - hmd->ld_offset_steady_since_ns >= (timepoint_ns)U_TIME_1MS_IN_NS * 500
			               ? ""
			               : " (time limit, still moving)");
		}
	}
	timepoint_ns ld_timestamp_ns = ld_vts_ns + hmd->ld_offset_ns;
	bool clock_ready =
	    hmd->timestamp_samples >= TIMESTAMP_SAMPLES && hmd->ld_offset_valid && hmd->ld_clock_settled;
	os_mutex_unlock(&hmd->data_lock);

	if (clock_ready) {
		psvr2_blobwatch_push_ld_payload(hmd->blobwatch, payload, payload->header.frame_counter, ld_timestamp_ns);
	}

	// Raw LED records for the first frames of a recording, to identify the brightness field offline.
	if (hmd->record_file != NULL && hmd->ld_stats_frames < 3000) {
		// The 64 byte LD header too, it has undecoded fields (exposure time is still unknown).
		const uint8_t *hraw = (const uint8_t *)&payload->header;
		fprintf(hmd->record_file, "H,%u,", payload->header.frame_counter);
		for (size_t b = 0; b < sizeof(payload->header); b++) {
			fprintf(hmd->record_file, "%02x", hraw[b]);
		}
		fprintf(hmd->record_file, "\n");

		for (int c = 0; c < 4; c++) {
			const struct ld_camera *cam = &payload->cameras[c];
			for (int j = 0; j < cam->num_leds && j < PSVR2_MAX_LEDS_PER_CAMERA; j++) {
				const uint8_t *raw = (const uint8_t *)&cam->leds[j];
				fprintf(hmd->record_file, "L,%u,%d,", payload->header.frame_counter, c);
				for (size_t b = 0; b < sizeof(struct ld_led); b++) {
					fprintf(hmd->record_file, "%02x", raw[b]);
				}
				fprintf(hmd->record_file, "\n");
			}
		}
	}

	// Periodic summary for testing: is LD data arriving, and are LEDs being seen per camera?
	if ((hmd->ld_stats_frames++ % 300) == 0) {
		PSVR2_DEBUG(hmd,
		            "LD stats: frame %u, clock %s, tracker %s, LEDs per camera [%u %u %u %u], "
		            "ts host-now %+.2fms",
		            payload->header.frame_counter, clock_ready ? "ready" : "waiting",
		            hmd->blobwatch != NULL ? "running" : "not created", payload->cameras[0].num_leds,
		            payload->cameras[1].num_leds, payload->cameras[2].num_leds, payload->cameras[3].num_leds,
		            clock_ready ? (double)(ld_timestamp_ns - (timepoint_ns)os_monotonic_get_ns()) / 1e6 : 0.0);
	}

	uint32_t ld_frames_delta =
	    hmd->have_last_led_frame ? (uint32_t)(payload->header.frame_counter - hmd->last_led_frame_counter) : 0;

	if (hmd->timing_source && clock_ready) {
		// Use the headset's own frame timestamp rather than USB arrival time, which adds transfer latency and
		// jitter. Sony's driver does the same for camera frames (header timestamp + IMU clock offset), per
		// PSVR2Toolkit's CaesarUsbThreadImage__pollHook.
		struct t_timing_event event = {
		    .type = T_TIMING_EVENT_TYPE_CAMERA_EXPOSURE_START,
		    .camera_exposure_start =
		        {
		            .sequence_id = payload->header.frame_counter,
		            .timestamp_ns = ld_timestamp_ns,
		            // Per frame, so a dropped LD payload doesn't report a doubled period. 0 lets pssense fall
		            // back to its own estimate (first frame, or no frame counter progress).
		            .frame_period_ns = ld_frames_delta > 0 ? (time_duration_ns)(payload->header.timestamp -
		                                                                        hmd->last_led_timestamp_us) *
		                                                         U_TIME_1US_IN_NS / ld_frames_delta
		                                                   : 0,
		            .exposure_time_ns =
		                (time_duration_ns)(debug_get_num_option_psvr2_camera_exposure_us() > 0
		                                       ? debug_get_num_option_psvr2_camera_exposure_us()
		                                       : (debug_get_bool_option_psvr2_sony_ld_config() ? 500 : 840)) *
		                U_TIME_1US_IN_NS,
		            // All blobs the headset detected this frame, for the PS Sense count calibration.
		            .blob_count = (uint32_t)payload->cameras[0].num_leds + payload->cameras[1].num_leds +
		                          payload->cameras[2].num_leds + payload->cameras[3].num_leds,
		            .has_blob_count = true,
		        },
		};
		hmd->last_led_timestamp_us = payload->header.timestamp;
		hmd->last_led_frame_counter = payload->header.frame_counter;
		hmd->have_last_led_frame = true;

		t_timing_event_sink_push_timing_event(hmd->timing_sink, &event);
	}

led_xfer_end:
	os_mutex_lock(&hmd->data_lock);
	libusb_submit_transfer(xfer);
	os_mutex_unlock(&hmd->data_lock);
}


struct gen_data_hdr
{
	uint16_t magic;
	uint16_t unk1;
	uint32_t len;
	uint16_t entries_unk;
	uint8_t unk2[22];
} PSVR2_PACKED;
static_assert(sizeof(struct gen_data_hdr) == 0x20, "bad size");

enum gen_data_entry_type
{
	// Position of camera 0 relative to hmd origin.
	GEN_DATA_CAMERA_CALIB_VRTRACE_0 = 0x0a,
	// seems to be debug calibration, zero'd out on my unit
	GEN_DATA_CAMERA_CALIB_VRTRACE_1 = 0x0b,
	GEN_DATA_IMU_CALIB = 0x0c,
	GEN_DATA_IMU_SET_CALIB = 0x0d,
	GEN_DATA_0F = 0x0f,
	GEN_DATA_CALIBRATION_BLOCK = 0x11,
	GEN_DATA_12 = 0x12,
	GEN_DATA_13 = 0x13,
	GEN_DATA_14 = 0x14,
};

struct gen_data_entry
{
	uint16_t type;
	uint8_t unk[2];
	uint32_t size;
	uint32_t offset;
};
static_assert(sizeof(struct gen_data_entry) == 12, "bad data");

static void
take_calibration_block(struct psvr2_hmd *hmd, uint8_t *data, uint32_t size, bool from_cache);

static void LIBUSB_CALL
vd_xfer_cb(struct libusb_transfer *xfer)
{
	DRV_TRACE_MARKER();

	if (!psvr2_usb_xfer_continue(xfer, "GenData")) {
		return;
	}

	struct psvr2_hmd *hmd = xfer->user_data;

	if (xfer->actual_length == 0) {
		goto vd_xfer_end;
	}

	PSVR2_TRACE(hmd, "Parsing VD xfer");

	uint32_t data_len = (uint32_t)xfer->actual_length;
	uint32_t to_go = data_len;
	uint8_t *buf = xfer->buffer;
	while (to_go >= 0x200) {
		struct gen_data_hdr *data_hdr = (struct gen_data_hdr *)buf;

		PSVR2_TRACE(hmd, "Parsing VD at %zu", (size_t)buf - (size_t)xfer->buffer);

		if (buf[0] != 'V' || buf[1] != 'D' || to_go < data_hdr->len || data_hdr->unk1 != 0x200u) {
			PSVR2_ERROR(hmd, "Error parsing gendata");
			break;
		}

		struct gen_data_entry *entry = (struct gen_data_entry *)(buf + 0x20);
		for (int i = 0; i < data_hdr->entries_unk; i++) {
			if (data_len < (entry->size + entry->offset)) {
				PSVR2_ERROR(hmd, "Invalid entry %d, %d", entry->offset, entry->size);
			}

			// PSVR2_DEBUG(hmd, "Entry with offset %u, size %u, type %u", entry->offset, entry->size,
			// entry->type);

			/*
			 * Record every gen-data entry raw, handled or not: some are still undecoded (camera-to-head
			 * extrinsics, which PSVR2Toolkit reads from Sony's driver, aren't in any decoded entry).
			 */
			if (hmd->record_file != NULL && data_len >= (entry->size + entry->offset)) {
				fprintf(hmd->record_file, "V,%" PRId64 ",%d,%u,", (int64_t)os_monotonic_get_ns(), entry->type,
				        (unsigned)entry->size);
				const uint8_t *p = buf + entry->offset;
				for (uint32_t b = 0; b < entry->size; b++) {
					fprintf(hmd->record_file, "%02x", p[b]);
				}
				fprintf(hmd->record_file, "\n");
			}

			switch (entry->type) {
			case GEN_DATA_CAMERA_CALIB_VRTRACE_0: {
				PSVR2_DEBUG_HEX(hmd, buf + entry->offset, entry->size);
				break;
			}
			case GEN_DATA_CAMERA_CALIB_VRTRACE_1: {
				PSVR2_DEBUG_HEX(hmd, buf + entry->offset, entry->size);
				break;
			}
			case GEN_DATA_IMU_CALIB: {
				PSVR2_DEBUG_HEX(hmd, buf + entry->offset, entry->size);
				break;
			}
			case GEN_DATA_IMU_SET_CALIB: {
				PSVR2_DEBUG_HEX(hmd, buf + entry->offset, entry->size);
				break;
			}
			case GEN_DATA_0F: {
				break;
			}
			case GEN_DATA_CALIBRATION_BLOCK: {
				take_calibration_block(hmd, buf + entry->offset, entry->size, false);
				break;
			}
			case GEN_DATA_12: {
				break;
			}
			case GEN_DATA_13: {
				break;
			}
			case GEN_DATA_14: {
				break;
			}
			default: {
				/*
				 * Undecoded data from the headset, possibly its stored play area / floor. Log the first few
				 * of each type in full and record every one, so it can be identified from a real session.
				 */
				uint32_t type_idx = entry->type % ARRAY_SIZE(hmd->vd_unknown_seen);
				if (hmd->vd_unknown_seen[type_idx]++ < 3) {
					PSVR2_WARN(hmd, "Unknown VD entry type %d, %u bytes (logged %u/3)", entry->type,
					           (unsigned)entry->size, hmd->vd_unknown_seen[type_idx]);
					PSVR2_DEBUG_HEX(hmd, buf + entry->offset, entry->size);
				}
			} break;
			}

			entry++;
		}

		to_go -= data_hdr->len;
		buf += data_hdr->len;
	}

vd_xfer_end:
	os_mutex_lock(&hmd->data_lock);
	libusb_submit_transfer(xfer);
	os_mutex_unlock(&hmd->data_lock);
}

static void LIBUSB_CALL
dump_xfer_cb(struct libusb_transfer *xfer)
{
	DRV_TRACE_MARKER();
	struct psvr2_hmd *hmd = xfer->user_data;
	const char *name = NULL;

	if (xfer == hmd->led_detector_xfer)
		name = "LED Detector";
	else if (xfer == hmd->relocalizer_xfer)
		name = "RP";
	else if (xfer == hmd->vd_xfer)
		name = "VD";
	assert(name != NULL);

	if (!psvr2_usb_xfer_continue(xfer, name)) {
		return;
	}

	PSVR2_TRACE(hmd, "%s xfer size %u", name, xfer->actual_length);
	PSVR2_TRACE_HEX(hmd, xfer->buffer, xfer->actual_length);

	os_mutex_lock(&hmd->data_lock);
	libusb_submit_transfer(xfer);
	os_mutex_unlock(&hmd->data_lock);
}

/*!
 * The saved copy of this headset's calibration block: $XDG_CONFIG_HOME (or ~/.config)/monado/psvr2-calibration-<serial>.bin.
 * The block (camera intrinsics and placement, from the factory) is the same every session.
 */
static bool
calibration_cache_path(struct psvr2_hmd *hmd, char *out, size_t len)
{
	const char *xdg = getenv("XDG_CONFIG_HOME");
	const char *home = getenv("HOME");
	char dir[512];
	if (xdg != NULL && xdg[0] != '\0') {
		snprintf(dir, sizeof dir, "%s/monado", xdg);
	} else if (home != NULL) {
		snprintf(dir, sizeof dir, "%s/.config/monado", home);
	} else {
		return false;
	}
	char serial[XRT_DEVICE_NAME_LEN];
	size_t n = 0;
	for (const char *c = hmd->base.serial; *c != '\0' && n + 1 < sizeof serial; c++) {
		serial[n++] = (isalnum((unsigned char)*c) || *c == '-' || *c == '_') ? *c : '_';
	}
	serial[n] = '\0';
	snprintf(out, len, "%s/psvr2-calibration-%s.bin", dir, serial);
	mkdir(dir, 0755); // may exist already
	return true;
}

/*!
 * The headset's calibration block (GEN_DATA_CALIBRATION_BLOCK), from the headset or (@p from_cache) the saved copy:
 * builds the LED tracker for the controllers (psvr2_parse_gen_calibration_block) and Basalt if on. A block from the
 * headset is saved, so a session where the headset doesn't send it (23:00 run 2026-11-17: never arrived, no error, no
 * LED tracker, controllers untracked all session) still gets one.
 */
static void
take_calibration_block(struct psvr2_hmd *hmd, uint8_t *data, uint32_t size, bool from_cache)
{
	os_mutex_lock(&hmd->calib_lock);
	bool first = hmd->calib_block == NULL;
	if (first) {
		hmd->calib_block = malloc(size);
		if (hmd->calib_block != NULL) {
			memcpy(hmd->calib_block, data, size);
			hmd->calib_block_size = size;
		}
	}
	psvr2_parse_gen_calibration_block(hmd, data, size);
	if (debug_get_bool_option_psvr2_basalt() && hmd->basalt == NULL && !hmd->basalt_tried) {
		// Once: it needs the camera calibration in this block. Published only when fully set up.
		hmd->basalt_tried = true;
		struct psvr2_basalt *b = psvr2_basalt_create(NULL, data, size);
		__atomic_store_n(&hmd->basalt, b, __ATOMIC_RELEASE);
		if (b == NULL) {
			PSVR2_ERROR(hmd, "PSVR2_BASALT: Basalt didn't start, running without the referee");
		}
	}
	os_mutex_unlock(&hmd->calib_lock);

	char path[1024];
	if (!from_cache && first && calibration_cache_path(hmd, path, sizeof path)) {
		// Save it when there's no copy yet or it differs (another headset with the same serial string is unlikely).
		bool same = false;
		FILE *f = fopen(path, "rb");
		if (f != NULL) {
			uint8_t *old = malloc(size + 1);
			same = old != NULL && fread(old, 1, size + 1, f) == size && memcmp(old, data, size) == 0;
			free(old);
			fclose(f);
		}
		if (!same) {
			f = fopen(path, "wb");
			if (f != NULL && fwrite(data, 1, size, f) == size) {
				PSVR2_INFO(hmd, "Saved the headset's calibration block (%u bytes) to %s", size, path);
			} else {
				PSVR2_WARN(hmd, "Couldn't save the calibration block to %s", path);
			}
			if (f != NULL) {
				fclose(f);
			}
		}
	}
}

//! No calibration block from the headset PSVR2_CALIB_WAIT_S after USB start: use the saved copy, if there is one.
static void
calibration_block_fallback(struct psvr2_hmd *hmd)
{
	char path[1024];
	if (!calibration_cache_path(hmd, path, sizeof path)) {
		return;
	}
	FILE *f = fopen(path, "rb");
	if (f == NULL) {
		PSVR2_ERROR(hmd, "The headset sent no calibration block and there's no saved copy (%s): the controllers "
		                 "won't be tracked optically this session; restarting usually brings it back",
		            path);
		return;
	}
	uint8_t buf[65536];
	size_t n = fread(buf, 1, sizeof buf, f);
	fclose(f);
	if (n < 16) {
		PSVR2_ERROR(hmd, "Saved calibration block %s is too short (%zu bytes)", path, n);
		return;
	}
	PSVR2_WARN(hmd, "The headset sent no calibration block in %.0f s: using the saved copy %s (%zu bytes)",
	           debug_get_float_option_psvr2_calib_wait_s(), path, n);
	take_calibration_block(hmd, buf, (uint32_t)n, true); // the parse checks its magic and size
}

static void *
psvr2_usb_thread(void *ptr)
{
	U_TRACE_SET_THREAD_NAME("PSVR2: USB communication");

	struct psvr2_hmd *hmd = ptr;

	timepoint_ns start_ns = os_monotonic_get_ns();
	bool calib_checked = false;
	os_thread_helper_lock(&hmd->usb_thread);
	while (os_thread_helper_is_running_locked(&hmd->usb_thread) && !hmd->usb_complete) {
		os_thread_helper_unlock(&hmd->usb_thread);

		libusb_handle_events_completed(hmd->ctx, &hmd->usb_complete);

		if (!calib_checked &&
		    os_monotonic_get_ns() - start_ns > time_s_to_ns(debug_get_float_option_psvr2_calib_wait_s())) {
			calib_checked = true;
			os_mutex_lock(&hmd->calib_lock);
			bool have = hmd->calib_block != NULL;
			os_mutex_unlock(&hmd->calib_lock);
			if (!have) {
				calibration_block_fallback(hmd);
			}
		}

		os_thread_helper_lock(&hmd->usb_thread);
	}

	os_thread_helper_unlock(&hmd->usb_thread);

	// Shut down USB communication
	psvr2_usb_stop(hmd);

	libusb_handle_events(hmd->ctx);

	return NULL;
}

struct psvr2_interface_info
{
	int interface_no;
	int altmode;
	const char *name;
};

struct psvr2_interface_info interface_list[] = {
    {.interface_no = PSVR2_STATUS_INTERFACE, .altmode = 1, .name = "status"},
    {.interface_no = PSVR2_SLAM_INTERFACE, .altmode = 0, .name = "SLAM"},
    {.interface_no = PSVR2_GAZE_INTERFACE, .altmode = 0, .name = "Gaze"},
    {.interface_no = PSVR2_CAMERA_INTERFACE, .altmode = 0, .name = "Camera"},
    {.interface_no = PSVR2_LD_INTERFACE, .altmode = 0, .name = "LED Detector"},
    {.interface_no = PSVR2_RP_INTERFACE, .altmode = 0, .name = "Relocalizer"},
    {.interface_no = PSVR2_VD_INTERFACE, .altmode = 0, .name = "VD"},
};

static bool
psvr2_usb_open(struct psvr2_hmd *hmd, struct xrt_prober_device *xpdev)
{
	int res;

	res = libusb_init(&hmd->ctx);
	if (res < 0) {
		PSVR2_ERROR(hmd, "Failed to init USB");
		return false;
	}

	hmd->dev = libusb_open_device_with_vid_pid(hmd->ctx, xpdev->vendor_id, xpdev->product_id);
	if (hmd->dev == NULL) {
		PSVR2_ERROR(hmd, "Failed to open USB device");
		return false;
	}

	for (size_t i = 0; i < sizeof(interface_list) / sizeof(interface_list[0]); i++) {
		int intf_no = interface_list[i].interface_no;
		int altmode = interface_list[i].altmode;
		const char *name = interface_list[i].name;

		res = libusb_claim_interface(hmd->dev, intf_no);
		if (res < 0) {
			PSVR2_ERROR(hmd, "Failed to claim USB %s interface", name);
			return false;
		}
		res = libusb_set_interface_alt_setting(hmd->dev, intf_no, altmode);
		if (res < 0) {
			PSVR2_ERROR(hmd, "Failed to set USB %s interface alt %d", name, altmode);
			return false;
		}
	}

	return true;
}

bool
get_psvr2_control(struct psvr2_hmd *hmd, uint16_t report_id, uint8_t subcmd, uint8_t *out_data, uint32_t buf_size)
{
	struct sie_ctrl_pkt pkt = {0};
	int ret;

	assert(buf_size <= sizeof(pkt.data));

	pkt.report_id = __cpu_to_le16(report_id);
	pkt.subcmd = __cpu_to_le16(subcmd);
	pkt.len = __cpu_to_le32(buf_size);

	ret = libusb_control_transfer(hmd->dev, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_ENDPOINT | 0x80, 0x1,
	                              report_id, 0x0, (unsigned char *)&pkt, buf_size + 8, 100);
	if (ret < 0) {
		PSVR2_ERROR(hmd, "Failed to get report id %u subcmd %u, reason %d", report_id, subcmd, ret);
		return false;
	}

	memcpy(out_data, pkt.data, buf_size);

	return true;
}

/*!
 * As send_psvr2_control, with the setup packet's wValue given: Sony's driver sends every report with wValue 0 (the
 * report ID is in the payload), ours puts the report ID there too; the LED detector settings are sent exactly as Sony
 * does.
 */
static bool
send_psvr2_control_value(
    struct psvr2_hmd *hmd, uint16_t report_id, uint8_t subcmd, uint16_t w_value, uint8_t *pkt_data, uint32_t pkt_len)
{
	struct sie_ctrl_pkt pkt;
	assert(pkt_len <= sizeof(pkt.data));
	pkt.report_id = __cpu_to_le16(report_id);
	pkt.subcmd = __cpu_to_le16(subcmd);
	pkt.len = __cpu_to_le32(pkt_len);
	memcpy(pkt.data, pkt_data, pkt_len);
	int ret = libusb_control_transfer(hmd->dev, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_ENDPOINT, 0x9, w_value,
	                                  0x0, (unsigned char *)&pkt, pkt_len + 8, 100);
	if (ret < 0) {
		PSVR2_ERROR(hmd, "Failed to send report id %u subcmd %u", report_id, subcmd);
		return false;
	}
	return true;
}

bool
send_psvr2_control(struct psvr2_hmd *hmd, uint16_t report_id, uint8_t subcmd, uint8_t *pkt_data, uint32_t pkt_len)
{
	struct sie_ctrl_pkt pkt;
	int ret;

	assert(pkt_len <= sizeof(pkt.data));

	pkt.report_id = __cpu_to_le16(report_id);
	pkt.subcmd = __cpu_to_le16(subcmd);
	pkt.len = __cpu_to_le32(pkt_len);
	memcpy(pkt.data, pkt_data, pkt_len);

	ret = libusb_control_transfer(hmd->dev, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_ENDPOINT, 0x9, report_id,
	                              0x0, (unsigned char *)&pkt, pkt_len + 8, 100);
	if (ret < 0) {
		PSVR2_ERROR(hmd, "Failed to send report id %u subcmd %u", report_id, subcmd);
		return false;
	}

	return true;
}

bool
psvr2_set_camera_mode(struct psvr2_hmd *hmd, enum psvr2_camera_mode mode)
{
	struct camera_cmd
	{
		__le32 data[2];
	} cmd;

	cmd.data[0] = __cpu_to_le32(0x1);
	cmd.data[1] = __cpu_to_le32(mode);

	PSVR2_DEBUG(hmd, "Setting camera mode to 0x%x", mode);

	return send_psvr2_control(hmd, PSVR2_REPORT_ID_SET_CAMERA_MODE, 0x1, (uint8_t *)(&cmd), sizeof(cmd));
}

static void
toggle_camera_enable(struct psvr2_hmd *hmd)
{
	hmd->camera_enable = !hmd->camera_enable;

	struct u_var_button *btn = &hmd->camera_enable_btn;
	snprintf(btn->label, sizeof(btn->label),
	         hmd->camera_enable ? "Disable camera streams" : "Enable camera streams");

	if (hmd->camera_enable) {
		psvr2_set_camera_mode(hmd, hmd->camera_mode);
	} else {
		psvr2_set_camera_mode(hmd, PSVR2_CAMERA_MODE_OFF);
	}
}

bool
set_brightness(struct psvr2_hmd *hmd, float brightness)
{
	uint8_t brightness_byte = CLAMP(brightness * 31, 0, 31);

	return send_psvr2_control(hmd, PSVR2_REPORT_ID_SET_BRIGHTNESS, 1, &brightness_byte, sizeof(brightness_byte));
}

bool
get_firmware_info(struct psvr2_hmd *hmd, struct pkt_firmware_info *out_firmware_info)
{
	if (!get_psvr2_control(hmd, 0x81, 0x1, (uint8_t *)out_firmware_info, sizeof(*out_firmware_info))) {
		PSVR2_ERROR(hmd, "Failed to get device firmware info packet.");
		return false;
	}

	return true;
}

static xrt_result_t
psvr2_get_brightness(struct xrt_device *xdev, float *brightness)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);

	os_mutex_lock(&hmd->data_lock);
	*brightness = hmd->brightness;
	os_mutex_unlock(&hmd->data_lock);

	return XRT_SUCCESS;
}

static xrt_result_t
psvr2_set_brightness(struct xrt_device *xdev, float brightness, bool relative)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);

	// Handle relative brightness adjustment
	brightness = relative ? hmd->brightness + brightness : brightness;

	if (!set_brightness(hmd, brightness)) {
		PSVR2_ERROR(hmd, "Failed to set brightness to %.2f", brightness);
		return XRT_ERROR_OUTPUT_REQUEST_FAILURE;
	}

	os_mutex_lock(&hmd->data_lock);
	hmd->brightness = brightness;
	os_mutex_unlock(&hmd->data_lock);

	return XRT_SUCCESS;
}

static xrt_result_t
psvr2_hmd_set_output(struct xrt_device *xdev, enum xrt_output_name name, const struct xrt_output_value *value)
{
	switch (name) {
	case XRT_OUTPUT_NAME_PSVR2_HAPTIC: {
		struct xrt_output_value_vibration vibration = value->vibration;
		(void)vibration;

		// @todo: Implement headset haptics.

		break;
	}
	default: return XRT_ERROR_OUTPUT_UNSUPPORTED;
	}

	return XRT_SUCCESS;
}

static xrt_result_t
psvr2_hmd_get_compositor_info(struct xrt_device *xdev,
                              const struct xrt_device_compositor_mode *mode,
                              struct xrt_device_compositor_info *out_info)
{
	// @note The scanout duration is the same for both 90hz and 120hz VR modes
	const double scanout_duration = 2040.0 / 2200.0;

	*out_info = (struct xrt_device_compositor_info){
	    .panel_refresh_type = XRT_PANEL_REFRESH_TYPE_ROLLING,
	    .scanout_direction = XRT_SCANOUT_DIRECTION_TOP_TO_BOTTOM,
	    .scanout_time_ns = mode->frame_interval_ns * scanout_duration,
	};

	return XRT_SUCCESS;
}

static void
cycle_camera_mode(struct psvr2_hmd *hmd)
{
	struct u_var_button *btn = &hmd->camera_mode_btn;

	switch (hmd->camera_mode) {
	default:
		hmd->camera_mode++;
		snprintf(btn->label, sizeof(btn->label), "Camera Mode 0x%x", hmd->camera_mode);
		break;
	case PSVR2_CAMERA_MODE_BOTTOM_SBS_BC4:
		hmd->camera_mode = PSVR2_CAMERA_MODE_BOTTOM_SBS_CROPPED;
		snprintf(btn->label, sizeof(btn->label), "Camera Mode 0x1");
		break;
	}

	if (hmd->camera_enable) {
		psvr2_set_camera_mode(hmd, hmd->camera_mode);
	} else {
		psvr2_set_camera_mode(hmd, PSVR2_CAMERA_MODE_OFF);
	}
}


static bool
psvr2_usb_start(struct psvr2_hmd *hmd)
{
	bool result = false;
	int res;

	os_thread_helper_lock(&hmd->usb_thread);

	/* Status endpoint */
	hmd->status_xfer = libusb_alloc_transfer(0);
	if (hmd->status_xfer == NULL) {
		PSVR2_ERROR(hmd, "Could not alloc USB transfer for status reports");
		goto out;
	}
	uint8_t *status_buf = malloc(USB_STATUS_XFER_SIZE);
	libusb_fill_interrupt_transfer(hmd->status_xfer, hmd->dev, LIBUSB_ENDPOINT_IN | PSVR2_STATUS_ENDPOINT,
	                               status_buf, USB_STATUS_XFER_SIZE, status_xfer_cb, hmd, 0);
	hmd->status_xfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;

	res = libusb_submit_transfer(hmd->status_xfer);
	if (res < 0) {
		PSVR2_ERROR(hmd, "Could not submit USB transfer for status reports");
		goto out;
	}
	hmd->usb_active_xfers++;

	/* Camera data */
	hmd->camera_enable = true;
	/*
	 * Mode 0xa (640x640 SBS, bottom and top pairs interleaved) by default. PSVR2_RECORD_CAMERA asks for mode 0x10
	 * instead: the 1000x1000 BC4 bottom pair that the BC4 parsing below (USB_CAM_MODE10_XFER_SIZE, "10" = 0x10)
	 * and the hand tracking prototype use. With 0xa the recordings got only 819456 byte mode 0xa transfers.
	 */
	hmd->camera_mode = hmd->camera_recorder != NULL || psvr2_hand_tracking_enabled() ||
	                           debug_get_bool_option_psvr2_basalt()
	                       ? PSVR2_CAMERA_MODE_BOTTOM_SBS_BC4
	                       : PSVR2_CAMERA_MODE_10;
	hmd->camera_mode_default = hmd->camera_mode;
	PSVR2_INFO(hmd, "Camera mode 0x%x", (unsigned)hmd->camera_mode);
	if (debug_get_bool_option_psvr2_sony_ld_config()) {
		// Byte for byte as Sony's driver sends them, in its order, before the camera mode.
		uint8_t ld_14[16] = {0x55, 0x00, 0x00, 0x00, 0xf4, 0x01, 0x00, 0x00,
		                     0x01, 0x00, 0x80, 0x24, 0x00, 0x00, 0x00, 0x00};
		uint8_t ld_09[24] = {0xfe, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0xb4, 0x20, 0x40, 0x20,
		                     0x80, 0x3c, 0xb4, 0x04, 0x0a, 0x20, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00};
		bool ok14 = send_psvr2_control_value(hmd, 0x14, 1, 0, ld_14, sizeof(ld_14));
		bool ok09 = send_psvr2_control_value(hmd, 0x09, 1, 0, ld_09, sizeof(ld_09));
		PSVR2_INFO(hmd, "Sent Sony's LED detector settings (0x14 %s, 0x09 %s)", ok14 ? "ok" : "failed",
		           ok09 ? "ok" : "failed");
	}
	psvr2_set_camera_mode(hmd, hmd->camera_mode);

	for (int i = 0; i < NUM_CAM_XFERS; i++) {
		hmd->camera_xfers[i] = libusb_alloc_transfer(0);
		if (hmd->camera_xfers[i] == NULL) {
			PSVR2_ERROR(hmd, "Could not alloc USB transfer %d for camera data", i);
			goto out;
		}

		uint8_t *recv_buf = malloc(USB_CAM_MODE10_XFER_SIZE);

		libusb_fill_bulk_transfer(hmd->camera_xfers[i], hmd->dev, LIBUSB_ENDPOINT_IN | PSVR2_CAMERA_ENDPOINT,
		                          recv_buf, USB_CAM_MODE10_XFER_SIZE, img_xfer_cb, hmd, 0);
		hmd->camera_xfers[i]->flags |= LIBUSB_TRANSFER_FREE_BUFFER;

		res = libusb_submit_transfer(hmd->camera_xfers[i]);
		if (res < 0) {
			PSVR2_ERROR(hmd, "Could not submit USB transfer %d for camera data", i);
			goto out;
		}
		hmd->usb_active_xfers++;
	}

	/* SLAM endpoint */
	hmd->slam_xfer = libusb_alloc_transfer(0);
	if (hmd->slam_xfer == NULL) {
		PSVR2_ERROR(hmd, "Could not alloc USB transfer for SLAM data");
		goto out;
	}
	uint8_t *slam_buf = malloc(USB_SLAM_XFER_SIZE);
	libusb_fill_bulk_transfer(hmd->slam_xfer, hmd->dev, LIBUSB_ENDPOINT_IN | PSVR2_SLAM_ENDPOINT, slam_buf,
	                          USB_SLAM_XFER_SIZE, slam_xfer_cb, hmd, 0);
	hmd->slam_xfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;

	res = libusb_submit_transfer(hmd->slam_xfer);
	if (res < 0) {
		PSVR2_ERROR(hmd, "Could not submit USB transfer for SLAM data");
		goto out;
	}
	hmd->usb_active_xfers++;

	/* LD endpoint */
	hmd->led_detector_xfer = libusb_alloc_transfer(0);
	if (hmd->led_detector_xfer == NULL) {
		PSVR2_ERROR(hmd, "Could not alloc USB transfer for LED Detector data");
		goto out;
	}
	uint8_t *led_detector_buf = malloc(USB_LD_XFER_SIZE);
	libusb_fill_bulk_transfer(hmd->led_detector_xfer, hmd->dev, LIBUSB_ENDPOINT_IN | PSVR2_LD_ENDPOINT,
	                          led_detector_buf, USB_LD_XFER_SIZE, led_xfer_cb, hmd, 0);
	hmd->led_detector_xfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;

	res = libusb_submit_transfer(hmd->led_detector_xfer);
	if (res < 0) {
		PSVR2_ERROR(hmd, "Could not submit USB transfer for LED Detector data");
		goto out;
	}
	hmd->usb_active_xfers++;

	/* RP endpoint */
	hmd->relocalizer_xfer = libusb_alloc_transfer(0);
	if (hmd->relocalizer_xfer == NULL) {
		PSVR2_ERROR(hmd, "Could not alloc USB transfer for RP data");
		goto out;
	}
	uint8_t *relocalizer_buf = malloc(USB_RP_XFER_SIZE);
	libusb_fill_bulk_transfer(hmd->relocalizer_xfer, hmd->dev, LIBUSB_ENDPOINT_IN | PSVR2_RP_ENDPOINT,
	                          relocalizer_buf, USB_RP_XFER_SIZE, dump_xfer_cb, hmd, 0);
	hmd->relocalizer_xfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;

	res = libusb_submit_transfer(hmd->relocalizer_xfer);
	if (res < 0) {
		PSVR2_ERROR(hmd, "Could not submit USB transfer for RP data");
		goto out;
	}
	hmd->usb_active_xfers++;

	/* VD endpoint */
	hmd->vd_xfer = libusb_alloc_transfer(0);
	if (hmd->vd_xfer == NULL) {
		PSVR2_ERROR(hmd, "Could not alloc USB transfer for VD data");
		goto out;
	}
	uint8_t *vd_buf = malloc(USB_VD_XFER_SIZE);
	libusb_fill_bulk_transfer(hmd->vd_xfer, hmd->dev, LIBUSB_ENDPOINT_IN | PSVR2_VD_ENDPOINT, vd_buf,
	                          USB_VD_XFER_SIZE, vd_xfer_cb, hmd, 0);
	hmd->vd_xfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;

	res = libusb_submit_transfer(hmd->vd_xfer);
	if (res < 0) {
		PSVR2_ERROR(hmd, "Could not submit USB transfer for VD data");
		goto out;
	}
	hmd->usb_active_xfers++;

	res = psvr2_start_gaze_tracking(hmd);
	if (res < 0) {
		PSVR2_ERROR(hmd, "Could not start gaze tracking");
		goto out;
	}

	result = true;

out:
	os_thread_helper_unlock(&hmd->usb_thread);
	return result;
}

static void
set_slam_correction(struct psvr2_hmd *hmd)
{
	os_mutex_lock(&hmd->data_lock);
	math_pose_invert(&hmd->last_slam_pose, &hmd->slam_correction_pose);
	os_mutex_unlock(&hmd->data_lock);
}

static void
reset_slam_correction(struct psvr2_hmd *hmd)
{
	os_mutex_lock(&hmd->data_lock);
	hmd->slam_correction_pose = (struct xrt_pose)SLAM_POSE_CORRECTION;
	os_mutex_unlock(&hmd->data_lock);
}

static void
update_brightness(struct psvr2_hmd *hmd)
{
	(void)set_brightness(hmd, hmd->brightness);
}

#define TRANSFERS_LIST(_, hmd)                                                                                         \
	for (int i = 0; i < NUM_CAM_XFERS; i++) {                                                                      \
		_(hmd->camera_xfers[i])                                                                                \
	}                                                                                                              \
	_(hmd->status_xfer)                                                                                            \
	_(hmd->slam_xfer)                                                                                              \
	_(hmd->led_detector_xfer)                                                                                      \
	_(hmd->relocalizer_xfer)                                                                                       \
	_(hmd->vd_xfer)                                                                                                \
	_(hmd->gaze_xfer)

static void
psvr2_usb_stop(struct psvr2_hmd *hmd)
{
#define X(xfer)                                                                                                        \
	if (xfer) {                                                                                                    \
		if (libusb_cancel_transfer(xfer) != 0) {                                                               \
			PSVR2_ERROR(hmd, "failed to cancel transfer");                                                 \
		}                                                                                                      \
	}

	os_mutex_lock(&hmd->data_lock);
	TRANSFERS_LIST(X, hmd);
	os_mutex_unlock(&hmd->data_lock);

#undef X
}

void
psvr2_usb_destroy(struct psvr2_hmd *hmd)
{
#define X(xfer)                                                                                                        \
	if (xfer) {                                                                                                    \
		libusb_free_transfer(xfer);                                                                            \
		xfer = NULL;                                                                                           \
	}

	TRANSFERS_LIST(X, hmd);

#undef X
}

struct distortion_calibration_block
{
	uint8_t version_unk;
	uint8_t unk[7];
	float distortion_params[32];
};

static void
psvr2_setup_distortion_and_fovs(struct psvr2_hmd *hmd)
{
	/* Each eye has an X offset, a Y offset, and two scale
	 * factors (the main scale factor, and another that
	 * allows for tilting the view, set to 0 for no tilt).
	 * It seems to be stored like this:
	 * struct calibration_t {
	 *	float offsetx_left;
	 *	float offsety_left;
	 *	float offsetx_right;
	 *	float offsety_right;
	 *	float scale1_left;
	 *	float scale2_left;
	 *	float scale1_right;
	 *	float scale2_right;
	 * } calibration;
	 * */
	struct distortion_calibration_block calibration_block;

	uint8_t buf[0x100];
	memset(buf, 0, sizeof buf); // a failed request reads as version 0 (the default model), not leftover memory
	bool lens_calib_ok = get_psvr2_control(hmd, 0x8f, 1, buf, sizeof buf);
	memcpy(&calibration_block, buf, sizeof calibration_block);

	memset(hmd->distortion_calibration, 0, sizeof(hmd->distortion_calibration));
	if (calibration_block.version_unk < 4) {
		/*
		 * Which lens model the headset got (Sony's driver computes its distortion from this headset calibration; with
		 * the default, ours differs from Sony's, e.g. SteamVR's "Raw ideal render target scale" 1.82 vs Sony's 1.35).
		 */
		PSVR2_WARN(hmd, "Lens calibration (0x8f): %s, version %u: using the default lens model, not this headset's",
		           lens_calib_ok ? "read" : "request failed", calibration_block.version_unk);
		hmd->distortion_calibration[0] = -0.09919293;
		hmd->distortion_calibration[2] = 0.09919293;
	} else {
		PSVR2_INFO(hmd, "Lens calibration (0x8f): version %u, params %.4f %.4f %.4f %.4f %.4f %.4f %.4f",
		           calibration_block.version_unk, calibration_block.distortion_params[0],
		           calibration_block.distortion_params[1], calibration_block.distortion_params[2],
		           calibration_block.distortion_params[3], calibration_block.distortion_params[4],
		           calibration_block.distortion_params[5], calibration_block.distortion_params[6]);
		float *p = calibration_block.distortion_params;

		hmd->distortion_calibration[0] = (((-p[0] - p[6]) * 29.9 + 14.95) / 1000.0 - 3.22) / 32.46199;
		hmd->distortion_calibration[1] = (((-p[1] * 29.9) + 14.95) / 1000.0) / 32.46199;

		hmd->distortion_calibration[2] = (((p[6] - p[2]) * 29.9 + 14.95) / 1000.0 + 3.22) / 32.46199;
		hmd->distortion_calibration[3] = (((-p[3] * 29.9) + 14.95) / 1000.0) / 32.46199;

		float left = -p[4] * M_PI / 180.0;
		hmd->distortion_calibration[4] = cosf(left);
		hmd->distortion_calibration[5] = sinf(left);

		float right = -p[5] * M_PI / 180.0;
		hmd->distortion_calibration[6] = cosf(right);
		hmd->distortion_calibration[7] = sinf(right);
	}

	struct xrt_fov *fovs = hmd->base.hmd->distortion.fov;
	fovs[0].angle_up = 53.0f * (M_PI / 180.0f);
	fovs[0].angle_down = -53.0f * (M_PI / 180.0f);
	fovs[0].angle_left = -61.5f * (M_PI / 180.0f);
	fovs[0].angle_right = 43.5f * (M_PI / 180.0f);

	fovs[1].angle_up = fovs[0].angle_up;
	fovs[1].angle_down = fovs[0].angle_down;
	fovs[1].angle_left = -fovs[0].angle_right;
	fovs[1].angle_right = -fovs[0].angle_left;
}

static xrt_result_t
psvr2_begin_feature(struct xrt_device *xdev, enum xrt_device_feature_type type)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);

	switch (type) {
	case XRT_DEVICE_FEATURE_EYE_TRACKING: hmd->eye_feature_enabled = true; break;
	case XRT_DEVICE_FEATURE_FACE_TRACKING: hmd->face_feature_enabled = true; break;
	default: return XRT_ERROR_FEATURE_NOT_SUPPORTED;
	}

	if (hmd->eye_feature_enabled || hmd->face_feature_enabled) {
		hmd->et_data.want_enabled = true;
	}

	return XRT_SUCCESS;
}

static xrt_result_t
psvr2_end_feature(struct xrt_device *xdev, enum xrt_device_feature_type type)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);

	switch (type) {
	case XRT_DEVICE_FEATURE_EYE_TRACKING: hmd->eye_feature_enabled = false; break;
	case XRT_DEVICE_FEATURE_FACE_TRACKING: hmd->face_feature_enabled = false; break;
	default: return XRT_ERROR_FEATURE_NOT_SUPPORTED;
	}

	if (!hmd->eye_feature_enabled && !hmd->face_feature_enabled) {
		hmd->et_data.want_enabled = false;
	}

	return XRT_SUCCESS;
}

void
psvr2_hmd_controllers_attached(struct xrt_device *hmd_xdev)
{
	struct psvr2_hmd *hmd = (struct psvr2_hmd *)hmd_xdev;
	// If the calibration block came before the controllers were attached, create the LED tracker from it now; if it
	// comes later, the USB thread does (both under calib_lock, the parse returns once the tracker exists).
	os_mutex_lock(&hmd->calib_lock);
	if (hmd->calib_block != NULL && hmd->led_tracker == NULL) {
		PSVR2_INFO(hmd, "Creating the LED tracker from the calibration block received before the controllers");
		psvr2_parse_gen_calibration_block(hmd, hmd->calib_block, hmd->calib_block_size);
	}
	os_mutex_unlock(&hmd->calib_lock);
}

//! Cleared by the builder when Sense controllers are connected (psvr2_hand_tracking_set_allowed).
static bool hand_tracking_allowed = true;

void
psvr2_hand_tracking_set_allowed(bool allowed)
{
	hand_tracking_allowed = allowed;
}

bool
psvr2_hand_tracking_enabled(void)
{
#ifdef XRT_BUILD_DRIVER_HANDTRACKING
	return hand_tracking_allowed && debug_get_bool_option_psvr2_hand_tracking();
#else
	return false;
#endif
}

void
psvr2_hmd_offer_hand_tracking_cameras(struct psvr2_hmd *hmd,
                                      const struct t_camera_calibration cams[2],
                                      const struct xrt_pose poses[2])
{
	os_mutex_lock(&hmd->data_lock);
	memcpy(hmd->ht_cams, cams, sizeof(hmd->ht_cams));
	memcpy(hmd->ht_cam_poses, poses, sizeof(hmd->ht_cam_poses));
	hmd->ht_cams_valid = true;
	struct xrt_device *ht = hmd->hand_tracker;
	os_mutex_unlock(&hmd->data_lock);
#ifdef XRT_BUILD_DRIVER_HANDTRACKING
	if (ht != NULL) {
		psvr2_hand_tracking_set_cameras(ht, cams, poses);
	}
#else
	(void)ht;
#endif
}

void
psvr2_hmd_attach_hand_tracking(struct xrt_device *hmd_xdev, struct xrt_device *hand_tracking)
{
	struct psvr2_hmd *hmd = (struct psvr2_hmd *)hmd_xdev;
	struct t_camera_calibration cams[2];
	struct xrt_pose poses[2];
	os_mutex_lock(&hmd->data_lock);
	hmd->hand_tracker = hand_tracking;
	bool have = hmd->ht_cams_valid;
	if (have) {
		memcpy(cams, hmd->ht_cams, sizeof(cams));
		memcpy(poses, hmd->ht_cam_poses, sizeof(poses));
	}
	os_mutex_unlock(&hmd->data_lock);
#ifdef XRT_BUILD_DRIVER_HANDTRACKING
	if (have) {
		psvr2_hand_tracking_set_cameras(hand_tracking, cams, poses);
	}
#else
	(void)have;
#endif
}

struct t_timing_event_source *
psvr2_hmd_get_timing_event_source(struct psvr2_hmd *hmd)
{
	if (hmd->timing_source != NULL) {
		return hmd->timing_source;
	}

	return NULL;
}


struct xrt_device *
psvr2_hmd_create(struct xrt_prober_device *xpdev)
{
	DRV_TRACE_MARKER();

	// This indicates you won't be using Monado's built-in tracking algorithms.
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	struct psvr2_hmd *hmd = U_DEVICE_ALLOCATE(struct psvr2_hmd, flags, PSVR2_HMD_INPUT_COUNT, 1);

	snprintf(hmd->base.tracking_origin->name, XRT_TRACKING_NAME_LEN, "PS VR2 Tracking");
	hmd->base.tracking_origin->type = XRT_TRACKING_TYPE_EXTERNAL_SLAM;
	hmd->base.tracking_origin->initial_offset = (struct xrt_pose)XRT_POSE_IDENTITY;
	hmd->base.tracking_origin->initial_offset.position.y = debug_get_float_option_psvr2_floor_height();
	hmd->recenter_enabled = debug_get_bool_option_psvr2_recenter();
	if (debug_get_option_psvr2_record_camera() != NULL) {
		hmd->camera_recorder = psvr2_camera_recorder_create(
		    debug_get_option_psvr2_record_camera(), debug_get_float_option_psvr2_record_camera_hz(),
		    USB_CAM_MODE10_XFER_SIZE);
	}
	if (debug_get_option_psvr2_record_slam_raw() != NULL) {
		// Every record (no rate limit), up to the transfer size.
		hmd->slam_recorder =
		    psvr2_camera_recorder_create(debug_get_option_psvr2_record_slam_raw(), 0.0f, USB_SLAM_XFER_SIZE);
	}
	psvr2_slam_carry_init(&hmd->slam_carry, debug_get_bool_option_psvr2_slam_carry());

	if (os_mutex_init(&hmd->data_lock) != 0) {
		PSVR2_ERROR(hmd, "Failed to init data mutex!");
		goto cleanup;
	}
	if (os_mutex_init(&hmd->calib_lock) != 0) {
		PSVR2_ERROR(hmd, "Failed to init calibration mutex!");
		goto cleanup;
	}

	if (os_thread_helper_init(&hmd->usb_thread) != 0) {
		PSVR2_ERROR(hmd, "Failed to initialise threading");
		goto cleanup;
	}

	m_relation_history_create(&hmd->slam_relation_history);

	if (!psvr2_usb_open(hmd, xpdev)) {
		goto cleanup;
	}

	psvr2_constellation_tracking_source_create(hmd, &hmd->constellation_origin_source);

	// This list should be ordered, most preferred first.
	size_t idx = 0;
	hmd->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;
	hmd->base.hmd->blend_mode_count = idx;

	u_device_populate_function_pointers(&hmd->base, psvr2_hmd_get_tracked_pose, psvr2_hmd_destroy);

	hmd->base.update_inputs = psvr2_hmd_update_inputs;
	hmd->base.get_view_poses = psvr2_hmd_get_view_poses;
	hmd->base.get_brightness = psvr2_get_brightness;
	hmd->base.set_brightness = psvr2_set_brightness;
	hmd->base.set_output = psvr2_hmd_set_output;
	hmd->base.get_compositor_info = psvr2_hmd_get_compositor_info;
	hmd->base.begin_feature = psvr2_begin_feature;
	hmd->base.end_feature = psvr2_end_feature;
	hmd->base.get_face_tracking = psvr2_get_face_tracking;

	hmd->pose = (struct xrt_pose)XRT_POSE_IDENTITY;
	hmd->log_level = debug_get_log_option_psvr2_log();
	hmd->T_imu_head = (struct xrt_pose){
	    .position = {.x = 0.000247f, .y = -0.000273f, .z = 0.104826f},
	    .orientation = XRT_QUAT_IDENTITY,
	};

	m_ff_vec3_f32_alloc(&hmd->ff_gyro, 1024);

	struct psvr2_pose_filter_params pose_filter_params;
	psvr2_pose_filter_default_params(&pose_filter_params);
	pose_filter_params.orient_tau_s = debug_get_float_option_psvr2_pose_orient_tau();
	pose_filter_params.pos_tau_s = debug_get_float_option_psvr2_pose_pos_tau();
	pose_filter_params.pivot_offset.z = -debug_get_float_option_psvr2_head_pivot_m();
	pose_filter_params.pos_blend_ns = (time_duration_ns)(debug_get_float_option_psvr2_pose_blend_ms() * U_TIME_1MS_IN_NS);
	pose_filter_params.corr_smooth_tau_s = debug_get_float_option_psvr2_smooth_correction_ms() / 1000.0f;
	pose_filter_params.accel_enable = debug_get_bool_option_psvr2_accel_fusion() || debug_get_bool_option_psvr2_basalt();
	if (debug_get_bool_option_psvr2_basalt()) {
		// The referee needs the accelerometer-aided position. 3 mm: 13 catches in the 19:12 replay, all real.
		pose_filter_params.ref_jump_m = 0.003f;
	}
	pose_filter_params.gyro_bias_tau_s = debug_get_float_option_psvr2_gyro_bias_tau();
	pose_filter_params.accel_gravity_pause_rad_s = DEG_TO_RAD(debug_get_float_option_psvr2_gravity_pause_deg());
	PSVR2_INFO(hmd, "Gyro bias estimation: %s (PSVR2_GYRO_BIAS_TAU %.1f s)",
	           pose_filter_params.gyro_bias_tau_s > 0.0f ? "on" : "off", pose_filter_params.gyro_bias_tau_s);
	PSVR2_INFO(hmd, "Head position: %s (PSVR2_ACCEL_FUSION)",
	         pose_filter_params.accel_enable ? "accelerometer-aided" : "SLAM only");
	psvr2_pose_filter_init(&hmd->pose_filter, &pose_filter_params);
	// ~2s of IMU samples (2kHz)
	hmd->ld_clock = m_clock_windowed_skew_tracker_alloc(4096);
	hmd->use_pose_filter = debug_get_bool_option_psvr2_pose_filter();

	const char *record_path = debug_get_option_psvr2_record_file();
	if (record_path != NULL && record_path[0] != '\0') {
		hmd->record_file = fopen(record_path, "w");
		if (hmd->record_file != NULL) {
			fprintf(hmd->record_file,
			        "# I,host_est_ns,vts_ns,gx,gy,gz,ax,ay,az | S,host_rx_ns,vts_ns,px,py,pz,qx,qy,qz,qw\n");
		}
	}

	// Print name.
	snprintf(hmd->base.str, XRT_DEVICE_NAME_LEN, "PS VR2 HMD");
	snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "PS VR2 HMD S/N");

	// Setup input.
	hmd->base.name = XRT_DEVICE_PSVR2;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;
	hmd->base.inputs[PSVR2_HMD_INPUT_HEAD_POSE].name = XRT_INPUT_GENERIC_HEAD_POSE;
	hmd->base.inputs[PSVR2_HMD_INPUT_HEAD_DETECT].name = XRT_INPUT_GENERIC_HEAD_DETECT;
	hmd->base.inputs[PSVR2_HMD_INPUT_FUNCTION_BUTTON].name = XRT_INPUT_PSVR2_SYSTEM_CLICK;
	hmd->base.inputs[PSVR2_HMD_INPUT_EYE_GAZE_POSE].name = XRT_INPUT_GENERIC_EYE_GAZE_POSE;
	hmd->base.inputs[PSVR2_HMD_INPUT_FB_FACE_TRACKING2_VISUAL].name = XRT_INPUT_FB_FACE_TRACKING2_VISUAL;
	hmd->base.inputs[PSVR2_HMD_INPUT_HTC_EYE_FACE_TRACKING].name = XRT_INPUT_HTC_EYE_FACE_TRACKING;
	hmd->base.inputs[PSVR2_HMD_INPUT_ANDROID_FACE_TRACKING].name = XRT_INPUT_ANDROID_FACE_TRACKING;

	hmd->base.outputs[0].name = XRT_OUTPUT_NAME_PSVR2_HAPTIC;

	hmd->base.supported.orientation_tracking = true;
	hmd->base.supported.position_tracking = true;
	hmd->base.supported.presence = true;
	hmd->base.supported.brightness_control = true;
	hmd->base.supported.compositor_info = true;
	hmd->base.supported.eye_gaze = true;
	hmd->base.supported.face_tracking = true;

	// Set up display details
	// refresh rate
	/*
	 * SteamVR's compositor runs the PS VR2 panel in its 4000x2040@89.91Hz mode (it matches a desired 90Hz; the
	 * 119.88Hz mode is also offered), but this said 120Hz, which SteamVR takes as Prop_DisplayFrequency for its frame
	 * timing and pose prediction: every frame's head pose was predicted for the wrong moment (8.3 vs 11.1ms frames),
	 * judder while turning. PSVR2_REFRESH_HZ sets it (89.91 default, or 119.88), and both compositors pick the display
	 * mode to match: SteamVR from Prop_DisplayFrequency, Monado direct in choose_best_vk_mode_auto (comp_window_direct.c).
	 */
	hmd->base.hmd->screens[0].nominal_frame_interval_ns =
	    time_s_to_ns(1.0f / debug_get_float_option_psvr2_refresh_hz());
	hmd->base.compute_distortion = psvr2_compute_distortion;

	struct xrt_hmd_parts *parts = hmd->base.hmd;
	parts->distortion.models = XRT_DISTORTION_MODEL_COMPUTE;
	parts->distortion.preferred = XRT_DISTORTION_MODEL_COMPUTE;

	// This default matches the default lens separation
	hmd->ipd_mm = 65;

	hmd->info.display.w_pixels = 4000;
	hmd->info.display.h_pixels = 2040;
	hmd->info.display.w_meters = 0.13f;
	hmd->info.display.h_meters = 0.07f;
	hmd->info.lens_horizontal_separation_meters = 0.13f / 2.0f;
	hmd->info.lens_vertical_position_meters = 0.07f / 2.0f;
	// These need to be set to avoid an error, but the fovs
	// computed further down are preferred.
	hmd->info.fov[0] = (float)(106.0 * (M_PI / 180.0));
	hmd->info.fov[1] = (float)(106.0 * (M_PI / 180.0));

	if (!u_device_setup_split_side_by_side(&hmd->base, &hmd->info)) {
		PSVR2_ERROR(hmd, "Failed to setup basic device info");
		goto cleanup;
	}

	psvr2_setup_distortion_and_fovs(hmd);

	u_distortion_mesh_fill_in_compute(&hmd->base);

	const struct xrt_pose slam_correction_pose = SLAM_POSE_CORRECTION;
	hmd->slam_correction_pose = slam_correction_pose;

	for (size_t i = 0; i < ARRAY_SIZE(hmd->debug_sinks); i++) {
		u_sink_debug_init(&hmd->debug_sinks[i]);
	}

	b_timing_source_create(&hmd->xfctx, &hmd->timing_sink, &hmd->timing_source);

	u_var_add_root(hmd, "PS VR2 HMD", true);
	u_var_add_pose(hmd, &hmd->pose, "pose");
	u_var_add_pose(hmd, &hmd->slam_correction_pose, "SLAM correction pose");
	{
		hmd->slam_correction_set_btn.cb = (void (*)(void *))set_slam_correction;
		hmd->slam_correction_set_btn.ptr = hmd;
		u_var_add_button(hmd, &hmd->slam_correction_set_btn, "Set");
	}
	{
		hmd->slam_correction_reset_btn.cb = (void (*)(void *))reset_slam_correction;
		hmd->slam_correction_reset_btn.ptr = hmd;
		u_var_add_button(hmd, &hmd->slam_correction_reset_btn, "Reset");
	}

	/* DROPDOWN */
	u_var_add_gui_header(hmd, NULL, "Last IMU data");
	{
		u_var_add_ro_u32(hmd, &hmd->last_imu_vts_us, "VTS Timestamp");
		u_var_add_ro_i64_ns(hmd, &hmd->last_imu_vts_ns, "VTS Timestamp (ns)");
		u_var_add_ro_i64_ns(hmd, &hmd->hw2mono_vts, "hw2mono_vts");
		u_var_add_u16(hmd, &hmd->last_imu_ts, "Timestamp");
		u_var_add_ro_vec3_f32(hmd, &hmd->last_accel, "accel");
		u_var_add_ro_vec3_f32(hmd, &hmd->last_gyro, "gyro");
	}

	u_var_add_gui_header(hmd, NULL, "Last SLAM data");
	{
		u_var_add_ro_u32(hmd, &hmd->last_slam_vts_us, "VTS Timestamp");
		u_var_add_ro_i64_ns(hmd, &hmd->last_slam_vts_ns, "VTS Timestamp (ns)");
		u_var_add_pose(hmd, &hmd->last_slam_pose, "Pose");
	}

	u_var_add_gui_header(hmd, NULL, "Status");
	{
		u_var_add_u8(hmd, &hmd->dprx_status, "HMD Display Port RX status");
		u_var_add_ro_i32(hmd, (int32_t *)&hmd->proximity_sensor, "HMD Proximity");
		u_var_add_bool(hmd, &hmd->function_button, "HMD Function button");
		u_var_add_u8(hmd, &hmd->ipd_mm, "HMD IPD (mm)");

		u_var_add_f32(hmd, &hmd->brightness, "Brightness");
		hmd->brightness_btn.cb = (void (*)(void *))update_brightness;
		hmd->brightness_btn.ptr = hmd;
		u_var_add_button(hmd, &hmd->brightness_btn, "Set Brightness");
	}

	u_var_add_gui_header(hmd, NULL, "Camera data");
	{
		hmd->camera_enable_btn.cb = (void (*)(void *))toggle_camera_enable;
		hmd->camera_enable_btn.ptr = hmd;
		u_var_add_button(hmd, &hmd->camera_enable_btn, "Disable camera streams");

		hmd->camera_mode_btn.cb = (void (*)(void *))cycle_camera_mode;
		hmd->camera_mode_btn.ptr = hmd;
		u_var_add_button(hmd, &hmd->camera_mode_btn, "Camera Mode 0x10");
	}
	for (int i = 0; i < 3; i++) {
		char name[32];
		sprintf(name, "Substream %d", i);
		u_var_add_sink_debug(hmd, &hmd->debug_sinks[i], name);
	}
	u_var_add_sink_debug(hmd, &hmd->debug_sinks[3], "Mode 1/10 stream");

	/* DROPDOWN */
	u_var_add_gui_header(hmd, NULL, "Logging");
	{
		u_var_add_log_level(hmd, &hmd->log_level, "log_level");
	}

	float initial_brightness = debug_get_float_option_psvr2_default_brightness();
	if (!set_brightness(hmd, initial_brightness)) {
		PSVR2_WARN(hmd, "Failed to set initial brightness");
	}
	hmd->brightness = initial_brightness;

	struct pkt_firmware_info firmware_info;
	if (get_firmware_info(hmd, &firmware_info)) {
		snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "%.*s", (int)sizeof(firmware_info.pcb_id),
		         firmware_info.pcb_id);

		PSVR2_INFO(hmd, "Headset has firmware version of %08x", __le32_to_cpu(firmware_info.version));
	} else {
		PSVR2_WARN(hmd, "Failed to get device firmware info, serial number will not be correct.");
	}

	// Start USB communications
	hmd->usb_complete = 0;
	if (os_thread_helper_start(&hmd->usb_thread, psvr2_usb_thread, hmd) != 0) {
		PSVR2_ERROR(hmd, "Failed to start USB thread");
		goto cleanup;
	}

	if (!psvr2_usb_start(hmd)) {
		PSVR2_ERROR(hmd, "Failed to submit USB transfers");
		goto cleanup;
	}

	return &hmd->base;

cleanup:
	psvr2_hmd_destroy(&hmd->base);
	return NULL;
}

void
psvr2_set_camera_image_callback(struct xrt_device *xdev, psvr2_camera_image_cb cb, void *user)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);
	os_mutex_lock(&hmd->data_lock);
	hmd->camera_image_cb = cb;
	hmd->camera_image_user = user;
	os_mutex_unlock(&hmd->data_lock);
}

void
psvr2_request_bottom_camera_images(struct xrt_device *xdev, bool on)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);
	os_mutex_lock(&hmd->data_lock);
	const uint32_t before = hmd->camera_image_requests;
	if (on) {
		hmd->camera_image_requests++;
	} else if (hmd->camera_image_requests > 0) {
		hmd->camera_image_requests--;
	}
	const uint32_t after = hmd->camera_image_requests;
	os_mutex_unlock(&hmd->data_lock);
	if ((before == 0) == (after == 0)) {
		return;
	}
	const enum psvr2_camera_mode mode = after > 0 ? PSVR2_CAMERA_MODE_BOTTOM_SBS_BC4 : hmd->camera_mode_default;
	if (mode == hmd->camera_mode) {
		return;
	}
	hmd->camera_mode = mode;
	PSVR2_INFO(hmd, "Camera mode 0x%x (%s)", (unsigned)mode,
	           after > 0 ? "the bottom camera pair, asked for by a client" : "back to the default");
	if (hmd->camera_enable) {
		psvr2_set_camera_mode(hmd, mode);
	}
}

bool
psvr2_get_bottom_cameras(struct xrt_device *xdev, struct t_camera_calibration *out_cams, struct xrt_pose *out_poses)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);
	os_mutex_lock(&hmd->data_lock);
	const bool valid = hmd->ht_cams_valid;
	if (valid) {
		memcpy(out_cams, hmd->ht_cams, sizeof(hmd->ht_cams));
		memcpy(out_poses, hmd->ht_cam_poses, sizeof(hmd->ht_cam_poses));
	}
	os_mutex_unlock(&hmd->data_lock);
	return valid;
}

uint32_t
psvr2_get_slam_reset_count(struct xrt_device *xdev)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);
	os_mutex_lock(&hmd->data_lock);
	uint32_t count = hmd->slam_carry.world_changes;
	os_mutex_unlock(&hmd->data_lock);
	return count;
}
