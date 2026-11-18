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
#pragma once

#include "psvr2_imu_time.h"
#include "tracking/t_constellation.h"
#include "tracking/t_time_sync.h"
#include "xrt/xrt_data_source.h"
#ifdef __cplusplus
extern "C" {
#endif

#include <xrt/xrt_defines.h>
#include <xrt/xrt_byte_order.h>
#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_tracking.h"

#include "os/os_threading.h"
#include "os/os_time.h"

#include "math/m_api.h"
#include "math/m_clock_tracking.h"
#include "math/m_mathinclude.h"
#include "math/m_relation_history.h"
#include "math/m_filter_one_euro.h"
#include "math/m_filter_fifo.h"

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

#include "psvr2_protocol.h"
#include "psvr2_blobwatch.h"
#include "psvr2_pose_filter.h"

#include <tracking/t_tracking.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <libusb.h>


#define NUM_CAM_XFERS 1

#define PSVR2_TRACE(p, ...) U_LOG_XDEV_IFL_T(&p->base, p->log_level, __VA_ARGS__)
#define PSVR2_TRACE_HEX(p, data, data_size) U_LOG_XDEV_IFL_T_HEX(&p->base, p->log_level, data, data_size)
#define PSVR2_DEBUG(p, ...) U_LOG_XDEV_IFL_D(&p->base, p->log_level, __VA_ARGS__)
#define PSVR2_DEBUG_HEX(p, data, data_size) U_LOG_XDEV_IFL_D_HEX(&p->base, p->log_level, data, data_size)
#define PSVR2_ERROR(p, ...) U_LOG_XDEV_IFL_E(&p->base, p->log_level, __VA_ARGS__)
#define PSVR2_WARN(p, ...) U_LOG_XDEV_IFL_W(&p->base, p->log_level, __VA_ARGS__)
#define PSVR2_INFO(p, ...) U_LOG_XDEV_IFL_I(&p->base, p->log_level, __VA_ARGS__)

#define TIMESTAMP_SAMPLES 100

struct imu_record
{
	uint32_t vts_us;
	int16_t accel[3];
	int16_t gyro[3];
	uint16_t dp_frame_cnt;
	uint16_t dp_line_cnt;
	uint16_t imu_ts_us;
	uint16_t status;
};

struct psvr2_et_eye_data
{
	bool gaze_point_valid;
	// gaze point in meters
	struct xrt_vec3 gaze_point;

	bool gaze_direction_valid;
	// gaze direction (normalized vector)
	struct xrt_vec3 gaze_direction;

	struct m_filter_euro_vec3 gaze_direction_filter;
	struct xrt_vec3 filtered_gaze_direction;

	// whether the pupil diameter is valid
	bool pupil_diameter_valid;
	// pupil diameter in meters
	float pupil_diameter;

	bool unk_float_2_valid;
	struct xrt_vec2 unk_float_2;

	bool unk_float_4_valid;
	struct xrt_vec2 unk_float_4;

	// whether the blink state is valid
	bool blink_valid;
	// whether the user is blinking
	bool blink;

	float blink_interp;
};

struct psvr2_et_combined_data
{
	bool gaze_point_valid;
	// gaze point in meters
	struct xrt_vec3 gaze_point;

	bool gaze_direction_valid;
	// gaze direction (normalized vector)
	struct xrt_vec3 gaze_direction;

	struct m_filter_euro_vec3 gaze_direction_filter;
	struct xrt_vec3 filtered_gaze_direction;

	bool is_valid;

	bool unk_float_8_valid;
	float unk_float_8;

	bool unk_float3_pair_valid;
	struct xrt_vec3 unk_float_12;
	struct xrt_vec3 unk_float_15;
	struct xrt_vec3 unk_float_18;
};

struct psvr2_et_data
{
	struct os_thread_helper eye_tracking_thread;

	//! Whether eye tracking is currently enabled
	bool want_enabled;
	bool force_enable;

	//! Whether the eye tracking enable command has been sent
	bool enabled;

	struct m_relation_history *gaze_relation_history;

	struct os_mutex data_mutex;
	bool data_mutex_created;

	struct psvr2_et_eye_data eyes[2];
	struct psvr2_et_combined_data combined;

	bool processed_sample_packet;

	uint32_t last_remote_report_sample_time_us;
	timepoint_ns last_remote_report_sample_time_ns;

	bool unk_float_4_valid;
	float unk_float_4;

	bool unk_float_5_valid;
	float unk_float_5;
};

/*!
 * PSVR2 HMD device
 *
 * @implements xrt_device
 */
struct psvr2_hmd
{
	struct xrt_device base;

	struct xrt_pose pose;

	enum u_logging_level log_level;

	struct os_mutex data_lock;

	/* Device status */
	uint8_t dprx_status;               //< DisplayPort receiver status
	xrt_atomic_s32_t proximity_sensor; //< Atomic state for whether the proximity sensor is triggered
	bool function_button;              //< Boolean state for whether the function button is pressed

	bool ipd_updated; //< Whether the IPD has been updated, and an HMD info refresh is needed
	uint8_t ipd_mm;   //< IPD dial value in mm, from 59 to 72mm

	bool camera_enable;                 //< Whether the camera is enabled
	enum psvr2_camera_mode camera_mode; //< The current camera mode
	struct u_var_button camera_enable_btn;
	struct u_var_button camera_mode_btn;

	struct u_var_button brightness_btn;
	float brightness;

	/* IMU input data */
	uint32_t last_imu_vts_us;   //< Last VTS timestamp, in microseconds
	uint16_t last_imu_ts;       //< Last IMU timestamp, in microseconds
	struct xrt_vec3 last_gyro;  //< Last gyro reading, in rad/s
	struct xrt_vec3 last_accel; //< Last accel reading, in m/s²

	/* SLAM input data */
	uint32_t last_slam_vts_us;      //< Last slam timestamp, in microseconds
	struct xrt_pose last_slam_pose; //< Last SLAM pose reading

	//! Carries the position across mid-session SLAM resets; its world_changes are psvr2_get_slam_reset_count.
	struct psvr2_slam_carry slam_carry;

	//! Optical hand tracking device (psvr2_hand_tracking.h), or NULL. Set once by the builder, under data_lock.
	struct xrt_device *hand_tracker;
	//! Bottom pair (cameras 0, 1) calibration and poses in the head frame, for hand tracking; under data_lock.
	bool ht_cams_valid;
	struct t_camera_calibration ht_cams[2];
	struct xrt_pose ht_cam_poses[2];

	//! The calibration block, kept so the builder can create the LED tracker once it attached the controllers.
	struct os_mutex calib_lock;
	uint8_t *calib_block;
	uint32_t calib_block_size;

	//! Camera image transfer counts for the periodic "Camera stream" report (USB thread only).
	struct
	{
		int64_t since_ns;
		uint32_t status[8]; // by libusb_transfer_status
		uint32_t with_data, empty;
		int last_length;
	} cam_stats;

	//! PSVR2_RECORD_CAMERA: continuous recording of the camera transfers, or NULL.
	struct psvr2_camera_recorder *camera_recorder;
	//! PSVR2_RECORD_SLAM_RAW: every raw SLAM transfer, or NULL.
	struct psvr2_camera_recorder *slam_recorder;

	//! PSVR2_RECORD_IMAGES: when images were last saved.
	int64_t last_image_record_ns;

	struct xrt_pose slam_correction_pose;

	/*!
	 * Re-centers the headset's SLAM space on the first pose of the session (PSVR2_RECENTER). The headset can
	 * relocalize into a map saved in an earlier session, with an arbitrary origin, which put the head far from
	 * the tracking origin the floor offset and play area assume.
	 */
	bool recenter_enabled;
	bool recenter_done;
	struct xrt_pose recenter_pose;
	struct u_var_button slam_correction_set_btn;
	struct u_var_button slam_correction_reset_btn;

	struct xrt_pose T_imu_head; //< Constant transform from SLAM tracker pose to head pose

	/* Display parameters */
	struct u_device_simple_info info;

	struct u_sink_debug led_debug_sinks[4];
	struct ld_camera *led_sink_cameras[2];

	/* Camera debug sinks */
	struct u_sink_debug debug_sinks[4];

	struct u_sink_debug blob_visualizer_sinks[4];

	/* USB communication */
	libusb_context *ctx;
	libusb_device_handle *dev;

	struct os_thread_helper usb_thread;
	int usb_complete;
	int usb_active_xfers;

	/* Status report */
	struct libusb_transfer *status_xfer;
	/* SLAM (bulk) transfer */
	struct libusb_transfer *slam_xfer;
	/* Camera (bulk) transfers */
	struct libusb_transfer *camera_xfers[NUM_CAM_XFERS];
	/* LD EP9 (bulk) transfer */
	struct libusb_transfer *led_detector_xfer;
	/* RP EP10 (bulk) transfer */
	struct libusb_transfer *relocalizer_xfer;
	/* VD EP11 (bulk) transfer */
	struct libusb_transfer *vd_xfer;
	/* Gaze transfer */
	struct libusb_transfer *gaze_xfer;

	/* Distortion calibration parameters, to be used with
	 * psvr2_compute_distortion_asymmetric. Very specific to
	 * PS VR2. */
	float distortion_calibration[8];

	/* Timing data */
	int timestamp_samples;

	timepoint_ns last_imu_vts_ns;
	timepoint_ns last_slam_vts_ns;
	timepoint_ns last_led_timestamp_us;
	uint32_t last_led_frame_counter;
	bool have_last_led_frame;
	uint64_t ld_stats_frames;
	timepoint_ns system_zero_ns;
	timepoint_ns last_imu_ns;

	time_duration_ns hw2mono_vts;
	time_duration_ns hw2mono_imu;
	/*!
	 * VTS -> host clock for the LD (camera) timestamps, protected by data_lock. hw2mono_vts is a fast exponential
	 * filter over USB arrival times, which left +-170us of jitter on camera exposure times (the headset's own LD
	 * timestamps are steady to 0.5us); LED sync moved the controller LEDs by that jitter. This minimum-delay tracker
	 * ignores the USB latency spread.
	 */
	struct m_clock_windowed_skew_tracker *ld_clock;
	/*!
	 * VTS -> host offset applied to LD timestamps, slew limited toward @ref ld_clock so the camera's hardware frame
	 * cadence (steady to 0.5us) reaches LED sync unchanged while the offset still follows real clock skew. The
	 * tracker alone took ~15s to settle (1.5ms rms early), and LED sync ran on those timestamps.
	 */
	bool ld_offset_valid;
	time_duration_ns ld_offset_ns;
	//! When the offset first became valid, and since when it has stayed close to the tracker (host monotonic).
	timepoint_ns ld_offset_valid_since_ns, ld_offset_steady_since_ns;
	//! The offset has settled once after startup; camera frames reach the tracker and LED sync only after that.
	bool ld_clock_settled;
	//! The LED detector settings echo in the frame header was checked (see PSVR2_SONY_LD_CONFIG).
	bool ld_config_checked;
	//! Host time of the first IMU sample, to keep the stream's startup backlog out of the camera clock.
	timepoint_ns imu_stream_start_ns;
	//! IMU sample times (two samples per VTS millisecond), see psvr2_imu_time.h.
	struct psvr2_imu_time imu_time;

	//! PSVR2_BASALT: Basalt VIO as the pose filter's referee (psvr2_basalt.h), and SLAM poses waiting for it.
	struct psvr2_basalt *basalt;
	bool basalt_tried;
	struct
	{
		timepoint_ns ts;
		struct xrt_pose pose;
	} ref_pend[16];
	uint32_t ref_pend_count;

	/* Tracking state */
	struct m_relation_history *slam_relation_history;
	struct m_ff_vec3_f32 *ff_gyro;

	//! Gyro + SLAM pose filter for the head pose (PSVR2_POSE_FILTER, on by default).
	bool use_pose_filter;
	struct psvr2_pose_filter pose_filter;

	//! Head pose filter stats, logged every couple of seconds.
	struct
	{
		timepoint_ns window_start_ns;
		uint32_t slam_count;
		uint32_t snaps;
		double orient_error_sum_rad;
		float orient_error_max_rad;
		double pos_residual_sum_m;
	} pose_stats;

	//! Raw IMU/SLAM recording for offline replay (PSVR2_RECORD_FILE), NULL when off.
	FILE *record_file;

	//! How many of each undecoded VD entry type have been logged.
	uint32_t vd_unknown_seen[32];

	/* Eye State */
	bool eye_feature_enabled;
	bool face_feature_enabled;

	struct psvr2_et_data et_data;

	/* LED Tracking Debug */
	struct xrt_frame_context xfctx;
	struct t_blob_sink *blob_sinks[4];
	struct psvr2_blobwatch *blobwatch;

	struct t_constellation_tracker *led_tracker;
	struct psvr2_constellation_tracking_source *constellation_origin_source;

	/* Timing source for sync to pssense */
	struct t_timing_event_sink *timing_sink;
	struct t_timing_event_source *timing_source;

	// Cheating super hard to get the tracking online
	struct xrt_device *pssense_left;
	struct xrt_device *pssense_right;
};

/// Casting helper function
static inline struct psvr2_hmd *
psvr2_hmd(struct xrt_device *xdev)
{
	return (struct psvr2_hmd *)xdev;
}

enum psvr2_hmd_input_name
{
	PSVR2_HMD_INPUT_HEAD_POSE,
	PSVR2_HMD_INPUT_HEAD_DETECT,
	PSVR2_HMD_INPUT_FUNCTION_BUTTON,
	PSVR2_HMD_INPUT_EYE_GAZE_POSE,
	PSVR2_HMD_INPUT_FB_FACE_TRACKING2_VISUAL,
	PSVR2_HMD_INPUT_HTC_EYE_FACE_TRACKING,
	PSVR2_HMD_INPUT_ANDROID_FACE_TRACKING,
	PSVR2_HMD_INPUT_COUNT,
};

void
psvr2_compute_distortion_asymmetric(
    float *calibration, struct xrt_uv_triplet *distCoords, int eEye, float fU, float fV);

bool
psvr2_usb_xfer_continue(struct libusb_transfer *xfer, const char *type);

bool
send_psvr2_control(struct psvr2_hmd *hmd, uint16_t report_id, uint8_t subcmd, uint8_t *pkt_data, uint32_t pkt_len);

void
psvr2_free_et_data(struct psvr2_hmd *hmd);

int
psvr2_start_gaze_tracking(struct psvr2_hmd *hmd);

xrt_result_t
psvr2_get_face_tracking(struct xrt_device *xdev,
                        enum xrt_input_name facial_expression_type,
                        int64_t at_timestamp_ns,
                        struct xrt_facial_expression_set *out_value);

bool
psvr2_set_camera_mode(struct psvr2_hmd *hmd, enum psvr2_camera_mode mode);


#ifdef __cplusplus
}
#endif
