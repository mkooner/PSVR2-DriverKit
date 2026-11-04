// Copyright 2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  EuRoC dataset recorder utility.
 * @author Mateo de Mayo <mateo.demayo@collabora.com>
 * @ingroup aux_tracking
 */

#pragma once

#include "xrt/xrt_tracking.h"
#include "xrt/xrt_frame.h"

#include "tracking/t_constellation.h"
#include "tracking/t_tracking.h"


#define CSV_EOL "\r\n"
#define CSV_PRECISION 10

#define EUROC_SENSOR_TYPE_IMU "imu"
#define EUROC_SENSOR_TYPE_CAMERA "camera"
#define EUROC_SENSOR_TYPE_LED_MODEL "led_model_mnd"

#define EUROC_DISTORTION_MODEL_RADTAN_4 "radial-tangential"
#define EUROC_DISTORTION_MODEL_RADTAN_5_MND "radial-tangential-5_mnd"
#define EUROC_DISTORTION_MODEL_RADTAN_8_MND "radial-tangential-8_mnd"
#define EUROC_DISTORTION_MODEL_RADTAN_14_MND "radial-tangential-14_mnd"
#define EUROC_DISTORTION_MODEL_KB4_MND "kannalla-brandt-4_mnd"
#define EUROC_DISTORTION_MODEL_WMR_MND "wmr_mnd"
#define EUROC_DISTORTION_MODEL_RIFT_CV1_MND "cv1_mnd"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * The settings used for EuRoC dataset recording.
 */
struct euroc_recorder_settings
{
	//! Directory to save the dataset or NULL for a default based on the current datetime.
	const char *record_path;
	//! The number of cameras being recorded.
	uint32_t cam_count;
	//! The number of IMUs being recorded.
	uint32_t imu_count;
	//! The amount of ground truth streams being pushed.
	uint32_t ground_truth_stream_count;
	//! Whether to start recording immediately on creation.
	bool record_from_start;

	//! Whether the @ref camera_model field is valid.
	bool camera_model_valid;
	struct
	{
		//! The intrinsics of each individual camera.
		struct t_camera_calibration intrinsics[XRT_TRACKING_MAX_CAMS];
		//! The poses of each individual camera. May be identity if unknown. In world space.
		struct xrt_pose poses[XRT_TRACKING_MAX_CAMS];
	} camera_model;

	//! Whether each IMU is an individual constellation device or not.
	bool constellation_recording;
	struct
	{
		//! The LED models of each device, must have @ref imu_count LED models populated.
		struct t_constellation_tracker_led_model *led_models[XRT_TRACKING_MAX_IMUS];
		//! The IMU calibration of each device, must have @ref imu_count calibrations populated.
		struct xrt_imu_calibration *imu_calibrations[XRT_TRACKING_MAX_IMUS];
	} constellation;
};

/*!
 * Converts a Monado distortion model to a EuRoC distortion model string.
 */
const char *
t_distortion_model_to_euroc(enum t_camera_distortion_model model);

/*!
 * Converts a EuRoC distortion model string to a Monado distortion model.
 */
enum t_camera_distortion_model
t_euroc_model_to_distortion_model(const char *model);

/*!
 * Create SLAM sinks to record samples in EuRoC format.
 *
 * @param xfctx Frame context for the sinks.
 * @param settings The settings for this EuRoC recorder.
 * @return struct xrt_tracking_sinks* Sinks to push samples to for recording.
 *
 * @ingroup aux_tracking
 */
struct xrt_tracking_sinks *
euroc_recorder_create(struct xrt_frame_context *xfctx, const struct euroc_recorder_settings *settings);

/*!
 * Start recording samples sent to the recorder sinks.
 *
 * @param er_sinks The recorder sinks returned by @ref euroc_recorder_create
 */
void
euroc_recorder_start(struct xrt_tracking_sinks *er_sinks);

/*!
 * Stop recording samples sent to the recorder sinks. You can start and
 * stop as many times as you like.
 *
 * @param er_sinks The recorder sinks returned by @ref euroc_recorder_create
 */
void
euroc_recorder_stop(struct xrt_tracking_sinks *er_sinks);

/*!
 * Add EuRoC recorder UI button to start recording after creation.
 *
 * @param er_sinks The sinks returned by @ref euroc_recorder_create
 * @param root The pointer to add UI button to
 * @param prefix Prefix in case you have multiple recorders, otherwise pass an empty string
 */
void
euroc_recorder_add_ui(struct xrt_tracking_sinks *er_sinks, void *root, const char *prefix);

#ifdef __cplusplus
}
#endif
