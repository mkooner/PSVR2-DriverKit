// Copyright 2020, Collabora, Ltd.
// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to RealSense devices.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_realsense
 */

#pragma once

#include "tracking/t_tracking.h"
#include "tracking/t_time_sync.h"


#ifdef __cplusplus
extern "C" {
#endif

struct xrt_frame_context;

struct rs_ds_context;

/*!
 * @defgroup drv_realsense Intel RealSense driver
 * @ingroup drv
 *
 * @brief Driver for the Intel Realsense devices.
 */

#define REALSENSE_MOVIDIUS_VID 0x03E7
#define REALSENSE_MOVIDIUS_PID 0x2150

#define REALSENSE_TM2_VID 0x8087
#define REALSENSE_TM2_PID 0x0B37

#define RS_TRACKING_DISABLED -1
#define RS_TRACKING_UNSPECIFIED 0
#define RS_TRACKING_DEVICE_SLAM 1
#define RS_TRACKING_HOST_SLAM 2


/*
 *
 * SLAM tracking data source/xrt_device
 *
 */

/*!
 * Create a auto prober for rs devices.
 *
 * @ingroup drv_realsense
 */
struct xrt_auto_prober *
rs_create_auto_prober(void);

/*!
 * Creates a RealSense SLAM source from the appropriate @p device_idx.
 * The streaming configuration is loaded from the global config file.
 *
 * @param xfctx Frame context this data source lifetime is tied to.
 * @param device_idx Index of the realsense device to use. Usually 0 if you only
 * have one RealSense device.
 * @return DataSource with SLAM streaming capabilities.
 */
struct xrt_ds *
rs_source_create(struct xrt_frame_context *xfctx, int device_idx);

/*!
 * Creates an xrt_device that exposes the onboard tracking of a Realsense device
 * (ie. probably a T265)
 * @return An xrt_device that you can call get_tracked_pose on with XRT_INPUT_GENERIC_TRACKER_POSE
 */
struct xrt_device *
rs_create_tracked_device_internal_slam(void);


/*
 *
 * Generic data source
 *
 */

/*!
 * Creates a data source context for enumerating and creating realsense data sources.
 */
struct rs_ds_context *
rs_ds_context_create(struct xrt_frame_context *xfctx);

/*!
 * Gets the amount of sensors present in the data source context.
 */
uint32_t
rs_ds_context_sensor_count(struct rs_ds_context *context);

/*!
 * Gets the amount of data sources present in the sensor.
 */
uint32_t
rs_ds_context_sensor_data_source_count(struct rs_ds_context *context, uint32_t sensor);

/*!
 * Gets the data source, and optionally the camera calibration, of a sensor's data source.
 *
 * @param[in]  context                 The data source context
 * @param[in]  sensor_idx              The index into the sensor array
 * @param[in]  data_source_idx         The index into the sensor's data source array
 * @param[out] out_calib               The camera calibration (optional)
 * @param[out] out_timing_event_source The camera's timing event source (optional)
 */
struct xrt_ds *
rs_ds_context_sensor_get_data_source(struct rs_ds_context *context,
                                     uint32_t sensor_idx,
                                     uint32_t data_source_idx,
                                     struct t_camera_calibration *out_calib,
                                     struct t_timing_event_source **out_timing_event_source);

/*!
 * @dir drivers/realsense
 *
 * @brief @ref drv_realsense files.
 */


#ifdef __cplusplus
}
#endif
