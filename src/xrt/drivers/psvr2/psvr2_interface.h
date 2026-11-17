// Copyright 2020-2021, Collabora, Ltd.
// Copyright 2023, Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PSVR2 HMD device
 *
 * @author Jan Schmidt <jan@centricular.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup drv_psvr2
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_psvr2 PS VR2 HMD driver
 * @ingroup drv
 *
 * @brief Driver for the Playstation VR2 headset
 *
 */

#define PSVR2_VID 0x054C
#define PSVR2_PID 0x0CDE

struct psvr2_hmd;

/*!
 * Create the PS VR2 HMD device
 *
 * @ingroup drv_psvr2
 */
struct xrt_device *
psvr2_hmd_create(struct xrt_prober_device *xpdev);

/*!
 * How many times this session the headset's SLAM re-originated its position without the driver carrying the position
 * over (during startup or in a reset loop), i.e. the tracking world's origin was replaced. @p xdev must be a PS VR2
 * headset (XRT_DEVICE_PSVR2).
 *
 * @ingroup drv_psvr2
 */
uint32_t
psvr2_get_slam_reset_count(struct xrt_device *xdev);


struct t_timing_event_source *
psvr2_hmd_get_timing_event_source(struct psvr2_hmd *hmd);

/*!
 * Tell the headset its Sense controllers are attached (pssense_left/right set): creates the LED tracker from an
 * already received calibration block.
 *
 * @ingroup drv_psvr2
 */
void
psvr2_hmd_controllers_attached(struct xrt_device *hmd_xdev);

/*!
 * Whether optical hand tracking was asked for (PSVR2_HAND_TRACKING): the headset then streams the bottom camera
 * pair (camera mode 0x10) and the builder creates the hand tracking device.
 *
 * @ingroup drv_psvr2
 */
bool
psvr2_hand_tracking_enabled(void);

/*!
 * Allow or forbid hand tracking, before the headset is created. The builder forbids it when Sense controllers are
 * connected: they take the hand roles, and the cameras then stay in their usual mode.
 *
 * @ingroup drv_psvr2
 */
void
psvr2_hand_tracking_set_allowed(bool allowed);

/*!
 * Give the hand tracking device (psvr2_hand_tracking_create) to the headset, which feeds it camera images and, once
 * the calibration block is parsed (before or after this call), the bottom pair's calibration.
 *
 * @ingroup drv_psvr2
 */
void
psvr2_hmd_attach_hand_tracking(struct xrt_device *hmd_xdev, struct xrt_device *hand_tracking);

/*!
 * Probing function for PlayStation VR2 devices.
 *
 * @ingroup drv_psvr2
 * @see xrt_prober_found_func_t
 */
int
psvr2_found(struct xrt_prober *xp,
            struct xrt_prober_device **devices,
            size_t device_count,
            size_t index,
            cJSON *attached_data,
            struct xrt_device **out_xdevs);

/*!
 * @dir drivers/psvr2
 *
 * @brief @ref drv_psvr2 files.
 */


#ifdef __cplusplus
}
#endif
