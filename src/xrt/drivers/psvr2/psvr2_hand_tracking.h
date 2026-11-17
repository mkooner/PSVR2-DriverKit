// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Optical hand tracking from the PS VR2 bottom camera pair, with Monado's Mercury tracker.
 *
 * The USB thread offers each camera mode 0x10 transfer (two 1016x1016 BC4 images); a worker thread decodes them,
 * unwarps each fisheye image to a pinhole view (Mercury's hand detector rarely finds hands in the raw fisheye images,
 * offline on recordings: 0 vs 81% of the right hand's frames), brightens them (the cameras expose dark for LED
 * tracking), and feeds them to Mercury through the async hand tracking helper. Hands are reported in the head
 * device's space: head pose * camera 0 in head * Mercury's hand in camera 0.
 *
 * Cameras are given after the headset's calibration block is parsed; until then, and while Mercury loads, hands
 * are reported inactive.
 * @ingroup drv_psvr2
 */

#pragma once

#include "xrt/xrt_device.h"
#include "tracking/t_tracking.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! Create the hand tracking device; @p head is the PS VR2 HMD (head pose, tracking origin). NULL on failure.
struct xrt_device *
psvr2_hand_tracking_create(struct xrt_device *head);

/*!
 * Give the bottom camera pair's calibration (cameras 0 and 1, T_DISTORTION_PSVR2, 1016x1016) and their poses in the
 * head frame (OpenXR camera axes). Mercury is started on the worker thread at the next frame.
 */
void
psvr2_hand_tracking_set_cameras(struct xrt_device *xdev,
                                const struct t_camera_calibration cams[2],
                                const struct xrt_pose cam_in_head[2]);

//! Offer one camera mode 0x10 transfer (USB thread). Copied if the worker is free, dropped otherwise.
void
psvr2_hand_tracking_push_transfer(struct xrt_device *xdev, const uint8_t *data, size_t length, int64_t timestamp_ns);

#ifdef __cplusplus
}
#endif
