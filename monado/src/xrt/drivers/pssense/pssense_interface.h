// Copyright 2023, Collabora, Ltd.
// Copyright 2023, Jarett Millard
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to @ref drv_pssense
 * @author Jarett Millard <jarett.millard@gmail.com>
 * @ingroup drv_pssense
 */

#pragma once

#include "xrt/xrt_frame.h"

#include "tracking/t_time_sync.h"

#include "constellation/t_constellation_tracker.h"

#include <stdlib.h>


#ifdef __cplusplus
extern "C" {
#endif
/*!
 * @defgroup drv_pssense PlayStation Sense driver
 * @ingroup drv
 *
 * @brief Driver for the PlayStation Sense motion controllers.
 */

#define PSSENSE_VID 0x054C
#define PSSENSE_PID_LEFT 0x0E45
#define PSSENSE_PID_RIGHT 0x0E46

/*!
 * Create a PlayStation Sense controller device.
 *
 * @param xp                   The prober creating the device.
 * @param xpdev                The prober device.
 * @param xfctx                The frame context.
 * @param[out] out_timing_sink Optional timing event sink output pointer.
 *
 * @ingroup drv_pssense
 */
struct xrt_device *
pssense_create(struct xrt_prober *xp,
               struct xrt_prober_device *xpdev,
               struct xrt_frame_context *xfctx,
               struct t_timing_event_sink **out_timing_sink);

/*!
 * Add a PlayStation Sense controller device to the constellation tracker.
 *
 * @param xdev The PlayStation Sense controller device.
 * @param tracker The constellation tracker to add the device to.
 * @return 0 on success, negative error code on failure.
 *
 * @ingroup drv_pssense
 */
int
pssense_add_to_constellation_tracker(struct xrt_device *xdev, struct t_constellation_tracker *tracker);

/*!
 * The controller's outline as its LED positions in the grip pose's frame (the pose clients get for the grip input),
 * so the lowest point of a resting controller can be found. Returns the number of points written.
 *
 * @param xdev          The PlayStation Sense controller device.
 * @param out_points    Points, in meters, in the grip frame.
 * @param max_points    Size of @p out_points.
 * @param out_radius_m  Optional: the LEDs' radius (how far the shell reaches past each point).
 *
 * @ingroup drv_pssense
 */
size_t
pssense_get_grip_outline(struct xrt_device *xdev, struct xrt_vec3 *out_points, size_t max_points, float *out_radius_m);

/*!
 * Where the OpenXR grip pose (XRT_INPUT_PSSENSE_GRIP_POSE) sits on the controller: its pose in the controller's
 * SteamVR root, the frame Sony's render model and its components are in. A SteamVR driver reports the root:
 * root = grip * inverse(this).
 *
 * @ingroup drv_pssense
 */
void
pssense_get_grip_in_steamvr_root(struct xrt_device *xdev, struct xrt_pose *out_pose);

/*!
 * @dir drivers/pssense
 *
 * @brief @ref drv_pssense files.
 */

#ifdef __cplusplus
}
#endif
