// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Background map: learns bright spots that stay fixed in the room and marks the blobs on them.
 *
 * Lamps, windows and reflections stay put in the room while controller LEDs move with the hands. Each blob's ray is
 * turned into a direction in the room with its camera's orientation, and a direction that keeps showing a blob while
 * it is in view becomes background; blobs on it are dropped before LED matching and the LED sync blob counts. This is
 * the "stationary landmark" approach Meta describes for Oculus Insight (Optimizing Oculus Insight Controller Tracking
 * to Work in Challenging Conditions, 2019), without its 3D triangulation: directions with a tolerance for parallax.
 *
 * Controller LEDs are never learned: blobs the tracker matched to a device, and blobs near them in the same frame,
 * neither add to the map nor are marked, and a background direction a matched blob falls on is forgotten.
 *
 * @ingroup aux_tracking
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "tracking/t_constellation.h"

#ifdef __cplusplus
extern "C" {
#endif

struct t_blob_background;

struct t_blob_background *
t_blob_background_create(void);

void
t_blob_background_destroy(struct t_blob_background **bg_ptr);

/*!
 * Feed one camera frame and mark its background blobs.
 *
 * @param camera_index  Which camera (for its field of view estimate).
 * @param timestamp_ns  Frame time.
 * @param rot_world_cam Camera orientation in the room, OpenXR camera axes (-Z forward, +Y up).
 * @param blobs         The frame's blobs, with center_homogenized filled (OpenCV axes, z = 1) and
 *                      matched_device_id set for blobs the tracker matched (carried from the previous frame).
 * @param num_blobs     Number of blobs.
 * @param out_background One flag per blob, true for background.
 * @return Number of blobs marked background.
 */
uint32_t
t_blob_background_process(struct t_blob_background *bg,
                          uint32_t camera_index,
                          int64_t timestamp_ns,
                          const struct xrt_quat *rot_world_cam,
                          const struct t_blob *blobs,
                          uint32_t num_blobs,
                          bool *out_background);

//! Landmarks currently held, and how many of them are background.
void
t_blob_background_stats(struct t_blob_background *bg, uint32_t *out_landmarks, uint32_t *out_background);

#ifdef __cplusplus
}
#endif
