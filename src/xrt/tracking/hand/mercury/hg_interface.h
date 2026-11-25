// Copyright 2022, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Public interface of Mercury hand tracking.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Moshi Turner <moshiturner@protonmail.com>
 * @ingroup aux_tracking
 */
#pragma once
#include "xrt/xrt_defines.h"
#include "tracking/t_tracking.h"
#include "tracking/t_hand_tracking.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Create a Mercury hand tracking pipeline.
 *
 * @ingroup aux_tracking
 */
struct t_hand_tracking_sync *
t_hand_tracking_sync_mercury_create(struct t_stereo_camera_calibration *calib,
                                    struct t_hand_tracking_create_info create_info,
                                    const char *models_folder);

/*!
 * The latest gesture measurements of @p hand (0 left, 1 right) from MediaPipe's models (MERCURY_MEDIAPIPE_GESTURES),
 * any thread. Returns false when gestures are off or their models didn't load; @p out is then cleared.
 */
bool
t_hand_tracking_sync_mercury_get_gestures(struct t_hand_tracking_sync *ht_sync, int hand, struct t_hand_gestures *out);

#ifdef __cplusplus
} // extern "C"
#endif
