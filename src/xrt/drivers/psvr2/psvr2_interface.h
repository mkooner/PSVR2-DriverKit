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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

/*!
 * A camera image pair from the headset's bottom cameras (camera mode 0x10, the pair Sony's driver shows as its
 * passthrough): @p bc4_pair is two BC4 images of 1024 (stride) x 1016 rows back to back (the transfer after its 256
 * byte header), @p size their bytes, @p exposure_mono_ns the exposure time on the host's monotonic clock. Called on
 * the USB thread: copy the data and return.
 *
 * @ingroup drv_psvr2
 */
typedef void (*psvr2_camera_image_cb)(void *user, const uint8_t *bc4_pair, size_t size, int64_t exposure_mono_ns);

/*!
 * Receive the bottom camera images (NULL @p cb: stop). @p xdev must be a PS VR2 headset.
 *
 * @ingroup drv_psvr2
 */
void
psvr2_set_camera_image_callback(struct xrt_device *xdev, psvr2_camera_image_cb cb, void *user);

/*!
 * Ask for (or release) the camera mode that streams the bottom camera pair (0x10). The headset goes back to its
 * default mode when the last request is released. @p xdev must be a PS VR2 headset.
 *
 * @ingroup drv_psvr2
 */
void
psvr2_request_bottom_camera_images(struct xrt_device *xdev, bool on);

struct t_camera_calibration;
struct xrt_pose;
struct xrt_vec3;

/*!
 * The bottom cameras' calibration (2 entries: left, right of the headset's own lens model) and their poses in the
 * head frame (OpenXR axes: x right, y up, looking along -z), from the headset's calibration. The poses: with
 * @p tracker_poses the controller tracker's camera chain (as hand tracking uses them), otherwise the cameras' measured
 * placement (psvr2_bottom_cameras_in_head). False until the calibration has been received.
 *
 * @ingroup drv_psvr2
 */
bool
psvr2_get_bottom_cameras(struct xrt_device *xdev,
                         struct t_camera_calibration *out_cams,
                         struct xrt_pose *out_poses,
                         bool tracker_poses);


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
 * A Sense controller that connected after the system was opened (the SteamVR plugin finds it while running): set
 * as the headset's left or right controller and added to the LED tracker if that already runs, or the tracker is
 * created now from an already received calibration block (which adds it), or later when the block comes. False if
 * that side already has a controller.
 *
 * @ingroup drv_psvr2
 */
bool
psvr2_hmd_add_late_controller(struct xrt_device *hmd_xdev, struct xrt_device *pssense_xdev, bool left);

/*!
 * Whether optical hand tracking was asked for (PSVR2_HAND_TRACKING): the headset then streams the bottom camera
 * pair (camera mode 0x10) and the builder creates the hand tracking device.
 *
 * @ingroup drv_psvr2
 */
bool
psvr2_hand_tracking_enabled(void);

/*!
 * The latest combined (both eyes) gaze: a unit direction in head space, OpenXR/OpenVR axes (-Z forward, +Y up,
 * +X right), built exactly as the headset's OpenXR eye gaze pose; whether the headset marked it valid; the sample's
 * time and the time its packet was processed, both host monotonic ns. False before the first gaze packet (eye
 * tracking off or not started).
 *
 * @ingroup drv_psvr2
 */
bool
psvr2_get_combined_gaze(struct xrt_device *xdev,
                        struct xrt_vec3 *out_dir,
                        bool *out_valid,
                        int64_t *out_sample_host_ns,
                        int64_t *out_packet_host_ns);

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
