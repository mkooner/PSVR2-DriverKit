// Copyright 2022, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Hand tracking interfaces.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup xrt_iface
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"


#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Image boundary type.
 *
 * Currently used by hand-tracking to determine if parts of the hand are not visible to the camera, ie. they are outside
 * of the camera's vignette.
 *
 * Feel free to move this out of t_hand_tracking if this becomes more generally applicable.
 *
 * @ingroup xrt_iface
 */
enum t_image_boundary_type
{
	HT_IMAGE_BOUNDARY_NONE,
	HT_IMAGE_BOUNDARY_CIRCLE,
};

/*!
 * @brief Circular image boundary.
 *
 * Currently used by hand-tracking to determine if parts of the hand are not visible to the camera, ie. they are outside
 * of the camera's vignette.
 *
 * Feel free to move this out of t_hand_tracking if this becomes more generally applicable.
 *
 * @ingroup xrt_iface
 */
struct t_image_boundary_circle
{
	// The center, in normalized 0-1 UV coordinates.
	// Should probably be between 0 and 1 in pixel coordinates.
	struct xrt_vec2 normalized_center;
	// The radius, divided by the image width.
	// For Index, should be around 0.5.
	float normalized_radius;
};

/*!
 * @brief Logical orientation of the camera image, relative to the user's head.
 * For example, Rift S uses CAMERA_ORIENTATION_90 for the two front cameras.
 *
 * Feel free to move this out of t_hand_tracking if this becomes more generally applicable.
 *
 */
enum t_camera_orientation
{
	CAMERA_ORIENTATION_0 = 0,     // Normal "horizontal" orientation
	CAMERA_ORIENTATION_90 = 90,   // Camera rotated 90° to the right
	CAMERA_ORIENTATION_180 = 180, // Camera rotated 180° upside down
	CAMERA_ORIENTATION_270 = 270, // Camera rotated 270° to the left
};


/*!
 * @brief Information about image boundary and camera orientation for one view.
 *
 * Currently used by hand-tracking to determine if parts of the hand are not
 * visible to the camera, ie. they are outside of the camera's vignette.
 *
 * @ingroup xrt_iface
 */
struct t_camera_extra_info_one_view
{
	enum t_image_boundary_type boundary_type;

	union {
		struct t_image_boundary_circle circle;
	} boundary;

	enum t_camera_orientation camera_orientation;
};

/*!
 * @brief Information about image boundaries and camera orientations for all the
 * cameras used in a tracking system.
 *
 * Currently used by hand-tracking to determine if parts of the hand are not
 * visible to the camera, ie. they are outside of the camera's vignette.
 *
 * @ingroup xrt_iface
 */
struct t_camera_extra_info
{
	//!@todo Hardcoded to 2 - needs to increase as we support headsets with more cameras.
	struct t_camera_extra_info_one_view views[2];
};

/*!
 * Creation info for the creation of a hand tracker
 */
struct t_hand_tracking_create_info
{
	struct t_camera_extra_info cams_info;     //!< Extra camera info
	struct xrt_device_masks_sink *masks_sink; //!< Optional sink to stream hand bounding boxes to
};

/*!
 * Hand gesture categories of MediaPipe's Gesture Recognizer ("canned" classifier; this order is its output order).
 */
enum t_hand_gesture_category
{
	T_HAND_GESTURE_NONE = 0,
	T_HAND_GESTURE_CLOSED_FIST = 1,
	T_HAND_GESTURE_OPEN_PALM = 2,
	T_HAND_GESTURE_POINTING_UP = 3,
	T_HAND_GESTURE_THUMB_DOWN = 4,
	T_HAND_GESTURE_THUMB_UP = 5,
	T_HAND_GESTURE_VICTORY = 6,
	T_HAND_GESTURE_I_LOVE_YOU = 7,
	T_HAND_GESTURE_COUNT = 8,
};

/*!
 * Gestures of a custom mapping fitted on labelled camera images of the headset (e.g. Mercury's PS VR2 mapping on
 * MediaPipe's gesture embedding and joints of both cameras).
 */
enum t_hand_custom_gesture
{
	T_HAND_CUSTOM_OPEN = 0,
	T_HAND_CUSTOM_PINCH = 1,
	T_HAND_CUSTOM_FIST = 2,
	T_HAND_CUSTOM_POINT = 3,
	T_HAND_CUSTOM_CLAW = 4,
	T_HAND_CUSTOM_COUNT = 5,
};

/*!
 * One hand's gesture measurements for one camera frame (a hand tracker that has them, e.g. Mercury with MediaPipe's
 * models). Measured on the hand alone: distances are in MediaPipe's hand-centred frame, not camera or room space.
 */
struct t_hand_gestures
{
	bool valid;                      //!< false: nothing below is meaningful
	int64_t timestamp_ns;            //!< the camera frame's timestamp (same as the hand joints')
	float confidence;                //!< the landmark model's hand presence, 0..1
	float pinch_distance_normalized; //!< thumb tip to index tip / palm width (index to little finger knuckle)
	uint8_t extended_mask;           //!< bit f: finger f extended (0 thumb ... 4 little finger)
	enum t_hand_gesture_category category; //!< the gesture classifier's top category
	float category_score;                  //!< its score, 0..1
	bool pinch_image_valid;                //!< false: pinch_image isn't meaningful
	float pinch_image;                     //!< the image pinch detector's probability (mean over the cameras), 0..1
	bool joint_features_valid;             //!< false: joint_features isn't meaningful
	//! The joint features of MediaPipe's landmarks (Mercury's gesture::jointFeatures order and units), from the
	//! camera with the highest hand presence: what calibrated gesture rules read.
	float joint_features[22];
	bool custom_valid;                     //!< false: the custom mapping below isn't meaningful
	enum t_hand_custom_gesture custom;     //!< the custom mapping's gesture (over its last frames)
	float custom_scores[T_HAND_CUSTOM_COUNT]; //!< its probability per gesture, 0..1
};

/*!
 * Why one hand was or wasn't tracked, counted per camera frame since the last read (a hand tracker that keeps them,
 * e.g. Mercury): to tell a hand that was seen and rejected from one that was never found.
 */
struct t_hand_loss_stats
{
	uint32_t frames;       //!< camera frames processed
	uint32_t tracked;      //!< frames the hand was tracked
	uint32_t not_detected; //!< not tracked: the hand detector found nothing
	uint32_t no_region;    //!< not tracked: tracked before, its predicted place left both views
	uint32_t outside_view; //!< not tracked: the predicted hand fell outside the views
	uint32_t keypoint;     //!< not tracked: found, then rejected by the keypoint model(s) in every view
	uint32_t reprojection; //!< not tracked: the 3D hand didn't match the keypoints
	uint32_t too_far;      //!< not tracked: the 3D hand was implausibly far away
	uint32_t overlap;      //!< not tracked: both hands overlapped
	uint32_t other;        //!< not tracked: another reason
	//! Keypoint rejections by the best "is a hand" score of Mercury's model among the views: <0.90, 0.90-0.96,
	//! 0.96-0.98, >=0.98 (rejected for another reason, e.g. MediaPipe's presence).
	uint32_t keypoint_score[4];
	uint32_t mediapipe_rejected_views; //!< views MediaPipe's landmark model didn't take as a hand (Mercury's then ran)
};

/*!
 * Synchronously processes frames and returns two hands.
 */
struct t_hand_tracking_sync
{
	/*!
	 * Process left and right view and get back a result synchronously.
	 */
	void (*process)(struct t_hand_tracking_sync *ht_sync,
	                struct xrt_frame *left_frame,
	                struct xrt_frame *right_frame,
	                struct xrt_hand_joint_set *out_left_hand,
	                struct xrt_hand_joint_set *out_right_hand,
	                int64_t *out_timestamp_ns);

	/*!
	 * Destroy this hand tracker sync object.
	 */
	void (*destroy)(struct t_hand_tracking_sync *ht_sync);
};

/*!
 * @copydoc t_hand_tracking_sync::process
 *
 * @public @memberof t_hand_tracking_sync
 */
static inline void
t_ht_sync_process(struct t_hand_tracking_sync *ht_sync,
                  struct xrt_frame *left_frame,
                  struct xrt_frame *right_frame,
                  struct xrt_hand_joint_set *out_left_hand,
                  struct xrt_hand_joint_set *out_right_hand,
                  int64_t *out_timestamp_ns)
{
	ht_sync->process(ht_sync, left_frame, right_frame, out_left_hand, out_right_hand, out_timestamp_ns);
}

/*!
 * @copydoc t_hand_tracking_sync::destroy
 *
 * Helper for calling through the function pointer: does a null check and sets
 * ht_sync_ptr to null if freed.
 *
 * @public @memberof t_hand_tracking_sync
 */
static inline void
t_ht_sync_destroy(struct t_hand_tracking_sync **ht_sync_ptr)
{
	struct t_hand_tracking_sync *ht_sync = *ht_sync_ptr;
	if (ht_sync == NULL) {
		return;
	}

	ht_sync->destroy(ht_sync);
	*ht_sync_ptr = NULL;
}

struct t_hand_tracking_async
{
	struct xrt_frame_node node;
	struct xrt_frame_sink left;
	struct xrt_frame_sink right;
	struct xrt_tracking_sinks sinks; //!< Pointers to `left` and `right` sinks

	void (*get_hand)(struct t_hand_tracking_async *ht_async,
	                 enum xrt_input_name name,
	                 int64_t desired_timestamp_ns,
	                 struct xrt_hand_joint_set *out_value,
	                 int64_t *out_timestamp_ns);
};

struct t_hand_tracking_async *
t_hand_tracking_async_default_create(struct xrt_frame_context *xfctx, struct t_hand_tracking_sync *sync);


#ifdef __cplusplus
} // extern "C"
#endif
