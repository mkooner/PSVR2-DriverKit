// Copyright 2023, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Debug instrumentation for mercury_train or others to control hand tracking.
 * @author Moshi Turner <moshiturner@protonmail.com>
 * @ingroup tracking
 */

#pragma once

#include "hg_interface.h"

#include "util/u_var.h"


#ifdef __cplusplus
namespace xrt::tracking::hand::mercury {
extern "C" {
#endif

struct hg_tuneable_values
{
	bool new_user_event = false;
	struct u_var_draggable_f32 after_detection_fac;
	struct u_var_draggable_f32 dyn_radii_fac;
	struct u_var_draggable_f32 dyn_joint_y_angle_error;
	struct u_var_draggable_f32 amount_to_lerp_prediction;
	struct u_var_draggable_f32 amt_use_depth;
	struct u_var_draggable_f32 mpiou_any;
	struct u_var_draggable_f32 mpiou_single_detection;
	struct u_var_draggable_f32 mpiou_double_detection;
	struct u_var_draggable_f32 max_reprojection_error;
	struct u_var_draggable_f32 opt_smooth_factor;
	struct u_var_draggable_f32 max_hand_dist;
	struct u_var_draggable_f32 min_detection_confidence;
	bool scribble_predictions_into_next_frame = false;
	bool scribble_keypoint_model_outputs = false;
	bool scribble_optimizer_outputs = true;
	bool always_run_detection_model = false;
	bool optimize_hand_size = true;
	int max_num_outside_view = 6;
	size_t num_frames_before_display = 10;
	bool enable_pose_predicted_input = true;
	bool enable_framerate_based_smoothing = false;

	// Stuff that's only really useful for dataset playback:
	bool detection_model_in_both_views = false;
	//! Start a hand from a confident detection in either view (views below the threshold are dropped), instead of
	//! requiring the mean of both views to pass. For camera pairs whose views differ a lot (PS VR2 bottom pair).
	bool detection_any_view = false;
	//! Also run hand detection on the mirrored image and use its right hand (mirrored back) as the left hand when
	//! stronger: the detector finds right hands much more readily (PS VR2 bottom cameras: left hand 0.00-0.12 vs
	//! mirrored 0.59-0.99 in frames where it was visible).
	bool detection_mirror_left = false;
	//! Detection confidence needed to start tracking a hand (Mercury's fixed value was 0.92).
	float acquire_confidence = 0.92f;
	//! Zoomed-in detection: crops per side when a hand isn't found in the whole view (0: off), each this share of it.
	int detection_tiles = 0;
	float detection_tile_fraction = 0.6f;
	int detection_tiles_per_frame = 1; //!< crops looked at per frame, cycling (0: all of them every frame)
	bool detection_tile_mirror = false; //!< also the mirrored left-hand pass on each crop
	//! The keypoint model's "is a hand" score needed to keep a view (Mercury's fixed value was 0.97).
	float keypoint_hand_confidence = 0.97f;
	/*!
	 * Finger keypoints from MediaPipe's hand landmark model (hand_landmarks_detector.onnx, Google's Hand Landmarker,
	 * Apache-2.0) instead of Mercury's own: same crops placed the same way, but 224 px and turned so the fingers point
	 * up (how that model was trained); its 21 points go to the same 3D fitting. Off: Mercury's model.
	 */
	bool mediapipe_keypoints = false;
	float mediapipe_box_fac = 2.0f;     //!< crop side / the hand's extent (MediaPipe's own tracking uses 2.0)
	float mediapipe_presence = 0.5f;    //!< the model's hand presence needed to keep a view (MediaPipe's own: 0.5)
	float mediapipe_xy_conf = 1.0f;     //!< confidence_xy given to every point (the model has no per-point one)
	float mediapipe_depth_conf = 0.0f;  //!< confidence_depth given to its relative depths (0: depth unused)
	/*!
	 * Gesture measurements from MediaPipe's landmarks (needs mediapipe_keypoints): pinch distance and finger
	 * extension from the landmarks, and the category of MediaPipe's Gesture Recognizer (gesture_embedder.onnx +
	 * canned_gesture_classifier.onnx). Read with t_hand_tracking_sync_mercury_get_gestures.
	 */
	bool mediapipe_gestures = false;
};

struct hg_tuneable_values *
t_hand_tracking_sync_mercury_get_tuneable_values_pointer(struct t_hand_tracking_sync *ht_sync);

#ifdef __cplusplus
}
} // namespace xrt::tracking::hand::mercury
#endif
