// Copyright 2026, Conner Bondurant
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PSVR2 implementation of constellation tracking and camera mosaic
 *
 * @author Conner Bondurant <conner.bondurant@triptych.me>
 * @ingroup drv_psvr2
 */

#include "constellation/t_constellation_tracker.h"

#include "math/m_api.h"
#include "psvr2.h"
#include "psvr2_interface.h"
#include "tracking/t_tracking.h"
#include "tracking/t_camera_models.h"
#include "util/u_sink.h"
#include "xrt/xrt_byte_order.h"
#include "xrt/xrt_defines.h"
#include <string.h>
#include "psvr2_constellation_tracking.h"
#include "psvr2_cam_mount.h"
#include "util/u_debug.h"

#include <stdio.h>

/*
 * Camera 0 (bottom left) mount on the headset, relative to the head/SLAM body frame (x right, y up, -z forward):
 * "pitch yaw roll" in degrees, applied after converting the OpenCV camera axes (x right, y down, +z forward).
 * Negative pitch tilts the camera down, positive yaw turns it left (outward for the left camera).
 * Not in the headset's calibration blocks, so it is configurable while it is being verified on hardware.
 */
/*!
 * The camera 0 mount as before 2026-11-17 (transposed calibration matrix, OpenCV-axes manual angles). Default on: the
 * camera chain below inverts the mount (the tracker's recorded camera 0 pose is the inverse of the mount computed here,
 * 0.08 deg), so the corrected mount must go in with the chain fixed, not on its own. Work in progress, see
 * psvr2-test/analysis/harness/mount_check.c.
 */
DEBUG_GET_ONCE_BOOL_OPTION(psvr2_cam0_mount_legacy, "PSVR2_CAM0_MOUNT_LEGACY", true)
DEBUG_GET_ONCE_OPTION(psvr2_cam0_mount, "PSVR2_CAM0_MOUNT", NULL)

/*
 * Camera rig position relative to the head pose origin, "x y z" in cm (head frame: x right, y up, -z forward), added to
 * every camera. Sony's driver takes the camera positions from its own config (PSVR2Toolkit hmd_device_hooks.cpp,
 * cam0Cfg->position), not from the calibration block, so camera 0 sits at the head origin by default. Measure it with
 * psvr2-test/analysis/rig_offset.py (controller resting in view while the head turns); the 00:04 run had only 3
 * usable windows and gave an implausible 36cm, so there is no data-backed default yet.
 */
DEBUG_GET_ONCE_OPTION(psvr2_cam_rig_offset, "PSVR2_CAM_RIG_OFFSET", NULL)

/*!
 * Offset in the calibration block of camera 0's camera-to-head rotation (row-major 3x3 floats); camera 1's is at
 * 0x7dc. PSVR2Toolkit reads the same offsets from Sony's driver and builds SteamVR's camera-to-head transform as
 * Rx(-15 deg) * R (hmd_device_hooks.cpp). Verified on hardware: R0^T * R1 matches the block's cam0->cam1 relation
 * to 0.01 deg.
 */
#include "../pssense/pssense_interface.h"

// Version of the camera relation data that is easier to work with for determining final orientation of all cameras
struct camera_relation_internal
{
	uint8_t parent_id;
	uint8_t camera_id;
	struct xrt_quat orientation;
	struct xrt_vec3 position;
};



void
psvr2_constellation_tracking_source_create(struct psvr2_hmd *hmd, struct psvr2_constellation_tracking_source **out_src)
{
	struct psvr2_constellation_tracking_source *src = U_TYPED_CALLOC(struct psvr2_constellation_tracking_source);
	src->base.get_tracked_pose = psvr2_constellation_get_tracked_pose;
	src->hmd = hmd;
	m_relation_history_create(&src->history);
	*out_src = src;
}

void
psvr2_constellation_tracking_source_destroy(struct psvr2_constellation_tracking_source **src_ptr)
{
	struct psvr2_constellation_tracking_source *src = *src_ptr;
	if (src == NULL) {
		return;
	}
	m_relation_history_destroy(&src->history);
	free(src);
	*src_ptr = NULL;
}

void
psvr2_constellation_tracking_source_push(struct psvr2_constellation_tracking_source *src,
                                         const struct xrt_space_relation *cam0_relation,
                                         timepoint_ns vts_ns)
{
	src->current_pose = *cam0_relation;
	m_relation_history_push(src->history, cam0_relation, vts_ns);
}

void
psvr2_constellation_get_tracked_pose(struct t_constellation_tracker_tracking_source *src,
                                     int64_t when_ns,
                                     struct xrt_space_relation *out_relation)
{
	struct psvr2_constellation_tracking_source *source = (struct psvr2_constellation_tracking_source *)src;

	/*
	 * The tracker asks in host monotonic time (blob timestamps are converted from the headset VTS clock), while
	 * SLAM poses are stored in VTS time, so convert back with the same offset. An int64 read, so no lock needed
	 * here (and taking data_lock could deadlock if the tracker calls in from a USB callback).
	 */
	timepoint_ns vts_ns = when_ns - source->hmd->hw2mono_vts;

	enum m_relation_history_result res = m_relation_history_get(source->history, vts_ns, out_relation);
	if (res == M_RELATION_HISTORY_RESULT_INVALID) {
		*out_relation = source->current_pose;
	}

	// The head pose each camera sample was placed from, paired with the pssense "OPT" log by timestamp.
	if (source->hmd->record_file != NULL) {
		const struct xrt_pose *p = &out_relation->pose;
		fprintf(source->hmd->record_file, "O,%" PRId64 ",%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f\n", (int64_t)when_ns,
		        p->position.x, p->position.y, p->position.z, p->orientation.x, p->orientation.y,
		        p->orientation.z, p->orientation.w);
	}
}


static struct calibration_camera_distortion *
psvr2_gen_calibration_block_get_camera_calibration(struct gen_calibration_block *block)
{
	return (struct calibration_camera_distortion *)block->data;
}

static struct calibration_camera_relation *
psvr2_gen_calibration_block_get_camera_relations(struct gen_calibration_block *block)
{
	return (struct calibration_camera_relation *)(block->data + block->camera_distortion_calibration_count *
	                                                                sizeof(struct calibration_camera_distortion));
}

void
psvr2_parse_gen_calibration_block(struct psvr2_hmd *hmd, uint8_t *data, uint32_t size)
{
	if (size < sizeof(struct gen_calibration_block)) {
		PSVR2_ERROR(hmd, "Calibration block too small");
		return;
	}

	/*
	 * The block arrives twice at startup, and the first time the builder often hasn't attached the controllers yet.
	 * It's parsed even then (hand tracking needs the bottom cameras), but only counts as done once the LED tracker
	 * exists: marking it done on the first arrival left the second one ignored and the controllers never tracked
	 * (22:53 run: block parsed once before the controllers were attached, no LED tracker, no optical fix all run).
	 */
	if (hmd->led_tracker != NULL) {
		return;
	}

	struct gen_calibration_block *block = (struct gen_calibration_block *)data;

	if (memcmp(block->magic, "SIE\x01", 4) != 0) {
		PSVR2_ERROR(hmd, "Invalid calibration block magic");
		return;
	}

	if (block->block_size != size) {
		PSVR2_ERROR(hmd, "Calibration block size mismatch, expected %u, got %u", block->block_size, size);
		return;
	}

	struct calibration_camera_distortion *calibrations = psvr2_gen_calibration_block_get_camera_calibration(block);
	struct calibration_camera_relation *relations = psvr2_gen_calibration_block_get_camera_relations(block);

	struct t_constellation_tracker_params constellation_tracker_params = {
	    .num_mosaics = 1,
	};

	struct t_constellation_tracker_camera_mosaic *mosaic = &constellation_tracker_params.mosaics[0];

	mosaic->tracking_origin = (struct t_constellation_tracker_tracking_source *)hmd->constellation_origin_source;

	for (int i = 0; i < block->camera_distortion_calibration_count && i < PSVR2_MOSAIC_COUNT; i++) {
		struct calibration_camera_distortion *calib = &calibrations[i];

		PSVR2_DEBUG(hmd, "Camera %d calibration: resolution %dx%d", calib->camera_id, calib->width_pixels,
		            calib->height_pixels);

		mosaic->cameras[mosaic->num_cameras++] = (struct t_constellation_tracker_camera){
		    .calibration =
		        (struct t_camera_calibration){
		            // NOTE: These values are normally 1016x1016,
		            // but the values returned from the ld endpoint are only within the range of 0-507
		            .image_size_pixels.w = calib->width_pixels,
		            .image_size_pixels.h = calib->height_pixels,
		            .distortion_model = T_DISTORTION_PSVR2,
		        },
		    .has_concrete_pose = true,
		    // Only correct for cam zero, we need to derive the rest in the next step
		    .pose_in_origin = {.position = XRT_VEC3_ZERO, .orientation = {.x = 0, .y = 1, .z = 0, .w = 0}},
		};

		for (int j = 0; j < 20; j++) {
			mosaic->cameras[i].calibration.psvr2.params[j] = __lef64_to_cpu(calib->distortion_coeffs[j]);
		}

		// intrinsics is double[3][3], pixel_matrix is float[3][3], we have to unpack in a way
		// that converts between them.
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				mosaic->cameras[i].calibration.intrinsics[j][k] =
				    __lef32_to_cpu(calib->pixel_matrix[j][k]);
	}

	// The order we get the relations in isn't very nice to work with by default.
	// need to convert them into a more usable form to apply them.
	struct xrt_matrix_4x4 camera_relative_poses[4] = {0};

	// Track the parent
	int parent_ids[4] = {0};
	parent_ids[0] = -1;

	// Camera 0 relative to the head pose the origin source reports.
	struct xrt_quat q_cv_to_head, q_mount;
	// OpenCV camera axes (x right, y down, z forward) into OpenVR/OpenXR camera axes (y up, -z forward).
	math_quat_from_angle_vector((float)M_PI, &(struct xrt_vec3){1, 0, 0}, &q_cv_to_head);

	float pitch_deg = 0, yaw_deg = 0, roll_deg = 0;
	const char *mount = debug_get_option_psvr2_cam0_mount();
	const bool legacy = debug_get_bool_option_psvr2_cam0_mount_legacy();
	bool have_mount = false;
	// The calibration's camera-to-head rotation is already in OpenXR camera axes, see below.
	bool mount_in_xr_axes = false;

	if (mount != NULL && sscanf(mount, "%f %f %f", &pitch_deg, &yaw_deg, &roll_deg) >= 2) {
		// Manual override.
		struct xrt_quat q_yaw, q_pitch, q_roll;
		(void)q_yaw;
		(void)q_pitch;
		(void)q_roll;
		psvr2_cam0_mount_from_angles(pitch_deg, yaw_deg, roll_deg, &q_mount);
		mount_in_xr_axes = !legacy; // legacy: OpenCV axes, flipped below (it faced the camera backwards)
		PSVR2_INFO(hmd, "Camera 0 mount from PSVR2_CAM0_MOUNT: pitch %.1f yaw %.1f roll %.1f deg", pitch_deg,
		           yaw_deg, roll_deg);
		have_mount = true;
	} else if (psvr2_cam0_mount_from_block(data, size, -15.0f, legacy, &q_mount)) {
		/*
		 * Rx(-15 deg) * R_cam0_to_head with the matrix as stored. Checked against camera 0's mount measured from
		 * motion (camera rotation vs gyro over 865 frame pairs, psvr2-test/vio/cam_imu_rot.cpp; the two bottom
		 * cameras 31.7 deg apart vs the calibration's 30.2), running this code on the recorded block
		 * (psvr2-test/analysis/harness/mount_check.c): 1.5 deg off, camera looking 28.9 deg down and 17.4 left
		 * (measured 28.2 down, 16.1 left). The previous transposed matrix was 41 deg off (level, looking right);
		 * PSVR2_CAM0_MOUNT_LEGACY=1 brings it back.
		 */
		PSVR2_INFO(hmd, "Camera 0 mount from the headset calibration (cam-to-head at 0x%x)%s",
		           PSVR2_CALIB_CAM0_TO_HEAD_OFFSET, legacy ? ", legacy transposed matrix" : "");
		have_mount = true;
		mount_in_xr_axes = true;
	}

	if (!have_mount) {
		PSVR2_WARN(hmd, "No camera 0 mount in the calibration block, using an estimate (set PSVR2_CAM0_MOUNT)");
		struct xrt_quat q_yaw, q_pitch;
		(void)q_yaw;
		(void)q_pitch;
		psvr2_cam0_mount_from_angles(-29.0f, 17.0f, 0.0f, &q_mount);
		mount_in_xr_axes = !legacy;
	}
	/*
	 * The tracker takes camera poses in OpenXR camera axes (Txr_origin_cam, converted to OpenCV internally). The
	 * calibration's rotations R0..R3 and its camera relations share one convention (R0^T * R1 matches the cam0->cam1
	 * relation to 0.01 deg), so camera i is Rx(-15) * R0 * (R0^T * Ri) = Rx(-15) * Ri, Sony's camera-to-head for
	 * SteamVR (OpenVR camera axes, as OpenXR). Applying the OpenCV -> OpenXR flip on top put it inside every
	 * camera's chain, Rx(-15) * R0 * F * R0^T * Ri: the tracker saw the cameras facing backwards and upside down,
	 * and on hardware put the controllers behind the head (impossible for forward facing cameras) in 87% of solves
	 * and above it in 99%; without it, 99% below and 87-96% in front. The manual and estimated mounts are given
	 * after converting OpenCV axes, so they keep it.
	 */
	if (!mount_in_xr_axes) {
		math_quat_rotate(&q_mount, &q_cv_to_head, &q_mount);
	}
	{
		float vp, vy;
		psvr2_cam_mount_view_angles(&q_mount, &vp, &vy);
		PSVR2_INFO(hmd, "Camera 0 looks %.1f deg %s, %.1f deg %s (measured from motion: 28 down, 16 left)", fabsf(vp),
		           vp < 0 ? "down" : "up", fabsf(vy), vy > 0 ? "left" : "right");
	}

	struct xrt_matrix_3x3 scratch_3x3;
	math_matrix_3x3_from_quat(&q_mount, &scratch_3x3);
	math_matrix_4x4_isometry_from_rt(&scratch_3x3, &(struct xrt_vec3){0, 0, 0}, &camera_relative_poses[0]);

	struct xrt_matrix_4x4 scratch;
	for (int i = 0; i < block->camera_relation_count; i++) {
		struct calibration_camera_relation *relation = &relations[i];
		/*
		 * A relation is the "to" camera's pose in the "from" camera, in OpenCV camera axes, with the rotation stored
		 * in the other matrix order (as the mount above): in the OpenXR axes the chain uses that is F O^T F, F t
		 * (F = diag(1, -1, -1)). Used as stored (O, t), cameras 2 and 3 never solved: with camera 0's controller
		 * poses, their predicted LEDs landed hundreds of pixels from their blobs, and cameras 0 and 1 disagreed by a
		 * fixed 3.9-4.2cm. Fitting each camera's pose from the recorded datasets (psvr2-test/analysis/fit_camera2.py,
		 * which recovers camera 1 from random starts to 0.25cm) and searching conventions over all cameras at once
		 * (chain_unify.py), this one explains them all, with the storage directions the block actually has (1->0 and
		 * 3->2 reversed, 0->2 not): camera 0's LEDs within 3px of each camera's blobs, camera 1 94.7% (was 3.3%),
		 * camera 2 within 0.9 deg / 0.2cm of its fitted pose (was 72 deg / 12cm), camera 3 61.4% (was 0%). For
		 * camera 1's pure yaw F O^T F is O, which is why its rotation looked right used as stored.
		 */
		struct xrt_matrix_3x3 stored = __le_matrix_3x3_to_cpu(relation->orientation);
		struct xrt_vec3 translation = __levec3_to_cpu(relation->translation);
		struct xrt_matrix_3x3 orientation;
		math_matrix_3x3_transpose(&stored, &orientation);
		orientation.v[1] = -orientation.v[1];
		orientation.v[2] = -orientation.v[2];
		orientation.v[3] = -orientation.v[3];
		orientation.v[6] = -orientation.v[6];
		translation.y = -translation.y;
		translation.z = -translation.z;
		PSVR2_DEBUG(hmd, "camera relation %d: to %d from %d", i, relation->to_camera_id, relation->from_camera_id);
		math_matrix_4x4_isometry_from_rt(&orientation, &translation, &scratch);

		// write the inverse of the pose if pointing in the wrong direction.
		if (relation->from_camera_id > relation->to_camera_id) {
			math_matrix_4x4_inverse(&scratch, &camera_relative_poses[relation->from_camera_id]);
			parent_ids[relation->from_camera_id] = relation->to_camera_id;
		} else {
			memcpy(&camera_relative_poses[relation->to_camera_id], &scratch, sizeof(struct xrt_matrix_4x4));
			parent_ids[relation->to_camera_id] = relation->from_camera_id;
		}
	}

	for (int i = 1; i < 4; i++) {
		PSVR2_DEBUG(hmd, "camera %d parent %d", i, parent_ids[i]);
	}

	// Write final poses
	for (size_t i = 0; i < mosaic->num_cameras; i++) {
		struct xrt_matrix_4x4 result;
		memcpy(&result, &camera_relative_poses[i], sizeof(struct xrt_matrix_4x4));

		int parent_id = parent_ids[i];
		while (parent_id != -1) {
			// this does internal copies before multiplying, input and output can be the same safely
			math_matrix_4x4_multiply(&camera_relative_poses[parent_id], &result, &result);
			parent_id = parent_ids[parent_id];
		}
		math_pose_from_isometry(&result, &mosaic->cameras[i].pose_in_origin);
		const char *rig = debug_get_option_psvr2_cam_rig_offset();
		float rx = 0, ry = 0, rz = 0;
		if (rig != NULL && sscanf(rig, "%f %f %f", &rx, &ry, &rz) == 3) {
			mosaic->cameras[i].pose_in_origin.position.x += rx / 100.0f;
			mosaic->cameras[i].pose_in_origin.position.y += ry / 100.0f;
			mosaic->cameras[i].pose_in_origin.position.z += rz / 100.0f;
		}
		PSVR2_DEBUG(hmd, "camera %zu pose position: (%f, %f, %f)", i,
		            mosaic->cameras[i].pose_in_origin.position.x, mosaic->cameras[i].pose_in_origin.position.y,
		            mosaic->cameras[i].pose_in_origin.position.z);
	}

	if (mosaic->num_cameras >= 2) {
		psvr2_hmd_offer_hand_tracking_cameras(
		    hmd, (struct t_camera_calibration[2]){mosaic->cameras[0].calibration, mosaic->cameras[1].calibration},
		    (struct xrt_pose[2]){mosaic->cameras[0].pose_in_origin, mosaic->cameras[1].pose_in_origin});
	}

	// The LED tracker only with controllers (as before hand tracking needed the parse above).
	if (hmd->pssense_left == NULL && hmd->pssense_right == NULL) {
		return;
	}

	t_constellation_tracker_create(&hmd->xfctx, &constellation_tracker_params, &hmd->led_tracker);

	// Controller poses are in the headset's SLAM space, so give them the same floor offset as the head.
	t_constellation_tracker_get_tracking_origin(hmd->led_tracker)->initial_offset =
	    hmd->base.tracking_origin->initial_offset;
	// Write blob sinks into the blobwatch
	for (size_t i = 0; i < mosaic->num_cameras; i++) {
		hmd->blob_sinks[i] = mosaic->cameras[i].blob_sink;
		// u_sink_blob_visualizer_create(&hmd->xfctx, hmd->blob_sinks[i], &hmd->debug_sinks[i], 600, 600,
		//                               &hmd->blob_sinks[i]);
	}


	struct t_camera_model_params camera_models[PSVR2_MOSAIC_COUNT] = {0};
	for (size_t i = 0; i < mosaic->num_cameras; i++) {
		// Same model the tracker builds from this calibration; the LD x bias is applied to the blobs.
		t_camera_model_params_from_t_camera_calibration(&mosaic->cameras[i].calibration, &camera_models[i]);
	}

	psvr2_blobwatch_create(
	    &hmd->xfctx,
	    (struct t_blob_sink *[]){hmd->blob_sinks[0], hmd->blob_sinks[1], hmd->blob_sinks[2], hmd->blob_sinks[3]},
	    camera_models, &hmd->blobwatch);

	if (hmd->pssense_left) {
		pssense_add_to_constellation_tracker(hmd->pssense_left, hmd->led_tracker);
	}
	if (hmd->pssense_right) {
		pssense_add_to_constellation_tracker(hmd->pssense_right, hmd->led_tracker);
	}
}

bool
psvr2_hmd_add_late_controller(struct xrt_device *hmd_xdev, struct xrt_device *pssense_xdev, bool left)
{
	struct psvr2_hmd *hmd = (struct psvr2_hmd *)hmd_xdev;
	// Under calib_lock, as the USB thread creates the LED tracker (and adds pssense_left/right) under it.
	os_mutex_lock(&hmd->calib_lock);
	struct xrt_device **slot = left ? &hmd->pssense_left : &hmd->pssense_right;
	if (*slot != NULL) {
		os_mutex_unlock(&hmd->calib_lock);
		return false;
	}
	*slot = pssense_xdev;
	if (hmd->led_tracker != NULL) {
		// The tracker takes devices while running (its device lock).
		pssense_add_to_constellation_tracker(pssense_xdev, hmd->led_tracker);
		PSVR2_INFO(hmd, "Late %s Sense controller added to the running LED tracker", left ? "left" : "right");
	} else if (hmd->calib_block != NULL) {
		PSVR2_INFO(hmd, "Late %s Sense controller: creating the LED tracker from the calibration block",
		           left ? "left" : "right");
		psvr2_parse_gen_calibration_block(hmd, hmd->calib_block, hmd->calib_block_size);
	}
	os_mutex_unlock(&hmd->calib_lock);
	return true;
}
