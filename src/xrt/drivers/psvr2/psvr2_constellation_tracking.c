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
#include "util/u_debug.h"

#include <stdio.h>

/*
 * Camera 0 (bottom left) mount on the headset, relative to the head/SLAM body frame (x right, y up, -z forward):
 * "pitch yaw roll" in degrees, applied after converting the OpenCV camera axes (x right, y down, +z forward).
 * Negative pitch tilts the camera down, positive yaw turns it left (outward for the left camera).
 * Not in the headset's calibration blocks, so it is configurable while it is being verified on hardware.
 */
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
#define PSVR2_CALIB_CAM0_TO_HEAD_OFFSET 0x6f0
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

	// For some reason this gets triggered twice, we only really want to configure this once.
	if ((hmd->pssense_left == NULL && hmd->pssense_right == NULL) || hmd->led_tracker != NULL) {
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
	bool have_mount = false;
	// The calibration's camera-to-head rotation is already in OpenXR camera axes, see below.
	bool mount_in_xr_axes = false;

	if (mount != NULL && sscanf(mount, "%f %f %f", &pitch_deg, &yaw_deg, &roll_deg) >= 2) {
		// Manual override.
		struct xrt_quat q_yaw, q_pitch, q_roll;
		math_quat_from_angle_vector(DEG_TO_RAD(yaw_deg), &(struct xrt_vec3){0, 1, 0}, &q_yaw);
		math_quat_from_angle_vector(DEG_TO_RAD(pitch_deg), &(struct xrt_vec3){1, 0, 0}, &q_pitch);
		math_quat_from_angle_vector(DEG_TO_RAD(roll_deg), &(struct xrt_vec3){0, 0, 1}, &q_roll);
		math_quat_rotate(&q_yaw, &q_pitch, &q_mount);
		math_quat_rotate(&q_mount, &q_roll, &q_mount);
		PSVR2_INFO(hmd, "Camera 0 mount from PSVR2_CAM0_MOUNT: pitch %.1f yaw %.1f roll %.1f deg", pitch_deg,
		           yaw_deg, roll_deg);
		have_mount = true;
	} else if (size >= PSVR2_CALIB_CAM0_TO_HEAD_OFFSET + 9 * sizeof(float)) {
		// From the headset's own calibration: Rx(-15 deg) * R_cam0_to_head, as Sony's driver does.
		struct xrt_matrix_3x3 r = {0};
		for (int j = 0; j < 9; j++) {
			uint32_t bits;
			memcpy(&bits, data + PSVR2_CALIB_CAM0_TO_HEAD_OFFSET + j * 4, sizeof(bits));
			bits = __le32_to_cpu(bits);
			memcpy(&r.v[j], &bits, sizeof(float));
		}
		/*
		 * The block stores the matrix in the other order from xrt_matrix_3x3's: replaying 2621 hardware solves
		 * through this code (psvr2-test/analysis) the transposed mount, with the relations as stored (below), brings
		 * the median disagreement between cameras 0 and 1 about the same controller from 43.9cm to 3.6cm, faces them
		 * +-15 deg outward (the bottom pair is 30.6 deg apart, PSVR2-Documentation) instead of lopsided and 25-30 deg
		 * up, puts the controllers 94% below and 100% in front of the head, and halves the tilt disagreement with the
		 * controllers' IMUs (9.6/14.2 -> 4.9/7.7 deg).
		 */
		math_matrix_3x3_transpose(&r, &r);
		struct xrt_quat q_r;
		math_quat_from_matrix_3x3(&r, &q_r);
		float n = sqrtf(q_r.x * q_r.x + q_r.y * q_r.y + q_r.z * q_r.z + q_r.w * q_r.w);
		if (n > 0.9f && n < 1.1f) {
			struct xrt_quat q_pitch15;
			math_quat_from_angle_vector(DEG_TO_RAD(-15.0), &(struct xrt_vec3){1, 0, 0}, &q_pitch15);
			math_quat_normalize(&q_r);
			math_quat_rotate(&q_pitch15, &q_r, &q_mount);
			PSVR2_INFO(hmd, "Camera 0 mount from the headset calibration (cam-to-head at 0x%x)",
			           PSVR2_CALIB_CAM0_TO_HEAD_OFFSET);
			have_mount = true;
			mount_in_xr_axes = true;
		}
	}

	if (!have_mount) {
		PSVR2_WARN(hmd, "No camera 0 mount in the calibration block, using an estimate (set PSVR2_CAM0_MOUNT)");
		struct xrt_quat q_yaw, q_pitch;
		math_quat_from_angle_vector(DEG_TO_RAD(17.0), &(struct xrt_vec3){0, 1, 0}, &q_yaw);
		math_quat_from_angle_vector(DEG_TO_RAD(-29.0), &(struct xrt_vec3){1, 0, 0}, &q_pitch);
		math_quat_rotate(&q_yaw, &q_pitch, &q_mount);
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

	struct xrt_matrix_3x3 scratch_3x3;
	math_matrix_3x3_from_quat(&q_mount, &scratch_3x3);
	math_matrix_4x4_isometry_from_rt(&scratch_3x3, &(struct xrt_vec3){0, 0, 0}, &camera_relative_poses[0]);

	struct xrt_matrix_4x4 scratch;
	for (int i = 0; i < block->camera_relation_count; i++) {
		struct calibration_camera_relation *relation = &relations[i];
		// Used as stored, see the mount above (the transpose here was half of the 43.9cm camera disagreement).
		struct xrt_matrix_3x3 orientation = __le_matrix_3x3_to_cpu(relation->orientation);
		struct xrt_vec3 translation = __levec3_to_cpu(relation->translation);
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
