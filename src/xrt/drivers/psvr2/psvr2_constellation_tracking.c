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
#include "../pssense/pssense_interface.h"

// Version of the camera relation data that is easier to work with for determining final orientation of all cameras
//! Pixels the LD payload's x coordinates are off by, at the 1016x1016 calibration resolution.
#define PSVR2_LD_X_BIAS_PX 8.0f

struct camera_relation_internal
{
	uint8_t parent_id;
	uint8_t camera_id;
	struct xrt_quat orientation;
	struct xrt_vec3 position;
};



void
psvr2_constellation_tracking_source_create(struct psvr2_constellation_tracking_source **out_src)
{
	struct psvr2_constellation_tracking_source *src = U_TYPED_CALLOC(struct psvr2_constellation_tracking_source);
	src->base.get_tracked_pose = psvr2_constellation_get_tracked_pose;
	*out_src = src;
}

void
psvr2_constellation_get_tracked_pose(struct t_constellation_tracker_tracking_source *src,
                                     int64_t when_ns,
                                     struct xrt_space_relation *out_relation)
{
	struct psvr2_constellation_tracking_source *source = (struct psvr2_constellation_tracking_source *)src;

	*out_relation = source->current_pose;
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

	// root is sitting at origin, with a slight hard coded rotation
	struct xrt_quat scratch_quat;
	struct xrt_matrix_3x3 scratch_3x3;
	math_quat_from_euler_angles(&(struct xrt_vec3){15, 145, 0}, &scratch_quat);
	math_matrix_3x3_from_quat(&scratch_quat, &scratch_3x3);
	math_matrix_4x4_isometry_from_rt(&scratch_3x3, &(struct xrt_vec3){0, 0, 0}, &camera_relative_poses[0]);

	struct xrt_matrix_4x4 scratch;
	for (int i = 0; i < block->camera_relation_count; i++) {
		struct calibration_camera_relation *relation = &relations[i];
		struct xrt_matrix_3x3 orientation = __le_matrix_3x3_to_cpu(relation->orientation);
		math_matrix_3x3_transpose(&orientation, &orientation);
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
		PSVR2_DEBUG(hmd, "camera %zu pose position: (%f, %f, %f)", i,
		            mosaic->cameras[i].pose_in_origin.position.x, mosaic->cameras[i].pose_in_origin.position.y,
		            mosaic->cameras[i].pose_in_origin.position.z);
	}

	t_constellation_tracker_create(&hmd->xfctx, &constellation_tracker_params, &hmd->led_tracker);
	// Write blob sinks into the blobwatch
	for (size_t i = 0; i < mosaic->num_cameras; i++) {
		hmd->blob_sinks[i] = mosaic->cameras[i].blob_sink;
		// u_sink_blob_visualizer_create(&hmd->xfctx, hmd->blob_sinks[i], &hmd->debug_sinks[i], 600, 600,
		//                               &hmd->blob_sinks[i]);
	}


	struct t_camera_model_params camera_models[PSVR2_MOSAIC_COUNT] = {0};
	for (size_t i = 0; i < mosaic->num_cameras; i++) {
		t_camera_model_params_from_t_camera_calibration(&mosaic->cameras[i].calibration, &camera_models[i]);

		/*
		 * The LD payload's x coordinates are 4px short at its binned 508x508 resolution, so 8px at the
		 * 1016x1016 calibration resolution. Shift the principal point to compensate, as PSVR2Toolkit does
		 * for Sony's tracker (driver_hooks/ir_tracker_hooks.cpp, IrTracker__ParseConfigHook).
		 */
		camera_models[i].cx -= PSVR2_LD_X_BIAS_PX;
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
