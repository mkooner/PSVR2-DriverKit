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
#include "xrt/xrt_byte_order.h"
#include "xrt/xrt_defines.h"
#include <string.h>
#include "psvr2_constellation_tracking.h"

// Version of the camera relation data that is easier to work with for determining final orientation of all cameras
struct camera_relation_internal {
	uint8_t parent_id;
	uint8_t camera_id;
	struct xrt_quat orientation;
	struct xrt_vec3 position;
};



static struct calibration_camera_distortion *
psvr2_gen_calibration_block_get_camera_calibration(struct gen_calibration_block *block)
{
	return (struct calibration_camera_distortion *)block->data;
}

static struct calibration_camera_relation *
psvr2_gen_calibration_block_get_camera_relations(struct gen_calibration_block *block)
{
	return (struct calibration_camera_relation *)(block->data + block->camera_distortion_calibration_count * sizeof(struct calibration_camera_distortion));
}

void psvr2_parse_gen_calibration_block(struct psvr2_hmd *hmd, uint8_t *data, uint32_t size)
{
	if (size < sizeof(struct gen_calibration_block)) {
		PSVR2_ERROR(hmd, "Calibration block too small");
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

	struct calibration_camera_distortion *calibrations =
	    psvr2_gen_calibration_block_get_camera_calibration(block);
	struct calibration_camera_relation *relations = psvr2_gen_calibration_block_get_camera_relations(block);
	
	struct t_constellation_tracker_params constellation_tracker_params = {
		.num_mosaics = 1,
	};
	
	struct t_constellation_tracker_camera_mosaic *mosaic = &constellation_tracker_params.mosaics[0];
	
	for (int i = 0; i < block->camera_distortion_calibration_count; i++) {
		struct calibration_camera_distortion *calib = &calibrations[i];
		
		PSVR2_DEBUG(hmd, "Camera %d calibration: resolution %dx%d", calib->camera_id, calib->width_pixels,
				calib->height_pixels);
		mosaic->cameras[mosaic->num_cameras++] = (struct t_constellation_tracker_camera){
			.calibration = (struct t_camera_calibration){
				.image_size_pixels.w = calib->width_pixels,
				.image_size_pixels.h = calib->height_pixels,
				.distortion_model = T_DISTORTION_OPENCV_RADTAN_14,
				.rt14 = {
					.k1 = __lef64_to_cpu(calib->distortion_coeffs[0]),
					.k2 = __lef64_to_cpu(calib->distortion_coeffs[1]),
					.p1 = __lef64_to_cpu(calib->distortion_coeffs[2]),
					.p2 = __lef64_to_cpu(calib->distortion_coeffs[3]),
					.k3 = __lef64_to_cpu(calib->distortion_coeffs[4]),
					.k4 = __lef64_to_cpu(calib->distortion_coeffs[5]),
					.k5 = __lef64_to_cpu(calib->distortion_coeffs[6]),
					.k6 = __lef64_to_cpu(calib->distortion_coeffs[7]),
					.s1 = __lef64_to_cpu(calib->distortion_coeffs[8]),
					.s2 = __lef64_to_cpu(calib->distortion_coeffs[9]),
					.s3 = __lef64_to_cpu(calib->distortion_coeffs[10]),
					.s4 = __lef64_to_cpu(calib->distortion_coeffs[11]),
					.tx = __lef64_to_cpu(calib->distortion_coeffs[12]),
					.ty = __lef64_to_cpu(calib->distortion_coeffs[13]),
					// NOTE: Theres 6 more terms of coefficients that could be utilized here,
					// but at the moment RADTAN_14 is the most thats even technically supported.
					// and even then, RADTAN_14 just falls back to RADTAN_8 for now.
				}
			},
			.has_concrete_pose = true,
			// Only correct for cam zero, we need to derive the rest in the next step
			.pose_in_origin = {.position = XRT_VEC3_ZERO,
								.orientation = {.x = 0, .y = 1, .z = 0, .w = 0}},
		};

		// intrinsics is double[3][3], pixel_matrix is float[3][3], we have to unpack in a way
		// that converts between them.
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				mosaic->cameras[i].calibration.intrinsics[j][k] = __lef32_to_cpu(calib->pixel_matrix[j][k]);
		
	}

	// The order we get the relations in isnt very nice to work with by default.
	// need to convert them into a more usable form to apply them.
	struct xrt_matrix_4x4 camera_relative_poses[4] = {0};
	
	// Track the parent
	int parent_ids[4] = {0};

	// root is sitting at origin.
	math_matrix_4x4_identity(&camera_relative_poses[0]);

	struct xrt_matrix_4x4 scratch;
	for (int i = 0; i < block->camera_relation_count; i++) {
		struct calibration_camera_relation *relation = &relations[i];
		struct xrt_matrix_3x3 orientation = __le_matrix_3x3_to_cpu(relation->orientation);
		struct xrt_vec3 translation = __levec3_to_cpu(relation->translation);
		math_matrix_4x4_isometry_from_rt(&orientation, &translation, &scratch);
		
		// write the inverse of the pose if pointing in the wrong direction.
		if (relation->from_camera_id > relation->to_camera_id){
			math_matrix_4x4_inverse(&scratch, &camera_relative_poses[relation->from_camera_id]);
			parent_ids[relation->from_camera_id] = relation->to_camera_id;
		}
		else
		{
			memcpy(&camera_relative_poses[relation->to_camera_id], &scratch, sizeof(struct xrt_matrix_4x4));
			parent_ids[relation->to_camera_id] = relation->from_camera_id;
		}
	}

	// Write final poses
	for (size_t i=0; i < mosaic->num_cameras; i++){
		struct xrt_matrix_4x4 result;
		memcpy(&result, &camera_relative_poses[i], sizeof(struct xrt_matrix_4x4));
		
		size_t parent_id = parent_ids[i];
		while (parent_id != 0){
			// this does internal copies before multiplying, input and output can be the same safely
			math_matrix_4x4_multiply(&camera_relative_poses[parent_id], &result, &result);
			parent_id = parent_ids[parent_id];
		}
		math_pose_from_isometry(&result, &mosaic->cameras[i].pose_in_origin);
		PSVR2_DEBUG(hmd, "camera %lu pose position: (%f, %f, %f)",
			i,
			mosaic->cameras[i].pose_in_origin.position.x,
			mosaic->cameras[i].pose_in_origin.position.y,
			mosaic->cameras[i].pose_in_origin.position.z);
	}

	t_constellation_tracker_create(&hmd->xfctx, &constellation_tracker_params, &hmd->led_tracker);
}