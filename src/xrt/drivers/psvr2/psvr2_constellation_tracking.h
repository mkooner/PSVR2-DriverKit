// Copyright 2026, Conner Bondurant
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PSVR2 implementation of constellation tracking and camera mosaic
 *
 * @author Conner Bondurant <conner.bondurant@triptych.me>
 * @ingroup drv_psvr2
 */

#pragma once

#include "psvr2.h"
#include "xrt/xrt_byte_order.h"
#include "tracking/t_constellation.h"

struct gen_calibration_block
{
	char magic[4];     // "SIE\x01" (@ref slam_usb_record has the same 0x01 trailing byte)
	__le16 block_size; // 2048 for the whole block, including this header, as far as has been observed.
	uint8_t unk1;
	char model_name[8];
	uint8_t camera_distortion_calibration_count;
	uint8_t camera_relation_count;
	uint8_t unknown_padding[7];
	uint8_t data[]; // variable length data depending on counts above
} PSVR2_PACKED;

struct calibration_camera_distortion
{
	__le32 camera_id;
	__le16 width_pixels;
	__le16 height_pixels;
	// Small implementation detail, this gets moved into a t_camera_calibration.intrinsics, which is `double[3][3]`.
	// Easier to unpack and cast from float to double if we dont represent it as a `__le_matrix_3x3`
	__lef32 pixel_matrix[3][3];
	__lef64 distortion_coeffs[20];
	uint8_t unknown_padding[24];
} PSVR2_PACKED;

struct calibration_camera_relation
{
	uint8_t to_camera_id;
	uint8_t from_camera_id;
	uint8_t reserved[2];
	struct __le_matrix_3x3 orientation; // 3x3 rotation matrix
	struct __levec3 translation;        // translation offset.
} PSVR2_PACKED;

/*!
 * Takes the calibration block from GEN_DATA_CALIBRATION_BLOCK returned on the vd endpoint.
 * Uses the calibration data to update the camera distortion models, and generate a camera mosaic.
 */
void
psvr2_parse_gen_calibration_block(struct psvr2_hmd *hmd, uint8_t *data, uint32_t size);


struct psvr2_constellation_tracking_source
{
	struct t_constellation_tracker_tracking_source base;
	struct psvr2_hmd *hmd;

	//! Latest camera 0 pose, used if the history has nothing yet.
	struct xrt_space_relation current_pose;

	//! Camera 0 poses from the headset SLAM, in headset VTS time.
	struct m_relation_history *history;
};

void
psvr2_constellation_tracking_source_create(struct psvr2_hmd *hmd, struct psvr2_constellation_tracking_source **out_src);

void
psvr2_constellation_tracking_source_destroy(struct psvr2_constellation_tracking_source **src_ptr);

void
psvr2_constellation_tracking_source_push(struct psvr2_constellation_tracking_source *src,
                                         const struct xrt_space_relation *cam0_relation,
                                         timepoint_ns vts_ns);

void
psvr2_constellation_get_tracked_pose(struct t_constellation_tracker_tracking_source *,
                                     int64_t when_ns,
                                     struct xrt_space_relation *out_relation);
