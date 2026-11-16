// Copyright 2026, Conner Bondurant
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PSVR2 blobwatch implementation deriving from internal led tracking
 *
 * @author Conner Bondurant <conner.bondurant@triptych.me>
 * @ingroup drv_psvr2
 */

#pragma once

#include "psvr2_protocol.h"
#include "xrt/xrt_frame.h"
#include "tracking/t_constellation.h"
#include "tracking/t_camera_models.h"
#include "util/u_file.h"
#include "os/os_threading.h"

// A way to easily allocate two buffers, and keep swapping which one is the scratch buffer
// and which one is the data from the previous frame.
struct psvr2_blobwatch_blob_swapbuffer
{
	struct t_blob *scratch;
	struct t_blob *previous;
	size_t num_previous;
	struct t_blob blobBuffA[PSVR2_MAX_LEDS_PER_CAMERA];
	struct t_blob blobBuffB[PSVR2_MAX_LEDS_PER_CAMERA];
};

#define PSVR2_MOSAIC_COUNT 4

/*!
 * Pixels the LD payload's x coordinates are short by, at the 1016x1016 calibration resolution (4px at the payload's
 * binned 508x508). Documented by PSVR2Toolkit (driver_hooks/ir_tracker_hooks.cpp, IrTracker__ParseConfigHook), which
 * compensates in Sony's tracker by moving cx. We move the blobs instead, so that every consumer of the camera
 * calibration (blobwatch undistortion, tracker pose metrics, debug views) agrees on one model.
 */
#define PSVR2_LD_X_BIAS_PX 8.0f

/*!
 * @implements t_blobwatch
 */
struct psvr2_blobwatch
{
	struct t_blobwatch base;

	struct t_blob_sink *blob_sink[PSVR2_MOSAIC_COUNT];

	uint32_t unused_id;

	struct psvr2_blobwatch_blob_swapbuffer swapbuffer[PSVR2_MOSAIC_COUNT];

	//! Camera models used to undistort blob centres, indexed like the LD payload cameras.
	struct t_camera_model_params camera_models[PSVR2_MOSAIC_COUNT];

	//! Protects the swapbuffers, which are written by the USB thread and labelled by the tracker thread.
	struct os_mutex mutex;

	uint32_t last_received_frame_id;
};


/* Blobwatch functions */
void
psvr2_blobwatch_create(struct xrt_frame_context *xfctx,
                       struct t_blob_sink *blob_sink[4],
                       const struct t_camera_model_params camera_models[PSVR2_MOSAIC_COUNT],
                       struct psvr2_blobwatch **out_blobwatch);

void
psvr2_blobwatch_push_ld_payload(struct psvr2_blobwatch *bw,
                                struct ld_payload *payload,
                                uint64_t sequence_id,
                                timepoint_ns timestamp_ns);
