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
#include "util/u_file.h"

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
 * @implements t_blobwatch
 */
struct psvr2_blobwatch
{
	struct t_blobwatch base;

	struct t_blob_sink *blob_sink[PSVR2_MOSAIC_COUNT];

	uint32_t unused_id;

	struct psvr2_blobwatch_blob_swapbuffer swapbuffer[PSVR2_MOSAIC_COUNT];

	uint32_t last_received_frame_id;
};


/* Blobwatch functions */
void
psvr2_blobwatch_create(struct xrt_frame_context *xfctx,
                       struct t_blob_sink *blob_sink[4],
                       struct psvr2_blobwatch **out_blobwatch);

void
psvr2_blobwatch_push_ld_payload(struct psvr2_blobwatch *bw, struct ld_payload *payload);
