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

/*!
 * @implements t_blobwatch
 */
struct psvr2_blobwatch
{
	struct t_blobwatch base;

	struct t_blob_sink *blob_sink[4];
};

/* Blobwatch functions */

void
psvr2_blobwatch_create(struct xrt_frame_context *xfctx,
                       struct t_blob_sink *blob_sink[4],
                       struct psvr2_blobwatch **out_blobwatch);

void
psvr2_blobwatch_push_ld_payload(struct psvr2_blobwatch *bw, struct ld_payload *payload);
