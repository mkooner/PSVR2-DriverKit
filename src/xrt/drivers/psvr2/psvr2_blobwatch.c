// Copyright 2026, Conner Bondurant
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PSVR2 blobwatch implementation deriving from internal led tracking
 *
 * @author Conner Bondurant <conner.bondurant@triptych.me>
 * @ingroup drv_psvr2
 */


#include "psvr2_protocol.h"
#include "psvr2_blobwatch.h"
#include "util/u_misc.h"

/*!
 * Notify the blobwatch that the blobs in the given observation with the correct ID set are
 * associated with the given device. The blobwatch can use this information to track which
 * blobs are associated with which devices across frames, and to provide this information to
 * the tracker across frames to save it from doing that work again.
 *
 * @param[in] tbw       The blobwatch to mark the blobs for.
 * @param[in] tbo       The observation containing the blobs to mark. The blobwatch will look at
 *                      the blob IDs and the matched_device_id field to determine which blobs
 *                      internally to mark with the given device ID.
 * @param[in] device_id The device ID to mark
 */
void
psvr2_blobwatch_mark_blob_device(struct t_blobwatch *tbw,
                                 const struct t_blob_observation *tbo,
                                 t_constellation_device_id_t device_id)
{}

void
led_to_blob(uint32_t id, struct ld_led *led, struct t_blob *out_blob)
{
	out_blob->blob_id = id;
	out_blob->center = (struct xrt_vec2){
	    .x = (float)(led->leftmost + led->rightmost) / 2.0f,
	    .y = (float)(led->topmost + led->bottommost) / 2.0f,
	};
	out_blob->motion_vector = (struct xrt_vec2){0}, // We don't have motion vector data, so set it to 0
	    out_blob->bounding_box = (struct xrt_rect){
	        .offset =
	            {
	                .w = (float)led->leftmost,
	                .h = (float)led->topmost,
	            },
	        .extent =
	            {
	                .w = (float)(led->rightmost - led->leftmost),
	                .h = (float)(led->bottommost - led->topmost),
	            },
	    };
	out_blob->size = (struct xrt_vec2){
	    .x = (float)(led->rightmost - led->leftmost),
	    .y = (float)(led->bottommost - led->topmost),
	};
}


// Static allocated storage as a scratch-pad for the leds converted to blob format
static struct t_blob blob_storage_array[PSVR2_MAX_LEDS_PER_CAMERA * 4];

void
psvr2_blobwatch_push_ld_payload(struct psvr2_blobwatch *bw, struct ld_payload *payload)
{
	for (size_t i = 0; i < ARRAY_SIZE(payload->cameras); i++) {
		struct ld_camera *camera = &payload->cameras[i];

		for (size_t j = 0; j < camera->num_leds; j++) {
			led_to_blob(j, &camera->leds[j], &blob_storage_array[i * PSVR2_MAX_LEDS_PER_CAMERA + j]);
		}

		struct t_blob_observation xbo = {
		    .source = &bw->base,
		    .id = i,
		    // Timestamp is in microseconds, needs conversion to nanoseconds
		    // TODO: alignment to the global time?
		    .timestamp_ns = payload->header.timestamp * 1000,
		    .blobs = blob_storage_array + i * PSVR2_MAX_LEDS_PER_CAMERA,
		    .num_blobs = camera->num_leds,
		};

		bw->blob_sink[i]->push_blobs(bw->blob_sink[i], &xbo);
	}
}

/*!
 * Destroy this blobwatch.
 */
void
psvr2_blobwatch_destroy(struct t_blobwatch *tbw)
{
	free(tbw);
}

void
psvr2_blob_stub_sink_push_blobs(struct t_blob_sink *sink, struct t_blob_observation *observation)
{
	// Do nothing, exists to handle if we dont use all cameras for some reason
}

void
psvr2_blob_stub_sink_destroy(struct t_blob_sink *self)
{
	// Stub sink 
}

static struct t_blob_sink stub_blob_sink = {
    .push_blobs = psvr2_blob_stub_sink_push_blobs,
    .destroy = psvr2_blob_stub_sink_destroy,
};


void
psvr2_blobwatch_create(struct xrt_frame_context *xfctx,
                       struct t_blob_sink *blob_sink[4],
                       struct psvr2_blobwatch **out_blobwatch)
{
	struct psvr2_blobwatch *bw = U_TYPED_CALLOC(struct psvr2_blobwatch);
	bw->base.mark_blob_device = psvr2_blobwatch_mark_blob_device;
	bw->base.destroy = psvr2_blobwatch_destroy;
	for (size_t i = 0; i < 4; i++) {
		bw->blob_sink[i] = blob_sink && blob_sink[i] ? blob_sink[i] : &stub_blob_sink;
	}

	*out_blobwatch = bw;
}