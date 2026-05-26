// Copyright 2026, Conner Bondurant
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PSVR2 blobwatch implementation deriving from internal led tracking
 *
 * @author Conner Bondurant <conner.bondurant@triptych.me>
 * @ingroup drv_psvr2
 */


#include "math/m_vec2.h"
#include "psvr2_protocol.h"
#include "psvr2_blobwatch.h"
#include "tracking/t_constellation.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "xrt/xrt_defines.h"
#include <assert.h>
#include <errno.h>
#include <string.h>

void
psvr2_blobwatch_mark_blob_device(struct t_blobwatch *tbw,
                                 const struct t_blob_observation *tbo,
                                 t_constellation_device_id_t device_id)
{
	struct psvr2_blobwatch *bw = (struct psvr2_blobwatch *)tbw;

	for (size_t j = 0; j < bw->swapbuffer[tbo->id].num_previous; j++) {
		for (size_t i = 0; i < tbo->num_blobs; i++) {
			if (tbo->blobs[i].blob_id == bw->swapbuffer[tbo->id].previous[j].blob_id) {
				bw->swapbuffer[tbo->id].previous[j].matched_device_id = device_id;
				break;
			}
		}
	}
}

void
led_to_blob(struct ld_led *led, struct t_blob *out_blob)
{
	out_blob->matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
	out_blob->matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
	out_blob->center = (struct xrt_vec2){
	    .x = (float)((led->leftmost + led->rightmost) / 2),
	    .y = (float)((led->topmost + led->bottommost) / 2),
	};
	out_blob->motion_vector = (struct xrt_vec2){0}, // We don't have motion vector data, so set it to 0
	    out_blob->bounding_box = (struct xrt_rect){
	        .offset =
	            {
	                .w = led->leftmost,
	                .h = led->topmost,
	            },
	        .extent =
	            {
	                .w = led->rightmost - led->leftmost,
	                .h = led->bottommost - led->topmost,
	            },
	    };
	out_blob->size = (struct xrt_vec2){
	    .x = (float)(led->rightmost - led->leftmost),
	    .y = (float)(led->bottommost - led->topmost),
	};
	out_blob->brightness = 1.0;
}


// Static allocated storage as a scratch-pad for the leds converted to blob format
void
psvr2_blobwatch_toggle_swapbuffer(struct psvr2_blobwatch_blob_swapbuffer *swb, size_t num_previous)
{
	struct t_blob *tmp = swb->previous;
	swb->previous = swb->scratch;
	swb->scratch = tmp;
	swb->num_previous = num_previous;
}


bool
rect_overlaps(struct xrt_rect a, struct xrt_rect b)
{
	return a.offset.h < b.offset.h + b.extent.h && b.offset.h < a.offset.h + a.extent.h &&
	       a.offset.w < b.offset.w + b.extent.w && b.offset.w < a.offset.w + a.extent.w;
}


void
psvr2_blobwatch_push_ld_payload(struct psvr2_blobwatch *bw, struct ld_payload *payload)
{
	if (bw == NULL) {
		return;
	}
	for (size_t i = 0; i < ARRAY_SIZE(payload->cameras) && i < PSVR2_MOSAIC_COUNT; i++) {

		struct ld_camera *camera = &payload->cameras[i];


		bool already_mapped[PSVR2_MAX_LEDS_PER_CAMERA] = {0};

		if (camera->num_leds == 0) {
			// dont even bother processing empty frames, that just causes issues with tracking continuity
			continue;
		}

		// TODO: instead of just overwriting the LEDs each update
		// Check if any LEDs look like an existing one, and just update it in place instead.
		// NOTE: It seems to be that the LEDs are reported sorted by height from the bottom of the "image".
		// (IDK off the top of my head which way y grows) This should be usable to simplify the comparison
		// algorithm for matching the prior frame to the current frame.
		for (size_t j = 0; j < camera->num_leds; j++) {
			struct t_blob *destinationBlob = &bw->swapbuffer[i].scratch[j];
			led_to_blob(&camera->leds[j], destinationBlob);

			size_t best_guess_index = 256;
			float best_guess_distance = 10000000;
			struct xrt_vec2 best_difference = {0};

			struct xrt_vec2 led_pos = destinationBlob->center;
			struct xrt_rect led_bounds = destinationBlob->bounding_box;

			for (size_t guess = 0; guess < bw->swapbuffer[i].num_previous; guess++) {
				if (already_mapped[guess]) {
					continue;
				}

				struct xrt_vec2 guess_pos = bw->swapbuffer[i].previous[guess].center;
				struct xrt_vec2 difference = m_vec2_sub(led_pos, guess_pos);


				struct xrt_rect guess_bounds = bw->swapbuffer[i].previous[guess].bounding_box;
				bool overlapping = rect_overlaps(led_bounds, guess_bounds);

				float distance = m_vec2_len(difference);

				if (best_guess_distance < difference.y) {
					// all further points are now too far away to possibly be a better match
					break;
				}

				if (overlapping) {
					// if (best_guess_distance > distance) {
					best_guess_index = guess;
					best_guess_distance = distance;
					best_difference = difference;
					break;
				}
			}

			if (best_guess_index == 256) {
				// No valid best guess, register a new ID to use
				destinationBlob->blob_id = bw->unused_id++;
			} else {
				// We found a valid guess, pull its metadata from last frame.
				already_mapped[best_guess_index] = true;
				destinationBlob->blob_id = bw->swapbuffer[i].previous[best_guess_index].blob_id;
				destinationBlob->matched_device_id =
				    bw->swapbuffer[i].previous[best_guess_index].matched_device_id;
				destinationBlob->motion_vector =
				    best_difference; // Not yet confirmed if I have this in the right direction
			}
		}

		struct t_blob_observation xbo = {
		    .source = &bw->base,
		    .id = i, // For this implementation, observation ID correlates to which camera detected the LED.
		    // Timestamp is in microseconds, needs conversion to nanoseconds
		    // TODO: alignment to the global time?
		    .timestamp_ns = payload->header.timestamp * 1000,
		    .blobs = bw->swapbuffer[i].scratch,
		    .num_blobs = camera->num_leds,
		};
		bw->blob_sink[i]->push_blobs(bw->blob_sink[i], &xbo);
		psvr2_blobwatch_toggle_swapbuffer(&bw->swapbuffer[i], camera->num_leds);
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

	for (size_t i = 0; i < PSVR2_MOSAIC_COUNT; i++) {
		memset(bw->swapbuffer[i].blobBuffA, 0, sizeof(bw->swapbuffer[i].blobBuffA));
		memset(bw->swapbuffer[i].blobBuffB, 0, sizeof(bw->swapbuffer[i].blobBuffB));
		bw->swapbuffer[i].scratch = (struct t_blob *)&bw->swapbuffer[i].blobBuffA;
		bw->swapbuffer[i].previous = (struct t_blob *)&bw->swapbuffer[i].blobBuffB;
		bw->swapbuffer[i].num_previous = 0;
	}

	bw->base.mark_blob_device = psvr2_blobwatch_mark_blob_device;
	bw->base.destroy = psvr2_blobwatch_destroy;
	for (size_t i = 0; i < PSVR2_MOSAIC_COUNT; i++) {
		bw->blob_sink[i] = blob_sink && blob_sink[i] ? blob_sink[i] : &stub_blob_sink;
	}

	*out_blobwatch = bw;
}
