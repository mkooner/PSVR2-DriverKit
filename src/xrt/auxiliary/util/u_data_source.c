// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Misc helpers for @ref xrt_ds implementations.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup aux_util
 */

#include "u_data_source.h"
#include "u_data_source_ni.h"

#include <assert.h>


void
u_data_source_populate_function_pointers(struct xrt_ds *xds)
{
	assert(xds != NULL);

	xds->enumerate_frame_modes = u_data_source_ni_enumerate_frame_modes;
	xds->configure_capture = u_data_source_ni_configure_capture;
	xds->frame_stream_start = u_data_source_ni_frame_stream_start;
	xds->tracking_streams_start = u_data_source_ni_tracking_streams_start;
	xds->stream_stop = u_data_source_ni_stream_stop;
	xds->is_running = u_data_source_ni_is_running;
}
