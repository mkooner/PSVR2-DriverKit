// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Not implemented function helpers for data sources.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup aux_util
 */

#include "util/u_data_source_ni.h"
#include "util/u_logging.h"


/*
 *
 * Not implemented function helpers.
 *
 */

#define E(FN) U_LOG_E("Function " #FN " is not implemented for '%s'", xds->name)

xrt_result_t
u_data_source_ni_enumerate_frame_modes(struct xrt_ds *xds, struct xrt_ds_frame_mode **out_modes, uint32_t *out_count)
{
	E(enumerate_frame_modes);
	return XRT_ERROR_NOT_IMPLEMENTED;
}

xrt_result_t
u_data_source_ni_get_tracking_capabilities(struct xrt_ds *xds, struct xrt_ds_tracking_capabilities *out_capabilities)
{
	E(get_tracking_capabilities);
	return XRT_ERROR_NOT_IMPLEMENTED;
}

xrt_result_t
u_data_source_ni_configure_capture(struct xrt_ds *xds, struct xrt_ds_frame_capture_parameters *cp)
{
	E(configure_capture);
	return XRT_ERROR_NOT_IMPLEMENTED;
}

xrt_result_t
u_data_source_ni_frame_stream_start(struct xrt_ds *xds,
                                    struct xrt_frame_sink *xs,
                                    enum xrt_ds_capture_type capture_type,
                                    uint32_t frame_mode_index)
{
	E(frame_stream_start);
	return XRT_ERROR_NOT_IMPLEMENTED;
}

xrt_result_t
u_data_source_ni_tracking_streams_start(struct xrt_ds *xds, struct xrt_tracking_sinks *sinks)
{
	E(tracking_streams_start);
	return XRT_ERROR_NOT_IMPLEMENTED;
}

xrt_result_t
u_data_source_ni_stream_stop(struct xrt_ds *xds)
{
	E(stream_stop);
	return XRT_ERROR_NOT_IMPLEMENTED;
}

xrt_result_t
u_data_source_ni_is_running(struct xrt_ds *xds, bool *out_is_running)
{
	E(is_running);
	return XRT_ERROR_NOT_IMPLEMENTED;
}
