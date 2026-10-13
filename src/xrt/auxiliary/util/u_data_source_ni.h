// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Not implemented function helpers for data sources.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup aux_util
 */

#pragma once

#include "xrt/xrt_data_source.h"

#ifdef __cplusplus
extern "C" {
#endif


/*
 *
 * Not implemented function helpers.
 *
 */

/*!
 * Not implemented function for @ref xrt_ds::enumerate_frame_modes.
 *
 * @ingroup aux_util
 */
xrt_result_t
u_data_source_ni_enumerate_frame_modes(struct xrt_ds *xds, struct xrt_ds_frame_mode **out_modes, uint32_t *out_count);

/*!
 * Not implemented function for @ref xrt_ds::configure_capture.
 *
 * @ingroup aux_util
 */
xrt_result_t
u_data_source_ni_configure_capture(struct xrt_ds *xds, struct xrt_ds_frame_capture_parameters *cp);

/*!
 * Not implemented function for @ref xrt_ds::frame_stream_start.
 *
 * @ingroup aux_util
 */
xrt_result_t
u_data_source_ni_frame_stream_start(struct xrt_ds *xds,
                                    struct xrt_frame_sink *xs,
                                    enum xrt_ds_capture_type capture_type,
                                    uint32_t frame_mode_index);

/*!
 * Not implemented function for @ref xrt_ds::tracking_streams_start.
 *
 * @ingroup aux_util
 */
xrt_result_t
u_data_source_ni_tracking_streams_start(struct xrt_ds *xds, struct xrt_tracking_sinks *sinks);

/*!
 * Not implemented function for @ref xrt_ds::stream_stop.
 *
 * @ingroup aux_util
 */
xrt_result_t
u_data_source_ni_stream_stop(struct xrt_ds *xds);

/*!
 * Not implemented function for @ref xrt_ds::is_running.
 *
 * @ingroup aux_util
 */
xrt_result_t
u_data_source_ni_is_running(struct xrt_ds *xds, bool *out_is_running);


#ifdef __cplusplus
}
#endif
