// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Header for glue classes to wrap xrt data source interfaces.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup aux_util
 */

#pragma once

#include "xrt/xrt_data_source.h"

#include "util/u_data_source.h"

#include "g_catch_guard.hpp"
#include "g_traits.hpp"

#include <type_traits>


namespace xrt::util {

/*!
 * Selects which optional functions that the @ref DataSourceBase wrapper
 * should set.
 */
struct DataSourceFunctions
{
	//! @ref xrt_ds::tracking_streams_start
	bool tracking_streams{false};
};

/*!
 * CRTP glue wrapper for @ref xrt_ds. Relies on standard layout to recover the
 * derived object from the C struct, and has some requirements and limitations
 * because of that. See @ref cpp-glue-wrappers for the guide and conventions
 * for these wrappers.
 *
 * @ref xrt_ds has no destroy function, a data source is instead destroyed
 * through a @ref xrt_frame_node added to a @ref xrt_frame_context, see
 * @ref FrameNodeBase.
 *
 * The identifying fields (name, product, manufacturer, serial and source_id)
 * are zeroed and are to be filled in by the derived class via @ref getXDS.
 */
template <class T, DataSourceFunctions functions> class DataSourceBase
{
public: // Methods
	DataSourceBase() noexcept
	{
		static_assert(std::is_standard_layout_v<DataSourceBase>,
		              "glue base must be standard layout for pointer recovery");
		static_assert(is_non_virtual_base_v<DataSourceBase, T>,
		              "glue base must be a non-virtual base of T for pointer recovery");

		auto &xds = *getXDS();

		u_data_source_populate_function_pointers(&xds);

		xds.enumerate_frame_modes = enumerateFrameModesWrap;
		xds.configure_capture = configureCaptureWrap;
		xds.frame_stream_start = frameStreamStartWrap;
		xds.stream_stop = streamStopWrap;
		xds.is_running = isRunningWrap;

		if constexpr (functions.tracking_streams) {
			xds.tracking_streams_start = trackingStreamsStartWrap;
		}
	}

	~DataSourceBase() noexcept = default;

	const T &
	derived() const noexcept
	{
		return static_cast<const T &>(*this);
	}

	T &
	derived() noexcept
	{
		return static_cast<T &>(*this);
	}

	static const T *
	fromXDS(const xrt_ds *xds) noexcept
	{
		return &(reinterpret_cast<const DataSourceBase *>(xds)->derived());
	}

	static T *
	fromXDS(xrt_ds *xds) noexcept
	{
		return &(reinterpret_cast<DataSourceBase *>(xds)->derived());
	}

	const xrt_ds *
	getXDS() const noexcept
	{
		return &mDataSource;
	}

	xrt_ds *
	getXDS() noexcept
	{
		return &mDataSource;
	}


private: // Members
	/*!
	 * Wrapped @ref xrt_ds. Must be the first data member: a pointer to it is
	 * then interconvertible with a pointer to this standard-layout base, which
	 * lets the glue cast a C pointer back to the derived C++ class. See
	 * @ref cpp-glue-wrappers.
	 */
	xrt_ds mDataSource = {};


private: // Functions
// Not `->derived()`, that is ambiguous when T has more than one glue base.
#define GET(xds) (*fromXDS(xds))

	static xrt_result_t
	enumerateFrameModesWrap(struct xrt_ds *xds, struct xrt_ds_frame_mode **out_modes, uint32_t *out_count) noexcept
	try {
		return GET(xds).enumerateFrameModes(out_modes, out_count);
	}
	G_CATCH_GUARDS_WITH_RETURN(XRT_ERROR_UNCAUGHT_EXCEPTION)

	static xrt_result_t
	configureCaptureWrap(struct xrt_ds *xds, struct xrt_ds_frame_capture_parameters *cp) noexcept
	try {
		return GET(xds).configureCapture(cp);
	}
	G_CATCH_GUARDS_WITH_RETURN(XRT_ERROR_UNCAUGHT_EXCEPTION)

	static xrt_result_t
	frameStreamStartWrap(struct xrt_ds *xds,
	                     struct xrt_frame_sink *xs,
	                     enum xrt_ds_capture_type capture_type,
	                     uint32_t frame_mode_index) noexcept
	try {
		return GET(xds).frameStreamStart(xs, capture_type, frame_mode_index);
	}
	G_CATCH_GUARDS_WITH_RETURN(XRT_ERROR_UNCAUGHT_EXCEPTION)

	static xrt_result_t
	trackingStreamsStartWrap(struct xrt_ds *xds, struct xrt_tracking_sinks *sinks) noexcept
	try {
		return GET(xds).trackingStreamsStart(sinks);
	}
	G_CATCH_GUARDS_WITH_RETURN(XRT_ERROR_UNCAUGHT_EXCEPTION)

	static xrt_result_t
	streamStopWrap(struct xrt_ds *xds) noexcept
	try {
		return GET(xds).streamStop();
	}
	G_CATCH_GUARDS_WITH_RETURN(XRT_ERROR_UNCAUGHT_EXCEPTION)

	static xrt_result_t
	isRunningWrap(struct xrt_ds *xds, bool *out_is_running) noexcept
	try {
		return GET(xds).isRunning(*out_is_running);
	}
	G_CATCH_GUARDS_WITH_RETURN(XRT_ERROR_UNCAUGHT_EXCEPTION)

#undef GET
};

} // namespace xrt::util
