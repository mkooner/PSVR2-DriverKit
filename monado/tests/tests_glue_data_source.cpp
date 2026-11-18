// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Tests for glue/g_data_source.hpp and glue/g_frame_node.hpp
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 */

#include "glue/g_data_source.hpp"
#include "glue/g_frame_node.hpp"

#include "catch_amalgamated.hpp"

#include <stdexcept>


//! Test data source with minimal functions
constexpr xrt::util::DataSourceFunctions kMinimalFunctions = {};

//! Test data source with everything enabled
constexpr xrt::util::DataSourceFunctions kFullFunctions = {
    .tracking_streams = true,
};

//! Test data source with tracking streams enabled, used to test exception handling
constexpr xrt::util::DataSourceFunctions kThrowingFunctions = {
    .tracking_streams = true,
};


/*!
 * Minimal test data source, also a frame node.
 */
class MinimalTestDataSource : public xrt::util::DataSourceBase<MinimalTestDataSource, kMinimalFunctions>,
                              public xrt::util::FrameNodeBase<MinimalTestDataSource>
{
public:
	bool running = false;
	bool broken_apart = false;
	bool destroyed = false;
	int enumerate_frame_modes_count = 0;
	int configure_capture_count = 0;
	xrt_frame_sink *sink = nullptr;
	xrt_ds_capture_type capture_type = XRT_DS_CAPTURE_TYPE_TRACKING;
	uint32_t frame_mode_index = 0;

	xrt_result_t
	enumerateFrameModes(struct xrt_ds_frame_mode **out_modes, uint32_t *out_count)
	{
		enumerate_frame_modes_count++;
		*out_modes = nullptr;
		*out_count = 0;
		return XRT_SUCCESS;
	}

	xrt_result_t
	configureCapture(struct xrt_ds_frame_capture_parameters *cp)
	{
		configure_capture_count++;
		return XRT_SUCCESS;
	}

	xrt_result_t
	frameStreamStart(struct xrt_frame_sink *xs, enum xrt_ds_capture_type type, uint32_t index)
	{
		sink = xs;
		capture_type = type;
		frame_mode_index = index;
		running = true;
		return XRT_SUCCESS;
	}

	xrt_result_t
	streamStop()
	{
		running = false;
		return XRT_SUCCESS;
	}

	xrt_result_t
	isRunning(bool &is_running)
	{
		is_running = this->running;
		return XRT_SUCCESS;
	}

	void
	breakApart()
	{
		broken_apart = true;
	}

	static void
	destroyFrameNode(struct xrt_frame_node *node)
	{
		auto *ds = MinimalTestDataSource::fromFrameNode(node);
		ds->destroyed = true;
	}
};

/*!
 * SLAM test data source.
 */
class SlamTestDataSource : public xrt::util::DataSourceBase<SlamTestDataSource, kFullFunctions>
{
public:
	int get_tracking_capabilities_count = 0;
	int tracking_streams_start_count = 0;

	xrt_result_t
	enumerateFrameModes(struct xrt_ds_frame_mode **out_modes, uint32_t *out_count)
	{
		return XRT_ERROR_NOT_IMPLEMENTED;
	}

	xrt_result_t
	getTrackingCapabilities(struct xrt_ds_tracking_capabilities *out_capabilities)
	{
		get_tracking_capabilities_count++;
		out_capabilities->cam_count = 2;
		out_capabilities->imu_count = 1;
		return XRT_SUCCESS;
	}

	xrt_result_t
	configureCapture(struct xrt_ds_frame_capture_parameters *cp)
	{
		return XRT_ERROR_NOT_IMPLEMENTED;
	}

	xrt_result_t
	frameStreamStart(struct xrt_frame_sink *xs, enum xrt_ds_capture_type type, uint32_t index)
	{
		return XRT_ERROR_NOT_IMPLEMENTED;
	}

	xrt_result_t
	trackingStreamsStart(struct xrt_tracking_sinks *sinks)
	{
		tracking_streams_start_count++;
		return XRT_SUCCESS;
	}

	xrt_result_t
	streamStop()
	{
		return XRT_ERROR_NOT_IMPLEMENTED;
	}

	xrt_result_t
	isRunning(bool &is_running)
	{
		return XRT_ERROR_NOT_IMPLEMENTED;
	}
};

/*!
 * Data source that throws from every callback.
 */
class ThrowingTestDataSource : public xrt::util::DataSourceBase<ThrowingTestDataSource, kThrowingFunctions>
{
public:
	xrt_result_t
	enumerateFrameModes(struct xrt_ds_frame_mode **out_modes, uint32_t *out_count)
	{
		throw std::runtime_error("enumerateModes");
	}

	xrt_result_t
	getTrackingCapabilities(struct xrt_ds_tracking_capabilities *out_capabilities)
	{
		throw std::runtime_error("getTrackingCapabilities");
	}

	xrt_result_t
	configureCapture(struct xrt_ds_frame_capture_parameters *cp)
	{
		throw std::runtime_error("configureCapture");
	}

	xrt_result_t
	frameStreamStart(struct xrt_frame_sink *xs, enum xrt_ds_capture_type type, uint32_t index)
	{
		throw std::runtime_error("streamStart");
	}

	xrt_result_t
	trackingStreamsStart(struct xrt_tracking_sinks *sinks)
	{
		throw std::runtime_error("trackingStreamsStart");
	}

	xrt_result_t
	streamStop()
	{
		throw std::runtime_error("streamStop");
	}

	xrt_result_t
	isRunning(bool &is_running)
	{
		throw std::runtime_error("isRunning");
	}
};


TEST_CASE("DataSourceBase_Minimal")
{
	MinimalTestDataSource ds;
	auto *xds = ds.getXDS();

	SECTION("Function pointers are set")
	{
		REQUIRE(xds->enumerate_frame_modes != nullptr);
		REQUIRE(xds->configure_capture != nullptr);
		REQUIRE(xds->frame_stream_start != nullptr);
		REQUIRE(xds->tracking_streams_start != nullptr);
		REQUIRE(xds->stream_stop != nullptr);
		REQUIRE(xds->is_running != nullptr);
	}

	SECTION("Disabled optional functions fail cleanly")
	{
		xrt_ds_tracking_capabilities capabilities = {};
		REQUIRE(xds->tracking_streams_start(xds, nullptr) == XRT_ERROR_NOT_IMPLEMENTED);
		REQUIRE(xds->get_tracking_capabilities(xds, &capabilities) == XRT_ERROR_NOT_IMPLEMENTED);
	}

	SECTION("Identifying fields are zeroed")
	{
		REQUIRE(xds->name[0] == '\0');
		REQUIRE(xds->product[0] == '\0');
		REQUIRE(xds->manufacturer[0] == '\0');
		REQUIRE(xds->serial[0] == '\0');
		REQUIRE(xds->source_id == 0);
	}

	SECTION("fromXDS conversion works")
	{
		REQUIRE(MinimalTestDataSource::fromXDS(xds) == &ds);
	}

	SECTION("Functions can be called")
	{
		xrt_ds_frame_mode *modes = nullptr;
		uint32_t count = 1;
		REQUIRE(xrt_ds_enumerate_frame_modes(xds, &modes, &count) == XRT_SUCCESS);
		REQUIRE(ds.enumerate_frame_modes_count == 1);
		REQUIRE(count == 0);

		xrt_ds_frame_capture_parameters cp = {};
		REQUIRE(xrt_ds_configure_capture(xds, &cp) == XRT_SUCCESS);
		REQUIRE(ds.configure_capture_count == 1);

		xrt_frame_sink sink = {};
		bool running;
		REQUIRE(xrt_ds_is_running(xds, &running) == XRT_SUCCESS);
		REQUIRE_FALSE(running);
		REQUIRE(xrt_ds_frame_stream_start(xds, &sink, XRT_DS_CAPTURE_TYPE_CALIBRATION, 3) == XRT_SUCCESS);
		REQUIRE(xrt_ds_is_running(xds, &running) == XRT_SUCCESS);
		REQUIRE(running);
		REQUIRE(ds.sink == &sink);
		REQUIRE(ds.capture_type == XRT_DS_CAPTURE_TYPE_CALIBRATION);
		REQUIRE(ds.frame_mode_index == 3);

		REQUIRE(xrt_ds_stream_stop(xds) == XRT_SUCCESS);
		REQUIRE(xrt_ds_is_running(xds, &running) == XRT_SUCCESS);
		REQUIRE_FALSE(running);
	}
}

TEST_CASE("DataSourceBase_Slam")
{
	SlamTestDataSource ds;
	auto *xds = ds.getXDS();

	REQUIRE(xds->tracking_streams_start != nullptr);
	REQUIRE(xrt_ds_tracking_streams_start(xds, nullptr) == XRT_SUCCESS);
	REQUIRE(ds.tracking_streams_start_count == 1);

	REQUIRE(xds->get_tracking_capabilities != nullptr);
	xrt_ds_tracking_capabilities capabilities = {};
	REQUIRE(xrt_ds_get_tracking_capabilities(xds, &capabilities) == XRT_SUCCESS);
	REQUIRE(ds.get_tracking_capabilities_count == 1);
	REQUIRE(capabilities.cam_count == 2);
	REQUIRE(capabilities.imu_count == 1);
}

TEST_CASE("DataSourceBase_Exceptions")
{
	ThrowingTestDataSource ds;
	auto *xds = ds.getXDS();

	xrt_ds_frame_mode *modes = nullptr;
	uint32_t count = 0;
	xrt_ds_frame_capture_parameters cp = {};
	xrt_ds_tracking_capabilities capabilities = {};

	bool running;
	REQUIRE(xrt_ds_enumerate_frame_modes(xds, &modes, &count) == XRT_ERROR_UNCAUGHT_EXCEPTION);
	REQUIRE(xrt_ds_get_tracking_capabilities(xds, &capabilities) == XRT_ERROR_UNCAUGHT_EXCEPTION);
	REQUIRE(xrt_ds_configure_capture(xds, &cp) == XRT_ERROR_UNCAUGHT_EXCEPTION);
	REQUIRE(xrt_ds_frame_stream_start(xds, nullptr, XRT_DS_CAPTURE_TYPE_TRACKING, 0) ==
	        XRT_ERROR_UNCAUGHT_EXCEPTION);
	REQUIRE(xrt_ds_tracking_streams_start(xds, nullptr) == XRT_ERROR_UNCAUGHT_EXCEPTION);
	REQUIRE(xrt_ds_stream_stop(xds) == XRT_ERROR_UNCAUGHT_EXCEPTION);
	REQUIRE(xrt_ds_is_running(xds, &running) == XRT_ERROR_UNCAUGHT_EXCEPTION);
}

TEST_CASE("FrameNodeBase_WithDataSource")
{
	MinimalTestDataSource ds;
	auto *node = ds.getFrameNode();

	SECTION("Function pointers are set")
	{
		REQUIRE(node->break_apart != nullptr);
		REQUIRE(node->destroy != nullptr);
	}

	SECTION("Both bases recover the same object")
	{
		REQUIRE(MinimalTestDataSource::fromFrameNode(node) == &ds);
		REQUIRE(MinimalTestDataSource::fromXDS(ds.getXDS()) == &ds);
	}

	SECTION("Frame context calls break apart then destroy")
	{
		xrt_frame_context xfctx = {};
		xrt_frame_context_add(&xfctx, node);

		REQUIRE_FALSE(ds.broken_apart);
		REQUIRE_FALSE(ds.destroyed);

		xrt_frame_context_destroy_nodes(&xfctx);

		REQUIRE(ds.broken_apart);
		REQUIRE(ds.destroyed);
		REQUIRE(xfctx.nodes == nullptr);
	}
}
