// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Internal structures for the realsense data source.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_realsense
 */

#pragma once

#include "xrt/xrt_data_source.h"
#include "xrt/xrt_frame.h"

#include "tracking/t_time_sync.h"
#include "tracking/t_tracking.h"

#include "math/m_clock_tracking.h"

#include "os/os_threading.h"

#include "util/u_sink.h"

#include "glue/g_data_source.hpp"
#include "glue/g_frame_node.hpp"

#include <librealsense2/rs.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>


namespace xrt::drivers::realsense {

namespace os = xrt::auxiliary::os;

// Forward-declares
struct DataSource;

struct Sensor;
struct FrameSensor;

/*!
 * Maps the device's hardware clock to the local monotonic clock.
 *
 * All sensors on a device are stamped from the same hardware clock, so they all feed
 * and query one shared tracker. Sensors call in from separate realsense threads.
 *
 * The tracker follows the earliest arrival of samples, so on its own it is off by the
 * arrival latency of whichever stream arrives soonest, plus its own smoothing lag.
 * @ref Device's probe thread reads the device clock directly to measure that error,
 * which @ref toLocal then corrects for.
 */
struct DeviceClock
{
public: // Types
	//! A bracket on the tracker's error (tracker skew minus the true clock offset) from one probe.
	struct ProbeBracket
	{
		time_duration_ns lo;
		time_duration_ns hi;
	};

	//! How many probe brackets are intersected for the correction.
	static constexpr size_t probe_window_size = 64;

public: // Fields
	m_clock_windowed_skew_tracker *tracker;
	mutable os::Mutex lock;

	//! Unwraps this device's timestamps into nanoseconds.
	m_clock_bit_extender clock_extender;

	//! Ring buffer of the most recent probe brackets.
	ProbeBracket probe_window[probe_window_size];
	size_t probe_window_pos{0};
	size_t probe_window_count{0};
	//! Intersection of all the brackets in the window.
	ProbeBracket probe_intersection{};

	//! The measured tracker error, subtracted from the tracker's estimate in @ref toLocal.
	time_duration_ns correction_ns{0};
	bool have_correction{false};
	//! Whether to apply @ref correction_ns, toggleable for comparison.
	bool apply_correction{true};

	//! Debug: width of @ref probe_intersection, how precisely the correction is known.
	time_duration_ns correction_bracket_ns{0};
	//! Debug: round trip time of the last probe.
	time_duration_ns last_probe_rtt_ns{0};

public: // Methods
	DeviceClock();

	~DeviceClock();

	DeviceClock(const DeviceClock &) = delete;
	DeviceClock(DeviceClock &&) = delete;
	DeviceClock &
	operator=(const DeviceClock &) = delete;
	DeviceClock &
	operator=(DeviceClock &&) = delete;

	/*!
	 * Adds a sample of a device timestamp that was observed at the local time @p local_ns.
	 *
	 * @return The device time in nanoseconds
	 */
	timepoint_ns
	push(timepoint_ns local_ns, uint32_t device_us);

	//! Converts a device timestamp to local time, returns false if there is not enough data yet.
	bool
	toLocal(timepoint_ns device_ns, timepoint_ns *out_local_ns);

	/*!
	 * Adds a direct read of the device clock, which happened somewhere between the local times @p t0_ns and
	 * @p t1_ns, and updates the correction from it.
	 */
	void
	addProbe(timepoint_ns t0_ns, timepoint_ns t1_ns, uint32_t device_us);
};

/*!
 * A physical realsense device, owning the realsense context and all of the sensors on it.
 *
 * A single device (e.g. a D435i) may expose multiple sensors of different kinds,
 * like a stereo IR frame sensor, an RGB frame sensor and an IMU sensor.
 */
struct Device
{
public: // Fields
	rs2::context context;
	//! The underlying realsense device
	rs2::device device;

	//! Product name of the device.
	std::string name;
	//! Serial number of the device.
	std::string serial;

	//! Clock shared by all of the sensors on this device.
	DeviceClock clock;

	//! All of the sensors on this device that we expose.
	std::vector<std::shared_ptr<Sensor>> sensors;

	//! Thread periodically reading the device clock to measure @ref clock's error.
	struct os_thread_helper probe_thread;
	//! How many of our sensors are streaming, protected by @ref probe_thread's lock.
	uint32_t streaming_sensor_count{0};

public: // Methods
	Device(const rs2::context &context, const rs2::device &device);

	~Device();

	// Sensors hold a pointer back to us, so we must be stable.
	Device(const Device &) = delete;
	Device(Device &&) = delete;
	Device &
	operator=(const Device &) = delete;
	Device &
	operator=(Device &&) = delete;

	//! Gets all the frame sensors on this device.
	std::vector<std::shared_ptr<FrameSensor>>
	frameSensors() const;

	//! Starts the clock probe thread, it only probes while a sensor is streaming.
	bool
	startClockProbe();

	//! Called by a sensor when it starts streaming.
	void
	sensorStarted();

	//! Called by a sensor when it stops streaming.
	void
	sensorStopped();

	//! Stops the clock probe thread, then closes and releases all of our sensors.
	void
	close();

private:
	void
	runClockProbe();

	static void *
	clockProbeThread(void *ptr);
};

/*!
 * Base class of all sensors on a @ref Device, handles refcounted starting/stopping of the underlying sensor.
 */
struct Sensor
{
public: // Fields
	//! The device that owns us, outlives us as it is only released after @ref Device::close.
	std::shared_ptr<Device> device;
	//! The underlying realsense sensor on the device.
	rs2::sensor sensor;

	mutable os::Mutex lock;
	uint32_t refcount{0};

public: // Methods
	Sensor(std::shared_ptr<Device> device, const rs2::sensor &sensor);

	virtual ~Sensor() = default;

	//! Called from the realsense thread for every frame the sensor produces.
	virtual void
	onFrame(rs2::frame &f) = 0;

	void
	refSensor();

	void
	unrefSensor();

	void
	close();
};

/*!
 * A sensor producing video frames, which are dispatched to one @ref DataSource per stream.
 */
struct FrameSensor : public Sensor
{
public: // Fields
	//! The individual data sources, one per stream on this sensor.
	std::vector<DataSource *> data_sources;

public: // Methods
	FrameSensor(std::shared_ptr<Device> device, const rs2::sensor &sensor);

	void
	onFrame(rs2::frame &f) override;
};

struct DataSource : public xrt::util::DataSourceBase<DataSource, util::DataSourceFunctions{}>,
                    public xrt::util::FrameNodeBase<DataSource>
{
public: // Fields
	std::shared_ptr<FrameSensor> frame_sensor;

	struct t_timing_event_sink *timing_event_sink{nullptr};
	struct t_timing_event_source *timing_event_source{nullptr};

	int stream_index;
	mutable os::Mutex lock;
	xrt_ds_frame_mode mode;
	xrt_frame_sink *target{nullptr};

	rs2::video_stream_profile profile;

	u_sink_debug debug_sink{};

public: // Methods
	DataSource(xrt_frame_context *xfctx,
	           std::shared_ptr<FrameSensor> &frame_sensor,
	           const rs2::video_stream_profile &profile);

	~DataSource();

	// Delete all copy/move ctors, since the pointers for the data source and frame node need to be stable
	DataSource(const DataSource &) = delete;
	DataSource(DataSource &&) = delete;
	DataSource &
	operator=(const DataSource &) = delete;
	DataSource &
	operator=(DataSource &&) = delete;

	/*
	 *
	 * Internal methods
	 *
	 */

	void
	handleFrame(rs2::frame &f, timepoint_ns local_frame_timestamp_ns);

	t_camera_calibration
	cameraCalibration();

	/*
	 *
	 * Data source methods
	 *
	 */

	xrt_result_t
	enumerateFrameModes(struct xrt_ds_frame_mode **out_modes, uint32_t *out_count);

	xrt_result_t
	configureCapture(struct xrt_ds_frame_capture_parameters *cp);

	xrt_result_t
	frameStreamStart(struct xrt_frame_sink *xs, enum xrt_ds_capture_type capture_type, uint32_t frame_mode_index);

	xrt_result_t
	streamStop();

	xrt_result_t
	isRunning(bool &is_running);

	void
	breakApart();

	static void
	destroyFrameNode(struct xrt_frame_node *node)
	{
		delete fromFrameNode(node);
	}
};

struct Context : public xrt::util::FrameNodeBase<Context>
{
public: // Fields
	//! All the created devices
	std::vector<std::shared_ptr<Device>> devices;

public: // Methods
	Context() = default;

	//! Gets all the frame sensors across all devices.
	std::vector<std::shared_ptr<FrameSensor>>
	frameSensors() const;

	// Delete all copy/move ctors, since the pointers for the frame node need to be stable
	Context(const Context &) = delete;
	Context(Context &&) = delete;
	Context &
	operator=(const Context &) = delete;
	Context &
	operator=(Context &&) = delete;

	void
	breakApart();

	static void
	destroyFrameNode(struct xrt_frame_node *node)
	{
		delete fromFrameNode(node);
	}
};

} // namespace xrt::drivers::realsense
