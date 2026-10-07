// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Realsense data source, tailored for use with the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_realsense
 *
 * Tested against an 435i, but should work with any realsense camera that
 * has an IR image sensor.
 */

#include "tracking/t_tracking.h"

#include "util/u_misc.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"

#include "os/os_time.h"

#include "b_timing_source.h"

#include "rs_interface.h"
#include "rs_ds.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>


namespace xrt::drivers::realsense {

//! How often the device clock is probed while streaming.
constexpr time_duration_ns probe_interval_ns = 50 * U_TIME_1MS_IN_NS;

/*!
 * Probes with a longer round trip are dropped.
 *
 * Round trips are usually ~200us, with a tail out to ~10ms. The slow ones bound the clock
 * offset too loosely to tighten the intersection, and would make a poor first estimate.
 */
constexpr time_duration_ns max_probe_rtt_ns = 1 * U_TIME_1MS_IN_NS;

/*!
 * New estimates further than this from the current correction are jumped to instead of eased towards.
 *
 * Covers the first estimate, and a different stream starting to dominate the tracker (~8ms).
 */
constexpr time_duration_ns correction_snap_ns = 1 * U_TIME_1MS_IN_NS;

//! The device is assumed to not support reading its clock after this many failures in a row.
constexpr uint32_t max_probe_failures_in_row = 10;

/*
 *
 * DeviceClock implementations
 *
 */

DeviceClock::DeviceClock() : lock(), clock_extender()
{
	this->tracker = m_clock_windowed_skew_tracker_alloc(128);
}

DeviceClock::~DeviceClock()
{
	if (this->tracker) {
		m_clock_windowed_skew_tracker_destroy(this->tracker);
	}
}

timepoint_ns
DeviceClock::push(timepoint_ns local_ns, uint32_t device_us)
{
	std::unique_lock<os::Mutex> lock(this->lock);

	timepoint_ns delta = m_clock_bit_extender_update(&this->clock_extender, device_us, U_TIME_1US_IN_NS);

	timepoint_ns device_ns = this->clock_extender.clock;
	if (delta != 0) {
		m_clock_windowed_skew_tracker_push(this->tracker, local_ns, device_ns);
	}
	return device_ns;
}

bool
DeviceClock::toLocal(timepoint_ns device_ns, timepoint_ns *out_local_ns)
{
	std::unique_lock<os::Mutex> lock(this->lock);
	if (!m_clock_windowed_skew_tracker_to_local(this->tracker, device_ns, out_local_ns)) {
		return false;
	}

	// Without a correction (not probed yet, or unsupported) this stays the earliest arrival time.
	if (this->have_correction && this->apply_correction) {
		*out_local_ns -= this->correction_ns;
	}
	return true;
}

void
DeviceClock::addProbe(timepoint_ns t0_ns, timepoint_ns t1_ns, uint32_t device_us)
{
	std::unique_lock<os::Mutex> lock(this->lock);

	this->last_probe_rtt_ns = t1_ns - t0_ns;
	if (this->last_probe_rtt_ns > max_probe_rtt_ns) {
		return;
	}

	// No samples have been pushed yet, so there is no estimate to measure.
	time_duration_ns skew_ns;
	if (!m_clock_windowed_skew_tracker_get_skew(this->tracker, &skew_ns)) {
		return;
	}

	// Put the probe on the samples' timeline, without moving the extender for the samples.
	const timepoint_ns device_ns = m_clock_bit_extender_peek(&this->clock_extender, device_us, U_TIME_1US_IN_NS);

	// The clock was read somewhere within [t0, t1], and truncated to whole microseconds, so the true offset
	// (local = device + offset) is within [t0 - device - 1us, t1 - device]. The tracker estimates
	// local = device + skew, so its error (skew - offset) is within:
	const ProbeBracket bracket = {
	    .lo = skew_ns - (t1_ns - device_ns),
	    .hi = skew_ns - (t0_ns - device_ns - U_TIME_1US_IN_NS),
	};

	// The error stays constant while the same streams feed the tracker, so every bracket should overlap.
	// One that doesn't means the error changed (tracker reset, or another stream now dominates it), start over.
	if (this->probe_window_count > 0 &&
	    (bracket.hi < this->probe_intersection.lo || bracket.lo > this->probe_intersection.hi)) {
		this->probe_window_count = 0;
	}

	this->probe_window[this->probe_window_pos] = bracket;
	this->probe_window_pos = (this->probe_window_pos + 1) % probe_window_size;
	this->probe_window_count = std::min(this->probe_window_count + 1, probe_window_size);

	// Every bracket holds the error, so it's within their intersection.
	ProbeBracket intersection = {
	    .lo = std::numeric_limits<time_duration_ns>::min(),
	    .hi = std::numeric_limits<time_duration_ns>::max(),
	};
	for (size_t i = 0; i < this->probe_window_count; i++) {
		const size_t index = (this->probe_window_pos + probe_window_size - 1 - i) % probe_window_size;
		intersection.lo = std::max(intersection.lo, this->probe_window[index].lo);
		intersection.hi = std::min(intersection.hi, this->probe_window[index].hi);
	}
	this->probe_intersection = intersection;
	this->correction_bracket_ns = intersection.hi - intersection.lo;

	const time_duration_ns estimate_ns = intersection.lo + (intersection.hi - intersection.lo) / 2;
	const time_duration_ns difference_ns = estimate_ns - this->correction_ns;
	if (!this->have_correction || difference_ns > correction_snap_ns || difference_ns < -correction_snap_ns) {
		this->correction_ns = estimate_ns;
		this->have_correction = true;
	} else {
		// Ease in, so the estimate's small steps as the window changes don't show up as timestamp jitter.
		this->correction_ns += difference_ns / 16;
	}
}

/*
 *
 * Device implementations
 *
 */

Device::Device(const rs2::context &context, const rs2::device &device) : context(context), device(device)
{
	this->name = "Realsense";
	if (device.supports(RS2_CAMERA_INFO_NAME)) {
		this->name = device.get_info(RS2_CAMERA_INFO_NAME);
	}

	this->serial = "N/A";
	if (device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
		this->serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	}

	if (os_thread_helper_init(&this->probe_thread) != 0) {
		throw std::runtime_error("Failed to init clock probe thread helper");
	}

	u_var_add_root(this, "Realsense Device", true);
	u_var_add_ro_text(this, this->serial.c_str(), "Serial");
	u_var_add_bool(this, &this->clock.apply_correction, "Apply latency correction");
	u_var_add_ro_i64_ns(this, &this->clock.correction_ns, "Latency correction");
	u_var_add_ro_i64_ns(this, &this->clock.correction_bracket_ns, "Latency correction bracket");
	u_var_add_ro_i64_ns(this, &this->clock.last_probe_rtt_ns, "Last clock probe round trip");
}

Device::~Device()
{
	u_var_remove_root(this);

	// Stops the thread if it's still running.
	os_thread_helper_destroy(&this->probe_thread);
}

bool
Device::startClockProbe()
{
	return os_thread_helper_start(&this->probe_thread, clockProbeThread, this) == 0;
}

void
Device::sensorStarted()
{
	os_thread_helper_lock(&this->probe_thread);
	this->streaming_sensor_count++;
	os_thread_helper_signal_locked(&this->probe_thread);
	os_thread_helper_unlock(&this->probe_thread);
}

void
Device::sensorStopped()
{
	os_thread_helper_lock(&this->probe_thread);
	assert(this->streaming_sensor_count > 0);
	this->streaming_sensor_count--;
	os_thread_helper_unlock(&this->probe_thread);
}

void *
Device::clockProbeThread(void *ptr)
{
	static_cast<Device *>(ptr)->runClockProbe();
	return nullptr;
}

void
Device::runClockProbe()
{
	const char *thread_name = "Realsense clock probe";
	U_TRACE_SET_THREAD_NAME(thread_name);
	os_thread_name_self(thread_name);

	uint32_t failures_in_row = 0;

	os_thread_helper_lock(&this->probe_thread);
	while (os_thread_helper_is_running_locked(&this->probe_thread)) {
		// Nothing feeds the tracker while no sensor streams, so there would be nothing to compare against.
		if (this->streaming_sensor_count == 0) {
			os_thread_helper_wait_locked(&this->probe_thread);
			continue;
		}
		os_thread_helper_unlock(&this->probe_thread);

		try {
			const timepoint_ns t0_ns = os_monotonic_get_ns();
			const double device_ms = this->device.get_device_time_ms();
			const timepoint_ns t1_ns = os_monotonic_get_ns();

			// The device clock is a 32-bit microsecond counter, which librealsense returns in milliseconds.
			this->clock.addProbe(t0_ns, t1_ns, static_cast<uint32_t>(std::llround(device_ms * 1000.0)));
			failures_in_row = 0;
		} catch (const rs2::error &e) {
			if (++failures_in_row >= max_probe_failures_in_row) {
				U_LOG_W(
				    "Failed to read the clock of %s %s %u times in a row, timestamps will not be "
				    "latency corrected: %s",
				    this->name.c_str(), this->serial.c_str(), failures_in_row, e.what());
				break;
			}
		}

		os_nanosleep(probe_interval_ns);
		os_thread_helper_lock(&this->probe_thread);
	}

	if (failures_in_row >= max_probe_failures_in_row) {
		// Gave up, idle until stopped so the thread is still joined normally.
		os_thread_helper_lock(&this->probe_thread);
		while (os_thread_helper_is_running_locked(&this->probe_thread)) {
			os_thread_helper_wait_locked(&this->probe_thread);
		}
	}
	os_thread_helper_unlock(&this->probe_thread);
}

std::vector<std::shared_ptr<FrameSensor>>
Device::frameSensors() const
{
	std::vector<std::shared_ptr<FrameSensor>> frame_sensors;
	for (const auto &sensor : this->sensors) {
		if (auto frame_sensor = std::dynamic_pointer_cast<FrameSensor>(sensor)) {
			frame_sensors.push_back(std::move(frame_sensor));
		}
	}
	return frame_sensors;
}

void
Device::close()
{
	// Stop probing first, so it isn't talking to the device while the sensors close.
	os_thread_helper_stop_and_wait(&this->probe_thread);

	for (auto &sensor : this->sensors) {
		sensor->close();
	}

	// Sensors hold a reference to us, so drop them to break the cycle.
	this->sensors.clear();
}

/*
 *
 * Sensor implementations
 *
 */

Sensor::Sensor(std::shared_ptr<Device> device, const rs2::sensor &sensor)
    : device(std::move(device)), sensor(sensor), lock()
{}

void
Sensor::refSensor()
{
	std::unique_lock<os::Mutex> lock(this->lock);

	if (refcount++ == 0) {
		this->sensor.start([this](rs2::frame f) { this->onFrame(f); });
		this->device->sensorStarted();
	}
}

void
Sensor::unrefSensor()
{
	std::unique_lock<os::Mutex> lock(this->lock);

	if (--refcount == 0) {
		this->sensor.stop();
		this->device->sensorStopped();
	}
}

void
Sensor::close()
{
	std::unique_lock<os::Mutex> lock(this->lock);

	// Stopping a sensor that isn't streaming throws.
	if (this->refcount > 0) {
		this->sensor.stop();
		this->refcount = 0;
	}
	this->sensor.close();
}

/*
 *
 * FrameSensor implementations
 *
 */

FrameSensor::FrameSensor(std::shared_ptr<Device> device, const rs2::sensor &sensor) : Sensor(std::move(device), sensor)
{}

void
FrameSensor::onFrame(rs2::frame &f)
{
	timepoint_ns now_ns = os_monotonic_get_ns();

	const auto timestamp_domain = f.get_frame_timestamp_domain();
	if (timestamp_domain != RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK) {
		U_LOG_E("Unsupported timestamp domain: %s", rs2_timestamp_domain_to_string(timestamp_domain));
		return;
	}

	uint32_t device_timestamp_us = static_cast<uint32_t>(f.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP));

	timepoint_ns device_timestamp_ns = this->device->clock.push(now_ns, device_timestamp_us);

	timepoint_ns local_frame_timestamp_ns;
	if (!this->device->clock.toLocal(device_timestamp_ns, &local_frame_timestamp_ns)) {
		return;
	}

	for (auto data_source : this->data_sources) {
		data_source->handleFrame(f, local_frame_timestamp_ns);
	}
}

/*
 *
 * DataSource implementations
 *
 */

DataSource::DataSource(xrt_frame_context *xfctx,
                       std::shared_ptr<FrameSensor> &frame_sensor,
                       const rs2::video_stream_profile &profile)
    : frame_sensor(frame_sensor), stream_index(profile.stream_index()), lock(), profile(profile)
{
	auto &xds = *getXDS();

	this->mode = {
	    .width = static_cast<uint32_t>(profile.width()),
	    .height = static_cast<uint32_t>(profile.height()),
	    .format = XRT_FORMAT_L8,
	    .stereo_format = XRT_STEREO_FORMAT_NONE,
	};

	snprintf(xds.name, sizeof(xds.name), "Realsense Data Source");

	const Device &device = *frame_sensor->device;
	snprintf(xds.product, sizeof(xds.product), "%s", device.name.c_str());
	snprintf(xds.manufacturer, sizeof(xds.manufacturer), "%s", device.name.c_str());
	snprintf(xds.serial, sizeof(xds.serial), "%s", device.serial.c_str());

	xds.source_id = this->stream_index;

	u_sink_debug_init(&this->debug_sink);

	u_var_add_root(this, "Realsense Data Source", true);
	u_var_add_sink_debug(this, &this->debug_sink, "Debug sink");

	// @todo check errors
	b_timing_source_create(xfctx, &this->timing_event_sink, &this->timing_event_source);
}

DataSource::~DataSource()
{
	// Must be removed before the debug sink is destroyed, otherwise u_var visitors will touch a dead sink.
	u_var_remove_root(this);

	u_sink_debug_destroy(&this->debug_sink);
}

void
DataSource::handleFrame(rs2::frame &f, timepoint_ns local_frame_timestamp_ns)
{
	// Not our frame.
	if (f.get_profile().stream_index() != this->stream_index) {
		return;
	}

	const auto vf = f.as<rs2::video_frame>();

	// how long was exposure
	const time_duration_ns actual_exposure_width_ns =
	    f.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE) * U_TIME_1US_IN_NS;

	// middle of exposure time in device time
	const timepoint_ns sensor_timestamp_device_ns =
	    f.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP) * U_TIME_1US_IN_NS;
	// frame timestamp in device time
	const timepoint_ns frame_timestamp_device_ns =
	    f.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP) * U_TIME_1US_IN_NS;
	// calculate the offset between the frame timestamp and the sensor timestamp, to get the center of
	// exposure from our converted timestamp, which itself gives the frame timestamp
	const time_duration_ns frame_to_sensor_offset_ns = sensor_timestamp_device_ns - frame_timestamp_device_ns;

	timepoint_ns local_exposure_start_timestamp_ns =
	    local_frame_timestamp_ns + frame_to_sensor_offset_ns - (actual_exposure_width_ns / 2);

	U_LOG_T("Start of exposure (monotonic): %" PRIi64 " ns", local_exposure_start_timestamp_ns);

	// nominal frame period from the stream profile, 0 if unknown
	// const int fps = this->profile.fps();
	// const uint64_t frame_period_ns = fps > 0 ? U_TIME_1S_IN_NS / static_cast<uint64_t>(fps) : 0;

	t_timing_event timing_event = {
	    .type = T_TIMING_EVENT_TYPE_CAMERA_EXPOSURE_START,
	    .camera_exposure_start =
	        {
	            .sequence_id = static_cast<uint64_t>(f.get_frame_number()),
	            .timestamp_ns = local_exposure_start_timestamp_ns,
	            .frame_period_ns = 0, // @todo compute this using the clock skew + frame_period_ns?
	            .exposure_time_ns = static_cast<uint64_t>(actual_exposure_width_ns),
	        },
	};
	t_timing_event_sink_push_timing_event(this->timing_event_sink, &timing_event);

	f.keep();

	// Hold a permanent reference to the frame
	rs2_frame_add_ref(f.get(), nullptr);
	xrt_frame *xf = U_TYPED_CALLOC(xrt_frame);
	*xf = {
	    .reference = {.count = 1},
	    .destroy =
	        [](struct xrt_frame *xf) {
		        rs2_frame *f = reinterpret_cast<rs2_frame *>(xf->owner);

		        rs2_release_frame(f);
		        free(xf);
	        },
	    .owner = f.get(),
	    .width = static_cast<uint32_t>(vf.get_width()),
	    .height = static_cast<uint32_t>(vf.get_height()),
	    .stride = static_cast<uint32_t>(vf.get_stride_in_bytes()),
	    .size = static_cast<uint32_t>(f.get_data_size()),
	    .data = const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(f.get_data())),
	    .format = this->mode.format,
	    .stereo_format = this->mode.stereo_format,
	    .timestamp = local_exposure_start_timestamp_ns,
	    .source_timestamp = static_cast<int64_t>(f.get_timestamp() * U_TIME_1MS_IN_NS),
	    .source_sequence = static_cast<uint64_t>(f.get_frame_number()),
	    .source_id = this->getXDS()->source_id,
	};

	if (u_sink_debug_is_active(&this->debug_sink)) {
		u_sink_debug_push_frame(&this->debug_sink, xf);
	}

	{
		std::unique_lock<os::Mutex> lock(this->lock);
		if (this->target != nullptr) {
			// Push the frame while we still hold a reference.
			xrt_sink_push_frame(this->target, xf);
		}
	}

	// We're done with the frame now.
	xrt_frame_reference(&xf, nullptr);
}

t_camera_calibration
DataSource::cameraCalibration()
{
	const auto intrinsics = this->profile.get_intrinsics();

	t_camera_distortion_model model;
	switch (intrinsics.model) {
	case RS2_DISTORTION_NONE: model = T_DISTORTION_PINHOLE; break;
	case RS2_DISTORTION_KANNALA_BRANDT4: model = T_DISTORTION_FISHEYE_KB4; break;
	default:
		for (size_t i = 0; i < ARRAY_SIZE(intrinsics.coeffs); i++) {
			// NOTE: this may seem stupid, and it is. but sadly required
			if (intrinsics.coeffs[i] != 0.) {
				throw std::runtime_error("Got unsupported realsense distortion model " +
				                         std::string(rs2_distortion_to_string(intrinsics.model)));
			}
		}
		model = T_DISTORTION_PINHOLE;
	}

	t_camera_calibration calibration = {
	    .image_size_pixels = {intrinsics.width, intrinsics.height},
	    .intrinsics = {{0}},
	    .distortion_parameters_as_array = {0},
	    .distortion_model = model,
	};

	t_camera_calibration_fill_intrinsics( //
	    &calibration,                     //
	    intrinsics.fx,                    //
	    intrinsics.fy,                    //
	    intrinsics.ppx,                   //
	    intrinsics.ppy);                  //

	static_assert(ARRAY_SIZE(calibration.distortion_parameters_as_array) >= ARRAY_SIZE(intrinsics.coeffs));
	for (size_t i = 0; i < ARRAY_SIZE(intrinsics.coeffs); i++) {
		calibration.distortion_parameters_as_array[i] = intrinsics.coeffs[i];
	}

	return calibration;
}

xrt_result_t
DataSource::enumerateFrameModes(xrt_ds_frame_mode **out_modes, uint32_t *out_count)
{
	*out_modes = U_TYPED_ARRAY_CALLOC(xrt_ds_frame_mode, 1);
	if (*out_modes == nullptr) {
		return XRT_ERROR_ALLOCATION;
	}

	*out_count = 1;
	**out_modes = this->mode;

	return XRT_SUCCESS;
}

xrt_result_t
DataSource::configureCapture(xrt_ds_frame_capture_parameters *cp)
{
	return XRT_SUCCESS;
}

xrt_result_t
DataSource::frameStreamStart(xrt_frame_sink *xs, xrt_ds_capture_type capture_type, uint32_t frame_mode_index)
{
	assert(frame_mode_index == 0);

	// ref the sensor so that it knows to start streaming
	this->frame_sensor->refSensor();

	std::unique_lock<os::Mutex> lock(this->lock);
	this->target = xs;

	return XRT_SUCCESS;
}

xrt_result_t
DataSource::streamStop()
{
	// unref the sensor so that it knows to stop streaming
	this->frame_sensor->unrefSensor();

	std::unique_lock<os::Mutex> lock(this->lock);
	this->target = nullptr;
	return XRT_SUCCESS;
}

xrt_result_t
DataSource::isRunning(bool &is_running)
{
	std::unique_lock<os::Mutex> lock(this->lock);
	is_running = this->target != nullptr;
	return XRT_SUCCESS;
}

void
DataSource::breakApart()
{
	streamStop();
}

/*
 *
 * Context implementations
 *
 */

std::vector<std::shared_ptr<FrameSensor>>
Context::frameSensors() const
{
	std::vector<std::shared_ptr<FrameSensor>> frame_sensors;
	for (const auto &device : this->devices) {
		auto device_frame_sensors = device->frameSensors();
		frame_sensors.insert(frame_sensors.end(), device_frame_sensors.begin(), device_frame_sensors.end());
	}
	return frame_sensors;
}

void
Context::breakApart()
{
	for (auto &device : this->devices) {
		device->close();
	}
}

/*
 *
 * Methods
 *
 */

static bool
filterProfiles(const rs2::stream_profile &profile)
{
	// Not a video profile.
	if (!profile.template is<rs2::video_stream_profile>()) {
		return true;
	}

	// non-IR
	if (profile.stream_type() != RS2_STREAM_INFRARED) {
		return true;
	}

	const rs2::video_stream_profile video_profile = profile.template as<rs2::video_stream_profile>();

	// Not the right format
	if (video_profile.format() != RS2_FORMAT_Y8) {
		return true;
	}

	// We can get away with low resolution, but not that low.
	//! @todo This should be done by the caller
	if (video_profile.width() < 400) {
		return true;
	}
	if (video_profile.height() < 200) {
		return true;
	}

	return false;
}

Context *
createContext(xrt_frame_context *xfctx)
{
	std::unique_ptr<Context> context = std::make_unique<Context>();

	rs2::context rs2_context{};

	const auto devices = rs2_context.query_devices();
	for (const auto &rs2_device : devices) {
		auto device = std::make_shared<Device>(rs2_context, rs2_device);
		const auto sensors = rs2_device.query_sensors();

		for (const auto &sensor : sensors) {
			auto profiles = sensor.get_stream_profiles();

			// Only video stream profiles that are usable for constellation tracking
			profiles.erase(std::remove_if(profiles.begin(), profiles.end(), filterProfiles),
			               profiles.end());

			if (profiles.empty()) {
				continue;
			}

			std::stable_sort(profiles.begin(), profiles.end(), [](const auto &lhs, const auto &rhs) {
				const rs2::video_stream_profile lhs_video_profile =
				    lhs.template as<rs2::video_stream_profile>();
				const rs2::video_stream_profile rhs_video_profile =
				    rhs.template as<rs2::video_stream_profile>();

				return lhs_video_profile.width() > rhs_video_profile.width();
			});
			std::stable_sort(profiles.begin(), profiles.end(),
			                 [](const auto &lhs, const auto &rhs) { return lhs.fps() > rhs.fps(); });

			std::map<int, rs2::stream_profile> selected_profile_map;
			for (const auto &profile : profiles) {
				if (selected_profile_map.contains(profile.stream_index())) {
					continue;
				}

				selected_profile_map.emplace(profile.stream_index(), profile);
			}

			std::vector<rs2::stream_profile> selected_profiles;
			std::transform(
			    selected_profile_map.begin(), selected_profile_map.end(),
			    std::back_inserter(selected_profiles),
			    [](const std::map<int, rs2::stream_profile>::value_type &pair) { return pair.second; });

			if (sensor.supports(RS2_OPTION_EXPOSURE)) {
				sensor.set_option(RS2_OPTION_EXPOSURE, 500.0f); // 500us exposure
			}
			sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 0.0f); // global time
			if (sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
				sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0); // Disable laser
			}

			sensor.open(selected_profiles);

			std::shared_ptr<FrameSensor> frame_sensor = std::make_shared<FrameSensor>(device, sensor);

			for (const auto &profile : selected_profiles) {
				std::unique_ptr<DataSource> data_source = std::make_unique<DataSource>( //
				    xfctx,                                                              //
				    frame_sensor,                                                       //
				    profile.as<rs2::video_stream_profile>());                           //

				frame_sensor->data_sources.push_back(data_source.release());

				xrt_frame_context_add(xfctx, frame_sensor->data_sources.back()->getFrameNode());
			}

			device->sensors.push_back(frame_sensor);
		}

		if (!device->sensors.empty()) {
			context->devices.push_back(device);
		}
	}

	if (context->devices.empty()) {
		throw std::runtime_error("No applicable sensors found on any device.");
	}

	// Only once nothing can throw anymore, so no thread is left behind on failure.
	for (auto &device : context->devices) {
		if (!device->startClockProbe()) {
			U_LOG_W(
			    "Failed to start the clock probe thread of %s %s, timestamps will not be latency corrected",
			    device->name.c_str(), device->serial.c_str());
		}
	}

	Context *raw_context = context.release();
	xrt_frame_context_add(xfctx, raw_context->getFrameNode());
	return raw_context;
}

} // namespace xrt::drivers::realsense

using namespace xrt::drivers::realsense;

struct rs_ds_context *
rs_ds_context_create(xrt_frame_context *xfctx)
{
	try {
		Context *context = createContext(xfctx);
		return reinterpret_cast<struct rs_ds_context *>(context);
	} catch (std::exception &e) {
		U_LOG_E("Got exception when creating context, %s", e.what());
		return nullptr;
	}
}

uint32_t
rs_ds_context_sensor_count(rs_ds_context *raw_context)
{
	Context *context = reinterpret_cast<Context *>(raw_context);

	return static_cast<uint32_t>(context->frameSensors().size());
}

uint32_t
rs_ds_context_sensor_data_source_count(rs_ds_context *raw_context, uint32_t sensor)
{
	Context *context = reinterpret_cast<Context *>(raw_context);

	const auto frame_sensors = context->frameSensors();

	assert(sensor < frame_sensors.size());
	if (sensor >= frame_sensors.size()) {
		return -1;
	}

	return frame_sensors[sensor]->data_sources.size();
}

xrt_ds *
rs_ds_context_sensor_get_data_source(rs_ds_context *raw_context,
                                     uint32_t sensor_idx,
                                     uint32_t data_source_idx,
                                     t_camera_calibration *out_calib,
                                     t_timing_event_source **out_timing_event_source)
{
	Context *context = reinterpret_cast<Context *>(raw_context);

	const auto frame_sensors = context->frameSensors();

	assert(sensor_idx < frame_sensors.size());
	if (sensor_idx >= frame_sensors.size()) {
		return nullptr;
	}

	auto &sensor = frame_sensors[sensor_idx];

	assert(data_source_idx < sensor->data_sources.size());
	if (data_source_idx >= sensor->data_sources.size()) {
		return nullptr;
	}

	DataSource *data_source = sensor->data_sources[data_source_idx];

	if (out_calib) {
		try {
			(*out_calib) = data_source->cameraCalibration();
		} catch (std::exception &e) {
			U_LOG_E("Got exception when getting camera calibration, %s", e.what());
			return nullptr;
		}
	}

	if (out_timing_event_source) {
		(*out_timing_event_source) = data_source->timing_event_source;
	}

	return data_source->getXDS();
}
