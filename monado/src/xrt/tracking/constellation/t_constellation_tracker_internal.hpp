// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Internal ures for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "xrt/xrt_config_build.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"

#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_var.h"
#include "util/u_threading.h"
#include "util/u_weak_ptr.hpp"
#include "util/u_sink.h"
#include "util/u_frame.h"
#include "util/u_frame_scribble.h"

#include "tracking/t_constellation.h"

#include "math/m_api.h"

#include "correspondence_search.h"
#include "led_search_model.h"
#include "pose_metrics.h"
#include "t_constellation_tracker.h"
#include "trace_log/t_constellation_tracker_trace_log.hpp"
#include "optimizer/pose_optimize.hpp"
#include "optimizer/rig_calibration.hpp"
#include "optimizer/sensor_fusion.hpp"

#include <array>
#include <atomic>
#include <ceres/context.h>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <stdexcept>
#include <thread>
#include <vector>


#define CT_TRACE(ct, ...) U_LOG_IFL_T(ct->log_level, __VA_ARGS__)
#define CT_DEBUG(ct, ...) U_LOG_IFL_D(ct->log_level, __VA_ARGS__)
#define CT_INFO(ct, ...) U_LOG_IFL_I(ct->log_level, __VA_ARGS__)
#define CT_WARN(ct, ...) U_LOG_IFL_W(ct->log_level, __VA_ARGS__)
#define CT_ERROR(ct, ...) U_LOG_IFL_E(ct->log_level, __VA_ARGS__)

#define MIN_ROT_ERROR DEG_TO_RAD(30)
#define MIN_POS_ERROR 0.10

/*
 *
 * Forward declares for C callback functions
 *
 */

extern "C" void
constellation_tracker_camera_push_blobs(t_blob_sink *tbs, t_blob_observation *tbo);

extern "C" void
constellation_tracker_camera_destroy(t_blob_sink *tbs);

extern "C" void *
constellation_tracker_camera_slow_thread(void *ptr);

extern "C" void *
constellation_tracker_camera_fast_thread(void *ptr);

extern "C" void
constellation_tracker_node_break_apart(xrt_frame_node *node);

extern "C" void
constellation_tracker_node_destroy(xrt_frame_node *node);


namespace xrt::tracking::constellation {

using namespace xrt::tracking::constellation::optimizer;

namespace os = xrt::auxiliary::os;

// Forward-declares
struct Camera;
struct CameraMosaic;
struct ConstellationTracker;
struct DataRecorder;
struct Device;

struct FoundDevicePose
{
	xrt_pose Tcv_cam_device XRT_POSE_IDENTITY;
	float average_blob_brightness;

	/*!
	 * Whether a found pose was already handled, this is present so that if a single sample is torn between some
	 * devices fast recovering and some devices slow recovering, we push the fast recovery poses the moment they're
	 * found rather than at the end of the slow recovery where the sample is "terminated".
	 */
	bool handled{false};
};

struct DeviceState
{
	//! The ID of the device.
	t_constellation_device_id_t device_id{XRT_CONSTELLATION_INVALID_DEVICE_ID};

	//! The predicted pose of the device in the world at the time of the sample.
	std::optional<xrt_pose> Txr_world_device_prior{std::nullopt};
	//! The predicted pose of the device relative to the camera at the time of the sample.
	std::optional<xrt_space_relation> Tcv_cam_device_predicted{std::nullopt};

	//! The final found pose of the device in this specific sample.
	std::optional<FoundDevicePose> found_pose{std::nullopt};

	t_constellation_tracker_trace_log_solve_type solve_type{CONSTELLATION_SOLVE_TYPE_NO_SOLVE};

	//! Whether the device needs to have the slow processing thread run over it
	bool needs_slow_processing{false};
};

struct CameraSample
{
public: // Fields
	t_blobwatch *source{nullptr};

	//! The trace log for this sample, null if tracing is disabled.
	TraceLogRef trace_log{};

	time_duration_ns pipeline_latency_ns{};
	std::optional<timepoint_ns> fast_duration_ns{std::nullopt};
	std::optional<timepoint_ns> slow_duration_ns{std::nullopt};

	uint64_t id{};
	uint64_t sequence_id{};
	int64_t timestamp_ns{};
	t_blob blobs[XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME]{};
	uint32_t blob_count{};

	//! The camera's position in the constellation tracker's tracking origin
	std::optional<xrt_pose> Txr_world_cam{std::nullopt};

	std::array<DeviceState, XRT_CONSTELLATION_MAX_DEVICES> device_states{};
	uint32_t device_count{};

	uint32_t mosaic_index;
	uint32_t camera_index;

public: // Methods
	std::optional<DeviceState *>
	getDeviceState(t_constellation_device_id_t device_id);

	DeviceState &
	putDeviceState(t_constellation_device_id_t device_id);

	CameraSample(t_blob_observation &blobservation, Camera *camera);

	CameraSample() = default;

	void
	markMatchingBlobs(ConstellationTracker *ct,
	                  t_constellation_tracker_led_model &led_model,
	                  t_constellation_device_id_t device_id,
	                  pose_metrics_blob_match_info &blob_match_info);

	t_blob_observation
	toBlobObservation() const &
	{
		t_blob_observation obs = {
		    .source = this->source,
		    // The returned observation is non-owning, so it can't carry a trace log reference.
		    .trace_log = nullptr,
		    .id = this->id,
		    .sequence_id = this->sequence_id,
		    .timestamp_ns = this->timestamp_ns,
		    .blobs = const_cast<t_blob *>(this->blobs),
		    .num_blobs = this->blob_count,
		    .frame = nullptr,
		};

		return obs;
	}

	t_blob_observation
	toBlobObservation() && = delete;
	t_blob_observation
	toBlobObservation() const && = delete;
};

struct CameraScribbleSettings
{
public: // Fields
	//! Whether to draw the blobs in the sample
	bool draw_blobs{true};
	//! Whether to draw the raw blob IDs
	bool draw_blob_ids{false};
	//! Whether to draw the device IDs on the blobs where possible
	bool draw_blob_device_ids{true};
	//! Whether to draw the LED IDs on the blobs where possible
	bool draw_blob_led_ids{false};

	//! Whether to draw the prior device pose in the sample, if it exists.
	bool draw_prior{false};
	//! Whether to draw the final device pose in the sample, if it exists.
	bool draw_found{false};

public: // Methods
	void
	setupDebugTracking(void *root);
};

struct Camera
{
public: // Fields
	t_blob_sink base = {
	    .push_blobs = constellation_tracker_camera_push_blobs,
	    .destroy = constellation_tracker_camera_destroy,
	};

	//! The owner tracker, so we can retrieve it from the blob sink callback
	ConstellationTracker *tracker;
	std::weak_ptr<CameraMosaic> mosaic;

	t_camera_calibration calibration;

	camera_model model;

	CameraScribbleSettings scribble_settings{};

	//! The index in the mosaic
	uint32_t index;

	//! Does "slow" processing for this camera when fast recovery paths fail.
	os_thread_helper slow_processing_thread{};
	struct
	{
		std::optional<CameraSample> sample{std::nullopt};

		correspondence_search *cs{nullptr};

		u_sink_debug debug_sink{};
	} slow_processing_thread_data;

	/*!
	 * Does "fast" processing for this camera, trying to recover a pose quickly. It's valid for this to happen at
	 * the same time as a slow process.
	 */
	os_thread_helper fast_processing_thread{};
	struct
	{
		std::optional<CameraSample> sample{std::nullopt};

		correspondence_search *cs{nullptr};

		u_sink_debug debug_sink{};
	} fast_processing_thread_data;

	//! Locks all processing data
	mutable os::Mutex processing_lock;
	//! All data protected by the processing lock
	struct
	{
		xrt_pose Txr_origin_cam;
		bool has_concrete_pose;
	} locked_data;

	std::atomic<uint32_t> shallow_searches_failed{0};

	//! Samples the slow search has taken (slow thread only): staggers the search for lost devices across cameras.
	uint64_t slow_samples{0};

	//! This camera's index in the EuRoC recorder session (flattened across mosaics), or -1 if it has none.
	std::atomic<int32_t> euroc_cam_index{-1};

public: // Methods (t_constellation_tracker.cpp)
	static Camera *
	Get(t_blob_sink *tbs)
	{
		return container_of(tbs, Camera, base);
	}

	Camera(ConstellationTracker *tracker,
	       std::weak_ptr<CameraMosaic> mosaic,
	       const t_constellation_tracker_camera &camera_params,
	       enum u_logging_level *log_level_ptr,
	       uint32_t index);

	~Camera();

	// Delete all copy/move ctors, since the pointers for `base` need to be stable
	Camera(const Camera &) = delete;
	Camera(Camera &&) = delete;
	Camera &
	operator=(const Camera &) = delete;
	Camera &
	operator=(Camera &&) = delete;

	std::optional<xrt_pose>
	getWorldPose(timepoint_ns when_ns);

	void
	deferSampleToSlowThread(CameraSample &sample);

	//! Fast matching based on prior pose
	bool
	tryDevicePose(Device *device,
	              CameraSample &sample,
	              DeviceState &device_state,
	              bool need_matching_ids,
	              const std::optional<xrt_space_relation> &Tcv_cam_device_prior,
	              const xrt_pose &Tcv_cam_device_candidate,
	              t_constellation_tracker_trace_log_solve_type solve_type);

	bool
	tryDevicePriorSnap(Device *device, CameraSample &sample, DeviceState &device_state, const xrt_space_relation &prior);

	bool
	tryDeviceBlobRecovery(Device *device,
	                      CameraSample &sample,
	                      DeviceState &device_state,
	                      const std::optional<xrt_space_relation> &Tcv_cam_device_prior);

	void
	handleStepComplete(CameraSample &sample);

	void
	handleProcessingComplete(CameraSample &sample);

	void
	processSampleSlow(CameraSample &sample);

	//! Returns whether a slow search is needed
	bool
	processSampleFast(CameraSample &sample);

	void
	pushPose(CameraSample &camera_sample,
	         DeviceState &device_state,
	         Device *device,
	         pose_metrics &score,
	         const xrt_pose &Tcv_cam_device,
	         bool was_optimized,
	         t_constellation_tracker_trace_log_solve_type solve_type);

public: // Methods (constellation_debug_scribble.cpp)
	void
	debugScribbleSample(CameraSample &sample, bool fast);
};

struct CameraMosaic
{
public: // Fields
	std::vector<std::unique_ptr<Camera>> cameras;
	//! The index of this mosaic in the tracker.
	uint32_t index;

	t_constellation_tracker_tracking_source *tracking_origin;

public: // Methods
	CameraMosaic(ConstellationTracker *tracker,
	             const t_constellation_tracker_camera_mosaic &mosaic_params,
	             uint32_t index);

	~CameraMosaic() = default;

	std::optional<xrt_pose>
	getTrackingOriginPose(timepoint_ns when_ns);
};

struct DeviceLastPose
{
public: // Fields
	xrt_pose Txr_world_device;
	xrt_pose Tcv_cam_device;
	timepoint_ns timestamp_ns;

public: // Methods
	DeviceLastPose(xrt_pose Txr_world_device, xrt_pose Tcv_cam_device, timepoint_ns timestamp_ns);
};

struct DeviceBase
{
	xrt_imu_sink imu_sink;
};

struct Device : public DeviceBase
{
public: // Fields
	//! The device params, with a LED model that has been copied into OpenCV space for tracking.
	t_constellation_tracker_device_params params;
	/*!
	 * The LED model as originally provided by the device, in the OpenXR frame. This is not copied, we assume the
	 * LEDs it points to outlive the device.
	 */
	t_constellation_tracker_led_model original_led_model;
	t_constellation_tracker_device *device;

	t_constellation_device_id_t id;

	//! Devices are disabled by default, and the constellation tracker must choose to enable them.
	std::atomic<bool> enabled{false};

	/*!
	 * Learned "up" in the device IMU's world frame (T_CONSTELLATION_IMU_UP_LEARN), which the IMU up checks use instead
	 * of +Y: the device's IMU-only orientation tilts during vigorous motion (00:06 run: 12 -> 73 deg from the optical up
	 * within 15 s), and past the gate every correct pose was rejected. Updated from poses that passed the checks.
	 */
	std::mutex imu_up_lock;
	xrt_vec3 imu_world_up{0.0f, 1.0f, 0.0f};

	//! The IMU's up direction in the device frame for an IMU orientation, with the learned up.
	xrt_vec3
	imuUpInDevice(const xrt_quat &q_imu);
	//! Move the learned up toward the up of an accepted optical pose.
	void
	learnImuUp(const xrt_quat &q_imu, const xrt_quat &q_optical);

	//! This device's IMU index in the EuRoC recorder session, or -1 if it has none.
	std::atomic<int32_t> euroc_imu_index{-1};

	//! The owner tracker, so we can retrieve it from the IMU sink callback
	ConstellationTracker *tracker;

	std::unique_ptr<sensor_fusion::SensorFusion> sensor_fusion{};

	// @todo remove when clang-format is updated in CI
	// clang-format off
	t_constellation_search_model *search_model{nullptr};

	// @todo These need to be pulled from the device and put into the sample. 
	//       Right now we just hardcode them since we don't have any real sensor fusion.
	xrt_vec3 prior_pos_error{MIN_POS_ERROR, MIN_POS_ERROR, MIN_POS_ERROR};
	xrt_vec3 prior_rot_error{MIN_ROT_ERROR, MIN_ROT_ERROR, MIN_ROT_ERROR};
	float gravity_error_rad{MIN_ROT_ERROR}; /* Gravity vector uncertainty in radians 0..M_PI */

	//! Gyro gate reference: the last accepted pose's orientation and the device IMU's at the same time.
	std::mutex gyro_gate_lock;
	bool gyro_ref_valid{false};
	int64_t gyro_ref_ts{0};
	xrt_quat gyro_ref_optical = XRT_QUAT_IDENTITY;
	xrt_quat gyro_ref_imu = XRT_QUAT_IDENTITY;
	uint32_t gyro_rejects_in_a_row{0};

	mutable os::Mutex data_lock;
	struct
	{
		std::optional<DeviceLastPose> last_known_pose;
	} locked_data;
	// clang-format on

public: // Methods
	Device(ConstellationTracker *tracker,
	       t_constellation_tracker_device_params *params,
	       t_constellation_tracker_device *device,
	       t_constellation_device_id_t id);

	~Device();

	// Delete move constructors
	Device(const Device &) = delete;
	Device(Device &&) = delete;
	Device &
	operator=(const Device &) = delete;
	Device &
	operator=(Device &&) = delete;

	static Device *
	fromXrtImuSink(xrt_imu_sink *sink)
	{
		// Go through DeviceBase* to Device* so that `Device` doesn't have to be a standard layout type and we
		// can still use `container_of` safely.
		return static_cast<Device *>(container_of(sink, DeviceBase, imu_sink));
	}

	void
	pushImuSample(xrt_imu_sample &sample);
};

// Separate base struct with our interface implementations so that `ConstellationTrackerBase` remains a standard layout
// type and we can safely use `container_of` on it.
struct ConstellationTrackerBase
{
	xrt_frame_node node = {
	    .next = nullptr,
	    .break_apart = constellation_tracker_node_break_apart,
	    .destroy = constellation_tracker_node_destroy,
	};
	xrt_tracking_origin tracking_origin = {
	    .name = "Constellation Tracker",
	    .type = XRT_TRACKING_TYPE_CONSTELLATION,
	    .initial_offset =
	        {
	            .orientation = XRT_QUAT_IDENTITY,
	            .position = {.x = 0, .y = 1.2, .z = 0},
	        },
	};
};

/*!
 * A EuRoC recorder built from a snapshot of the tracker's devices and cameras, with its own u_var root (keyed by the
 * address of this struct) and its own frame context so it can be torn down independently of the tracker.
 */
struct EurocRecorderSession
{
	// having one here looks icky but it prevents leaks cus we create&destroy
	xrt_frame_context xfctx{};
	xrt_tracking_sinks *sinks{nullptr};

	char camera_info[64]{};
	char device_info[64]{};

	~EurocRecorderSession();
};

/*!
 * Counters for the tracker report, logged at info level every 5 s (diag_maybe_report): where a controller's tracking
 * stops (no blobs, searches that find nothing, refinement failing, poses rejected by the IMU gates, or accepted), so a
 * normal run shows it without the debug log. Relaxed atomics, bumped on the camera threads; no per-blob work.
 */
struct TrackerDiag
{
	static constexpr int kDevices = 4;
	std::atomic<uint64_t> observations{0}; //!< Blob observations received (all cameras).
	std::atomic<uint64_t> blobs{0};        //!< Blobs in them, for the average.
	std::atomic<uint64_t> max_blobs{0};    //!< Most blobs in one observation.
	std::atomic<uint64_t> fast_drops{0};   //!< Samples dropped because the fast thread was still busy.
	std::atomic<uint64_t> slow_drops{0};   //!< Samples dropped because the slow thread was still busy.
	struct Dev
	{
		std::atomic<uint64_t> slow_searches{0}; //!< Correspondence searches run.
		std::atomic<uint64_t> slow_found{0};    //!< ... that found a pose.
		std::atomic<uint64_t> fast_found{0};    //!< Poses found by the fast path (prediction, recovery, snap).
		std::atomic<uint64_t> refine_failed{0}; //!< Found poses whose RANSAC refinement failed.
		std::atomic<uint64_t> rejected_up{0};   //!< Rejected: up direction disagrees with the IMU.
		std::atomic<uint64_t> rejected_gyro{0}; //!< Rejected: rotation disagrees with the gyro.
		std::atomic<uint64_t> accepted{0};      //!< Poses accepted ("Found pose for device").
	} dev[kDevices];
	std::atomic<int64_t> last_report_ns{0};
};

struct ConstellationTracker : public ConstellationTrackerBase
{
public: // Fields
	//! Whether the constellation tracker is running
	bool running = true;

	//! Counters for the periodic tracker report.
	TrackerDiag diag;

	enum u_logging_level log_level;

	t_constellation_tracker_params params;

	bool deterministic{false};
	//! Reject solved poses whose up direction is further than this from the device's IMU, degrees (0 = off).
	float gravity_gate_deg{35.0f};
	//! Reject solved poses whose rotation since the device's last accepted pose disagrees with its IMU's by more
	//! than this, degrees (0 = off).
	float gyro_gate_deg{15.0f};

	std::vector<std::shared_ptr<CameraMosaic>> mosaics;

	// Put here to be destroyed before devices (reverse declaration order)
	std::unique_ptr<ceres::Context> ceres_context{};

	std::shared_mutex device_lock;
	std::vector<std::unique_ptr<Device>> devices;
	t_constellation_device_id_t next_device_id{0};

	std::unique_ptr<DataRecorder> data_recorder{};

	mutable os::Mutex rig_calibration_lock{};
	os_thread_helper rig_calibration_thread{};
	bool running_calibration{false};
	std::unique_ptr<rig_calibration::RigCalibration> rig_calibration{};

	//! The current EuRoC recorder snapshot, if any. Only touched from the toggle thread (and on destroy).
	std::unique_ptr<EurocRecorderSession> euroc_session{};
	//! The sinks of @ref euroc_session, published only while it is fully set up. Read by the push paths, a debug
	//! aid so there's no protection against it being destroyed mid-push.
	std::atomic<xrt_tracking_sinks *> euroc_sinks{nullptr};
	u_var_button euroc_toggle_btn{};
	//! The label state the button is showing, only touched from the GUI thread.
	bool euroc_session_wanted{false};
	//! Whether a toggle is currently running on @ref euroc_toggle_thread.
	std::atomic<bool> euroc_toggle_pending{false};
	std::thread euroc_toggle_thread{};

#ifdef XRT_FEATURE_RERUN
	std::unique_ptr<struct RerunContext> rerun_stream{};
#endif

public: // Methods
	static ConstellationTracker *
	Get(xrt_frame_node *node)
	{
		return static_cast<ConstellationTracker *>(container_of(node, ConstellationTrackerBase, node));
	}

	static ConstellationTracker *
	Get(t_constellation_tracker *tracker)
	{
		return reinterpret_cast<ConstellationTracker *>(tracker);
	}

	ConstellationTracker(t_constellation_tracker_params *params);

	~ConstellationTracker();

	// Delete all copy/move ctors, since the pointers for `node` need to be stable
	ConstellationTracker(const ConstellationTracker &) = delete;
	ConstellationTracker(ConstellationTracker &&) = delete;
	ConstellationTracker &
	operator=(const ConstellationTracker &) = delete;
	ConstellationTracker &
	operator=(ConstellationTracker &&) = delete;

	bool
	needSensorCalibration();

	bool
	checkSensorCalibration();

	void
	setupVariableTracking();

	/*!
	 * Creates (or destroys, if one exists) the EuRoC recorder snapshot of the current devices and cameras. Must
	 * not be called with the u_var lock held, since it adds/removes u_var roots.
	 */
	void
	setEurocRecorderSession(bool enabled);

	//! Waits for any in-flight toggle to finish and destroys the EuRoC recorder session.
	void
	shutdownEurocRecorderSession();

	t_constellation_device_id_t
	addDevice(t_constellation_tracker_device_params *params, t_constellation_tracker_device *device);

	void
	removeDevice(t_constellation_device_id_t device_id);

	void
	getTrackedPose(t_constellation_device_id_t device_id, timepoint_ns when_ns, xrt_space_relation &out_relation);
};

}; // namespace xrt::tracking::constellation
