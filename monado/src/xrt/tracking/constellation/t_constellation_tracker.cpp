// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation of the main logic for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#include "xrt/xrt_config_build.h"
#include "xrt/xrt_defines.h"

#include "util/u_time.h"

#include "tracking/t_euroc_recorder.h"

#include "math/m_predict.h"
#include "math/m_vec3.h"

#include "constellation/correspondence_search.h"

#include "t_constellation_tracker_internal.hpp"
#include "t_constellation_tracker_dataset.hpp"

#ifdef XRT_FEATURE_RERUN
#include "constellation_tracker_rerun.hpp"
#endif

#include <atomic>
#include <algorithm>
#include <vector>
#include <limits>
#include <string>
#include <cstring>


namespace xrt::tracking::constellation {

DEBUG_GET_ONCE_LOG_OPTION(constellation_tracker_log, "CONSTELLATION_TRACKER_LOG", U_LOGGING_WARN)
// See the gravity gate in Camera::pushPose; 0 turns it off.
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_gravity_gate_deg, "T_CONSTELLATION_GRAVITY_GATE_DEG", 35.0f)
// See the gyro gate in Camera::pushPose; 0 turns it off.
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_gyro_gate_deg, "T_CONSTELLATION_GYRO_GATE_DEG", 15.0f)
/*!
 * After a solve, how long the slow search leaves a device to the fast one (ms). Was 150: replaying the 19:34 PS VR2
 * run (psvr2-test/ctreplay), 50 cut the time in optical gaps from 45/53% to 35/33% (left/right controller), 0 to
 * 33/30%, with no position jumps or gravity gate failures; 50 costs about the same CPU as 150, 0 about 30% more.
 */
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_slow_skip_ms, "T_CONSTELLATION_SLOW_SKIP_MS", 50)
/*!
 * Upstream 026c4ab4e: a device nearer the camera may take blobs already matched to a farther one (overlapping
 * controllers: two-handed grips, hands crossing). Off until measured (psvr2-test/ctreplay).
 */
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_near_takes_blobs, "T_CONSTELLATION_NEAR_TAKES_BLOBS", false)
//! How long shallow searches must have failed before a deep search (ms), see processSampleSlow.
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_deep_delay_ms, "T_CONSTELLATION_DEEP_DELAY_MS", 1000)
/*
 * Searching for a lost device (no solve for T_CONSTELLATION_LOST_MS): each camera searches for it only every Nth sample,
 * the cameras and devices offset so they take turns. 1 = every sample (the old behaviour). The sunny PS VR2 replay
 * (2026-11-17 15:22) spent 82% of the tracker's time in samples where no device was solved by any camera.
 */
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_lost_search_every, "T_CONSTELLATION_LOST_SEARCH_EVERY", 1)
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_lost_ms, "T_CONSTELLATION_LOST_MS", 300)
/*
 * Blobs larger than this (pixels, the larger side) are left out of matching; 0 = all. In that replay the LED-matched
 * blobs were at most 30 px (p99) while sun patches reached 98-240 px: 48 px dropped 15% of the other blobs and 0.05% of
 * the LED ones. On by default (48): sunny replay right controller fixes 73% -> 78%, 16 -> 11 gaps, -16% tracker time;
 * the 10:58 normal-light replay unchanged (deterministic and real time). T_CONSTELLATION_MAX_BLOB_PX=0 turns it off.
 * T_CONSTELLATION_LOST_SEARCH_EVERY=2 alone halved the sunny replay's tracker time with no fixes lost and changed nothing
 * in normal light, but together with the size limit it lost fixes in the normal-light real-time replay (left 91-92% ->
 * 86-88%, three runs each), cause not yet found: off by default.
 */
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_max_blob_px, "T_CONSTELLATION_MAX_BLOB_PX", 48)
//! Above this rotation rate (deg/s), the fast path skips blob recovery and wipes the blob associations.
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_blob_recovery_max_dps, "T_CONSTELLATION_BLOB_RECOVERY_MAX_DPS", 360.0f)
/*!
 * Fast path: when the prior doesn't match, find the image shift that lines its LEDs up with blobs and solve from those.
 * Replays of 3 PS VR2 sessions (psvr2-test/ctreplay, real-time, live IMU lag): losses over 150 ms 43/60/87 -> 20/22/36,
 * time without a fix about halved, no extra CPU; with the blob recovery prior gate, no single-frame spikes.
 */
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_prior_snap, "T_CONSTELLATION_PRIOR_SNAP", true)
//! 1: solve through the blob recovery (trusts any RANSAC solve over 4 blobs); 2: require an unambiguous shift and a
//! good match. Mode 2 was a guard against "spikes" that turned out to be the replay mixing two cameras' poses.
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_prior_snap_mode, "T_CONSTELLATION_PRIOR_SNAP_MODE", 1)
//! Mode 1: the snapped pose must be this close to the prior (m, deg); good snaps were within 2 cm / 6.7 deg (p90).
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_prior_snap_max_m, "T_CONSTELLATION_PRIOR_SNAP_MAX_M", 0.05f)
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_prior_snap_max_deg, "T_CONSTELLATION_PRIOR_SNAP_MAX_DEG", 15.0f)
//! Blob recovery: reject a solve further than the snap limits from the prior (when there is one).
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_blob_recovery_prior_gate, "T_CONSTELLATION_BLOB_RECOVERY_PRIOR_GATE", true)
/*!
 * Learn each device IMU's tilt error from accepted poses and use it in the IMU up checks (gain per accepted pose, 0
 * off). The PS VR2 controllers' IMU-only up drifted 12 -> 73 deg from the optical one in vigorous play (00:06 run) and
 * the 35 deg gate then rejected every correct pose: the right controller froze for ~35 s with its LEDs in view.
 * Replayed with 0.05: that window 4% -> 82% of frames with a fix, time without a fix 31/41% -> 17/18% (00:06) and
 * 25/21% -> 17/11% (23:22), unchanged on calmer runs, no single-frame spikes.
 *
 * A mitigation. The controllers' gyro bias is now removed at rest (pssense_imu_rest.h); the rest of the error is a
 * ~30 deg pitch between the PS VR2 camera rig and gravity (controller gravity vs the tracker's up in every recording,
 * headset SLAM level to 0.5 deg), which was the tracker's camera mount error, fixed at the source on 2026-11-19
 * (PSVR2_CAM_ALIGN; PSSENSE_IMU_REF_CORRECTION_DEG back to 0), and the right controller's gyro cross-axis error
 * (~2.3% X-Z), still open. Remove once that is fixed too.
 */
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_imu_up_learn, "T_CONSTELLATION_IMU_UP_LEARN", 0.05f)
//! Largest prior image error the snap searches (undistorted px).
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_prior_snap_radius_px, "T_CONSTELLATION_PRIOR_SNAP_RADIUS_PX", 60.0f)
//! Run the prior-seeded shallow correspondence search in the fast path when the prior itself didn't match.
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_fast_prior_search, "T_CONSTELLATION_FAST_PRIOR_SEARCH", false)
DEBUG_GET_ONCE_OPTION(constellation_tracker_data_recorder_output, "CONSTELLATION_TRACKER_DATA_RECORDER_OUTPUT", "")

// Unconditionally present to allow warning that the feature is not enabled.
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_enable_rerun, "CONSTELLATION_TRACKER_RERUN_ENABLE", false)
#ifdef XRT_FEATURE_RERUN
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_rerun_spawn, "CONSTELLATION_TRACKER_RERUN_SPAWN", true)
#endif

/*
 *
 * Helper functions
 *
 */

//! The report counters of device @p id, or NULL for an id past TrackerDiag::kDevices.
static TrackerDiag::Dev *
diag_dev(ConstellationTracker *tracker, int id)
{
	return (id >= 0 && id < TrackerDiag::kDevices) ? &tracker->diag.dev[id] : nullptr;
}

static void
diag_dev_inc(ConstellationTracker *tracker, int id, std::atomic<uint64_t> TrackerDiag::Dev::*field)
{
	TrackerDiag::Dev *d = diag_dev(tracker, id);
	if (d != nullptr) {
		(d->*field).fetch_add(1, std::memory_order_relaxed);
	}
}

static void
diag_count_observation(ConstellationTracker *tracker, uint32_t num_blobs)
{
	TrackerDiag &g = tracker->diag;
	g.observations.fetch_add(1, std::memory_order_relaxed);
	g.blobs.fetch_add(num_blobs, std::memory_order_relaxed);
	uint64_t prev = g.max_blobs.load(std::memory_order_relaxed);
	while (num_blobs > prev && !g.max_blobs.compare_exchange_weak(prev, num_blobs, std::memory_order_relaxed)) {
	}
}

/*!
 * Every 5 s, one camera thread logs the tracker report and resets the counters: blobs per observation, dropped
 * samples, and per controller what the searches found and why poses were turned down.
 */
static void
diag_maybe_report(ConstellationTracker *tracker)
{
	TrackerDiag &g = tracker->diag;
	const int64_t now_ns = (int64_t)os_monotonic_get_ns();
	int64_t last_ns = g.last_report_ns.load(std::memory_order_relaxed);
	if (last_ns == 0) {
		g.last_report_ns.compare_exchange_strong(last_ns, now_ns, std::memory_order_relaxed);
		return;
	}
	if (now_ns - last_ns < (int64_t)U_TIME_1S_IN_NS * 5 ||
	    !g.last_report_ns.compare_exchange_strong(last_ns, now_ns, std::memory_order_relaxed)) {
		return;
	}
	const double secs = (double)(now_ns - last_ns) / (double)U_TIME_1S_IN_NS;
	const uint64_t obs = g.observations.exchange(0, std::memory_order_relaxed);
	const uint64_t blobs = g.blobs.exchange(0, std::memory_order_relaxed);
	CT_INFO(tracker,
	        "Tracker report (%.0fs): %" PRIu64 " observations, %.1f blobs each (max %" PRIu64 "), dropped %" PRIu64
	        " fast / %" PRIu64 " slow samples",
	        secs, obs, obs > 0 ? (double)blobs / (double)obs : 0.0, g.max_blobs.exchange(0, std::memory_order_relaxed),
	        g.fast_drops.exchange(0, std::memory_order_relaxed), g.slow_drops.exchange(0, std::memory_order_relaxed));
	const int n = std::min<int>((int)tracker->next_device_id, TrackerDiag::kDevices);
	for (int i = 0; i < n; i++) {
		TrackerDiag::Dev &d = g.dev[i];
		const uint64_t searches = d.slow_searches.exchange(0, std::memory_order_relaxed);
		const uint64_t found = d.slow_found.exchange(0, std::memory_order_relaxed);
		const uint64_t fast = d.fast_found.exchange(0, std::memory_order_relaxed);
		const uint64_t refine = d.refine_failed.exchange(0, std::memory_order_relaxed);
		const uint64_t up = d.rejected_up.exchange(0, std::memory_order_relaxed);
		const uint64_t gyro = d.rejected_gyro.exchange(0, std::memory_order_relaxed);
		const uint64_t accepted = d.accepted.exchange(0, std::memory_order_relaxed);
		CT_INFO(tracker,
		        "Tracker report device %d: searches %" PRIu64 " found %" PRIu64 ", fast path %" PRIu64
		        ", refinement failed %" PRIu64 ", rejected by IMU up %" PRIu64 " / gyro %" PRIu64
		        ", accepted %" PRIu64,
		        i, searches, found, fast, refine, up, gyro, accepted);
	}
}

//! Gets the gravity vector of a pose, in the pose's local frame.
static void
get_pose_gravity_vector(xrt_pose &T_world_pose, xrt_vec3 &gravity)
{
	// Extract the gravity vector from the pose's orientation
	gravity = XRT_VEC3_UNIT_Y;

	xrt_quat T_pose_world_orientation;
	math_quat_invert(&T_world_pose.orientation, &T_pose_world_orientation);

	math_quat_rotate_vec3(&T_pose_world_orientation, &gravity, &gravity);
}

static uint32_t
num_blobs_for_device(CameraSample &sample, t_constellation_device_id_t device_id)
{
	uint32_t out_num_blobs = 0;
	for (uint32_t i = 0; i < sample.blob_count; i++) {
		t_blob &b = sample.blobs[i];
		if (b.matched_device_id == device_id) {
			out_num_blobs++;
		}
	}
	return out_num_blobs;
}

/*!
 * Sort key used to search for the devices closest to the camera first, since a near device covers more of the image and
 * is the one most likely to explain the blobs. Devices we have no prediction for sort last.
 *
 * @note The prediction is stored on the sample, so this is usable from the slow thread as well, where the fast thread's
 *       predictions are no longer being recomputed.
 */
static float
device_state_camera_distance_sqrd(const DeviceState *device_state)
{
	if (device_state == nullptr || !device_state->Tcv_cam_device_predicted.has_value()) {
		return std::numeric_limits<float>::infinity();
	}

	return m_vec3_len_sqrd(device_state->Tcv_cam_device_predicted->pose.position);
}

/*!
 * Collects the tracker's devices into @p out_devices, ordered nearest-camera-first for @p sample.
 *
 * The caller must hold @ref ConstellationTracker::device_lock for as long as it uses the result, since these are
 * borrowed pointers into @ref ConstellationTracker::devices.
 */
static uint32_t
collect_devices_nearest_first(ConstellationTracker *tracker,
                              CameraSample &sample,
                              std::array<Device *, XRT_CONSTELLATION_MAX_DEVICES> &out_devices)
{
	uint32_t device_count = 0;
	for (std::unique_ptr<Device> &device : tracker->devices) {
		assert(device_count < out_devices.max_size());
		out_devices[device_count++] = device.get();
	}

	std::stable_sort(out_devices.data(), out_devices.data() + device_count, [&sample](Device *x, Device *y) {
		return device_state_camera_distance_sqrd(sample.getDeviceState(x->id).value_or(nullptr)) <
		       device_state_camera_distance_sqrd(sample.getDeviceState(y->id).value_or(nullptr));
	});

	// The nearer-device blob rule's ranking: only devices with a predicted pose in this camera.
	sample.device_ids_by_distance.fill(XRT_CONSTELLATION_INVALID_DEVICE_ID);
	sample.have_ranking = false;
	if (debug_get_bool_option_constellation_tracker_near_takes_blobs()) {
		uint32_t ranked = 0;
		for (uint32_t i = 0; i < device_count; i++) {
			const float d = device_state_camera_distance_sqrd(sample.getDeviceState(out_devices[i]->id).value_or(nullptr));
			if (std::isfinite(d)) {
				sample.device_ids_by_distance[ranked++] = out_devices[i]->id;
			}
		}
		sample.have_ranking = ranked > 1;
	}

	return device_count;
}

extern "C" void *
run_room_calibration_func(void *ptr)
{
	ConstellationTracker *tracker = static_cast<ConstellationTracker *>(ptr);
	auto &calib = tracker->rig_calibration;

	rig_calibration::CalibrationResult result;
	if (!calib->optimize(result)) {
		std::unique_lock<os::Mutex> lock(tracker->rig_calibration_lock);

		CT_ERROR(tracker, "Offline sensor calibration failed!");
		tracker->running_calibration = false;
		return nullptr;
	}

	for (const auto &calibration_camera : result.cameras) {
		if (calibration_camera.optimized) {
			auto &camera =
			    tracker->mosaics[calibration_camera.mosaic_idx]->cameras[calibration_camera.camera_idx];

			std::unique_lock<os::Mutex> cam_lock(camera->processing_lock);
			camera->locked_data.has_concrete_pose = true;
			// Copy the OpenCV pose in
			camera->locked_data.Txr_origin_cam = calibration_camera.Tcv_world_cam;
			// Convert to OpenXR
			math_pose_convert_from_opencv(&camera->locked_data.Txr_origin_cam,
			                              &camera->locked_data.Txr_origin_cam);
		}
	}

	{
		std::unique_lock<os::Mutex> lock(tracker->rig_calibration_lock);
		tracker->running_calibration = false;
	}

	std::shared_lock devices_lock(tracker->device_lock);
	for (auto &device : tracker->devices) {
		// Re-enable all devices now that setup is done.
		device->enabled.store(true);

		if (device->id == result.device_id) {
			device->sensor_fusion->setDeviceImuCalibration(sensor_fusion::DeviceImuCalibration{
			    .accel_bias = result.imu.accel_bias,
			    .gyro_bias = result.imu.gyro_bias,
			    .accel_scale = result.imu.accel_scale,
			    .Qcv_model_imu = result.imu.Qcv_model_imu,
			});
		}
	}

	return nullptr;
}

/*
 *
 * CameraSample implementations
 *
 */

std::optional<DeviceState *>
CameraSample::getDeviceState(t_constellation_device_id_t device_id)
{
	for (uint32_t i = 0; i < this->device_count; i++) {
		if (this->device_states[i].device_id == device_id) {
			return &this->device_states[i];
		}
	}

	return std::nullopt;
}

DeviceState &
CameraSample::putDeviceState(t_constellation_device_id_t device_id)
{
	assert(this->device_count < XRT_CONSTELLATION_MAX_DEVICES);

	DeviceState &new_device_state = this->device_states[this->device_count++];
	new_device_state = {
	    .device_id = device_id,
	    .Txr_world_device_prior = std::nullopt,
	    .found_pose = std::nullopt,
	    .needs_slow_processing = false,
	};

	return new_device_state;
}


CameraSample::CameraSample(t_blob_observation &blobservation, Camera *camera)
{
	// Copy the blob observation into this sample, since we need the data to be safe.
	this->source = blobservation.source;
	// Takes our own reference, the blob observation keeps its reference.
	this->trace_log = TraceLogRef(blobservation.trace_log);
	this->pipeline_latency_ns = os_monotonic_get_ns() - blobservation.timestamp_ns;
	this->id = blobservation.id;
	this->sequence_id = blobservation.sequence_id;
	this->timestamp_ns = blobservation.timestamp_ns;
	/*
	 * The only place blobs enter the tracker, so the bound holds for every array sized by
	 * XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME behind it (the correspondence search's among them).
	 */
	uint32_t num_blobs = blobservation.num_blobs;
	if (num_blobs > XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME) {
		static bool warned = false;
		if (!warned) {
			warned = true;
			U_LOG_W("Camera frame with %u blobs, the tracker holds %u: using the first %u", num_blobs,
			        (unsigned)XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME, (unsigned)XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME);
		}
		num_blobs = XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME;
	}
	memcpy(blobs, blobservation.blobs, sizeof(t_blob) * num_blobs);
	this->blob_count = num_blobs;

	// Get the camera pose
	this->Txr_world_cam = camera->getWorldPose(blobservation.timestamp_ns);

	this->device_states = {};
	this->device_count = 0;

	auto mosaic = camera->mosaic.lock();
	U_ASSERT_WEAK_PTR_THROW(mosaic, "Camera's mosaic was destroyed while we still had a sample referencing it");

	this->mosaic_index = mosaic->index;
	this->camera_index = camera->index;
}

void
CameraSample::markMatchingBlobs(ConstellationTracker *ct,
                                t_constellation_tracker_led_model &led_model,
                                t_constellation_device_id_t device_id,
                                pose_metrics_blob_match_info &blob_match_info)
{
	// First clear existing blob labels for this device
	for (uint32_t i = 0; i < this->blob_count; i++) {
		t_blob &b = this->blobs[i];

		// Skip blobs which already have an ID not belonging to this device
		if (b.matched_device_id != device_id) {
			continue;
		}

		if (b.matched_device_led_id != XRT_CONSTELLATION_INVALID_LED_ID) {
			// @todo is this needed?
			// b.prev_led_id = b.led_id;
		}

		b.matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
		b.matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
	}

	// Iterate the visible LEDs and mark matching blobs with this device ID and LED ID
	for (int i = 0; i < blob_match_info.num_visible_leds; i++) {
		pose_metrics_visible_led_info &led_info = blob_match_info.visible_leds[i];
		t_constellation_tracker_led *led = led_info.led;

		if (led_info.matched_blob != NULL) {
			t_blob *b = led_info.matched_blob;

			b->matched_device_led_id = led->id;
			b->matched_device_id = device_id;

			CT_DEBUG(ct, "Marking LED %d/%d at %f,%f angle %f now %d (was %d)", device_id, led->id,
			         b->center_distorted.x, b->center_distorted.y, RAD_TO_DEG(acosf(led_info.facing_dot)),
			         b->matched_device_led_id, /* b->prev_led_id */ -1);
		} else {
			CT_DEBUG(ct, "No blob for device %d LED %d @ %f,%f size %f px angle %f", device_id, led->id,
			         led_info.pos_px_undistorted.x, led_info.pos_px_undistorted.y,
			         2 * led_info.led_radius_px_undistorted, RAD_TO_DEG(acosf(led_info.facing_dot)));
		}
	}
}

/*
 *
 * Camera implementations
 *
 */

Camera::Camera(ConstellationTracker *tracker,
               std::weak_ptr<CameraMosaic> mosaic,
               const t_constellation_tracker_camera &camera_params,
               enum u_logging_level *log_level_ptr,
               uint32_t index)
    : tracker(tracker), mosaic(mosaic), calibration(camera_params.calibration), index(index)
{
	struct t_camera_model_params calib_true;
	t_camera_model_params_from_t_camera_calibration(&this->calibration, &calib_true);
	this->model = camera_model_init(calibration.image_size_pixels.w, calibration.image_size_pixels.h, &calib_true);

	this->locked_data = {
	    .Txr_origin_cam = camera_params.pose_in_origin,
	    .has_concrete_pose = camera_params.has_concrete_pose,
	};

	this->slow_processing_thread_data.cs = correspondence_search_new(log_level_ptr, &this->model);
	this->fast_processing_thread_data.cs = correspondence_search_new(log_level_ptr, &this->model);

	if (!this->tracker->deterministic) {
		if (os_thread_helper_init(&this->slow_processing_thread) < 0) {
			throw std::runtime_error("Slow processing thread failed to init");
		}
		if (os_thread_helper_init(&this->fast_processing_thread) < 0) {
			throw std::runtime_error("Fast processing thread failed to init");
		}

		if (os_thread_helper_start(&this->slow_processing_thread, constellation_tracker_camera_slow_thread,
		                           this) < 0) {
			throw std::runtime_error("Starting slow processing thread failed");
		}
		if (os_thread_helper_start(&this->fast_processing_thread, constellation_tracker_camera_fast_thread,
		                           this) < 0) {
			throw std::runtime_error("Starting fast processing thread failed");
		}
	}

	u_sink_debug_init(&this->slow_processing_thread_data.debug_sink);
	u_sink_debug_init(&this->fast_processing_thread_data.debug_sink);
}

Camera::~Camera()
{
	if (this->slow_processing_thread.initialized) {
		os_thread_helper_destroy(&this->slow_processing_thread);
	}

	if (this->fast_processing_thread.initialized) {
		os_thread_helper_destroy(&this->fast_processing_thread);
	}

	if (this->slow_processing_thread_data.cs) {
		correspondence_search_free(this->slow_processing_thread_data.cs);
		this->slow_processing_thread_data.cs = nullptr;
	}

	if (this->fast_processing_thread_data.cs) {
		correspondence_search_free(this->fast_processing_thread_data.cs);
		this->fast_processing_thread_data.cs = nullptr;
	}

	u_sink_debug_destroy(&this->slow_processing_thread_data.debug_sink);
	u_sink_debug_destroy(&this->fast_processing_thread_data.debug_sink);
}

std::optional<xrt_pose>
Camera::getWorldPose(timepoint_ns when_ns)
{
	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(
	    mosaic, "Camera's mosaic was destroyed while we still had a pointer to it, this should never happen",
	    std::nullopt);

	auto Txr_world_origin = mosaic->getTrackingOriginPose(when_ns);
	if (!Txr_world_origin.has_value()) {
		return std::nullopt;
	}

	std::unique_lock<os::Mutex> lock(this->processing_lock);

	if (!this->locked_data.has_concrete_pose) {
		return std::nullopt;
	}

	xrt_pose Txr_world_cam;
	math_pose_transform(&Txr_world_origin.value(), &this->locked_data.Txr_origin_cam, &Txr_world_cam);
	return Txr_world_cam;
}

void
Camera::deferSampleToSlowThread(CameraSample &sample)
{
	os_thread_helper_lock(&this->slow_processing_thread);
	{
		if (this->slow_processing_thread_data.sample.has_value()) {
			// Warn that we're dropping a frame.
			this->tracker->diag.slow_drops.fetch_add(1, std::memory_order_relaxed);
			CT_WARN(this->tracker,
			        "Dropping slow sample %" PRIu64 " at ts %" PRIi64 ". Tracker is likely running slow.",
			        this->slow_processing_thread_data.sample->id,
			        this->slow_processing_thread_data.sample->timestamp_ns);
		}

		this->slow_processing_thread_data.sample = sample;
		os_thread_helper_signal_locked(&this->slow_processing_thread);
	}
	os_thread_helper_unlock(&this->slow_processing_thread);
}

bool
Camera::tryDevicePose(Device *device,
                      CameraSample &sample,
                      DeviceState &device_state,
                      bool need_matching_ids,
                      const std::optional<xrt_space_relation> &Tcv_cam_device_prior,
                      const xrt_pose &Tcv_cam_device_candidate,
                      t_constellation_tracker_trace_log_solve_type solve_type)
{
	bool success;
	pose_metrics score;
	if (Tcv_cam_device_prior.has_value()) {
		success = pose_metrics_evaluate_pose_with_prior_ranked( //
		    &score,                                      //
		    &Tcv_cam_device_candidate,                   //
		    false,                                       //
		    &Tcv_cam_device_prior.value().pose,          //
		    &device->prior_pos_error,                    //
		    &device->prior_rot_error,                    //
		    sample.blobs,                                //
		    sample.blob_count,                           //
		    &device->params.led_model,                   //
		    device->id, sample.nearRanking(),                                  //
		    &this->model,                                //
		    NULL);                                       //
	} else {
		success = pose_metrics_evaluate_pose_ranked( //
		    &score,                           //
		    &Tcv_cam_device_candidate,        //
		    sample.blobs,                     //
		    sample.blob_count,                //
		    &device->params.led_model,        //
		    device->id, sample.nearRanking(),                       //
		    &this->model,                     //
		    NULL);                            //
	}

	// The actual call itself failed.
	if (!success) {
		return false;
	}

	bool was_good = POSE_HAS_FLAGS(&score, POSE_MATCH_GOOD | (need_matching_ids ? POSE_MATCH_LED_IDS : 0));
	bool was_strong = POSE_HAS_FLAGS(&score, POSE_MATCH_STRONG);

	if (was_good || was_strong) {
		this->pushPose(               //
		    sample,                   //
		    device_state,             //
		    device,                   //
		    score,                    //
		    Tcv_cam_device_candidate, //
		    false,                    // was_optimized
		    solve_type);              //
		return true;
	}

	return false;
}

/*!
 * The prior is usually a few degrees / cm off when the fast path fails (median 5 deg / 2 cm, 12-25 px, at any speed on
 * the 23:01 run replay), and in about half of those frames one image shift lines most of its LEDs up with blobs. Find
 * the shift with the most LEDs on a blob, label those blobs and let the blob recovery (RANSAC-PnP from the prior, then
 * the usual pose checks) solve from them. Labels it set are removed again if that fails.
 */
bool
Camera::tryDevicePriorSnap(Device *device, CameraSample &sample, DeviceState &device_state, const xrt_space_relation &prior)
{
	pose_metrics_blob_match_info info;
	pose_metrics_match_pose_to_blobs_ranked(&prior.pose, sample.blobs, sample.blob_count, &device->params.led_model,
	                                 device->id, sample.nearRanking(), &this->model, &info);
	if (info.num_visible_leds < 4) {
		return false;
	}

	const float radius = debug_get_float_option_constellation_tracker_prior_snap_radius_px();
	// Candidate blobs: not claimed by another device.
	std::array<int, XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME> cand{};
	int nc = 0;
	for (uint32_t b = 0; b < sample.blob_count && nc < (int)cand.size(); b++) {
		const t_blob &blob = sample.blobs[b];
		if (blob.matched_device_id == XRT_CONSTELLATION_INVALID_DEVICE_ID || blob.matched_device_id == device->id) {
			cand[nc++] = (int)b;
		}
	}
	if (nc < 4) {
		return false;
	}

	// Shift votes: every (LED, blob) pair within the radius proposes blob - LED; count LEDs with a blob near each.
	// Shift candidates with their support, to pick the best and check it's unambiguous.
	struct cand_shift
	{
		xrt_vec2 shift;
		int support;
	};
	std::vector<cand_shift> shifts;
	int best_support = 0;
	xrt_vec2 best_shift = {0, 0};
	for (int l = 0; l < info.num_visible_leds; l++) {
		const xrt_vec2 lp = info.visible_leds[l].pos_px_undistorted;
		for (int c = 0; c < nc; c++) {
			const xrt_vec2 bp = sample.blobs[cand[c]].center_undistorted;
			xrt_vec2 shift = {bp.x - lp.x, bp.y - lp.y};
			if (shift.x * shift.x + shift.y * shift.y > radius * radius) {
				continue;
			}
			int support = 0;
			for (int l2 = 0; l2 < info.num_visible_leds; l2++) {
				const xrt_vec2 q = {info.visible_leds[l2].pos_px_undistorted.x + shift.x,
				                    info.visible_leds[l2].pos_px_undistorted.y + shift.y};
				float tol = fmaxf(4.0f, (float)info.visible_leds[l2].led_radius_px_undistorted * 1.5f);
				for (int c2 = 0; c2 < nc; c2++) {
					const xrt_vec2 b2 = sample.blobs[cand[c2]].center_undistorted;
					if ((b2.x - q.x) * (b2.x - q.x) + (b2.y - q.y) * (b2.y - q.y) <= tol * tol) {
						support++;
						break;
					}
				}
			}
			shifts.push_back({shift, support});
			if (support > best_support) {
				best_support = support;
				best_shift = shift;
			}
		}
	}
	/*
	 * Unambiguous only: a shift one LED spacing off can line up nearly as many LEDs (the ring is regular), and solving
	 * from it gave single-frame 9-16 cm spikes that pass every pose check. Require 5 LEDs and 2 more than any clearly
	 * different shift.
	 */
	const bool strict = debug_get_num_option_constellation_tracker_prior_snap_mode() == 2;
	if (best_support < (strict ? 5 : 4)) {
		return false;
	}
	int second = 0;
	for (const auto &cs : shifts) {
		float dx = cs.shift.x - best_shift.x, dy = cs.shift.y - best_shift.y;
		if (dx * dx + dy * dy > 8.0f * 8.0f && cs.support > second) {
			second = cs.support;
		}
	}
	if (strict && best_support - second < 2) {
		return false;
	}

	// Label: each LED takes the nearest free blob within its tolerance after the shift.
	for (int c = 0; c < nc; c++) {
		t_blob &blob = sample.blobs[cand[c]];
		blob.matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
		blob.matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
	}
	int labelled = 0;
	for (int l = 0; l < info.num_visible_leds; l++) {
		const xrt_vec2 q = {info.visible_leds[l].pos_px_undistorted.x + best_shift.x,
		                    info.visible_leds[l].pos_px_undistorted.y + best_shift.y};
		float tol = fmaxf(4.0f, (float)info.visible_leds[l].led_radius_px_undistorted * 1.5f);
		int best = -1;
		float best_d = tol * tol;
		for (int c = 0; c < nc; c++) {
			const t_blob &blob = sample.blobs[cand[c]];
			if (blob.matched_device_id != XRT_CONSTELLATION_INVALID_DEVICE_ID) {
				continue;
			}
			float d = (blob.center_undistorted.x - q.x) * (blob.center_undistorted.x - q.x) +
			          (blob.center_undistorted.y - q.y) * (blob.center_undistorted.y - q.y);
			if (d <= best_d) {
				best_d = d;
				best = cand[c];
			}
		}
		if (best >= 0) {
			sample.blobs[best].matched_device_id = device->id;
			sample.blobs[best].matched_device_led_id = info.visible_leds[l].led->id;
			labelled++;
		}
	}

	if (!strict && labelled >= 4) {
		/*
		 * As the blob recovery (RANSAC-PnP from the prior, trusted with over 4 blobs or a good match), plus a limit
		 * on how far the solve may move from the prior: the replays' good snaps stayed within 2 cm / 6.7 deg (p90),
		 * the rare single-frame spikes landed 9-16 cm from the previous fix.
		 */
		xrt_pose solved;
		if (optimizePose(this->tracker->log_level, this->tracker->deterministic, this->model, prior.pose,
		                 sample.blobs, sample.blob_count, &device->params.led_model, device->id, solved)) {
			xrt_vec3 d = solved.position;
			math_vec3_subtract(&prior.pose.position, &d);
			xrt_quat inv, dq;
			math_quat_invert(&prior.pose.orientation, &inv);
			math_quat_rotate(&inv, &solved.orientation, &dq);
			float rot_deg = RAD_TO_DEG(2.0f * acosf(fminf(1.0f, fabsf(dq.w))));
			pose_metrics score;
			bool ok = pose_metrics_evaluate_pose_with_prior_ranked(&score, &solved, true, &prior.pose,
			                                                &device->prior_pos_error, &device->prior_rot_error,
			                                                sample.blobs, sample.blob_count,
			                                                &device->params.led_model, device->id, sample.nearRanking(), &this->model,
			                                                NULL);
			if (ok && (score.matched_blobs > 4 || POSE_HAS_FLAGS(&score, POSE_MATCH_GOOD)) &&
			    m_vec3_len(d) <= debug_get_float_option_constellation_tracker_prior_snap_max_m() &&
			    rot_deg <= debug_get_float_option_constellation_tracker_prior_snap_max_deg()) {
				this->pushPose(sample, device_state, device, score, solved, true,
				               CONSTELLATION_SOLVE_TYPE_BLOB_LABELS);
				if (device_state.found_pose.has_value()) {
					CT_DEBUG(this->tracker,
					         "Prior snap for device %d: shift (%.0f, %.0f) px, %d labelled, %.3f m %.1f deg "
					         "from the prior, solved",
					         device->id, best_shift.x, best_shift.y, labelled, m_vec3_len(d), rot_deg);
					return true;
				}
			}
		}
	}
	if (strict && labelled >= 4) {
		/*
		 * Solve from the labels, but unlike the blob recovery (which trusts any RANSAC solve with more than 4 blobs)
		 * require a good match against the prior: a shift one LED spacing off still RANSACs cleanly onto the wrong
		 * LEDs, and trusting that gave single-frame 9-16 cm spikes on the replays (10 in 2 sessions).
		 */
		xrt_pose solved;
		if (optimizePose(this->tracker->log_level, this->tracker->deterministic, this->model, prior.pose,
		                 sample.blobs, sample.blob_count, &device->params.led_model, device->id, solved)) {
			pose_metrics score;
			// The fit alone: the prior's own error is what the snap bridges, so its agreement test can't be used.
			bool ok = pose_metrics_evaluate_pose_ranked(&score, &solved, sample.blobs, sample.blob_count,
			                                     &device->params.led_model, device->id, sample.nearRanking(), &this->model, NULL);
			if (ok && POSE_HAS_FLAGS(&score, POSE_MATCH_GOOD)) {
				this->pushPose(sample, device_state, device, score, solved, true,
				               CONSTELLATION_SOLVE_TYPE_BLOB_LABELS);
				if (device_state.found_pose.has_value()) {
					xrt_vec3 d = solved.position;
					math_vec3_subtract(&prior.pose.position, &d);
					xrt_quat inv, dq;
					math_quat_invert(&prior.pose.orientation, &inv);
					math_quat_rotate(&inv, &solved.orientation, &dq);
					CT_DEBUG(this->tracker,
					         "Prior snap t %" PRId64 " device %d: shift (%.0f, %.0f) px, support %d vs %d, %d "
					         "labelled, %u matched of %u visible, reproj %.2f, from prior %.3f m %.1f deg, z %.2f",
					         sample.timestamp_ns, device->id, best_shift.x, best_shift.y, best_support, second,
					         labelled, score.matched_blobs, score.visible_leds, score.reprojection_error,
					         m_vec3_len(d), RAD_TO_DEG(2.0f * acosf(fminf(1.0f, fabsf(dq.w)))), solved.position.z);
					return true;
				}
			}
		}
	}

	for (int c = 0; c < nc; c++) {
		t_blob &blob = sample.blobs[cand[c]];
		if (blob.matched_device_id == device->id) {
			blob.matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
			blob.matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
		}
	}
	return false;
}

bool
Camera::tryDeviceBlobRecovery(Device *device,
                              CameraSample &sample,
                              DeviceState &device_state,
                              const std::optional<xrt_space_relation> &Tcv_cam_device_prior)
{
	auto tracker = this->tracker;

	const uint32_t needed_blobs = 4;
	uint32_t num_blobs = 0;
	for (uint32_t index = 0; index < sample.blob_count; index++) {
		t_blob &b = sample.blobs[index];
		if (b.matched_device_id == device->id) {
			num_blobs++;

			if (num_blobs >= needed_blobs) {
				break;
			}
		}
	}
	if (num_blobs < needed_blobs) {
		return false;
	}

	// RANSAC-PnP with the matched blobs
	// The optimization should still be able to run with an identity pose.
	xrt_pose Tcv_cam_device =
	    Tcv_cam_device_prior
	        .value_or(xrt_space_relation{
	            .relation_flags = static_cast<xrt_space_relation_flags>(0),
	            .pose =
	                {
	                    .orientation = {.x = 0, .y = 1, .z = 0, .w = 0}, // 180 around Y, facing the camera
	                    .position = {0, 0, 1},
	                },
	            .linear_velocity = {},
	            .angular_velocity = {},
	        })
	        .pose;
	xrt_pose Tcv_cam_device_optimized;
	bool success = optimizePose(   //
	    tracker->log_level,        //
	    tracker->deterministic,    //
	    this->model,               //
	    Tcv_cam_device,            //
	    sample.blobs,              //
	    sample.blob_count,         //
	    &device->params.led_model, //
	    device->id,                //
	    Tcv_cam_device_optimized); //
	if (!success) {
		CT_DEBUG(tracker, "Camera %p RANSAC-PnP blob recovery for device %d from %u blobs failed", (void *)this,
		         device->id, sample.blob_count);
		return false;
	}

	CT_DEBUG(tracker, "Camera %p RANSAC-PnP blob recovery for device %d from %u blobs succeeded", (void *)this,
	         device->id, sample.blob_count);
	Tcv_cam_device = Tcv_cam_device_optimized;

	pose_metrics score;
	// Evaluate the pose, using the prior if available
	if (Tcv_cam_device_prior.has_value()) {
		success = pose_metrics_evaluate_pose_with_prior_ranked( //
		    &score,                                      //
		    &Tcv_cam_device,                             //
		    true,                                        //
		    &Tcv_cam_device_prior.value().pose,          //
		    &device->prior_pos_error,                    //
		    &device->prior_rot_error,                    //
		    sample.blobs,                                //
		    sample.blob_count,                           //
		    &device->params.led_model,                   //
		    device->id, sample.nearRanking(),                                  //
		    &this->model,                                //
		    NULL);
	} else {
		success = pose_metrics_evaluate_pose_ranked(&score, &Tcv_cam_device, sample.blobs, sample.blob_count,
		                                     &device->params.led_model, device->id, sample.nearRanking(), &this->model, NULL);
	}

	// If the evaluate failed entirely, this isn't a success.
	if (!success) {
		return false;
	}

	/*
	 * @note I saw some cases where an otherwise valid solve was dropped because we didn't trust the RANSAC solve.
	 *       I don't think this will make things *worse*, but it seems to at least get some more valid solves
	 *       through, so this is fine for now, set as a constexpr bool for easy tweaking purposes.
	 *       Since a RANSAC has already made an assessment over the validity of the pose, and we already trust that
	 *       assessment, this shouldn't cause many problems. RANSAC is also only meaningful if more than 4 blobs
	 *       were matched in the RANSAC run, otherwise RANSAC did not even run.
	 */
	constexpr bool kAlwaysTrustRansac = true;

	/*
	 * Labels carried from earlier frames (or set by the prior snap) are occasionally wrong, and RANSAC fits them
	 * cleanly: with the snap on, all of the replays' single-frame 9-16 cm spikes were blob-label solves. A solve that
	 * far from the prior isn't the device moving.
	 */
	if (debug_get_bool_option_constellation_tracker_blob_recovery_prior_gate() && Tcv_cam_device_prior.has_value()) {
		const xrt_pose &pr = Tcv_cam_device_prior->pose;
		xrt_vec3 d = Tcv_cam_device.position;
		math_vec3_subtract(&pr.position, &d);
		xrt_quat inv, dq;
		math_quat_invert(&pr.orientation, &inv);
		math_quat_rotate(&inv, &Tcv_cam_device.orientation, &dq);
		float rot_deg = RAD_TO_DEG(2.0f * acosf(fminf(1.0f, fabsf(dq.w))));
		if (m_vec3_len(d) > debug_get_float_option_constellation_tracker_prior_snap_max_m() ||
		    rot_deg > debug_get_float_option_constellation_tracker_prior_snap_max_deg()) {
			CT_DEBUG(tracker, "Blob recovery for device %d rejected: %.3f m / %.1f deg from the prior", device->id,
			         m_vec3_len(d), rot_deg);
			return false;
		}
	}

	if ((kAlwaysTrustRansac && score.matched_blobs > 4) || POSE_HAS_FLAGS(&score, POSE_MATCH_GOOD)) {
		CT_DEBUG(tracker, "Camera %p RANSAC-PnP recovered pose was good enough for device %d from %u blobs",
		         (void *)this, device->id, sample.blob_count);

		this->pushPose(                            //
		    sample,                                //
		    device_state,                          //
		    device,                                //
		    score,                                 //
		    Tcv_cam_device,                        //
		    true,                                  // was_optimized
		    CONSTELLATION_SOLVE_TYPE_BLOB_LABELS); //

		return true;
	} else {
		CT_DEBUG(tracker, "Camera %p RANSAC-PnP recovered pose for device %d from %u blobs was not good enough",
		         (void *)this, device->id, sample.blob_count);
	}

	return false;
}

void
Camera::handleStepComplete(CameraSample &sample)
{
	auto tracker = this->tracker;

#ifdef XRT_FEATURE_RERUN
	if (tracker->rerun_stream) {
		tracker->rerun_stream->logSample(*tracker, sample);
	}
#endif

	// Both callers hold `tracker->device_lock`, which is what lets this resolve the sample's devices.
	tracker->rig_calibration->pushCameraSample(tracker, sample);

	if (!tracker->checkSensorCalibration()) {
		CT_ERROR(tracker, "Sensor tried to do calibration but it failed.");
	}

	for (uint32_t i = 0; i < sample.device_count; i++) {
		auto &sample_device = sample.device_states[i];
		auto device_iter = std::find_if(
		    tracker->devices.begin(), tracker->devices.end(),
		    [&sample_device](const auto &device) { return sample_device.device_id == device->id; });
		// Skip removed devices
		if (device_iter == tracker->devices.end()) {
			continue;
		}
		auto &device = *device_iter;

		// Mark the device's found pose as handled.
		if (auto &found_pose = sample_device.found_pose) {
			if (!found_pose->handled) {
				device->sensor_fusion->pushCameraSample(tracker, sample);
			}

			found_pose->handled = true;
		}
	}
}

void
Camera::handleProcessingComplete(CameraSample &sample)
{
	if (sample.trace_log) {
		struct t_constellation_tracker_trace_log_matched_device devices[XRT_CONSTELLATION_MAX_DEVICES] = {};
		for (auto &trace_device : devices) {
			trace_device.device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
		}

		for (uint32_t i = 0; i < sample.device_count; i++) {
			const auto &device_state = sample.device_states[i];
			auto &trace_device = devices[i];
			trace_device.device_id = device_state.device_id;
			trace_device.device_solve_type = device_state.solve_type;
		}

		t_constellation_tracker_trace_log_entry entry{
		    .type = CONSTELLATION_TRACE_LOG_ENTRY_MATCHING,
		    .matching =
		        {
		            .pipeline_latency_ns = sample.pipeline_latency_ns,
		            .fast_taken = sample.fast_duration_ns.has_value(),
		            .fast_duration_ns = sample.fast_duration_ns.value_or(-1),
		            .slow_taken = sample.slow_duration_ns.has_value(),
		            .slow_duration_ns = sample.slow_duration_ns.value_or(-1),
		            .devices = {},
		        },
		};
		mempcpy(&entry.matching.devices, devices, sizeof(devices));
		t_constellation_tracker_trace_log_push_entry(sample.trace_log.get(), &entry);
	}
}

void
Camera::processSampleSlow(CameraSample &sample)
{
	ConstellationTracker *tracker = this->tracker;

	auto &data = this->slow_processing_thread_data;

	CT_TRACE(tracker, "Starting slow processing for camera %p with %u blobs", (void *)this, sample.blob_count);
	const auto process_start_ns = os_monotonic_get_ns();

	correspondence_search_set_blobs(data.cs, sample.blobs, sample.blob_count);
	data.cs->ranking_valid = sample.have_ranking;
	std::memcpy(data.cs->device_ids_by_distance, sample.device_ids_by_distance.data(),
	            sizeof(data.cs->device_ids_by_distance));

	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(mosaic,
	                      "Camera mosaic was destroyed while processing a sample, this should never happen since "
	                      "the mosaic owns the camera");

	std::shared_lock lock(tracker->device_lock);

	auto Txr_world_cam = sample.Txr_world_cam;

	// Borrowed pointers, only valid while we hold the device lock taken above.
	std::array<Device *, XRT_CONSTELLATION_MAX_DEVICES> devices{};
	const uint32_t device_count = collect_devices_nearest_first(tracker, sample, devices);

	bool any_shallow_failed = false;
	const uint64_t slow_n = this->slow_samples++;
	const int64_t lost_every = debug_get_num_option_constellation_tracker_lost_search_every();

	for (int pass = 0; pass < 2; pass++) {
		for (uint32_t device_iter = 0; device_iter < device_count; device_iter++) {
			Device *device = devices[device_iter];

			// Device is not enabled.
			if (!device->enabled.load()) {
				continue;
			}
			// LEDs not synchronised to the exposures: a search can't find them, see is_led_synced.
			if (device->device->is_led_synced != NULL && !device->device->is_led_synced(device->device)) {
				continue;
			}
			bool shallow = pass == 0;

			// Do a shallow search first go around
			correspondence_search_flags search_flags =
			    shallow ? CS_FLAG_SHALLOW_SEARCH : CS_FLAG_DEEP_SEARCH;

			search_flags = (correspondence_search_flags)(search_flags | CS_FLAG_STOP_FOR_STRONG_MATCH);

			DeviceState *device_state_ptr = sample.getDeviceState(device->id).value_or(nullptr);
			// If there was no device state in the sample, that means this device appeared after the
			// constellation tracker started this sample, so we need to fill out the device state here.
			if (device_state_ptr == nullptr) {
				device_state_ptr = &sample.putDeviceState(device->id);

				// we need to do a slow process for this device since it wasn't present in the fast
				// processing
				device_state_ptr->needs_slow_processing = true;
			}
			DeviceState &device_state = *device_state_ptr;

			if (!device_state.needs_slow_processing) {
				// we already did a fast process for this device and it succeeded, so there is no need
				// to do a slow search
				continue;
			}

			// Check if there's some global reason we should skip slow processing
			{
				std::lock_guard<os::Mutex> lock(device->data_lock);

				bool enable_slow_skip =
				    device->locked_data.last_known_pose.has_value() && // device has been seen
				    Txr_world_cam.has_value();                         // we have a pose

				if (enable_slow_skip) {
					const auto &last_known_pose = *device->locked_data.last_known_pose;

					time_duration_ns time_since_solve_ns =
					    sample.timestamp_ns - last_known_pose.timestamp_ns;

					if (time_since_solve_ns <
					    (U_TIME_1MS_IN_NS * debug_get_num_option_constellation_tracker_slow_skip_ms())) {
						// If the device has been seen in the last 150ms, don't try to brute
						// force search it, hope that it will be seen by a fast solve in that
						// time period.
						device_state.solve_type =
						    CONSTELLATION_SOLVE_TYPE_NO_SOLVE_FAST_SOLVED_RECENTLY;
						continue;
					} else if (debug_get_num_option_constellation_tracker_deep_delay_ms() > 0 &&
					           (time_since_solve_ns <
					                (U_TIME_1MS_IN_NS *
					                 debug_get_num_option_constellation_tracker_deep_delay_ms()) ||
					            this->shallow_searches_failed.load() < 10) &&
					           (search_flags & CS_FLAG_DEEP_SEARCH)) {
						// Don't do a deep search until a shallow search has failed for one full
						// second, or we've failed less than 10 shallow searches for any device.
						device_state.solve_type =
						    CONSTELLATION_SOLVE_TYPE_NO_SOLVE_DELAYED_DEEP;
						continue;
					}
				}
			}


			// A lost device: this camera searches for it only every lost_every-th sample, in turn with the others.
			if (lost_every > 1) {
				bool lost = true;
				{
					std::lock_guard<os::Mutex> lock(device->data_lock);
					if (device->locked_data.last_known_pose.has_value()) {
						const time_duration_ns since =
						    sample.timestamp_ns - device->locked_data.last_known_pose->timestamp_ns;
						lost = since >
						       (time_duration_ns)(U_TIME_1MS_IN_NS *
						                          debug_get_num_option_constellation_tracker_lost_ms());
					}
				}
				if (lost && ((slow_n + this->index + (uint64_t)device->id) % (uint64_t)lost_every) != 0) {
					device_state.solve_type = CONSTELLATION_SOLVE_TYPE_NO_SOLVE_DELAYED_DEEP;
					continue;
				}
			}

			xrt_pose Tcv_cam_device = XRT_POSE_IDENTITY;
			if (device_state.Txr_world_device_prior.has_value() && Txr_world_cam.has_value()) {
				xrt_pose Txr_cam_world;
				math_pose_invert(&Txr_world_cam.value(), &Txr_cam_world);

				xrt_pose Txr_cam_device;
				math_pose_transform(&Txr_cam_world, &device_state.Txr_world_device_prior.value(),
				                    &Txr_cam_device);

				math_pose_convert_from_opencv(&Txr_cam_device, &Tcv_cam_device);

				search_flags = (correspondence_search_flags)(search_flags | CS_FLAG_HAVE_POSE_PRIOR);
			}

			// Arbitrary threshold to prevent trusting a gravity vector if the device itself isn't confident
			// in it's own gravity.
			const float gravity_error_threshold_rad = DEG_TO_RAD(25.f);

			xrt_vec3 cv_camera_gravity_vector = {0.0, 1.0, 0.0};
			if ((search_flags & CS_FLAG_HAVE_POSE_PRIOR) != 0 &&
			    device->gravity_error_rad < gravity_error_threshold_rad) {
				// If we have a pose for the camera and we have a prior pose
				// (required by correspondence for search gravity matching)
				if (Txr_world_cam.has_value()) {
					xrt_pose Tcv_world_cam;
					math_pose_convert_from_opencv(&Txr_world_cam.value(), &Tcv_world_cam);

					// Acquire the camera's gravity vector under the processing lock
					get_pose_gravity_vector(Tcv_world_cam, cv_camera_gravity_vector);

					// Add in to check gravity
					search_flags =
					    (correspondence_search_flags)(search_flags | CS_FLAG_MATCH_GRAVITY);
				}
			}

			/*
			 * The device IMU's up, for the search's own check of every candidate (see correspondence_search.c):
			 * needs the camera's pose for its up direction, and works without a pose prior.
			 */
			xrt_vec3 imu_up_in_device, up_in_camera = {0, 1, 0};
			bool have_imu_up = false;
			xrt_quat q_imu;
			if (tracker->gravity_gate_deg > 0.0f && Txr_world_cam.has_value() &&
			    device->device->get_imu_orientation != NULL &&
			    device->device->get_imu_orientation(device->device, sample.timestamp_ns, &q_imu)) {
				xrt_pose Tcv_world_cam;
				math_pose_convert_from_opencv(&Txr_world_cam.value(), &Tcv_world_cam);
				get_pose_gravity_vector(Tcv_world_cam, up_in_camera);
				imu_up_in_device = device->imuUpInDevice(q_imu);
				have_imu_up = true;
			}

			pose_metrics score;
			diag_dev_inc(tracker, device->id, &TrackerDiag::Dev::slow_searches);
			bool found_pose = correspondence_search_find_one_pose( //
			    data.cs,                                           //
			    device->search_model,                              //
			    search_flags,                                      //
			    &Tcv_cam_device,                                   //
			    &device->prior_pos_error,                          //
			    &device->prior_rot_error,                          //
			    &cv_camera_gravity_vector,                         //
			    device->gravity_error_rad,                         //
			    have_imu_up ? &imu_up_in_device : NULL,            //
			    have_imu_up ? &up_in_camera : NULL,                //
			    DEG_TO_RAD(tracker->gravity_gate_deg),             //
			    &score);                                           //
			if (found_pose) {
				diag_dev_inc(tracker, device->id, &TrackerDiag::Dev::slow_found);
				const auto solve_type = shallow ? CONSTELLATION_SOLVE_TYPE_SHALLOW_CORRESPONDENCE_SEARCH
				                                : CONSTELLATION_SOLVE_TYPE_DEEP_CORRESPONDENCE_SEARCH;

				this->pushPose(     //
				    sample,         //
				    device_state,   //
				    device,         //
				    score,          //
				    Tcv_cam_device, //
				    false,          // was_optimized
				    solve_type);    //

				// We found a pose for this device in this sample
				device_state.needs_slow_processing = false;
			} else {
				CT_TRACE(tracker, "Camera %p slow processing for device %d failed to find a pose",
				         (void *)this, device->id);

				if (shallow) {
					any_shallow_failed = true;
				}

				// No solve was found. May get overwritten by the next iteration
				device_state.solve_type = CONSTELLATION_SOLVE_TYPE_NO_SOLVE;
			}
		}
	}

	if (any_shallow_failed) {
		this->shallow_searches_failed.fetch_add(1);
	} else {
		this->shallow_searches_failed.store(0);
	}

	for (uint32_t i = 0; i < sample.device_count; i++) {
		auto &device = sample.device_states[i];

		device.needs_slow_processing = false;
	}

	this->debugScribbleSample(sample, false);

	this->handleStepComplete(sample);

	sample.slow_duration_ns = os_monotonic_get_ns() - process_start_ns;
	this->handleProcessingComplete(sample);
}

bool
Camera::processSampleFast(CameraSample &sample)
{
	ConstellationTracker *tracker = this->tracker;

	CT_TRACE(tracker, "Starting fast processing for camera %p with %u blobs", (void *)this, sample.blob_count);
	const auto process_start_ns = os_monotonic_get_ns();

	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(mosaic,
	                      "Camera mosaic was destroyed while processing a sample, this should never happen since "
	                      "the mosaic owns the camera",
	                      false);

	auto Txr_world_cam = sample.Txr_world_cam;
	auto Tcv_world_cam = Txr_world_cam;
	auto Tcv_cam_world = Txr_world_cam;
	if (Txr_world_cam.has_value()) {
		math_pose_convert_from_opencv(&Txr_world_cam.value(), &Tcv_world_cam.value());
		math_pose_invert(&Tcv_world_cam.value(), &Tcv_cam_world.value());
	}

	std::shared_lock lock(tracker->device_lock);

	// Figure out where all the devices are first, so we can then sort it.
	for (std::unique_ptr<Device> &device : tracker->devices) {
		xrt_space_relation device_predicted_relation = XRT_SPACE_RELATION_ZERO; //< AKA "the prior"
		device->sensor_fusion->getKeyframeTrackedPose(sample.timestamp_ns, device_predicted_relation);

		bool prior_pose_valid =
		    (device_predicted_relation.relation_flags &
		     (XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT)) ==
		    (XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT);

		std::optional<xrt_space_relation> Tcv_cam_device_predicted = std::nullopt; //< AKA "the prior"
		// Whether the prior pose is actually valid and we have a pose for the camera
		if (Tcv_cam_world.has_value() && prior_pose_valid) {
			xrt_space_relation Tcv_world_device_predicted = device_predicted_relation;
			math_space_relation_convert_from_opencv(&device_predicted_relation,
			                                        &Tcv_world_device_predicted);

			Tcv_cam_device_predicted.emplace(Tcv_world_device_predicted); // initialize it to a value
			math_pose_transform(&Tcv_cam_world.value(), &Tcv_world_device_predicted.pose,
			                    &Tcv_cam_device_predicted.value().pose);
			// @note technically we need to transform the velocities here but we aren't actually reading the
			//       values, just the magnitude rn so its fine.
		}

		auto &device_state = sample.putDeviceState(device->id);
		device_state.Txr_world_device_prior =
		    prior_pose_valid ? std::optional<xrt_pose>(device_predicted_relation.pose) : std::nullopt;
		device_state.Tcv_cam_device_predicted = Tcv_cam_device_predicted;
	}

	// Borrowed pointers, only valid while we hold the device lock taken above.
	std::array<Device *, XRT_CONSTELLATION_MAX_DEVICES> devices{};
	const uint32_t device_count = collect_devices_nearest_first(tracker, sample, devices);

	bool need_slow_search = false;
	for (uint32_t device_iter = 0; device_iter < device_count; device_iter++) {
		Device *device = devices[device_iter];

		// Device is not enabled.
		if (!device->enabled.load()) {
			continue;
		}

		// The first pass above put a state in the sample for every device we are about to walk.
		DeviceState &device_state = *sample.getDeviceState(device->id).value();

		const auto Tcv_cam_device_predicted = device_state.Tcv_cam_device_predicted; //< AKA "the prior"

		bool do_blob_recovery = true;
		if (Tcv_cam_device_predicted.has_value()) {
			// If the device is moving more than 5 meters per second, or it's rotating at more than one
			// rotation every second, then skip blob recovery, and also wipe associations, that's too fast
			// to keep sane blob associations.
			if (m_vec3_len(Tcv_cam_device_predicted->linear_velocity) > 5 ||
			    m_vec3_len(Tcv_cam_device_predicted->angular_velocity) >
			        DEG_TO_RAD(debug_get_float_option_constellation_tracker_blob_recovery_max_dps())) {
				do_blob_recovery = false;
			}
		}

		// if we have a valid prior pose, try to use it for fast matching
		if (Tcv_cam_device_predicted.has_value() &&                    //
		    this->tryDevicePose(device,                                //
		                        sample,                                //
		                        device_state,                          //
		                        false,                                 //
		                        std::nullopt,                          //
		                        Tcv_cam_device_predicted.value().pose, //
		                        CONSTELLATION_SOLVE_TYPE_PREDICTED_PRIOR)) {
			diag_dev_inc(tracker, device->id, &TrackerDiag::Dev::fast_found);
			CT_DEBUG(tracker, "Fast processing for device %d succeeded with predicted pose", device->id);
			continue; // try the next device, we found a pose!
		}

		// Try to use blobs after we've checked the prior itself
		bool wipe_blob_associations = false;
		if (do_blob_recovery &&                       //
		    this->tryDeviceBlobRecovery(device,       //
		                                sample,       //
		                                device_state, //
		                                Tcv_cam_device_predicted)) {
			diag_dev_inc(tracker, device->id, &TrackerDiag::Dev::fast_found);
			CT_DEBUG(tracker, "Fast processing for device %d succeeded with blob recovery", device->id);
			continue; // try the next device, we found a pose!
		} else {
			wipe_blob_associations = true;
		}

		if (debug_get_bool_option_constellation_tracker_prior_snap() && Tcv_cam_device_predicted.has_value() &&
		    this->tryDevicePriorSnap(device, sample, device_state, Tcv_cam_device_predicted.value())) {
			diag_dev_inc(tracker, device->id, &TrackerDiag::Dev::fast_found);
			CT_DEBUG(tracker, "Fast processing for device %d succeeded with a prior snap", device->id);
			continue;
		}

		// Try to get a last known pose
		std::optional<xrt_pose> Tcv_cam_device_last_known = std::nullopt;
		{
			std::unique_lock<os::Mutex> lock(device->data_lock);

			// If we know where the camera is, we found any pose, and that pose had a valid world-space
			// pose.
			if (device->locked_data.last_known_pose.has_value()) {
				Tcv_cam_device_last_known = device->locked_data.last_known_pose->Tcv_cam_device;
			}
		}

		if (Tcv_cam_device_last_known.has_value() &&               //
		    this->tryDevicePose(device,                            //
		                        sample,                            //
		                        device_state,                      //
		                        true,                              //
		                        Tcv_cam_device_predicted,          //
		                        Tcv_cam_device_last_known.value(), //
		                        CONSTELLATION_SOLVE_TYPE_LAST_KNOWN_POSE)) {
			diag_dev_inc(tracker, device->id, &TrackerDiag::Dev::fast_found);
			CT_DEBUG(tracker, "Fast processing for device %d succeeded with last known pose", device->id);
			continue; // try the next device, we found a pose!
		}

		if (wipe_blob_associations) {
			// Blob-based recovery failed, unmark all blobs for this device.
			for (uint32_t i = 0; i < sample.blob_count; i++) {
				t_blob &b = sample.blobs[i];
				if (b.matched_device_id == device->id) {
					b.matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
					b.matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
				}
			}
		}

		/*
		 * The prior didn't match directly, but it's usually close: when the slow search found these, the prior was
		 * a median 5 deg / 2 cm off (12-25 px) at any speed (23:01 run replay). The slow search's prior-seeded
		 * shallow search finds them; doing it here doesn't wait for the slow thread, which drops frames when busy.
		 */
		if (debug_get_bool_option_constellation_tracker_fast_prior_search() && Tcv_cam_device_predicted.has_value()) {
			auto &cs = this->fast_processing_thread_data.cs;
			correspondence_search_set_blobs(cs, sample.blobs, sample.blob_count);
			cs->ranking_valid = sample.have_ranking;
			std::memcpy(cs->device_ids_by_distance, sample.device_ids_by_distance.data(),
			            sizeof(cs->device_ids_by_distance));
			xrt_pose Tcv_cam_device = Tcv_cam_device_predicted->pose;
			correspondence_search_flags flags = (correspondence_search_flags)(
			    CS_FLAG_SHALLOW_SEARCH | CS_FLAG_HAVE_POSE_PRIOR | CS_FLAG_STOP_FOR_STRONG_MATCH);
			xrt_vec3 cv_gravity = {0.0, 1.0, 0.0};
			xrt_vec3 up_in_camera = {0, 1, 0}, imu_up_in_device = {0, 1, 0};
			bool have_imu_up = false;
			if (Tcv_world_cam.has_value()) {
				xrt_pose Tcv_world_cam_v = Tcv_world_cam.value();
				if (device->gravity_error_rad < DEG_TO_RAD(25.f)) {
					get_pose_gravity_vector(Tcv_world_cam_v, cv_gravity);
					flags = (correspondence_search_flags)(flags | CS_FLAG_MATCH_GRAVITY);
				}
				xrt_quat q_imu;
				if (tracker->gravity_gate_deg > 0.0f && device->device->get_imu_orientation != NULL &&
				    device->device->get_imu_orientation(device->device, sample.timestamp_ns, &q_imu)) {
					get_pose_gravity_vector(Tcv_world_cam_v, up_in_camera);
					imu_up_in_device = device->imuUpInDevice(q_imu);
					have_imu_up = true;
				}
			}
			pose_metrics score;
			if (correspondence_search_find_one_pose(cs, device->search_model, flags, &Tcv_cam_device,
			                                        &device->prior_pos_error, &device->prior_rot_error,
			                                        &cv_gravity, device->gravity_error_rad,
			                                        have_imu_up ? &imu_up_in_device : NULL,
			                                        have_imu_up ? &up_in_camera : NULL,
			                                        DEG_TO_RAD(tracker->gravity_gate_deg), &score)) {
				this->pushPose(sample, device_state, device, score, Tcv_cam_device, false,
				               CONSTELLATION_SOLVE_TYPE_SHALLOW_CORRESPONDENCE_SEARCH);
				if (device_state.found_pose.has_value()) {
					diag_dev_inc(tracker, device->id, &TrackerDiag::Dev::fast_found);
					CT_DEBUG(tracker, "Fast processing for device %d succeeded with a prior-seeded search",
					         device->id);
					continue;
				}
			}
		}

		device_state.needs_slow_processing = true;

		need_slow_search = true;
	}

	this->debugScribbleSample(sample, true);

	// Only save samples on the fast processing thread, since the slow processing thread is *triggered* by the fast
	// processing thread.
	if (tracker->data_recorder) {
		tracker->data_recorder->recordSample(sample);
	}

	/*
	 * Handle any poses that this sample may now have, it's okay if this is called again in the slow thread, that's
	 * what @ref FoundDevicePose::handled is for.
	 */
	this->handleStepComplete(sample);

	sample.fast_duration_ns = os_monotonic_get_ns() - process_start_ns;
	if (!need_slow_search) {
		this->handleProcessingComplete(sample);
	}

	return need_slow_search;
}

void
Camera::pushPose(CameraSample &camera_sample,
                 DeviceState &device_state,
                 Device *device,
                 pose_metrics &score,
                 const xrt_pose &Tcv_cam_device_initial,
                 bool was_optimized,
                 t_constellation_tracker_trace_log_solve_type solve_type)
{
	// We should never find two poses for the same device in a single frame
	assert(device_state.found_pose.has_value() == false);

	ConstellationTracker *tracker = this->tracker;

	uint32_t blobs_marked_before_update = num_blobs_for_device(camera_sample, device->id);

	// Diagnostics: when the slow search had to find it, how far off was the prior the fast path tried?
	if ((solve_type == CONSTELLATION_SOLVE_TYPE_SHALLOW_CORRESPONDENCE_SEARCH ||
	     solve_type == CONSTELLATION_SOLVE_TYPE_DEEP_CORRESPONDENCE_SEARCH) &&
	    device_state.Tcv_cam_device_predicted.has_value()) {
		const xrt_pose &pr = device_state.Tcv_cam_device_predicted->pose;
		xrt_quat inv, dq;
		math_quat_invert(&pr.orientation, &inv);
		math_quat_rotate(&inv, &Tcv_cam_device_initial.orientation, &dq);
		float rot_deg = RAD_TO_DEG(2.0f * acosf(fminf(1.0f, fabsf(dq.w))));
		xrt_vec3 d = Tcv_cam_device_initial.position;
		math_vec3_subtract(&pr.position, &d);
		// Image shift of the device centre between prior and solve, in focal lengths (x by f for pixels).
		float z = fmaxf(0.05f, Tcv_cam_device_initial.position.z);
		float shift = sqrtf((pr.position.x / fmaxf(0.05f, pr.position.z) - Tcv_cam_device_initial.position.x / z) *
		                        (pr.position.x / fmaxf(0.05f, pr.position.z) - Tcv_cam_device_initial.position.x / z) +
		                    (pr.position.y / fmaxf(0.05f, pr.position.z) - Tcv_cam_device_initial.position.y / z) *
		                        (pr.position.y / fmaxf(0.05f, pr.position.z) - Tcv_cam_device_initial.position.y / z));
		CT_DEBUG(tracker,
		         "Prior vs slow solve for device %d: rot %.1f deg, pos %.3f m, image shift %.3f, prior speed %.2f m/s "
		         "%.0f deg/s",
		         device->id, rot_deg, m_vec3_len(d), shift,
		         m_vec3_len(device_state.Tcv_cam_device_predicted->linear_velocity),
		         RAD_TO_DEG(m_vec3_len(device_state.Tcv_cam_device_predicted->angular_velocity)));
	}

	// Match visible blobs to the pose we found
	pose_metrics_blob_match_info blob_match_info;
	pose_metrics_match_pose_to_blobs_ranked(&Tcv_cam_device_initial, camera_sample.blobs, camera_sample.blob_count,
	                                 &device->params.led_model, device->id, camera_sample.nearRanking(), &this->model, &blob_match_info);

	// Mark all the new blobs using the match info
	camera_sample.markMatchingBlobs(tracker, device->params.led_model, device->id, blob_match_info);

	xrt_pose Tcv_cam_device = Tcv_cam_device_initial;

	// Only do an optimization if we haven't already optimized, or we marked new blobs.
	// This prevents us from optimizing a pose multiple times in a single frame.
	if (!was_optimized || num_blobs_for_device(camera_sample, device->id) > blobs_marked_before_update) {
		// Try to optimize the pose again, unmarking outliers
		xrt_pose Tcv_cam_device_optimized;
		bool success = optimizePose(   //
		    tracker->log_level,        //
		    tracker->deterministic,    //
		    this->model,               //
		    Tcv_cam_device,            //
		    camera_sample.blobs,       //
		    camera_sample.blob_count,  //
		    &device->params.led_model, //
		    device->id,                //
		    Tcv_cam_device_optimized); //

		if (!success) {
			diag_dev_inc(tracker, device->id, &TrackerDiag::Dev::refine_failed);
			CT_DEBUG(tracker,
			         "Camera %d (group %d) RANSAC-PnP refinement for device %d from %u "
			         "blobs failed",
			         0, 0, device->id, camera_sample.blob_count);
		} else {
			Tcv_cam_device = Tcv_cam_device_optimized;

			CT_DEBUG(tracker,
			         "Camera %d (group %d) RANSAC-PnP refinement for device %d from %u "
			         "blobs. Produced pose %f,%f,%f,%f pos %f,%f,%f",
			         0, 0, device->id, camera_sample.blob_count, Tcv_cam_device.orientation.x,
			         Tcv_cam_device.orientation.y, Tcv_cam_device.orientation.z,
			         Tcv_cam_device.orientation.w, Tcv_cam_device.position.x, Tcv_cam_device.position.y,
			         Tcv_cam_device.position.z);

			// Now that we've optimized the pose, we gotta update the match info
			pose_metrics_match_pose_to_blobs_ranked(&Tcv_cam_device, camera_sample.blobs, camera_sample.blob_count,
			                                 &device->params.led_model, device->id, camera_sample.nearRanking(), &this->model,
			                                 &blob_match_info);

			// Unmark any other blobs
			camera_sample.markMatchingBlobs(tracker, device->params.led_model, device->id, blob_match_info);
		}

		// If after RANSAC and optimization no blobs remain, skip, so that we aren't putting an invalid pose
		// into the fusion/latest field.
		if (num_blobs_for_device(camera_sample, device->id) == 0) {
			device_state.solve_type = CONSTELLATION_SOLVE_NO_BLOBS_AFTER_RANSAC;
			return;
		}

		// We need to re-evaluate the pose after optimization, since the reprojection error may have changed.
		pose_metrics_evaluate_pose_ranked(    //
		    &score,                    //
		    &Tcv_cam_device,           //
		    camera_sample.blobs,       //
		    camera_sample.blob_count,  //
		    &device->params.led_model, //
		    device->id, camera_sample.nearRanking(),                //
		    &this->model,              //
		    NULL);                     //
	}

	// Move to OpenXR space
	xrt_pose Txr_cam_device;
	math_pose_convert_from_opencv(&Tcv_cam_device, &Txr_cam_device);

	CT_DEBUG(tracker, "Pose: orient %f %f %f %f pos %f %f %f", Txr_cam_device.orientation.x,
	         Txr_cam_device.orientation.y, Txr_cam_device.orientation.z, Txr_cam_device.orientation.w,
	         Txr_cam_device.position.x, Txr_cam_device.position.y, Txr_cam_device.position.z);

	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(mosaic,
	                      "Camera mosaic was destroyed while processing a sample, this should never happen since "
	                      "the mosaic owns the camera");

	float average_brightness = 0.0f;
	uint32_t used_blobs = 0;
	for (uint32_t i = 0; i < camera_sample.blob_count; i++) {
		struct t_blob &b = camera_sample.blobs[i];
		if (b.matched_device_id == device->id) {
			average_brightness += b.brightness;
			used_blobs++;
		}
	}
	if (used_blobs > 0) {
		average_brightness /= used_blobs;
	}

	// Mark that we found a pose
	device_state.found_pose = {
	    .Tcv_cam_device = Tcv_cam_device,
	    .average_blob_brightness = average_brightness,
	};
	device_state.solve_type = solve_type;

	auto Txr_world_cam = camera_sample.Txr_world_cam;

	std::optional<xrt_pose> Txr_world_device = Txr_world_cam;
	if (Txr_world_cam.has_value()) {
		math_pose_transform(&Txr_world_cam.value(), &Txr_cam_device, &Txr_world_device.value());
	}

	/*
	 * Gravity gate: a correspondence off by one step of a symmetric LED pattern still fits the blobs well, but the
	 * pose is rotated by that step. On PS VR2 (01:05 run) such solves were off from the controller's own IMU up
	 * direction by 50-70 deg, every one of a controller's > 10cm position jumps coincided with one, and they made up
	 * half of one controller's solves, while correct ones agreed to a median 5-6 deg with almost nothing between 20
	 * and 50. The IMU up is independent of the optical solution, so reject such a pose before it reaches the fusion
	 * and the next frame's prior. Replayed on that run with a 35 deg gate: > 10cm consecutive jumps 53.6% -> 0.3%.
	 */
	xrt_quat q_imu;
	if (Txr_world_device.has_value() && device->device->get_imu_orientation != NULL &&
	    device->device->get_imu_orientation(device->device, camera_sample.timestamp_ns, &q_imu)) {
		const xrt_quat &q_opt = Txr_world_device.value().orientation;

		if (tracker->gravity_gate_deg > 0.0f) {
			xrt_vec3 world_up = XRT_VEC3_UNIT_Y, optical_up, imu_up;
			xrt_quat q_inv;
			math_quat_invert(&q_opt, &q_inv);
			math_quat_rotate_vec3(&q_inv, &world_up, &optical_up);
			imu_up = device->imuUpInDevice(q_imu);
			float angle_deg = RAD_TO_DEG(acosf(CLAMP(m_vec3_dot(optical_up, imu_up), -1.0f, 1.0f)));
			if (angle_deg > tracker->gravity_gate_deg) {
				diag_dev_inc(tracker, device->id, &TrackerDiag::Dev::rejected_up);
				CT_DEBUG(tracker,
				         "Rejecting pose for device %d: up direction %.1f deg from its IMU (gate %.1f deg)",
				         device->id, angle_deg, tracker->gravity_gate_deg);
				device_state.found_pose.reset();
				device_state.solve_type = CONSTELLATION_SOLVE_NO_BLOBS_AFTER_RANSAC;
				return;
			}
		}

		/*
		 * Gyro gate: a flip about the vertical axis keeps the up direction, so compare the rotation since the last
		 * accepted pose with the IMU's over the same time (body frame, so the IMU's arbitrary yaw cancels). On the
		 * 01:05 run, consecutive accepted solves agreed with the IMU to a median 0.4-0.6 deg (p95 < 2.5), the flips
		 * the gravity gate misses were off by a ring step (60-72 deg); replayed, a 15 deg gate left 0 flips. Over
		 * more than 0.25s the reference is too old to compare. After 5 rejections in a row the new pose becomes the
		 * reference, so a reference that was itself a missed flip can't lock the device out (never needed in the
		 * replay).
		 */
		if (tracker->gyro_gate_deg > 0.0f) {
			std::lock_guard<std::mutex> guard(device->gyro_gate_lock);
			bool reject = false;
			if (device->gyro_ref_valid && camera_sample.timestamp_ns > device->gyro_ref_ts &&
			    camera_sample.timestamp_ns - device->gyro_ref_ts < 250 * U_TIME_1MS_IN_NS) {
				xrt_quat inv_ref, d_opt, d_imu, d_imu_inv, err;
				math_quat_invert(&device->gyro_ref_optical, &inv_ref);
				math_quat_rotate(&inv_ref, &q_opt, &d_opt);
				math_quat_invert(&device->gyro_ref_imu, &inv_ref);
				math_quat_rotate(&inv_ref, &q_imu, &d_imu);
				math_quat_invert(&d_imu, &d_imu_inv);
				math_quat_rotate(&d_imu_inv, &d_opt, &err);
				float err_deg = RAD_TO_DEG(2.0f * acosf(CLAMP(fabsf(err.w), 0.0f, 1.0f)));
				if (err_deg > tracker->gyro_gate_deg) {
					if (++device->gyro_rejects_in_a_row < 5) {
						reject = true;
						diag_dev_inc(tracker, device->id, &TrackerDiag::Dev::rejected_gyro);
						CT_DEBUG(tracker,
						         "Rejecting pose for device %d: rotated %.1f deg differently from its "
						         "IMU since the last accepted pose (gate %.1f deg)",
						         device->id, err_deg, tracker->gyro_gate_deg);
					} else {
						CT_DEBUG(tracker,
						         "Device %d: %u poses in a row disagree with the gyro reference, taking "
						         "the new one as the reference",
						         device->id, device->gyro_rejects_in_a_row);
					}
				}
			}
			if (reject) {
				device_state.found_pose.reset();
				device_state.solve_type = CONSTELLATION_SOLVE_NO_BLOBS_AFTER_RANSAC;
				return;
			}
			device->gyro_rejects_in_a_row = 0;
			device->gyro_ref_valid = true;
			device->gyro_ref_ts = camera_sample.timestamp_ns;
			device->gyro_ref_optical = q_opt;
			device->gyro_ref_imu = q_imu;
		}
		// Passed the IMU checks: let it pull the learned IMU up toward the optical one.
		device->learnImuUp(q_imu, q_opt);
	}

	// Compute the metrics
	t_constellation_tracker_sample_metrics metrics = {
	    .matched_blob_count = score.matched_blobs,
	    .visible_led_count = score.visible_leds,
	    .reprojection_error = score.matched_blobs > 0 ? sqrtf(score.reprojection_error / score.matched_blobs) : 0.0,
	};

	// Push the sample to the device
	t_constellation_tracker_sample sample = {
	    .timestamp_ns = camera_sample.timestamp_ns,
	    .sequence_id = camera_sample.sequence_id,
	    .has_world_pose = Txr_world_device.has_value(),
	    .world_pose = XRT_POSE_IDENTITY,
	    .mosaic_index = mosaic->index,
	    .camera_index = this->index,
	    .average_brightness = average_brightness,
	    .metrics = metrics,
	    .leds = {},
	};
	if (Txr_world_device.has_value()) {
		sample.world_pose = Txr_world_device.value();
	}

	for (int i = 0; i < blob_match_info.num_visible_leds; i++) {
		const auto &visible_led = blob_match_info.visible_leds[i];

		// Skip visible LEDs that have no matched blob
		if (visible_led.matched_blob == nullptr) {
			continue;
		}

		// Assert the IDs are valid indices into the array.
		assert(visible_led.led->id >= 0);
		assert((size_t)visible_led.led->id < ARRAY_SIZE(sample.leds));

		// Mark the LED as observed
		sample.leds[visible_led.led->id] = {
		    .observed = true,
		    .brightness = visible_led.matched_blob->brightness,
		    .greysum = visible_led.matched_blob->greysum,
		    .facing_dot = static_cast<float>(visible_led.facing_dot),
		};
	}

	// Push the sample now that the data is filled out.
	t_constellation_tracker_device_push_sample(device->device, &sample);

	if (Txr_world_device.has_value()) {
		std::unique_lock<os::Mutex> lock(device->data_lock);

		// If we already found a pose in the future, then don't mark blobs, since the device has definitely
		// moved.
		if (!device->locked_data.last_known_pose.has_value() ||
		    device->locked_data.last_known_pose->timestamp_ns <= camera_sample.timestamp_ns) {
			// Call back to the blobwatch to update the blobs for this device. Done after pose optimization
			// since the RANSAC process will unlabel any outliers.
			auto tbo = camera_sample.toBlobObservation();
			t_blobwatch_mark_blob_device(camera_sample.source, &tbo, device->id);
		}

		device->locked_data.last_known_pose =
		    DeviceLastPose(Txr_world_device.value(), Tcv_cam_device, camera_sample.timestamp_ns);
	}

	diag_dev_inc(tracker, device->id, &TrackerDiag::Dev::accepted);
	CT_DEBUG(tracker, "Found pose for device %d", device->id);
}

/*
 *
 * CameraMosaic implementations
 *
 */

CameraMosaic::CameraMosaic(ConstellationTracker *tracker,
                           const t_constellation_tracker_camera_mosaic &mosaic_params,
                           uint32_t index)
    : index(index)
{
	this->tracking_origin = mosaic_params.tracking_origin;

	// NOTE: the stability of this vector is important since we're passing to C callbacks and APIs!
	this->cameras.reserve(mosaic_params.num_cameras);
}

std::optional<xrt_pose>
CameraMosaic::getTrackingOriginPose(timepoint_ns when_ns)
{
	if (this->tracking_origin) {
		xrt_space_relation relation;
		t_constellation_tracker_tracking_source_get_tracked_pose(this->tracking_origin, when_ns, &relation);

		// If the tracking source has a valid position, grab it
		if ((relation.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0 &&
		    (relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
			return relation.pose;
		} else {
			return std::nullopt;
		}
	}

	return std::optional<xrt_pose>(XRT_POSE_IDENTITY);
}

/*
 *
 * DeviceLastPose implementations
 *
 */

DeviceLastPose::DeviceLastPose(xrt_pose Txr_world_device, xrt_pose Tcv_cam_device, timepoint_ns timestamp_ns)
    : Txr_world_device(Txr_world_device), Tcv_cam_device(Tcv_cam_device), timestamp_ns(timestamp_ns)
{}

/*
 *
 * Device implementations
 *
 */

xrt_vec3
Device::imuUpInDevice(const xrt_quat &q_imu)
{
	xrt_vec3 up = XRT_VEC3_UNIT_Y;
	if (debug_get_float_option_constellation_tracker_imu_up_learn() > 0.0f) {
		std::lock_guard<std::mutex> g(this->imu_up_lock);
		up = this->imu_world_up;
	}
	xrt_quat inv;
	math_quat_invert(&q_imu, &inv);
	xrt_vec3 out;
	math_quat_rotate_vec3(&inv, &up, &out);
	return out;
}

void
Device::learnImuUp(const xrt_quat &q_imu, const xrt_quat &q_optical)
{
	const float gain = debug_get_float_option_constellation_tracker_imu_up_learn();
	if (gain <= 0.0f) {
		return;
	}
	// The optical up (world +Y) in the device frame, then into the IMU's world frame.
	xrt_quat inv;
	math_quat_invert(&q_optical, &inv);
	xrt_vec3 world_up = XRT_VEC3_UNIT_Y, up_dev, up_imu_world;
	math_quat_rotate_vec3(&inv, &world_up, &up_dev);
	math_quat_rotate_vec3(&q_imu, &up_dev, &up_imu_world);
	std::lock_guard<std::mutex> g(this->imu_up_lock);
	xrt_vec3 u = this->imu_world_up;
	u.x += gain * (up_imu_world.x - u.x);
	u.y += gain * (up_imu_world.y - u.y);
	u.z += gain * (up_imu_world.z - u.z);
	math_vec3_normalize(&u);
	this->imu_world_up = u;
}

Device::Device(ConstellationTracker *tracker,
               t_constellation_tracker_device_params *params,
               t_constellation_tracker_device *device,
               t_constellation_device_id_t id)
    : params(*params), original_led_model(params->led_model), device(device), id(id), tracker(tracker), data_lock(),
      locked_data({.last_known_pose = std::nullopt})
{
	this->imu_sink = {
	    .push_imu =
	        [](xrt_imu_sink *ptr, xrt_imu_sample *sample) {
		        auto self = Device::fromXrtImuSink(ptr);
		        self->pushImuSample(*sample);
	        },
	};
	params->imu_sink = &this->imu_sink;

	// Copy the LED model leds into safe memory, since we want to mutate it into OpenCV space
	this->params.led_model.leds = new t_constellation_tracker_led[this->params.led_model.led_count];
	memcpy(this->params.led_model.leds, params->led_model.leds,
	       sizeof(t_constellation_tracker_led) * this->params.led_model.led_count);

	// flip all LEDs from OpenXR -> OpenCV coordinate space, since the tracker works in OpenCV space
	for (uint32_t i = 0; i < this->params.led_model.led_count; i++) {
		t_constellation_tracker_led &dst = this->params.led_model.leds[i];
		const t_constellation_tracker_led &src = params->led_model.leds[i];

		dst = src;
		math_vec3_convert_from_opencv(&src.position, &dst.position);
		math_vec3_convert_from_opencv(&src.normal, &dst.normal);
	}

	this->search_model = t_constellation_search_model_new(this->id, &this->params.led_model);
}

Device::~Device()
{
	// Clear the sensor fusion so it gets destructed.
	this->sensor_fusion.reset();

	if (this->search_model) {
		t_constellation_search_model_free(this->search_model);
		this->search_model = nullptr;
	}

	if (this->params.led_model.leds) {
		delete[] this->params.led_model.leds;
		this->params.led_model.leds = nullptr;
	}
}

void
Device::pushImuSample(xrt_imu_sample &sample)
{
	this->tracker->rig_calibration->pushImuSample(this->id, sample);

	this->sensor_fusion->pushImuSample(sample);

	// The recorder's IMU sink for this device, if a recorder exists.
	xrt_tracking_sinks *euroc_sinks = this->tracker->euroc_sinks.load();
	int32_t euroc_idx = this->euroc_imu_index.load();
	xrt_imu_sink *euroc_imu_sink =
	    (euroc_sinks != nullptr && euroc_idx >= 0) ? euroc_sinks->imus[euroc_idx] : nullptr;

	if (euroc_imu_sink != nullptr) {
		xrt_sink_push_imu(euroc_imu_sink, &sample);
	}
}

/*
 *
 * ConstellationTracker implementations
 *
 */

ConstellationTracker::ConstellationTracker(t_constellation_tracker_params *params)
    : log_level(debug_get_log_option_constellation_tracker_log()), params(*params), rig_calibration_lock()
{
	this->ceres_context.reset(ceres::Context::Create());
	this->gravity_gate_deg = debug_get_float_option_constellation_tracker_gravity_gate_deg();
	this->gyro_gate_deg = debug_get_float_option_constellation_tracker_gyro_gate_deg();

	// If the deterministic flag is set, we enable single-threaded processing, and mark it as deterministic.
	if ((params->flags & T_CONSTELLATION_TRACKER_FLAGS_DETERMINISTIC) != 0) {
		this->deterministic = true;
	}

	this->mosaics.reserve(params->num_mosaics);

	// Fill in our internal data structures based on the provided params
	for (uint32_t mosaic_idx = 0; mosaic_idx < params->num_mosaics; mosaic_idx++) {
		const t_constellation_tracker_camera_mosaic &mosaic_params = params->mosaics[mosaic_idx];

		std::shared_ptr<CameraMosaic> mosaic = std::make_shared<CameraMosaic>(this, mosaic_params, mosaic_idx);

		// Assert pointer stability!
		assert(mosaic->cameras.capacity() >= mosaic_params.num_cameras);

		for (uint32_t cam_idx = 0; cam_idx < mosaic_params.num_cameras; cam_idx++) {
			const t_constellation_tracker_camera &camera_params = mosaic_params.cameras[cam_idx];

			// This can't be in the constructor since the `shared_ptr` of the `mosaic` isn't formed
			// yet, but the camera needs to own a weak_ptr to it's mosaic.
			mosaic->cameras.push_back(
			    std::make_unique<Camera>(this, mosaic, camera_params, &this->log_level, cam_idx));
		}

		this->mosaics.push_back(mosaic);
	}

	// Fill in the blob sinks for each camera
	for (size_t i = 0; i < this->mosaics.size(); i++) {
		std::shared_ptr<CameraMosaic> &mosaic = this->mosaics[i];

		for (size_t j = 0; j < mosaic->cameras.size(); j++) {
			Camera *camera = mosaic->cameras[j].get();

			params->mosaics[i].cameras[j].blob_sink = &camera->base;
		}
	}

	std::string data_recorder_output = debug_get_option_constellation_tracker_data_recorder_output();
	if (!data_recorder_output.empty()) {
		this->data_recorder = std::make_unique<DataRecorder>(this, data_recorder_output);
		CT_INFO(this, "Constellation tracker data recorder enabled, outputting to %s",
		        data_recorder_output.c_str());
	}

	if (debug_get_bool_option_constellation_tracker_enable_rerun()) {
#ifdef XRT_FEATURE_RERUN
		this->rerun_stream = std::make_unique<RerunContext>();
		CT_INFO(this, "Constellation tracker Rerun stream enabled, outputting to constellation_tracker.rerun");

		if (debug_get_bool_option_constellation_tracker_rerun_spawn()) {
			this->rerun_stream->spawnViewer();
		} else {
			this->rerun_stream->stream->connect_grpc().exit_on_failure();
		}
#else
		CT_ERROR(this, "Rerun stream requested but XRT_FEATURE_RERUN is not enabled");
#endif
	}

	this->rig_calibration = std::make_unique<rig_calibration::RigCalibration>();

	for (auto &mosaic : this->mosaics) {
		for (auto &camera : mosaic->cameras) {
			// The solve works in OpenCV space, and its world is the tracking origin: that is the frame the
			// solved poses are written back into below in checkSensorCalibration.
			xrt_pose Tcv_origin_cam;
			math_pose_convert_from_opencv(&camera->locked_data.Txr_origin_cam, &Tcv_origin_cam);

			// Add the camera to the sensor calibration job
			this->rig_calibration->addCamera({
			    .mosaic_idx = mosaic->index,
			    .camera_idx = camera->index,
			    .params = camera->model,
			    .Tcv_world_cam = Tcv_origin_cam,
			    .has_concrete_pose = camera->locked_data.has_concrete_pose,
			});
		}
	}

	CT_DEBUG(this, "Created constellation tracker with %zu mosaics", this->mosaics.size());
}

ConstellationTracker::~ConstellationTracker()
{
	if (this->rig_calibration_thread.initialized) {
		os_thread_helper_destroy(&this->rig_calibration_thread);
	}

	CT_DEBUG(this, "Destroying constellation tracker");
}

bool
ConstellationTracker::needSensorCalibration()
{
	bool need_sensor_calibration = false;

	// Check if any cameras need calibration.
	for (const auto &mosaic : this->mosaics) {
		for (const auto &camera : mosaic->cameras) {
			if (!camera->locked_data.has_concrete_pose) {
				need_sensor_calibration = true;
			}
		}
	}

	return need_sensor_calibration;
}

bool
ConstellationTracker::checkSensorCalibration()
{
	std::unique_lock<os::Mutex> lock(this->rig_calibration_lock);

	if (this->running_calibration) {
		return true;
	}

	uint32_t num_cameras = 0;
	for (const auto &mosaic : this->mosaics) {
		num_cameras += mosaic->cameras.size();
	}

	// No calibration needed
	if (!this->needSensorCalibration()) {
		return true;
	}

	auto &calib = this->rig_calibration;

	// Start collecting data if we haven't
	if (!calib->isCollectingData()) {
		calib->beginCollectingData();

		// Enable the first device, disable all others, for the purposes of clean data.
		bool is_first = true;
		for (auto &device : this->devices) {
			device->enabled.store(is_first);
			is_first = false;
		}
	}

	size_t num_samples;
	if (!calib->numSamplesForBestDevice(num_samples)) {
		// Nothing yet
		return true;
	}

	// @todo Figure out a good number for this, it probably is going to depend on time, "seen by multiple cameras at
	//       once", and other factors rather than pure sample count?
	if (num_samples < (num_cameras * 1500)) {
		return true;
	}

	// Stop collecting data
	calib->endCollectingData();

	if (!this->rig_calibration_thread.initialized) {
		os_thread_helper_init(&this->rig_calibration_thread);
	}

	// If one was already running, wait for it to stop.
	os_thread_helper_stop_and_wait(&this->rig_calibration_thread);

	// Mark that we're now going to be running one
	this->running_calibration = true;

	// Try to start the new thread
	os_thread_helper_start(&this->rig_calibration_thread, run_room_calibration_func, this);

	return true;
}

void
ConstellationTracker::setupVariableTracking()
{
	u_var_add_root(this, "Constellation Tracker", true);
	u_var_add_log_level(this, &this->log_level, "Log Level");

	this->euroc_toggle_btn.cb = [](void *ptr) {
		auto *self = static_cast<ConstellationTracker *>(ptr);

		// This runs from the GUI with the u_var lock held, so we can't add/remove u_var roots here. Hand the
		// work to a thread, and drop the click if the previous one hasn't finished.
		if (self->euroc_toggle_pending.exchange(true)) {
			return;
		}
		if (self->euroc_toggle_thread.joinable()) {
			self->euroc_toggle_thread.join();
		}

		self->euroc_session_wanted = !self->euroc_session_wanted;
		bool wanted = self->euroc_session_wanted;
		snprintf(self->euroc_toggle_btn.label, sizeof(self->euroc_toggle_btn.label), "%s",
		         wanted ? "Destroy EuRoC recorder" : "Create EuRoC recorder");

		self->euroc_toggle_thread = std::thread([self, wanted]() {
			self->setEurocRecorderSession(wanted);
			self->euroc_toggle_pending = false;
		});
	};
	this->euroc_toggle_btn.ptr = this;
	snprintf(this->euroc_toggle_btn.label, sizeof(this->euroc_toggle_btn.label), "Create EuRoC recorder");
	u_var_add_button(this, &this->euroc_toggle_btn, "EuRoC Recorder");

	uint32_t mosaic_idx = 0;
	for (auto &mosaic : this->mosaics) {
		uint32_t camera_idx = 0;
		for (auto &camera : mosaic->cameras) {
			auto str =
			    "Camera " + std::to_string(camera_idx) + " (Mosaic " + std::to_string(mosaic_idx) + ")";
			u_var_add_gui_header(this, nullptr, str.c_str());

			u_var_add_sink_debug(this, &camera->fast_processing_thread_data.debug_sink,
			                     "Blob Debug Sink (Fast)");
			u_var_add_sink_debug(this, &camera->slow_processing_thread_data.debug_sink,
			                     "Blob Debug Sink (Slow)");

			u_var_add_pose(this, &camera->locked_data.Txr_origin_cam, "Camera Origin Pose");
			u_var_add_bool(this, &camera->locked_data.has_concrete_pose, "Has Concrete Pose");

			camera->scribble_settings.setupDebugTracking(this);

			camera_idx++;
		}
		mosaic_idx++;
	}
}

EurocRecorderSession::~EurocRecorderSession()
{
	// Remove the root first so the GUI can't touch the recorder while its nodes are destroyed.
	u_var_remove_root(this);
	xrt_frame_context_destroy_nodes(&this->xfctx);
}

void
ConstellationTracker::shutdownEurocRecorderSession()
{
	if (this->euroc_toggle_thread.joinable()) {
		this->euroc_toggle_thread.join();
	}
	this->setEurocRecorderSession(false);
}

void
ConstellationTracker::setEurocRecorderSession(bool enabled)
{
	if (!enabled) {
		if (!this->euroc_session) {
			return;
		}

		this->euroc_sinks.store(nullptr);

		{
			std::shared_lock lock(this->device_lock);
			for (auto &device : this->devices) {
				device->euroc_imu_index = -1;
			}
			for (auto &mosaic : this->mosaics) {
				for (auto &camera : mosaic->cameras) {
					camera->euroc_cam_index = -1;
				}
			}
		}

		this->euroc_session.reset();
		return;
	}

	if (this->euroc_session) {
		return;
	}

	auto session = std::make_unique<EurocRecorderSession>();

	euroc_recorder_settings settings;
	U_ZERO(&settings);
	settings.record_from_start = false;
	settings.constellation_recording = true;
	settings.camera_model_valid = true;

	{
		std::shared_lock lock(this->device_lock);

		if (this->devices.size() > XRT_TRACKING_MAX_IMUS) {
			CT_ERROR(this, "Too many devices (%zu) for a EuRoC recorder, max is %d", this->devices.size(),
			         XRT_TRACKING_MAX_IMUS);
			return;
		}

		// Flatten the mosaics into a single list of cameras
		timepoint_ns now_ns = os_monotonic_get_ns();
		uint32_t cam_count = 0;
		for (auto &mosaic : this->mosaics) {
			for (auto &camera : mosaic->cameras) {
				if (cam_count >= XRT_TRACKING_MAX_CAMS) {
					CT_ERROR(this, "Too many cameras for a EuRoC recorder, max is %d",
					         XRT_TRACKING_MAX_CAMS);
					return;
				}

				// Map (mosaic, camera) -> recorder camera index
				camera->euroc_cam_index = (int32_t)cam_count;
				settings.camera_model.intrinsics[cam_count] = camera->calibration;
				settings.camera_model.poses[cam_count] =
				    camera->getWorldPose(now_ns).value_or(xrt_pose XRT_POSE_IDENTITY);
				cam_count++;
			}
		}
		settings.cam_count = cam_count;

		// Hand the recorder the original (OpenXR frame) LED models, the tracker's own are in OpenCV space.
		uint32_t imu_count = 0;
		for (auto &device : this->devices) {
			// Map device ID -> recorder IMU index
			device->euroc_imu_index = (int32_t)imu_count;
			settings.constellation.led_models[imu_count] = &device->original_led_model;
			settings.constellation.imu_calibrations[imu_count] = &device->params.imu_calibration;
			imu_count++;
		}
		settings.imu_count = imu_count;
	}

	session->sinks = euroc_recorder_create(&session->xfctx, &settings);

	snprintf(session->camera_info, sizeof(session->camera_info), "%u", settings.cam_count);
	snprintf(session->device_info, sizeof(session->device_info), "%u", settings.imu_count);

	EurocRecorderSession *raw = session.get();
	u_var_add_root(raw, "Constellation EuRoC Recorder", true);
	u_var_add_ro_text(raw, raw->camera_info, "Cameras");
	u_var_add_ro_text(raw, raw->device_info, "Devices");
	euroc_recorder_add_ui(raw->sinks, raw, "");

	this->euroc_session = std::move(session);

	// Everything is set up, let the push paths see the sinks.
	this->euroc_sinks.store(this->euroc_session->sinks);
}

t_constellation_device_id_t
ConstellationTracker::addDevice(t_constellation_tracker_device_params *params, t_constellation_tracker_device *device)
{
	if (params->led_model.led_count > XRT_CONSTELLATION_MAX_LEDS_PER_DEVICE) {
		throw std::runtime_error("Device has too many LEDs, maximum is " +
		                         std::to_string(XRT_CONSTELLATION_MAX_LEDS_PER_DEVICE));
	}

	std::unique_lock lock(this->device_lock);

	if (this->devices.size() >= XRT_CONSTELLATION_MAX_DEVICES) {
		throw std::runtime_error("Maximum number of devices already added to constellation tracker");
	}

	t_constellation_device_id_t id = this->next_device_id++;

	this->devices.push_back(std::make_unique<Device>(this, params, device, id));

	CT_DEBUG(this, "Added device with ID %d to constellation tracker", id);

	const auto &new_device = this->devices.back();

	// Enable the device if there's no calibration needed or this is the only device
	if (!this->needSensorCalibration() || this->devices.size() == 1) {
		new_device->enabled.store(true);
	}

	if (this->data_recorder) {
		this->data_recorder->recordDeviceInfo(*new_device);
	}

	new_device->sensor_fusion = std::make_unique<sensor_fusion::SensorFusion>( //
	    this->ceres_context.get(),                                             //
	    this->deterministic,                                                   //
	    id,                                                                    //
	    new_device->search_model->led_model,                                   //
	    new_device->params.imu_calibration);                                   //

	this->rig_calibration->addDevice(id, new_device->params.imu_calibration);

	for (auto &mosaic : this->mosaics) {
		for (auto &camera : mosaic->cameras) {
			// Add the camera to the sensor fusion job
			new_device->sensor_fusion->addCamera({
			    .mosaic_idx = mosaic->index,
			    .camera_idx = camera->index,
			    .params = camera->model,
			});
		}
	}

	return id;
}

void
ConstellationTracker::removeDevice(t_constellation_device_id_t device_id)
{
	std::unique_lock lock(this->device_lock);

	size_t index = 0;
	for (auto &device : this->devices) {
		if (device->id == device_id) {
			break;
		}
		index++;
	}

	if (index == this->devices.size()) {
		throw std::invalid_argument("The device ID is not present in the device list.");
	}

	// Remove the device
	this->devices.erase(this->devices.begin() + index);

	this->rig_calibration->removeDevice(device_id);
}

void
ConstellationTracker::getTrackedPose(t_constellation_device_id_t device_id,
                                     timepoint_ns when_ns,
                                     xrt_space_relation &out_relation)
{
	out_relation = XRT_SPACE_RELATION_ZERO;

	std::shared_lock lock(this->device_lock);
	for (const auto &device : this->devices) {
		if (device->id != device_id) {
			continue;
		}

		device->sensor_fusion->getTrackedPose(when_ns, out_relation);

		return;
	}

	throw std::runtime_error("Device missing, failed to get tracked pose.");
}

}; // namespace xrt::tracking::constellation

using namespace xrt::tracking::constellation;

void *
constellation_tracker_camera_slow_thread(void *ptr)
{
	Camera *camera = (Camera *)ptr;

	os_thread_helper_lock(&camera->slow_processing_thread);
	while (os_thread_helper_is_running_locked(&camera->slow_processing_thread)) {
		os_thread_helper_wait_locked(&camera->slow_processing_thread);

		std::optional<CameraSample> maybe_sample = camera->slow_processing_thread_data.sample;
		camera->slow_processing_thread_data.sample.reset();

		os_thread_helper_unlock(&camera->slow_processing_thread);

		if (auto sample = maybe_sample) {
			camera->processSampleSlow(*sample);
		}

		os_thread_helper_lock(&camera->slow_processing_thread);
	}
	os_thread_helper_unlock(&camera->slow_processing_thread);

	return NULL;
}

void *
constellation_tracker_camera_fast_thread(void *ptr)
{
	Camera *camera = (Camera *)ptr;

	os_thread_helper_lock(&camera->fast_processing_thread);
	while (os_thread_helper_is_running_locked(&camera->fast_processing_thread)) {
		os_thread_helper_wait_locked(&camera->fast_processing_thread);

		std::optional<CameraSample> maybe_sample = camera->fast_processing_thread_data.sample;
		camera->fast_processing_thread_data.sample.reset();

		os_thread_helper_unlock(&camera->fast_processing_thread);

		if (auto sample = maybe_sample) {
			if (camera->processSampleFast(*sample)) {
				CT_TRACE(camera->tracker,
				         "Fast processing for camera %p failed, deferring to slow thread",
				         (void *)camera);
				camera->deferSampleToSlowThread(*sample);
			}
		}

		os_thread_helper_lock(&camera->fast_processing_thread);
	}
	os_thread_helper_unlock(&camera->fast_processing_thread);

	return NULL;
}

void
constellation_tracker_camera_push_blobs(t_blob_sink *tbs, t_blob_observation *tbo)
{
	Camera *camera = Camera::Get(tbs);
	ConstellationTracker *tracker = camera->tracker;

	CT_TRACE(tracker, "Received blob observation at %" PRIi64 " with %u blobs", tbo->timestamp_ns, tbo->num_blobs);

#ifdef XRT_FEATURE_RERUN
	if (tracker->rerun_stream && tbo->frame != nullptr) {
		auto mosaic = camera->mosaic.lock();
		if (mosaic) {
			tracker->rerun_stream->logImageFrame(*tracker, mosaic->index, camera->index, *tbo->frame);
		}
	}
#endif

	// The recorder's camera sink for this camera, if a recorder exists.
	xrt_tracking_sinks *euroc_sinks = tracker->euroc_sinks.load();
	int32_t euroc_idx = camera->euroc_cam_index.load();
	xrt_frame_sink *euroc_cam_sink =
	    (euroc_sinks != nullptr && euroc_idx >= 0) ? euroc_sinks->cams[euroc_idx] : nullptr;
	if (euroc_cam_sink) {
		xrt_sink_push_frame(euroc_cam_sink, tbo->frame);
	}

	// We own the frame reference, and don't need it past this point.
	xrt_frame_reference(&tbo->frame, nullptr);

	std::vector<t_blob> kept_blobs;
	t_blob_observation filtered_tbo;
	const int64_t max_blob_px = debug_get_num_option_constellation_tracker_max_blob_px();
	if (max_blob_px > 0 && tbo->num_blobs > 0) {
		kept_blobs.reserve(tbo->num_blobs);
		for (uint32_t i = 0; i < tbo->num_blobs; i++) {
			const t_blob &b = tbo->blobs[i];
			if (std::max(b.size.x, b.size.y) <= (float)max_blob_px) {
				kept_blobs.push_back(b);
			}
		}
		if (kept_blobs.size() != tbo->num_blobs) {
			filtered_tbo = *tbo;
			filtered_tbo.blobs = kept_blobs.data();
			filtered_tbo.num_blobs = (uint32_t)kept_blobs.size();
			tbo = &filtered_tbo;
		}
	}

	diag_maybe_report(tracker);
	diag_count_observation(tracker, tbo->num_blobs);
	if (tbo->num_blobs == 0) {
		CT_TRACE(tracker, "No blobs in observation, skipping processing");
		// We own the trace log reference, release it.
		t_constellation_tracker_trace_log_ref(&tbo->trace_log, nullptr);
		return;
	}

	if (tracker->deterministic) {
		auto sample = CameraSample(*tbo, camera);

		// If we're in single-threaded mode, just process the sample immediately on the fast thread
		if (camera->processSampleFast(sample)) {
			CT_TRACE(tracker,
			         "Fast processing for camera %p failed in single-threaded mode, doing slow processing",
			         (void *)camera);

			camera->processSampleSlow(sample);
		}
	} else {
		// Send to the fast thread
		os_thread_helper_lock(&camera->fast_processing_thread);
		{
			if (camera->fast_processing_thread_data.sample.has_value()) {
				// Warn that we're dropping a frame
				tracker->diag.fast_drops.fetch_add(1, std::memory_order_relaxed);
				CT_WARN(tracker,
				        "Dropping fast sample %" PRIu64 " at ts %" PRIi64
				        ". Tracker is likely running slow.",
				        camera->fast_processing_thread_data.sample->id,
				        camera->fast_processing_thread_data.sample->timestamp_ns);
			}

			camera->fast_processing_thread_data.sample = CameraSample(*tbo, camera);
			os_thread_helper_signal_locked(&camera->fast_processing_thread);
		}
		os_thread_helper_unlock(&camera->fast_processing_thread);
	}

	// We own the trace log reference, the sample(s) have taken their own, so release ours.
	t_constellation_tracker_trace_log_ref(&tbo->trace_log, nullptr);
}

void
constellation_tracker_camera_destroy(t_blob_sink *tbs)
{
	// do nothing, the constellation tracker will clean up the blob sinks when it is destroyed.
}

void
constellation_tracker_node_break_apart(xrt_frame_node *node)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(node);

	tracker->running = false;

	// Stop all the threads
	for (auto &mosaic : tracker->mosaics) {
		for (auto &camera : mosaic->cameras) {
			if (camera->slow_processing_thread.initialized) {
				os_thread_helper_stop_and_wait(&camera->slow_processing_thread);
			}

			if (camera->fast_processing_thread.initialized) {
				os_thread_helper_stop_and_wait(&camera->fast_processing_thread);
			}
		}
	}
}

void
constellation_tracker_node_destroy(xrt_frame_node *node)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(node);

	tracker->shutdownEurocRecorderSession();

	u_var_remove_root(tracker);

	delete tracker;
}

int
t_constellation_tracker_create(xrt_frame_context *xfctx,
                               t_constellation_tracker_params *params,
                               t_constellation_tracker **out_tracker)
{
	try {
		ConstellationTracker *tracker = new ConstellationTracker(params);

		// Add us to the frame context
		xrt_frame_context_add(xfctx, &tracker->node);

		*out_tracker = (t_constellation_tracker *)tracker;

		tracker->setupVariableTracking();
	} catch (const std::exception &e) {
		U_LOG_E("Failed to create constellation tracker: %s", e.what());
		return -1;
	}

	return 0;
}

int
t_constellation_tracker_add_device(t_constellation_tracker *raw_tracker,
                                   t_constellation_tracker_device_params *params,
                                   t_constellation_tracker_device *device,
                                   t_constellation_device_id_t *out_device_id)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(raw_tracker);

	try {
		t_constellation_device_id_t device_id = tracker->addDevice(params, device);
		*out_device_id = device_id;
	} catch (const std::exception &e) {
		CT_ERROR(tracker, "Failed to add device to constellation tracker: %s", e.what());
		return -1;
	}

	return 0;
}

int
t_constellation_tracker_remove_device(t_constellation_tracker *raw_tracker, t_constellation_device_id_t device)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(raw_tracker);

	try {
		tracker->removeDevice(device);
	} catch (const std::exception &e) {
		CT_ERROR(tracker, "Failed to remove device from constellation tracker: %s", e.what());
		return -1;
	}

	return 0;
}

xrt_tracking_origin *
t_constellation_tracker_get_tracking_origin(t_constellation_tracker *raw_tracker)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(raw_tracker);

	return &tracker->tracking_origin;
}

int
t_constellation_tracker_get_tracked_pose(struct t_constellation_tracker *raw_tracker,
                                         t_constellation_device_id_t device,
                                         timepoint_ns when_ns,
                                         struct xrt_space_relation *out_relation)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(raw_tracker);

	try {
		tracker->getTrackedPose(device, when_ns, *out_relation);
	} catch (const std::exception &e) {
		CT_ERROR(tracker, "Failed to get the tracked pose for a device from constellation tracker: %s",
		         e.what());
		return -1;
	}

	return 0;
}
