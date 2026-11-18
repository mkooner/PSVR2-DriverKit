// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Header defining the constellation tracker trace log system.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup xrt_iface
 */

#pragma once

#include "xrt/xrt_defines.h"

#include "util/u_time.h"

#include "constellation/t_constellation_tracker.h"


#ifdef __cplusplus
extern "C" {
#endif

struct t_constellation_tracker_trace_log;

/*!
 * @interface t_constellation_trace_log_sink
 *
 * A generic interface to receive completed @ref t_constellation_tracker_trace_log objects, once every stage of the
 * pipeline holding a reference to the trace log has released it.
 *
 * The sink must outlive every trace log created with it.
 */
struct t_constellation_trace_log_sink
{
	/*!
	 * Push a completed trace log into the sink. Called from whichever thread released the last reference.
	 *
	 * @param[in] tls       The sink to push the trace log into.
	 * @param[in] trace_log The completed trace log. It is freed once this returns, so the sink must copy out
	 *                      anything it wants to keep.
	 */
	void (*push_trace_log)(struct t_constellation_trace_log_sink *tls,
	                       const struct t_constellation_tracker_trace_log *trace_log);
};

/*!
 * Helper function for @ref t_constellation_trace_log_sink::push_trace_log.
 *
 * @copydoc t_constellation_trace_log_sink::push_trace_log
 *
 * @public @memberof t_constellation_trace_log_sink
 */
XRT_NONNULL_ALL static inline void
t_constellation_trace_log_sink_push_trace_log(struct t_constellation_trace_log_sink *tls,
                                              const struct t_constellation_tracker_trace_log *trace_log)
{
	tls->push_trace_log(tls, trace_log);
}

enum t_constellation_tracker_trace_log_entry_type
{
	//! Blobs are being found in the image
	CONSTELLATION_TRACE_LOG_ENTRY_BLOBWATCH,
	//! Blobs are being visualized into a frame for debug display
	CONSTELLATION_TRACE_LOG_BLOB_VISUALIZER,
	//! Blobs are being assigned to LEDs
	CONSTELLATION_TRACE_LOG_ENTRY_MATCHING,
	//! A device's sensor fusion is folding the sample in. Pushed once per device the sample was given to.
	CONSTELLATION_TRACE_LOG_ENTRY_SENSOR_FUSION,
	//! The number of entry types.
	CONSTELLATION_TRACE_LOG_MAX_ENTRIES,
};

/*!
 * How many entries a trace log can hold, one of every type, except sensor fusion which can have one per device.
 */
#define CONSTELLATION_TRACE_LOG_ENTRY_CAPACITY (CONSTELLATION_TRACE_LOG_MAX_ENTRIES - 1 + XRT_CONSTELLATION_MAX_DEVICES)

struct t_constellation_tracker_trace_log_blobwatch_entry
{
	//! The difference from frame timestamp -> blobwatch start.
	time_duration_ns pipeline_latency_ns;
	//! The amount of time spend processing the frame.
	time_duration_ns process_time_ns;
};

struct t_constellation_tracker_trace_log_blob_visualizer_entry
{
	//! The difference from frame timestamp -> blobwatch start.
	time_duration_ns pipeline_latency_ns;
	//! The amount of time spend processing the frame.
	time_duration_ns process_time_ns;
};

enum t_constellation_tracker_trace_log_solve_type
{
	CONSTELLATION_SOLVE_TYPE_INVALID,

	//! The predicted prior of the pose matched the watched blobs.
	CONSTELLATION_SOLVE_TYPE_PREDICTED_PRIOR,
	//! The blob labels from the blobwatch were accepted.
	CONSTELLATION_SOLVE_TYPE_BLOB_LABELS,
	//! The last known pose of the device was tried and succeeded.
	CONSTELLATION_SOLVE_TYPE_LAST_KNOWN_POSE,
	//! A shallow search for valid poses on the watched blobs produced an accepted pose.
	CONSTELLATION_SOLVE_TYPE_SHALLOW_CORRESPONDENCE_SEARCH,
	//! A deep search for valid poses on the watched blobs produced an accepted pose.
	CONSTELLATION_SOLVE_TYPE_DEEP_CORRESPONDENCE_SEARCH,

	//! No solve. After doing RANSAC/optimization no blobs matched.
	CONSTELLATION_SOLVE_NO_BLOBS_AFTER_RANSAC,
	//! No solve. It was seen too recently for a slow search to be attempted.
	CONSTELLATION_SOLVE_TYPE_NO_SOLVE_FAST_SOLVED_RECENTLY,
	//! No solve. A deep search is delayed until shallow fails for too long.
	CONSTELLATION_SOLVE_TYPE_NO_SOLVE_DELAYED_DEEP,
	//! No solve. All slow recovery strategies failed.
	CONSTELLATION_SOLVE_TYPE_NO_SOLVE,
};

struct t_constellation_tracker_trace_log_matched_device
{
	t_constellation_device_id_t device_id;
	enum t_constellation_tracker_trace_log_solve_type device_solve_type;
};

struct t_constellation_tracker_trace_log_matching_entry
{
	//! The difference from frame timestamp -> this camera's blob matching start.
	time_duration_ns pipeline_latency_ns;

	//! Whether the fast path was taken at all
	bool fast_taken;
	//! The amount of time spent during fast recovery
	time_duration_ns fast_duration_ns;

	//! Whether the slow path was taken at all
	bool slow_taken;
	//! The amount of time taken during slow recovery
	time_duration_ns slow_duration_ns;

	struct t_constellation_tracker_trace_log_matched_device devices[XRT_CONSTELLATION_MAX_DEVICES];
};

enum t_constellation_tracker_trace_log_fusion_sample_result
{
	CONSTELLATION_FUSION_SAMPLE_INVALID,

	//! The sample opened a new keyframe.
	CONSTELLATION_FUSION_SAMPLE_NEW_KEYFRAME,
	//! The sample joined a keyframe another camera of the mosaic already opened.
	CONSTELLATION_FUSION_SAMPLE_EXISTING_KEYFRAME,

	//! Dropped. The camera had no world pose at the time of the sample.
	CONSTELLATION_FUSION_SAMPLE_DROPPED_NO_CAMERA_POSE,
	//! Dropped. The fusion thread was too far behind and its queue was full.
	CONSTELLATION_FUSION_SAMPLE_DROPPED_QUEUE_FULL,
	//! Dropped. The keyframe it belongs to was already sealed.
	CONSTELLATION_FUSION_SAMPLE_DROPPED_KEYFRAME_SEALED,
	//! Dropped. Older than the newest keyframe, but matching none in the window.
	CONSTELLATION_FUSION_SAMPLE_DROPPED_TOO_OLD,
	//! Dropped. The sample had no usable pose for the device, or an observation couldn't be built from it.
	CONSTELLATION_FUSION_SAMPLE_DROPPED_BAD_OBSERVATION,
};

enum t_constellation_tracker_trace_log_fusion_solve_result
{
	//! The sample was dropped before it was folded in, so no solve could ever cover it.
	CONSTELLATION_FUSION_SOLVE_NOT_RUN,
	//! A solve covered the sample's keyframe, so its pose is available to consumers.
	CONSTELLATION_FUSION_SOLVE_SUCCEEDED,
	//! A solve was run while the sample was waiting, but its result was unusable, so fusion was reset.
	CONSTELLATION_FUSION_SOLVE_FAILED,
	/*!
	 * No solve ever covered the sample's keyframe before it left the window, either by being evicted, by fusion being
	 * reset for another reason, or by fusion being destroyed.
	 */
	CONSTELLATION_FUSION_SOLVE_NEVER_COVERED,
};

struct t_constellation_tracker_trace_log_sensor_fusion_entry
{
	//! The device whose sensor fusion this entry is from.
	t_constellation_device_id_t device_id;

	//! The difference from frame timestamp -> sensor fusion starting on this sample (includes any time queued).
	time_duration_ns start_latency_ns;

	/*!
	 * The difference from frame timestamp -> the fused pose from this sample being available to consumers through
	 * @ref t_constellation_tracker_get_tracked_pose, ie the end of the first solve that covered its keyframe, which
	 * may be several passes after it was folded in. Only valid if @ref solve_result is
	 * @ref CONSTELLATION_FUSION_SOLVE_SUCCEEDED, -1 otherwise.
	 */
	time_duration_ns pipeline_latency_ns;

	//! What happened to the sample.
	enum t_constellation_tracker_trace_log_fusion_sample_result sample_result;
	//! The amount of time spent folding the sample into the window.
	time_duration_ns process_time_ns;

	//! How the sample's wait for a solve ended.
	enum t_constellation_tracker_trace_log_fusion_solve_result solve_result;
	/*!
	 * The amount of time spent in the fusion pass that covered (or failed on) the sample's keyframe, preintegration,
	 * building and solving. Shared by every sample released by the same pass. Zero for any other @ref solve_result.
	 */
	time_duration_ns solve_time_ns;
	//! How many samples were folded in the same pass as this one, zero if the sample was dropped before a pass.
	uint32_t batch_size;
	//! How many successful solves ran without covering the sample's keyframe, before its wait ended.
	uint32_t num_solves_waited;
};

struct t_constellation_tracker_trace_log_entry
{
	enum t_constellation_tracker_trace_log_entry_type type;
	union {
		struct t_constellation_tracker_trace_log_blobwatch_entry blobwatch;
		struct t_constellation_tracker_trace_log_blob_visualizer_entry blob_visualizer;
		struct t_constellation_tracker_trace_log_matching_entry matching;
		struct t_constellation_tracker_trace_log_sensor_fusion_entry sensor_fusion;
	};
};

/*!
 * Contains a full trace log throughout the tracking pipeline.
 *
 * @extends xrt_reference
 */
struct t_constellation_tracker_trace_log
{
	struct xrt_reference reference;
	//! The sink this trace log is pushed into once the last reference is released, not owned.
	struct t_constellation_trace_log_sink *sink;

	struct t_constellation_tracker_trace_log_entry entries[CONSTELLATION_TRACE_LOG_ENTRY_CAPACITY];
	/*!
	 * Number of entries, atomic since multiple pipeline stages may push entries concurrently. Never exceeds the size
	 * of @ref entries. Only stable once the last reference is released (ie in the sink).
	 */
	xrt_atomic_s32_t num_entries;
};


/*!
 * Creates a new trace log. A null @p sink is treated as "disabled", in which case this returns null.
 */
struct t_constellation_tracker_trace_log *
t_constellation_tracker_trace_log_create(struct t_constellation_trace_log_sink *sink);

/*!
 * Update the reference counts on frame(s).
 *
 * @param[in,out] dst Pointer to a object reference: if the object reference is
 *                non-null will decrement its counter. The reference that
 *                @p dst points to will be set to @p src.
 * @param[in] src New object for @p dst to refer to (may be null).
 *                If non-null, will have its refcount increased.
 *
 * @ingroup xrt_iface
 * @relates t_constellation_tracker_trace_log
 */
void
t_constellation_tracker_trace_log_ref(struct t_constellation_tracker_trace_log **dst,
                                      struct t_constellation_tracker_trace_log *src);

void
t_constellation_tracker_trace_log_push_entry(struct t_constellation_tracker_trace_log *trace_log,
                                             const struct t_constellation_tracker_trace_log_entry *entry);

#ifdef __cplusplus
}
#endif
