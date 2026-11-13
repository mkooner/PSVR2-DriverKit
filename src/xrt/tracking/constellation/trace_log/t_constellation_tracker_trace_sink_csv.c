// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Simple trace log sink that writes the constellation tracker trace logs out to CSV files.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#include "xrt/xrt_compiler.h"

#include "util/u_misc.h"

#include "constellation/trace_log/t_constellation_tracker_trace_sink_csv.h"

#include <inttypes.h>
#include <stdarg.h>


/*!
 * @implements t_constellation_trace_log_sink
 */
struct t_constellation_tracker_trace_sink_csv
{
	struct t_constellation_trace_log_sink base;

	struct t_constellation_tracker_trace_sink_csv_files files;
	uint32_t mosaic_index;
	uint32_t camera_index;

	//! Sinks can be pushed into from multiple threads (eg the fast and slow tracker threads).
	xrt_atomic_s32_t next_trace_index;
};

struct line
{
	char buf[512];
	size_t len;
};


/*
 *
 * Helper functions
 *
 */

static inline struct t_constellation_tracker_trace_sink_csv *
trace_sink_csv(struct t_constellation_trace_log_sink *tls)
{
	return (struct t_constellation_tracker_trace_sink_csv *)tls;
}

XRT_PRINTF_FORMAT(2, 3) static void
line_append(struct line *l, const char *fmt, ...)
{
	if (l->len >= sizeof(l->buf)) {
		return;
	}

	va_list args;
	va_start(args, fmt);
	int ret = vsnprintf(l->buf + l->len, sizeof(l->buf) - l->len, fmt, args);
	va_end(args);

	if (ret > 0) {
		l->len += (size_t)ret;
	}
}

static void
line_finish(struct line *l, FILE *file)
{
	line_append(l, "\n");
	// A single call, so concurrent writers to the same file don't interleave within a line.
	fputs(l->buf, file);
}

//! Appends `,<value>` if @p present, otherwise just `,`.
static void
line_append_duration(struct line *l, bool present, time_duration_ns value)
{
	if (present) {
		line_append(l, ",%" PRId64, value);
	} else {
		line_append(l, ",");
	}
}

static const char *
solve_type_str(enum t_constellation_tracker_trace_log_solve_type type)
{
	switch (type) {
	case CONSTELLATION_SOLVE_TYPE_INVALID: return "invalid";
	case CONSTELLATION_SOLVE_TYPE_PREDICTED_PRIOR: return "predicted-prior";
	case CONSTELLATION_SOLVE_TYPE_BLOB_LABELS: return "blob-labels";
	case CONSTELLATION_SOLVE_TYPE_LAST_KNOWN_POSE: return "last-known-pose";
	case CONSTELLATION_SOLVE_TYPE_SHALLOW_CORRESPONDENCE_SEARCH: return "shallow-search";
	case CONSTELLATION_SOLVE_TYPE_DEEP_CORRESPONDENCE_SEARCH: return "deep-search";
	case CONSTELLATION_SOLVE_NO_BLOBS_AFTER_RANSAC: return "no-solve-no-blobs-after-ransac";
	case CONSTELLATION_SOLVE_TYPE_NO_SOLVE_FAST_SOLVED_RECENTLY: return "no-solve-fast-solved-recently";
	case CONSTELLATION_SOLVE_TYPE_NO_SOLVE_DELAYED_DEEP: return "no-solve-delayed-deep";
	case CONSTELLATION_SOLVE_TYPE_NO_SOLVE: return "no-solve";
	}

	return "unknown";
}

static const char *
fusion_sample_result_str(enum t_constellation_tracker_trace_log_fusion_sample_result result)
{
	switch (result) {
	case CONSTELLATION_FUSION_SAMPLE_INVALID: return "invalid";
	case CONSTELLATION_FUSION_SAMPLE_NEW_KEYFRAME: return "new-keyframe";
	case CONSTELLATION_FUSION_SAMPLE_EXISTING_KEYFRAME: return "existing-keyframe";
	case CONSTELLATION_FUSION_SAMPLE_DROPPED_NO_CAMERA_POSE: return "dropped-no-camera-pose";
	case CONSTELLATION_FUSION_SAMPLE_DROPPED_QUEUE_FULL: return "dropped-queue-full";
	case CONSTELLATION_FUSION_SAMPLE_DROPPED_KEYFRAME_SEALED: return "dropped-keyframe-sealed";
	case CONSTELLATION_FUSION_SAMPLE_DROPPED_TOO_OLD: return "dropped-too-old";
	case CONSTELLATION_FUSION_SAMPLE_DROPPED_BAD_OBSERVATION: return "dropped-bad-observation";
	}

	return "unknown";
}

static const char *
fusion_solve_result_str(enum t_constellation_tracker_trace_log_fusion_solve_result result)
{
	switch (result) {
	case CONSTELLATION_FUSION_SOLVE_NOT_RUN: return "not-run";
	case CONSTELLATION_FUSION_SOLVE_SUCCEEDED: return "succeeded";
	case CONSTELLATION_FUSION_SOLVE_FAILED: return "failed";
	case CONSTELLATION_FUSION_SOLVE_NEVER_COVERED: return "never-covered";
	}

	return "unknown";
}

//! Find the sensor fusion entry for @p device_id, if that device's sensor fusion was given the sample.
static const struct t_constellation_tracker_trace_log_sensor_fusion_entry *
find_fusion_entry(const struct t_constellation_tracker_trace_log *trace_log, t_constellation_device_id_t device_id)
{
	for (int32_t i = 0; i < trace_log->num_entries; i++) {
		const struct t_constellation_tracker_trace_log_entry *entry = &trace_log->entries[i];
		if (entry->type == CONSTELLATION_TRACE_LOG_ENTRY_SENSOR_FUSION &&
		    entry->sensor_fusion.device_id == device_id) {
			return &entry->sensor_fusion;
		}
	}

	return NULL;
}

static void
write_timings(struct t_constellation_tracker_trace_sink_csv *csv,
              int32_t trace_index,
              const struct t_constellation_tracker_trace_log *trace_log)
{
	// Pick out the last entry of each type, so the columns are always in the same order.
	const struct t_constellation_tracker_trace_log_entry *by_type[CONSTELLATION_TRACE_LOG_MAX_ENTRIES] = {0};
	for (int32_t i = 0; i < trace_log->num_entries; i++) {
		const struct t_constellation_tracker_trace_log_entry *entry = &trace_log->entries[i];
		if (entry->type < CONSTELLATION_TRACE_LOG_MAX_ENTRIES) {
			by_type[entry->type] = entry;
		}
	}

	const struct t_constellation_tracker_trace_log_entry *bw = by_type[CONSTELLATION_TRACE_LOG_ENTRY_BLOBWATCH];
	const struct t_constellation_tracker_trace_log_entry *vis = by_type[CONSTELLATION_TRACE_LOG_BLOB_VISUALIZER];
	const struct t_constellation_tracker_trace_log_entry *match = by_type[CONSTELLATION_TRACE_LOG_ENTRY_MATCHING];

	struct line l = {0};
	line_append(&l, "%u,%u,%" PRId32, csv->mosaic_index, csv->camera_index, trace_index);

	line_append_duration(&l, bw != NULL, bw ? bw->blobwatch.pipeline_latency_ns : 0);
	line_append_duration(&l, bw != NULL, bw ? bw->blobwatch.process_time_ns : 0);

	line_append_duration(&l, vis != NULL, vis ? vis->blob_visualizer.pipeline_latency_ns : 0);
	line_append_duration(&l, vis != NULL, vis ? vis->blob_visualizer.process_time_ns : 0);

	line_append_duration(&l, match != NULL, match ? match->matching.pipeline_latency_ns : 0);
	line_append_duration(&l, match && match->matching.fast_taken, match ? match->matching.fast_duration_ns : 0);
	line_append_duration(&l, match && match->matching.slow_taken, match ? match->matching.slow_duration_ns : 0);

	line_finish(&l, csv->files.timings);
}

static void
write_matching(struct t_constellation_tracker_trace_sink_csv *csv,
               int32_t trace_index,
               const struct t_constellation_tracker_trace_log *trace_log,
               const struct t_constellation_tracker_trace_log_matching_entry *matching)
{
	for (uint32_t i = 0; i < ARRAY_SIZE(matching->devices); i++) {
		const struct t_constellation_tracker_trace_log_matched_device *device = &matching->devices[i];

		if (device->device_id < 0 || (size_t)device->device_id >= ARRAY_SIZE(csv->files.device_matching)) {
			continue;
		}

		FILE *file = csv->files.device_matching[device->device_id];
		if (file == NULL) {
			continue;
		}

		struct line l = {0};
		line_append(&l, "%u,%u,%" PRId32, csv->mosaic_index, csv->camera_index, trace_index);
		line_append_duration(&l, true, matching->pipeline_latency_ns);
		line_append(&l, ",%d", matching->fast_taken ? 1 : 0);
		line_append_duration(&l, matching->fast_taken, matching->fast_duration_ns);
		line_append(&l, ",%d", matching->slow_taken ? 1 : 0);
		line_append_duration(&l, matching->slow_taken, matching->slow_duration_ns);
		line_append(&l, ",%s", solve_type_str(device->device_solve_type));

		// Sensor fusion columns, empty if the device's sensor fusion was never given this sample.
		const struct t_constellation_tracker_trace_log_sensor_fusion_entry *fusion =
		    find_fusion_entry(trace_log, device->device_id);
		if (fusion != NULL) {
			line_append_duration(&l, true, fusion->start_latency_ns);
			line_append_duration(&l, fusion->solve_result == CONSTELLATION_FUSION_SOLVE_SUCCEEDED,
			                     fusion->pipeline_latency_ns);
			line_append(&l, ",%s", fusion_sample_result_str(fusion->sample_result));
			line_append_duration(&l, true, fusion->process_time_ns);
			line_append(&l, ",%s", fusion_solve_result_str(fusion->solve_result));
			line_append_duration(&l,
			                     fusion->solve_result == CONSTELLATION_FUSION_SOLVE_SUCCEEDED ||
			                         fusion->solve_result == CONSTELLATION_FUSION_SOLVE_FAILED,
			                     fusion->solve_time_ns);
			line_append(&l, ",%u", fusion->batch_size);
			line_append(&l, ",%u", fusion->num_solves_waited);
		} else {
			line_append(&l, ",,,,,,,,");
		}

		line_finish(&l, file);
	}
}


/*
 *
 * Sink functions
 *
 */

static void
trace_sink_csv_push_trace_log(struct t_constellation_trace_log_sink *tls,
                              const struct t_constellation_tracker_trace_log *trace_log)
{
	struct t_constellation_tracker_trace_sink_csv *csv = trace_sink_csv(tls);

	int32_t trace_index = xrt_atomic_s32_inc_return(&csv->next_trace_index) - 1;

	if (csv->files.timings != NULL) {
		write_timings(csv, trace_index, trace_log);
	}

	for (int32_t i = 0; i < trace_log->num_entries; i++) {
		const struct t_constellation_tracker_trace_log_entry *entry = &trace_log->entries[i];
		if (entry->type == CONSTELLATION_TRACE_LOG_ENTRY_MATCHING) {
			write_matching(csv, trace_index, trace_log, &entry->matching);
		}
	}
}


/*
 *
 * 'Exported' functions
 *
 */

struct t_constellation_trace_log_sink *
t_constellation_tracker_trace_sink_csv_create(const struct t_constellation_tracker_trace_sink_csv_files *files,
                                              uint32_t mosaic_index,
                                              uint32_t camera_index)
{
	struct t_constellation_tracker_trace_sink_csv *csv = U_TYPED_CALLOC(struct t_constellation_tracker_trace_sink_csv);
	csv->base.push_trace_log = trace_sink_csv_push_trace_log;
	csv->files = *files;
	csv->mosaic_index = mosaic_index;
	csv->camera_index = camera_index;

	return &csv->base;
}

void
t_constellation_tracker_trace_sink_csv_destroy(struct t_constellation_trace_log_sink **sink_ptr)
{
	struct t_constellation_trace_log_sink *sink = *sink_ptr;
	if (sink == NULL) {
		return;
	}

	free(trace_sink_csv(sink));
	*sink_ptr = NULL;
}

void
t_constellation_tracker_trace_sink_csv_write_timings_header(FILE *file)
{
	fputs(
	    "mosaic-idx,cam-idx,trace-idx,"
	    "blobwatch-latency-ns,blobwatch-process-ns,"
	    "visualizer-latency-ns,visualizer-process-ns,"
	    "matching-latency-ns,matching-fast-ns,matching-slow-ns\n",
	    file);
}

void
t_constellation_tracker_trace_sink_csv_write_matching_header(FILE *file)
{
	fputs(
	    "mosaic-idx,cam-idx,trace-idx,"
	    "latency-ns,fast-taken,fast-ns,slow-taken,slow-ns,solve-type,"
	    "fusion-start-latency-ns,pose-available-latency-ns,fusion-sample-result,fusion-process-ns,fusion-solve-result,fusion-solve-ns,"
	    "fusion-batch-size,fusion-solves-waited\n",
	    file);
}
