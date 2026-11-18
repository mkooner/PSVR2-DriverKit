// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Simple trace log sink that writes the constellation tracker trace logs out to CSV files.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "constellation/trace_log/t_constellation_tracker_trace_log.h"

#include <stdio.h>


#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Files written to by a CSV trace log sink. Not owned by the sink, the caller opens and closes them, and they may be
 * shared between multiple sinks (eg one sink per camera, all writing to the same files). Every line is written with a
 * single stdio call, so lines from different sinks/threads don't interleave.
 *
 * Every line starts with `mosaic-idx,cam-idx,trace-idx`, where `trace-idx` is a per-sink incrementing index, so
 * `mosaic-idx,cam-idx,trace-idx` uniquely identifies a trace log across all files.
 */
struct t_constellation_tracker_trace_sink_csv_files
{
	//! One line per trace log, with the timings of every pipeline step concatenated. Steps that didn't run are empty.
	FILE *timings;

	/*!
	 * Indexed by device ID, one line per matching entry containing that device, followed by that device's sensor
	 * fusion entry for the same trace log (empty if its sensor fusion was never given the sample). Null entries, and
	 * devices with IDs outside of this array, are skipped.
	 */
	FILE *device_matching[XRT_CONSTELLATION_MAX_DEVICES];
};

/*!
 * Create a CSV trace log sink for a single camera.
 *
 * @param[in] files        The files to write to, copied, the files themselves must outlive the sink.
 * @param[in] mosaic_index The mosaic index written at the start of every line.
 * @param[in] camera_index The camera index written at the start of every line.
 *
 * @public @memberof t_constellation_tracker_trace_sink_csv
 */
struct t_constellation_trace_log_sink *
t_constellation_tracker_trace_sink_csv_create(const struct t_constellation_tracker_trace_sink_csv_files *files,
                                              uint32_t mosaic_index,
                                              uint32_t camera_index);

/*!
 * Destroy a sink created by @ref t_constellation_tracker_trace_sink_csv_create. Every trace log created with it must
 * have been released before calling this. Handles nulls, sets your pointer to null.
 *
 * @public @memberof t_constellation_tracker_trace_sink_csv
 */
void
t_constellation_tracker_trace_sink_csv_destroy(struct t_constellation_trace_log_sink **sink_ptr);

//! Write the CSV header line for @ref t_constellation_tracker_trace_sink_csv_files::timings.
void
t_constellation_tracker_trace_sink_csv_write_timings_header(FILE *file);

//! Write the CSV header line for @ref t_constellation_tracker_trace_sink_csv_files::device_matching.
void
t_constellation_tracker_trace_sink_csv_write_matching_header(FILE *file);

#ifdef __cplusplus
}
#endif
