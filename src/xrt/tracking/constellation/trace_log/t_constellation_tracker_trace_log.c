// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation of the constellation tracker trace log system.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup xrt_iface
 */

#include "util/u_misc.h"
#include "util/u_debug.h"

#include "constellation/trace_log/t_constellation_tracker_trace_log.h"


/*
 *
 * Helper functions
 *
 */

static void
t_constellation_tracker_trace_log_destroy(struct t_constellation_tracker_trace_log *trace_log)
{
	t_constellation_trace_log_sink_push_trace_log(trace_log->sink, trace_log);
	free(trace_log);
}

/*
 *
 * 'Exported' functions
 *
 */

struct t_constellation_tracker_trace_log *
t_constellation_tracker_trace_log_create(struct t_constellation_trace_log_sink *sink)
{
	// No sink is our "disabled" marker
	if (sink == NULL) {
		return NULL;
	}

	struct t_constellation_tracker_trace_log *trace_log = U_TYPED_CALLOC(struct t_constellation_tracker_trace_log);
	trace_log->reference.count = 1;
	trace_log->sink = sink;

	return trace_log;
}

void
t_constellation_tracker_trace_log_ref(struct t_constellation_tracker_trace_log **dst,
                                      struct t_constellation_tracker_trace_log *src)
{
	struct t_constellation_tracker_trace_log *old_dst = *dst;

	if (old_dst == src) {
		return;
	}

	if (src) {
		xrt_reference_inc(&src->reference);
	}

	*dst = src;

	if (old_dst) {
		if (xrt_reference_dec_and_is_zero(&old_dst->reference)) {
			t_constellation_tracker_trace_log_destroy(old_dst);
		}
	}
}

void
t_constellation_tracker_trace_log_push_entry(struct t_constellation_tracker_trace_log *trace_log,
                                             const struct t_constellation_tracker_trace_log_entry *entry)
{
	// If the trace log isn't enabled (they got a null entry), they shouldn't call this, to discourage creating
	// entries on the stack for no reason.
	assert(trace_log != NULL);

	// Atomically claim a slot, multiple pipeline stages (eg the blob visualizer and tracker threads) may push entries
	// at the same time. Uses a CAS loop rather than an increment so num_entries never exceeds the entries array.
	int32_t index;
	do {
		index = trace_log->num_entries;

		// We only have so many steps that push new entries.
		if (index >= (int32_t)ARRAY_SIZE(trace_log->entries)) {
			assert(false && "Too many entries pushed to trace log");
			return;
		}
	} while (xrt_atomic_s32_cmpxchg(&trace_log->num_entries, index, index + 1) != index);

	// The slot is ours alone. The entry is visible to the sink, since releasing our reference is a full barrier.
	trace_log->entries[index] = *entry;
}
