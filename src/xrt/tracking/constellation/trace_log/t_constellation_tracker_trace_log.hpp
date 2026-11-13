// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  C++ helpers for the constellation tracker trace log system.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "t_constellation_tracker_trace_log.h"

#include <utility>


namespace xrt::tracking::constellation {

/*!
 * Owning, reference counted handle to a @ref t_constellation_tracker_trace_log.
 *
 * Copying takes a new reference, moving transfers the reference, and destruction releases it, so anything holding
 * one of these can use the default copy/move/destroy semantics without leaking.
 */
class TraceLogRef
{
public:
	TraceLogRef() = default;

	//! Takes a new reference to @p trace_log (may be null), the caller keeps its own reference.
	explicit TraceLogRef(t_constellation_tracker_trace_log *trace_log)
	{
		t_constellation_tracker_trace_log_ref(&this->trace_log, trace_log);
	}

	TraceLogRef(const TraceLogRef &other)
	{
		t_constellation_tracker_trace_log_ref(&this->trace_log, other.trace_log);
	}

	TraceLogRef(TraceLogRef &&other) noexcept : trace_log(std::exchange(other.trace_log, nullptr)) {}

	TraceLogRef &
	operator=(const TraceLogRef &other)
	{
		// Self-assignment safe, the ref function is a no-op if dst == src.
		t_constellation_tracker_trace_log_ref(&this->trace_log, other.trace_log);
		return *this;
	}

	TraceLogRef &
	operator=(TraceLogRef &&other) noexcept
	{
		if (this != &other) {
			reset();
			this->trace_log = std::exchange(other.trace_log, nullptr);
		}
		return *this;
	}

	~TraceLogRef()
	{
		reset();
	}

	void
	reset()
	{
		t_constellation_tracker_trace_log_ref(&this->trace_log, nullptr);
	}

	//! Non-owning access to the trace log, may be null if tracing is disabled.
	t_constellation_tracker_trace_log *
	get() const
	{
		return this->trace_log;
	}

	explicit
	operator bool() const
	{
		return this->trace_log != nullptr;
	}

private:
	t_constellation_tracker_trace_log *trace_log{nullptr};
};

} // namespace xrt::tracking::constellation
