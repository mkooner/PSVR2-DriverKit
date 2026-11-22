// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Startup milestone log lines: "startup: <what> (mono <ms> ms)".
 *
 * One line per milestone (driver Init, USB up, first head pose, LED timing found, first controller pose...), each
 * with the monotonic clock, so lines from the SteamVR plugin and from Monado's drivers (one process, separate logs,
 * not all of them timestamped) line up into one timeline; psvr2-test/summarize.sh prints it. Callers log each
 * milestone once (their own flag): never from a per-frame path without one.
 * @ingroup aux_util
 */

#pragma once

#include "os/os_time.h"
#include "util/u_logging.h"

#include <stdarg.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline void
u_startup_mark(const char *fmt, ...) XRT_PRINTF_FORMAT(1, 2);

static inline void
u_startup_mark(const char *fmt, ...)
{
	char what[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(what, sizeof(what), fmt, args);
	va_end(args);
	// Always at info level: one line per milestone, and the global level (warn by default) would hide them.
	U_LOG_IFL_I(U_LOGGING_INFO, "startup: %s (mono %.1f ms)", what, (double)os_monotonic_get_ns() / 1e6);
}

#ifdef __cplusplus
}
#endif
