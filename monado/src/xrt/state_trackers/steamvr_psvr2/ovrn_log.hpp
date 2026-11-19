// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Logging to SteamVR's vrserver log.
 * @ingroup st_ovrn
 */

#pragma once

#include "openvr_driver.h"

#include <cstdarg>
#include <cstdio>

namespace ovrn {

inline void
log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

inline void
log(const char *fmt, ...)
{
	char buf[1024];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	if (vr::VRDriverContext() != nullptr && vr::VRDriverLog() != nullptr) {
		vr::VRDriverLog()->Log(buf);
	}
}

} // namespace ovrn
