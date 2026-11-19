// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Exports HmdDriverFactory for the native PS VR2 SteamVR driver.
 * @ingroup st_ovrn
 */

#include "ovrn_interface.h"

__attribute__((visibility("default"))) void *
HmdDriverFactory(const char *pInterfaceName, int *pReturnCode)
{
	return ovrn_hmd_driver_impl(pInterfaceName, pReturnCode);
}
