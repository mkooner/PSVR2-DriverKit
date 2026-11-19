// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Entry point of the native PS VR2 SteamVR driver (st_ovrn).
 *
 * @defgroup st_ovrn Native PS VR2 SteamVR driver
 *
 * A SteamVR (OpenVR) driver whose structure follows ALVR's server_openvr (alvr-org/ALVR b2952e712f90, MIT:
 * alvr_server.cpp, TrackedDevice.*, HMD.*, Controller.*): one provider, a common tracked-device base class, an HMD
 * that is a plain IVRDisplayComponent (SteamVR owns the display, composites, paces and reprojects) and whose pose
 * is predicted by the driver and handed over with zero velocity. The headset and controller data come from a
 * backend (ovrn_backend.hpp) that knows nothing about OpenVR.
 *
 * @ingroup xrt
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//! Implementation of HmdDriverFactory.
void *
ovrn_hmd_driver_impl(const char *pInterfaceName, int *pReturnCode);

#ifdef __cplusplus
}
#endif
