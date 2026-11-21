// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Disconnect a Bluetooth device through BlueZ (the system's Bluetooth service, over the system D-Bus).
 *
 * The SteamVR driver disconnects the Sense controllers when SteamVR closes, so they don't stay on (as Sony's driver
 * does; DriverKit's launcher did it with bluetoothctl, but only for a SteamVR it started itself).
 * @ingroup st_ovrn
 */

#pragma once

#include <string>

namespace ovrn {

/*!
 * Ask BlueZ to disconnect the device with Bluetooth address @p address ("88:03:4C:C1:17:6F"), on whichever adapter
 * has it (all adapters asked at once). Blocks for at most about 1 s. False with a reason in @p why if it couldn't (no D-Bus support built,
 * no system bus, no such device).
 */
bool
bluetooth_disconnect(const std::string &address, std::string *why);

//! Whether @p s looks like a Bluetooth address (six hex pairs with colons).
bool
is_bluetooth_address(const std::string &s);

} // namespace ovrn
