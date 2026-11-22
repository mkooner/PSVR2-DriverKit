// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  DriverKit's settings file and the settings read from it by more than one part of the driver.
 * @ingroup st_ovrn
 */

#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace ovrn {

/*!
 * DriverKit's settings file ($XDG_CONFIG_HOME/PSVR2-DriverKit/settings.conf, KEY=VALUE lines), read by the driver
 * itself so a setting applies however SteamVR is started (from Steam too, not only from DriverKit).
 */
inline std::string
driverkit_settings_path()
{
	const char *xdg = getenv("XDG_CONFIG_HOME");
	const char *home = getenv("HOME");
	const std::string base = xdg != NULL && *xdg != '\0' ? std::string(xdg)
	                         : home != NULL           ? std::string(home) + "/.config"
	                                                  : std::string();
	return base.empty() ? std::string() : base + "/PSVR2-DriverKit/settings.conf";
}

//! One key's value in DriverKit's settings file (empty: not there, or no file).
inline std::string
driverkit_setting(const char *key)
{
	const std::string path = driverkit_settings_path();
	FILE *f = path.empty() ? NULL : fopen(path.c_str(), "r");
	if (f == NULL) {
		return {};
	}
	std::string value;
	char line[512];
	const size_t n = strlen(key);
	while (fgets(line, sizeof(line), f) != NULL) {
		if (strncmp(line, key, n) == 0 && line[n] == '=') {
			value = line + n + 1;
			while (!value.empty() && (value.back() == '\n' || value.back() == '\r' || value.back() == ' ')) {
				value.pop_back();
			}
		}
	}
	fclose(f);
	return value;
}

/*!
 * What the headset's cameras do in SteamVR (DriverKit's "Headset cameras in SteamVR"):
 *  - on: as always: SteamVR's camera switched on for the session (see-through on the headset button, Room View)
 *    and the picture streamed while SteamVR reads it.
 *  - noimages: the same SteamVR camera setup, but no picture is ever sent.
 *  - off: no camera for SteamVR at all (no camera component, SteamVR's camera settings left alone).
 *
 * For finding what crashes SteamVR's compositor on an Intel Arc A750 (2026-11-22, 8 dumps): Mesa 26.3.1's Intel
 * Vulkan driver divides by a descriptor binding's array_size of 0 while compiling one of the compositor's shaders
 * (build_packed_binding_table, anv_nir_apply_pipeline_layout.c:2587, still unguarded on Mesa main), about 2 s after
 * every start with this driver and never with Sony's, which sets up no camera view. Which compositor feature builds
 * that shader is the open question: one start per mode answers whether it's the camera view or our picture.
 */
enum class CameraMode
{
	On,
	NoImages,
	Off,
};

inline const char *
camera_mode_name(CameraMode m)
{
	return m == CameraMode::On ? "on" : m == CameraMode::NoImages ? "noimages" : "off";
}

/*!
 * The camera mode: OVRN_CAMERA_MODE (DriverKit's launch), else CAMERA in DriverKit's settings file (SteamVR started
 * from Steam), else on. OVRN_CAMERA=0 (older switch) is off. Anything else is logged by the caller and taken as on.
 */
inline CameraMode
camera_mode(std::string *out_source, std::string *out_invalid)
{
	const char *old = getenv("OVRN_CAMERA");
	if (old != nullptr && strcmp(old, "0") == 0) {
		*out_source = "OVRN_CAMERA=0";
		return CameraMode::Off;
	}
	const char *env = getenv("OVRN_CAMERA_MODE");
	std::string v = env != nullptr ? env : "";
	*out_source = "OVRN_CAMERA_MODE";
	if (v.empty()) {
		v = driverkit_setting("CAMERA");
		*out_source = "DriverKit's setting";
	}
	if (v.empty()) {
		*out_source = "default";
		return CameraMode::On;
	}
	if (v == "on") {
		return CameraMode::On;
	}
	if (v == "noimages") {
		return CameraMode::NoImages;
	}
	if (v == "off") {
		return CameraMode::Off;
	}
	*out_invalid = v;
	*out_source = "default, the setting wasn't on, noimages or off";
	return CameraMode::On;
}

} // namespace ovrn
