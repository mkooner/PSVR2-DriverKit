// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Where the driver's headset and controller data come from. No OpenVR and no Monado types here: the
 *         driver (ovrn_hmd.hpp, ovrn_controller.hpp) only sees this interface.
 * @ingroup st_ovrn
 */

#pragma once

#include <cstdint>
#include <ctime>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ovrn {

//! CLOCK_MONOTONIC in ns: the one clock of the driver and its backends.
inline int64_t
now_ns()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

struct Vec3
{
	double x = 0, y = 0, z = 0;
};

struct Quat
{
	double x = 0, y = 0, z = 0, w = 1;
};

struct Pose
{
	Quat orientation;
	Vec3 position;
};

//! A device pose at one time, in the backend's tracking space (right-handed, +y up, -z forward, metres).
struct TrackedPose
{
	Pose pose;
	Vec3 linear_velocity;  //!< m/s, tracking space
	Vec3 angular_velocity; //!< rad/s, tracking space
	bool orientation_valid = false;
	bool orientation_tracked = false;
	bool position_valid = false;
	bool position_tracked = false;
};

//! Half angles in radians, signed as OpenXR's XrFovf (left and down negative).
struct Fov
{
	float left = -0.785f, right = 0.785f, up = 0.785f, down = -0.785f;
};

struct EyeInfo
{
	uint32_t viewport_x = 0, viewport_y = 0, viewport_w = 0, viewport_h = 0; //!< on the panel, pixels
	uint32_t render_w = 0, render_h = 0;                                     //!< recommended render target
	Fov fov;
	Pose eye_in_head;
};

struct DisplayInfo
{
	uint32_t width = 0, height = 0; //!< the whole panel, pixels
	double refresh_hz = 90.0;
	double vsync_to_photons_s = 0.0;
	double photons_to_vblank_s = -1.0; //!< < 0: not reported
	int distortion_mesh_resolution = 0; //!< 0: SteamVR's default
	EyeInfo eye[2];
};

struct DeviceInfo
{
	std::string manufacturer;
	std::string model;
	std::string serial;
	std::string controller_type; //!< SteamVR's name for the device type (empty: none)
	std::string input_profile;   //!< path of its input profile in the driver's resources (empty: none)
	std::string render_model;    //!< empty: none
	//! More string properties (SteamVR's property number, value), e.g. the status icons.
	std::vector<std::pair<int, std::string>> string_props;
	//! More properties by SteamVR's property number.
	std::vector<std::pair<int, bool>> bool_props;
	std::vector<std::pair<int, int32_t>> int_props;
	std::vector<std::pair<int, float>> float_props;
	//! A hidden-area mask file to hand SteamVR ("ham_vert <eye> <type> <u> <v>" lines); empty: none.
	std::string hidden_area_file;
};

//! Where the backend's tracking space lies in SteamVR's world (the room calibration): p_world = rotation * p + t.
struct WorldTransform
{
	Quat rotation;
	Vec3 translation;
};

enum class InputKind
{
	Boolean,
	ScalarOneSided, //!< 0..1
	ScalarTwoSided, //!< -1..1
};

struct InputDesc
{
	std::string path; //!< SteamVR's component path, e.g. /input/l2/value
	InputKind kind;
};

enum class Hand
{
	Left = 0,
	Right = 1,
};

//! Distortion of one output point: where to sample the rendered eye image for red, green and blue.
struct DistortionUv
{
	float r[2], g[2], b[2];
};

class Backend
{
public:
	virtual ~Backend() = default;

	//! Opens the devices. False with a reason when there is no headset.
	virtual bool
	start(std::string *why) = 0;
	virtual void
	stop() = 0;
	virtual const char *
	name() const = 0;

	virtual DeviceInfo
	headInfo() = 0;
	virtual DisplayInfo
	display() = 0;
	//! The eyes' poses in the head now (they change with the IPD dial). False: unchanged since display().
	virtual bool
	eyePoses(Pose out[2]) = 0;
	//! The head pose at @p at_ns (the backend predicts when it is in the future). False: no pose at all.
	virtual bool
	headPose(int64_t at_ns, TrackedPose *out) = 0;
	//! False: no distortion for this point (the driver then uses the identity).
	virtual bool
	distortion(int eye, float u, float v, DistortionUv *out) = 0;

	virtual bool
	hasController(Hand hand) = 0;
	virtual DeviceInfo
	controllerInfo(Hand hand) = 0;
	virtual bool
	controllerPose(Hand hand, int64_t at_ns, TrackedPose *out) = 0;
	//! The controller's inputs, in the order controllerInputs() fills them, and its haptic output's path (or empty).
	virtual std::vector<InputDesc>
	controllerInputDescs(Hand hand, std::string *out_haptic_path) = 0;
	//! The inputs' values now (booleans as 0 or 1). Returns how many were filled.
	virtual size_t
	controllerInputs(Hand hand, float *out, size_t max) = 0;
	virtual void
	haptic(Hand hand, float duration_s, float frequency_hz, float amplitude) = 0;

	//! The room: the same transform for every device, and SteamVR's universe it belongs to (2: the saved room).
	virtual WorldTransform
	worldFromDriver() = 0;
	virtual uint64_t
	universe() = 0;
	//! A chaperone for SteamVR when the backend has no room of its own (empty: none).
	virtual std::string
	chaperoneJson() = 0;
	//! Once per SteamVR frame (RunFrame thread): read the controllers' inputs, room housekeeping.
	virtual void
	runFrame(int64_t now_ns) = 0;
};

//! A simulated headset and two controllers at fixed poses (no hardware): for the driver skeleton and its tests.
std::unique_ptr<Backend>
make_sim_backend();

//! Monado's devices (the PS VR2 and Sense drivers, unchanged), created inside vrserver without a compositor.
std::unique_ptr<Backend>
make_monado_backend();

} // namespace ovrn
