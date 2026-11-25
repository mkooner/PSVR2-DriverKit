// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Where the driver's headset and controller data come from. No OpenVR and no Monado types here: the
 *         driver (ovrn_hmd.hpp, ovrn_controller.hpp) only sees this interface.
 * @ingroup st_ovrn
 */

#pragma once

#include "ovrn_gesture_state.hpp"

#include <cstdint>
#include <ctime>
#include <memory>
#include <functional>
#include <string>
#include <utility>
#include <vector>

class OvrnDirectMode; // ovrn_direct_mode.hpp

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

/*!
 * One tracked hand's joints at one time: OpenXR's 26 (XR_EXT_hand_tracking order and axes), each in the backend's
 * tracking space (as TrackedPose), the same source OpenXR apps get.
 */
struct HandJoints
{
	static constexpr int kCount = 26;
	bool active = false;      //!< the hand is tracked now (else nothing below is meaningful)
	int64_t timestamp_ns = 0; //!< when the joints were seen or predicted for (monotonic)
	Pose joint[kCount];
	bool joint_valid[kCount] = {}; //!< position and orientation valid
	float radius[kCount] = {};     //!< metres
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
	/*!
	 * Controllers: what legacy (pre-SteamVR Input) games are shown, the input profile's legacy binding (touch, vive,
	 * knuckles or native) and how it was chosen, for the compatibility line in the log. Empty: not a controller.
	 */
	std::string legacy_mode;
	std::string legacy_mode_source;
	//! More string properties (SteamVR's property number, value), e.g. the status icons.
	std::vector<std::pair<int, std::string>> string_props;
	//! More properties by SteamVR's property number.
	std::vector<std::pair<int, bool>> bool_props;
	std::vector<std::pair<int, int32_t>> int_props;
	std::vector<std::pair<int, float>> float_props;
	//! A hidden-area mask file to hand SteamVR ("ham_vert <eye> <type> <u> <v>" lines); empty: none.
	std::string hidden_area_file;
	/*!
	 * Controllers: where the hand's grip (OpenXR's grip pose) is in the pose the driver reports (the controller's
	 * SteamVR root, which its render model and input profile poses are relative to). The hand skeleton is fitted in
	 * the grip and placed with this. Identity: the reported pose is the grip.
	 */
	Pose grip_in_root;
};

//! a * b for rotations (Quat is x, y, z, w).
inline Quat
quat_mul(const Quat &a, const Quat &b)
{
	return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
	        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

inline Vec3
quat_rotate(const Quat &q, const Vec3 &v)
{
	const Vec3 u{q.x, q.y, q.z};
	auto cross = [](const Vec3 &a, const Vec3 &b) {
		return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
	};
	const Vec3 c = cross(u, v);
	const Vec3 t{2 * c.x, 2 * c.y, 2 * c.z};
	const Vec3 c2 = cross(u, t);
	return {v.x + q.w * t.x + c2.x, v.y + q.w * t.y + c2.y, v.z + q.w * t.z + c2.z};
}

/*!
 * The controller's root from its grip: root = grip * inverse(grip_in_root), with the velocities of that point (the
 * angular velocity is the same; the linear one adds w x r for the root's offset r from the grip, in tracking space).
 */
inline TrackedPose
root_from_grip(const TrackedPose &grip, const Pose &grip_in_root)
{
	const Quat gi{-grip_in_root.orientation.x, -grip_in_root.orientation.y, -grip_in_root.orientation.z,
	              grip_in_root.orientation.w};
	const Vec3 t = quat_rotate(gi, grip_in_root.position); // inverse: rotation gi, translation -gi * t
	const Vec3 root_in_grip{-t.x, -t.y, -t.z};
	TrackedPose out = grip;
	out.pose.orientation = quat_mul(grip.pose.orientation, gi);
	const Vec3 r = quat_rotate(grip.pose.orientation, root_in_grip);
	out.pose.position = {grip.pose.position.x + r.x, grip.pose.position.y + r.y, grip.pose.position.z + r.z};
	const Vec3 &w = grip.angular_velocity;
	out.linear_velocity = {grip.linear_velocity.x + w.y * r.z - w.z * r.y,
	                       grip.linear_velocity.y + w.z * r.x - w.x * r.z,
	                       grip.linear_velocity.z + w.x * r.y - w.y * r.x};
	return out;
}

/*!
 * The headset's passthrough cameras (the PS VR2's bottom pair), as SteamVR's camera component needs them: per camera
 * the pixel intrinsics, an Extended_FTheta lens model fitted to the camera's own (r_d = t + k0 t^3 + k1 t^5 + k2 t^7 +
 * k3 t^9, t the angle off axis, r_d the distorted radius over the focal length), and its pose in the head (OpenVR
 * camera axes: x right, y up, looking along -z).
 */
struct CameraInfo
{
	bool tracker_poses = false; //!< cam_in_head from the controller tracker's camera chain (OVRN_CAMERA_POSES=tracker)
	uint32_t width = 0, height = 0; //!< one camera's image, pixels
	double fx[2] = {}, fy[2] = {}, cx[2] = {}, cy[2] = {};
	double ftheta[2][8] = {};
	Pose cam_in_head[2];
	float fit_rms_px[2] = {}; //!< how well the fitted lens model matches the camera's own, pixels
	//! Send the picture at half size (OVRN_CAMERA_HALF=1, or CAMERA_HALF=1 in DriverKit's settings file).
	bool half = false;
	std::string half_source = "default";
	//! Never send a picture (DriverKit's CAMERA=noimages, ovrn_settings.hpp): SteamVR's camera set up, no images.
	bool no_images = false;
};

//! One image pair (two BC4 images, 1024 stride x 1016 rows, back to back) and its exposure time (now_ns() clock).
using CameraImageFn = std::function<void(const uint8_t *bc4_pair, size_t size, int64_t exposure_ns)>;

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

	//! Whether the controller is connected now (it may have been once and be off now). Default: hasController.
	virtual bool
	controllerConnected(Hand hand)
	{
		return hasController(hand);
	}

	//! Whether the backend can track bare hands (optical hand tracking).
	virtual bool
	hasHandTracking()
	{
		return false;
	}
	//! The hand's joints at @p at_ns (predicted or the latest), never blocking. False: no hand tracking at all.
	virtual bool
	handJoints(Hand hand, int64_t at_ns, HandJoints *out)
	{
		(void)hand;
		(void)at_ns;
		(void)out;
		return false;
	}

	/*!
	 * The hand's latest gesture measurements (MediaPipe's, from the hand tracker), never blocking. False: the tracker
	 * has none (gestures off or unavailable): the joint-based gestures are used. With true, out->valid tells whether
	 * they belong to the hand as tracked now.
	 */
	//! The hand tracker's profile ("mercury", "mediapipe"; empty: none or not running yet), for per-profile gestures.
	virtual std::string
	handTrackingProfile()
	{
		return {};
	}

	virtual bool
	handGestures(Hand hand, gest::Sample *out)
	{
		(void)hand;
		(void)out;
		return false;
	}

	//! The room: the same transform for every device, and SteamVR's universe it belongs to (2: the saved room).
	virtual WorldTransform
	worldFromDriver() = 0;
	virtual uint64_t
	universe() = 0;
	//! A chaperone for SteamVR when the backend has no room of its own (empty: none).
	virtual std::string
	chaperoneJson() = 0;
	//! The driver-owned chaperone file SteamVR keeps the rooms in (Prop_DriverProvidedChaperonePath_String; empty:
	//! SteamVR's own file).
	virtual std::string
	chaperonePath()
	{
		return {};
	}
	//! The headset's own button (PS VR2: the function button under the visor, its passthrough button on a PS5);
	//! read in runFrame.
	virtual bool
	headButton()
	{
		return false;
	}
	//! The headset's eye tracking (both eyes combined), for SteamVR's /eyetracking input.
	struct EyeGaze
	{
		Vec3 dir;              //!< unit gaze direction in head space, OpenVR axes (-Z forward, +Y up, +X right)
		bool valid;            //!< the headset marked this sample valid
		int64_t sample_ns;     //!< the sample's time, host monotonic
		int64_t packet_age_ns; //!< how long ago its packet arrived (0 = just now)
	};
	//! Whether the headset tracks the eyes (and it was turned on); eyeGaze() then gives the latest sample.
	virtual bool
	hasEyeTracking()
	{
		return false;
	}
	//! False when there's no sample yet.
	virtual bool
	eyeGaze(EyeGaze *out)
	{
		(void)out;
		return false;
	}
	//! Whether the headset has a wear (proximity) sensor; headWorn() is its state, read in runFrame.
	virtual bool
	hasProximity()
	{
		return false;
	}
	virtual bool
	headWorn()
	{
		return false;
	}
	//! Whether the headset has passthrough cameras at all (their calibration may still be on its way).
	virtual bool
	hasCameras()
	{
		return false;
	}
	//! The passthrough cameras (false: none, or their calibration hasn't arrived yet).
	virtual bool
	cameraInfo(CameraInfo *out)
	{
		(void)out;
		return false;
	}
	//! Start the camera images (called on the USB thread; copy and return), or stop them.
	virtual bool
	cameraStart(CameraImageFn fn)
	{
		(void)fn;
		return false;
	}
	virtual void
	cameraStop()
	{}
	//! SteamVR's Room Setup events (VREvent_ChaperoneRoomSetupStarting / ...Committed), RunFrame thread.
	virtual void
	roomSetupEvent(uint32_t event_type)
	{
		(void)event_type;
	}
	//! Once per SteamVR frame (RunFrame thread): read the controllers' inputs, room housekeeping.
	virtual void
	runFrame(int64_t now_ns) = 0;

	/*!
	 * Driver direct mode (DriverKit's setting, off by default; ovrn_settings.hpp direct_mode_wanted): the component
	 * SteamVR hands its finished frames to, shown by Monado's compositor. Owned by the backend; nullptr when direct
	 * mode is off or Monado's compositor couldn't start (SteamVR then drives the display as usual).
	 */
	virtual OvrnDirectMode *
	directMode()
	{
		return nullptr;
	}
};

//! A simulated headset and two controllers at fixed poses (no hardware): for the driver skeleton and its tests.
std::unique_ptr<Backend>
make_sim_backend();

//! Monado's devices (the PS VR2 and Sense drivers, unchanged), created inside vrserver; with Monado's compositor only
//! in driver direct mode.
std::unique_ptr<Backend>
make_monado_backend();

} // namespace ovrn
