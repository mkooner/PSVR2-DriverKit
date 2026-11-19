// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Room (playspace) calibration for the SteamVR driver: the room transform, captures and files.
 *
 * Frames, all right handed, Y up (gravity aligned), -Z forward, meters:
 *  - raw:  the poses the driver computes (tracking origin offset applied). For the PS VR2 its origin is wherever the
 *          head was at power on, so it changes every session. It is level: the headset's SLAM is gravity aligned.
 *  - room: a frame fixed to the physical room, saved across sessions. Its floor is y = 0. SteamVR's universe 2 is
 *          set up in this frame, so the boundary drawn in Room Setup stays where it was drawn.
 *
 * T_room_raw (p_room = R * p_raw + t, R a rotation about Y only) is sent to SteamVR as each device's
 * qWorldFromDriverRotation / vecWorldFromDriverTranslation, the same value for the headset and both controllers,
 * so it is applied exactly once and the devices can't separate.
 *
 * A capture is a controller resting on the floor: its lowest point gives the floor height and its position and heading
 * give the anchor. The first capture defines the room frame (floor to y = 0, no turn or shift); later captures, with
 * the controller in the same place pointing the same way, recover T_room_raw for the new session.
 *
 * @ingroup st_ovrd
 */

#pragma once

#include "xrt/xrt_defines.h"

#include <string>


//! A position and a heading (rotation about +Y, radians, 0 = facing -Z, positive turns left).
struct ovrd_room_yaw_pose
{
	struct xrt_vec3 position;
	float yaw;
};

//! p_room = rotation * p_raw + translation, rotation about Y only.
struct ovrd_room_transform
{
	struct xrt_quat rotation;
	struct xrt_vec3 translation;
};

/*!
 * Which controller axis gives a capture's heading. A Sense controller put down usually rests on its ring with the
 * grip's forward axis pointing up (recorded rests: world up = (0, 0.27, -0.96) in the grip frame), so the heading
 * comes from whichever axis lies most level at the first capture, and later captures use the same one.
 */
enum ovrd_room_heading_axis
{
	OVRD_ROOM_AXIS_FORWARD = 0, //!< grip -Z
	OVRD_ROOM_AXIS_UP = 1,      //!< grip +Y
	OVRD_ROOM_AXIS_RIGHT = 2,   //!< grip +X
};

//! The saved room: where the anchor capture lies in the room frame.
struct ovrd_room_anchor
{
	struct ovrd_room_yaw_pose anchor; //!< In the room frame, position.y = 0 (on the floor).
	enum ovrd_room_heading_axis axis; //!< The controller axis the anchor's heading was measured with.
	int64_t created_unix_s;
	int64_t updated_unix_s;
	uint32_t captures;
};

//! Identity transform.
struct ovrd_room_transform
ovrd_room_transform_identity(void);

//! A rotation of @p yaw radians about +Y.
struct xrt_quat
ovrd_room_quat_from_yaw(float yaw);

/*!
 * Heading of an orientation: the yaw of its -Z axis projected on the floor. NAN if that axis is within ~30 degrees
 * of vertical (no reliable heading).
 */
float
ovrd_room_heading(const struct xrt_quat &orientation);

//! The same for another axis of the orientation (the yaw of where that axis points, projected on the floor).
float
ovrd_room_heading_of_axis(const struct xrt_quat &orientation, enum ovrd_room_heading_axis axis);

//! The axis of @p orientation that lies most level (for the first capture).
enum ovrd_room_heading_axis
ovrd_room_most_level_axis(const struct xrt_quat &orientation);

//! Transform a point.
struct xrt_vec3
ovrd_room_apply(const struct ovrd_room_transform &T, const struct xrt_vec3 &p);

//! Inverse transform.
struct ovrd_room_transform
ovrd_room_invert(const struct ovrd_room_transform &T);

//! a * b (apply b first).
struct ovrd_room_transform
ovrd_room_compose(const struct ovrd_room_transform &a, const struct ovrd_room_transform &b);

/*!
 * T_room_raw from a capture in the raw frame (position.y = floor height in raw) and a saved anchor. With no saved
 * anchor (@p saved == nullptr) the room frame is the raw frame lowered to the floor, and @p out_anchor gets the
 * capture expressed in it; with one, the capture is matched to it and @p out_anchor gets the saved anchor.
 */
struct ovrd_room_transform
ovrd_room_transform_from_capture(const struct ovrd_room_yaw_pose &capture_raw,
                                 const struct ovrd_room_yaw_pose *saved,
                                 struct ovrd_room_yaw_pose *out_anchor);

//! Lowest point (min y) of an outline (points in the grip frame) placed at @p grip in the raw frame.
float
ovrd_room_lowest_point(const struct xrt_pose &grip, const struct xrt_vec3 *outline, size_t count, float radius_m);


/*
 *
 * The capture gesture.
 *
 */

/*!
 * The floor capture: Create held while the controller lies still. Fed once per controller frame; separate from the
 * SteamVR glue so recorded runs can be replayed through it (psvr2-test/analysis/harness/capture_replay.cpp).
 */
class OvrdRoomCapture
{
public:
	//! Hold this long with the controller still.
	static constexpr int64_t HOLD_NS = 2500LL * 1000 * 1000;
	//! The controller may drift this much (m) during the hold, and turn this fast (rad/s).
	static constexpr float MAX_DRIFT_M = 0.01f;
	static constexpr float MAX_ANGULAR_SPEED = 0.2f;
	//! The floor must be at least this far below the head (m): the controller is on the floor, not a table or lap.
	static constexpr float MIN_BELOW_HEAD_M = 0.5f;
	//! No capture this soon after the headset's tracking re-originated (resets come in bursts while it settles).
	static constexpr int64_t AFTER_RESET_NS = 5000LL * 1000 * 1000;

	enum class Result
	{
		NONE,
		CAPTURED,
		REJECTED,
	};

	/*!
	 * One frame. @p grip is the grip pose in the raw frame, @p outline the controller outline in the grip frame,
	 * @p head the head's raw position (nullptr if unknown), @p last_reset_ns when the headset's tracking last
	 * re-originated (0 never), @p axis the saved room's heading axis or nullptr for a first capture. Messages for the
	 * log (restarts, cancels, rejections) are appended to @p out_log.
	 */
	Result
	feed(bool held,
	     const struct xrt_space_relation &grip,
	     const struct xrt_vec3 *outline,
	     size_t outline_count,
	     float outline_radius,
	     const struct xrt_vec3 *head,
	     int64_t last_reset_ns,
	     const enum ovrd_room_heading_axis *axis,
	     int64_t now_ns,
	     std::string *out_log);

	//! After CAPTURED: the capture in the raw frame (position.y = the floor) and the axis its heading used.
	struct ovrd_room_yaw_pose
	captured() const
	{
		return m_result;
	}
	enum ovrd_room_heading_axis
	captured_axis() const
	{
		return m_axis;
	}
	//! After CAPTURED: the floor's depth below the head (m).
	float
	captured_below_head() const
	{
		return m_below_head;
	}

private:
	void
	restart(const struct xrt_space_relation &grip, const enum ovrd_room_heading_axis *axis, int64_t now_ns);

	enum ovrd_room_heading_axis m_axis = OVRD_ROOM_AXIS_FORWARD;
	bool m_active = false;
	bool m_done = false;
	bool m_complained = false;
	int64_t m_start_ns = 0;
	struct xrt_vec3 m_start = {0, 0, 0};
	uint32_t m_samples = 0;
	uint32_t m_heading_samples = 0;
	double m_sum_lowest = 0, m_sum_x = 0, m_sum_z = 0, m_sum_sin = 0, m_sum_cos = 0;
	struct ovrd_room_yaw_pose m_result = {{0, 0, 0}, 0};
	float m_below_head = 0;
};


/*
 *
 * Files.
 *
 */

//! SteamVR's chaperone file, from ~/.config/openvr/openvrpaths.vrpath ("config"[0]). Empty if not found.
std::string
ovrd_room_chaperone_path(void);

/*!
 * Prepare SteamVR's chaperone file before SteamVR reads it (atomically, never touching other universes):
 *  - universe 3 (this session's unanchored frame) is reset to the plain origin with a @p session_size square play area
 *    (no bounds if 0), floor 1.6 m below the head at power on; anything saved in it last session is meaningless now.
 *  - universe 2 (the room) gets the plain origin only if it has no entry yet, so Room Setup has a calibrated frame
 *    to draw in (without one SteamVR logs "Failed to update shared data" and Room Setup shows the devices as Off).
 * @p out_has_room is set when universe 2 already has a drawn room (a play area or collision bounds).
 */
/*!
 * @p room_size > 0: the room (universe 2) gets a square play area of that size from the driver, as ALVR gives SteamVR
 * a rectangle instead of using Room Setup, unless a room was drawn there in Room Setup (a drawn room is never
 * replaced). The driver's rectangle is marked ("monado_rectangle") so a new size replaces it.
 */
bool
ovrd_room_prepare_chaperone(const std::string &path,
                            float session_size,
                            bool *out_has_room,
                            std::string *out_error,
                            float room_size = 0.0f);

//! Load the saved room, false if missing or invalid (version, finite numbers).
bool
ovrd_room_load(const std::string &path, struct ovrd_room_anchor *out);

//! Save the room atomically (temporary file, fsync, rename).
bool
ovrd_room_save(const std::string &path, const struct ovrd_room_anchor &room);
