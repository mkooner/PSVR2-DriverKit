// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The room calibration's state (OvrdRoom), shared by the SteamVR plugin (ovrd_driver.cpp) and the native
 *         PS VR2 SteamVR driver (st_ovrn): the room transform every device's pose carries, captures, universes.
 * @ingroup st_ovrd
 */

#pragma once

#include "ovrd_log.hpp"
#include "ovrd_room.hpp"

#include "math/m_api.h"
#include "math/m_vec3.h"
#include "util/u_debug.h"
#include "util/u_file.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_device.h"

#ifdef XRT_BUILD_DRIVER_PSVR2
#include "util/u_json.h" // psvr2_interface.h uses cJSON
#include "psvr2/psvr2_interface.h"
#endif

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <mutex>
#include <string>
#include <unistd.h>

/*!
 * Room calibration (see ovrd_room.hpp): keep SteamVR's room in place across sessions by sending one room transform
 * with every pose, and use SteamVR's universe 3 until a floor capture anchors the room (universe 2). Set to false
 * for the previous behaviour (a driver-provided play area in universe 2, nothing saved).
 */
DEBUG_GET_ONCE_BOOL_OPTION(ovrd_room, "OVRD_ROOM", true)

/*
 *
 * Room calibration.
 *
 */

/*!
 * Room state shared by the headset and the controllers.
 *
 * Threads: the pose threads (headset 1 kHz, each controller 1 kHz) call apply() and note_head(); SteamVR's RunFrame
 * thread calls everything else, including the file writes. The transform is a small struct copied under a mutex.
 */
class OvrdRoom
{
public:
	static constexpr uint64_t UNIVERSE_SESSION = 3;
	static constexpr uint64_t UNIVERSE_ROOM = 2;
	//! A head step this large within WORLD_JUMP_WINDOW_NS means the headset's tracking world was replaced.
	static constexpr float WORLD_JUMP_M = 0.3f;
	static constexpr timepoint_ns WORLD_JUMP_WINDOW_NS = 50 * U_TIME_1MS_IN_NS;

	enum class CaptureResult
	{
		NONE,
		CAPTURED,
		REJECTED,
	};

	bool
	enabled() const
	{
		return m_enabled;
	}

	//! The universe to report: universe 2 when disabled (the previous behaviour).
	uint64_t
	universe() const
	{
		return m_enabled ? m_published_universe : UNIVERSE_ROOM;
	}

	//! Prepare SteamVR's chaperone file and load the saved room. Call before any device is added.
	void
	init(float session_size, float room_size = 0.0f)
	{
		m_enabled = debug_get_bool_option_ovrd_room();
		if (!m_enabled) {
			ovrd_log("Room: disabled (OVRD_ROOM=0), providing a fixed play area instead\n");
			return;
		}

		std::string chaperone = ovrd_room_chaperone_path();
		bool has_drawn_room = false;
		std::string error;
		if (chaperone.empty() ||
		    !ovrd_room_prepare_chaperone(chaperone, session_size, &has_drawn_room, &error, room_size)) {
			ovrd_log("Room: can't prepare SteamVR's chaperone file '%s' (%s), providing a fixed play area instead\n",
			         chaperone.c_str(), chaperone.empty() ? "openvrpaths.vrpath has no config path" : error.c_str());
			m_enabled = false;
			return;
		}

		char path[1024];
		u_file_get_path_in_config_dir("ovrd_room.json", path, sizeof(path));
		m_room_path = path;
		m_have_room = ovrd_room_load(m_room_path, &m_room);

		const char *runtime_dir = getenv("XDG_RUNTIME_DIR");
		if (runtime_dir != nullptr) {
			m_command_path = std::string(runtime_dir) + "/monado-room-command";
		}

		char rect[96] = "has no room drawn yet";
		if (room_size > 0.0f) {
			snprintf(rect, sizeof(rect), "gets a %.1f m square play area from the driver", room_size);
		}
		ovrd_log("Room: chaperone file %s prepared (universe 3 for this session, universe 2 %s)\n", chaperone.c_str(),
		         has_drawn_room ? "holds a room drawn in Room Setup" : rect);
		if (m_have_room) {
			ovrd_log("Room: saved room %s (anchor %.3f %.3f yaw %.1f deg, %u captures). Put the left controller "
			         "back in its spot, the same way round, crouch next to it and hold Create to put the room "
			         "back; until then this session uses universe 3\n",
			         m_room_path.c_str(), m_room.anchor.position.x, m_room.anchor.position.z,
			         m_room.anchor.yaw * 57.29578f, m_room.captures);
		} else {
			ovrd_log("Room: no saved room (%s). Put the left controller on the floor, crouch next to it so the "
			         "headset sees it (within ~0.6 m), and hold Create for %.1f s to measure the floor and anchor "
			         "the room\n",
			         m_room_path.c_str(), OvrdRoomCapture::HOLD_NS / 1e9);
		}
		if (!m_command_path.empty()) {
			ovrd_log("Room: 'echo forget > %s' deletes the saved room\n", m_command_path.c_str());
		}
	}

	//! The headset, to watch for its tracking world being replaced (PS VR2: SLAM resets).
	void
	set_hmd(struct xrt_device *xhmd)
	{
		m_xhmd = xhmd;
	}

	//! Remember a device's property container so universe changes reach it.
	void
	add_container(vr::PropertyContainerHandle_t container)
	{
		if (container != vr::k_ulInvalidPropertyContainer && m_container_count < ARRAY_SIZE(m_containers)) {
			m_containers[m_container_count++] = container;
		}
	}

	//! The current room transform (SteamVR's world from our driver space).
	ovrd_room_transform
	transform()
	{
		if (!m_enabled) {
			return ovrd_room_transform_identity();
		}
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_transform;
	}

	//! Put the room transform in a pose (all devices get the same one).
	void
	apply(vr::DriverPose_t &pose)
	{
		ovrd_room_transform T = ovrd_room_transform_identity();
		if (m_enabled) {
			std::lock_guard<std::mutex> lock(m_mutex);
			T = m_transform;
		}
		pose.qWorldFromDriverRotation = vr::HmdQuaternion_t{T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z};
		pose.vecWorldFromDriverTranslation[0] = T.translation.x;
		pose.vecWorldFromDriverTranslation[1] = T.translation.y;
		pose.vecWorldFromDriverTranslation[2] = T.translation.z;
	}

	//! The headset's raw position (pose thread): for the capture's sanity check and world-jump detection.
	void
	note_head(const struct xrt_vec3 &position, timepoint_ns now)
	{
		if (!m_enabled) {
			return;
		}
		std::lock_guard<std::mutex> lock(m_mutex);
		if (m_head_valid && now - m_head_ns < WORLD_JUMP_WINDOW_NS &&
		    m_vec3_len(position - m_head) > WORLD_JUMP_M) {
			m_world_jumped = true;
			m_jump_m = m_vec3_len(position - m_head);
		}
		m_head = position;
		m_head_ns = now;
		m_head_valid = true;
	}

	//! SteamVR's RunFrame thread: universe changes, world jumps, commands.
	void
	run_frame(timepoint_ns now)
	{
		if (!m_enabled) {
			return;
		}

		bool jumped = false;
		float jump_m = 0;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			jumped = m_world_jumped;
			jump_m = m_jump_m;
			m_world_jumped = false;
		}
		uint32_t resets = slam_reset_count();
		if (resets != m_slam_resets) {
			m_slam_resets = resets;
			m_last_reset_ns = now;
			if (m_anchored) {
				ovrd_log("Room: the headset's tracking re-originated (SLAM reset %u), so the room can't be "
				         "trusted; back to universe 3 until the next capture\n",
				         resets);
				unanchor();
			}
		}

		if (jumped && m_anchored) {
			ovrd_log("Room: the head moved %.2f m in one step: the headset's tracking was reset, so the room can't "
			         "be trusted; back to universe 3 until the next capture\n",
			         jump_m);
			unanchor();
		}

		if (!m_command_path.empty() && now - m_last_command_poll_ns > 500 * U_TIME_1MS_IN_NS) {
			m_last_command_poll_ns = now;
			poll_command();
		}

		uint64_t wanted = m_anchored ? UNIVERSE_ROOM : UNIVERSE_SESSION;
		if (wanted != m_published_universe) {
			m_published_universe = wanted;
			for (size_t i = 0; i < m_container_count; i++) {
				vr::VRProperties()->SetUint64Property(m_containers[i], vr::Prop_CurrentUniverseId_Uint64, wanted);
			}
			ovrd_log("Room: SteamVR universe %llu (%s)\n", (unsigned long long)wanted,
			         wanted == UNIVERSE_ROOM ? "the saved room" : "this session only, no saved room");
		}
	}

	/*!
	 * A controller frame (RunFrame thread): the capture gesture. @p grip is the grip pose in the raw frame,
	 * @p outline its outline in the grip frame.
	 */
	CaptureResult
	controller_frame(bool create_held,
	                 const struct xrt_space_relation &grip,
	                 const struct xrt_vec3 *outline,
	                 size_t outline_count,
	                 float outline_radius,
	                 timepoint_ns now)
	{
		if (!m_enabled || outline_count == 0) {
			return CaptureResult::NONE;
		}
		struct xrt_vec3 head = {0, 0, 0};
		bool head_valid = false;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			head = m_head;
			head_valid = m_head_valid;
		}
		std::string messages;
		OvrdRoomCapture::Result result =
		    m_capture.feed(create_held, grip, outline, outline_count, outline_radius, head_valid ? &head : nullptr,
		                   m_last_reset_ns, m_have_room ? &m_room.axis : nullptr, now, &messages);
		size_t pos = 0;
		while (pos < messages.size()) {
			size_t nl = messages.find('\n', pos);
			ovrd_log("Room: %s\n", messages.substr(pos, nl - pos).c_str());
			pos = nl == std::string::npos ? messages.size() : nl + 1;
		}
		if (result == OvrdRoomCapture::Result::REJECTED) {
			return CaptureResult::REJECTED;
		}
		if (result != OvrdRoomCapture::Result::CAPTURED) {
			return CaptureResult::NONE;
		}
		return finish_capture();
	}

private:
	CaptureResult
	finish_capture()
	{
		ovrd_room_yaw_pose capture = m_capture.captured();
		float floor_y = capture.position.y;
		float heading = capture.yaw;
		float below_head = m_capture.captured_below_head();
		ovrd_room_yaw_pose anchor;
		ovrd_room_transform T =
		    ovrd_room_transform_from_capture(capture, m_have_room ? &m_room.anchor : nullptr, &anchor);

		int64_t now_s = (int64_t)time(nullptr);
		ovrd_room_anchor room = m_room;
		if (!m_have_room) {
			room.created_unix_s = now_s;
			room.captures = 0;
		}
		room.anchor = anchor;
		room.axis = m_capture.captured_axis();
		room.updated_unix_s = now_s;
		room.captures++;
		bool saved = ovrd_room_save(m_room_path, room);

		ovrd_log("Room: captured: floor %.3f m below the head (raw floor y %.3f), controller at %.3f %.3f heading "
		         "%.1f deg; %s; room transform turn %.1f deg, shift %.3f %.3f %.3f%s\n",
		         below_head, floor_y, capture.position.x, capture.position.z, heading * 57.29578f,
		         m_have_room ? "matched to the saved room" : "new room frame",
		         2.0f * atan2f(T.rotation.y, T.rotation.w) * 57.29578f, T.translation.x, T.translation.y,
		         T.translation.z, saved ? "" : " (saving the room file FAILED, it won't persist)");

		m_room = room;
		m_have_room = m_have_room || saved;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_transform = T;
		}
		m_anchored = true;
		return CaptureResult::CAPTURED;
	}

	uint32_t
	slam_reset_count() const
	{
#ifdef XRT_BUILD_DRIVER_PSVR2
		if (m_xhmd != nullptr && m_xhmd->name == XRT_DEVICE_PSVR2) {
			return psvr2_get_slam_reset_count(m_xhmd);
		}
#endif
		return 0;
	}

	void
	unanchor()
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_transform = ovrd_room_transform_identity();
		m_anchored = false;
	}

	void
	poll_command()
	{
		FILE *file = fopen(m_command_path.c_str(), "r");
		if (file == nullptr) {
			return;
		}
		char command[64] = "";
		if (fgets(command, sizeof(command), file) == nullptr) {
			command[0] = '\0';
		}
		fclose(file);
		unlink(m_command_path.c_str());

		if (strncmp(command, "forget", 6) == 0) {
			if (unlink(m_room_path.c_str()) == 0 || !m_have_room) {
				ovrd_log("Room: forgot the saved room (SteamVR's universe 2 boundary stays in its file; draw a new "
				         "one after the next capture)\n");
			}
			m_have_room = false;
			m_room = ovrd_room_anchor{};
			unanchor();
		} else if (strncmp(command, "anchor-test", 11) == 0) {
			// Debug only: switch to universe 2 with this session's frame as is (nothing measured or saved), to
			// check SteamVR's handling of the universe change without a controller capture.
			ovrd_log("Room: anchor-test: universe 2 with the session frame (debug, nothing saved)\n");
			std::lock_guard<std::mutex> lock(m_mutex);
			m_transform = ovrd_room_transform_identity();
			m_anchored = true;
		} else {
			ovrd_log("Room: unknown command '%s' (known: forget, anchor-test)\n", command);
		}
	}

	bool m_enabled = false;
	struct xrt_device *m_xhmd = nullptr;
	uint32_t m_slam_resets = 0;
	timepoint_ns m_last_reset_ns = 0;
	std::string m_room_path;
	std::string m_command_path;
	bool m_have_room = false;
	ovrd_room_anchor m_room = {};
	bool m_anchored = false;
	uint64_t m_published_universe = UNIVERSE_SESSION;
	vr::PropertyContainerHandle_t m_containers[8] = {};
	size_t m_container_count = 0;
	timepoint_ns m_last_command_poll_ns = 0;
	OvrdRoomCapture m_capture;

	std::mutex m_mutex; // guards the fields below
	ovrd_room_transform m_transform = ovrd_room_transform_identity();
	struct xrt_vec3 m_head = {0, 0, 0};
	timepoint_ns m_head_ns = 0;
	bool m_head_valid = false;
	bool m_world_jumped = false;
	float m_jump_m = 0;
};
