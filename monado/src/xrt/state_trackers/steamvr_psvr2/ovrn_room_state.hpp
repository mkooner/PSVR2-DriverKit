// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The room calibration's state (OvrdRoom), shared by the SteamVR plugin (ovrd_driver.cpp) and the native
 *         PS VR2 SteamVR driver (st_ovrn): the room transform every device's pose carries, captures, universes.
 * @ingroup st_ovrn
 */

#pragma once

#include "ovrn_room_log.hpp"
#include "ovrn_room.hpp"

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
#include <sys/stat.h>
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
	//! The old plugin's universes, in SteamVR's own chaperone file (the PS VR2 driver passes its own to init()).
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
		return m_enabled ? m_published_universe : m_room_universe;
	}

	/*!
	 * Prepare the chaperone file and load the saved room. Call before any device is added.
	 *
	 * @p chaperone_path: a driver-owned chaperone file (published as Prop_DriverProvidedChaperonePath_String by the
	 * caller); empty: SteamVR's own file (the old plugin). @p room_universe / @p session_universe: the universe ids
	 * (the saved room, and this session's frame until a capture anchors the room). @p room_size > 0 gives the saved
	 * room a driver-made square when none was drawn (the old plugin's ALVR-style option; 0: never, a room comes only
	 * from Room Setup).
	 */
	void
	init(float session_size,
	     float room_size = 0.0f,
	     const std::string &chaperone_path = std::string(),
	     uint64_t room_universe = UNIVERSE_ROOM,
	     uint64_t session_universe = UNIVERSE_SESSION)
	{
		m_room_universe = room_universe;
		m_session_universe = session_universe;
		m_session_size = session_size;
		m_published_universe = session_universe;
		m_enabled = debug_get_bool_option_ovrd_room();
		if (!m_enabled) {
			ovrd_log("Room: disabled (OVRD_ROOM=0), providing a fixed play area instead\n");
			return;
		}

		std::string chaperone = chaperone_path.empty() ? ovrd_room_chaperone_path() : chaperone_path;
		m_chaperone_path = chaperone;
		bool has_drawn_room = false;
		std::string error;
		const std::string room_id = std::to_string(room_universe), session_id = std::to_string(session_universe);
		if (chaperone.empty() || !ovrd_room_prepare_chaperone(chaperone, session_size, &has_drawn_room, &error,
		                                                      room_size, room_id.c_str(), session_id.c_str())) {
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
		ovrd_log("Room: chaperone file %s prepared (universe %llu for this session, universe %llu %s)\n",
		         chaperone.c_str(), (unsigned long long)session_universe, (unsigned long long)room_universe,
		         has_drawn_room ? "holds a room drawn in Room Setup" : rect);
		m_chaperone_mtime = file_mtime(chaperone);
		log_universe("at start", m_room_universe);
		if (m_have_room) {
			ovrd_log("Room: saved room %s (anchor %.3f %.3f yaw %.1f deg, %u captures). Put the left controller "
			         "back in its spot, the same way round, crouch next to it and hold Create to put the room "
			         "back; until then this session uses universe %llu\n",
			         m_room_path.c_str(), m_room.anchor.position.x, m_room.anchor.position.z,
			         m_room.anchor.yaw * 57.29578f, m_room.captures, (unsigned long long)session_universe);
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
			// The room is kept: the headset driver carries the head position across the reset, so the room stays
			// where it was give or take the carry's error (a few cm). Dropping it would make the boundary jump or
			// vanish, which is worse; a new capture on the mark refines it.
			ovrd_log("Room: the headset's tracking world was replaced (count %u); %s\n", resets,
			         m_anchored ? "the room is kept, but may be off by a few cm until the next capture"
			                    : "no room anchored yet");
		}

		if (jumped) {
			ovrd_log("Room: the head moved %.2f m in one step (tracking jump)%s\n", jump_m,
			         m_anchored ? "; the room is kept: capture again on the mark if the boundary looks shifted" : "");
		}

		// SteamVR wrote the chaperone file (Room Setup's commit, a floor calibration, a recentre): what it holds now.
		if (now - m_last_file_check_ns > U_TIME_1S_IN_NS) {
			m_last_file_check_ns = now;
			const int64_t mtime = file_mtime(m_chaperone_path);
			if (mtime != m_chaperone_mtime) {
				m_chaperone_mtime = mtime;
				log_universe("SteamVR changed the chaperone file", m_published_universe);
				note_session_room(); // also when the commit event came before SteamVR's write
			}
		}
		if (now - m_last_state_log_ns > 30 * U_TIME_1S_IN_NS) {
			m_last_state_log_ns = now;
			log_state();
		}

		if (!m_command_path.empty() && now - m_last_command_poll_ns > 500 * U_TIME_1MS_IN_NS) {
			m_last_command_poll_ns = now;
			poll_command();
		}

		uint64_t wanted = m_anchored ? m_room_universe : m_session_universe;
		if (wanted != m_published_universe) {
			m_published_universe = wanted;
			for (size_t i = 0; i < m_container_count; i++) {
				vr::VRProperties()->SetUint64Property(m_containers[i], vr::Prop_CurrentUniverseId_Uint64, wanted);
			}
			ovrd_log("Room: SteamVR universe %llu (%s)\n", (unsigned long long)wanted,
			         wanted == m_room_universe ? "the saved room" : "this session only, no saved room");
			log_universe("now active", wanted);
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
		                   m_last_reset_ns, matches_saved_room() ? &m_room.axis : nullptr, now, &messages);
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

	/*!
	 * SteamVR's Room Setup started (VREvent_ChaperoneRoomSetupStarting, RunFrame thread). True when the room isn't
	 * anchored yet: a room drawn now would be in this session's frame only (gone at the next start), so the caller
	 * prompts for the floor capture (a haptic pattern on the left controller) and this logs how.
	 */
	bool
	room_setup_started()
	{
		if (!m_enabled) {
			return false;
		}
		if (m_anchored) {
			ovrd_log("Room: Room Setup started; it draws in the saved room (universe %llu), kept across sessions\n",
			         (unsigned long long)m_room_universe);
			return false;
		}
		ovrd_log("Room: Room Setup started before the room was anchored: the room drawn is kept for this session. To "
		         "keep it for good, anchor it afterwards: put the left controller on a floor spot you can find again "
		         "(your floor mark), crouch next to it and hold Create for %.1f s (short buzz); the room then moves "
		         "into the saved room\n",
		         OvrdRoomCapture::HOLD_NS / 1e9);
		return true;
	}

	//! SteamVR's Room Setup committed a room (VREvent_ChaperoneRoomSetupCommitted, RunFrame thread).
	void
	room_setup_committed()
	{
		if (m_enabled) {
			log_universe("Room Setup committed", m_published_universe);
			note_session_room();
		}
	}

	//! The chaperone file in use ("" when the room code is off).
	const std::string &
	chaperone_path() const
	{
		return m_chaperone_path;
	}

	bool
	anchored() const
	{
		return m_anchored;
	}

private:
	static int64_t
	file_mtime(const std::string &path)
	{
		struct stat st;
		if (path.empty() || stat(path.c_str(), &st) != 0) {
			return 0;
		}
		return (int64_t)st.st_mtim.tv_sec * 1000000000LL + st.st_mtim.tv_nsec;
	}

	/*!
	 * A room drawn in this session's universe before anchoring (2026-11-19: three rooms drawn that way, all lost at
	 * the next start). The next floor capture then starts a new room frame and moves it into the saved room.
	 */
	void
	note_session_room()
	{
		if (m_anchored || m_published_universe != m_session_universe) {
			return;
		}
		const std::string sid = std::to_string(m_session_universe);
		if (ovrd_room_universe_drawn(m_chaperone_path, sid.c_str(), m_session_size)) {
			if (!m_session_room_drawn) {
				ovrd_log("Room: a room was drawn in this session's universe %llu; anchor it (floor capture) to keep "
				         "it: it then replaces the saved room\n",
				         (unsigned long long)m_session_universe);
			}
			m_session_room_drawn = true;
		}
	}

	//! The capture is matched to the saved room (its anchor and axis) unless a new room was drawn this session.
	bool
	matches_saved_room() const
	{
		return m_have_room && !m_session_room_drawn;
	}

	//! What universe @p id of the chaperone file holds, and anything wrong with it.
	void
	log_universe(const char *why, uint64_t id)
	{
		ovrd_room_universe_info info;
		const std::string sid = std::to_string(id);
		if (!ovrd_room_read_universe(m_chaperone_path, sid.c_str(), &info) || !info.found) {
			ovrd_log("Room: %s: universe %llu not in %s\n", why, (unsigned long long)id, m_chaperone_path.c_str());
			return;
		}
		ovrd_log("Room: %s: universe %llu: play area %.2f x %.2f m, %zu walls (perimeter %.2f m, floor area %.2f m2, "
		         "%s), standing at %.3f %.3f %.3f yaw %.1f deg%s, written %s%s\n",
		         why, (unsigned long long)id, info.play_width, info.play_depth, info.walls, info.perimeter_m,
		         info.area_m2, info.winding > 0 ? "counter-clockwise" : info.winding < 0 ? "clockwise" : "open",
		         info.standing.position.x, info.standing.position.y, info.standing.position.z,
		         info.standing.yaw * 57.29578f, info.has_setup_standing ? ", own setup-standing origin" : "",
		         info.time.empty() ? "?" : info.time.c_str(), info.issues.empty() ? ", geometry valid" : "");
		for (const std::string &issue : info.issues) {
			ovrd_log("Room:   problem: %s\n", issue.c_str());
		}
	}

	//! The room's state (every 30 s): universe, anchor, transform, the head in SteamVR's world.
	void
	log_state()
	{
		ovrd_room_transform T;
		struct xrt_vec3 head;
		bool head_valid;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			T = m_transform;
			head = m_head;
			head_valid = m_head_valid;
		}
		const struct xrt_vec3 world = ovrd_room_apply(T, head);
		ovrd_log("Room: state: universe %llu (%s), room transform turn %.1f deg shift %.3f %.3f %.3f, head raw %.3f "
		         "%.3f %.3f -> world %.3f %.3f %.3f%s\n",
		         (unsigned long long)m_published_universe, m_anchored ? "anchored room" : "session only",
		         2.0f * atan2f(T.rotation.y, T.rotation.w) * 57.29578f, T.translation.x, T.translation.y,
		         T.translation.z, head.x, head.y, head.z, world.x, world.y, world.z, head_valid ? "" : " (no head pose)");
	}

	CaptureResult
	finish_capture()
	{
		ovrd_room_yaw_pose capture = m_capture.captured();
		float floor_y = capture.position.y;
		float heading = capture.yaw;
		float below_head = m_capture.captured_below_head();
		ovrd_room_yaw_pose anchor;
		const bool fresh = !matches_saved_room();
		ovrd_room_transform T =
		    ovrd_room_transform_from_capture(capture, fresh ? nullptr : &m_room.anchor, &anchor);

		/*
		 * A room drawn this session: the new room frame is the raw frame with the floor moved (no turn), so the
		 * drawn room moves into the saved universe with its origins shifted the same way, before SteamVR is
		 * switched to it (it reads the universe from the file then).
		 */
		bool moved_room = false;
		if (m_session_room_drawn) {
			std::string note;
			const std::string from = std::to_string(m_session_universe), to = std::to_string(m_room_universe);
			moved_room = ovrd_room_move_universe(m_chaperone_path, from.c_str(), to.c_str(), T.translation,
			                                     m_session_size, &note);
			ovrd_log("Room: the room drawn this session %s the saved room (universe %llu): %s\n",
			         moved_room ? "now is" : "could NOT become", (unsigned long long)m_room_universe, note.c_str());
			if (moved_room) {
				log_universe("saved room after the move", m_room_universe);
			}
			m_session_room_drawn = false;
		}

		int64_t now_s = (int64_t)time(nullptr);
		ovrd_room_anchor room = m_room;
		if (fresh) {
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
		         !fresh ? "matched to the saved room" : moved_room ? "new room frame for the room drawn this session" : "new room frame",
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
	uint64_t m_room_universe = UNIVERSE_ROOM;
	uint64_t m_session_universe = UNIVERSE_SESSION;
	std::string m_chaperone_path;
	int64_t m_chaperone_mtime = 0;
	timepoint_ns m_last_file_check_ns = 0;
	timepoint_ns m_last_state_log_ns = 0;
	struct xrt_device *m_xhmd = nullptr;
	uint32_t m_slam_resets = 0;
	timepoint_ns m_last_reset_ns = 0;
	std::string m_room_path;
	std::string m_command_path;
	bool m_have_room = false;
	ovrd_room_anchor m_room = {};
	bool m_anchored = false;
	bool m_session_room_drawn = false;
	float m_session_size = 0.0f; //!< the default square the session universe starts with
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
