// Copyright 2020,2024 Collabora, Ltd.
// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Main driver code for @ref st_ovrd.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Christoph Haag <christoph.haag@collabora.com>
 * @author Daniel Willmott <web@dan-w.com>
 * @author Moshi Turner <moshiturner@protonmail.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @ingroup st_ovrd
 */

#include <cstring>
#include <cassert>
#include <array>
#include <thread>

#include "math/m_api.h"
#include "ovrd_log.hpp"
#include "openvr_driver.h"
#include "ovrd_hidden_area.hpp"
#include "ovrd_pose_log.hpp"
#include "ovrd_display_clock.hpp"
#include "ovrd_pose_replay.hpp"
#include "ovrd_direct_mode.hpp"

extern "C" {
#include "ovrd_interface.h"

#include <math.h>

#include <math/m_space.h>
#include "os/os_time.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_hand_tracking.h"
#include "util/u_builder_helpers.h"

#include "xrt/xrt_space.h"
#include "xrt/xrt_system.h"
#include "xrt/xrt_session.h"
#include "xrt/xrt_compositor.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_instance.h"

#include "b_ovrd_generated_bindings.h"
}
#include "math/m_vec3.h"
#include "util/u_file.h"
#include "util/u_time.h"
#include "xrt/xrt_config_drivers.h"

#include "ovrd_room.hpp"

#ifdef XRT_BUILD_DRIVER_PSSENSE
#include "pssense/pssense_interface.h"
#endif
#ifdef XRT_BUILD_DRIVER_PSVR2
#include "psvr2/psvr2_interface.h"
#endif

#include <atomic>
#include <mutex>
#include <string>
#include <unistd.h>

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#endif

//! When set, all controllers pretend to be Index controllers. Provides best
//! compatibility with legacy games due to steamvr's legacy binding for Index
//! controllers, but input mapping may be incomplete or not ideal.
DEBUG_GET_ONCE_BOOL_OPTION(emulate_index_controller, "STEAMVR_EMULATE_INDEX_CONTROLLER", false)

/*!
 * Per-eye render size for SteamVR in percent of the eye's display size. 0 (unset): 170 for the PS VR2, as Sony's driver
 * recommends (3400x3468 per eye = 2000x2040 x 1.7, hmdq/HMD Geometry Database entry and its vrcompositor.txt), 140 for
 * other devices (Monado's default).
 */
DEBUG_GET_ONCE_NUM_OPTION(scale_percentage, "XRT_COMPOSITOR_SCALE_PERCENTAGE", 0)

/*!
 * Play area handed to SteamVR for our tracking universe (side length in meters, 0 disables).
 *
 * Inside-out headsets like the PS VR2 have no base stations, and without a chaperone for the driver's universe
 * SteamVR sends the user through base station room setup, which can't complete. Providing one skips that.
 */
DEBUG_GET_ONCE_FLOAT_OPTION(ovrd_play_area_size, "OVRD_PLAY_AREA_SIZE", 2.0f)

/*!
 * Set when Sony's PS VR2 Sense render models are available in this driver's resources (the test launcher links them
 * in from the PlayStation VR2 App install). Otherwise SteamVR's built-in Oculus Touch models stand in.
 */
DEBUG_GET_ONCE_BOOL_OPTION(ovrd_pssense_render_models, "OVRD_PSSENSE_RENDER_MODELS", false)
//! Sony's PS VR2 icons are linked at resources/icons/playstation_vr2 (by psvr2-test/launch-steamvr.sh).
DEBUG_GET_ONCE_BOOL_OPTION(ovrd_psvr2_sony_icons, "OVRD_PSVR2_SONY_ICONS", false)

/*!
 * Room calibration (see ovrd_room.hpp): keep SteamVR's room in place across sessions by sending one room transform
 * with every pose, and use SteamVR's universe 3 until a floor capture anchors the room (universe 2). Set to false
 * for the previous behaviour (a driver-provided play area in universe 2, nothing saved).
 */
DEBUG_GET_ONCE_BOOL_OPTION(ovrd_room, "OVRD_ROOM", true)

/*!
 * Seconds from vsync to photons reported to SteamVR (Prop_SecondsFromVsyncToPhotons_Float), which sets how far ahead
 * SteamVR extrapolates the head pose: the next vsync plus this. Negative: the default for the headset, see below.
 */
DEBUG_GET_ONCE_FLOAT_OPTION(ovrd_vsync_to_photons_s, "OVRD_VSYNC_TO_PHOTONS_S", -1.0f)

/*!
 * Seconds from photons to the next vblank reported to SteamVR (Prop_SecondsFromPhotonsToVblank_Float), part of how
 * SteamVR times frames against the display. Negative: the default for the headset, see below; 0: not reported.
 */
DEBUG_GET_ONCE_FLOAT_OPTION(ovrd_photons_to_vblank_s, "OVRD_PHOTONS_TO_VBLANK_S", -1.0f)

/*!
 * Give SteamVR a hidden-area mesh per eye (the parts of the render target the lens distortion never samples), so apps
 * skip shading them; Sony's driver hides 22.3% (vrcompositor.txt "HiddenArea(22.30%)"). Off by default: our
 * distortion samples the whole render target (psvr2-test/analysis/harness/hidden_area_check: 0.0% hidden), the lens's
 * visible edge on the panel isn't modelled. OVRD_HIDDEN_AREA=1: on.
 */
DEBUG_GET_ONCE_BOOL_OPTION(ovrd_hidden_area, "OVRD_HIDDEN_AREA", false)
/*!
 * A hidden-area mask to hand SteamVR as it is: "ham_vert <eye> <type> <u> <v>" lines (tools/vrdisplaydump output, taken
 * with Sony's driver running, gives Sony's mask: 22.3% of a 3400x3468 eye). Default ~/.config/monado/psvr2-hidden-area.txt
 * when that exists. Takes precedence over OVRD_HIDDEN_AREA.
 */
DEBUG_GET_ONCE_OPTION(ovrd_hidden_area_file, "OVRD_HIDDEN_AREA_FILE", NULL)

/*!
 * Resolution of the distortion mesh SteamVR builds from ComputeDistortion (Prop_DistortionMeshResolution_Int32).
 * SteamVR's default is 49x49, which bends the PS VR2 picture by up to ~2.7 px in the centre and ~16 px in the middle of
 * the view between mesh points (psvr2-test/analysis/harness/mesh_res_check); turning the head slides the scene through
 * those cells, so it wobbles. Sony's driver sets 80 (its vrcompositor.txt: "mesh resolution 80"); Monado's own
 * compositor uses 64 (XRT_MESH_SIZE). Built once at startup, no per-frame cost. 0: SteamVR's default.
 */
DEBUG_GET_ONCE_NUM_OPTION(ovrd_distortion_mesh_res, "OVRD_DISTORTION_MESH_RES", 80)

/*!
 * How the headset pose is handed to SteamVR (OVRD_POSE_MODE), one variable at a time for diagnosis:
 * - "steamvr" (default, A): the pose for now, poseTimeOffset 0, with velocities; SteamVR predicts.
 * - "measured" (B): no driver-side prediction: the pose at now - OVRD_POSE_AGE_MS (about when the newest IMU sample was
 *   taken) with poseTimeOffset = -age and velocities, as Valve's lighthouse driver reports its poses; SteamVR predicts.
 * - "novelocity" (C): the pose for now without velocities: SteamVR shows it as it is (SteamVR-OpenHMD's model).
 * - "display": the pose predicted to the time its frame is shown, without velocities (ALVR's model,
 *   ovrd_display_clock.hpp). Tried 2026-11-17 22:44: frames still a refresh off (10.3% early), so not the fix.
 * OVRD_POSE_REPLAY=pose-log.csv plays a recorded OVRD_POSE_LOG instead of the headset (D), any mode, same input.
 */
DEBUG_GET_ONCE_OPTION(ovrd_pose_mode, "OVRD_POSE_MODE", "steamvr")
DEBUG_GET_ONCE_FLOAT_OPTION(ovrd_pose_age_ms, "OVRD_POSE_AGE_MS", 2.0f)
/*!
 * OVRD_ANGVEL_AVG_MS > 0: send SteamVR the mean of the last N ms of angular velocity instead of the newest gyro sample
 * (0: off, the default). Offline, on three live pose logs (2026-11-18), averaging 3-10 ms raised SteamVR's 22 ms
 * prediction error (median 0.048 -> 0.053-0.065 deg) and left the shown path's jitter the same; kept as a switch.
 */
DEBUG_GET_ONCE_NUM_OPTION(ovrd_angvel_avg_ms, "OVRD_ANGVEL_AVG_MS", 0)
DEBUG_GET_ONCE_OPTION(ovrd_pose_replay, "OVRD_POSE_REPLAY", NULL)
/*!
 * The refresh a pose submitted now is for: the first vsync more than this far ahead. SteamVR's frame timings carry
 * their display vsync, and the app asks for its pose ~3 ms before the refresh before it (Valve's running start), so its
 * frame shows at the vsync 3-14 ms after it asks; 8.6 ms is the middle, away from both edges.
 */
DEBUG_GET_ONCE_FLOAT_OPTION(ovrd_display_cut_ms, "OVRD_DISPLAY_CUT_MS", 8.6f)
//! When a refresh's light reaches the eyes after its vsync: vsync-to-photons + half the scanout (1.36 + 10.3 / 2).
DEBUG_GET_ONCE_FLOAT_OPTION(ovrd_display_after_vsync_ms, "OVRD_DISPLAY_AFTER_VSYNC_MS", 6.5f)

//! OVRD_POSE_LOG=path: record every headset pose handed to SteamVR (ovrd_pose_log.hpp). Unset: nothing recorded.
DEBUG_GET_ONCE_OPTION(ovrd_pose_log, "OVRD_POSE_LOG", NULL)

/*!
 * Driver direct mode prototype (ovrd_direct_mode.hpp): SteamVR hands its frames to the plugin instead of driving the
 * display. Nothing is shown in the headset yet, the frames are only logged (OVRD_DIRECT_MODE_LOG).
 */
DEBUG_GET_ONCE_BOOL_OPTION(ovrd_direct_mode, "OVRD_DIRECT_MODE", false)
DEBUG_GET_ONCE_OPTION(ovrd_direct_mode_log, "OVRD_DIRECT_MODE_LOG", NULL)
//! In direct mode, wait for SteamVR's writes into each frame's images before Monado reads them (OvrdDirectMode::gpu_sync).
DEBUG_GET_ONCE_BOOL_OPTION(ovrd_direct_mode_gpu_sync, "OVRD_DIRECT_MODE_GPU_SYNC", true)
/*!
 * In direct mode, hand SteamVR the headset pose Monado's tracking predicts for the display time of the frame being
 * rendered (Monado's compositor gives it), with zero velocity so SteamVR doesn't extrapolate: the game then draws with
 * the same prediction as on the OpenXR path, instead of SteamVR's straight-line one (ALVR also sends a pre-predicted
 * head pose with zero velocity).
 */
DEBUG_GET_ONCE_BOOL_OPTION(ovrd_direct_mode_pose, "OVRD_DIRECT_MODE_POSE", true)
//! Direct mode diagnostics: capture one frame's layer images this many seconds into the game (OvrdDirectMode).
DEBUG_GET_ONCE_FLOAT_OPTION(ovrd_direct_mode_capture_s, "OVRD_DIRECT_MODE_CAPTURE", 0.0f)

//! OVRD_LOG_EVENTS=1: log SteamVR events the plugin doesn't handle (off: SteamVR's frame loop doesn't write them).
DEBUG_GET_ONCE_BOOL_OPTION(ovrd_log_events, "OVRD_LOG_EVENTS", false)

#define MODELNUM_LEN (XRT_DEVICE_NAME_LEN + 9) // "[Monado] "


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
	init(float session_size)
	{
		m_enabled = debug_get_bool_option_ovrd_room();
		if (!m_enabled) {
			ovrd_log("Room: disabled (OVRD_ROOM=0), providing a fixed play area instead\n");
			return;
		}

		std::string chaperone = ovrd_room_chaperone_path();
		bool has_drawn_room = false;
		std::string error;
		if (chaperone.empty() || !ovrd_room_prepare_chaperone(chaperone, session_size, &has_drawn_room, &error)) {
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

		ovrd_log("Room: chaperone file %s prepared (universe 3 for this session, universe 2 %s)\n", chaperone.c_str(),
		         has_drawn_room ? "holds a drawn room" : "has no room drawn yet");
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

static OvrdRoom g_room;

#define OPENVR_BONE_COUNT 31

// Debug define(s), always off.
#undef DUMP_POSE
#undef DUMP_POSE_CONTROLLERS


/*
 * Controller
 */

struct MonadoInputComponent
{
	bool has_component;
	bool x;
	bool y;
};

struct SteamVRDriverControl
{
	const char *steamvr_control_path;
	vr::VRInputComponentHandle_t control_handle;
};

struct SteamVRDriverControlInput : SteamVRDriverControl
{
	enum xrt_input_type monado_input_type;
	enum xrt_input_name monado_input_name;

	struct MonadoInputComponent component;

	//! A boolean Monado input given to SteamVR as a 0/1 scalar (the profile declares a value).
	bool bool_as_scalar = false;
};

struct SteamVRDriverControlOutput : SteamVRDriverControl
{
	enum xrt_output_type monado_output_type;
	enum xrt_output_name monado_output_name;
};

static void
copy_vec3(struct xrt_vec3 *from, double *to)
{
	to[0] = from->x;
	to[1] = from->y;
	to[2] = from->z;
}

static void
copy_quat(struct xrt_quat *from, vr::HmdQuaternion_t *to)
{
	to->x = from->x;
	to->y = from->y;
	to->z = from->z;
	to->w = from->w;
}

static void
apply_pose(struct xrt_space_relation *rel, vr::DriverPose_t *m_pose)
{
	if ((rel->relation_flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) != 0) {
		copy_quat(&rel->pose.orientation, &m_pose->qRotation);
	} else {
		m_pose->result = vr::TrackingResult_Running_OutOfRange;
		m_pose->poseIsValid = false;
	}

	if ((rel->relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0) {
		copy_vec3(&rel->pose.position, m_pose->vecPosition);
	} else {
	}

	if ((rel->relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT) != 0) {
		// linear velocity in world space
		copy_vec3(&rel->linear_velocity, m_pose->vecVelocity);
	}

	if ((rel->relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT) != 0) {
		// angular velocity reported by monado in world space,
		// expected by steamvr to be in "controller space"
		struct xrt_quat orientation_inv;
		math_quat_invert(&rel->pose.orientation, &orientation_inv);

		struct xrt_vec3 vel;
		math_quat_rotate_derivative(&orientation_inv, &rel->angular_velocity, &vel);

		copy_vec3(&vel, m_pose->vecAngularVelocity);
	}
}

#define OPENVR_BONE_COUNT 31

// these are currently only used for the root and wrist transforms, but are kept here as they are useful for debugging.
vr::VRBoneTransform_t rightOpenPose[OPENVR_BONE_COUNT] = {
    {{0.000000f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f}}, // Root
    {{0.034038f, 0.036503f, 0.164722f, 1.000000f}, {-0.055147f, -0.078608f, 0.920279f, -0.379296f}},
    //
    {{0.012083f, 0.028070f, 0.025050f, 1.000000f}, {0.567418f, -0.464112f, 0.623374f, -0.272106f}}, // Thumb
    {{-0.040406f, -0.000000f, 0.000000f, 1.000000f}, {0.994838f, 0.082939f, 0.019454f, 0.055130f}},
    {{-0.032517f, -0.000000f, -0.000000f, 1.000000f}, {0.974793f, -0.003213f, 0.021867f, -0.222015f}},
    {{-0.030464f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f}},
    //
    {{-0.000632f, 0.026866f, 0.015002f, 1.000000f}, {0.421979f, -0.644251f, 0.422133f, 0.478202f}}, // Index
    {{-0.074204f, 0.005002f, -0.000234f, 1.000000f}, {0.995332f, 0.007007f, -0.039124f, 0.087949f}},
    {{-0.043930f, 0.000000f, 0.000000f, 1.000000f}, {0.997891f, 0.045808f, 0.002142f, -0.045943f}},
    {{-0.028695f, -0.000000f, -0.000000f, 1.000000f}, {0.999649f, 0.001850f, -0.022782f, -0.013409f}},
    {{-0.022821f, -0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, 0.000000f, -0.000000f}},
    //
    {{-0.002177f, 0.007120f, 0.016319f, 1.000000f}, {0.541276f, -0.546723f, 0.460749f, 0.442520f}}, // Middle
    {{-0.070953f, -0.000779f, -0.000997f, 1.000000f}, {0.980294f, -0.167261f, -0.078959f, 0.069368f}},
    {{-0.043108f, -0.000000f, -0.000000f, 1.000000f}, {0.997947f, 0.018493f, 0.013192f, 0.059886f}},
    {{-0.033266f, -0.000000f, -0.000000f, 1.000000f}, {0.997394f, -0.003328f, -0.028225f, -0.066315f}},
    {{-0.025892f, 0.000000f, -0.000000f, 1.000000f}, {0.999195f, -0.000000f, 0.000000f, 0.040126f}},
    //
    {{-0.000513f, -0.006545f, 0.016348f, 1.000000f}, {0.550143f, -0.516692f, 0.429888f, 0.495548f}}, // Ring
    {{-0.065876f, -0.001786f, -0.000693f, 1.000000f}, {0.990420f, -0.058696f, -0.101820f, 0.072495f}},
    {{-0.040697f, -0.000000f, -0.000000f, 1.000000f}, {0.999545f, -0.002240f, 0.000004f, 0.030081f}},
    {{-0.028747f, 0.000000f, 0.000000f, 1.000000f}, {0.999102f, -0.000721f, -0.012693f, 0.040420f}},
    {{-0.022430f, 0.000000f, -0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}},

    {{0.002478f, -0.018981f, 0.015214f, 1.000000f}, {0.523940f, -0.526918f, 0.326740f, 0.584025f}}, // Pinky
    {{-0.062878f, -0.002844f, -0.000332f, 1.000000f}, {0.986609f, -0.059615f, -0.135163f, 0.069132f}},
    {{-0.030220f, -0.000000f, -0.000000f, 1.000000f}, {0.994317f, 0.001896f, -0.000132f, 0.106446f}},
    {{-0.018187f, -0.000000f, -0.000000f, 1.000000f}, {0.995931f, -0.002010f, -0.052079f, -0.073526f}},
    {{-0.018018f, -0.000000f, 0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}},

    {{0.006059f, 0.056285f, 0.060064f, 1.000000f}, {0.737238f, 0.202745f, -0.594267f, -0.249441f}}, // Aux
    {{0.040416f, -0.043018f, 0.019345f, 1.000000f}, {-0.290331f, 0.623527f, 0.663809f, 0.293734f}},
    {{0.039354f, -0.075674f, 0.047048f, 1.000000f}, {-0.187047f, 0.678062f, 0.659285f, 0.265683f}},
    {{0.038340f, -0.090987f, 0.082579f, 1.000000f}, {-0.183037f, 0.736793f, 0.634757f, 0.143936f}},
    {{0.031806f, -0.087214f, 0.121015f, 1.000000f}, {-0.003659f, 0.758407f, 0.639342f, 0.126678f}},
};

vr::VRBoneTransform_t leftOpenPose[OPENVR_BONE_COUNT] = {
    {{0.000000f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f}},   // Root
                                                                                                      //
    {{-0.034038f, 0.036503f, 0.164722f, 1.000000f}, {-0.055147f, -0.078608f, -0.920279f, 0.379296f}}, // Thumb
    {{-0.012083f, 0.028070f, 0.025050f, 1.000000f}, {0.464112f, 0.567418f, 0.272106f, 0.623374f}},
    {{0.040406f, 0.000000f, -0.000000f, 1.000000f}, {0.994838f, 0.082939f, 0.019454f, 0.055130f}},
    {{0.032517f, 0.000000f, 0.000000f, 1.000000f}, {0.974793f, -0.003213f, 0.021867f, -0.222015f}},
    {{0.030464f, -0.000000f, -0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f}},
    //
    {{0.000632f, 0.026866f, 0.015002f, 1.000000f}, {0.644251f, 0.421979f, -0.478202f, 0.422133f}}, // Index
    {{0.074204f, -0.005002f, 0.000234f, 1.000000f}, {0.995332f, 0.007007f, -0.039124f, 0.087949f}},
    {{0.043930f, -0.000000f, -0.000000f, 1.000000f}, {0.997891f, 0.045808f, 0.002142f, -0.045943f}},
    {{0.028695f, 0.000000f, 0.000000f, 1.000000f}, {0.999649f, 0.001850f, -0.022782f, -0.013409f}},
    {{0.022821f, 0.000000f, -0.000000f, 1.000000f}, {1.000000f, -0.000000f, 0.000000f, -0.000000f}},
    //
    {{0.002177f, 0.007120f, 0.016319f, 1.000000f}, {0.546723f, 0.541276f, -0.442520f, 0.460749f}}, // Middle
    {{0.070953f, 0.000779f, 0.000997f, 1.000000f}, {0.980294f, -0.167261f, -0.078959f, 0.069368f}},
    {{0.043108f, 0.000000f, 0.000000f, 1.000000f}, {0.997947f, 0.018493f, 0.013192f, 0.059886f}},
    {{0.033266f, 0.000000f, 0.000000f, 1.000000f}, {0.997394f, -0.003328f, -0.028225f, -0.066315f}},
    {{0.025892f, -0.000000f, 0.000000f, 1.000000f}, {0.999195f, -0.000000f, 0.000000f, 0.040126f}},

    {{0.000513f, -0.006545f, 0.016348f, 1.000000f}, {0.516692f, 0.550143f, -0.495548f, 0.429888f}}, // Ring
    {{0.065876f, 0.001786f, 0.000693f, 1.000000f}, {0.990420f, -0.058696f, -0.101820f, 0.072495f}},
    {{0.040697f, 0.000000f, 0.000000f, 1.000000f}, {0.999545f, -0.002240f, 0.000004f, 0.030081f}},
    {{0.028747f, -0.000000f, -0.000000f, 1.000000f}, {0.999102f, -0.000721f, -0.012693f, 0.040420f}},
    {{0.022430f, -0.000000f, 0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}},

    {{-0.002478f, -0.018981f, 0.015214f, 1.000000f}, {0.526918f, 0.523940f, -0.584025f, 0.326740f}}, // Pinky
    {{0.062878f, 0.002844f, 0.000332f, 1.000000f}, {0.986609f, -0.059615f, -0.135163f, 0.069132f}},
    {{0.030220f, 0.000000f, 0.000000f, 1.000000f}, {0.994317f, 0.001896f, -0.000132f, 0.106446f}},
    {{0.018187f, 0.000000f, 0.000000f, 1.000000f}, {0.995931f, -0.002010f, -0.052079f, -0.073526f}},
    {{0.018018f, 0.000000f, -0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}},

    {{-0.006059f, 0.056285f, 0.060064f, 1.000000f}, {0.737238f, 0.202745f, 0.594267f, 0.249441f}}, // Aux
    {{-0.040416f, -0.043018f, 0.019345f, 1.000000f}, {-0.290331f, 0.623527f, -0.663809f, -0.293734f}},
    {{-0.039354f, -0.075674f, 0.047048f, 1.000000f}, {-0.187047f, 0.678062f, -0.659285f, -0.265683f}},
    {{-0.038340f, -0.090987f, 0.082579f, 1.000000f}, {-0.183037f, 0.736793f, -0.634757f, -0.143936f}},
    {{-0.031806f, -0.087214f, 0.121015f, 1.000000f}, {-0.003659f, 0.758407f, -0.639342f, -0.126678f}},
};

template <class T, class U>
void
convert_quaternion(const T &p_quatA, U &p_quatB)
{
	p_quatB.x = p_quatA.x;
	p_quatB.y = p_quatA.y;
	p_quatB.z = p_quatA.z;
	p_quatB.w = p_quatA.w;
}

xrt_quat
apply_bone_hand_transform(xrt_quat p_rot, xrt_hand hand)
{
	std::swap(p_rot.x, p_rot.z);
	p_rot.z *= -1.f;
	if (hand == XRT_HAND_RIGHT)
		return p_rot;

	p_rot.x *= -1.f;
	p_rot.y *= -1.f;
	return p_rot;
}

void
metacarpal_joints_to_bone_transform(struct xrt_hand_joint_set *hand_joint_set,
                                    vr::VRBoneTransform_t *out_bone_transforms,
                                    xrt_hand hand)
{
	struct xrt_hand_joint_value *joint_values = hand_joint_set->values.hand_joint_set_default;

	// Apply orientations for four-finger metacarpals.
	for (int joint :
	     {XRT_HAND_JOINT_THUMB_METACARPAL, XRT_HAND_JOINT_INDEX_METACARPAL, XRT_HAND_JOINT_MIDDLE_METACARPAL,
	      XRT_HAND_JOINT_RING_METACARPAL, XRT_HAND_JOINT_LITTLE_METACARPAL}) {
		struct xrt_hand_joint_value *current_joint = &joint_values[joint];
		struct xrt_hand_joint_value *parent_joint = &joint_values[XRT_HAND_JOINT_WRIST];

		xrt_quat diff_openxr;
		// These should do the exact same things.
		xrt_quat parent_inv;
		math_quat_invert(&parent_joint->relation.pose.orientation, &parent_inv);
		math_quat_rotate(&parent_inv, &current_joint->relation.pose.orientation, &diff_openxr);
		xrt_quat diff_openvr = apply_bone_hand_transform(diff_openxr, hand);


		/**
		 * * if you try applying the metacarpal transforms without the magic quaternion, everything from the
		 * metacarpals onwards is rotated 90 degrees.
		 * In the neutral pose sample, all the metacarpals have a
		 * rotation relatively close to {w=0.5, x=0.5, y=-0.5, z=0.5} which is an Important Quaternion because
		 * it probably represents some 90 degree rotation. Maybe, and this was just a random guess, if I took
		 * the regular metacarpal orientations and rotated them by that quat, everything would work.
		 */
		xrt_quat magic_prerotate = XRT_QUAT_IDENTITY;
		magic_prerotate.w = 0.5;
		magic_prerotate.x = 0.5;
		magic_prerotate.y = -0.5;
		magic_prerotate.z = 0.5;

		if (hand == XRT_HAND_RIGHT) {
			magic_prerotate.y *= -1.f;
			magic_prerotate.x *= -1.f;
		}

		xrt_quat final_diff;
		math_quat_rotate(&magic_prerotate, &diff_openvr, &final_diff);
		convert_quaternion(final_diff, out_bone_transforms[joint].orientation);

		xrt_vec3 global_diff_from_this_to_parent =
		    m_vec3_sub(current_joint->relation.pose.position, parent_joint->relation.pose.position);

		xrt_vec3 translation_wrist_rel;
		math_quat_rotate_vec3(&parent_inv, &global_diff_from_this_to_parent, &translation_wrist_rel);

		// Y = X?
		out_bone_transforms[joint].position.v[0] = translation_wrist_rel.y;
		out_bone_transforms[joint].position.v[1] = translation_wrist_rel.x;
		out_bone_transforms[joint].position.v[2] = -translation_wrist_rel.z;
		out_bone_transforms[joint].position.v[3] = 1.f;

		if (hand == XRT_HAND_RIGHT)
			out_bone_transforms[joint].position.v[1] *= -1.f;
	}
}

void
flexion_joints_to_bone_transform(struct xrt_hand_joint_set *hand_joint_set,
                                 vr::VRBoneTransform_t *out_bone_transforms,
                                 xrt_hand hand)
{
	struct xrt_hand_joint_value *joint_values = hand_joint_set->values.hand_joint_set_default;

	// Apply orientations for four-finger pxm and onward
	int parent = -1;
	for (int joint = XRT_HAND_JOINT_THUMB_METACARPAL; joint < XRT_HAND_JOINT_COUNT; joint++) {
		if (u_hand_joint_is_metacarpal((xrt_hand_joint)joint)) {
			parent = joint;
			continue;
		}
		struct xrt_hand_joint_value *current_joint = &joint_values[joint];
		struct xrt_hand_joint_value *parent_joint = &joint_values[parent];


		xrt_quat diff_openxr;
		math_quat_unrotate(&parent_joint->relation.pose.orientation, &current_joint->relation.pose.orientation,
		                   &diff_openxr);

		xrt_quat diff_openvr = apply_bone_hand_transform(diff_openxr, hand);
		convert_quaternion(diff_openvr, out_bone_transforms[joint].orientation);
		xrt_vec3 global_diff_from_this_to_parent =
		    m_vec3_sub(current_joint->relation.pose.position, parent_joint->relation.pose.position);


		float bone_length = m_vec3_len(global_diff_from_this_to_parent);
		// OpenVR left hand has +X forward. Weird, huh?
		out_bone_transforms[joint].position = {bone_length, 0, 0, 1};

		if (hand == XRT_HAND_RIGHT)
			out_bone_transforms[joint].position.v[0] *= -1.f;

		parent = joint;
	}
}

void
hand_joint_set_to_bone_transform(struct xrt_hand_joint_set hand_joint_set,
                                 vr::VRBoneTransform_t *out_bone_transforms,
                                 xrt_hand hand)
{
	// fill bone transforms with a default open pose to manipulate later
	for (int i : {XRT_HAND_JOINT_WRIST, XRT_HAND_JOINT_PALM}) {
		out_bone_transforms[i] = hand == XRT_HAND_LEFT ? leftOpenPose[i] : rightOpenPose[i];
	}

	metacarpal_joints_to_bone_transform(&hand_joint_set, out_bone_transforms, hand);
	flexion_joints_to_bone_transform(&hand_joint_set, out_bone_transforms, hand);
}

class CDeviceDriver_Monado_Controller : public vr::ITrackedDeviceServerDriver
{
public:
	CDeviceDriver_Monado_Controller(struct xrt_instance *xinst, struct xrt_device *xdev, enum xrt_hand hand)
	    : m_xdev(xdev), m_hand(hand)
	{
		ovrd_log("Creating Controller %s\n", xdev->str);

		m_handed_controller = true;

		m_emulate_index_controller = debug_get_bool_option_emulate_index_controller();

		if (m_emulate_index_controller) {
			ovrd_log("Emulating Index Controller\n");
		} else {
			ovrd_log("Using Monado Controller profile\n");
		}

		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
		m_pose = {};

		snprintf(m_sModelNumber, MODELNUM_LEN, "[Monado] %s", xdev->str);
		strncpy(m_sSerialNumber, xdev->serial, XRT_DEVICE_NAME_LEN);

		switch (this->m_xdev->name) {
		case XRT_DEVICE_INDEX_CONTROLLER:
			if (hand == XRT_HAND_LEFT) {
				m_render_model =
				    "{indexcontroller}valve_controller_knu_1_0_"
				    "left";
			}
			if (hand == XRT_HAND_RIGHT) {
				m_render_model =
				    "{indexcontroller}valve_controller_knu_1_0_"
				    "right";
			}
			break;
		case XRT_DEVICE_TOUCH_CONTROLLER:
			if (hand == XRT_HAND_LEFT) {
				m_render_model = "oculus_cv1_controller_left";
			}
			if (hand == XRT_HAND_RIGHT) {
				m_render_model = "oculus_cv1_controller_right";
			}
			break;
		case XRT_DEVICE_VIVE_WAND: m_render_model = "vr_controller_vive_1_5"; break;
		case XRT_DEVICE_VIVE_TRACKER_GEN1:
		case XRT_DEVICE_VIVE_TRACKER_GEN2:
		case XRT_DEVICE_VIVE_TRACKER_GEN3:
		case XRT_DEVICE_VIVE_TRACKER_TUNDRA: m_render_model = "{htc}vr_tracker_vive_1_0"; break;
		case XRT_DEVICE_FLIPVR:
			if (hand == XRT_HAND_LEFT) {
				m_render_model = "{shiftall}flipvr_controller_vc1b_left";
			}
			if (hand == XRT_HAND_RIGHT) {
				m_render_model = "{shiftall}flipvr_controller_vc1b_right";
			}
			break;
		case XRT_DEVICE_PSSENSE: {
			bool sony = debug_get_bool_option_ovrd_pssense_render_models();
			if (hand == XRT_HAND_LEFT) {
				m_render_model = sony ? "{monado}playstation_vr2_sense_left" : "oculus_cv1_controller_left";
			} else {
				m_render_model = sony ? "{monado}playstation_vr2_sense_right" : "oculus_cv1_controller_right";
			}
		} break;
		case XRT_DEVICE_PSMV:
		case XRT_DEVICE_HYDRA:
		case XRT_DEVICE_DAYDREAM:
		case XRT_DEVICE_GENERIC_HMD:
		default: m_render_model = "locator_one_sided"; break;
		}

		ovrd_log("Render model based on Monado: %s\n", m_render_model);

		vr::VRServerDriverHost()->TrackedDeviceAdded(GetSerialNumber().c_str(),
		                                             vr::TrackedDeviceClass_Controller, this);
	}
	virtual ~CDeviceDriver_Monado_Controller() {}

	void
	AddControl(const char *steamvr_control_path,
	           enum xrt_input_name monado_input_name,
	           struct MonadoInputComponent *component)
	{
		enum xrt_input_type monado_input_type = XRT_GET_INPUT_TYPE(monado_input_name);

		SteamVRDriverControlInput in;

		in.monado_input_type = monado_input_type;
		in.steamvr_control_path = steamvr_control_path;
		in.monado_input_name = monado_input_name;
		if (component != NULL) {
			in.component = *component;
		} else {
			in.component.has_component = false;
		}

		if (monado_input_type == XRT_INPUT_TYPE_BOOLEAN) {
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, steamvr_control_path,
			                                            &in.control_handle);

		} else if (monado_input_type == XRT_INPUT_TYPE_VEC1_MINUS_ONE_TO_ONE) {
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, steamvr_control_path,
			                                           &in.control_handle, vr::VRScalarType_Absolute,
			                                           vr::VRScalarUnits_NormalizedTwoSided);

		} else if (monado_input_type == XRT_INPUT_TYPE_VEC1_ZERO_TO_ONE) {
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, steamvr_control_path,
			                                           &in.control_handle, vr::VRScalarType_Absolute,
			                                           vr::VRScalarUnits_NormalizedOneSided);

		} else if (monado_input_type == XRT_INPUT_TYPE_VEC2_MINUS_ONE_TO_ONE) {
			// 2D values are added as 2 1D values
			// usually those are [-1,1]
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, steamvr_control_path,
			                                           &in.control_handle, vr::VRScalarType_Absolute,
			                                           vr::VRScalarUnits_NormalizedTwoSided);
		}

		m_input_controls.push_back(in);
		ovrd_log("Added input %s\n", steamvr_control_path);
	}

	//! A boolean Monado input as a 0/1 scalar SteamVR component.
	void
	AddBoolAsScalarControl(const char *steamvr_control_path, enum xrt_input_name monado_input_name)
	{
		SteamVRDriverControlInput in;
		in.monado_input_type = XRT_GET_INPUT_TYPE(monado_input_name);
		in.steamvr_control_path = steamvr_control_path;
		in.monado_input_name = monado_input_name;
		in.component.has_component = false;
		in.bool_as_scalar = true;
		vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, steamvr_control_path, &in.control_handle,
		                                           vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
		m_input_controls.push_back(in);
		ovrd_log("Added input %s (button as value)\n", steamvr_control_path);
	}

	/*!
	 * PS VR2 Sense with Sony's input names (playstation_vr2_sense_controller_profile.json, as Sony's driver):
	 * L2/R2 the trigger, L1/R1 the grip button (a click on the Sense, given as a 0/1 value as Sony's profile declares
	 * it), the _sensor inputs the finger proximity, the face buttons, create/options and PS per hand.
	 */
	void
	AddPssenseControls()
	{
		bool left = m_hand == XRT_HAND_LEFT;
		struct MonadoInputComponent x = {true, true, false};
		struct MonadoInputComponent y = {true, false, true};

		AddControl(left ? "/input/l2/value" : "/input/r2/value", XRT_INPUT_PSSENSE_TRIGGER_VALUE, NULL);
		AddControl(left ? "/input/l2/touch" : "/input/r2/touch", XRT_INPUT_PSSENSE_TRIGGER_TOUCH, NULL);
		AddControl(left ? "/input/l2_sensor/value" : "/input/r2_sensor/value",
		           XRT_INPUT_PSSENSE_TRIGGER_PROXIMITY_FLOAT, NULL);
		AddBoolAsScalarControl(left ? "/input/l1/value" : "/input/r1/value", XRT_INPUT_PSSENSE_SQUEEZE_CLICK);
		AddControl(left ? "/input/l1/touch" : "/input/r1/touch", XRT_INPUT_PSSENSE_SQUEEZE_TOUCH, NULL);
		AddControl(left ? "/input/l1_sensor/value" : "/input/r1_sensor/value",
		           XRT_INPUT_PSSENSE_SQUEEZE_PROXIMITY_FLOAT, NULL);

		AddControl(left ? "/input/left_stick/x" : "/input/right_stick/x", XRT_INPUT_PSSENSE_THUMBSTICK, &x);
		AddControl(left ? "/input/left_stick/y" : "/input/right_stick/y", XRT_INPUT_PSSENSE_THUMBSTICK, &y);
		AddControl(left ? "/input/left_stick/click" : "/input/right_stick/click", XRT_INPUT_PSSENSE_THUMBSTICK_CLICK,
		           NULL);
		AddControl(left ? "/input/left_stick/touch" : "/input/right_stick/touch", XRT_INPUT_PSSENSE_THUMBSTICK_TOUCH,
		           NULL);

		if (left) {
			AddControl("/input/square/click", XRT_INPUT_PSSENSE_SQUARE_CLICK, NULL);
			AddControl("/input/square/touch", XRT_INPUT_PSSENSE_SQUARE_TOUCH, NULL);
			AddControl("/input/triangle/click", XRT_INPUT_PSSENSE_TRIANGLE_CLICK, NULL);
			AddControl("/input/triangle/touch", XRT_INPUT_PSSENSE_TRIANGLE_TOUCH, NULL);
			AddControl("/input/create/click", XRT_INPUT_PSSENSE_SHARE_CLICK, NULL);
			AddControl("/input/left_ps/click", XRT_INPUT_PSSENSE_PS_CLICK, NULL);
		} else {
			AddControl("/input/cross/click", XRT_INPUT_PSSENSE_CROSS_CLICK, NULL);
			AddControl("/input/cross/touch", XRT_INPUT_PSSENSE_CROSS_TOUCH, NULL);
			AddControl("/input/circle/click", XRT_INPUT_PSSENSE_CIRCLE_CLICK, NULL);
			AddControl("/input/circle/touch", XRT_INPUT_PSSENSE_CIRCLE_TOUCH, NULL);
			AddControl("/input/options/click", XRT_INPUT_PSSENSE_OPTIONS_CLICK, NULL);
			AddControl("/input/right_ps/click", XRT_INPUT_PSSENSE_PS_CLICK, NULL);
		}

		AddOutputControl(XRT_OUTPUT_NAME_PSSENSE_VIBRATION, "/output/haptic");
	}

	void
	AddOutputControl(enum xrt_output_name monado_output_name, const char *steamvr_control_path)
	{
		SteamVRDriverControlOutput out;

		/// @todo when there are multiple output types: XRT_GET_OUTPUT_TYPE(monado_output_name);
		enum xrt_output_type monado_output_type = XRT_OUTPUT_TYPE_VIBRATION;

		out.monado_output_type = monado_output_type;
		out.steamvr_control_path = steamvr_control_path;
		out.monado_output_name = monado_output_name;

		vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, out.steamvr_control_path,
		                                           &out.control_handle);

		m_output_controls.push_back(out);
		ovrd_log("Added output %s\n", steamvr_control_path);
	}

	void
	AddSkeletonControl(const char *steamvr_skeleton_name,
	                   const char *steamvr_control_path,
	                   enum xrt_input_name monado_input_name)
	{
		enum xrt_input_type monado_input_type = XRT_GET_INPUT_TYPE(monado_input_name);

		SteamVRDriverControlInput in;

		in.monado_input_type = monado_input_type;
		in.steamvr_control_path = steamvr_control_path;
		in.monado_input_name = monado_input_name;
		in.component.has_component = false;

		vr::EVRInputError err = vr::VRDriverInput()->CreateSkeletonComponent(
		    m_ulPropertyContainer, steamvr_skeleton_name, steamvr_control_path, "/pose/raw",
		    vr::VRSkeletalTracking_Full, NULL, OPENVR_BONE_COUNT, &in.control_handle);
		if (err) {
			ovrd_log("Error adding skeletal input: %i", err);
			return;
		}


		m_skeletal_input_control = in;
		ovrd_log("Added skeleton input %s\n", steamvr_control_path);
	}

	//! Hand-tracked controller (ht_ctrl_emu) as an Index controller: pinch as the trigger, plus the hand skeleton.
	void
	AddHandCtrlEmuControls()
	{
		AddControl("/input/trigger/value", XRT_INPUT_HAND_CTRL_EMU_PINCH_VALUE, NULL);
		AddControl("/input/trigger/click", XRT_INPUT_HAND_CTRL_EMU_PINCH_BOOL, NULL);
		const std::string str_hand = m_hand == XRT_HAND_LEFT ? "left" : "right";
		AddSkeletonControl(("/input/skeleton/" + str_hand).c_str(), ("/skeleton/hand/" + str_hand).c_str(),
		                   m_hand == XRT_HAND_LEFT ? XRT_INPUT_HT_UNOBSTRUCTED_LEFT
		                                           : XRT_INPUT_HT_UNOBSTRUCTED_RIGHT);
	}

	void
	AddEmulatedIndexControls()
	{
		switch (this->m_xdev->name) {
		case XRT_DEVICE_INDEX_CONTROLLER: {
			AddControl("/input/trigger/value", XRT_INPUT_INDEX_TRIGGER_VALUE, NULL);

			AddControl("/input/trigger/click", XRT_INPUT_INDEX_TRIGGER_CLICK, NULL);
			AddControl("/input/trigger/touch", XRT_INPUT_INDEX_TRIGGER_TOUCH, NULL);


			AddControl("/input/system/click", XRT_INPUT_INDEX_SYSTEM_CLICK, NULL);
			AddControl("/input/system/touch", XRT_INPUT_INDEX_SYSTEM_TOUCH, NULL);

			AddControl("/input/a/click", XRT_INPUT_INDEX_A_CLICK, NULL);
			AddControl("/input/a/touch", XRT_INPUT_INDEX_A_TOUCH, NULL);

			AddControl("/input/b/click", XRT_INPUT_INDEX_B_CLICK, NULL);
			AddControl("/input/b/touch", XRT_INPUT_INDEX_B_TOUCH, NULL);


			AddControl("/input/grip/force", XRT_INPUT_INDEX_SQUEEZE_FORCE, NULL);

			AddControl("/input/grip/value", XRT_INPUT_INDEX_SQUEEZE_VALUE, NULL);

			struct MonadoInputComponent x = {true, true, false};
			struct MonadoInputComponent y = {true, false, true};

			AddControl("/input/thumbstick/click", XRT_INPUT_INDEX_THUMBSTICK_CLICK, NULL);

			AddControl("/input/thumbstick/touch", XRT_INPUT_INDEX_THUMBSTICK_TOUCH, NULL);

			AddControl("/input/thumbstick/x", XRT_INPUT_INDEX_THUMBSTICK, &x);

			AddControl("/input/thumbstick/y", XRT_INPUT_INDEX_THUMBSTICK, &y);


			AddControl("/input/trackpad/force", XRT_INPUT_INDEX_TRACKPAD_FORCE, NULL);

			AddControl("/input/trackpad/touch", XRT_INPUT_INDEX_TRACKPAD_TOUCH, NULL);

			AddControl("/input/trackpad/x", XRT_INPUT_INDEX_TRACKPAD, &x);

			AddControl("/input/trackpad/y", XRT_INPUT_INDEX_TRACKPAD, &y);

			if (m_xdev->supported.hand_tracking) {
				ovrd_log("Enabling skeletal input as this device supports it");

				// skeletal input compatibility with games is a bit funky with any controllers
				// other than the index controller, so only do skeletal input with index
				// emulation
				const std::string str_hand = m_hand == XRT_HAND_LEFT ? "left" : "right";


				AddSkeletonControl(("/input/skeleton/" + str_hand).c_str(),
				                   ("/skeleton/hand/" + str_hand).c_str(),
				                   XRT_INPUT_HT_UNOBSTRUCTED_RIGHT);
				RunFrame();
			} else
				ovrd_log("Not enabling skeletal input as this device does not support it");


			AddOutputControl(XRT_OUTPUT_NAME_INDEX_HAPTIC, "/output/haptic");
		}

		break;
		case XRT_DEVICE_VIVE_WAND: {
			AddControl("/input/trigger/value", XRT_INPUT_VIVE_TRIGGER_VALUE, NULL);

			AddControl("/input/trigger/click", XRT_INPUT_VIVE_TRIGGER_CLICK, NULL);


			AddControl("/input/system/click", XRT_INPUT_VIVE_SYSTEM_CLICK, NULL);

			AddControl("/input/a/click", XRT_INPUT_VIVE_TRACKPAD_CLICK, NULL);

			AddControl("/input/b/click", XRT_INPUT_VIVE_MENU_CLICK, NULL);

			struct MonadoInputComponent x = {true, true, false};
			struct MonadoInputComponent y = {true, false, true};

			AddControl("/input/trackpad/touch", XRT_INPUT_VIVE_TRACKPAD_TOUCH, NULL);

			AddControl("/input/trackpad/x", XRT_INPUT_VIVE_TRACKPAD, &x);

			AddControl("/input/trackpad/y", XRT_INPUT_VIVE_TRACKPAD, &y);

			AddOutputControl(XRT_OUTPUT_NAME_VIVE_HAPTIC, "/output/haptic");
		} break;
		case XRT_DEVICE_PSMV: {

			AddControl("/input/trigger/value", XRT_INPUT_PSMV_TRIGGER_VALUE, NULL);

			AddControl("/input/trigger/click", XRT_INPUT_PSMV_MOVE_CLICK, NULL);

			AddControl("/input/system/click", XRT_INPUT_PSMV_PS_CLICK, NULL);

			AddControl("/input/a/click", XRT_INPUT_PSMV_CROSS_CLICK, NULL);

			AddControl("/input/b/click", XRT_INPUT_PSMV_SQUARE_CLICK, NULL);

			AddOutputControl(XRT_OUTPUT_NAME_PSMV_RUMBLE_VIBRATION, "/output/haptic");
		} break;
		case XRT_DEVICE_FLIPVR: {

			AddControl("/input/trigger/value", XRT_INPUT_FLIPVR_TRIGGER_VALUE, NULL);

			AddControl("/input/trigger/click", XRT_INPUT_FLIPVR_TRIGGER_CLICK, NULL);
			AddControl("/input/trigger/touch", XRT_INPUT_FLIPVR_TRIGGER_TOUCH, NULL);

			AddControl("/input/system/click", XRT_INPUT_FLIPVR_SYSTEM_CLICK, NULL);

			AddControl("/input/a/click", XRT_INPUT_FLIPVR_A_CLICK, NULL);
			AddControl("/input/a/touch", XRT_INPUT_FLIPVR_A_TOUCH, NULL);

			AddControl("/input/b/click", XRT_INPUT_FLIPVR_B_CLICK, NULL);
			AddControl("/input/b/touch", XRT_INPUT_FLIPVR_B_TOUCH, NULL);

			AddControl("/input/a/click", XRT_INPUT_FLIPVR_X_CLICK, NULL);
			AddControl("/input/a/touch", XRT_INPUT_FLIPVR_X_TOUCH, NULL);

			AddControl("/input/b/click", XRT_INPUT_FLIPVR_Y_CLICK, NULL);
			AddControl("/input/b/touch", XRT_INPUT_FLIPVR_Y_TOUCH, NULL);

			AddControl("/input/grip/value", XRT_INPUT_FLIPVR_SQUEEZE_VALUE, NULL);

			AddControl("/input/grip/click", XRT_INPUT_FLIPVR_SQUEEZE_CLICK, NULL);

			struct MonadoInputComponent x = {true, true, false};
			struct MonadoInputComponent y = {true, false, true};

			AddControl("/input/thumbstick/click", XRT_INPUT_FLIPVR_THUMBSTICK_CLICK, NULL);

			AddControl("/input/thumbstick/touch", XRT_INPUT_FLIPVR_THUMBSTICK_TOUCH, NULL);

			AddControl("/input/thumbstick/x", XRT_INPUT_FLIPVR_THUMBSTICK, &x);

			AddControl("/input/thumbstick/y", XRT_INPUT_FLIPVR_THUMBSTICK, &y);

			AddOutputControl(XRT_OUTPUT_NAME_FLIPVR_HAPTIC, "/output/haptic");
		}

		case XRT_DEVICE_TOUCH_CONTROLLER: break;    // TODO
		case XRT_DEVICE_WMR_CONTROLLER: break;      // TODO
		case XRT_DEVICE_XBOX_CONTROLLER: break;     // TODO
		case XRT_DEVICE_VIVE_TRACKER_GEN1: break;   // TODO
		case XRT_DEVICE_VIVE_TRACKER_GEN2: break;   // TODO
		case XRT_DEVICE_VIVE_TRACKER_GEN3: break;   // TODO
		case XRT_DEVICE_VIVE_TRACKER_TUNDRA: break; // TODO
		case XRT_DEVICE_REALSENSE: break;
		case XRT_DEVICE_DEPTHAI: break;

		case XRT_DEVICE_HAND_INTERACTION: break;  // there is no hardware
		case XRT_DEVICE_GO_CONTROLLER: break;     // hardware has no haptics
		case XRT_DEVICE_DAYDREAM: break;          // hardware has no haptics
		case XRT_DEVICE_HYDRA: break;             // hardware has no haptics
		case XRT_DEVICE_SIMPLE_CONTROLLER: break; // shouldn't happen
		case XRT_DEVICE_HAND_TRACKER: break;      // shouldn't happen
		case XRT_DEVICE_GENERIC_HMD:
		case XRT_DEVICE_VIVE_PRO: break; // no
		case XRT_DEVICE_EXT_HAND_INTERACTION: break;

		default: break;
		}
	}

	static struct profile_template *
	get_profile_template(enum xrt_device_name device_name)
	{
		for (int i = 0; i < OXR_BINDINGS_PROFILE_TEMPLATE_COUNT; i++) {
			if (profile_templates[i].name == device_name)
				return &profile_templates[i];
		}
		return NULL;
	}

	void
	AddMonadoInput(struct binding_template *b)
	{
		enum xrt_input_name monado_input_name = b->input;
		const char *steamvr_path = b->steamvr_path;

		enum xrt_input_type monado_input_type = XRT_GET_INPUT_TYPE(monado_input_name);

		switch (monado_input_type) {
		case XRT_INPUT_TYPE_BOOLEAN:
		case XRT_INPUT_TYPE_VEC1_MINUS_ONE_TO_ONE:
		case XRT_INPUT_TYPE_VEC1_ZERO_TO_ONE: AddControl(steamvr_path, monado_input_name, NULL); break;

		case XRT_INPUT_TYPE_VEC2_MINUS_ONE_TO_ONE: {
			std::string xpath = std::string(steamvr_path) + std::string("/x");
			std::string ypath = std::string(steamvr_path) + std::string("/y");

			struct MonadoInputComponent x = {true, true, false};
			struct MonadoInputComponent y = {true, false, true};

			AddControl(xpath.c_str(), monado_input_name, &x);
			AddControl(ypath.c_str(), monado_input_name, &y);
		} break;
		case XRT_INPUT_TYPE_POSE:
			//! @todo how to handle poses?
		case XRT_INPUT_TYPE_HAND_TRACKING:
		case XRT_INPUT_TYPE_FACE_TRACKING:
		case XRT_INPUT_TYPE_BODY_TRACKING:
		case XRT_INPUT_TYPE_VEC3_MINUS_ONE_TO_ONE: break;
		}
	}

	void
	AddMonadoControls()
	{
		struct profile_template *p = get_profile_template(m_xdev->name);
		if (!p) {
			ovrd_log("No profile template for %s\n", m_xdev->str);
			return;
		}

		for (size_t i = 0; i < p->binding_count; i++) {
			struct binding_template *b = &p->bindings[i];

			if (b->input != 0) {
				AddMonadoInput(b);
			}
			if (b->output != 0) {
				AddOutputControl(b->output, b->steamvr_path);
			}
		}
	}

	void
	PoseUpdateThreadFunction()
	{
		ovrd_log("Starting controller pose update thread for %s\n", m_xdev->str);

		while (m_poseUpdating) {
			//! @todo figure out the best pose update rate
			std::this_thread::sleep_for(std::chrono::milliseconds(1));

			if (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid) {
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, GetPose(),
				                                                   sizeof(vr::DriverPose_t));
			}
		}

		ovrd_log("Stopping controller pose update thread for %s\n", m_xdev->str);
	}

	vr::EVRInitError
	Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		ovrd_log("Activating Controller SteamVR[%d]\n", unObjectId);

		if (!m_handed_controller) {
			//! @todo handle trackers etc
			ovrd_log("Unhandled: %s\n", m_xdev->str);
			return vr::VRInitError_Unknown;
		}

		m_unObjectId = unObjectId;

		if (this->m_xdev == NULL) {
			ovrd_log("Error: xdev NULL\n");
			return vr::VRInitError_Init_InterfaceNotFound;
		}

		// clang-format off


		m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

		// Universe 2 (the saved room) or 3 (this session only), see OvrdRoom; never 0 (invalid) or 1 (Oculus)
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_CurrentUniverseId_Uint64, g_room.universe());
		g_room.add_container(m_ulPropertyContainer);
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);

		if (m_hand == XRT_HAND_LEFT) {
			ovrd_log("Left Controller\n");
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_LeftHand);
		} else if (m_hand == XRT_HAND_RIGHT) {
			ovrd_log("Right Controller\n");
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_RightHand);
		}
		m_pose.poseIsValid = false;
		m_pose.deviceIsConnected = true;
		m_pose.result = vr::TrackingResult_Uninitialized;
		m_pose.willDriftInYaw = !m_xdev->supported.position_tracking;

		if (m_emulate_index_controller) {
			m_input_profile = std::string("{indexcontroller}/input/index_controller_profile.json");
			m_controller_type = "knuckles";
			if (m_hand == XRT_HAND_LEFT) {
				m_render_model = "{indexcontroller}valve_controller_knu_1_0_left";
			} else if (m_hand == XRT_HAND_RIGHT) {
				m_render_model = "{indexcontroller}valve_controller_knu_1_0_right";
			}
		} else if (m_xdev->name == XRT_DEVICE_HAND_CTRL_EMU) {
			/*
			 * Optical hand tracking's emulated controllers (ht_ctrl_emu): presented as Index controllers, the
			 * profile SteamVR and apps use for full hand skeletons, with pinch as the trigger. Monado's binding
			 * templates have no entry for this device, so the generic path below refused it.
			 */
			m_input_profile = std::string("{indexcontroller}/input/index_controller_profile.json");
			m_controller_type = "knuckles";
		} else if (m_xdev->name == XRT_DEVICE_PSSENSE) {
			/*
			 * Present the Sense controllers as Sony's driver does, so SteamVR and apps recognise them as PS VR2
			 * Sense: controller type playstation_vr2_sense, Sony's input names (see AddPssenseControls), and a
			 * remapping (Sony's format) that lets apps with only Touch, Index, Vive or simple controller bindings
			 * use them. As oculus_touch with Monado's own input names, SteamVR showed them as Touch controllers.
			 */
			m_input_profile = "{monado}/input/playstation_vr2_sense_controller_profile.json";
			m_controller_type = "playstation_vr2_sense";
		} else {

			struct profile_template *p = get_profile_template(m_xdev->name);

			if (p == NULL) {
				ovrd_log("Monado device has unknown profile: %d\n", m_xdev->name);
				return vr::VRInitError_Unknown;
			}

			m_input_profile = std::string("{monado}/input/") + std::string(p->steamvr_input_profile_path);
			m_controller_type = p->steamvr_controller_type;
		}

		ovrd_log("Using input profile %s\n", m_input_profile.c_str());
		ovrd_log("Using render model%s\n", m_render_model);
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_InputProfilePath_String, m_input_profile.c_str());
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, m_render_model);
		if (!m_emulate_index_controller && m_xdev->name == XRT_DEVICE_PSSENSE) {
			bool left = m_hand == XRT_HAND_LEFT;
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, m_controller_type);
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, left ? "PlayStation VR2 Sense Left" : "PlayStation VR2 Sense Right");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "Sony Interactive Entertainment");
			if (debug_get_bool_option_ovrd_psvr2_sony_icons()) {
				// Sony's icons, linked into our resources by the launcher (psvr2-test/launch-steamvr.sh).
				const char *s = left ? "left" : "right";
				char p[256];
				const struct { vr::ETrackedDeviceProperty prop; const char *state; } icons[] = {
				    {vr::Prop_NamedIconPathDeviceOff_String, "off"},
				    {vr::Prop_NamedIconPathDeviceSearching_String, "searching"},
				    {vr::Prop_NamedIconPathDeviceSearchingAlert_String, "searching"},
				    {vr::Prop_NamedIconPathDeviceReady_String, "ready"},
				    {vr::Prop_NamedIconPathDeviceReadyAlert_String, "ready"},
				    {vr::Prop_NamedIconPathDeviceNotReady_String, "error"},
				    {vr::Prop_NamedIconPathDeviceStandby_String, "standby"},
				    {vr::Prop_NamedIconPathDeviceAlertLow_String, "ready_low"},
				};
				for (const auto &ic : icons) {
					snprintf(p, sizeof(p), "{monado}/icons/playstation_vr2/%s_controller_status_%s@2x.%s", s, ic.state,
					         strcmp(ic.state, "searching") == 0 ? "gif" : "png");
					vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, ic.prop, p);
				}
			}
		} else {
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, m_xdev->str);
		}

		// clang-format on

		m_input_controls.clear();
		if (m_emulate_index_controller) {
			AddEmulatedIndexControls();
		} else if (m_xdev->name == XRT_DEVICE_HAND_CTRL_EMU) {
			AddHandCtrlEmuControls();
		} else if (m_xdev->name == XRT_DEVICE_PSSENSE) {
			AddPssenseControls();
		} else {
			AddMonadoControls();
		}

		ovrd_log("Controller %d activated\n", m_unObjectId);

		m_poseUpdateThread = new std::thread(&CDeviceDriver_Monado_Controller::PoseUpdateThreadFunction, this);
		if (!m_poseUpdateThread) {
			ovrd_log("Unable to create pose updated thread for %s\n", m_xdev->str);
			return vr::VRInitError_Driver_Failed;
		}

		return vr::VRInitError_None;
	}

	void
	Deactivate()
	{
		ovrd_log("deactivate controller\n");
		m_poseUpdating = false;
		m_poseUpdateThread->join();
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
	}

	void
	EnterStandby()
	{
		ovrd_log("standby controller\n");
	}

	void *
	GetComponent(const char *pchComponentNameAndVersion)
	{
		// deprecated API
		// ovrd_log("get controller component %s\n",
		// pchComponentNameAndVersion);
		return NULL;
	}

	/** debug request from a client */
	void
	DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize)
	{
		if (unResponseBufferSize >= 1)
			pchResponseBuffer[0] = 0;
	}

	vr::DriverPose_t
	GetPose()
	{
		// monado predicts pose "now", see xrt_device_get_tracked_pose
		m_pose.poseTimeOffset = 0;

		m_pose.poseIsValid = true;
		m_pose.result = vr::TrackingResult_Running_OK;
		m_pose.deviceIsConnected = true;

		enum xrt_input_name grip_name;

		//! @todo better method to find grip name
		if (m_xdev->name == XRT_DEVICE_VIVE_WAND) {
			grip_name = XRT_INPUT_VIVE_GRIP_POSE;
		} else if (m_xdev->name == XRT_DEVICE_INDEX_CONTROLLER) {
			grip_name = XRT_INPUT_INDEX_GRIP_POSE;
		} else if (m_xdev->name == XRT_DEVICE_FLIPVR) {
			grip_name = XRT_INPUT_FLIPVR_GRIP_POSE;
		} else if (m_xdev->name == XRT_DEVICE_PSMV) {
			grip_name = XRT_INPUT_PSMV_GRIP_POSE;
		} else if (m_xdev->name == XRT_DEVICE_DAYDREAM) {
			grip_name = XRT_INPUT_DAYDREAM_POSE;
		} else if (m_xdev->name == XRT_DEVICE_HYDRA) {
			grip_name = XRT_INPUT_HYDRA_GRIP_POSE;
		} else if (m_xdev->name == XRT_DEVICE_TOUCH_CONTROLLER) {
			grip_name = XRT_INPUT_TOUCH_GRIP_POSE;
		} else if (m_xdev->name == XRT_DEVICE_SIMPLE_CONTROLLER) {
			grip_name = XRT_INPUT_SIMPLE_GRIP_POSE;
		} else if (m_xdev->name == XRT_DEVICE_PSSENSE) {
			grip_name = XRT_INPUT_PSSENSE_GRIP_POSE;
		} else if (m_xdev->name == XRT_DEVICE_HAND_CTRL_EMU) {
			grip_name = XRT_INPUT_HAND_CTRL_EMU_GRIP_POSE;
		} else {
			ovrd_log("Unhandled device name %u\n", m_xdev->name);
			grip_name = XRT_INPUT_GENERIC_HEAD_POSE; // ???
		}

		timepoint_ns now_ns = os_monotonic_get_ns();

		struct xrt_space_relation rel;
		xrt_device_get_tracked_pose(m_xdev, grip_name, now_ns, &rel);

		// Report what the device actually knows, rather than always "valid".
		m_pose.poseIsValid = (rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0;
		m_pose.result = (rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0
		                    ? vr::TrackingResult_Running_OK
		                    : vr::TrackingResult_Fallback_RotationOnly;

		struct xrt_pose *offset = &m_xdev->tracking_origin->initial_offset;

		struct xrt_relation_chain chain = {};
		m_relation_chain_push_relation(&chain, &rel);
		m_relation_chain_push_pose_if_not_identity(&chain, offset);
		m_relation_chain_resolve(&chain, &rel);

		apply_pose(&rel, &m_pose);

#ifdef DUMP_POSE_CONTROLLERS
		ovrd_log("get controller %d pose %f %f %f %f, %f %f %f\n", m_unObjectId, m_pose.qRotation.x,
		         m_pose.qRotation.y, m_pose.qRotation.z, m_pose.qRotation.w, m_pose.vecPosition[0],
		         m_pose.vecPosition[1], m_pose.vecPosition[2]);
#endif
		vr::HmdQuaternion_t identityquat{1, 0, 0, 0};
		g_room.apply(m_pose); // the room transform, the same for the headset and both controllers
		m_pose.qDriverFromHeadRotation = identityquat;
		m_pose.vecDriverFromHeadTranslation[0] = 0.f;
		m_pose.vecDriverFromHeadTranslation[1] = 0.f;
		m_pose.vecDriverFromHeadTranslation[2] = 0.f;

		return m_pose;
	}

	/*!
	 * Room capture (left Sense controller): Create held with the controller lying still on the floor. A short
	 * buzz (0.25 s) says it worked, a long one (0.9 s) that it didn't (the reason is in the log).
	 */
	void
	RoomCaptureFrame()
	{
#ifdef XRT_BUILD_DRIVER_PSSENSE
		if (!g_room.enabled() || m_hand != XRT_HAND_LEFT || m_xdev->name != XRT_DEVICE_PSSENSE) {
			return;
		}
		if (m_outline_count == 0) {
			m_outline_count = pssense_get_grip_outline(m_xdev, m_outline, ARRAY_SIZE(m_outline), &m_outline_radius);
		}
		bool create = false;
		for (uint32_t i = 0; i < m_xdev->input_count; i++) {
			if (m_xdev->inputs[i].name == XRT_INPUT_PSSENSE_SHARE_CLICK) {
				create = m_xdev->inputs[i].value.boolean;
				break;
			}
		}
		timepoint_ns now_ns = os_monotonic_get_ns();
		struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
		if (create) {
			xrt_device_get_tracked_pose(m_xdev, XRT_INPUT_PSSENSE_GRIP_POSE, now_ns, &rel);
			struct xrt_relation_chain chain = {};
			m_relation_chain_push_relation(&chain, &rel);
			m_relation_chain_push_pose_if_not_identity(&chain, &m_xdev->tracking_origin->initial_offset);
			m_relation_chain_resolve(&chain, &rel);
		}
		OvrdRoom::CaptureResult result =
		    g_room.controller_frame(create, rel, m_outline, m_outline_count, m_outline_radius, now_ns);
		if (result != OvrdRoom::CaptureResult::NONE && !m_output_controls.empty()) {
			struct xrt_output_value out = {};
			out.type = XRT_OUTPUT_VALUE_TYPE_VIBRATION;
			out.vibration.amplitude = 0.6f;
			out.vibration.frequency = XRT_FREQUENCY_UNSPECIFIED;
			out.vibration.duration_ns = 250 * U_TIME_1MS_IN_NS;
			if (result == OvrdRoom::CaptureResult::REJECTED) {
				// Three short pulses would need a timer; one long one is distinct enough from the short success buzz.
				out.vibration.duration_ns = 900 * U_TIME_1MS_IN_NS;
			}
			m_xdev->set_output(m_xdev, m_output_controls.at(0).monado_output_name, &out);
		}
#endif
	}

	void
	RunFrame()
	{
		m_xdev->update_inputs(m_xdev);

		RoomCaptureFrame();


		for (const auto &in : m_input_controls) {

			// ovrd_log("Update %d: %s\n", i,
			// m_controls[i].steamvr_control_path);

			enum xrt_input_name binding_name = in.monado_input_name;

			struct xrt_input *input = NULL;
			for (uint32_t ii = 0; ii < m_xdev->input_count; ii++) {
				if (m_xdev->inputs[ii].name == binding_name) {
					input = &m_xdev->inputs[ii];
					break;
				}
			}

			if (input == NULL) {
				ovrd_log("Input for %s not found!\n", in.steamvr_control_path);
				continue;
			}

			vr::VRInputComponentHandle_t handle = in.control_handle;

			if (in.bool_as_scalar) {
				vr::VRDriverInput()->UpdateScalarComponent(handle, input->value.boolean ? 1.0f : 0.0f, 0);
				continue;
			}
			if (in.monado_input_type == XRT_INPUT_TYPE_BOOLEAN) {
				bool state = input->value.boolean;
				vr::VRDriverInput()->UpdateBooleanComponent(handle, state, 0);
				// ovrd_log("Update %s: %d\n",
				// m_controls[i].steamvr_control_path, state);
				// U_LOG_D("Update %s: %d",
				//       m_controls[i].steamvr_control_path,
				//       state);
			}
			if (in.monado_input_type == XRT_INPUT_TYPE_VEC1_MINUS_ONE_TO_ONE ||
			    in.monado_input_type == XRT_INPUT_TYPE_VEC1_ZERO_TO_ONE ||
			    in.monado_input_type == XRT_INPUT_TYPE_VEC2_MINUS_ONE_TO_ONE) {

				float value;
				if (in.component.has_component && in.component.x) {
					value = input->value.vec2.x;
				} else if (in.component.has_component && in.component.y) {
					value = input->value.vec2.y;
				} else {
					value = input->value.vec1.x;
				}

				vr::VRDriverInput()->UpdateScalarComponent(handle, value, 0);
				// ovrd_log("Update %s: %f\n",
				// m_controls[i].steamvr_control_path,
				// state->x);
				// U_LOG_D("Update %s: %f",
				//       m_controls[i].steamvr_control_path,
				//       value);
			}
		}

		if (m_xdev->supported.hand_tracking && m_skeletal_input_control.control_handle) {
			constexpr const std::size_t sk_comp_size = 2;

			using ht_input_name_list = std::array<const enum xrt_input_name, sk_comp_size>;
			constexpr const std::array<const ht_input_name_list, sk_comp_size> ht_input_name_map = {
			    ht_input_name_list{
			        XRT_INPUT_HT_UNOBSTRUCTED_LEFT,
			        XRT_INPUT_HT_CONFORMING_LEFT,
			    },
			    ht_input_name_list{
			        XRT_INPUT_HT_UNOBSTRUCTED_RIGHT,
			        XRT_INPUT_HT_CONFORMING_RIGHT,
			    },
			};
			using bone_transform_list = std::array<vr::VRBoneTransform_t, OPENVR_BONE_COUNT>;
			using bone_transforms_list = std::vector<bone_transform_list>;

			bone_transforms_list bone_transforms;
			bone_transforms.reserve(sk_comp_size);
			const auto &ht_input_names = ht_input_name_map[m_hand];
			const timepoint_ns now_ns = os_monotonic_get_ns();

			// The emulated hand controllers only have the unobstructed skeleton; asking for the conforming one
			// logged an error per frame per hand (8800 lines in a 40 s run). The other component reuses it below.
			const size_t ht_input_count = m_xdev->name == XRT_DEVICE_HAND_CTRL_EMU ? 1 : ht_input_names.size();
			for (size_t i = 0; i < ht_input_count; ++i) {
				int64_t out_timestamp_ns;
				struct xrt_hand_joint_set joint_set_value = {};
				if (m_xdev->get_hand_tracking(m_xdev, ht_input_names[i], now_ns, &joint_set_value,
				                              &out_timestamp_ns) != XRT_SUCCESS) {
					continue;
				}
				auto &bts = bone_transforms.emplace_back();
				hand_joint_set_to_bone_transform(joint_set_value, bts.data(), m_hand);
				// hand_joint_set_to_bone_transforms(out_joint_set_value, bone_transforms);
			}

			if (bone_transforms.empty())
				return;
			if (bone_transforms.size() < sk_comp_size) {
				bone_transforms.push_back(bone_transforms.back());
			}
			assert(bone_transforms.size() >= sk_comp_size);

			vr::EVRInputError err = vr::VRDriverInput()->UpdateSkeletonComponent(
			    m_skeletal_input_control.control_handle, vr::VRSkeletalMotionRange_WithoutController,
			    bone_transforms[0].data(), OPENVR_BONE_COUNT);
			if (err != vr::VRInputError_None) {
				ovrd_log("error updating skeleton: %i ", err);
			}

			err = vr::VRDriverInput()->UpdateSkeletonComponent(
			    m_skeletal_input_control.control_handle, vr::VRSkeletalMotionRange_WithController,
			    bone_transforms[1].data(), OPENVR_BONE_COUNT);
			if (err != vr::VRInputError_None) {
				ovrd_log("error updating skeleton: %i ", err);
			}
		}
	}


	vr::VRControllerState_t
	GetControllerState()
	{
		// deprecated API
		vr::VRControllerState_t controllerstate{};
		return controllerstate;
	}

	bool
	TriggerHapticPulse(uint32_t unAxisId, uint16_t usPulseDurationMicroseconds)
	{
		// deprecated API
		return false;
	}

	std::string
	GetSerialNumber() const
	{
		ovrd_log("get controller serial number: %s\n", m_sSerialNumber);
		return m_sSerialNumber;
	}


	struct xrt_device *m_xdev;
	vr::DriverPose_t m_pose = {};

	//! The controller's outline in its grip frame, for the room capture's floor measurement.
	struct xrt_vec3 m_outline[64] = {};
	size_t m_outline_count = 0;
	float m_outline_radius = 0;
	vr::TrackedDeviceIndex_t m_unObjectId;
	vr::PropertyContainerHandle_t m_ulPropertyContainer;

	bool m_emulate_index_controller = false;

	std::vector<struct SteamVRDriverControlInput> m_input_controls;
	struct SteamVRDriverControlInput m_skeletal_input_control;

	std::vector<struct SteamVRDriverControlOutput> m_output_controls;

private:
	char m_sSerialNumber[XRT_DEVICE_NAME_LEN];
	char m_sModelNumber[MODELNUM_LEN];

	const char *m_controller_type = NULL;

	const char *m_render_model = NULL;
	enum xrt_hand m_hand;
	bool m_handed_controller;

	std::string m_input_profile;

	bool m_poseUpdating = true;
	std::thread *m_poseUpdateThread = NULL;
};

/*
 *
 * Device driver
 *
 */

class CDeviceDriver_Monado : public vr::ITrackedDeviceServerDriver, public vr::IVRDisplayComponent
{
public:
	CDeviceDriver_Monado(struct xrt_instance *xinst, struct xrt_device *xdev) : m_xdev(xdev)
	{
		/*
		 * PS VR2: Sony's driver reports 0.00136 s for this panel (read through SteamVR with psvr2-test's vrposelog,
		 * 2026-11-16). With the previous guess of 0.011 SteamVR extrapolated 22 ms ahead instead of Sony's 12.4 ms,
		 * which in replay nearly doubled the visible jitter of fast head turns (0.79 vs 0.48 deg/frame p90).
		 */
		float vsync_to_photons = debug_get_float_option_ovrd_vsync_to_photons_s();
		if (vsync_to_photons < 0.0f) {
			vsync_to_photons = m_xdev->name == XRT_DEVICE_PSVR2 ? 0.00136f : 0.011f; //! @todo others: measure
		}
		m_flSecondsFromVsyncToPhotons = vsync_to_photons;

		/*
		 * PS VR2: Sony's driver reports 0.0103 s ("playstation_vr2: VsyncToPhotons: 0.001360, PhotonsToVblank:
		 * 0.010300" in vrserver.txt, 2026-11-17); we didn't report it at all, the only display timing value where we
		 * differed while our view shook in slow head motion and Sony's didn't (same SteamVR, GPU, mode, async off).
		 */
		float photons_to_vblank = debug_get_float_option_ovrd_photons_to_vblank_s();
		if (photons_to_vblank < 0.0f) {
			photons_to_vblank = m_xdev->name == XRT_DEVICE_PSVR2 ? 0.0103f : 0.0f;
		}
		m_flSecondsFromPhotonsToVblank = photons_to_vblank;

		float ns = (float)m_xdev->hmd->screens->nominal_frame_interval_ns;
		m_flDisplayFrequency = 1.f / ns * 1000.f * 1000.f * 1000.f;
		ovrd_log("display frequency from device: %f\n", m_flDisplayFrequency);

		// steamvr can really misbehave when freq is inf or so
		if (m_flDisplayFrequency < 0 || m_flDisplayFrequency > 1000) {
			ovrd_log("Setting display frequency to 60 Hz!\n");
			m_flDisplayFrequency = 60.f;
		}

		//! @todo get ipd user setting from monado session
		float ipd_meters = 0.063f;
		struct xrt_vec3 ipd_vec = {ipd_meters, 0, 0};

		timepoint_ns now_ns = os_monotonic_get_ns();

		//! @todo more than 2 views
		struct xrt_space_relation head_relation;
		xrt_device_get_view_poses(xdev, &ipd_vec, now_ns, XRT_VIEW_TYPE_STEREO, 2, &head_relation, m_fovs,
		                          m_view_pose);

		//! @todo more versatile IPD calculation
		float actual_ipd = -m_view_pose[0].position.x + m_view_pose[1].position.x;

		m_flIPD = actual_ipd;

		ovrd_log("Seconds from Vsync to Photons: %f\n", m_flSecondsFromVsyncToPhotons);
		ovrd_log("Seconds from Photons to Vblank: %f%s\n", m_flSecondsFromPhotonsToVblank,
		         m_flSecondsFromPhotonsToVblank > 0.0f ? "" : " (not reported)");
		ovrd_log("Display Frequency: %f\n", m_flDisplayFrequency);
		ovrd_log("IPD: %f\n", m_flIPD);
	};
	// clang-format off
	virtual ~CDeviceDriver_Monado() {};

	// ITrackedDeviceServerDriver
	virtual vr::EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId);
	virtual void Deactivate();
	virtual void EnterStandby();
	virtual void *GetComponent(const char *pchComponentNameAndVersion);
	virtual void DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize);
	virtual vr::DriverPose_t GetPose();
	vr::DriverPose_t GetPoseAt(timepoint_ns at_ns, bool zero_velocity, double time_offset_s);
	OvrdPoseReplay m_replay;       //!< OVRD_POSE_REPLAY
	int64_t m_replayStartNs = 0;

	// IVRDisplayComponent
	virtual void GetWindowBounds(int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight);
	virtual bool IsDisplayOnDesktop();
	virtual bool IsDisplayRealDisplay();
	virtual void GetRecommendedRenderTargetSize(uint32_t *pnWidth, uint32_t *pnHeight);
	virtual void GetEyeOutputViewport(vr::EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight);
	virtual void GetProjectionRaw(vr::EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom);
	virtual vr::DistortionCoordinates_t ComputeDistortion(vr::EVREye eEye, float fU, float fV);
	void SetHiddenAreaMeshes();
	bool LoadHiddenAreaFile();
	void UpdateEyeToHead(bool force);
	virtual bool ComputeInverseDistortion(vr::HmdVector2_t *pResult, vr::EVREye eEye, uint32_t unChannel, float fU, float fV);

	// clang-format on

private:
	struct xrt_device *m_xdev = NULL;

	// clang-format off

	vr::TrackedDeviceIndex_t m_trackedDeviceIndex = 0;
	vr::PropertyContainerHandle_t m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

	float m_flSecondsFromVsyncToPhotons = -1;
	float m_flSecondsFromPhotonsToVblank = 0;
	float m_flDisplayFrequency = -1;
	float m_flIPD = -1;

	struct xrt_fov m_fovs[2];
	struct xrt_pose m_view_pose[2];

	bool m_poseUpdating = true;
	std::thread *m_poseUpdateThread = NULL;
	virtual void PoseUpdateThreadFunction();
	OvrdPoseLog m_poseLog;          //!< OVRD_POSE_LOG
	bool m_directModeOn = false;    //!< OVRD_DIRECT_MODE
	OvrdDirectMode m_directMode;
public:
	//! Monado's compositor for direct mode (created by the server driver), before Activate.
	struct xrt_compositor_native *m_directXcn = NULL;
private:
	std::atomic<bool> m_lastPosTracked{false}; //!< position tracked flag of the last GetPose (pose log; 2 threads)
	int64_t m_eyeCheckNs = 0;      //!< last IPD check (pose thread)

	// clang-format on
};

static void
create_translation_rotation_matrix(struct xrt_pose *pose, struct vr::HmdMatrix34_t *res)
{
	struct xrt_vec3 t = pose->position;
	struct xrt_quat r = pose->orientation;
	res->m[0][0] = (1.0f - 2.0f * (r.y * r.y + r.z * r.z));
	res->m[1][0] = (r.x * r.y + r.z * r.w) * 2.0f;
	res->m[2][0] = (r.x * r.z - r.y * r.w) * 2.0f;
	res->m[0][1] = (r.x * r.y - r.z * r.w) * 2.0f;
	res->m[1][1] = (1.0f - 2.0f * (r.x * r.x + r.z * r.z));
	res->m[2][1] = (r.y * r.z + r.x * r.w) * 2.0f;
	res->m[0][2] = (r.x * r.z + r.y * r.w) * 2.0f;
	res->m[1][2] = (r.y * r.z - r.x * r.w) * 2.0f;
	res->m[2][2] = (1.0f - 2.0f * (r.x * r.x + r.y * r.y));
	res->m[0][3] = t.x;
	res->m[1][3] = t.y;
	res->m[2][3] = t.z;
}

void
CDeviceDriver_Monado::PoseUpdateThreadFunction()
{
	ovrd_log("Starting HMD pose update thread\n");

	const char *log_path = debug_get_option_ovrd_pose_log();
	if (log_path != NULL && log_path[0] != '\0') {
		if (m_poseLog.open(log_path)) {
			ovrd_log("Recording submitted headset poses to %s\n", log_path);
		} else {
			ovrd_log("Can't write the pose log %s\n", log_path);
		}
	}

	const char *mode = debug_get_option_ovrd_pose_mode();
	if (mode == NULL) {
		mode = "steamvr";
	}
	bool display_mode = strcmp(mode, "display") == 0;
	bool measured_mode = strcmp(mode, "measured") == 0;
	bool novel_mode = strcmp(mode, "novelocity") == 0;
	if (!display_mode && !measured_mode && !novel_mode && strcmp(mode, "steamvr") != 0) {
		ovrd_log("Unknown OVRD_POSE_MODE '%s', using steamvr\n", mode);
		mode = "steamvr";
	}
	int64_t age_ns = (int64_t)(debug_get_float_option_ovrd_pose_age_ms() * 1e6);
	int avg_n = (int)debug_get_num_option_ovrd_angvel_avg_ms();
	if (avg_n < 0) avg_n = 0;
	if (avg_n > 50) avg_n = 50;
	double avg_ring[50][3] = {};
	int avg_pos = 0, avg_count = 0;
	if (avg_n > 0) {
		ovrd_log("Angular velocity sent: mean of the last %d ms (OVRD_ANGVEL_AVG_MS)\n", avg_n);
	}
	const char *replay = debug_get_option_ovrd_pose_replay();
	if (replay != NULL && replay[0] != '\0') {
		if (m_replay.load(replay)) {
			m_replayStartNs = (int64_t)os_monotonic_get_ns();
			ovrd_log("Replaying %zu recorded headset poses (%.1f s, looped) from %s instead of the headset\n",
			         m_replay.size(), m_replay.duration_s(), replay);
		} else {
			ovrd_log("Can't use OVRD_POSE_REPLAY %s (missing or too short), using the headset\n", replay);
		}
	}
	int64_t cut_ns = (int64_t)(debug_get_float_option_ovrd_display_cut_ms() * 1e6);
	int64_t after_ns = (int64_t)(debug_get_float_option_ovrd_display_after_vsync_ms() * 1e6);
	OvrdDisplayClock clock((int64_t)(1e9 / m_flDisplayFrequency));
	bool announced = false;
	int64_t start_ns = (int64_t)os_monotonic_get_ns();
	int64_t last_clock_log_ns = 0;
	ovrd_log("Headset pose mode: %s%s%s\n", mode,
	         m_directModeOn && m_directMode.hasOutput() && debug_get_bool_option_ovrd_direct_mode_pose()
	             ? " (until direct mode's first frame)"
	             : "",
	         display_mode    ? " (predicted to display time, no velocities)"
	         : measured_mode ? " (pose at its measurement time, negative poseTimeOffset, velocities)"
	         : novel_mode    ? " (now, no velocities)"
	                         : " (now + velocities, SteamVR predicts)");

	const bool direct_pose = m_directModeOn && m_directMode.hasOutput() && debug_get_bool_option_ovrd_direct_mode_pose();
	if (direct_pose) {
		ovrd_log("Headset pose: Monado's prediction for each frame's display time (direct mode, zero velocity); "
		         "this replaces OVRD_POSE_MODE\n");
		if (strcmp(mode, "steamvr") != 0) {
			ovrd_log("OVRD_POSE_MODE=%s is ignored in direct mode (OVRD_DIRECT_MODE_POSE=0 to use it)\n", mode);
		}
	}
	while (m_poseUpdating) {
		//! @todo figure out the best pose update rate
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		int64_t t0 = (int64_t)os_monotonic_get_ns();
		int64_t target_ns = t0;
		bool use_display = false;
		if (display_mode) {
			vr::Compositor_FrameTiming ft = {};
			ft.m_nSize = sizeof(ft);
			if (vr::VRServerDriverHost()->GetFrameTimings(&ft, 1) == 1) {
				clock.observe(t0, ft.m_flSystemTimeInSeconds, ft.m_nFrameIndex);
			}
			if (!clock.ready() && !announced && t0 - start_ns > 10 * (int64_t)U_TIME_1S_IN_NS) {
				ovrd_log("Display clock: no compositor frame timings after 10 s, poses for now + velocities\n");
				announced = true;
			}
			if (clock.ready()) {
				target_ns = clock.display_time_ns(t0, cut_ns, after_ns);
				use_display = true;
				if (!announced || t0 - last_clock_log_ns > 10 * (int64_t)U_TIME_1S_IN_NS) {
					ovrd_log("Display clock: SteamVR time + %.4f s = monotonic; poses %.1f ms ahead\n",
					         clock.offset_s(), (target_ns - t0) / 1e6);
					announced = true;
					last_clock_log_ns = t0;
				}
			}
		}
		// Direct mode with Monado's compositor: the pose for the display time of the frame being rendered.
		if (direct_pose) {
			const int64_t at = m_directMode.displayTarget();
			if (at > t0 - 30 * (int64_t)U_TIME_1MS_IN_NS && at < t0 + 100 * (int64_t)U_TIME_1MS_IN_NS) {
				target_ns = at;
				use_display = true;
			}
		}
		vr::DriverPose_t pose;
		if (use_display) {
			pose = GetPoseAt(target_ns, true, 0.0);
		} else if (measured_mode) {
			target_ns = t0 - age_ns;
			pose = GetPoseAt(target_ns, false, -(double)age_ns * 1e-9);
		} else {
			// steamvr, novelocity, and display until its clock is known (no frames yet): the pose for now.
			pose = GetPoseAt(t0, novel_mode, 0.0);
		}
		if (avg_n > 0 && !use_display && !novel_mode) {
			// Mean of the last avg_n angular velocities (one per ~1 ms pose update), head-local as sent.
			for (int i = 0; i < 3; i++) {
				avg_ring[avg_pos][i] = pose.vecAngularVelocity[i];
			}
			avg_pos = (avg_pos + 1) % avg_n;
			if (avg_count < avg_n) {
				avg_count++;
			}
			for (int i = 0; i < 3; i++) {
				double sum = 0;
				for (int k = 0; k < avg_count; k++) {
					sum += avg_ring[k][i];
				}
				pose.vecAngularVelocity[i] = sum / avg_count;
			}
		}
		int64_t t1 = (int64_t)os_monotonic_get_ns();
		vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_trackedDeviceIndex, pose, sizeof(vr::DriverPose_t));
		if (t1 - m_eyeCheckNs > 500 * (int64_t)U_TIME_1MS_IN_NS) {
			m_eyeCheckNs = t1;
			UpdateEyeToHead(false); // the headset's IPD dial
		}
		if (m_poseLog.is_open()) {
			OvrdPoseLog::Record r = {};
			r.submit_ns = t1;
			r.target_ns = target_ns;
			r.get_pose_ns = t1 - t0;
			r.q[0] = (float)pose.qRotation.x, r.q[1] = (float)pose.qRotation.y;
			r.q[2] = (float)pose.qRotation.z, r.q[3] = (float)pose.qRotation.w;
			for (int i = 0; i < 3; i++) {
				r.p[i] = (float)pose.vecPosition[i];
				r.v[i] = (float)pose.vecVelocity[i];
				r.w[i] = (float)pose.vecAngularVelocity[i];
			}
			if (use_display || measured_mode) {
				vr::DriverPose_t now_pose = GetPoseAt(t0, false, 0.0);
				r.nq[0] = (float)now_pose.qRotation.x, r.nq[1] = (float)now_pose.qRotation.y;
				r.nq[2] = (float)now_pose.qRotation.z, r.nq[3] = (float)now_pose.qRotation.w;
				for (int i = 0; i < 3; i++) r.np[i] = (float)now_pose.vecPosition[i];
			} else {
				for (int i = 0; i < 4; i++) r.nq[i] = r.q[i];
				for (int i = 0; i < 3; i++) r.np[i] = r.p[i];
			}
			r.valid = pose.poseIsValid;
			r.result = (uint8_t)pose.result;
			r.pos_tracked = m_lastPosTracked;
			m_poseLog.push(r);
		}
	}
	m_poseLog.close();
	ovrd_log("Stopping HMD pose update thread\n");
}

vr::EVRInitError
CDeviceDriver_Monado::Activate(vr::TrackedDeviceIndex_t unObjectId)
{
	ovrd_log("Activate tracked device %u: %s\n", unObjectId, m_xdev->str);

	m_trackedDeviceIndex = unObjectId;

	// clang-format off

	m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);
	//! @todo: proper serial and model number

	vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_HMD);
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_TrackingSystemName_String, "monado");
	if (m_xdev->name == XRT_DEVICE_PSVR2) {
		// As Sony's driver presents it, so SteamVR recognises the headset as a PS VR2.
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "Sony Interactive Entertainment");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "PlayStation VR2");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "playstation_vr2_hmd");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_InputProfilePath_String, "{monado}/input/playstation_vr2_profile.json");
		if (debug_get_bool_option_ovrd_psvr2_sony_icons()) {
			const struct { vr::ETrackedDeviceProperty prop; const char *path; } icons[] = {
			    {vr::Prop_NamedIconPathDeviceOff_String, "{monado}/icons/playstation_vr2/headset_status_off@2x.png"},
			    {vr::Prop_NamedIconPathDeviceSearching_String, "{monado}/icons/playstation_vr2/headset_status_searching@2x.gif"},
			    {vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{monado}/icons/playstation_vr2/headset_status_searching@2x.gif"},
			    {vr::Prop_NamedIconPathDeviceReady_String, "{monado}/icons/playstation_vr2/headset_status_ready@2x.png"},
			    {vr::Prop_NamedIconPathDeviceReadyAlert_String, "{monado}/icons/playstation_vr2/headset_status_ready@2x.png"},
			    {vr::Prop_NamedIconPathDeviceNotReady_String, "{monado}/icons/playstation_vr2/headset_status_error@2x.png"},
			    {vr::Prop_NamedIconPathDeviceStandby_String, "{monado}/icons/playstation_vr2/headset_status_standby@2x.png"},
			};
			for (const auto &ic : icons) {
				vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, ic.prop, ic.path);
			}
		}
	} else {
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "Monado");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, m_xdev->str);
	}
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_UserIpdMeters_Float, m_flIPD);
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.f);
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_DisplayFrequency_Float, m_flDisplayFrequency);
	vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, vr::Prop_SecondsFromVsyncToPhotons_Float, m_flSecondsFromVsyncToPhotons);
	if (m_flSecondsFromPhotonsToVblank > 0.0f) {
		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_SecondsFromPhotonsToVblank_Float,
		                                     m_flSecondsFromPhotonsToVblank);
	}
	if (!LoadHiddenAreaFile() && debug_get_bool_option_ovrd_hidden_area()) {
		SetHiddenAreaMeshes();
	}
	{
		int mesh_res = (int)debug_get_num_option_ovrd_distortion_mesh_res();
		if (mesh_res > 0) {
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DistortionMeshResolution_Int32,
			                                     mesh_res);
			ovrd_log("Distortion mesh resolution: %d\n", mesh_res);
		}
	}

	// Universe 2 (the saved room) or 3 (this session only), see OvrdRoom; never 0 (invalid) or 1 (Oculus)
	vr::VRProperties()->SetUint64Property( m_ulPropertyContainer, vr::Prop_CurrentUniverseId_Uint64, g_room.universe());
	g_room.add_container(m_ulPropertyContainer);
	// clang-format on

	// Our poses are already floor-relative (the tracking origin's offset is applied), so the standing origin is
	// the raw tracking origin. Seated apps reset their own zero pose.
	// With the room calibration on, the play area is in SteamVR's chaperone file instead (OvrdRoom::init): a
	// driver-provided chaperone stops SteamVR from saving the room drawn in Room Setup.
	float play_area = debug_get_float_option_ovrd_play_area_size();
	if (play_area > 0.0f && !g_room.enabled()) {
		float h = play_area / 2.0f;
		char chaperone[2048];
		snprintf(chaperone, sizeof(chaperone),
		         "{\"jsonid\":\"chaperone_info\",\"version\":5,\"universes\":[{\"universeID\":\"2\","
		         "\"play_area\":[%.3f,%.3f],"
		         "\"collision_bounds\":["
		         "[[-%.3f,0,-%.3f],[-%.3f,2.43,-%.3f],[-%.3f,2.43,%.3f],[-%.3f,0,%.3f]],"
		         "[[-%.3f,0,%.3f],[-%.3f,2.43,%.3f],[%.3f,2.43,%.3f],[%.3f,0,%.3f]],"
		         "[[%.3f,0,%.3f],[%.3f,2.43,%.3f],[%.3f,2.43,-%.3f],[%.3f,0,-%.3f]],"
		         "[[%.3f,0,-%.3f],[%.3f,2.43,-%.3f],[-%.3f,2.43,-%.3f],[-%.3f,0,-%.3f]]],"
		         "\"seated\":{\"translation\":[0,0,0],\"yaw\":0},"
		         "\"standing\":{\"translation\":[0,0,0],\"yaw\":0}}]}",
		         play_area, play_area,                    //
		         h, h, h, h, h, h, h, h,                  //
		         h, h, h, h, h, h, h, h,                  //
		         h, h, h, h, h, h, h, h,                  //
		         h, h, h, h, h, h, h, h);                 //
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_DriverProvidedChaperoneJson_String,
		                                      chaperone);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer,
		                                    vr::Prop_DriverProvidedChaperoneVisibility_Bool, true);
		ovrd_log("Providing a %.1fm x %.1fm play area for universe 2, skipping room setup\n", play_area,
		         play_area);
	}
	// clang-format off

	// avoid "not fullscreen" warnings from vrmonitor
	//vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_IsOnDesktop_Bool, false);

	// clang-format on
	if (debug_get_bool_option_ovrd_direct_mode()) {
		// As ALVR: SteamVR's compositor renders into our images and hands them over, paced by our vsync events.
		m_directModeOn = true;
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDriverDirectModeComponent_Bool,
		                                    true);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer,
		                                    vr::Prop_DriverDirectModeSendsVsyncEvents_Bool, true);
		// SteamVR's world -> the device's space: undo the room transform, then the tracking origin offset (the
		// reverse of GetPoseAt).
		struct xrt_device *xdev = m_xdev;
		m_directMode.world_to_device = [xdev](const struct xrt_pose &world) {
			ovrd_room_transform T = g_room.transform();
			struct xrt_pose room = {T.rotation, T.translation};
			struct xrt_pose room_inv, offset_inv, driver, device;
			math_pose_invert(&room, &room_inv);
			math_pose_transform(&room_inv, &world, &driver);
			math_pose_invert(&xdev->tracking_origin->initial_offset, &offset_inv);
			math_pose_transform(&offset_inv, &driver, &device);
			return device;
		};
		m_directMode.check_head = m_xdev;
		if (debug_get_bool_option_ovrd_direct_mode_pose()) {
			// The pose for the new frame's display time, submitted before SteamVR wakes the game (see on_frame_start).
			m_directMode.on_frame_start = [this](int64_t display_ns) {
				if (m_trackedDeviceIndex == vr::k_unTrackedDeviceIndexInvalid) {
					return;
				}
				vr::DriverPose_t pose = GetPoseAt(display_ns, true, 0.0);
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_trackedDeviceIndex, pose,
				                                                   sizeof(vr::DriverPose_t));
			};
		}
		m_directMode.gpu_sync = debug_get_bool_option_ovrd_direct_mode_gpu_sync();
		m_directMode.capture_after_s = debug_get_float_option_ovrd_direct_mode_capture_s();
		if (m_directXcn != NULL) {
			OvrdDirectMode::Output out;
			out.xcn = m_directXcn;
			out.head = m_xdev;
			// SteamVR renders each eye with GetProjectionRaw's frustum and our eye-to-head poses.
			out.fov[0] = m_xdev->hmd->distortion.fov[0];
			out.fov[1] = m_xdev->hmd->distortion.fov[1];
			out.eye_in_head[0] = m_view_pose[0];
			out.eye_in_head[1] = m_view_pose[1];
			m_directMode.setOutput(out);
		}
		m_directMode.start(m_flDisplayFrequency, debug_get_option_ovrd_direct_mode_log());
	}
	// clang-format off

	// clang-format on


	UpdateEyeToHead(true);


	m_poseUpdateThread = new std::thread(&CDeviceDriver_Monado::PoseUpdateThreadFunction, this);
	if (!m_poseUpdateThread) {
		ovrd_log("Unable to create pose updated thread for %s\n", m_xdev->str);
		return vr::VRInitError_Driver_Failed;
	}

	return vr::VRInitError_None;
}

void
CDeviceDriver_Monado::Deactivate()
{
	m_poseUpdating = false;
	if (m_poseUpdateThread != NULL && m_poseUpdateThread->joinable()) {
		m_poseUpdateThread->join();
	}
	m_directMode.stop();
	ovrd_log("Deactivate\n");
}

void
CDeviceDriver_Monado::EnterStandby()
{
	ovrd_log("Enter Standby\n");
}

void *
CDeviceDriver_Monado::GetComponent(const char *pchComponentNameAndVersion)
{
	// clang-format off
	if (strcmp(pchComponentNameAndVersion, vr::IVRDisplayComponent_Version) == 0) {
		return (vr::IVRDisplayComponent *)this;
	}
	if (m_directModeOn && strcmp(pchComponentNameAndVersion, vr::IVRDriverDirectModeComponent_Version) == 0) {
		ovrd_log("Direct mode: SteamVR asked for the direct mode component\n");
		return (vr::IVRDriverDirectModeComponent *)&m_directMode;
	}
	// clang-format on

	return NULL;
}

void
CDeviceDriver_Monado::DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize)
{
	//! @todo
}

static constexpr inline vr::HmdQuaternion_t
HmdQuaternion_Init(double w, double x, double y, double z)
{
	return vr::HmdQuaternion_t{
	    .w = w,
	    .x = x,
	    .y = y,
	    .z = z,
	};
}

vr::DriverPose_t
CDeviceDriver_Monado::GetPose()
{
	return GetPoseAt(os_monotonic_get_ns(), false, 0.0);
}

/*!
 * The headset pose at @p at_ns, handed over with poseTimeOffset @p time_offset_s; @p zero_velocity leaves the
 * velocities out so SteamVR shows the pose as it is. From the replay (OVRD_POSE_REPLAY) instead of the headset when on.
 */
vr::DriverPose_t
CDeviceDriver_Monado::GetPoseAt(timepoint_ns at_ns, bool zero_velocity, double time_offset_s)
{
	timepoint_ns now_ns = os_monotonic_get_ns();
	if (m_replay.size() > 0) {
		OvrdPoseReplay::Sample r = m_replay.at(at_ns - m_replayStartNs);
		vr::DriverPose_t t = {};
		t.poseTimeOffset = time_offset_s;
		t.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
		t.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);
		t.qRotation = HmdQuaternion_Init(r.q[3], r.q[0], r.q[1], r.q[2]);
		for (int i = 0; i < 3; i++) {
			t.vecPosition[i] = r.p[i];
			t.vecVelocity[i] = zero_velocity ? 0 : r.v[i];
			t.vecAngularVelocity[i] = zero_velocity ? 0 : r.wl[i];
		}
		t.result = vr::TrackingResult_Running_OK;
		t.poseIsValid = true;
		t.deviceIsConnected = true;
		m_lastPosTracked = true;
		g_room.apply(t);
		return t;
	}
	struct xrt_space_relation rel;
	xrt_device_get_tracked_pose(m_xdev, XRT_INPUT_GENERIC_HEAD_POSE, at_ns, &rel);

	struct xrt_pose *offset = &m_xdev->tracking_origin->initial_offset;

	struct xrt_relation_chain chain = {};
	m_relation_chain_push_relation(&chain, &rel);
	m_relation_chain_push_pose_if_not_identity(&chain, offset);
	m_relation_chain_resolve(&chain, &rel);

	vr::DriverPose_t t = {
	    // monado predicts pose "now", see xrt_device_get_tracked_pose
	    .poseTimeOffset = 0,
	    .qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0),
	    .vecWorldFromDriverTranslation = {0, 0, 0},

	    .qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0),
	    .vecDriverFromHeadTranslation = {0, 0, 0},

	    .vecPosition = {0, 0, 0},
	    .vecVelocity = {0, 0, 0},
	    .vecAcceleration = {0, 0, 0},

	    .qRotation = HmdQuaternion_Init(1, 0, 0, 0),

	    .vecAngularVelocity = {0, 0, 0},
	    .vecAngularAcceleration = {0, 0, 0},

	    .result = vr::TrackingResult_Running_OK,
	    .poseIsValid = (rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0,
	    .willDriftInYaw = !m_xdev->supported.position_tracking,
	    //! @todo: Monado head model?
	    .shouldApplyHeadModel = !m_xdev->supported.position_tracking,
	    .deviceIsConnected = true,
	};
	t.poseTimeOffset = time_offset_s;
	apply_pose(&rel, &t);
	if (zero_velocity) {
		for (int i = 0; i < 3; i++) {
			t.vecVelocity[i] = 0;
			t.vecAngularVelocity[i] = 0;
		}
	}
	m_lastPosTracked = (rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0;
	if (m_lastPosTracked) {
		g_room.note_head(rel.pose.position, now_ns);
	}
	g_room.apply(t); // the room transform, the same for the headset and both controllers

#ifdef DUMP_POSE
	ovrd_log("get hmd pose %f %f %f %f, %f %f %f\n", t.qRotation.x, t.qRotation.y, t.qRotation.z, t.qRotation.w,
	         t.vecPosition[0], t.vecPosition[1], t.vecPosition[2]);
#endif

	//! @todo
	// copy_vec3(&rel.angular_velocity, t.vecAngularVelocity);
	// copy_vec3(&rel.angular_acceleration, t.vecAngularAcceleration);

	// ovrd_log("Vel: %f %f %f\n", t.vecAngularVelocity[0],
	// t.vecAngularVelocity[1], t.vecAngularVelocity[2]); ovrd_log("Accel:
	// %f %f %f\n", t.vecAngularAcceleration[0],
	// t.vecAngularAcceleration[1], t.vecAngularAcceleration[2]);

	return t;
}

/*!
 * Eye-to-head transforms from the device's view poses (IVRServerDriverHost::SetDisplayEyeToHead) and the IPD property.
 * At activation, and from the pose thread twice a second when the headset's IPD dial moved (> 0.5 mm): SteamVR only
 * knew the IPD at startup, while the OpenXR path takes the dial every frame (psvr2_hmd_get_view_poses).
 */
void
CDeviceDriver_Monado::UpdateEyeToHead(bool force)
{
	struct xrt_vec3 ipd_vec = {0.063f, 0, 0}; // the device replaces x with its dial
	struct xrt_space_relation head_relation;
	struct xrt_fov fovs[2];
	struct xrt_pose view_pose[2];
	xrt_device_get_view_poses(m_xdev, &ipd_vec, os_monotonic_get_ns(), XRT_VIEW_TYPE_STEREO, 2, &head_relation, fovs,
	                          view_pose);
	float ipd = view_pose[1].position.x - view_pose[0].position.x;
	if (!force && fabsf(ipd - m_flIPD) < 0.0005f) {
		return;
	}
	m_view_pose[0] = view_pose[0];
	m_view_pose[1] = view_pose[1];
	if (m_directModeOn) {
		m_directMode.setEyes(m_view_pose);
	}
	vr::HmdMatrix34_t left, right;
	create_translation_rotation_matrix(&m_view_pose[0], &left);
	create_translation_rotation_matrix(&m_view_pose[1], &right);
	vr::VRServerDriverHost()->SetDisplayEyeToHead(m_trackedDeviceIndex, left, right);
	if (!force) {
		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_UserIpdMeters_Float, ipd);
		ovrd_log("IPD dial: %.1f mm -> %.1f mm\n", m_flIPD * 1000.0f, ipd * 1000.0f);
	}
	m_flIPD = ipd;
}

//! A hidden-area mask from a file (OVRD_HIDDEN_AREA_FILE): true when one was found and handed to SteamVR.
bool
CDeviceDriver_Monado::LoadHiddenAreaFile()
{
	std::string path;
	const char *env = debug_get_option_ovrd_hidden_area_file();
	if (env != NULL && env[0] != '\0') {
		path = env;
	} else {
		const char *xdg = getenv("XDG_CONFIG_HOME");
		const char *home = getenv("HOME");
		if (xdg != NULL && xdg[0] != '\0') {
			path = std::string(xdg) + "/monado/psvr2-hidden-area.txt";
		} else if (home != NULL) {
			path = std::string(home) + "/.config/monado/psvr2-hidden-area.txt";
		}
		if (m_xdev->name != XRT_DEVICE_PSVR2) {
			return false; // the default file is PS VR2 only
		}
	}
	FILE *f = path.empty() ? NULL : fopen(path.c_str(), "r");
	if (f == NULL) {
		if (env != NULL && env[0] != '\0') {
			ovrd_log("Hidden area: can't read %s\n", path.c_str());
		}
		return false;
	}
	std::vector<vr::HmdVector2_t> verts[2][3];
	char line[256];
	while (fgets(line, sizeof line, f) != NULL) {
		int eye, type;
		float u, v;
		if (sscanf(line, "ham_vert %d %d %f %f", &eye, &type, &u, &v) == 4 && eye >= 0 && eye < 2 && type >= 0 &&
		    type < 3) {
			verts[eye][type].push_back({{u, v}});
		}
	}
	fclose(f);
	bool any = false;
	for (int e = 0; e < 2; e++) {
		if (verts[e][0].size() < 3 || verts[e][0].size() % 3 != 0) {
			continue;
		}
		double area = 0;
		for (size_t i = 0; i + 2 < verts[e][0].size(); i += 3) {
			const auto &A = verts[e][0][i].v, &B = verts[e][0][i + 1].v, &C = verts[e][0][i + 2].v;
			area += fabs((B[0] - A[0]) * (C[1] - A[1]) - (C[0] - A[0]) * (B[1] - A[1])) / 2;
		}
		for (int t = 0; t < 3; t++) {
			if (!verts[e][t].empty()) {
				vr::VRHiddenArea()->SetHiddenArea((vr::EVREye)e, (vr::EHiddenAreaMeshType)t, verts[e][t].data(),
				                                  (uint32_t)verts[e][t].size());
			}
		}
		ovrd_log("Hidden area mesh, eye %d, from %s: %zu triangles, %.1f%% of the render target hidden\n", e,
		         path.c_str(), verts[e][0].size() / 3, area * 100.0);
		any = true;
	}
	return any;
}

//! Hidden-area meshes from our own distortion, see ovrd_hidden_area.hpp.
void
CDeviceDriver_Monado::SetHiddenAreaMeshes()
{
	for (int e = 0; e < 2; e++) {
		vr::EVREye eye = (vr::EVREye)e;
		ovrd_hidden_area::Mesh mesh;
		bool ok = ovrd_hidden_area::build(
		    [&](float u, float v, float out[6]) {
			    vr::DistortionCoordinates_t d = ComputeDistortion(eye, u, v);
			    out[0] = d.rfRed[0], out[1] = d.rfRed[1], out[2] = d.rfGreen[0], out[3] = d.rfGreen[1];
			    out[4] = d.rfBlue[0], out[5] = d.rfBlue[1];
		    },
		    mesh);
		if (!ok) {
			ovrd_log("Hidden area: eye %d centre maps outside the render target, no mesh\n", e);
			continue;
		}
		std::vector<vr::HmdVector2_t> tris, loop;
		for (auto &p : mesh.triangles) tris.push_back({{p.x, p.y}});
		for (auto &p : mesh.loop) loop.push_back({{p.x, p.y}});
		vr::VRHiddenArea()->SetHiddenArea(eye, vr::k_eHiddenAreaMesh_Standard, tris.data(), (uint32_t)tris.size());
		vr::VRHiddenArea()->SetHiddenArea(eye, vr::k_eHiddenAreaMesh_LineLoop, loop.data(), (uint32_t)loop.size());
		ovrd_log("Hidden area mesh, eye %d: %zu triangles, %.1f%% of the render target hidden\n", e,
		         tris.size() / 3, mesh.hidden_fraction * 100.0);
	}
}

void
CDeviceDriver_Monado::GetWindowBounds(int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight)
{
	// offset in extended mode, e.g. to the right of a 1920x1080 monitor
	*pnX = 1920;
	*pnY = 0;

	*pnWidth = m_xdev->hmd->screens[0].w_pixels;
	*pnHeight = m_xdev->hmd->screens[0].h_pixels;
	;

	ovrd_log("Window Bounds: %dx%d\n", *pnWidth, *pnHeight);
}

bool
CDeviceDriver_Monado::IsDisplayOnDesktop()
{
	return false;
}

bool
CDeviceDriver_Monado::IsDisplayRealDisplay()
{
	// In direct mode the frames come to us; SteamVR must not open the headset's display itself (as ALVR).
	if (m_directModeOn) {
		return false;
	}
	return true;
}

void
CDeviceDriver_Monado::GetRecommendedRenderTargetSize(uint32_t *pnWidth, uint32_t *pnHeight)
{
	int scale = debug_get_num_option_scale_percentage();
	if (scale <= 0) {
		scale = m_xdev->name == XRT_DEVICE_PSVR2 ? 170 : 140;
	}
	float fscale = (float)scale / 100.f;



	/*
	 * Per eye: SteamVR takes this as each eye's render target (its log: "HMD driver recommended: WxH ... MP/sec" =
	 * W * H * 2 eyes * Hz). This used the whole panel (both eyes, 4000 px wide for the PS VR2), so every eye was rendered
	 * twice as wide as needed (5600x2856 per eye at 140%). The view's display size is the per-eye size the OpenXR path
	 * uses too (xrt_view::display, scaled by XRT_COMPOSITOR_SCALE_PERCENTAGE).
	 */
	*pnWidth = m_xdev->hmd->views[0].display.w_pixels * fscale;
	*pnHeight = m_xdev->hmd->views[0].display.h_pixels * fscale;

	ovrd_log("Render Target Size: %dx%d (%fx)\n", *pnWidth, *pnHeight, fscale);
}

void
CDeviceDriver_Monado::GetEyeOutputViewport(
    vr::EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight)
{
	*pnWidth = m_xdev->hmd->views[eEye].viewport.w_pixels;
	*pnHeight = m_xdev->hmd->views[eEye].viewport.h_pixels;

	*pnX = m_xdev->hmd->views[eEye].viewport.x_pixels;
	*pnY = m_xdev->hmd->views[eEye].viewport.y_pixels;

	ovrd_log("Output Viewport for eye %d: %dx%d offset %dx%d\n", eEye, *pnWidth, *pnHeight, *pnX, *pnY);
}

void
CDeviceDriver_Monado::GetProjectionRaw(vr::EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom)
{
	*pfLeft = tanf(m_xdev->hmd->distortion.fov[eEye].angle_left);
	*pfRight = tanf(m_xdev->hmd->distortion.fov[eEye].angle_right);
	*pfTop = tanf(-m_xdev->hmd->distortion.fov[eEye].angle_up);
	*pfBottom = tanf(-m_xdev->hmd->distortion.fov[eEye].angle_down);
	ovrd_log("Projection Raw: L%f R%f T%f B%f\n", *pfLeft, *pfRight, *pfTop, *pfBottom);
}

vr::DistortionCoordinates_t
CDeviceDriver_Monado::ComputeDistortion(vr::EVREye eEye, float fU, float fV)
{
	/** Used to return the post-distortion UVs for each color channel.
	 * UVs range from 0 to 1 with 0,0 in the upper left corner of the
	 * source render target. The 0,0 to 1,1 range covers a single eye. */

	struct xrt_vec2 *rot = m_xdev->hmd->views[eEye].rot.vecs;

	// multiply 2x2 rotation matrix with fU, fV scaled to [-1, 1]
	float U = rot[0].x * (fU * 2 - 1) + rot[0].y * (fV * 2 - 1);
	float V = rot[1].x * (fU * 2 - 1) + rot[1].y * (fV * 2 - 1);

	// scale U, V back to [0, 1]
	U = (U + 1) / 2;
	V = (V + 1) / 2;

	struct xrt_uv_triplet d;

	xrt_result_t xret = m_xdev->compute_distortion(m_xdev, eEye, U, V, &d);
	if (xret != XRT_SUCCESS) {
		ovrd_log("Failed to compute distortion for view %d at %f,%f! Got error %d\n", eEye, U, V, xret);

		vr::DistortionCoordinates_t coordinates;
		coordinates.rfBlue[0] = U;
		coordinates.rfBlue[1] = V;
		coordinates.rfGreen[0] = U;
		coordinates.rfGreen[1] = V;
		coordinates.rfRed[0] = U;
		coordinates.rfRed[1] = V;
		return coordinates;
	}

	vr::DistortionCoordinates_t coordinates;
	coordinates.rfRed[0] = d.r.x;
	coordinates.rfRed[1] = d.r.y;
	coordinates.rfGreen[0] = d.g.x;
	coordinates.rfGreen[1] = d.g.y;
	coordinates.rfBlue[0] = d.b.x;
	coordinates.rfBlue[1] = d.b.y;

	// ovrd_log("Computed distortion for view %d at %f,%f -> %f,%f | %f,%f |
	// %f,%f!\n", eEye, U, V, d.r.x, d.r.y, d.g.x, d.g.y, d.b.x, d.b.y);

	return coordinates;
}

bool
CDeviceDriver_Monado::ComputeInverseDistortion(
    vr::HmdVector2_t *pResult, vr::EVREye eEye, uint32_t unChannel, float fU, float fV)
{
	// Unsupported, so return false
	return false;
}


/*
 *
 * Device driver server
 *
 */

//! Durations in ns per part of RunFrame, for the 10 s log (measurement only).
struct OvrdRunFrameStats
{
	static constexpr int PARTS = 6;
	const char *names[PARTS] = {"interval", "left controller", "right controller", "room", "events", "total"};
	std::vector<int64_t> v[PARTS];
	void
	add(int part, int64_t ns)
	{
		v[part].push_back(ns);
	}
	void
	reset()
	{
		for (auto &x : v) {
			x.clear();
		}
	}
	void
	log()
	{
		char buf[1024];
		int n = snprintf(buf, sizeof buf, "RunFrame (10 s, %zu calls):", v[5].size());
		for (int p = 0; p < PARTS && n < (int)sizeof buf; p++) {
			auto x = v[p];
			if (x.empty()) {
				continue;
			}
			std::sort(x.begin(), x.end());
			n += snprintf(buf + n, sizeof buf - n, " %s median %.2f p99 %.2f max %.2f ms;", names[p],
			              x[x.size() / 2] / 1e6, x[(size_t)(0.99 * (x.size() - 1))] / 1e6, x.back() / 1e6);
		}
		ovrd_log("%s\n", buf);
	}
};

class CServerDriver_Monado : public vr::IServerTrackedDeviceProvider
{
public:
	CServerDriver_Monado() : m_MonadoDeviceDriver(NULL) {}

	// clang-format off
	virtual ~CServerDriver_Monado() {};
	virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext);
	virtual void Cleanup();
	virtual const char *const * GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual void RunFrame();
	OvrdRunFrameStats m_rf_stats;
	int64_t m_rf_last_ns = 0, m_rf_log_ns = 0;
	virtual bool ShouldBlockStandbyMode() { return false; }
	virtual void EnterStandby() {}
	virtual void LeaveStandby() {}
	virtual void HandleHapticEvent(vr::VREvent_t *event);
	// clang-format on

private:
	struct xrt_instance *m_xinst = NULL;
	struct xrt_system *m_xsys = NULL;
	struct xrt_system_devices *m_xsysd = NULL;
	struct xrt_space_overseer *m_xso = NULL;
	// Direct mode (OVRD_DIRECT_MODE): Monado's compositor inside vrserver, SteamVR's frames are its client.
	struct xrt_system_compositor *m_xsysc = NULL;
	struct xrt_session *m_xs = NULL;
	struct xrt_compositor_native *m_xcn = NULL;
	bool CreateDirectModeCompositor();
	void DestroyDirectModeCompositor();
	struct xrt_device *m_xhmd = NULL;

	CDeviceDriver_Monado *m_MonadoDeviceDriver = NULL;
	CDeviceDriver_Monado_Controller *m_left = NULL;
	CDeviceDriver_Monado_Controller *m_right = NULL;
};

CServerDriver_Monado g_serverDriverMonado;

#define NUM_XDEVS 16

vr::EVRInitError
CServerDriver_Monado::Init(vr::IVRDriverContext *pDriverContext)
{

	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
	ovrd_log_init(vr::VRDriverLog());

	ovrd_log("Initializing Monado driver\n");

	//! @todo instance initialization is difficult to replicate

	xrt_result_t xret;
	// Before any device is added, so SteamVR reads the prepared chaperone file.
	g_room.init(debug_get_float_option_ovrd_play_area_size());

	/*
	 * PS VR2 head position aided by the 2 kHz accelerometer by default under SteamVR (the OpenXR path keeps the
	 * driver's own default): replaying the 2026-11-18 09:14, 10:01 and 10:11 recordings through the real pose filter
	 * at SteamVR's 11 ms prediction, position jitter while turning p90 0.13-0.15 -> 0.08-0.10 mm, fast turns
	 * 0.29-0.60 -> 0.18-0.44 mm, prediction error p90 0.58-0.85 -> 0.11-0.15 mm, still unchanged; the user reported
	 * it "feels better" on 2026-11-17. PSVR2_ACCEL_FUSION=0 turns it off.
	 */
	setenv("PSVR2_ACCEL_FUSION", "1", 0);

	xret = xrt_instance_create(NULL, &m_xinst);
	if (xret != XRT_SUCCESS) {
		ovrd_log("Failed to create instance\n");
		return vr::VRInitError_Init_HmdNotFound;
	}

	// Direct mode: with Monado's system compositor (it takes the headset's display), else devices only.
	xret = xrt_instance_create_system(m_xinst, &m_xsys, &m_xsysd, &m_xso,
	                                  debug_get_bool_option_ovrd_direct_mode() ? &m_xsysc : NULL);
	if (xret < 0 && m_xsysc == NULL && debug_get_bool_option_ovrd_direct_mode()) {
		ovrd_log("Direct mode: no Monado compositor (%d), the frames are only logged\n", (int)xret);
		if (m_xsys != NULL) {
			xrt_space_overseer_destroy(&m_xso);
			xrt_system_devices_destroy(&m_xsysd);
			xrt_system_destroy(&m_xsys);
		}
		xret = xrt_instance_create_system(m_xinst, &m_xsys, &m_xsysd, &m_xso, NULL);
	}
	if (xret < 0) {
		ovrd_log("Failed to create system devices\n");
		xrt_instance_destroy(&m_xinst);
		return vr::VRInitError_Init_HmdNotFound;
	}
	if (m_xsysd->static_roles.head == NULL) {
		ovrd_log("Didn't get a HMD device!\n");
		xrt_space_overseer_destroy(&m_xso);
		xrt_system_devices_destroy(&m_xsysd);
		xrt_system_destroy(&m_xsys);
		xrt_instance_destroy(&m_xinst);
		return vr::VRInitError_Init_HmdNotFound;
	}

	m_xhmd = m_xsysd->static_roles.head;
	g_room.set_hmd(m_xhmd);

	ovrd_log("Selected HMD %s\n", m_xhmd->str);
	m_MonadoDeviceDriver = new CDeviceDriver_Monado(m_xinst, m_xhmd);
	if (m_xsysc != NULL && CreateDirectModeCompositor()) {
		m_MonadoDeviceDriver->m_directXcn = m_xcn;
	}
	//! @todo provide a serial number
	vr::VRServerDriverHost()->TrackedDeviceAdded(m_xhmd->str, vr::TrackedDeviceClass_HMD, m_MonadoDeviceDriver);

	xrt_system_roles system_roles = XRT_SYSTEM_ROLES_INIT;
	xrt_system_devices_get_roles(m_xsysd, &system_roles);

	struct xrt_device *left_xdev = nullptr;
	if (system_roles.left >= 0 && system_roles.left < XRT_SYSTEM_MAX_DEVICES) {
		left_xdev = m_xsysd->static_xdevs[system_roles.left];
	}
	struct xrt_device *right_xdev = nullptr;
	if (system_roles.right >= 0 && system_roles.right < XRT_SYSTEM_MAX_DEVICES) {
		right_xdev = m_xsysd->static_xdevs[system_roles.right];
	}

	// use steamvr room setup instead
	struct xrt_vec3 offset = {0, 0, 0};
	u_builder_setup_tracking_origins(m_xhmd, nullptr, left_xdev, right_xdev, nullptr, &offset);

	if (left_xdev) {
		m_left = new CDeviceDriver_Monado_Controller(m_xinst, left_xdev, XRT_HAND_LEFT);
		ovrd_log("Added left Controller: %s\n", left_xdev->str);
	}
	if (right_xdev) {
		m_right = new CDeviceDriver_Monado_Controller(m_xinst, right_xdev, XRT_HAND_RIGHT);
		ovrd_log("Added right Controller: %s\n", right_xdev->str);
	}

	return vr::VRInitError_None;
}

bool
CServerDriver_Monado::CreateDirectModeCompositor()
{
	// As an in-process OpenXR session does (oxr_session_create_native_compositor + xrBeginSession).
	struct xrt_session_info xsi = {};
	xrt_result_t xret = xrt_system_create_session(m_xsys, &xsi, &m_xs, &m_xcn);
	if (xret != XRT_SUCCESS || m_xcn == NULL) {
		ovrd_log("Direct mode: can't create a session on Monado's compositor (%d)\n", (int)xret);
		xrt_session_destroy(&m_xs);
		return false;
	}
	if (m_xsysc->xmcc != NULL) {
		xrt_syscomp_set_state(m_xsysc, &m_xcn->base, true, true, os_monotonic_get_ns());
		xrt_syscomp_set_z_order(m_xsysc, &m_xcn->base, 0);
	}
	struct xrt_begin_session_info begin = {};
	begin.view_type = XRT_VIEW_TYPE_STEREO;
	xret = xrt_comp_begin_session(&m_xcn->base, &begin);
	if (xret != XRT_SUCCESS) {
		ovrd_log("Direct mode: Monado's compositor won't begin the session (%d)\n", (int)xret);
		xrt_comp_native_destroy(&m_xcn);
		xrt_session_destroy(&m_xs);
		return false;
	}
	ovrd_log("Direct mode: Monado's compositor is running, SteamVR's frames go to it\n");
	return true;
}

void
CServerDriver_Monado::DestroyDirectModeCompositor()
{
	if (m_xcn != NULL) {
		xrt_comp_end_session(&m_xcn->base);
		xrt_comp_native_destroy(&m_xcn);
	}
	xrt_session_destroy(&m_xs);
	xrt_syscomp_destroy(&m_xsysc);
}

void
CServerDriver_Monado::Cleanup()
{
	if (m_MonadoDeviceDriver != NULL) {
		delete m_MonadoDeviceDriver;
		m_MonadoDeviceDriver = NULL;
	}
	DestroyDirectModeCompositor();

	xrt_space_overseer_destroy(&m_xso);
	xrt_system_devices_destroy(&m_xsysd);
	xrt_system_destroy(&m_xsys);
	m_xhmd = NULL;
	// Null when a controller wasn't connected: vrserver segfaulted here on shutdown (2026-11-17 23:42 and 23:43,
	// coredumpctl: CServerDriver_Monado::Cleanup, ovrd_driver.cpp:2725-2726).
	if (m_left != NULL) {
		m_left->m_xdev = NULL;
	}
	if (m_right != NULL) {
		m_right->m_xdev = NULL;
	}

	if (m_xinst) {
		xrt_instance_destroy(&m_xinst);
	}
}

void
CServerDriver_Monado::HandleHapticEvent(vr::VREvent_t *event)
{
	float freq = event->data.hapticVibration.fFrequency;
	float amp = event->data.hapticVibration.fAmplitude;
	float duration = event->data.hapticVibration.fDurationSeconds;

	ovrd_log("Haptic vibration %fs %fHz %famp\n", duration, freq, amp);

	CDeviceDriver_Monado_Controller *controller = NULL;

	if (m_left && m_left->m_ulPropertyContainer == event->data.hapticVibration.containerHandle) {
		controller = m_left;
		ovrd_log("Haptic vibration left\n");
	} else if (m_right && m_right->m_ulPropertyContainer == event->data.hapticVibration.containerHandle) {
		controller = m_right;
		ovrd_log("Haptic vibration right\n");
	} else {
		ovrd_log("Haptic vibration ignored\n");
		return;
	}

	// Zero-init and set the type: it was left uninitialized, so drivers that check it (pssense) rejected the
	// request as an unsupported output and haptics never played.
	struct xrt_output_value out = {};
	out.type = XRT_OUTPUT_VALUE_TYPE_VIBRATION;
	out.vibration.amplitude = amp;
	if (duration > 0.00001) {
		out.vibration.duration_ns = (time_duration_ns)(duration * 1000.f * 1000.f * 1000.f);
	} else {
		out.vibration.duration_ns = XRT_MIN_HAPTIC_DURATION;
	}
	out.vibration.frequency = freq;

	if (controller->m_output_controls.empty()) {
		ovrd_log("Controller %s has no outputs\n", controller->m_xdev->str);
		return;
	}

	// TODO: controllers with more than 1 haptic motors
	SteamVRDriverControlOutput *control = &controller->m_output_controls.at(0);

	enum xrt_output_name name = control->monado_output_name;
	ovrd_log("Haptic vibration %s, %d\n", control->steamvr_control_path, name);
	controller->m_xdev->set_output(controller->m_xdev, name, &out);
}

void
CServerDriver_Monado::RunFrame()
{
	/*
	 * Timing of SteamVR's main loop as seen from here (measurement only): how often vrserver calls RunFrame and how
	 * long each part of ours takes. RunFrame runs in vrserver's main loop ("should not perform calls that might
	 * block", Valve's driver docs); apps got our headset pose from a snapshot one or two refreshes old (22:44 run),
	 * which a late main loop would explain. Logged every 10 s.
	 */
	int64_t t_start = (int64_t)os_monotonic_get_ns();
	if (m_rf_last_ns != 0) {
		m_rf_stats.add(0, t_start - m_rf_last_ns);
	}
	m_rf_last_ns = t_start;
	if (m_left) {
		m_left->RunFrame();
	}
	int64_t t_left = (int64_t)os_monotonic_get_ns();
	if (m_right) {
		m_right->RunFrame();
	}
	int64_t t_right = (int64_t)os_monotonic_get_ns();
	g_room.run_frame(os_monotonic_get_ns());
	int64_t t_room = (int64_t)os_monotonic_get_ns();
	m_rf_stats.add(1, t_left - t_start);
	m_rf_stats.add(2, t_right - t_left);
	m_rf_stats.add(3, t_room - t_right);

	// https://github.com/ValveSoftware/openvr/issues/719#issuecomment-358038640
	struct vr::VREvent_t event;
	while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(struct vr::VREvent_t))) {
		switch (event.eventType) {
		case vr::VREvent_Input_HapticVibration: HandleHapticEvent(&event); break;
		case vr::VREvent_PropertyChanged:
			// ovrd_log("Property changed\n");
			break;
		case vr::VREvent_TrackedDeviceActivated:
			ovrd_log("Device activated %d\n", event.trackedDeviceIndex);
			break;
		case vr::VREvent_TrackedDeviceUserInteractionStarted:
			ovrd_log("Device interaction started %d\n", event.trackedDeviceIndex);
			break;
		case vr::VREvent_IpdChanged: ovrd_log("ipd changed to %fm\n", event.data.ipd.ipdMeters); break;
		// This event currently spams the console, so is currently commented out. see
		// https://github.com/ValveSoftware/SteamVR-for-Linux/issues/307
		// case vr::VREvent_ActionBindingReloaded: ovrd_log("action binding reloaded\n"); break;
		case vr::VREvent_StatusUpdate: ovrd_log("EVRState: %d\n", event.data.status.statusState); break;

		case vr::VREvent_TrackedDeviceRoleChanged:
			// device roles are for legacy input
		case vr::VREvent_ChaperoneUniverseHasChanged:
		case vr::VREvent_ProcessQuit:
		case vr::VREvent_QuitAcknowledged:
		case vr::VREvent_ProcessDisconnected:
		case vr::VREvent_ProcessConnected:
		case vr::VREvent_DashboardActivated:
		case vr::VREvent_DashboardDeactivated:
		case vr::VREvent_Compositor_ChaperoneBoundsShown:
		case vr::VREvent_Compositor_ChaperoneBoundsHidden: break;

		default:
			if (debug_get_bool_option_ovrd_log_events()) {
				ovrd_log("Unhandled Event: %d\n", event.eventType);
			}
		}
	}
	int64_t t_end = (int64_t)os_monotonic_get_ns();
	m_rf_stats.add(4, t_end - t_room);
	m_rf_stats.add(5, t_end - t_start);
	if (t_end - m_rf_log_ns > 10 * (int64_t)U_TIME_1S_IN_NS) {
		if (m_rf_log_ns != 0) {
			m_rf_stats.log();
		}
		m_rf_stats.reset();
		m_rf_log_ns = t_end;
	}
}


/*
 *
 * Watchdog code
 *
 */

class CWatchdogDriver_Monado : public vr::IVRWatchdogProvider
{
public:
	CWatchdogDriver_Monado()
	{
		ovrd_log("Created watchdog object\n");
		m_pWatchdogThread = nullptr;
	}

	// clang-format off
	virtual ~CWatchdogDriver_Monado() {};
	virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext);
	virtual void Cleanup();
	// clang-format on

private:
	std::thread *m_pWatchdogThread;
};

CWatchdogDriver_Monado g_watchdogDriverMonado;
bool g_bExiting = false;

void
WatchdogThreadFunction()
{
	while (!g_bExiting) {
#if defined(_WINDOWS)
		// on windows send the event when the Y key is pressed.
		if ((0x01 & GetAsyncKeyState('Y')) != 0) {
			// Y key was pressed.
			vr::VRWatchdogHost()->WatchdogWakeUp(vr::TrackedDeviceClass_HMD);
		}
		std::this_thread::sleep_for(std::chrono::microseconds(500));
#else
		ovrd_log("Watchdog wakeup\n");
		// for the other platforms, just send one every five seconds
		std::this_thread::sleep_for(std::chrono::seconds(1));
		vr::VRWatchdogHost()->WatchdogWakeUp(vr::ETrackedDeviceClass::TrackedDeviceClass_HMD);
#endif
	}

	ovrd_log("Watchdog exit\n");
}

vr::EVRInitError
CWatchdogDriver_Monado::Init(vr::IVRDriverContext *pDriverContext)
{
	VR_INIT_WATCHDOG_DRIVER_CONTEXT(pDriverContext);
	ovrd_log_init(vr::VRDriverLog());

	// Watchdog mode on Windows starts a thread that listens for the 'Y' key
	// on the keyboard to be pressed. A real driver should wait for a system
	// button event or something else from the the hardware that signals
	// that the VR system should start up.
	g_bExiting = false;

	ovrd_log("starting watchdog thread\n");

	m_pWatchdogThread = new std::thread(WatchdogThreadFunction);
	if (!m_pWatchdogThread) {
		ovrd_log("Unable to create watchdog thread\n");
		return vr::VRInitError_Driver_Failed;
	}

	return vr::VRInitError_None;
}

void
CWatchdogDriver_Monado::Cleanup()
{
	g_bExiting = true;
	if (m_pWatchdogThread) {
		m_pWatchdogThread->join();
		delete m_pWatchdogThread;
		m_pWatchdogThread = nullptr;
	}

	ovrd_log_cleanup();
}


/*
 *
 * 'Exported' functions.
 *
 */

void *
ovrd_hmd_driver_impl(const char *pInterfaceName, int *pReturnCode)
{
	// clang-format off
	if (0 == strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName)) {
		return &g_serverDriverMonado;
	}
	if (0 == strcmp(vr::IVRWatchdogProvider_Version, pInterfaceName)) {
		return &g_watchdogDriverMonado;
	}
	// clang-format on

	ovrd_log("Unimplemented interface: %s\n", pInterfaceName);

	if (pReturnCode) {
		*pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
	}

	return NULL;
}

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
