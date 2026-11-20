// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Backend on Monado's devices: an xrt_instance without a compositor inside vrserver, the head device and
 *         the hand controllers read through the generic xrt_device interface. Nothing in Monado is changed for it.
 * @ingroup st_ovrn
 */

#include "ovrn_backend.hpp"

// The room calibration (ovrn_room*, from the previous plugin, removed 2026-11-18): the saved room and its files stay valid.
#include "ovrn_room_state.hpp"

#include "math/m_api.h"
#include "math/m_space.h"
#include "util/u_builder_helpers.h"
#include "util/u_debug.h"
#include "os/os_time.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_instance.h"
#include "xrt/xrt_space.h"
#include "xrt/xrt_system.h"
#include "xrt/xrt_tracking.h"

#ifdef XRT_BUILD_DRIVER_PSSENSE
#include "pssense/pssense_interface.h"
#endif

#ifdef XRT_BUILD_DRIVER_PSVR2
#include "ovrn_camera_fit.hpp"
#include "psvr2/psvr2_interface.h"
#include "psvr2/psvr2_hand_tracking.h"
#endif

#if defined(XRT_BUILD_DRIVER_PSVR2) && defined(XRT_BUILD_DRIVER_PSSENSE)
#define OVRN_CONTROLLER_HOTPLUG 1
#include "tracking/t_time_sync.h"
#include "xrt/xrt_frame.h"
#include "util/u_builder_search.h"
#include "xrt/xrt_prober.h"
#include <chrono>
#include <condition_variable>
#include <thread>
#endif

#include <algorithm>
#include <dlfcn.h>
#include <sys/stat.h>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <vector>

// Per-eye render target as a percentage of the eye's display size (0: 100 for the PS VR2, the panel's own 2000x2040
// per eye, the user's choice 2026-11-18; 170 = Sony's 3400x3468 was tried at 16:43 with Sony's display properties
// and looked twice as bad; 140 otherwise). psvr2-test/launch-steamvr.sh SCALE=<%> overrides it.
DEBUG_GET_ONCE_NUM_OPTION(ovrn_scale_percentage, "XRT_COMPOSITOR_SCALE_PERCENTAGE", 0)
// Seconds from vsync to photons / from photons to vblank (-1: 0.00136 / 0.0103 for the PS VR2, the values Sony's
// driver logs: "VsyncToPhotons: 0.001360, PhotonsToVblank: 0.010300", vrserver.txt 2026-11-17).
DEBUG_GET_ONCE_FLOAT_OPTION(ovrn_vsync_to_photons_s, "OVRN_VSYNC_TO_PHOTONS_S", -1.0f)
// The PS VR2's display properties and hidden-area mask as Sony's driver reports them (see headInfo), by group, to find
// which one makes SteamVR pace the game as with Sony's driver: "all", or a comma list of throttle, framerate, syslayer,
// texindex, supersample, dsc, edid, display, mask; "none": none. Tried all at once 2026-11-18 16:43 together with
// Sony's 3400x3468 render size: worse; 17:30 at 2000x2040: "first 20 s good, extended play bugs came back" (the
// game-side pacing problem fixed later). Default "all" since 2026-11-19: everything Sony's driver and PSVR2Toolkit
// report for the headset, so games see the same headset as with Sony's driver. OVRN_HIDDEN_AREA_FILE: another mask.
DEBUG_GET_ONCE_OPTION(ovrn_sony_props, "OVRN_SONY_PROPS", "all")
DEBUG_GET_ONCE_OPTION(ovrn_hidden_area_file, "OVRN_HIDDEN_AREA_FILE", NULL)
DEBUG_GET_ONCE_FLOAT_OPTION(ovrn_photons_to_vblank_s, "OVRN_PHOTONS_TO_VBLANK_S", -1.0f)
DEBUG_GET_ONCE_NUM_OPTION(ovrn_distortion_mesh_res, "OVRN_DISTORTION_MESH_RES", 80)
// Side of the square play area given to SteamVR, metres (psvr2-test/launch-steamvr.sh PLAY_AREA).
DEBUG_GET_ONCE_FLOAT_OPTION(ovrn_play_area_size, "OVRD_PLAY_AREA_SIZE", 2.0f)
// Sony's Sense render models and PS VR2 icons in the driver's resources (copied there by build-driver.sh, linked by
// launch-steamvr.sh): used when they're there. OVRD_PSSENSE_RENDER_MODELS / OVRD_PSVR2_SONY_ICONS = 0 or 1 override.
DEBUG_GET_ONCE_TRISTATE_OPTION(ovrn_pssense_render_models, "OVRD_PSSENSE_RENDER_MODELS")
DEBUG_GET_ONCE_TRISTATE_OPTION(ovrn_psvr2_sony_icons, "OVRD_PSVR2_SONY_ICONS")

namespace {

//! The driver's folder (<plugin>/bin/linux64/driver_monado_psvr2.so -> <plugin>), from where this library was loaded.
std::string
plugin_dir()
{
	Dl_info info = {};
	if (dladdr(reinterpret_cast<void *>(&plugin_dir), &info) == 0 || info.dli_fname == nullptr) {
		return {};
	}
	std::string p = info.dli_fname;
	for (int i = 0; i < 3; i++) {
		const size_t slash = p.rfind('/');
		if (slash == std::string::npos) {
			return {};
		}
		p.resize(slash);
	}
	return p;
}

bool
plugin_has(const char *relative)
{
	static const std::string dir = plugin_dir();
	struct stat st;
	return !dir.empty() && stat((dir + "/resources/" + relative).c_str(), &st) == 0;
}

//! Sony's Sense render models (resources/rendermodels/playstation_vr2_sense_*), unless the option says otherwise.
bool
use_sense_render_models()
{
	switch (debug_get_tristate_option_ovrn_pssense_render_models()) {
	case DEBUG_TRISTATE_ON: return true;
	case DEBUG_TRISTATE_OFF: return false;
	default:
		return plugin_has("rendermodels/playstation_vr2_sense_left") &&
		       plugin_has("rendermodels/playstation_vr2_sense_right");
	}
}

//! Sony's PS VR2 status icons (resources/icons/playstation_vr2/), unless the option says otherwise.
bool
use_sony_icons()
{
	switch (debug_get_tristate_option_ovrn_psvr2_sony_icons()) {
	case DEBUG_TRISTATE_ON: return true;
	case DEBUG_TRISTATE_OFF: return false;
	default: return plugin_has("icons/playstation_vr2/headset_status_off@2x.png");
	}
}

} // namespace

namespace ovrn {

namespace {

//! One SteamVR input component of a controller and the Monado input it reads.
struct InputMap
{
	const char *path;
	enum xrt_input_name name;
	int component;       //!< 0: the value itself, 1: x of a 2D input, 2: y
	bool bool_as_scalar; //!< a Monado button given to SteamVR as a 0/1 value
};

/*!
 * PS VR2 Sense with Sony's input names (playstation_vr2_sense_controller_profile.json, as Sony's driver): L2/R2 the
 * trigger, L1/R1 the grip button (a click on the Sense, given as a 0/1 value as Sony's profile declares it), the
 * _sensor inputs the finger proximity, the face buttons, create/options and PS per hand.
 */
const InputMap kSenseLeft[] = {
    {"/input/l2/value", XRT_INPUT_PSSENSE_TRIGGER_VALUE, 0, false},
    {"/input/l2/touch", XRT_INPUT_PSSENSE_TRIGGER_TOUCH, 0, false},
    {"/input/l2_sensor/value", XRT_INPUT_PSSENSE_TRIGGER_PROXIMITY_FLOAT, 0, false},
    {"/input/l1/value", XRT_INPUT_PSSENSE_SQUEEZE_CLICK, 0, true},
    {"/input/l1/touch", XRT_INPUT_PSSENSE_SQUEEZE_TOUCH, 0, false},
    {"/input/l1_sensor/value", XRT_INPUT_PSSENSE_SQUEEZE_PROXIMITY_FLOAT, 0, false},
    {"/input/left_stick/x", XRT_INPUT_PSSENSE_THUMBSTICK, 1, false},
    {"/input/left_stick/y", XRT_INPUT_PSSENSE_THUMBSTICK, 2, false},
    {"/input/left_stick/click", XRT_INPUT_PSSENSE_THUMBSTICK_CLICK, 0, false},
    {"/input/left_stick/touch", XRT_INPUT_PSSENSE_THUMBSTICK_TOUCH, 0, false},
    {"/input/square/click", XRT_INPUT_PSSENSE_SQUARE_CLICK, 0, false},
    {"/input/square/touch", XRT_INPUT_PSSENSE_SQUARE_TOUCH, 0, false},
    {"/input/triangle/click", XRT_INPUT_PSSENSE_TRIANGLE_CLICK, 0, false},
    {"/input/triangle/touch", XRT_INPUT_PSSENSE_TRIANGLE_TOUCH, 0, false},
    {"/input/create/click", XRT_INPUT_PSSENSE_SHARE_CLICK, 0, false},
    {"/input/left_ps/click", XRT_INPUT_PSSENSE_PS_CLICK, 0, false},
};
const InputMap kSenseRight[] = {
    {"/input/r2/value", XRT_INPUT_PSSENSE_TRIGGER_VALUE, 0, false},
    {"/input/r2/touch", XRT_INPUT_PSSENSE_TRIGGER_TOUCH, 0, false},
    {"/input/r2_sensor/value", XRT_INPUT_PSSENSE_TRIGGER_PROXIMITY_FLOAT, 0, false},
    {"/input/r1/value", XRT_INPUT_PSSENSE_SQUEEZE_CLICK, 0, true},
    {"/input/r1/touch", XRT_INPUT_PSSENSE_SQUEEZE_TOUCH, 0, false},
    {"/input/r1_sensor/value", XRT_INPUT_PSSENSE_SQUEEZE_PROXIMITY_FLOAT, 0, false},
    {"/input/right_stick/x", XRT_INPUT_PSSENSE_THUMBSTICK, 1, false},
    {"/input/right_stick/y", XRT_INPUT_PSSENSE_THUMBSTICK, 2, false},
    {"/input/right_stick/click", XRT_INPUT_PSSENSE_THUMBSTICK_CLICK, 0, false},
    {"/input/right_stick/touch", XRT_INPUT_PSSENSE_THUMBSTICK_TOUCH, 0, false},
    {"/input/cross/click", XRT_INPUT_PSSENSE_CROSS_CLICK, 0, false},
    {"/input/cross/touch", XRT_INPUT_PSSENSE_CROSS_TOUCH, 0, false},
    {"/input/circle/click", XRT_INPUT_PSSENSE_CIRCLE_CLICK, 0, false},
    {"/input/circle/touch", XRT_INPUT_PSSENSE_CIRCLE_TOUCH, 0, false},
    {"/input/options/click", XRT_INPUT_PSSENSE_OPTIONS_CLICK, 0, false},
    {"/input/right_ps/click", XRT_INPUT_PSSENSE_PS_CLICK, 0, false},
};
constexpr size_t kSenseInputs = sizeof(kSenseLeft) / sizeof(kSenseLeft[0]);

class MonadoBackend : public Backend
{
public:
	~MonadoBackend() override
	{
		stop();
	}

	bool
	start(std::string *why) override
	{
		/*
		 * PS VR2 head position aided by the accelerometer under SteamVR, as the previous plugin (replays of the
		 * 2026-11-18 recordings: position jitter while turning p90 0.13-0.15 -> 0.08-0.10 mm at SteamVR's
		 * prediction). The OpenXR path keeps the driver's own default; PSVR2_ACCEL_FUSION=0 turns it off here.
		 */
		setenv("PSVR2_ACCEL_FUSION", "1", 0);
		ovrd_log_init(vr::VRDriverLog());
		/*
		 * The room, before any device is added: SteamVR's own chaperone file, in universes of our own (Sony's driver
		 * reads SteamVR's file too, it doesn't give SteamVR one). Until 2026-11-19 the driver published its own file
		 * (<config>/monado_psvr2/chaperone_info.vrchap) as Prop_DriverProvidedChaperonePath_String: SteamVR read it,
		 * but every Room Setup save failed ("[CVRChaperoneServer] Failed to presiste chaperone data", in every session
		 * with a commit), so drawn rooms were lost; SteamVR saves its own file fine. A room drawn in the old file is
		 * copied over once and the old file renamed. Universe kSessionUniverse is this session's frame (the
		 * headset starts a new tracking world at every start: all 26 recordings of 2026-11-18 began at 0 0 0), with
		 * a square play area around the start (PLAY_AREA, the DriverKit's default play area; 0: none);
		 * kRoomUniverse is the saved room, active once a floor capture anchors it: the same default square until a
		 * room is drawn in Room Setup, which then replaces it and is kept (the default play area as ALVR sets one;
		 * the DriverKit's "Set Default Play Area" takes the drawn room out again, 2026-11-19).
		 */
		const std::string global = ovrd_room_chaperone_path();
		if (!global.empty()) {
			const std::string old_file = global.substr(0, global.rfind('/')) + "/monado_psvr2/chaperone_info.vrchap";
			struct stat st;
			if (stat(old_file.c_str(), &st) == 0) {
				std::string note;
				const std::string room_id = std::to_string(kRoomUniverse);
				ovrd_room_migrate_universe(old_file, global, room_id.c_str(), &note);
				const std::string done = old_file + ".moved-to-steamvr";
				const bool renamed = rename(old_file.c_str(), done.c_str()) == 0;
				ovrd_log("Room: the driver's old chaperone file: %s; %s\n", note.c_str(),
				         renamed ? ("renamed to " + done).c_str() : "could not rename it");
			}
		}
		const float play_area = debug_get_float_option_ovrn_play_area_size();
		// Empty path: the room code uses SteamVR's own file (openvrpaths.vrpath's config folder).
		m_room.init(play_area, play_area, std::string(), kRoomUniverse, kSessionUniverse);
		m_handTrackingWanted = driverkitSetting("HAND_TRACKING") == "1";
#ifdef XRT_BUILD_DRIVER_PSVR2
		psvr2_hand_tracking_want(m_handTrackingWanted);
#endif
		ovrd_log("ovrn: hand tracking %s (DriverKit's setting, %s)\n",
		         m_handTrackingWanted ? "on: the cameras track bare hands while a controller is off" : "off",
		         driverkitSettingsPath().c_str());
		if (xrt_instance_create(NULL, &m_xinst) != XRT_SUCCESS) {
			*why = "xrt_instance_create failed";
			return false;
		}
		// Devices only: SteamVR owns the display, so no Monado compositor is created.
		xrt_result_t xret = xrt_instance_create_system(m_xinst, &m_xsys, &m_xsysd, &m_xso, NULL);
		if (xret < 0 || m_xsysd == NULL || m_xsysd->static_roles.head == NULL || m_xsysd->static_roles.head->hmd == NULL) {
			*why = "no headset found by Monado";
			stop();
			return false;
		}
		m_head = m_xsysd->static_roles.head;
		struct xrt_system_roles roles = XRT_SYSTEM_ROLES_INIT;
		xrt_system_devices_get_roles(m_xsysd, &roles);
		if (roles.left >= 0 && roles.left < XRT_SYSTEM_MAX_DEVICES) {
			m_hand[0] = m_xsysd->static_xdevs[roles.left];
		}
		if (roles.right >= 0 && roles.right < XRT_SYSTEM_MAX_DEVICES) {
			m_hand[1] = m_xsysd->static_xdevs[roles.right];
		}
		// SteamVR's room setup places the devices: no offset of Monado's own.
		struct xrt_vec3 offset = {0, 0, 0};
		u_builder_setup_tracking_origins(m_head, NULL, m_hand[0], m_hand[1], NULL, &offset);
		m_room.set_hmd(m_head);
		for (int i = 0; i < 2; i++) {
			struct xrt_device *h = m_hand[i];
			if (h != NULL && h->name != XRT_DEVICE_PSSENSE) {
				ovrd_log("ovrn: %s controller '%s' is not a PS VR2 Sense controller, not exposed\n",
				         i == 0 ? "left" : "right", h->str);
				m_hand[i] = NULL;
			}
		}
		readEyes(m_eyes);
		startEyeTracking();
#ifdef OVRN_CONTROLLER_HOTPLUG
		startHotplug();
#endif
		return true;
	}

	void
	stop() override
	{
#ifdef OVRN_CONTROLLER_HOTPLUG
		stopHotplug();
#endif
		if (m_eyeTracking && m_head != NULL) {
			xrt_device_end_feature(m_head, XRT_DEVICE_FEATURE_EYE_TRACKING);
		}
		m_eyeTracking = false;
		m_head = NULL;
		m_hand[0] = m_hand[1] = NULL;
		if (m_xso != NULL) {
			xrt_space_overseer_destroy(&m_xso);
		}
		if (m_xsysd != NULL) {
			xrt_system_devices_destroy(&m_xsysd);
		}
#ifdef OVRN_CONTROLLER_HOTPLUG
		// Late controllers: their devices went with the system's (appended to it, after the headset, as the
		// builder's), then their frame nodes, as the system frees its own after its devices.
		xrt_frame_context_destroy_nodes(&m_lateFctx);
#endif
		if (m_xsys != NULL) {
			xrt_system_destroy(&m_xsys);
		}
		if (m_xinst != NULL) {
			xrt_instance_destroy(&m_xinst);
		}
	}

	const char *
	name() const override
	{
		return "monado";
	}

	DeviceInfo
	headInfo() override
	{
		DeviceInfo d;
		if (m_head->name == XRT_DEVICE_PSVR2) {
			d.manufacturer = "Sony Interactive Entertainment";
			d.model = "PlayStation VR2";
		} else {
			d.manufacturer = "Monado";
			d.model = m_head->str;
		}
		d.serial = m_head->serial[0] != '\0' ? m_head->serial : m_head->str;
		if (m_head->name == XRT_DEVICE_PSVR2) {
			// As Sony's driver presents it, so SteamVR recognises the headset as a PS VR2.
			d.controller_type = "playstation_vr2_hmd";
			d.input_profile = "{monado_psvr2}/input/playstation_vr2_profile.json";
			if (use_sony_icons()) {
				const char *dir = "{monado_psvr2}/icons/playstation_vr2/headset_status_";
				d.string_props = {
				    {vr::Prop_NamedIconPathDeviceOff_String, std::string(dir) + "off@2x.png"},
				    {vr::Prop_NamedIconPathDeviceSearching_String, std::string(dir) + "searching@2x.gif"},
				    {vr::Prop_NamedIconPathDeviceSearchingAlert_String, std::string(dir) + "searching@2x.gif"},
				    {vr::Prop_NamedIconPathDeviceReady_String, std::string(dir) + "ready@2x.png"},
				    {vr::Prop_NamedIconPathDeviceReadyAlert_String, std::string(dir) + "ready@2x.png"},
				    {vr::Prop_NamedIconPathDeviceNotReady_String, std::string(dir) + "error@2x.png"},
				    {vr::Prop_NamedIconPathDeviceStandby_String, std::string(dir) + "standby@2x.png"},
				};
			}
			// What Sony's driver reports for this display and we don't by default (tools/vrdisplaydump of a SteamVR
			// session with Sony's driver and the test game, 2026-11-18 16:30, logs/display-sony-20261118-163024.txt).
			// Not copied: the proximity sensor (1025) and eye gaze (6009), which this driver doesn't provide.
			const std::string groups = std::string(",") + debug_get_option_ovrn_sony_props() + ",";
			auto want = [&](const char *g) {
				return groups.find(",all,") != std::string::npos ||
				       groups.find(std::string(",") + g + ",") != std::string::npos;
			};
			if (want("throttle")) {
				d.bool_props.push_back({vr::Prop_Hmd_SupportsAppThrottling_Bool, true});
			}
			if (want("framerate")) {
				d.bool_props.push_back({vr::Prop_DisplaySupportsRuntimeFramerateChange_Bool, true});
			}
			if (want("syslayer")) {
				d.bool_props.push_back({vr::Prop_ForceSystemLayerUseAppPoses_Bool, true});
			}
			if (want("texindex")) {
				d.bool_props.push_back({2116 /* Prop_Hmd_AllowsClientToControlTextureIndex */, true});
			}
			if (want("supersample")) {
				d.bool_props.push_back({2115 /* Prop_Hmd_AllowSupersampleFiltering_Bool */, true});
			}
			if (want("dsc")) {
				// Reserved in this header; newer ones: display stream compression version, slice count, bits per pixel
				// x 16.
				d.int_props.push_back({2110, 17});
				d.int_props.push_back({2111, 4});
				d.int_props.push_back({2112, 160});
			}
			if (want("edid")) {
				d.int_props.push_back({vr::Prop_EdidVendorID_Int32, 55629});
				d.int_props.push_back({vr::Prop_EdidProductID_Int32, 49671});
			}
			if (want("display")) {
				d.bool_props.push_back({vr::Prop_DisplayAllowNightMode_Bool, true});
				d.bool_props.push_back({vr::Prop_DisplaySupportsAnalogGain_Bool, true});
				d.float_props = {
				    {vr::Prop_DisplayMinAnalogGain_Float, 0.0f},  {vr::Prop_DisplayMaxAnalogGain_Float, 1.0f},
				    {vr::Prop_DashboardScale_Float, 0.9f},        {vr::Prop_IpdUIRangeMinMeters_Float, 0.059f},
				    {vr::Prop_IpdUIRangeMaxMeters_Float, 0.072f},
				};
			}
			if (want("mask")) {
				// Sony's hidden-area mask (22.3% of each eye's render target), saved from that dump.
				const char *env = debug_get_option_ovrn_hidden_area_file();
				const char *xdg = getenv("XDG_CONFIG_HOME");
				const char *home = getenv("HOME");
				if (env != NULL && env[0] != '\0') {
					d.hidden_area_file = strcmp(env, "none") == 0 ? "" : env;
				} else if (xdg != NULL && xdg[0] != '\0') {
					d.hidden_area_file = std::string(xdg) + "/monado/psvr2-hidden-area.txt";
				} else if (home != NULL) {
					d.hidden_area_file = std::string(home) + "/.config/monado/psvr2-hidden-area.txt";
				}
			}
			if (groups != ",," && groups != ",none,") {
				ovrd_log("ovrn: Sony display properties (OVRN_SONY_PROPS): %s, %zu set\n", debug_get_option_ovrn_sony_props(),
				         d.bool_props.size() + d.int_props.size() + d.float_props.size() +
				             (d.hidden_area_file.empty() ? 0 : 1));
			}
		}
		return d;
	}

	DisplayInfo
	display() override
	{
		const struct xrt_hmd_parts *hmd = m_head->hmd;
		const bool psvr2 = m_head->name == XRT_DEVICE_PSVR2;
		DisplayInfo d;
		d.width = hmd->screens[0].w_pixels;
		d.height = hmd->screens[0].h_pixels;
		const double hz = 1e9 / (double)hmd->screens[0].nominal_frame_interval_ns;
		d.refresh_hz = (std::isfinite(hz) && hz > 1.0 && hz < 1000.0) ? hz : 60.0;
		float v2p = debug_get_float_option_ovrn_vsync_to_photons_s();
		d.vsync_to_photons_s = v2p >= 0.0f ? v2p : (psvr2 ? 0.00136 : 0.011);
		float p2v = debug_get_float_option_ovrn_photons_to_vblank_s();
		d.photons_to_vblank_s = p2v >= 0.0f ? p2v : (psvr2 ? 0.0103 : -1.0);
		d.distortion_mesh_resolution = (int)debug_get_num_option_ovrn_distortion_mesh_res();
		int scale = (int)debug_get_num_option_ovrn_scale_percentage();
		if (scale <= 0) {
			// 140%, as Monado's own compositor renders for OpenXR apps (comp_settings.c): 2800x2856 per eye on the
			// PS VR2. OpenXR looked sharper at "the same" size until 2026-11-18 showed it wasn't (2800 vs 2000).
			scale = 140;
		}
		Pose eyes[2];
		{
			std::lock_guard<std::mutex> lock(m_eyeMutex);
			eyes[0] = m_eyes[0];
			eyes[1] = m_eyes[1];
		}
		for (int e = 0; e < 2; e++) {
			EyeInfo &eye = d.eye[e];
			eye.viewport_x = hmd->views[e].viewport.x_pixels;
			eye.viewport_y = hmd->views[e].viewport.y_pixels;
			eye.viewport_w = hmd->views[e].viewport.w_pixels;
			eye.viewport_h = hmd->views[e].viewport.h_pixels;
			eye.render_w = (uint32_t)(hmd->views[0].display.w_pixels * (scale / 100.0f));
			eye.render_h = (uint32_t)(hmd->views[0].display.h_pixels * (scale / 100.0f));
			const struct xrt_fov &f = hmd->distortion.fov[e];
			eye.fov = {f.angle_left, f.angle_right, f.angle_up, f.angle_down};
			eye.eye_in_head = eyes[e];
		}
		return d;
	}

	bool
	eyePoses(Pose out[2]) override
	{
		Pose now[2];
		readEyes(now);
		std::lock_guard<std::mutex> lock(m_eyeMutex);
		const bool changed = fabs(now[0].position.x - m_eyes[0].position.x) > 1e-5 ||
		                     fabs(now[1].position.x - m_eyes[1].position.x) > 1e-5;
		if (changed) {
			m_eyes[0] = now[0];
			m_eyes[1] = now[1];
			out[0] = now[0];
			out[1] = now[1];
		}
		return changed;
	}

	bool
	headPose(int64_t at_ns, TrackedPose *out) override
	{
		if (!devicePose(m_head, XRT_INPUT_GENERIC_HEAD_POSE, at_ns, out)) {
			return false;
		}
		if (out->position_valid) {
			// The room watches the head for its tracking world being replaced, and for the capture's checks.
			struct xrt_vec3 p = {(float)out->pose.position.x, (float)out->pose.position.y,
			                     (float)out->pose.position.z};
			m_room.note_head(p, now_ns());
		}
		return true;
	}

	bool
	distortion(int eye, float u, float v, DistortionUv *out) override
	{
		// The view's 2x2 rotation, on u, v scaled to [-1, 1].
		const struct xrt_vec2 *rot = m_head->hmd->views[eye].rot.vecs;
		float ru = rot[0].x * (u * 2 - 1) + rot[0].y * (v * 2 - 1);
		float rv = rot[1].x * (u * 2 - 1) + rot[1].y * (v * 2 - 1);
		ru = (ru + 1) / 2;
		rv = (rv + 1) / 2;
		struct xrt_uv_triplet d;
		if (m_head->compute_distortion == NULL ||
		    m_head->compute_distortion(m_head, (uint32_t)eye, ru, rv, &d) != XRT_SUCCESS) {
			*out = {{ru, rv}, {ru, rv}, {ru, rv}};
			return true;
		}
		*out = {{d.r.x, d.r.y}, {d.g.x, d.g.y}, {d.b.x, d.b.y}};
		return true;
	}

	bool
	hasController(Hand hand) override
	{
		return m_hand[(int)hand] != NULL;
	}

	//! A Sense controller that was there but whose connection ended (turned off, out of range) isn't connected.
	bool
	controllerConnected(Hand hand) override
	{
		struct xrt_device *xdev = m_hand[(int)hand];
		if (xdev == NULL) {
			return false;
		}
#ifdef XRT_BUILD_DRIVER_PSSENSE
		if (xdev->name == XRT_DEVICE_PSSENSE && pssense_is_disconnected(xdev)) {
			return false;
		}
#endif
		return true;
	}

	/*!
	 * The Sense controllers as Sony's driver presents them, so SteamVR and apps recognise them: controller type
	 * playstation_vr2_sense, Sony's input names, and a remapping in the profile that lets apps with only Touch, Index,
	 * Vive or simple controller bindings use them.
	 */
	DeviceInfo
	controllerInfo(Hand hand) override
	{
		struct xrt_device *xdev = m_hand[(int)hand];
		const bool left = hand == Hand::Left;
		DeviceInfo d;
		d.manufacturer = "Sony Interactive Entertainment";
		d.model = left ? "PlayStation VR2 Sense Left" : "PlayStation VR2 Sense Right";
		d.serial = xdev->serial[0] != '\0' ? xdev->serial : xdev->str;
		if (d.serial == headInfo().serial || d.serial == std::string(m_hand[left ? 1 : 0] ? m_hand[left ? 1 : 0].load()->serial : "")) {
			d.serial += left ? "-left" : "-right";
		}
		d.controller_type = "playstation_vr2_sense";
		d.input_profile = "{monado_psvr2}/input/playstation_vr2_sense_controller_profile.json";
		d.grip_in_root = gripInRoot(hand);
		if (use_sense_render_models()) {
			d.render_model = left ? "{monado_psvr2}playstation_vr2_sense_left" : "{monado_psvr2}playstation_vr2_sense_right";
		} else {
			d.render_model = left ? "oculus_cv1_controller_left" : "oculus_cv1_controller_right";
		}
		if (use_sony_icons()) {
			const std::string dir =
			    std::string("{monado_psvr2}/icons/playstation_vr2/") + (left ? "left" : "right") + "_controller_status_";
			d.string_props = {
			    {vr::Prop_NamedIconPathDeviceOff_String, dir + "off@2x.png"},
			    {vr::Prop_NamedIconPathDeviceSearching_String, dir + "searching@2x.gif"},
			    {vr::Prop_NamedIconPathDeviceSearchingAlert_String, dir + "searching@2x.gif"},
			    {vr::Prop_NamedIconPathDeviceReady_String, dir + "ready@2x.png"},
			    {vr::Prop_NamedIconPathDeviceReadyAlert_String, dir + "ready@2x.png"},
			    {vr::Prop_NamedIconPathDeviceNotReady_String, dir + "error@2x.png"},
			    {vr::Prop_NamedIconPathDeviceStandby_String, dir + "standby@2x.png"},
			    {vr::Prop_NamedIconPathDeviceAlertLow_String, dir + "ready_low@2x.png"},
			};
		}
		return d;
	}

	/*!
	 * The controller's SteamVR root, as Sony's driver reports it: SteamVR places Sony's render model and the input
	 * profile's poses (handgrip, tip, aim) relative to it. Monado gives the OpenXR grip, 10 cm further along the
	 * controller and tilted 20.6 deg; reported as the root, the drawn controller sat ~11 cm off its real LEDs (LED
	 * model vs Sony's mesh: 82 mm median off as the grip, 1.3 mm as the root).
	 */
	bool
	controllerPose(Hand hand, int64_t at_ns, TrackedPose *out) override
	{
		struct xrt_device *xdev = m_hand[(int)hand];
		TrackedPose grip;
		if (xdev == NULL || !devicePose(xdev, XRT_INPUT_PSSENSE_GRIP_POSE, at_ns, &grip)) {
			return false;
		}
		*out = root_from_grip(grip, gripInRoot(hand));
		return true;
	}

	/*!
	 * DriverKit's settings file ($XDG_CONFIG_HOME/PSVR2-DriverKit/settings.conf, KEY=VALUE lines), read by the driver
	 * itself so a setting applies however SteamVR is started (from Steam too, not only from DriverKit).
	 */
	static std::string
	driverkitSettingsPath()
	{
		const char *xdg = getenv("XDG_CONFIG_HOME");
		const char *home = getenv("HOME");
		const std::string base = xdg != NULL && *xdg != '\0' ? std::string(xdg)
		                         : home != NULL           ? std::string(home) + "/.config"
		                                                  : std::string();
		return base.empty() ? std::string() : base + "/PSVR2-DriverKit/settings.conf";
	}

	static std::string
	driverkitSetting(const char *key)
	{
		const std::string path = driverkitSettingsPath();
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
	 * Hands are tracked while a controller is off (never connected, turned off, out of range), so the cameras and
	 * Mercury only work when hands can be used; both controllers on switches it off again (RunFrame thread).
	 */
	void
	switchHandTracking()
	{
#ifdef XRT_BUILD_DRIVER_PSVR2
		if (!m_handTrackingWanted || m_head == NULL || m_head->name != XRT_DEVICE_PSVR2) {
			return;
		}
		struct xrt_device *ht = psvr2_hmd_get_hand_tracker(m_head);
		if (ht == NULL) {
			return;
		}
		const bool on = !controllerConnected(Hand::Left) || !controllerConnected(Hand::Right);
		if (on != m_handTrackingOn) {
			m_handTrackingOn = on;
			psvr2_hand_tracking_set_active(ht, on);
			ovrd_log("ovrn: hand tracking %s\n", on ? "on (a controller is off)" : "off (both controllers are on)");
		}
#endif
	}

	//! The hand tracking device for a hand (optical hand tracking from the headset's cameras), or NULL.
	struct xrt_device *
	handTracker(Hand hand) const
	{
		if (m_xsysd == NULL) {
			return NULL;
		}
		return hand == Hand::Left ? m_xsysd->static_roles.hand_tracking.unobstructed.left
		                          : m_xsysd->static_roles.hand_tracking.unobstructed.right;
	}

	bool
	hasHandTracking() override
	{
		return handTracker(Hand::Left) != NULL || handTracker(Hand::Right) != NULL;
	}

	/*!
	 * The tracked hand from Monado's hand tracking (xrt_hand_joint_set: the hand pose in the tracker's space, the
	 * joints relative to it), in the backend's tracking space like the device poses (the tracking origin's offset
	 * applied). The tracker answers from its latest result without waiting for a camera frame.
	 */
	bool
	handJoints(Hand hand, int64_t at_ns, HandJoints *out) override
	{
		struct xrt_device *xdev = handTracker(hand);
		if (xdev == NULL) {
			return false;
		}
		*out = HandJoints();
		struct xrt_hand_joint_set set = {};
		int64_t ts = 0;
		const enum xrt_input_name name =
		    hand == Hand::Left ? XRT_INPUT_HT_UNOBSTRUCTED_LEFT : XRT_INPUT_HT_UNOBSTRUCTED_RIGHT;
		if (xrt_device_get_hand_tracking(xdev, name, at_ns, &set, &ts) != XRT_SUCCESS) {
			return true; // a tracker, but no hand now
		}
		const int need = XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
		if (!set.is_active || (set.hand_pose.relation_flags & need) != need) {
			return true;
		}
		struct xrt_pose hand_pose = set.hand_pose.pose;
		if (xdev->tracking_origin != NULL) {
			struct xrt_pose world;
			math_pose_transform(&xdev->tracking_origin->initial_offset, &hand_pose, &world);
			hand_pose = world;
		}
		out->active = true;
		out->timestamp_ns = ts;
		for (int i = 0; i < HandJoints::kCount; i++) {
			const struct xrt_hand_joint_value &v = set.values.hand_joint_set_default[i];
			struct xrt_pose p;
			math_pose_transform(&hand_pose, &v.relation.pose, &p);
			out->joint[i].orientation = {p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
			out->joint[i].position = {p.position.x, p.position.y, p.position.z};
			out->joint_valid[i] = (v.relation.relation_flags & need) == need;
			out->radius[i] = v.radius;
		}
		return true;
	}

	//! Where the grip is in the controller's SteamVR root (identity without the Sense driver).
	Pose
	gripInRoot(Hand hand)
	{
		Pose p;
#ifdef XRT_BUILD_DRIVER_PSSENSE
		struct xrt_device *xdev = m_hand[(int)hand];
		if (xdev != NULL && xdev->name == XRT_DEVICE_PSSENSE) {
			struct xrt_pose g;
			pssense_get_grip_in_steamvr_root(xdev, &g);
			p.orientation = {g.orientation.x, g.orientation.y, g.orientation.z, g.orientation.w};
			p.position = {g.position.x, g.position.y, g.position.z};
		}
#else
		(void)hand;
#endif
		return p;
	}

	std::vector<InputDesc>
	controllerInputDescs(Hand hand, std::string *out_haptic_path) override
	{
		const InputMap *map = hand == Hand::Left ? kSenseLeft : kSenseRight;
		std::vector<InputDesc> out;
		for (size_t i = 0; i < kSenseInputs; i++) {
			InputKind kind = InputKind::Boolean;
			if (map[i].bool_as_scalar) {
				kind = InputKind::ScalarOneSided;
			} else {
				switch (XRT_GET_INPUT_TYPE(map[i].name)) {
				case XRT_INPUT_TYPE_BOOLEAN: kind = InputKind::Boolean; break;
				case XRT_INPUT_TYPE_VEC1_ZERO_TO_ONE: kind = InputKind::ScalarOneSided; break;
				default: kind = InputKind::ScalarTwoSided; break;
				}
			}
			out.push_back({map[i].path, kind});
		}
		*out_haptic_path = "/output/haptic";
		return out;
	}

	//! The values read by the last runFrame (update_inputs runs there, once per SteamVR frame).
	size_t
	controllerInputs(Hand hand, float *out, size_t max) override
	{
		struct xrt_device *xdev = m_hand[(int)hand];
		if (xdev == NULL || max < kSenseInputs) {
			return 0;
		}
		const InputMap *map = hand == Hand::Left ? kSenseLeft : kSenseRight;
		for (size_t i = 0; i < kSenseInputs; i++) {
			out[i] = 0.0f;
			for (uint32_t k = 0; k < xdev->input_count; k++) {
				if (xdev->inputs[k].name != map[i].name) {
					continue;
				}
				const union xrt_input_value &v = xdev->inputs[k].value;
				if (XRT_GET_INPUT_TYPE(map[i].name) == XRT_INPUT_TYPE_BOOLEAN) {
					out[i] = v.boolean ? 1.0f : 0.0f;
				} else if (map[i].component == 1) {
					out[i] = v.vec2.x;
				} else if (map[i].component == 2) {
					out[i] = v.vec2.y;
				} else {
					out[i] = v.vec1.x;
				}
				break;
			}
		}
		return kSenseInputs;
	}

	void
	haptic(Hand hand, float duration_s, float frequency_hz, float amplitude) override
	{
		struct xrt_device *xdev = m_hand[(int)hand];
		if (xdev == NULL || xdev->set_output == NULL) {
			return;
		}
		struct xrt_output_value out = {};
		out.type = XRT_OUTPUT_VALUE_TYPE_VIBRATION;
		out.vibration.amplitude = amplitude;
		out.vibration.frequency = frequency_hz;
		out.vibration.duration_ns =
		    duration_s > 0.00001f ? (time_duration_ns)(duration_s * 1e9f) : XRT_MIN_HAPTIC_DURATION;
		xdev->set_output(xdev, XRT_OUTPUT_NAME_PSSENSE_VIBRATION, &out);
	}

	WorldTransform
	worldFromDriver() override
	{
		const ovrd_room_transform t = m_room.transform();
		WorldTransform w;
		w.rotation = {t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w};
		w.translation = {t.translation.x, t.translation.y, t.translation.z};
		return w;
	}

	uint64_t
	universe() override
	{
		return m_room.universe();
	}

	//! With the room calibration off: a square play area in universe 2, so SteamVR skips its room setup.
	bool
	headButton() override
	{
		return m_headButton;
	}

	bool
	hasEyeTracking() override
	{
		return m_eyeTracking;
	}

	bool
	eyeGaze(EyeGaze *out) override
	{
#ifdef XRT_BUILD_DRIVER_PSVR2
		if (!m_eyeTracking) {
			return false;
		}
		struct xrt_vec3 d;
		bool valid = false;
		int64_t sample_ns = 0, packet_ns = 0;
		if (!psvr2_get_combined_gaze(m_head, &d, &valid, &sample_ns, &packet_ns)) {
			return false;
		}
		out->dir = {d.x, d.y, d.z};
		out->valid = valid;
		out->sample_ns = sample_ns;
		out->packet_age_ns = (int64_t)os_monotonic_get_ns() - packet_ns;
		return true;
#else
		(void)out;
		return false;
#endif
	}

	bool
	hasProximity() override
	{
		return m_head != NULL && m_head->name == XRT_DEVICE_PSVR2;
	}

	bool
	headWorn() override
	{
		return m_headWorn;
	}

	std::string
	chaperoneJson() override
	{
		const float size = debug_get_float_option_ovrn_play_area_size();
		if (m_room.enabled() || size <= 0.0f) {
			return std::string();
		}
		const float h = size / 2.0f;
		char json[2048];
		snprintf(json, sizeof(json),
		         "{\"jsonid\":\"chaperone_info\",\"version\":5,\"universes\":[{\"universeID\":\"2\","
		         "\"play_area\":[%.3f,%.3f],"
		         "\"collision_bounds\":["
		         "[[-%.3f,0,-%.3f],[-%.3f,2.43,-%.3f],[-%.3f,2.43,%.3f],[-%.3f,0,%.3f]],"
		         "[[-%.3f,0,%.3f],[-%.3f,2.43,%.3f],[%.3f,2.43,%.3f],[%.3f,0,%.3f]],"
		         "[[%.3f,0,%.3f],[%.3f,2.43,%.3f],[%.3f,2.43,-%.3f],[%.3f,0,-%.3f]],"
		         "[[%.3f,0,-%.3f],[%.3f,2.43,-%.3f],[-%.3f,2.43,-%.3f],[-%.3f,0,-%.3f]]],"
		         "\"seated\":{\"translation\":[0,0,0],\"yaw\":0},"
		         "\"standing\":{\"translation\":[0,0,0],\"yaw\":0}}]}",
		         size, size, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h, h,
		         h, h);
		return json;
	}

	//! RunFrame thread: read the controllers, the room capture gesture (left Sense on the floor, Create held), the room.
	void
	runFrame(int64_t now) override
	{
#ifdef OVRN_CONTROLLER_HOTPLUG
		for (int i = 0; i < 2; i++) {
			if (m_hand[i] == NULL && m_late[i] != NULL) {
				m_hand[i] = m_late[i].load(); // the provider adds it to SteamVR after this runFrame
			}
		}
#endif
		for (int i = 0; i < 2; i++) {
			if (m_hand[i] != NULL) {
				xrt_device_update_inputs(m_hand[i]);
			}
		}
		switchHandTracking();
		const bool was_down = m_headButton;
		m_headButton = false;
		if (m_head != NULL && m_head->name == XRT_DEVICE_PSVR2 && xrt_device_update_inputs(m_head) == XRT_SUCCESS) {
			for (uint32_t k = 0; k < m_head->input_count; k++) {
				if (m_head->inputs[k].name == XRT_INPUT_PSVR2_SYSTEM_CLICK) {
					m_headButton = m_head->inputs[k].value.boolean;
				} else if (m_head->inputs[k].name == XRT_INPUT_GENERIC_HEAD_DETECT) {
					m_headWorn = m_head->inputs[k].value.boolean;
				}
			}
		}
		m_headPresses += (m_headButton && !was_down) ? 1 : 0;
		roomCaptureFrame(now);
		m_room.run_frame(now);
		promptFrame(now);
		if (now - m_lastDeviceLogNs > 30 * U_TIME_1S_IN_NS) {
			m_lastDeviceLogNs = now;
			logDevices(now);
		}
	}

	std::string
	//! None: SteamVR keeps the rooms in its own file (see start()).
	chaperonePath() override
	{
		return std::string();
	}

#ifdef XRT_BUILD_DRIVER_PSVR2
	bool
	hasCameras() override
	{
		return m_head != NULL && m_head->name == XRT_DEVICE_PSVR2;
	}

	bool
	cameraInfo(CameraInfo *out) override
	{
		if (m_head == NULL || m_head->name != XRT_DEVICE_PSVR2) {
			return false;
		}
		struct t_camera_calibration cams[2];
		struct xrt_pose poses[2];
		// OVRN_CAMERA_POSES=tracker: the controller tracker's camera poses for the picture, to compare (2026-11-19);
		// default: the cameras' measured placement. Either way the controller tracker itself is untouched.
		const char *which = getenv("OVRN_CAMERA_POSES");
		const bool tracker = which != nullptr && strcmp(which, "tracker") == 0;
		if (!psvr2_get_bottom_cameras(m_head, cams, poses, tracker)) {
			return false;
		}
		CameraInfo info;
		info.tracker_poses = tracker;
		info.width = cams[0].image_size_pixels.w;
		info.height = cams[0].image_size_pixels.h;
		for (int c = 0; c < 2; c++) {
			info.fx[c] = cams[c].intrinsics[0][0];
			info.fy[c] = cams[c].intrinsics[1][1];
			info.cx[c] = cams[c].intrinsics[0][2];
			info.cy[c] = cams[c].intrinsics[1][2];
			info.cam_in_head[c] = toPose(poses[c]);
			info.fit_rms_px[c] = fit_ftheta(cams[c], info.ftheta[c]);
		}
		*out = info;
		return info.width > 0 && info.height > 0;
	}

	bool
	cameraStart(CameraImageFn fn) override
	{
		if (m_head == NULL || m_head->name != XRT_DEVICE_PSVR2) {
			return false;
		}
		{
			std::lock_guard<std::mutex> lock(m_cameraMutex);
			m_cameraFn = std::move(fn);
		}
		psvr2_set_camera_image_callback(m_head, &MonadoBackend::cameraImage, this);
		psvr2_request_bottom_camera_images(m_head, true);
		return true;
	}

	void
	cameraStop() override
	{
		if (m_head == NULL || m_head->name != XRT_DEVICE_PSVR2) {
			return;
		}
		psvr2_request_bottom_camera_images(m_head, false);
		psvr2_set_camera_image_callback(m_head, NULL, NULL);
		std::lock_guard<std::mutex> lock(m_cameraMutex);
		m_cameraFn = nullptr;
	}
#endif

	void
	roomSetupEvent(uint32_t event_type) override
	{
		if (event_type == vr::VREvent_ChaperoneRoomSetupStarting) {
			if (m_room.room_setup_started()) {
				m_promptPulses = 3; // three short buzzes on the left controller: anchor the room first
				m_promptNextNs = 0;
			}
		} else if (event_type == vr::VREvent_ChaperoneRoomSetupCommitted) {
			m_room.room_setup_committed();
		}
	}

private:
	// The universes in the driver's own chaperone file (any id but 0 and 1, DOC "Driver_API_Documentation": 1 is
	// reserved for Oculus). Distinct from the old plugin's 2 and 3 in SteamVR's own file.
	static constexpr uint64_t kRoomUniverse = 5022;
	static constexpr uint64_t kSessionUniverse = 5023;

	//! The "anchor the room first" prompt: short buzzes 0.35 s apart on the left controller.
	void
	promptFrame(int64_t now)
	{
		if (m_promptPulses <= 0 || now < m_promptNextNs) {
			return;
		}
		haptic(Hand::Left, 0.12f, 160.0f, 1.0f);
		m_promptPulses--;
		m_promptNextNs = now + 350 * U_TIME_1MS_IN_NS;
	}

	//! Every 30 s: the headset and both controllers in SteamVR's world (the room transform applied), one frame.
#ifdef OVRN_CONTROLLER_HOTPLUG
	/*
	 * Sense controllers that connect after the start (2026-11-18 20:37 and 20:38: the left one connected 7 s and
	 * 0.9 s after the driver had looked once, and SteamVR got only the right one). Every 2 s while a side is
	 * missing: probe, create it as the builder does, add it to the headset's LED tracking, and hand it to runFrame.
	 * OVRN_CONTROLLER_HOTPLUG=0 turns it off.
	 */
	void
	startHotplug()
	{
		const char *e = getenv("OVRN_CONTROLLER_HOTPLUG");
		if ((e != nullptr && strcmp(e, "0") == 0) || m_head == NULL || m_head->name != XRT_DEVICE_PSVR2) {
			return;
		}
		m_hpQuit = false;
		m_hotplug = std::thread(&MonadoBackend::hotplugLoop, this);
	}

	void
	stopHotplug()
	{
		{
			std::lock_guard<std::mutex> l(m_hpMutex);
			m_hpQuit = true;
			m_hpCv.notify_all();
		}
		if (m_hotplug.joinable()) {
			m_hotplug.join();
		}
	}

	void
	hotplugLoop()
	{
		std::unique_lock<std::mutex> l(m_hpMutex);
		while (!m_hpQuit) {
			m_hpCv.wait_for(l, std::chrono::seconds(2), [&] { return m_hpQuit; });
			if (m_hpQuit) {
				break;
			}
			const bool missing = (m_hand[0] == NULL && m_late[0] == NULL) || (m_hand[1] == NULL && m_late[1] == NULL);
			if (!missing && !anyDisconnected()) {
				continue;
			}
			l.unlock();
			hotplugScan();
			l.lock();
		}
	}

	//! A controller whose connection ended (Bluetooth dropped, turned off) and that should be reconnected.
	struct xrt_device *
	disconnectedHand(int i)
	{
		struct xrt_device *xdev = m_hand[i] != NULL ? m_hand[i].load() : m_late[i].load();
		return xdev != NULL && xdev->name == XRT_DEVICE_PSSENSE && pssense_is_disconnected(xdev) ? xdev : NULL;
	}

	bool
	anyDisconnected()
	{
		return disconnectedHand(0) != NULL || disconnectedHand(1) != NULL;
	}

	void
	hotplugScan()
	{
		struct xrt_prober *xp = NULL;
		if (xrt_instance_get_prober(m_xinst, &xp) != XRT_SUCCESS || xp == NULL || xrt_prober_probe(xp) != XRT_SUCCESS) {
			return;
		}
		struct xrt_prober_device **devs = NULL;
		size_t count = 0;
		if (xrt_prober_lock_list(xp, &devs, &count) != XRT_SUCCESS) {
			return;
		}
		struct xrt_device *made[2] = {NULL, NULL};
		struct t_timing_event_sink *sinks[2] = {NULL, NULL};
		for (int i = 0; i < 2; i++) {
			/*
			 * A controller that dropped off Bluetooth and came back is a new HID device for the same controller
			 * (2026-11-19 23:12: the left one reconnected as hidraw11 and stayed dead in SteamVR until restart).
			 * Hand it the new connection; the device, its SteamVR controller and its LED tracking stay.
			 */
			struct xrt_device *gone = disconnectedHand(i);
			if (gone != NULL) {
				struct xrt_prober_device *xpdev = u_builder_find_prober_device(
				    devs, count, PSSENSE_VID, i == 0 ? PSSENSE_PID_LEFT : PSSENSE_PID_RIGHT, XRT_BUS_TYPE_ANY);
				if (xpdev != NULL && pssense_reconnect(gone, xp, xpdev) > 0) {
					ovrd_log("ovrn: %s controller reconnected after dropping out: %s (%s)\n", i == 0 ? "left" : "right",
					         gone->str, gone->serial);
				}
				continue;
			}
			if (m_hand[i] != NULL || m_late[i] != NULL) {
				continue;
			}
			struct xrt_prober_device *xpdev = u_builder_find_prober_device(
			    devs, count, PSSENSE_VID, i == 0 ? PSSENSE_PID_LEFT : PSSENSE_PID_RIGHT, XRT_BUS_TYPE_ANY);
			if (xpdev != NULL) {
				made[i] = pssense_create(xp, xpdev, &m_lateFctx, &sinks[i]);
			}
		}
		xrt_prober_unlock_list(xp, &devs);

		for (int i = 0; i < 2; i++) {
			if (made[i] == NULL) {
				continue;
			}
			struct t_timing_event_source *source = psvr2_hmd_get_timing_event_source((struct psvr2_hmd *)m_head);
			if (source != NULL && sinks[i] != NULL) {
				t_timing_event_source_add_sink(source, sinks[i]);
			}
			psvr2_hmd_add_late_controller(m_head, made[i], i == 0);
			// Destroyed with the system's devices, after the headset (as the builder's controllers are).
			if (m_xsysd->static_xdev_count < XRT_SYSTEM_MAX_DEVICES) {
				m_xsysd->static_xdevs[m_xsysd->static_xdev_count++] = made[i];
			}
			ovrd_log("ovrn: %s controller connected after the start: %s (%s), added\n", i == 0 ? "left" : "right",
			         made[i]->str, made[i]->serial);
			m_late[i] = made[i];
		}
	}
#endif

	void
	logDevices(int64_t now)
	{
		const ovrd_room_transform T = m_room.transform();
		char line[512];
		int n = snprintf(line, sizeof(line), "Room: devices in the world frame (universe %llu):",
		                 (unsigned long long)m_room.universe());
		struct
		{
			const char *name;
			struct xrt_device *xdev;
			enum xrt_input_name input;
		} devs[3] = {{"head", m_head, XRT_INPUT_GENERIC_HEAD_POSE},
		             {"left", m_hand[0], XRT_INPUT_PSSENSE_GRIP_POSE},
		             {"right", m_hand[1], XRT_INPUT_PSSENSE_GRIP_POSE}};
		for (const auto &d : devs) {
			TrackedPose t;
			if (d.xdev == NULL || n >= (int)sizeof(line) || !devicePose(d.xdev, d.input, now, &t) || !t.position_valid) {
				n += snprintf(line + n, sizeof(line) - n, " %s -", d.name);
				continue;
			}
			const struct xrt_vec3 p = ovrd_room_apply(
			    T, xrt_vec3{(float)t.pose.position.x, (float)t.pose.position.y, (float)t.pose.position.z});
			n += snprintf(line + n, sizeof(line) - n, " %s %.3f %.3f %.3f%s", d.name, p.x, p.y, p.z,
			              t.position_tracked ? "" : " (not tracked)");
		}
		ovrd_log("%s; headset button presses %u\n", line, m_headPresses);
	}

	static Pose
	toPose(const struct xrt_pose &p)
	{
		Pose out;
		out.orientation = {p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
		out.position = {p.position.x, p.position.y, p.position.z};
		return out;
	}

#ifdef XRT_BUILD_DRIVER_PSVR2
	//! The headset driver's camera callback (USB thread): hand the image pair on, exposure on now_ns()'s clock.
	static void
	cameraImage(void *user, const uint8_t *bc4_pair, size_t size, int64_t exposure_mono_ns)
	{
		MonadoBackend *self = static_cast<MonadoBackend *>(user);
		std::lock_guard<std::mutex> lock(self->m_cameraMutex);
		if (self->m_cameraFn) {
			self->m_cameraFn(bc4_pair, size, exposure_mono_ns);
		}
	}

#endif

	std::mutex m_cameraMutex;
	CameraImageFn m_cameraFn;

	int m_promptPulses = 0;
	int64_t m_promptNextNs = 0;
	bool m_headButton = false; //!< RunFrame thread
	bool m_headWorn = false;   //!< RunFrame thread: the headset's wear sensor
	bool m_eyeTracking = false; //!< the headset's eye tracking was turned on (startEyeTracking)

	/*!
	 * Turn the PS VR2's eye tracking on for SteamVR (its /eyetracking input, see ovrn_hmd.hpp): the headset then
	 * streams gaze. OVRN_EYE_TRACKING=0 leaves it off (the eye cameras stay dark, no eye-gaze capability reported).
	 */
	void
	startEyeTracking()
	{
		const char *e = getenv("OVRN_EYE_TRACKING");
		if ((e != NULL && strcmp(e, "0") == 0) || m_head == NULL || m_head->name != XRT_DEVICE_PSVR2 ||
		    !m_head->supported.eye_gaze) {
			return;
		}
		m_eyeTracking = xrt_device_begin_feature(m_head, XRT_DEVICE_FEATURE_EYE_TRACKING) == XRT_SUCCESS;
		ovrd_log("ovrn: eye tracking %s (OVRN_EYE_TRACKING=0 off)\n", m_eyeTracking ? "on" : "could not be turned on");
	}
	uint32_t m_headPresses = 0; //!< headset button presses (in the 30 s device line)
	int64_t m_lastDeviceLogNs = 0;

	/*!
	 * Room capture (left Sense controller): Create held with the controller lying still on the floor. A short buzz
	 * (0.25 s) says it worked, a long one (0.9 s) that it didn't (the reason is in the log).
	 */
	void
	roomCaptureFrame(int64_t now)
	{
#ifdef XRT_BUILD_DRIVER_PSSENSE
		struct xrt_device *xdev = m_hand[0];
		if (!m_room.enabled() || xdev == NULL) {
			return;
		}
		if (m_outlineCount == 0) {
			m_outlineCount = pssense_get_grip_outline(xdev, m_outline, ARRAY_SIZE(m_outline), &m_outlineRadius);
		}
		bool create = false;
		for (uint32_t i = 0; i < xdev->input_count; i++) {
			if (xdev->inputs[i].name == XRT_INPUT_PSSENSE_SHARE_CLICK) {
				create = xdev->inputs[i].value.boolean;
				break;
			}
		}
		struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
		if (create) {
			xrt_device_get_tracked_pose(xdev, XRT_INPUT_PSSENSE_GRIP_POSE, now, &rel);
			struct xrt_relation_chain chain = {};
			m_relation_chain_push_relation(&chain, &rel);
			m_relation_chain_push_pose_if_not_identity(&chain, &xdev->tracking_origin->initial_offset);
			m_relation_chain_resolve(&chain, &rel);
		}
		OvrdRoom::CaptureResult result =
		    m_room.controller_frame(create, rel, m_outline, m_outlineCount, m_outlineRadius, now);
		if (result != OvrdRoom::CaptureResult::NONE) {
			haptic(Hand::Left, result == OvrdRoom::CaptureResult::REJECTED ? 0.9f : 0.25f, 0.0f, 0.6f);
		}
#else
		(void)now;
#endif
	}

	void
	readEyes(Pose out[2])
	{
		struct xrt_vec3 ipd = {0.063f, 0, 0};
		struct xrt_space_relation head;
		struct xrt_fov fovs[2];
		struct xrt_pose poses[2];
		xrt_device_get_view_poses(m_head, &ipd, now_ns(), XRT_VIEW_TYPE_STEREO, 2, &head, fovs, poses);
		for (int e = 0; e < 2; e++) {
			out[e].orientation = {poses[e].orientation.x, poses[e].orientation.y, poses[e].orientation.z,
			                      poses[e].orientation.w};
			out[e].position = {poses[e].position.x, poses[e].position.y, poses[e].position.z};
		}
	}

	static bool
	devicePose(struct xrt_device *xdev, enum xrt_input_name input, int64_t at_ns, TrackedPose *out)
	{
		struct xrt_space_relation rel = {};
		if (xrt_device_get_tracked_pose(xdev, input, at_ns, &rel) != XRT_SUCCESS) {
			return false;
		}
		struct xrt_relation_chain chain = {};
		m_relation_chain_push_relation(&chain, &rel);
		m_relation_chain_push_pose_if_not_identity(&chain, &xdev->tracking_origin->initial_offset);
		m_relation_chain_resolve(&chain, &rel);

		*out = TrackedPose();
		const int f = rel.relation_flags;
		out->orientation_valid = (f & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0;
		out->orientation_tracked = (f & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) != 0;
		out->position_valid = (f & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0;
		out->position_tracked = (f & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0;
		if (out->orientation_valid) {
			out->pose.orientation = {rel.pose.orientation.x, rel.pose.orientation.y, rel.pose.orientation.z,
			                         rel.pose.orientation.w};
		}
		if (out->position_valid) {
			out->pose.position = {rel.pose.position.x, rel.pose.position.y, rel.pose.position.z};
		}
		if ((f & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT) != 0) {
			out->linear_velocity = {rel.linear_velocity.x, rel.linear_velocity.y, rel.linear_velocity.z};
		}
		if ((f & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT) != 0) {
			out->angular_velocity = {rel.angular_velocity.x, rel.angular_velocity.y, rel.angular_velocity.z};
		}
		return true;
	}

	struct xrt_instance *m_xinst = NULL;
	struct xrt_system *m_xsys = NULL;
	struct xrt_system_devices *m_xsysd = NULL;
	struct xrt_space_overseer *m_xso = NULL;
	struct xrt_device *m_head = NULL;
	//! RunFrame and pose threads read them; set at start, or by runFrame when a late controller is ready.
	std::atomic<struct xrt_device *> m_hand[2] = {{NULL}, {NULL}};
	bool m_handTrackingWanted = false; //!< DriverKit's setting
	bool m_handTrackingOn = false;     //!< RunFrame thread
#ifdef OVRN_CONTROLLER_HOTPLUG
	std::atomic<struct xrt_device *> m_late[2] = {{NULL}, {NULL}}; //!< made by the hotplug thread
	struct xrt_frame_context m_lateFctx = {};
	std::thread m_hotplug;
	std::mutex m_hpMutex;
	std::condition_variable m_hpCv;
	bool m_hpQuit = false;
#endif
	std::mutex m_eyeMutex;
	Pose m_eyes[2];
	OvrdRoom m_room;
	struct xrt_vec3 m_outline[64] = {}; //!< the left controller's outline in its grip frame (room capture)
	size_t m_outlineCount = 0;
	float m_outlineRadius = 0;
};

} // namespace

std::unique_ptr<Backend>
make_monado_backend()
{
	return std::unique_ptr<Backend>(new MonadoBackend());
}

} // namespace ovrn
