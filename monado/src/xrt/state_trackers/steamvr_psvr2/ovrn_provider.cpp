// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The driver's provider: backend, devices and the pose thread (after ALVR's DriverProvider,
 *         alvr_server.cpp:79-174).
 * @ingroup st_ovrn
 */

#include "util/u_startup_log.h"
#include "ovrn_backend.hpp"
#include "ovrn_controller.hpp"
#include "ovrn_hand.hpp"
#include "ovrn_hmd.hpp"
#include "ovrn_interface.h"
#include "ovrn_log.hpp"
#include "ovrn_settings.hpp"

#include "util/u_git_tag.h"

#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

namespace ovrn {

namespace {

class Provider final : public vr::IServerTrackedDeviceProvider
{
public:
	//! The driver unloaded without Cleanup: the pose thread must not outlive the provider.
	~Provider()
	{
		if (m_running.exchange(false) && m_poseThread.joinable()) {
			m_poseThread.join();
		}
	}

	vr::EVRInitError
	Init(vr::IVRDriverContext *context) override
	{
		VR_INIT_SERVER_DRIVER_CONTEXT(context);
		u_startup_mark("driver: Init begins");
		// Which build this is (git describe of the source at configure time), to compare users' logs.
		log("ovrn: driver build %s\n", u_git_tag);
		applyCompositorDefaults();
		applySeeThroughSettings();
		// OVRN_BACKEND=sim: the simulated headset (tests); otherwise Monado's devices.
		const char *which = getenv("OVRN_BACKEND");
		m_backend = (which != nullptr && strcmp(which, "sim") == 0) ? make_sim_backend() : make_monado_backend();
		std::string why;
		if (!m_backend->start(&why)) {
			log("ovrn: backend %s didn't start: %s\n", m_backend->name(), why.c_str());
			m_backend.reset();
			VR_CLEANUP_SERVER_DRIVER_CONTEXT();
			return vr::VRInitError_Driver_HmdUnknown;
		}
		m_hmd.reset(new Hmd(m_backend.get()));
		if (!m_hmd->registerDevice()) {
			Cleanup();
			return vr::VRInitError_Driver_Failed;
		}
		for (Hand hand : {Hand::Left, Hand::Right}) {
			if (m_backend->hasController(hand)) {
				addController(hand);
			}
		}
		addHandsIfTracked();
		m_universe = m_backend->universe();
		m_running = true;
		m_poseThread = std::thread(&Provider::poseLoop, this);
		log("ovrn: started, backend %s, %zu controllers, %zu tracked hands\n", m_backend->name(), m_controllers.size(),
		    m_hands.size());
		u_startup_mark("driver: Init done (devices added)");
		return vr::VRInitError_None;
	}

	void
	Cleanup() override
	{
		if (m_running.exchange(false) && m_poseThread.joinable()) {
			m_poseThread.join();
		}
		m_live[0] = m_live[1] = nullptr;
		m_liveHands[0] = m_liveHands[1] = nullptr;
		for (auto &c : m_controllers) {
			c->hapticLog(now_ns(), true); // what's left of the vibration summary
		}
		m_controllers.clear();
		m_hands.clear();
		m_hmd.reset();
		if (m_backend) {
			m_backend->stop();
			m_backend.reset();
		}
		restoreCompositorDefaults();
		restoreSeeThroughSettings();
		VR_CLEANUP_SERVER_DRIVER_CONTEXT();
	}

	const char *const *
	GetInterfaceVersions() override
	{
		return vr::k_InterfaceVersions;
	}

	void
	RunFrame() override
	{
		if (!m_backend) {
			return;
		}
		// The backend first (it reads the controllers), then their inputs and the room's universe to SteamVR.
		const int64_t now = now_ns();
		m_backend->runFrame(now);
		// A controller that connected after the start (the backend picks it up in runFrame).
		for (Hand hand : {Hand::Left, Hand::Right}) {
			if (m_live[(int)hand] == nullptr && m_backend->hasController(hand)) {
				addController(hand); // its activation sets the current universe
				log("ovrn: %s controller added to SteamVR after the start\n", hand == Hand::Left ? "left" : "right");
			}
		}
		for (auto &c : m_controllers) {
			c->updateInputs();
		}
		addHandsIfTracked();
		for (auto &h : m_hands) {
			h->updateInputs(now);
		}
		if (m_hmd) {
			m_hmd->updateInputs(now);
		}
		const uint64_t universe = m_backend->universe();
		if (universe != m_universe) {
			m_universe = universe;
			if (m_hmd) {
				m_hmd->setUniverse(universe);
			}
			for (auto &c : m_controllers) {
				c->setUniverse(universe);
			}
			for (auto &h : m_hands) {
				h->setUniverse(universe);
			}
		}
		vr::VREvent_t event;
		while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(event))) {
			if (event.eventType == vr::VREvent_ChaperoneRoomSetupStarting ||
			    event.eventType == vr::VREvent_ChaperoneRoomSetupCommitted) {
				m_backend->roomSetupEvent(event.eventType);
				continue;
			}
			if (event.eventType == vr::VREvent_RoomViewShown || event.eventType == vr::VREvent_RoomViewHidden) {
				const bool on = event.eventType == vr::VREvent_RoomViewShown;
				if (m_hmd) {
					m_hmd->setSeeThrough(on);
				}
				log("ovrn: see-through %s\n", on ? "on (game paused, SteamVR renders less)" : "off");
				continue;
			}
			if (event.eventType != vr::VREvent_Input_HapticVibration) {
				continue;
			}
			const vr::VREvent_HapticVibration_t &h = event.data.hapticVibration;
			for (auto &c : m_controllers) {
				if (c->active() && c->container() == h.containerHandle) {
					c->haptic(h.fDurationSeconds, h.fFrequency, h.fAmplitude);
				}
			}
		}
		for (auto &c : m_controllers) {
			c->hapticLog(now_ns());
		}
	}

	bool
	ShouldBlockStandbyMode() override
	{
		return false;
	}
	void
	EnterStandby() override
	{}
	void
	LeaveStandby() override
	{}

private:
	static constexpr const char *kAsyncSection = "steamvr";
	static constexpr const char *kAsyncKey = "enableLinuxVulkanAsync";
	//! Our own note in SteamVR's settings that we set kAsyncKey, so a session that ended without Cleanup (a crash)
	//! still gets it taken out again by the next one.
	static constexpr const char *kOwnSection = "driver_monado_psvr2";
	static constexpr const char *kAsyncNoteKey = "asyncReprojectionSetByDriver";

	/*!
	 * SteamVR session defaults for this driver, however SteamVR was started (Steam, the DriverKit launcher,
	 * launch-steamvr.sh), set before vrserver starts the compositor (it does so ~20 ms after Init, and the compositor
	 * inherits vrserver's environment and reads its settings at start): the user's best run (2026-11-18 17:44).
	 *
	 * - The SteamVRLinuxFixes Vulkan layer off for the compositor (DISABLE_STEAMVR_LINUX_FIXES, the layer's own switch):
	 *   on SteamVR 2.19.3 its WaitForPendingPresent patch finds nothing, but it still replaces vrcompositor's GPU event
	 *   queries using struct offsets from an older build; with it the user saw clearly more judder (17:46 vs 17:44).
	 *   OVRN_LINUX_FIXES=1 keeps it.
	 * - SteamVR's async reprojection only when asked for (OVRN_ASYNC=1, DriverKit's "SteamVR async reprojection"
	 *   setting, off by default since 2026-11-21): late frames are then moved to the new head pose instead of shown
	 *   again (17:44: 6 frames dropped, 304 reprojected; without it 50 dropped, 60 timed out). For that session only:
	 *   Cleanup takes the setting out again, so other drivers (Sony's) keep SteamVR's default. Otherwise SteamVR's
	 *   own setting is left as it is (a leftover of ours from a session that crashed is taken out).
	 * - SteamVR's Facet renderer (steamvr/useFacetRenderer, an undocumented setting users turn on by hand for speed)
	 *   off for the session: with it the compositor aborted about 1.5 s after every start on a user's PC (2026-11-21,
	 *   SteamVR 2.18.4, RX 7600: ASSERT "NOIMPL" at facetrenderer.cpp:3572, SteamVR error -203), and the same build
	 *   ran with it off. Cleanup turns it back on. OVRN_FACET=1 leaves it on.
	 */
	void
	applyCompositorDefaults()
	{
		const char *fixes = getenv("OVRN_LINUX_FIXES");
		if (fixes == nullptr || strcmp(fixes, "1") != 0) {
			setenv("DISABLE_STEAMVR_LINUX_FIXES", "1", 1);
			log("ovrn: SteamVRLinuxFixes layer off for the compositor (OVRN_LINUX_FIXES=1 keeps it)\n");
		}
		vr::IVRSettings *settings = vr::VRSettings();
		if (settings == nullptr) {
			return;
		}
		applyFacetOff(settings);
		vr::EVRSettingsError err = vr::VRSettingsError_None;
		const bool noted = settings->GetBool(kOwnSection, kAsyncNoteKey, &err) && err == vr::VRSettingsError_None;
		const char *async = getenv("OVRN_ASYNC");
		if (async == nullptr || strcmp(async, "1") != 0) {
			if (noted) {
				settings->RemoveKeyInSection(kAsyncSection, kAsyncKey);
				settings->RemoveKeyInSection(kOwnSection, kAsyncNoteKey);
				log("ovrn: async reprojection setting left by an earlier session taken out\n");
			}
			log("ovrn: async reprojection: SteamVR's own setting (OVRN_ASYNC=1 asks for it)\n");
			return;
		}
		err = vr::VRSettingsError_None;
		const bool on = settings->GetBool(kAsyncSection, kAsyncKey, &err);
		const bool present = err == vr::VRSettingsError_None;
		if (present && on && !noted) {
			log("ovrn: async reprojection already on in SteamVR's settings (left as it is)\n");
			return; // the user's own setting
		}
		settings->SetBool(kAsyncSection, kAsyncKey, true);
		settings->SetBool(kOwnSection, kAsyncNoteKey, true);
		m_asyncSet = true;
		log("ovrn: SteamVR async reprojection on for this session (OVRN_ASYNC=1)\n");
	}

	static constexpr const char *kFacetKey = "useFacetRenderer";
	static constexpr const char *kFacetNoteKey = "facetRendererTurnedOffByDriver";

	//! The Facet renderer off for this session if the user's settings have it on (see applyCompositorDefaults).
	void
	applyFacetOff(vr::IVRSettings *settings)
	{
		const char *facet = getenv("OVRN_FACET");
		if (facet != nullptr && strcmp(facet, "1") == 0) {
			log("ovrn: SteamVR's Facet renderer setting left as it is (OVRN_FACET=1)\n");
			return;
		}
		vr::EVRSettingsError err = vr::VRSettingsError_None;
		const bool noted = settings->GetBool(kOwnSection, kFacetNoteKey, &err) && err == vr::VRSettingsError_None;
		err = vr::VRSettingsError_None;
		const bool on = settings->GetBool(kAsyncSection, kFacetKey, &err) && err == vr::VRSettingsError_None;
		if (!on && !noted) {
			return; // off or not set: SteamVR's standard renderer
		}
		// noted: an earlier session turned it off and ended without Cleanup; it still owes the user their setting.
		settings->SetBool(kAsyncSection, kFacetKey, false);
		settings->SetBool(kOwnSection, kFacetNoteKey, true);
		m_facetSet = true;
		log("ovrn: SteamVR's Facet renderer (useFacetRenderer) off for this session: it aborts the compositor at "
		    "start; back on at shutdown (OVRN_FACET=1 leaves it on)\n");
	}

	void
	restoreCompositorDefaults()
	{
		vr::IVRSettings *settings = vr::VRSettings();
		if (settings == nullptr) {
			return;
		}
		if (m_facetSet) {
			settings->SetBool(kAsyncSection, kFacetKey, true);
			settings->RemoveKeyInSection(kOwnSection, kFacetNoteKey);
			m_facetSet = false;
			log("ovrn: SteamVR's Facet renderer setting back on (the user's own)\n");
		}
		if (!m_asyncSet) {
			return;
		}
		settings->RemoveKeyInSection(kAsyncSection, kAsyncKey);
		settings->RemoveKeyInSection(kOwnSection, kAsyncNoteKey);
		m_asyncSet = false;
		log("ovrn: SteamVR async reprojection setting taken out again (SteamVR's default)\n");
	}

	bool m_asyncSet = false;
	bool m_facetSet = false;

	/*
	 * See-through, as on Sony's driver (its see-through mode, 2026-11-19): the headset's function button shows the
	 * whole camera picture, and pressing it again goes back to the game. Drawn by SteamVR, which owns the display here:
	 * the button is bound to SteamVR's Toggle Room View (resources/input/vrcompositor_binding_playstation_vr2_hmd.json)
	 * and, for the session, SteamVR's camera view is the opaque picture (camera/roomViewStyle 4) with no edge or ghost
	 * styles, and the camera on. The user's own camera settings are noted in our section and put back at Cleanup
	 * (also after a crash, by the next start). OVRN_SEE_THROUGH=0 leaves SteamVR's camera settings alone.
	 */
	struct CameraSetting
	{
		const char *key;
		bool is_bool;
		int32_t ours;
	};
	// (as openvr_driver.h's k_pch_Camera_* names, written out: those aren't constant expressions)
	static constexpr CameraSetting kSeeThroughSettings[] = {
	    {"enableCamera", true, 1},
	    {"roomViewStyle", false, vr::CAMERA_ROOMVIEW_STYLE_VIDEO_OPAQUE},
	    {"roomView", false, 1}, // on (2D): SteamVR's camera view may be opened
	};
	static constexpr const char *kSeeThroughNoteKey = "seeThroughSettingsSet";

	static std::string
	savedKey(const char *key, const char *what)
	{
		return std::string("seeThroughUser_") + key + what;
	}

	void
	applySeeThroughSettings()
	{
		const char *e = getenv("OVRN_SEE_THROUGH");
		vr::IVRSettings *settings = vr::VRSettings();
		std::string source, invalid;
		// CAMERA=off (ovrn_settings.hpp): no camera for SteamVR, so its camera settings stay the user's own.
		if ((e != nullptr && strcmp(e, "0") == 0) || settings == nullptr) {
			return;
		}
		vr::EVRSettingsError err = vr::VRSettingsError_None;
		const bool noted = settings->GetBool(kOwnSection, kSeeThroughNoteKey, &err) && err == vr::VRSettingsError_None;
		if (camera_mode(&source, &invalid) == CameraMode::Off) {
			// A session that ended without Cleanup left ours in place: put the user's back now.
			if (noted) {
				m_seeThroughSet = true;
				restoreSeeThroughSettings();
			}
			return;
		}
		for (const CameraSetting &c : kSeeThroughSettings) {
			// The user's value, unless a previous session (ended without Cleanup) already noted it.
			if (!noted) {
				err = vr::VRSettingsError_None;
				const int32_t v = c.is_bool ? (int32_t)settings->GetBool(vr::k_pch_Camera_Section, c.key, &err)
				                            : settings->GetInt32(vr::k_pch_Camera_Section, c.key, &err);
				const bool present = err == vr::VRSettingsError_None;
				settings->SetBool(kOwnSection, savedKey(c.key, "_present").c_str(), present);
				settings->SetInt32(kOwnSection, savedKey(c.key, "_value").c_str(), present ? v : 0);
			}
			if (c.is_bool) {
				settings->SetBool(vr::k_pch_Camera_Section, c.key, c.ours != 0);
			} else {
				settings->SetInt32(vr::k_pch_Camera_Section, c.key, c.ours);
			}
		}
		settings->SetBool(kOwnSection, kSeeThroughNoteKey, true);
		m_seeThroughSet = true;
		log("ovrn: see-through: the headset button shows the whole camera picture (SteamVR camera on, opaque view; "
		    "your camera settings come back at shutdown; OVRN_SEE_THROUGH=0 leaves them alone)\n");
	}

	void
	restoreSeeThroughSettings()
	{
		vr::IVRSettings *settings = vr::VRSettings();
		if (!m_seeThroughSet || settings == nullptr) {
			return;
		}
		for (const CameraSetting &c : kSeeThroughSettings) {
			vr::EVRSettingsError err = vr::VRSettingsError_None;
			const bool present = settings->GetBool(kOwnSection, savedKey(c.key, "_present").c_str(), &err);
			err = vr::VRSettingsError_None;
			const int32_t v = settings->GetInt32(kOwnSection, savedKey(c.key, "_value").c_str(), &err);
			if (!present) {
				settings->RemoveKeyInSection(vr::k_pch_Camera_Section, c.key);
			} else if (c.is_bool) {
				settings->SetBool(vr::k_pch_Camera_Section, c.key, v != 0);
			} else {
				settings->SetInt32(vr::k_pch_Camera_Section, c.key, v);
			}
			settings->RemoveKeyInSection(kOwnSection, savedKey(c.key, "_present").c_str());
			settings->RemoveKeyInSection(kOwnSection, savedKey(c.key, "_value").c_str());
		}
		settings->RemoveKeyInSection(kOwnSection, kSeeThroughNoteKey);
		m_seeThroughSet = false;
		log("ovrn: see-through: your SteamVR camera settings put back\n");
	}

	bool m_seeThroughSet = false;

	/*!
	 * One thread hands every device's pose to SteamVR on a steady 1 ms grid of absolute deadlines (no drift from the
	 * work done per step, unlike sleep_for(1 ms) loops). A step that overran skips to the next grid point.
	 */
	void
	poseLoop()
	{
		const int64_t step = 1000000;
		int64_t next = now_ns() + step;
		while (m_running) {
			struct timespec ts = {(time_t)(next / 1000000000LL), (long)(next % 1000000000LL)};
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
			const int64_t now = now_ns();
			if (m_hmd && m_hmd->active()) {
				m_hmd->updatePose(now);
			}
			for (auto &live : m_live) {
				Controller *c = live.load();
				if (c != nullptr && c->active()) {
					c->updatePose(now);
				}
			}
			for (auto &live : m_liveHands) {
				HandDevice *h = live.load();
				if (h != nullptr && h->active()) {
					h->updatePose(now);
				}
			}
			next += step;
			const int64_t after = now_ns();
			if (next <= after) {
				next += ((after - next) / step + 1) * step;
			}
		}
	}

	std::unique_ptr<Backend> m_backend;
	std::unique_ptr<Hmd> m_hmd;
	std::vector<std::unique_ptr<Controller>> m_controllers; //!< owned; added on the Init / RunFrame thread
	std::atomic<Controller *> m_live[2] = {{nullptr}, {nullptr}}; //!< by hand, for the pose thread

	void
	addController(Hand hand)
	{
		m_controllers.emplace_back(new Controller(m_backend.get(), hand));
		Controller *c = m_controllers.back().get();
		c->registerDevice();
		m_live[(int)hand] = c;
	}
	/*!
	 * Tracked hands: added once the backend has hand tracking (both hands together), as their own devices next to
	 * the controllers; each is connected only while its side's controller isn't there (HandDevice::inUse).
	 */
	void
	addHandsIfTracked()
	{
		if (!m_hands.empty() || !m_backend->hasHandTracking()) {
			return;
		}
		for (Hand hand : {Hand::Left, Hand::Right}) {
			m_hands.emplace_back(new HandDevice(m_backend.get(), hand));
			HandDevice *h = m_hands.back().get();
			h->registerDevice();
			m_liveHands[(int)hand] = h;
		}
	}
	std::vector<std::unique_ptr<HandDevice>> m_hands;               //!< owned; added on the Init / RunFrame thread
	std::atomic<HandDevice *> m_liveHands[2] = {{nullptr}, {nullptr}}; //!< by hand, for the pose thread
	uint64_t m_universe = 0; //!< RunFrame thread
	std::atomic<bool> m_running{false};
	std::thread m_poseThread;
};

Provider g_provider;

} // namespace

} // namespace ovrn

void *
ovrn_hmd_driver_impl(const char *pInterfaceName, int *pReturnCode)
{
	if (pInterfaceName != nullptr && strcmp(pInterfaceName, vr::IServerTrackedDeviceProvider_Version) == 0) {
		return static_cast<vr::IServerTrackedDeviceProvider *>(&ovrn::g_provider);
	}
	if (pReturnCode != nullptr) {
		*pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
	}
	return nullptr;
}
