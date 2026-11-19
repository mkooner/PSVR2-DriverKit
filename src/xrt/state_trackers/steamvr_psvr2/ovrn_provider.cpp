// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The driver's provider: backend, devices and the pose thread (after ALVR's DriverProvider,
 *         alvr_server.cpp:79-174).
 * @ingroup st_ovrn
 */

#include "ovrn_backend.hpp"
#include "ovrn_controller.hpp"
#include "ovrn_hmd.hpp"
#include "ovrn_interface.h"
#include "ovrn_log.hpp"

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
		applyCompositorDefaults();
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
				m_controllers.emplace_back(new Controller(m_backend.get(), hand));
				m_controllers.back()->registerDevice();
			}
		}
		m_universe = m_backend->universe();
		m_running = true;
		m_poseThread = std::thread(&Provider::poseLoop, this);
		log("ovrn: started, backend %s, %zu controllers\n", m_backend->name(), m_controllers.size());
		return vr::VRInitError_None;
	}

	void
	Cleanup() override
	{
		if (m_running.exchange(false) && m_poseThread.joinable()) {
			m_poseThread.join();
		}
		m_controllers.clear();
		m_hmd.reset();
		if (m_backend) {
			m_backend->stop();
			m_backend.reset();
		}
		restoreCompositorDefaults();
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
		for (auto &c : m_controllers) {
			c->updateInputs();
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
		}
		vr::VREvent_t event;
		while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(event))) {
			if (event.eventType == vr::VREvent_ChaperoneRoomSetupStarting ||
			    event.eventType == vr::VREvent_ChaperoneRoomSetupCommitted) {
				m_backend->roomSetupEvent(event.eventType);
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
	 * - SteamVR's async reprojection on (off by default on NVIDIA under Linux): late frames are moved to the new head
	 *   pose instead of shown again (17:44: 6 frames dropped, 304 reprojected, no "Timed out" fallback; without it 50
	 *   dropped, 60 timed out). For this session only: Cleanup takes the setting out again, so other drivers (Sony's)
	 *   keep SteamVR's default. OVRN_ASYNC=0 leaves SteamVR's setting alone.
	 */
	void
	applyCompositorDefaults()
	{
		const char *fixes = getenv("OVRN_LINUX_FIXES");
		if (fixes == nullptr || strcmp(fixes, "1") != 0) {
			setenv("DISABLE_STEAMVR_LINUX_FIXES", "1", 1);
			log("ovrn: SteamVRLinuxFixes layer off for the compositor (OVRN_LINUX_FIXES=1 keeps it)\n");
		}
		const char *async = getenv("OVRN_ASYNC");
		if (async != nullptr && strcmp(async, "0") == 0) {
			return;
		}
		vr::IVRSettings *settings = vr::VRSettings();
		if (settings == nullptr) {
			return;
		}
		vr::EVRSettingsError err = vr::VRSettingsError_None;
		const bool noted = settings->GetBool(kOwnSection, kAsyncNoteKey, &err) && err == vr::VRSettingsError_None;
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
		log("ovrn: SteamVR async reprojection on for this session (OVRN_ASYNC=0 leaves SteamVR's default)\n");
	}

	void
	restoreCompositorDefaults()
	{
		if (!m_asyncSet || vr::VRSettings() == nullptr) {
			return;
		}
		vr::VRSettings()->RemoveKeyInSection(kAsyncSection, kAsyncKey);
		vr::VRSettings()->RemoveKeyInSection(kOwnSection, kAsyncNoteKey);
		m_asyncSet = false;
		log("ovrn: SteamVR async reprojection setting taken out again (SteamVR's default)\n");
	}

	bool m_asyncSet = false;

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
			for (auto &c : m_controllers) {
				if (c->active()) {
					c->updatePose(now);
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
	std::vector<std::unique_ptr<Controller>> m_controllers;
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
