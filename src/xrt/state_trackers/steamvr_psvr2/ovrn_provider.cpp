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
		m_backend->runFrame(now_ns());
		for (auto &c : m_controllers) {
			c->updateInputs();
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
