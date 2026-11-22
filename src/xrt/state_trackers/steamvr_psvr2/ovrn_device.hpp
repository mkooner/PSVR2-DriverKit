// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Common base of the driver's tracked devices (after ALVR's TrackedDevice, alvr_server/TrackedDevice.{h,cpp}).
 * @ingroup st_ovrn
 */

#pragma once

#include "ovrn_backend.hpp"
#include "ovrn_log.hpp"

#include "os/os_time.h"
#include "util/u_startup_log.h"

#include <atomic>
#include <mutex>
#include <string>

namespace ovrn {

class TrackedDevice : public vr::ITrackedDeviceServerDriver
{
public:
	TrackedDevice(vr::ETrackedDeviceClass device_class, std::string serial)
	    : m_class(device_class), m_serial(std::move(serial))
	{
		m_lastPose = identityPose();
		m_lastPose.poseIsValid = false;
		m_lastPose.deviceIsConnected = false;
		m_lastPose.result = vr::TrackingResult_Uninitialized;
	}

	//! TrackedDeviceAdded; SteamVR then calls Activate.
	bool
	registerDevice()
	{
		if (!vr::VRServerDriverHost()->TrackedDeviceAdded(m_serial.c_str(), m_class, this)) {
			log("ovrn: TrackedDeviceAdded failed for %s\n", m_serial.c_str());
			return false;
		}
		return true;
	}

	const std::string &
	serial() const
	{
		return m_serial;
	}

	bool
	active() const
	{
		return m_objectId.load() != vr::k_unTrackedDeviceIndexInvalid;
	}

	//! Samples the backend and hands the pose for now to SteamVR (called from the provider's pose thread).
	virtual void
	updatePose(int64_t now_ns) = 0;

	//! The room's universe changed (RunFrame thread).
	void
	setUniverse(uint64_t universe)
	{
		if (active()) {
			vr::VRProperties()->SetUint64Property(m_container, vr::Prop_CurrentUniverseId_Uint64, universe);
		}
	}

	vr::PropertyContainerHandle_t
	container() const
	{
		return m_container;
	}

	// ITrackedDeviceServerDriver

	vr::EVRInitError
	Activate(uint32_t object_id) final
	{
		const int64_t begin_ns = os_monotonic_get_ns();
		u_startup_mark("%s: Activate begins", startupLabel().c_str());
		m_container = vr::VRProperties()->TrackedDeviceToPropertyContainer(object_id);
		vr::EVRInitError err = activate(object_id);
		if (err == vr::VRInitError_None) {
			m_objectId = object_id; // poses flow from here on
		}
		u_startup_mark("%s: Activate done (%.1f ms, %s)", startupLabel().c_str(),
		               (double)(os_monotonic_get_ns() - begin_ns) / 1e6,
		               err == vr::VRInitError_None ? "ok" : "failed");
		return err;
	}

	/*!
	 * OpenVR: the device must not call SteamVR after Deactivate returns. The pose thread's calls for this device
	 * run under m_activeMutex (withActive), so taking it here waits for one in flight, as Valve's sample drivers
	 * wait for their pose thread in Deactivate; then the device frees its own resources (deactivate). Cleanup
	 * comes after.
	 */
	void
	Deactivate() final
	{
		{
			std::lock_guard<std::mutex> lock(m_activeMutex);
			m_objectId = vr::k_unTrackedDeviceIndexInvalid;
			m_container = vr::k_ulInvalidPropertyContainer;
		}
		deactivate();
	}

	void
	EnterStandby() override
	{}

	void *
	GetComponent(const char *) override
	{
		return nullptr;
	}

	void
	DebugRequest(const char *, char *response, uint32_t size) override
	{
		if (size >= 1) {
			response[0] = '\0';
		}
	}

	vr::DriverPose_t
	GetPose() final
	{
		std::lock_guard<std::mutex> lock(m_poseMutex);
		return m_lastPose;
	}

protected:
	//! Properties and components; @p object_id is the device's index.
	virtual vr::EVRInitError
	activate(uint32_t object_id) = 0;

	//! Device-specific resources to free at Deactivate, after the last call to SteamVR for this device.
	virtual void
	deactivate()
	{}

	/*!
	 * Runs @p fn(index, container) while the device is active, under m_activeMutex: for calls to SteamVR from
	 * threads other than SteamVR's own (the pose thread), so none is made after Deactivate returns.
	 */
	template <typename Fn>
	bool
	withActive(Fn &&fn)
	{
		std::lock_guard<std::mutex> lock(m_activeMutex);
		const uint32_t id = m_objectId.load();
		if (id == vr::k_unTrackedDeviceIndexInvalid) {
			return false;
		}
		fn(id, m_container);
		return true;
	}

	void
	submitPose(const vr::DriverPose_t &pose)
	{
		markPoseMilestones(pose);
		{
			std::lock_guard<std::mutex> lock(m_poseMutex);
			m_lastPose = pose;
		}
		withActive([&](uint32_t id, vr::PropertyContainerHandle_t) {
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id, pose, sizeof(vr::DriverPose_t));
		});
	}

	//! "headset", "controller <serial>" or "hand <serial>", for the startup milestones.
	std::string
	startupLabel() const
	{
		switch (m_class) {
		case vr::TrackedDeviceClass_HMD: return "headset";
		case vr::TrackedDeviceClass_Controller: return "controller " + m_serial;
		default: return "device " + m_serial;
		}
	}

	/*!
	 * Startup milestones from the poses sent (pose thread only): the first pose, the first valid one, each tracking
	 * result the first time SteamVR is given it, and the first 2 s of valid poses in a row ("steady"). At most a
	 * handful of lines per device, never one per pose.
	 */
	void
	markPoseMilestones(const vr::DriverPose_t &pose)
	{
		if (!m_startupFirstPose) {
			m_startupFirstPose = true;
			u_startup_mark("%s: first pose sent", startupLabel().c_str());
		}
		// Results are 1, 100, 101, 200, 201, 300: one bit each (hundreds * 8 + units).
		const unsigned r = (unsigned)pose.result;
		const unsigned bit = r / 100 * 8 + r % 100;
		if (bit < 32 && r % 100 < 8 && (m_startupResultsSeen & (1u << bit)) == 0) {
			m_startupResultsSeen |= 1u << bit;
			u_startup_mark("%s: tracking result %s first sent (pose %s)", startupLabel().c_str(), resultName(pose.result),
			               pose.poseIsValid ? "valid" : "not valid");
		}
		const bool ok = pose.poseIsValid && pose.result == vr::TrackingResult_Running_OK;
		const int64_t now = os_monotonic_get_ns();
		if (ok && !m_startupFirstValid) {
			m_startupFirstValid = true;
			u_startup_mark("%s: first valid pose (Running_OK)", startupLabel().c_str());
		}
		if (!ok) {
			m_startupValidSince = 0;
		} else if (m_startupValidSince == 0) {
			m_startupValidSince = now;
		} else if (!m_startupSteady && now - m_startupValidSince >= 2 * 1000 * 1000 * 1000LL) {
			m_startupSteady = true;
			u_startup_mark("%s: steady (2 s of valid poses in a row)", startupLabel().c_str());
		}
	}

	static const char *
	resultName(vr::ETrackingResult r)
	{
		switch (r) {
		case vr::TrackingResult_Uninitialized: return "Uninitialized";
		case vr::TrackingResult_Calibrating_InProgress: return "Calibrating_InProgress";
		case vr::TrackingResult_Calibrating_OutOfRange: return "Calibrating_OutOfRange";
		case vr::TrackingResult_Running_OK: return "Running_OK";
		case vr::TrackingResult_Running_OutOfRange: return "Running_OutOfRange";
		case vr::TrackingResult_Fallback_RotationOnly: return "Fallback_RotationOnly";
		default: return "other";
		}
	}

	bool m_startupFirstPose = false, m_startupFirstValid = false, m_startupSteady = false;
	uint32_t m_startupResultsSeen = 0;
	int64_t m_startupValidSince = 0;

	//! The identity every device reports: names, type, profile, model, the backend's extra properties, the universe.
	void
	setIdentity(const DeviceInfo &info, uint64_t universe)
	{
		vr::CVRPropertyHelpers *props = vr::VRProperties();
		props->SetStringProperty(m_container, vr::Prop_TrackingSystemName_String, "monado_psvr2");
		props->SetStringProperty(m_container, vr::Prop_ManufacturerName_String, info.manufacturer.c_str());
		props->SetStringProperty(m_container, vr::Prop_ModelNumber_String, info.model.c_str());
		props->SetStringProperty(m_container, vr::Prop_SerialNumber_String, info.serial.c_str());
		if (!info.controller_type.empty()) {
			props->SetStringProperty(m_container, vr::Prop_ControllerType_String, info.controller_type.c_str());
		}
		if (!info.input_profile.empty()) {
			props->SetStringProperty(m_container, vr::Prop_InputProfilePath_String, info.input_profile.c_str());
		}
		if (!info.render_model.empty()) {
			props->SetStringProperty(m_container, vr::Prop_RenderModelName_String, info.render_model.c_str());
		}
		for (const auto &p : info.string_props) {
			props->SetStringProperty(m_container, (vr::ETrackedDeviceProperty)p.first, p.second.c_str());
		}
		for (const auto &p : info.bool_props) {
			props->SetBoolProperty(m_container, (vr::ETrackedDeviceProperty)p.first, p.second);
		}
		for (const auto &p : info.int_props) {
			props->SetInt32Property(m_container, (vr::ETrackedDeviceProperty)p.first, p.second);
		}
		for (const auto &p : info.float_props) {
			props->SetFloatProperty(m_container, (vr::ETrackedDeviceProperty)p.first, p.second);
		}
		props->SetUint64Property(m_container, vr::Prop_CurrentUniverseId_Uint64, universe);
	}

	//! The room transform, the same for every device, so SteamVR applies it exactly once to each.
	static void
	setWorld(vr::DriverPose_t &pose, const WorldTransform &t)
	{
		pose.qWorldFromDriverRotation = {t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z};
		pose.vecWorldFromDriverTranslation[0] = t.translation.x;
		pose.vecWorldFromDriverTranslation[1] = t.translation.y;
		pose.vecWorldFromDriverTranslation[2] = t.translation.z;
	}

	//! @p v rotated into the frame of @p q (q^-1 v q): OpenVR wants angular velocities in the device's own frame.
	static Vec3
	rotateInverse(const Quat &q, const Vec3 &v)
	{
		const double x = -q.x, y = -q.y, z = -q.z, w = q.w;
		const double tx = 2 * (y * v.z - z * v.y), ty = 2 * (z * v.x - x * v.z), tz = 2 * (x * v.y - y * v.x);
		Vec3 r;
		r.x = v.x + w * tx + (y * tz - z * ty);
		r.y = v.y + w * ty + (z * tx - x * tz);
		r.z = v.z + w * tz + (x * ty - y * tx);
		return r;
	}

	//! Zero pose with identity quaternions (a zeroed quaternion is invalid).
	static vr::DriverPose_t
	identityPose()
	{
		vr::DriverPose_t p = {};
		p.qWorldFromDriverRotation.w = 1.0;
		p.qDriverFromHeadRotation.w = 1.0;
		p.qRotation.w = 1.0;
		return p;
	}

	const vr::ETrackedDeviceClass m_class;
	const std::string m_serial;
	std::atomic<uint32_t> m_objectId{vr::k_unTrackedDeviceIndexInvalid};
	vr::PropertyContainerHandle_t m_container = vr::k_ulInvalidPropertyContainer;

private:
	std::mutex m_poseMutex;
	vr::DriverPose_t m_lastPose;
	//! Held by Deactivate and around every call to SteamVR from another thread (withActive).
	std::mutex m_activeMutex;
};

} // namespace ovrn
