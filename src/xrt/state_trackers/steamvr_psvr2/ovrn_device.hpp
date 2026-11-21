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
		m_container = vr::VRProperties()->TrackedDeviceToPropertyContainer(object_id);
		vr::EVRInitError err = activate(object_id);
		if (err == vr::VRInitError_None) {
			m_objectId = object_id; // poses flow from here on
		}
		return err;
	}

	void
	Deactivate() final
	{
		m_objectId = vr::k_unTrackedDeviceIndexInvalid;
		m_container = vr::k_ulInvalidPropertyContainer;
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

	void
	submitPose(const vr::DriverPose_t &pose)
	{
		{
			std::lock_guard<std::mutex> lock(m_poseMutex);
			m_lastPose = pose;
		}
		const uint32_t id = m_objectId.load();
		if (id != vr::k_unTrackedDeviceIndexInvalid) {
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id, pose, sizeof(vr::DriverPose_t));
		}
	}

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
};

} // namespace ovrn
