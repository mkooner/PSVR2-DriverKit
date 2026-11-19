// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  A hand controller: identity, pose, inputs and haptics, all from the backend (after ALVR's Controller.cpp).
 * @ingroup st_ovrn
 */

#pragma once

#include "ovrn_device.hpp"
#include "ovrn_hand_skeleton.hpp"

#include <cstring>
#include <vector>

namespace ovrn {

class Controller final : public TrackedDevice
{
public:
	Controller(Backend *backend, Hand hand)
	    : TrackedDevice(vr::TrackedDeviceClass_Controller, backend->controllerInfo(hand).serial), m_backend(backend),
	      m_hand(hand), m_info(backend->controllerInfo(hand)), m_skeleton(hand == Hand::Left)
	{}

	Hand
	hand() const
	{
		return m_hand;
	}

	//! The pose for now with its velocities: SteamVR predicts controllers itself (as ALVR, Controller.cpp:202-268).
	void
	updatePose(int64_t now_ns) override
	{
		TrackedPose t;
		vr::DriverPose_t pose = identityPose();
		pose.deviceIsConnected = true;
		setWorld(pose, m_backend->worldFromDriver());
		if (!m_backend->controllerPose(m_hand, now_ns, &t) || !t.orientation_valid) {
			pose.poseIsValid = false;
			pose.result = vr::TrackingResult_Uninitialized;
			submitPose(pose);
			return;
		}
		pose.poseIsValid = true;
		pose.result = t.position_tracked ? vr::TrackingResult_Running_OK : vr::TrackingResult_Fallback_RotationOnly;
		const Quat &q = t.pose.orientation;
		pose.qRotation = {q.w, q.x, q.y, q.z};
		pose.vecPosition[0] = t.pose.position.x;
		pose.vecPosition[1] = t.pose.position.y;
		pose.vecPosition[2] = t.pose.position.z;
		pose.vecVelocity[0] = t.linear_velocity.x;
		pose.vecVelocity[1] = t.linear_velocity.y;
		pose.vecVelocity[2] = t.linear_velocity.z;
		// OpenVR wants the angular velocity in the device's own frame: q^-1 * w.
		const Vec3 w = rotateInverse(q, t.angular_velocity);
		pose.vecAngularVelocity[0] = w.x;
		pose.vecAngularVelocity[1] = w.y;
		pose.vecAngularVelocity[2] = w.z;
		submitPose(pose);
	}

	//! The inputs' current values to SteamVR (RunFrame thread, after the backend's runFrame).
	void
	updateInputs()
	{
		if (!active() || m_handles.empty()) {
			return;
		}
		m_values.resize(m_handles.size());
		const size_t n = m_backend->controllerInputs(m_hand, m_values.data(), m_values.size());
		for (size_t i = 0; i < n && i < m_handles.size(); i++) {
			if (m_kinds[i] == InputKind::Boolean) {
				vr::VRDriverInput()->UpdateBooleanComponent(m_handles[i], m_values[i] > 0.5f, 0.0);
			} else {
				vr::VRDriverInput()->UpdateScalarComponent(m_handles[i], m_values[i], 0.0);
			}
		}
		updateSkeleton(n);
	}

	//! The finger state for the skeleton from this frame's input values (public for the tests).
	skel::HandInput
	handInput(size_t n) const
	{
		skel::HandInput in;
		auto value = [&](int idx) { return idx >= 0 && (size_t)idx < n ? m_values[idx] : 0.0f; };
		in.trigger = value(m_skelIdx.trigger);
		in.trigger_touch = value(m_skelIdx.trigger_touch) > 0.5f || in.trigger > 0.0f;
		in.grip = value(m_skelIdx.grip);
		for (int idx : m_skelIdx.thumb) {
			in.thumb_touch = in.thumb_touch || value(idx) > 0.5f;
		}
		return in;
	}

	void
	haptic(float duration_s, float frequency_hz, float amplitude)
	{
		m_backend->haptic(m_hand, duration_s, frequency_hz, amplitude);
	}

protected:
	vr::EVRInitError
	activate(uint32_t) override
	{
		setIdentity(m_info, m_backend->universe());
		vr::VRProperties()->SetInt32Property(m_container, vr::Prop_ControllerRoleHint_Int32,
		                                     m_hand == Hand::Left ? vr::TrackedControllerRole_LeftHand
		                                                          : vr::TrackedControllerRole_RightHand);
		std::string haptic_path;
		const std::vector<InputDesc> descs = m_backend->controllerInputDescs(m_hand, &haptic_path);
		m_handles.clear();
		m_kinds.clear();
		for (const InputDesc &d : descs) {
			vr::VRInputComponentHandle_t h = vr::k_ulInvalidInputComponentHandle;
			if (d.kind == InputKind::Boolean) {
				vr::VRDriverInput()->CreateBooleanComponent(m_container, d.path.c_str(), &h);
			} else {
				vr::VRDriverInput()->CreateScalarComponent(m_container, d.path.c_str(), &h, vr::VRScalarType_Absolute,
				                                           d.kind == InputKind::ScalarTwoSided
				                                               ? vr::VRScalarUnits_NormalizedTwoSided
				                                               : vr::VRScalarUnits_NormalizedOneSided);
			}
			m_handles.push_back(h);
			m_kinds.push_back(d.kind);
		}
		if (!haptic_path.empty()) {
			vr::VRDriverInput()->CreateHapticComponent(m_container, haptic_path.c_str(), &m_haptic);
		}
		setupSkeleton(descs);
		log("ovrn: %s controller %s (%s), %zu inputs%s\n", m_hand == Hand::Left ? "left" : "right",
		    m_info.model.c_str(), m_info.serial.c_str(), m_handles.size(), haptic_path.empty() ? "" : ", haptics");
		return vr::VRInitError_None;
	}

private:
	//! Which inputs drive the skeleton: trigger value and touch, grip, and the thumb's touches (stick, face buttons).
	struct SkeletonInputs
	{
		int trigger = -1;
		int trigger_touch = -1;
		int grip = -1;
		std::vector<int> thumb;
	};

	/*!
	 * The hand skeleton, as Sony's driver has one per Sense (/input/skeleton/left|right, our profile's skeleton
	 * inputs): partial tracking from the buttons (ALVR's bone tables, ovrn_hand_skeleton.hpp), both motion ranges
	 * set before the first frame so a game never reads a skeleton without bones.
	 */
	void
	setupSkeleton(const std::vector<InputDesc> &descs)
	{
		m_skelIdx = SkeletonInputs{};
		for (size_t i = 0; i < descs.size(); i++) {
			const std::string &p = descs[i].path;
			const bool stick_touch = p == "/input/left_stick/touch" || p == "/input/right_stick/touch";
			if (p == "/input/l2/value" || p == "/input/r2/value") {
				m_skelIdx.trigger = (int)i;
			} else if (p == "/input/l2/touch" || p == "/input/r2/touch") {
				m_skelIdx.trigger_touch = (int)i;
			} else if (p == "/input/l1/value" || p == "/input/r1/value") {
				m_skelIdx.grip = (int)i;
			} else if (stick_touch || p == "/input/square/touch" || p == "/input/triangle/touch" ||
			           p == "/input/cross/touch" || p == "/input/circle/touch") {
				m_skelIdx.thumb.push_back((int)i);
			}
		}
		const bool left = m_hand == Hand::Left;
		const vr::EVRInputError err = vr::VRDriverInput()->CreateSkeletonComponent(
		    m_container, left ? "/input/skeleton/left" : "/input/skeleton/right",
		    left ? "/skeleton/hand/left" : "/skeleton/hand/right", "/pose/raw", vr::VRSkeletalTracking_Partial,
		    nullptr, 0, &m_skeletonHandle);
		if (err != vr::VRInputError_None) {
			m_skeletonHandle = vr::k_ulInvalidInputComponentHandle;
			log("ovrn: %s hand skeleton not created (error %d)\n", left ? "left" : "right", (int)err);
			return;
		}
		m_skeleton.update(skel::HandInput{});
		submitSkeleton();
		log("ovrn: %s hand skeleton (from trigger, grip and thumb touches)\n", left ? "left" : "right");
	}

	void
	updateSkeleton(size_t n)
	{
		if (m_skeletonHandle == vr::k_ulInvalidInputComponentHandle) {
			return;
		}
		m_skeleton.update(handInput(n));
		submitSkeleton();
	}

	void
	submitSkeleton()
	{
		vr::VRBoneTransform_t bones[skel::kBoneCount];
		m_skeleton.pose(true, bones);
		vr::VRDriverInput()->UpdateSkeletonComponent(m_skeletonHandle, vr::VRSkeletalMotionRange_WithController,
		                                             bones, skel::kBoneCount);
		m_skeleton.pose(false, bones);
		vr::VRDriverInput()->UpdateSkeletonComponent(m_skeletonHandle, vr::VRSkeletalMotionRange_WithoutController,
		                                             bones, skel::kBoneCount);
	}

	Backend *const m_backend;
	const Hand m_hand;
	const DeviceInfo m_info;
	std::vector<vr::VRInputComponentHandle_t> m_handles; //!< RunFrame thread (and Activate)
	std::vector<InputKind> m_kinds;
	std::vector<float> m_values;
	vr::VRInputComponentHandle_t m_haptic = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t m_skeletonHandle = vr::k_ulInvalidInputComponentHandle;
	SkeletonInputs m_skelIdx;
	skel::HandSkeleton m_skeleton;
};

} // namespace ovrn
