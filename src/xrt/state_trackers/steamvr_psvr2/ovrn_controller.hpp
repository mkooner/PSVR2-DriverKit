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
	{
		const Pose &g = m_info.grip_in_root;
		m_skeleton.setGripInRoot({(float)g.position.x, (float)g.position.y, (float)g.position.z},
		                         {(float)g.orientation.w, (float)g.orientation.x, (float)g.orientation.y,
		                          (float)g.orientation.z});
	}

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
			noteInputValue(i, m_values[i]);
		}
		m_updates += n > 0 ? 1 : 0;
		if (!m_inputsReported && now_ns() - m_activateNs >= kInputReportAfterNs) {
			reportInputs("after 2 minutes");
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
		in.trigger_proximity = value(m_skelIdx.trigger_near);
		in.grip = value(m_skelIdx.grip);
		in.grip_touch = value(m_skelIdx.grip_touch) > 0.5f;
		in.grip_proximity = value(m_skelIdx.grip_near);
		// the thumb is on one control at a time; the buttons win over the stick (the thumb's edge brushes the stick)
		in.thumb = value(m_skelIdx.button_a_touch) > 0.5f   ? skel::Thumb::ButtonA
		           : value(m_skelIdx.button_b_touch) > 0.5f ? skel::Thumb::ButtonB
		           : value(m_skelIdx.stick_touch) > 0.5f    ? skel::Thumb::Stick
		                                                    : skel::Thumb::None;
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
		m_paths.clear();
		size_t failed = 0;
		for (size_t i = 0; i < descs.size(); i++) {
			m_paths.push_back(descs[i].path);
			if (m_handles[i] == vr::k_ulInvalidInputComponentHandle) {
				log("ovrn: %s controller: input %s not created by SteamVR\n",
				    m_hand == Hand::Left ? "left" : "right", descs[i].path.c_str());
				failed++;
			}
		}
		m_first.assign(descs.size(), 0.0f);
		m_seen.assign(descs.size(), false);
		m_changed.assign(descs.size(), false);
		m_updates = 0;
		m_inputsReported = false;
		m_activateNs = now_ns();
		/*
		 * The compatibility line: the native identity SteamVR keeps, the input profile, and what legacy games
		 * are shown (the profile's legacy binding; a game's own binding in SteamVR's settings still wins, which
		 * the driver can't see).
		 */
		const bool has_haptic = m_haptic != vr::k_ulInvalidInputComponentHandle;
		const bool has_skeleton = m_skeletonHandle != vr::k_ulInvalidInputComponentHandle;
		log("ovrn: %s controller compatibility: type %s, input profile %s, legacy games see %s (%s), headset "
		    "not simulated; %zu of %zu inputs created%s%s\n",
		    m_hand == Hand::Left ? "left" : "right", m_info.controller_type.c_str(),
		    m_info.input_profile.c_str(), legacyDescription(m_info.legacy_mode),
		    m_info.legacy_mode_source.c_str(), descs.size() - failed, descs.size(),
		    has_haptic ? ", haptics" : ", NO haptics", has_skeleton ? ", skeleton" : ", no skeleton");
		return vr::VRInitError_None;
	}

	//! SteamVR deactivates the controller: the input summary, if the 2 minutes weren't up yet.
	void
	deactivate() override
	{
		if (!m_inputsReported && !m_paths.empty()) {
			reportInputs("at shutdown");
		}
	}

private:
	static constexpr int64_t kInputReportAfterNs = 120LL * 1000000000LL;

	static const char *
	legacyDescription(const std::string &mode)
	{
		if (mode == "touch") {
			return "Oculus Touch controllers";
		}
		if (mode == "vive") {
			return "Vive controllers";
		}
		if (mode == "knuckles") {
			return "Index controllers";
		}
		if (mode == "native") {
			return "the PS VR2 Sense (no emulation)";
		}
		return "the profile's default";
	}

	//! Notes whether input @p i ever moved from its first value (RunFrame thread).
	void
	noteInputValue(size_t i, float v)
	{
		if (i >= m_seen.size()) {
			return;
		}
		if (!m_seen[i]) {
			m_seen[i] = true;
			m_first[i] = v;
		} else if (!m_changed[i] && (v - m_first[i] > 0.01f || m_first[i] - v > 0.01f)) {
			m_changed[i] = true;
		}
	}

	/*!
	 * Which inputs changed at least once, and which never did (pressed, touched or moved): an input that never
	 * changes while the controller is used points at a component that isn't reaching SteamVR (or wasn't used).
	 */
	void
	reportInputs(const char *when)
	{
		m_inputsReported = true;
		std::string never;
		size_t changed = 0;
		for (size_t i = 0; i < m_paths.size(); i++) {
			if (m_changed[i]) {
				changed++;
			} else {
				never += (never.empty() ? "" : ", ") + m_paths[i];
			}
		}
		log("ovrn: %s controller inputs %s: %zu of %zu changed in %llu updates; never changed: %s\n",
		    m_hand == Hand::Left ? "left" : "right", when, changed, m_paths.size(),
		    (unsigned long long)m_updates, never.empty() ? "none" : never.c_str());
	}

	//! Which inputs drive the skeleton: trigger value and touch, grip, and the thumb's touches (stick, face buttons).
	struct SkeletonInputs
	{
		int trigger = -1;
		int trigger_touch = -1;
		int trigger_near = -1; //!< l2_sensor / r2_sensor: how near the index finger is
		int grip = -1;
		int grip_touch = -1;
		int grip_near = -1; //!< l1_sensor / r1_sensor: how near the middle finger is
		int stick_touch = -1;
		int button_a_touch = -1; //!< square / cross
		int button_b_touch = -1; //!< triangle / circle
	};

	/*!
	 * The hand skeleton, as Sony's driver has one per Sense (/input/skeleton/left|right, our profile's skeleton
	 * inputs): partial tracking from the Sense's finger sensors (ovrn_hand_skeleton.hpp), both motion ranges
	 * set before the first frame so a game never reads a skeleton without bones.
	 */
	void
	setupSkeleton(const std::vector<InputDesc> &descs)
	{
		m_skelIdx = SkeletonInputs{};
		for (size_t i = 0; i < descs.size(); i++) {
			const std::string &p = descs[i].path;
			if (p == "/input/l2/value" || p == "/input/r2/value") {
				m_skelIdx.trigger = (int)i;
			} else if (p == "/input/l2/touch" || p == "/input/r2/touch") {
				m_skelIdx.trigger_touch = (int)i;
			} else if (p == "/input/l2_sensor/value" || p == "/input/r2_sensor/value") {
				m_skelIdx.trigger_near = (int)i;
			} else if (p == "/input/l1/value" || p == "/input/r1/value") {
				m_skelIdx.grip = (int)i;
			} else if (p == "/input/l1/touch" || p == "/input/r1/touch") {
				m_skelIdx.grip_touch = (int)i;
			} else if (p == "/input/l1_sensor/value" || p == "/input/r1_sensor/value") {
				m_skelIdx.grip_near = (int)i;
			} else if (p == "/input/left_stick/touch" || p == "/input/right_stick/touch") {
				m_skelIdx.stick_touch = (int)i;
			} else if (p == "/input/square/touch" || p == "/input/cross/touch") {
				m_skelIdx.button_a_touch = (int)i;
			} else if (p == "/input/triangle/touch" || p == "/input/circle/touch") {
				m_skelIdx.button_b_touch = (int)i;
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
		log("ovrn: %s hand skeleton from the Sense's finger sensors: %s\n", left ? "left" : "right",
		    m_skeleton.describe().c_str());
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
	// The input summary (RunFrame thread, and Activate / Deactivate on the same thread).
	std::vector<std::string> m_paths;
	std::vector<float> m_first;
	std::vector<bool> m_seen;
	std::vector<bool> m_changed;
	uint64_t m_updates = 0;
	bool m_inputsReported = false;
	int64_t m_activateNs = 0;
};

} // namespace ovrn
