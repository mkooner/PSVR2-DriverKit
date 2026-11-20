// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  A tracked bare hand as a SteamVR controller: its own device type (playstation_vr2_hand), the palm as the
 *         pose, the full hand skeleton from the tracked joints, pinch and grab as the trigger and grip.
 *
 * The hand is its own device next to the Sense controllers, never pretending to be another controller: SteamVR apps
 * that support skeletal input get the real joints through /input/skeleton/left|right (VRSkeletalTracking_Full, both
 * motion ranges the same, as no controller is held), and the profile's remapping lets apps with only Index or Touch
 * bindings use the trigger and grip. The hand stands in for the Sense controller of its side: it is connected only
 * while that controller isn't there.
 *
 * Its pose is OpenXR's palm joint (-Z along the fingers, +Y out of the back of the hand: as a controller held pointing
 * forward), the skeleton's root. Poses come from the backend's hand tracking, which answers from its latest result:
 * nothing here waits for a camera frame.
 * @ingroup st_ovrn
 */

#pragma once

#include "ovrn_device.hpp"
#include "ovrn_hand_gestures.hpp"

#include <string>

namespace ovrn {

class HandDevice final : public TrackedDevice
{
public:
	HandDevice(Backend *backend, Hand hand)
	    : TrackedDevice(vr::TrackedDeviceClass_Controller, serialFor(backend, hand)), m_backend(backend), m_hand(hand),
	      m_joints(skel::detail::reference_rest(hand == Hand::Left, &m_restSource), hand == Hand::Left)
	{}

	Hand
	hand() const
	{
		return m_hand;
	}

	//! Whether the hand stands in for its controller now: hand tracking is there and the Sense of that side isn't.
	bool
	inUse() const
	{
		return m_backend->hasHandTracking() && !m_backend->hasController(m_hand);
	}

	//! The palm's pose (pose thread). No velocities: SteamVR then shows the hand where it was last tracked.
	void
	updatePose(int64_t now_ns) override
	{
		vr::DriverPose_t pose = identityPose();
		setWorld(pose, m_backend->worldFromDriver());
		pose.deviceIsConnected = inUse();
		HandJoints j;
		if (!pose.deviceIsConnected || !m_backend->handJoints(m_hand, now_ns, &j) || !j.active || !j.joint_valid[0]) {
			pose.poseIsValid = false;
			pose.result = pose.deviceIsConnected ? vr::TrackingResult_Running_OutOfRange
			                                     : vr::TrackingResult_Uninitialized;
			submitPose(pose);
			return;
		}
		const Pose &palm = j.joint[0];
		pose.poseIsValid = true;
		pose.result = vr::TrackingResult_Running_OK;
		pose.qRotation = {palm.orientation.w, palm.orientation.x, palm.orientation.y, palm.orientation.z};
		pose.vecPosition[0] = palm.position.x;
		pose.vecPosition[1] = palm.position.y;
		pose.vecPosition[2] = palm.position.z;
		submitPose(pose);
	}

	//! The skeleton and the gesture inputs (RunFrame thread, once per frame). A lost hand keeps its last skeleton.
	void
	updateInputs(int64_t now_ns)
	{
		if (!active() || m_skeleton == vr::k_ulInvalidInputComponentHandle) {
			return;
		}
		HandJoints j;
		if (!inUse() || !m_backend->handJoints(m_hand, now_ns, &j) || !j.active || !j.joint_valid[0]) {
			m_tracked = false;
			setGestures(skel::Gestures{});
			return;
		}
		const skel::Joints in_root = inPalm(j);
		vr::VRBoneTransform_t bones[skel::kBoneCount];
		if (m_joints.convert(in_root, bones)) {
			vr::VRDriverInput()->UpdateSkeletonComponent(m_skeleton, vr::VRSkeletalMotionRange_WithController, bones,
			                                             skel::kBoneCount);
			vr::VRDriverInput()->UpdateSkeletonComponent(m_skeleton, vr::VRSkeletalMotionRange_WithoutController,
			                                             bones, skel::kBoneCount);
			m_updates++;
		}
		if (!m_tracked) {
			log("ovrn: %s hand tracked\n", m_hand == Hand::Left ? "left" : "right");
		}
		m_tracked = true;
		setGestures(skel::gestures(in_root, j.radius));
	}

	//! How many skeletons SteamVR got (for the tests and the log).
	uint64_t
	skeletonUpdates() const
	{
		return m_updates;
	}

protected:
	vr::EVRInitError
	activate(uint32_t) override
	{
		setIdentity(info(), m_backend->universe());
		vr::VRProperties()->SetInt32Property(m_container, vr::Prop_ControllerRoleHint_Int32,
		                                     m_hand == Hand::Left ? vr::TrackedControllerRole_LeftHand
		                                                          : vr::TrackedControllerRole_RightHand);
		vr::IVRDriverInput *in = vr::VRDriverInput();
		in->CreateScalarComponent(m_container, "/input/trigger/value", &m_trigger, vr::VRScalarType_Absolute,
		                          vr::VRScalarUnits_NormalizedOneSided);
		in->CreateBooleanComponent(m_container, "/input/trigger/click", &m_triggerClick);
		in->CreateScalarComponent(m_container, "/input/grip/value", &m_grip, vr::VRScalarType_Absolute,
		                          vr::VRScalarUnits_NormalizedOneSided);
		in->CreateBooleanComponent(m_container, "/input/grip/click", &m_gripClick);
		const bool left = m_hand == Hand::Left;
		const vr::EVRInputError err = in->CreateSkeletonComponent(
		    m_container, left ? "/input/skeleton/left" : "/input/skeleton/right",
		    left ? "/skeleton/hand/left" : "/skeleton/hand/right", "/pose/raw", vr::VRSkeletalTracking_Full, nullptr,
		    0, &m_skeleton);
		if (err != vr::VRInputError_None) {
			m_skeleton = vr::k_ulInvalidInputComponentHandle;
			log("ovrn: %s tracked hand: skeleton not created (error %d)\n", left ? "left" : "right", (int)err);
		} else {
			// Both motion ranges hold a hand from the start (the reference hand at rest) until it's tracked.
			vr::VRBoneTransform_t bones[skel::kBoneCount];
			m_joints.convert(m_joints.openxr_frames_at_rest(), bones);
			in->UpdateSkeletonComponent(m_skeleton, vr::VRSkeletalMotionRange_WithController, bones, skel::kBoneCount);
			in->UpdateSkeletonComponent(m_skeleton, vr::VRSkeletalMotionRange_WithoutController, bones,
			                            skel::kBoneCount);
		}
		log("ovrn: %s tracked hand (%s), full skeleton from the cameras' hand tracking; bones from %s\n",
		    left ? "left" : "right", m_serial.c_str(), m_restSource.c_str());
		return vr::VRInitError_None;
	}

private:
	static std::string
	serialFor(Backend *backend, Hand hand)
	{
		return backend->headInfo().serial + (hand == Hand::Left ? "-hand-left" : "-hand-right");
	}

	DeviceInfo
	info() const
	{
		DeviceInfo d;
		d.manufacturer = m_backend->headInfo().manufacturer;
		d.model = m_hand == Hand::Left ? "PS VR2 tracked hand (left)" : "PS VR2 tracked hand (right)";
		d.serial = m_serial;
		d.controller_type = "playstation_vr2_hand";
		d.input_profile = "{monado_psvr2}/input/playstation_vr2_hand_profile.json";
		return d;
	}

	//! The joints in the palm's frame (the skeleton's root).
	static skel::Joints
	inPalm(const HandJoints &j)
	{
		using namespace skel::detail;
		auto q = [](const Quat &r) { return normalized(Q{(float)r.w, (float)r.x, (float)r.y, (float)r.z}); };
		auto v = [](const Vec3 &p) { return V{(float)p.x, (float)p.y, (float)p.z}; };
		const Q palm = q(j.joint[0].orientation);
		const Q inv{palm.w, -palm.x, -palm.y, -palm.z};
		const V origin = v(j.joint[0].position);
		skel::Joints out;
		for (int i = 0; i < skel::kJointCount; i++) {
			out.pos[i] = rotate(inv, v(j.joint[i].position) - origin);
			out.rot[i] = normalized(inv * q(j.joint[i].orientation));
			out.valid[i] = j.joint_valid[i];
		}
		return out;
	}

	void
	setGestures(const skel::Gestures &g)
	{
		m_triggerOn = skel::click(g.pinch, m_triggerOn);
		m_gripOn = skel::click(g.grab, m_gripOn);
		vr::IVRDriverInput *in = vr::VRDriverInput();
		in->UpdateScalarComponent(m_trigger, g.pinch, 0.0);
		in->UpdateBooleanComponent(m_triggerClick, m_triggerOn, 0.0);
		in->UpdateScalarComponent(m_grip, g.grab, 0.0);
		in->UpdateBooleanComponent(m_gripClick, m_gripOn, 0.0);
	}

	Backend *const m_backend;
	const Hand m_hand;
	std::string m_restSource;
	const skel::JointSkeleton m_joints;
	vr::VRInputComponentHandle_t m_skeleton = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t m_trigger = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t m_triggerClick = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t m_grip = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t m_gripClick = vr::k_ulInvalidInputComponentHandle;
	bool m_triggerOn = false, m_gripOn = false, m_tracked = false;
	uint64_t m_updates = 0;
};

} // namespace ovrn
