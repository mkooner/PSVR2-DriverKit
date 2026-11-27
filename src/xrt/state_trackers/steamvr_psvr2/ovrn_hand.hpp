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
 *
 * It presents itself as SteamVR's own tracked hands do (Valve's vrlink driver, svl_hand_interaction_augmented): hand
 * priority -1 in the profile, the hand tracking status icons (from SteamVR's vrlink resources, referenced in place),
 * /pose/raw, /pose/grip and /pose/tip made by the driver (tip: the dashboard's laser, from the index knuckle along the
 * hand), per-finger pinches, and a system button on each hand: an open palm toward the face, raised, held about 1 s (with
 * MediaPipe's gestures; without them, the palm toward the face with an index pinch held), which the dashboard binding
 * uses to open and close the dashboard (Valve's has it on the left hand only).
 * @ingroup st_ovrn
 */

#pragma once

#include "ovrn_device.hpp"
#include "ovrn_hand_gestures.hpp"
#include "ovrn_gesture_state.hpp"
#include "ovrn_gesture_rules.hpp"
#include "ovrn_gesture_rules_default.hpp"
#include "ovrn_microgestures.hpp"

#include <cstdlib>
#include <string>

namespace ovrn {

class HandDevice final : public TrackedDevice
{
public:
	HandDevice(Backend *backend, Hand hand)
	    : TrackedDevice(vr::TrackedDeviceClass_Controller, serialFor(backend, hand)), m_backend(backend), m_hand(hand),
	      m_joints(skel::detail::reference_rest(hand == Hand::Left, &m_restSource), hand == Hand::Left),
	      m_mp(gestureConfig()), m_mpDebug(envFlag("OVRN_GESTURE_DEBUG"))
	{}

	~HandDevice()
	{
		if (s_systemHolder == this) {
			s_systemHolder = nullptr;
		}
	}

	Hand
	hand() const
	{
		return m_hand;
	}

	//! Whether the hand stands in for its controller now: hand tracking is there and the Sense of that side isn't.
	bool
	inUse() const
	{
		return m_backend->hasHandTracking() && !m_backend->controllerConnected(m_hand);
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
		gestureLog(now_ns);
		m_stat.frames++;
		if (!inUse() || !m_backend->handJoints(m_hand, now_ns, &j) || !j.active || !j.joint_valid[0]) {
			m_tracked = false;
			logGestureEvents(m_mp.reset("hand not tracked"), now_ns);
			setHandGestures(nullptr);
			releaseFormulaGestures();
			ruleChanges(m_rules.reset(now_ns, "hand not tracked", m_ruleChanges, kRuleChanges), &sample_none());
			setGestures(skel::Gestures{});
			// A lost hand lets go of the gesture's system press (the headset button still works: sendSystem).
			m_systemPinch = false;
			m_systemSinceNs = 0;
			m_systemOn = systemPress(false);
			m_systemGesture.reset();
			sendSystem();
			return;
		}
		m_stat.tracked++;
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
		const skel::Gestures g = skel::gestures(in_root, j.radius);
		updateTip(in_root);
		updatePoke(in_root);
		/*
		 * MediaPipe's gesture measurements when the hand tracker has them (MERCURY_MEDIAPIPE_GESTURES): pinch and
		 * fist from them, through the debounced state machine. While they're on but this frame's is invalid (hand
		 * doubtful, old), the gestures are released rather than switching to the joint rules and back. Without
		 * them (off, models missing), the joint-based gestures as before.
		 */
		gest::Sample sample;
		const bool mp_on = m_backend->handGestures(m_hand, &sample);
		if (mp_on) {
			// The tracker's profile decides the gesture tuning; a change (start, switch) starts the gestures afresh.
			const std::string profile = m_backend->handTrackingProfile();
			if (profile != m_gestureProfile) {
				m_gestureProfile = profile;
				logGestureEvents(m_mp.reset("hand tracking profile changed"), now_ns);
				m_mp = gest::Machine(gestureConfig(profile));
				m_systemGesture = gest::SystemGesture(systemGestureConfig(profile));
				log("ovrn: %s hand gestures: tuning for the '%s' hand tracking profile\n",
				    m_hand == Hand::Left ? "left" : "right", profile.empty() ? "default" : profile.c_str());
			}
		}
		if (mp_on) {
			/*
			 * The calibrated rules (fitted on labelled recordings; ovrn_gesture_rules.hpp) on this frame's joint
			 * features: an enabled fist rule decides the fist, an active point / claw rules out a pinch. The pinch
			 * itself stays the custom mapping's (the joints don't carry it on these cameras).
			 */
			ruleChanges(m_rules.update(sample.features, sample.valid && sample.features_valid, sample.confidence,
			                           sample.timestamp_ns, m_ruleChanges, kRuleChanges),
			            &sample);
			sample.rule_fist_decides = m_rules.decides(gest::rules::Output::Grip);
			sample.rule_fist = m_rules.activeOutput(gest::rules::Output::Grip);
			sample.rule_block_pinch = m_rules.activeOutput(gest::rules::Output::BlockPinch);
			/*
			 * The formula pinch (thumb tip to index tip) decides whenever this frame has the joint measurements, or
			 * while its pinch is held through a short dropout (the rules' grace time); otherwise the fallback (the
			 * image detector if installed, else the gesture mapping) covers the frame.
			 */
			sample.rule_pinch = m_rules.activeOutput(gest::rules::Output::Pinch);
			sample.rule_pinch_decides = m_rules.decides(gest::rules::Output::Pinch) &&
			                            ((sample.valid && sample.features_valid) || sample.rule_pinch);
			m_stat.pinch_formula += sample.rule_pinch_decides ? 1 : 0;
			m_stat.pinch_image += !sample.rule_pinch_decides && sample.pinch_image_valid ? 1 : 0;
			m_stat.pinch_mapping +=
			    !sample.rule_pinch_decides && !sample.pinch_image_valid && sample.custom_valid ? 1 : 0;
			logGestureEvents(m_mp.update(sample, now_ns), now_ns, &sample);
			updateFormulaGestures(j, now_ns, sample);
			m_stat.mp_frames++;
			m_stat.mp_valid += m_mp.state().valid ? 1 : 0;
		} else if (m_mp.state().valid) {
			logGestureEvents(m_mp.reset("gesture measurements unavailable"), now_ns);
			releaseFormulaGestures();
			ruleChanges(m_rules.reset(now_ns, "gesture measurements unavailable", m_ruleChanges, kRuleChanges),
			            &sample_none());
		}
		/*
		 * The dashboard gesture's pose: the palm toward the face, raised (kSystemMinHeightM below the eyes or
		 * higher). Height is what tells it from everyday use (see kSystemMinHeightM).
		 */
		bool system = false, system_mp = false;
		TrackedPose head;
		if (m_backend->headPose(now_ns, &head) && head.position_valid) {
			using namespace skel::detail;
			const Pose &palm = j.joint[0];
			const V eye{(float)head.pose.position.x, (float)head.pose.position.y, (float)head.pose.position.z};
			const double height = palm.position.y - head.pose.position.y;
			const bool facing =
			    skel::palm_faces(V{(float)palm.position.x, (float)palm.position.y, (float)palm.position.z},
			                     Q{(float)palm.orientation.w, (float)palm.orientation.x, (float)palm.orientation.y,
			                       (float)palm.orientation.z},
			                     eye);
			system = facing && height > kSystemMinHeightM;
			system_mp = facing && height > kSystemMinHeightMpM;
		}
		/*
		 * The system button (SteamVR's dashboard toggle):
		 *  - with MediaPipe's gestures: an open palm toward the face, raised, held about 1 s (gest::SystemGesture);
		 *  - without them (Mercury's joints only): the palm-toward-face pinch held kSystemHoldNs.
		 * Either way one press for both hands (systemPress).
		 */
		bool gesture_down = false;
		if (mp_on) {
			const gest::State &st = m_mp.state();
			// MediaPipe's raw reading this frame (the window judges its flicker; see gest::SystemGesture).
			const gest::Config &mc = m_mp.config();
			const bool cat_ok = sample.valid && sample.category_score >= mc.min_category_score;
			const bool open_raw = cat_ok && sample.category == gest::OpenPalm;
			const bool bad_raw = (cat_ok && sample.category == gest::ClosedFist) ||
			                     (sample.valid && sample.pinch_distance_normalized < mc.pinch_on);
			const bool was_down = m_systemGesture.down();
			gesture_down = m_systemGesture.update(now_ns, system_mp, sample.valid, open_raw, bad_raw);
			if (m_mpDebug && was_down != gesture_down) {
				log("ovrn: %s dashboard gesture: %s\n", m_hand == Hand::Left ? "left" : "right",
				    gesture_down ? "PRESSED (open palm toward the face, raised, held)" : "released (pose ended)");
			}
			m_systemPinch = false;
			m_systemSinceNs = 0;
		} else {
			m_systemGesture.reset();
			m_systemPinch = skel::click(system ? g.pinch : 0.0f, m_systemPinch);
			if (!m_systemPinch) {
				m_systemSinceNs = 0;
			} else if (m_systemSinceNs == 0) {
				m_systemSinceNs = now_ns;
			}
			gesture_down = m_systemPinch && now_ns - m_systemSinceNs >= kSystemHoldNs;
		}
		/*
		 * The dashboard from the left hand only, as SteamVR's own tracked hands (vrlink's dashboard binding toggles it
		 * from the left hand's system button): its open-palm gesture or its palm pinch (palm toward the headset + the
		 * pinch; ovrn_microgestures.hpp). The right hand's are bindable inputs, not the system button.
		 */
		m_systemOn = systemPress(m_hand == Hand::Left && (gesture_down || m_menuPalmPinchOn));
		sendSystem();

		m_stat.palm_facing += system ? 1 : 0;
		m_stat.max_pinch = std::max(m_stat.max_pinch, g.pinch);
		m_stat.max_grab = std::max(m_stat.max_grab, g.grab);
		if (m_systemOn && !m_stat.system_was) {
			m_stat.system_clicks++;
		}
		m_stat.system_was = m_systemOn;

		skel::Gestures out = g;
		if (mp_on) {
			// Trigger = pinch, grip = fist (the dashboard gesture is an open palm: neither).
			const gest::State &st = m_mp.state();
			out.pinch = m_mp.pinchValue();
			out.finger_pinch[0] = out.pinch;
			out.grab = st.fist ? 1.0f : 0.0f;
			// Middle / ring / little pinch strength from MediaPipe's landmarks (thumb tip to fingertip / palm width).
			if (m_strengthValid) {
				for (int i = 0; i < 3; i++) {
					out.finger_pinch[1 + i] = m_strength[i];
				}
			}
			// The left hand's palm pinch opens the dashboard: it isn't also a click (Meta reserves the palm pinch the
			// same way). The right hand's is a normal click (and its own bindable input).
			const bool menu_owns_pinch = m_menuPalmPinchOn && m_hand == Hand::Left;
			if (menu_owns_pinch) {
				out.pinch = 0.0f;
				out.finger_pinch[0] = 0.0f;
			}
			setOutputs(out, st.pinch && !menu_owns_pinch, st.fist);
			setHandGestures(&st);
			return;
		}
		/*
		 * Joint gestures: while the palm faces the head, this hand's pinches don't reach the app, as the pinch is
		 * the dashboard gesture there (one pinch would toggle the dashboard and click in the game at once).
		 */
		if (system || m_systemPinch) {
			out.pinch = 0.0f;
			for (float &p : out.finger_pinch) {
				p = 0.0f;
			}
		}
		setGestures(out);
	}

	/*!
	 * The headset button while bare hands are in use (RunFrame thread, from the provider): pressed, it presses this
	 * hand's system button, which SteamVR's dashboard binding for the hands turns into "toggle dashboard".
	 */
	void
	setHeadButton(bool down)
	{
		m_headButton = down;
	}

	/*!
	 * One dashboard press for both hands: each hand has its own system button, and SteamVR toggles the dashboard on
	 * each press, so both hands making the gesture together opened and closed it at once (2026-11-24 23:37 session:
	 * opened, closed 0.17 s later; replay: both hands pressed at 12.5 s). A hand presses only while the other one
	 * isn't; a press that started while the other hand held it stays ignored until that fist ends.
	 */
	bool
	systemPress(bool gesture_down)
	{
		if (!gesture_down) {
			m_systemIgnored = false;
			if (s_systemHolder == this) {
				s_systemHolder = nullptr;
			}
			return false;
		}
		if (s_systemHolder == this) {
			return true;
		}
		if (s_systemHolder == nullptr && !m_systemIgnored) {
			s_systemHolder = this;
			return true;
		}
		m_systemIgnored = true;
		return false;
	}

	//! The system button: the gesture's hold or the headset button (sent when it changes).
	void
	sendSystem()
	{
		const bool down = m_systemOn || m_headButton;
		if (down != m_systemSent) {
			m_systemSent = down;
			vr::VRDriverInput()->UpdateBooleanComponent(m_system, down, 0.0);
		}
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
		static const char *const kPinch[4] = {"/input/index_pinch/value", "/input/middle_pinch/value",
		                                      "/input/ring_pinch/value", "/input/pinky_pinch/value"};
		for (int f = 0; f < 4; f++) {
			in->CreateScalarComponent(m_container, kPinch[f], &m_pinch[f], vr::VRScalarType_Absolute,
			                          vr::VRScalarUnits_NormalizedOneSided);
		}
		in->CreateBooleanComponent(m_container, "/input/system/click", &m_system);
		for (int i = 0; i < kHandGestureCount; i++) {
			in->CreateBooleanComponent(m_container, kHandGestures[i].path, &m_handGesture[i]);
		}
		for (int i = 0; i < kFormulaInputs; i++) {
			in->CreateBooleanComponent(m_container, kFormulaInputPaths[i], &m_formulaInput[i]);
		}
		// The poses apps and the dashboard bind (no render model gives them): raw and grip at the palm, tip ahead.
		in->CreatePoseComponent(m_container, "/pose/raw", &m_poseRaw);
		in->CreatePoseComponent(m_container, "/pose/grip", &m_poseGrip);
		in->CreatePoseComponent(m_container, "/pose/tip", &m_poseTip);
		in->CreatePoseComponent(m_container, "/pose/poke", &m_posePoke);
		const vr::HmdMatrix34_t identity = {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}}};
		in->UpdatePoseComponent(m_poseRaw, &identity, 0.0);
		in->UpdatePoseComponent(m_poseGrip, &identity, 0.0);
		setTip({0.0f, 0.0f, -0.08f}); // until the hand is seen: 8 cm ahead of the palm, about the index knuckle
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
		/*
		 * The model SteamVR's own tracked hands use: a small pointer marker at /pose/tip (vrlink's shuttlecock).
		 * SteamVR draws the hand itself from the skeleton; without a model it drew a generic controller (2026-11-20).
		 */
		d.render_model = "{vrlink}/rendermodels/shuttlecock";
		// SteamVR's own hand tracking icons (its vrlink driver's resources, part of every SteamVR install).
		const std::string icon = std::string("{vrlink}/icons/") + (m_hand == Hand::Left ? "left" : "right") + "_handtracking_";
		d.string_props = {
		    {vr::Prop_NamedIconPathDeviceOff_String, icon + "off.png"},
		    {vr::Prop_NamedIconPathDeviceSearching_String, icon + "searching.gif"},
		    {vr::Prop_NamedIconPathDeviceSearchingAlert_String, icon + "searching.gif"},
		    {vr::Prop_NamedIconPathDeviceReady_String, icon + "ready.png"},
		    {vr::Prop_NamedIconPathDeviceReadyAlert_String, icon + "ready_info.png"},
		    {vr::Prop_NamedIconPathDeviceNotReady_String, icon + "off.png"},
		    {vr::Prop_NamedIconPathDeviceStandby_String, icon + "off.png"},
		    {vr::Prop_NamedIconPathDeviceAlertLow_String, icon + "ready_info.png"},
		};
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

	/*!
	 * The tip pose (the dashboard's laser) follows the index knuckle in the palm's frame, smoothed (0.1 s) so the
	 * laser's origin doesn't shake with the joints; it keeps the palm's direction (along the hand).
	 */
	void
	updateTip(const skel::Joints &in_palm)
	{
		using namespace skel::detail;
		if (!in_palm.valid[7]) {
			return;
		}
		const float a = 0.1f; // per frame at ~90 Hz: about 0.1 s
		m_tipPos = m_tipPos + (in_palm.pos[7] - m_tipPos) * a;
		setTip(m_tipPos);
	}

	void
	setTip(skel::detail::V p)
	{
		m_tipPos = p;
		const vr::HmdMatrix34_t m = {{{1, 0, 0, p.x}, {0, 1, 0, p.y}, {0, 0, 1, p.z}}};
		vr::VRDriverInput()->UpdatePoseComponent(m_poseTip, &m, 0.0);
	}

	/*!
	 * Every 10 s while the hand was tracked: what the gestures sent SteamVR (RunFrame thread), to tell "the gesture
	 * never fired" from "it fired but nothing reacted": tracked share, trigger (pinch) / grip (grab) / system clicks
	 * and time held, peak pinch and grab values, how often the palm faced the head (the dashboard gesture's pose).
	 */
	void
	gestureLog(int64_t now_ns)
	{
		if (m_stat.since_ns == 0) {
			m_stat.since_ns = now_ns;
			return;
		}
		if (now_ns - m_stat.since_ns < 10'000'000'000LL) {
			return;
		}
		const gest::Stats s = m_mp.takeStats(); // this period's, logged or not
		if (m_stat.tracked > 0) {
			const double n = (double)m_stat.frames;
			log("ovrn: %s hand gestures (10 s): tracked %.0f%%; trigger %u clicks, on %.0f%%; grip %u clicks, on "
			    "%.0f%%; system %u; peak pinch %.2f, grab %.2f; palm toward the head %.0f%%\n",
			    m_hand == Hand::Left ? "left" : "right", 100.0 * m_stat.tracked / n, m_stat.trigger_clicks,
			    100.0 * m_stat.trigger_on / n, m_stat.grip_clicks, 100.0 * m_stat.grip_on / n, m_stat.system_clicks,
			    m_stat.max_pinch, m_stat.max_grab, 100.0 * m_stat.palm_facing / n);
			if (m_stat.mp_frames > 0) {
				log("ovrn: %s hand MediaPipe gestures (10 s): valid %.0f%% of tracked; %u frames, rejected: "
				    "%u invalid, %u old, %u low confidence, %u re-found; closest pinch %.2f (on < %.2f, off > "
				    "%.2f); categories: fist %u, open palm %u, pointing up %u, victory %u, I-love-you %u, none %u\n",
				    m_hand == Hand::Left ? "left" : "right", 100.0 * m_stat.mp_valid / m_stat.mp_frames,
				    s.frames, s.rejected_invalid, s.rejected_stale, s.rejected_low_confidence, s.reassociated,
				    s.frames > 0 ? s.min_pinch : 0.0f, m_mp.config().pinch_on, m_mp.config().pinch_off,
				    s.category[gest::ClosedFist], s.category[gest::OpenPalm], s.category[gest::PointingUp],
				    s.category[gest::Victory], s.category[gest::ILoveYou], s.category[gest::None]);
				log("ovrn: %s hand Hand Landmarker gestures (10 s presses): point %u, thumbs up %u, thumbs down %u, "
				    "victory %u, I-love-you %u, open palm %u\n",
				    m_hand == Hand::Left ? "left" : "right", m_handGestureClicks[0], m_handGestureClicks[1],
				    m_handGestureClicks[2], m_handGestureClicks[3], m_handGestureClicks[4], m_handGestureClicks[5]);
				for (uint32_t &c : m_handGestureClicks) {
					c = 0;
				}
				log("ovrn: %s hand formula gestures (10 s presses): palm pinch %u, poke ready %u, thumb tap %u, double "
				    "tap %u, swipe left %u, right %u, forward %u, back %u; palm angle now %.0f deg (limit %.0f); pinch "
				    "strength middle %.2f ring %.2f little %.2f\n",
				    m_hand == Hand::Left ? "left" : "right", m_formulaPresses[0], m_formulaPresses[1], m_formulaPresses[2],
				    m_formulaPresses[3], m_formulaPresses[4], m_formulaPresses[5], m_formulaPresses[6],
				    m_formulaPresses[7], m_palmAngle, m_microParams.palm_angle_deg, m_strength[0], m_strength[1],
				    m_strength[2]);
				for (uint32_t &c : m_formulaPresses) {
					c = 0;
				}
				const uint32_t pn = m_stat.pinch_formula + m_stat.pinch_image + m_stat.pinch_mapping;
				log("ovrn: %s hand pinch source (10 s): formula %.0f%%, image detector %.0f%%, gesture mapping "
				    "%.0f%% of %u frames\n",
				    m_hand == Hand::Left ? "left" : "right", pn ? 100.0 * m_stat.pinch_formula / pn : 0.0,
				    pn ? 100.0 * m_stat.pinch_image / pn : 0.0, pn ? 100.0 * m_stat.pinch_mapping / pn : 0.0, pn);
			}
		}
		const bool system_was = m_stat.system_was;
		m_stat = GestureStats{};
		m_stat.since_ns = now_ns;
		m_stat.system_was = system_was;
	}

	//! The joint-based gestures: clicks from the values with their hysteresis.
	void
	setGestures(const skel::Gestures &g)
	{
		setOutputs(g, skel::click(g.pinch, m_triggerOn), skel::click_grab(g.grab, m_gripOn));
		setHandGestures(nullptr); // Hand Landmarker's gestures need its measurements
	}

	//! The trigger, grip and finger pinch inputs (values from @p g, clicks given).
	void
	setOutputs(const skel::Gestures &g, bool trigger_click, bool grip_click)
	{
		const bool trigger_was = m_triggerOn, grip_was = m_gripOn;
		m_triggerOn = trigger_click;
		m_gripOn = grip_click;
		m_stat.trigger_clicks += m_triggerOn && !trigger_was ? 1 : 0;
		m_stat.grip_clicks += m_gripOn && !grip_was ? 1 : 0;
		m_stat.trigger_on += m_triggerOn ? 1 : 0;
		m_stat.grip_on += m_gripOn ? 1 : 0;
		vr::IVRDriverInput *in = vr::VRDriverInput();
		in->UpdateScalarComponent(m_trigger, g.pinch, 0.0);
		in->UpdateBooleanComponent(m_triggerClick, m_triggerOn, 0.0);
		in->UpdateScalarComponent(m_grip, g.grab, 0.0);
		in->UpdateBooleanComponent(m_gripClick, m_gripOn, 0.0);
		for (int f = 0; f < 4; f++) {
			in->UpdateScalarComponent(m_pinch[f], g.finger_pinch[f], 0.0);
		}
	}

	static bool
	envFlag(const char *name)
	{
		const char *v = std::getenv(name);
		return v != nullptr && (v[0] == '1' || v[0] == 'y' || v[0] == 't');
	}

	static float
	envFloat(const char *name, float fallback)
	{
		const char *v = std::getenv(name);
		return v != nullptr && v[0] != '\0' ? (float)std::atof(v) : fallback;
	}

	/*!
	 * The gesture state machine's tuning for the hand tracker's profile (MediaPipe's gesture crop sits on the 3D hand,
	 * which comes from a different model per profile, so each profile gets its own values; 2026-11-25: both still the
	 * values tuned with MediaPipe building the hand). OVRN_MP_* override them (development).
	 */
	static gest::Config
	gestureConfig(const std::string &profile = {})
	{
		gest::Config c;
		if (profile == "mercury") {
			// Mercury's model builds the 3D hand, MediaPipe reads the gestures: to be tuned (see psvr2-test/hands).
		} else {
			// MediaPipe builds the 3D hand (and the default before the tracker reports its profile): gest::Config.
		}
		c.pinch_on = envFloat("OVRN_MP_PINCH_ON", c.pinch_on);
		c.pinch_off = envFloat("OVRN_MP_PINCH_OFF", c.pinch_off);
		c.min_confidence = envFloat("OVRN_MP_MIN_CONFIDENCE", c.min_confidence);
		c.min_category_score = envFloat("OVRN_MP_CATEGORY_SCORE", c.min_category_score);
		c.confirm_frames = (int)envFloat("OVRN_MP_CONFIRM_FRAMES", (float)c.confirm_frames);
		return c;
	}

	//! The dashboard gesture's timing for the hand tracker's profile (as gestureConfig).
	static gest::SystemGesture::Config
	systemGestureConfig(const std::string &profile)
	{
		gest::SystemGesture::Config c;
		(void)profile; // both profiles: gest::SystemGesture::Config's values for now
		return c;
	}

	//! OVRN_GESTURE_DEBUG=1: each gesture change with its reason and the measurement behind it.
	void
	logGestureEvents(const gest::Events &e, int64_t now_ns, const gest::Sample *s = nullptr)
	{
		if (!m_mpDebug || e.reason == nullptr) {
			return;
		}
		const char *side = m_hand == Hand::Left ? "left" : "right";
		if (s != nullptr) {
			log("ovrn: %s gesture: %s%s%s%s (%s); pinch %.3f, confidence %.2f, category %d score %.2f, age %.0f ms\n",
			    side, e.pinch_pressed ? "pinch pressed " : "", e.pinch_released ? "pinch released " : "",
			    e.fist_pressed ? "fist pressed " : "", e.fist_released ? "fist released " : "", e.reason,
			    s->pinch_distance_normalized, s->confidence, s->category, s->category_score,
			    (now_ns - s->timestamp_ns) / 1e6);
		} else {
			log("ovrn: %s gesture: %s%s%s%s (%s)\n", side, e.pinch_pressed ? "pinch pressed " : "",
			    e.pinch_released ? "pinch released " : "", e.fist_pressed ? "fist pressed " : "",
			    e.fist_released ? "fist released " : "", e.reason);
		}
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
	vr::VRInputComponentHandle_t m_pinch[4] = {vr::k_ulInvalidInputComponentHandle, vr::k_ulInvalidInputComponentHandle,
	                                           vr::k_ulInvalidInputComponentHandle, vr::k_ulInvalidInputComponentHandle};
	vr::VRInputComponentHandle_t m_system = vr::k_ulInvalidInputComponentHandle;

	// ---------------------------------------------------------------- formula gestures (ovrn_microgestures.hpp)
	gest::micro::Params m_microParams = gest::micro::Params::fromMap(m_hand == Hand::Left ? 0 : 1,
	                                                                  gestureRules().params[m_hand == Hand::Left ? 0 : 1]);
	gest::micro::Micro m_micro = gest::micro::Micro(m_microParams);
	bool m_menuPalmPinchOn = false;
	float m_palmAngle = NAN;
	bool m_strengthValid = false;
	float m_strength[3] = {};
	bool m_pokeReady = false;
	//! Inputs: the palm pinch, poke ready, thumb tap, double tap, swipes left / right / forward / back.
	static constexpr int kFormulaInputs = 8;
	static constexpr const char *kFormulaInputPaths[kFormulaInputs] = {
	    "/input/menu_palm_pinch/click", "/input/index_poke/click",       "/input/thumb_tap/click",
	    "/input/thumb_double_tap/click", "/input/thumb_swipe_left/click", "/input/thumb_swipe_right/click",
	    "/input/thumb_swipe_forward/click", "/input/thumb_swipe_back/click"};
	vr::VRInputComponentHandle_t m_formulaInput[kFormulaInputs] = {
	    vr::k_ulInvalidInputComponentHandle, vr::k_ulInvalidInputComponentHandle, vr::k_ulInvalidInputComponentHandle,
	    vr::k_ulInvalidInputComponentHandle, vr::k_ulInvalidInputComponentHandle, vr::k_ulInvalidInputComponentHandle,
	    vr::k_ulInvalidInputComponentHandle, vr::k_ulInvalidInputComponentHandle};
	bool m_formulaOn[kFormulaInputs] = {};
	int64_t m_pulseUntil[kFormulaInputs] = {}; //!< events (tap, swipes) are held this long so apps see them
	uint32_t m_formulaPresses[kFormulaInputs] = {};
	vr::VRInputComponentHandle_t m_posePoke = vr::k_ulInvalidInputComponentHandle;

	static gest::micro::Vec3
	g3(const Vec3 &v)
	{
		return {(float)v.x, (float)v.y, (float)v.z};
	}

	void
	setFormulaInput(int i, bool on)
	{
		if (on && !m_formulaOn[i]) {
			m_formulaPresses[i]++;
		}
		if (on != m_formulaOn[i] || on) {
			vr::VRDriverInput()->UpdateBooleanComponent(m_formulaInput[i], on, 0.0);
		}
		m_formulaOn[i] = on;
	}

	//! Everything formula-based let go (the hand isn't tracked, or its measurements stopped).
	void
	releaseFormulaGestures()
	{
		m_menuPalmPinchOn = false;
		m_strengthValid = false;
		m_pokeReady = false;
		m_micro.reset();
		for (int i = 0; i < kFormulaInputs; i++) {
			m_pulseUntil[i] = 0;
			setFormulaInput(i, false);
		}
	}

	/*!
	 * The formula gestures for this frame (after the MediaPipe gesture state's update): the palm pinch (Mercury's
	 * joints and the head pose, both in the room frame), the pinch strengths, poke ready and the thumb tap / swipes
	 * (MediaPipe's joint features), and the poke pose (Mercury's fingertip, in the hand's frame).
	 */
	void
	updateFormulaGestures(const HandJoints &j, int64_t now_ns, const gest::Sample &sample)
	{
		using namespace gest::micro;
		using GV = gest::micro::Vec3;
		const gest::State &st = m_mp.state();
		const bool feats = sample.valid && sample.features_valid;

		// Palm pinch: palm normal = normalize(cross(index knuckle - wrist, little knuckle - wrist)), oriented by the
		// palm joint's inner axis; palm centre = mean of the wrist and the four knuckles; the angle to the headset.
		bool facing = false;
		m_palmAngle = NAN;
		TrackedPose head;
		static const int kW = 1, kI = 7, kM = 12, kR = 17, kL = 22; // OpenXR joints: wrist, proximal (knuckles)
		if (m_backend->headPose(now_ns, &head) && head.position_valid && j.joint_valid[0] && j.joint_valid[kW] &&
		    j.joint_valid[kI] && j.joint_valid[kM] && j.joint_valid[kR] && j.joint_valid[kL]) {
			using namespace skel::detail;
			const Pose &palm = j.joint[0];
			const V inner = rotate(Q{(float)palm.orientation.w, (float)palm.orientation.x, (float)palm.orientation.y,
			                         (float)palm.orientation.z},
			                       V{0, -1, 0});
			GV n;
			const GV center = geo::scale(
			    geo::add(geo::add(geo::add(g3(j.joint[kW].position), g3(j.joint[kI].position)),
			                      geo::add(g3(j.joint[kM].position), g3(j.joint[kR].position))),
			             g3(j.joint[kL].position)),
			    0.2f);
			float angle = 0.0f;
			if (palmNormal(g3(j.joint[kW].position), g3(j.joint[kI].position), g3(j.joint[kL].position),
			               GV{inner.x, inner.y, inner.z}, &n) &&
			    palmAngleDeg(n, center, g3(head.pose.position), &angle)) {
				m_palmAngle = angle;
				facing = angle < m_microParams.palm_angle_deg;
			}
		}
		const bool menu = st.valid && st.pinch && facing;
		if (menu != m_menuPalmPinchOn && m_mpDebug) {
			log("ovrn: %s palm pinch: %s (palm angle %.0f deg, limit %.0f; pinch %d)\n",
			    m_hand == Hand::Left ? "left" : "right", menu ? "on" : "off", m_palmAngle,
			    m_microParams.palm_angle_deg, st.pinch ? 1 : 0);
		}
		m_menuPalmPinchOn = menu;
		setFormulaInput(0, menu);

		// Pinch strengths (middle, ring, little), smoothed.
		if (feats) {
			for (int i = 0; i < 3; i++) {
				const float raw = pinchStrength(sample.features[geo::F_THUMB_MIDDLE + i], m_microParams.pinch_open[i],
				                                m_microParams.pinch_closed[i]);
				m_strength[i] = m_strengthValid ? m_microParams.pinch_alpha * raw +
				                                      (1.0f - m_microParams.pinch_alpha) * m_strength[i]
				                                : raw;
			}
		}
		m_strengthValid = feats;

		// Poke ready (index extended, middle not).
		m_pokeReady = feats && pokeReady(sample.features, m_microParams);
		setFormulaInput(1, m_pokeReady);

		// Thumb tap and swipes: discrete events, each held 100 ms for apps; not while a pinch, fist or palm pinch is on.
		const uint32_t ev =
		    m_micro.update(sample.timestamp_ns, sample.features, feats, st.pinch || st.fist || m_menuPalmPinchOn);
		static const uint32_t bits[6] = {E_TAP, E_DOUBLE_TAP, E_SWIPE_LEFT, E_SWIPE_RIGHT, E_SWIPE_FORWARD, E_SWIPE_BACK};
		for (int k = 0; k < 6; k++) {
			if (ev & bits[k]) {
				m_pulseUntil[2 + k] = now_ns + 100'000'000;
				if (m_mpDebug) {
					static const char *const names[6] = {"thumb tap", "thumb double tap", "thumb swipe left",
					                                     "thumb swipe right", "thumb swipe forward", "thumb swipe back"};
					log("ovrn: %s %s\n", m_hand == Hand::Left ? "left" : "right", names[k]);
				}
			}
			setFormulaInput(2 + k, now_ns < m_pulseUntil[2 + k]);
		}
	}

	/*!
	 * The poke pose (/pose/poke) in the hand's frame: at the index fingertip (Mercury's joint), its -Z along the last
	 * two index segments (distal weighted more), its +Y toward the back of the hand.
	 */
	void
	updatePoke(const skel::Joints &in_palm)
	{
		using namespace gest::micro;
		using GV = gest::micro::Vec3;
		static const int kPip = 8, kDip = 9, kTip = 10; // OpenXR: index intermediate (PIP), distal (DIP), tip
		if (!in_palm.valid[kPip] || !in_palm.valid[kDip] || !in_palm.valid[kTip]) {
			return;
		}
		const auto v = [](const skel::detail::V &a) { return GV{a.x, a.y, a.z}; };
		GV dir, x, y;
		if (!fingertipDirection(v(in_palm.pos[kPip]), v(in_palm.pos[kDip]), v(in_palm.pos[kTip]),
		                        m_microParams.poke_proximal_weight, m_microParams.poke_distal_weight, &dir)) {
			return;
		}
		const GV z = geo::scale(dir, -1.0f); // SteamVR poses point along -Z
		if (!geo::normalized(geo::cross(GV{0, 1, 0}, z), &x)) {
			if (!geo::normalized(geo::cross(GV{0, 0, 1}, z), &x)) {
				return;
			}
		}
		y = geo::cross(z, x);
		const GV p = v(in_palm.pos[kTip]);
		const vr::HmdMatrix34_t m = {{{x.x, y.x, z.x, p.x}, {x.y, y.y, z.y, p.y}, {x.z, y.z, z.z, p.z}}};
		vr::VRDriverInput()->UpdatePoseComponent(m_posePoke, &m, 0.0);
	}
	/*!
	 * Hand Landmarker's own gestures as buttons (its Gesture Recognizer categories, confirmed by the gesture state;
	 * all released while the hand isn't tracked): for games and SteamVR's binding screen.
	 */
	struct HandGestureInput
	{
		const char *path;
		int category;
	};
	static constexpr int kHandGestureCount = 6;
	static constexpr HandGestureInput kHandGestures[kHandGestureCount] = {
	    {"/input/point/click", gest::PointingUp}, {"/input/thumbs_up/click", gest::ThumbUp},
	    {"/input/thumbs_down/click", gest::ThumbDown}, {"/input/victory/click", gest::Victory},
	    {"/input/i_love_you/click", gest::ILoveYou}, {"/input/open_palm/click", gest::OpenPalm}};
	vr::VRInputComponentHandle_t m_handGesture[kHandGestureCount] = {
	    vr::k_ulInvalidInputComponentHandle, vr::k_ulInvalidInputComponentHandle,
	    vr::k_ulInvalidInputComponentHandle, vr::k_ulInvalidInputComponentHandle,
	    vr::k_ulInvalidInputComponentHandle, vr::k_ulInvalidInputComponentHandle};
	bool m_handGestureOn[kHandGestureCount] = {};
	uint32_t m_handGestureClicks[kHandGestureCount] = {};

	//! The Hand Landmarker gesture buttons from @p st (nullptr: all released).
	void
	setHandGestures(const gest::State *st)
	{
		vr::IVRDriverInput *in = vr::VRDriverInput();
		for (int i = 0; i < kHandGestureCount; i++) {
			const bool on = st != nullptr && st->valid && st->category[kHandGestures[i].category];
			if (on && !m_handGestureOn[i]) {
				m_handGestureClicks[i]++;
			}
			if (on != m_handGestureOn[i] || on) {
				in->UpdateBooleanComponent(m_handGesture[i], on, 0.0);
			}
			m_handGestureOn[i] = on;
		}
	}
	vr::VRInputComponentHandle_t m_poseRaw = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t m_poseGrip = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t m_poseTip = vr::k_ulInvalidInputComponentHandle;
	skel::detail::V m_tipPos{0.0f, 0.0f, -0.08f};
	bool m_triggerOn = false, m_gripOn = false, m_tracked = false, m_systemOn = false;
	//! Joint gestures' system pinch: its click state and since when it's held (0: not held).
	bool m_systemPinch = false;
	bool m_headButton = false, m_systemSent = false; //!< the headset button's share of the system button, last sent
	int64_t m_systemSinceNs = 0;
	//! Joint gestures only: how long the palm-toward-face pinch must be held to press the system button.
	static constexpr int64_t kSystemHoldNs = 500'000'000;
	/*!
	 * How high the palm must be for the system gesture: no lower than this below the eyes (metres, negative). Height
	 * is what tells the gesture from everyday use: deliberate dashboard attempts had the palm 1-17 cm below the eyes
	 * (2026-11-24 20:50, 21:47 sessions), normal use about 20 cm (23:37). -0.25 for one build toggled the dashboard
	 * on ordinary open-hand-then-grab moves (replay of 23:37: 9 presses in 40 s; 2 with -0.15 and SystemGesture's
	 * 0.6 s / 0.5 s).
	 */
	static constexpr double kSystemMinHeightM = -0.15;
	/*!
	 * With MediaPipe's gestures: the open palm no lower than 20 cm below the head (room frame, as measured here).
	 * Holding an open hand up toward the headset is common (the benchmark's "open hand" prompts). Scored on the
	 * 2026-11-25 benchmark through hand_sim (this code, the session's head poses) with a 1 s hold: 15 cm caught 2 of
	 * the 4 dashboard prompts, 20 cm 4 of 4 with no press in the 54 other prompts or free use, 25 cm 4 of 4 but 5
	 * presses in open-hand and pinch prompts.
	 */
	static constexpr double kSystemMinHeightMpM = -0.20;
	struct GestureStats
	{
		int64_t since_ns = 0;
		uint32_t frames = 0, tracked = 0, palm_facing = 0;
		uint32_t trigger_clicks = 0, grip_clicks = 0, system_clicks = 0, trigger_on = 0, grip_on = 0;
		//! Frames whose pinch the formula rule, the image detector, the gesture mapping decided.
		uint32_t pinch_formula = 0, pinch_image = 0, pinch_mapping = 0;
		uint32_t mp_frames = 0, mp_valid = 0; //!< frames with MediaPipe measurements, and how many valid
		float max_pinch = 0.0f, max_grab = 0.0f;
		bool system_was = false;
	} m_stat; //!< RunFrame thread: the 10 s gesture summary (gestureLog)
	uint64_t m_updates = 0;
	//! The gesture rules config, loaded once (both hands; the file or the built-in one).
	static const gest::rules::Config &
	gestureRules()
	{
		static const gest::rules::Config c = [] {
			std::string note;
			gest::rules::Config cfg = gest::rules::loadConfig(gest::rules::kDefaultConfig, &note);
			log("ovrn: gesture rules: %s\n", note.c_str());
			static const char *const hn[2] = {"left", "right"};
			for (int h = 0; h < 2; h++) {
				for (const gest::rules::Gesture &g : cfg.hands[h].gestures) {
					log("ovrn: gesture rules: %s %s: %s (%s)\n", hn[h], g.id.c_str(),
					    g.enabled && !g.terms.empty() ? "on" : "off", g.status.c_str());
				}
			}
			return cfg;
		}();
		return c;
	}

	static const gest::Sample &
	sample_none()
	{
		static const gest::Sample s;
		return s;
	}

	//! Logs the rules' state changes into an active gesture or out of one (and every change with OVRN_GESTURE_DEBUG).
	void
	ruleChanges(int n, const gest::Sample *s)
	{
		for (int i = 0; i < n; i++) {
			const gest::rules::Change &c = m_ruleChanges[i];
			const bool edge = c.to == gest::rules::Phase::Active || c.from == gest::rules::Phase::Active ||
			                  c.from == gest::rules::Phase::Releasing;
			if (c.gesture < 0 || !(edge || m_mpDebug)) {
				continue;
			}
			const gest::rules::Gesture &g = m_rules.hand().gestures[c.gesture];
			std::string terms;
			for (const gest::rules::Term &t : g.terms) {
				char buf[96];
				snprintf(buf, sizeof(buf), "%s%s %.3f (on %s %.3f, off %.3f)", terms.empty() ? "" : ", ",
				         gest::rules::geo::featureName(t.feature), m_rules.smoothed()[t.feature],
				         t.below ? "<=" : ">=", t.on, t.off);
				terms += buf;
			}
			log("ovrn: %s gesture rule %s: %s -> %s (%s); %s; presence %.2f\n", m_hand == Hand::Left ? "left" : "right",
			    g.id.c_str(), gest::rules::phaseName(c.from), gest::rules::phaseName(c.to), c.reason, terms.c_str(),
			    s != nullptr ? s->confidence : 0.0f);
		}
	}

	static constexpr int kRuleChanges = 16;
	gest::rules::Change m_ruleChanges[kRuleChanges];
	gest::Machine m_mp;   //!< RunFrame thread: MediaPipe gesture state
	gest::rules::Engine m_rules = gest::rules::Engine(gestureRules().hands[m_hand == Hand::Left ? 0 : 1],
	                                                  gestureRules().smooth_frames, gestureRules().grace_ns);
	const bool m_mpDebug; //!< OVRN_GESTURE_DEBUG
	gest::SystemGesture m_systemGesture; //!< RunFrame thread: the dashboard gesture with MediaPipe's gestures
	std::string m_gestureProfile;        //!< RunFrame thread: the profile the gesture tuning is for
	bool m_systemIgnored = false;         //!< this hand's gesture press started while the other hand held it
	//! The hand whose gesture holds the dashboard press now (RunFrame thread; both hands run there).
	static inline HandDevice *s_systemHolder = nullptr;
};

} // namespace ovrn
