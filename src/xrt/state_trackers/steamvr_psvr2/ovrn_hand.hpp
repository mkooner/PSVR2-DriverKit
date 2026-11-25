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
 * hand), per-finger pinches, and a system button on each hand: the palm toward the face with an index pinch, which
 * the dashboard binding uses to open and close the dashboard (Valve's has it on the left hand only).
 * @ingroup st_ovrn
 */

#pragma once

#include "ovrn_device.hpp"
#include "ovrn_hand_gestures.hpp"
#include "ovrn_gesture_state.hpp"

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
			setGestures(skel::Gestures{});
			// A lost hand lets go of the gesture's system press (the headset button still works: sendSystem).
			m_systemPinch = false;
			m_systemSinceNs = 0;
			m_systemOn = false;
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
		/*
		 * MediaPipe's gesture measurements when the hand tracker has them (MERCURY_MEDIAPIPE_GESTURES): pinch and
		 * fist from them, through the debounced state machine. While they're on but this frame's is invalid (hand
		 * doubtful, old), the gestures are released rather than switching to the joint rules and back. Without
		 * them (off, models missing), the joint-based gestures as before.
		 */
		gest::Sample sample;
		const bool mp_on = m_backend->handGestures(m_hand, &sample);
		if (mp_on) {
			logGestureEvents(m_mp.update(sample, now_ns), now_ns, &sample);
			m_stat.mp_frames++;
			m_stat.mp_valid += m_mp.state().valid ? 1 : 0;
		} else if (m_mp.state().valid) {
			logGestureEvents(m_mp.reset("gesture measurements unavailable"), now_ns);
		}
		/*
		 * The system gesture (either hand), as Meta's: the palm toward the face, raised to about eye level, and an
		 * index pinch held. "At eye level" (Meta's own instruction) is what keeps everyday hand shapes out: on the
		 * 2026-11-24 20:05 session all 4 dashboard openings, 3 of them unwanted (claw, fist, a tilted pinch), had
		 * the palm 20-51 cm below the eyes, where the hands are while using them; the cameras still track a hand up to
		 * about eye level (earlier sessions: half the tracked frames 10-20 cm below).
		 */
		bool system = false;
		TrackedPose head;
		if (m_backend->headPose(now_ns, &head) && head.position_valid) {
			using namespace skel::detail;
			const Pose &palm = j.joint[0];
			const V eye{(float)head.pose.position.x, (float)head.pose.position.y, (float)head.pose.position.z};
			const bool raised = palm.position.y - head.pose.position.y > kSystemMinHeightM;
			system = raised &&
			         skel::palm_faces(V{(float)palm.position.x, (float)palm.position.y, (float)palm.position.z},
			                          Q{(float)palm.orientation.w, (float)palm.orientation.x,
			                            (float)palm.orientation.y, (float)palm.orientation.z},
			                          eye);
		}
		/*
		 * A hold, as Meta's system gesture (palm toward you, pinch held until its ring fills): the system button goes
		 * down only after the palm-toward-head pinch has lasted kSystemHoldNs (now 0.5 s), and up when it ends. Before, it fired on
		 * the first frame: on 2026-11-24 (14:35 session) curling a hand into a fist with the palm toward the face read
		 * as a pinch and toggled the dashboard 4 times unasked (palms really faced the cameras, camera images 15.5 s
		 * and 23 s); a fist curl lasts a fraction of a second.
		 */
		// With MediaPipe's measurements, only a deliberate pinch (other fingers open) counts for the system gesture.
		const float system_pinch = mp_on ? (m_mp.state().deliberatePinch ? 1.0f : 0.0f) : g.pinch;
		m_systemPinch = skel::click(system ? system_pinch : 0.0f, m_systemPinch);
		if (!m_systemPinch) {
			m_systemSinceNs = 0;
			m_systemOn = false;
		} else if (m_systemSinceNs == 0) {
			m_systemSinceNs = now_ns;
		} else if (now_ns - m_systemSinceNs >= kSystemHoldNs) {
			m_systemOn = true;
		}
		sendSystem();
		/*
		 * While the palm faces the head (the system gesture's pose), this hand's pinches don't reach the app: one
		 * pinch would otherwise toggle the dashboard and click in the game at once. Meta does the same with its
		 * system gesture (apps get no pinch while it's in progress; required by its store's VRC.Quest.Input.8).
		 */
		m_stat.palm_facing += system ? 1 : 0;
		m_stat.max_pinch = std::max(m_stat.max_pinch, g.pinch);
		m_stat.max_grab = std::max(m_stat.max_grab, g.grab);
		if (m_systemOn && !m_stat.system_was) {
			m_stat.system_clicks++;
		}
		m_stat.system_was = m_systemOn;
		skel::Gestures out = g;
		if (mp_on) {
			const gest::State &st = m_mp.state();
			out.pinch = m_mp.pinchValue();
			out.finger_pinch[0] = out.pinch;
			out.grab = st.fist ? 1.0f : 0.0f;
			if (system || m_systemPinch) {
				out.pinch = 0.0f;
				for (float &p : out.finger_pinch) {
					p = 0.0f;
				}
			}
			setOutputs(out, st.pinch && !(system || m_systemPinch), st.fist);
			return;
		}
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
		// The poses apps and the dashboard bind (no render model gives them): raw and grip at the palm, tip ahead.
		in->CreatePoseComponent(m_container, "/pose/raw", &m_poseRaw);
		in->CreatePoseComponent(m_container, "/pose/grip", &m_poseGrip);
		in->CreatePoseComponent(m_container, "/pose/tip", &m_poseTip);
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
	 * and time held, peak pinch and grab values, how often the palm faced the head (pinches are held back then).
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

	//! The gesture state machine's tuning; OVRN_MP_* override the defaults (development).
	static gest::Config
	gestureConfig()
	{
		gest::Config c;
		c.pinch_on = envFloat("OVRN_MP_PINCH_ON", c.pinch_on);
		c.pinch_off = envFloat("OVRN_MP_PINCH_OFF", c.pinch_off);
		c.min_confidence = envFloat("OVRN_MP_MIN_CONFIDENCE", c.min_confidence);
		c.min_category_score = envFloat("OVRN_MP_CATEGORY_SCORE", c.min_category_score);
		c.confirm_frames = (int)envFloat("OVRN_MP_CONFIRM_FRAMES", (float)c.confirm_frames);
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
	vr::VRInputComponentHandle_t m_poseRaw = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t m_poseGrip = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t m_poseTip = vr::k_ulInvalidInputComponentHandle;
	skel::detail::V m_tipPos{0.0f, 0.0f, -0.08f};
	bool m_triggerOn = false, m_gripOn = false, m_tracked = false, m_systemOn = false;
	//! The system gesture: the palm-toward-head pinch's click state and since when it's held (0: not held).
	bool m_systemPinch = false;
	bool m_headButton = false, m_systemSent = false; //!< the headset button's share of the system button, last sent
	int64_t m_systemSinceNs = 0;
	/*!
	 * How long the palm-toward-head pinch must be held to press the system button (Meta: until a ring fills, about 1 s
	 * by third-party accounts, not published). 0.6 s first (2026-11-24 15:03 session): the dashboard never opened;
	 * with the palm toward the head Mercury barely sees the pinch (replay: pinch median 0 while the palm faced the
	 * head, the longest clicked pinch 0.37 s). 0.3 s: still longer than an accidental flick, within what it sees.
	 */
	static constexpr int64_t kSystemHoldNs = 500'000'000;
	/*!
	 * 0.5 s since 2026-11-24 evening (MediaPipe's pinch is seen at palm-toward-face angles, where Mercury's wasn't);
	 * Meta holds "until the icon fills". Replay of the labelled session: openings outside the pinch prompts 5 at
	 * 0.3 s, 1 at 0.5 s.
	 */
	//! How high the palm must be for the system gesture: no lower than this below the eyes (metres, negative).
	static constexpr double kSystemMinHeightM = -0.15;
	struct GestureStats
	{
		int64_t since_ns = 0;
		uint32_t frames = 0, tracked = 0, palm_facing = 0;
		uint32_t trigger_clicks = 0, grip_clicks = 0, system_clicks = 0, trigger_on = 0, grip_on = 0;
		uint32_t mp_frames = 0, mp_valid = 0; //!< frames with MediaPipe measurements, and how many valid
		float max_pinch = 0.0f, max_grab = 0.0f;
		bool system_was = false;
	} m_stat; //!< RunFrame thread: the 10 s gesture summary (gestureLog)
	uint64_t m_updates = 0;
	gest::Machine m_mp;   //!< RunFrame thread: MediaPipe gesture state
	const bool m_mpDebug; //!< OVRN_GESTURE_DEBUG
};

} // namespace ovrn
