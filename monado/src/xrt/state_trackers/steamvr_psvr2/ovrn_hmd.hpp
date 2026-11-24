// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The headset: a plain display device. SteamVR owns the display, composites, paces and reprojects; the
 *         driver describes the display and hands over head poses (after ALVR's HMD.cpp on Linux: IVRDisplayComponent,
 *         IsDisplayRealDisplay true). In driver direct mode (DriverKit's setting, off by default) SteamVR hands its
 *         frames to the driver instead and Monado's compositor shows them (ovrn_direct_mode.hpp).
 * @ingroup st_ovrn
 */

#pragma once

#include "ovrn_camera.hpp"
#include "ovrn_direct_mode.hpp"
#include "ovrn_device.hpp"
#include "ovrn_presence_log.hpp"
#include "ovrn_settings.hpp"

#include "os/os_time.h"

#include <cmath>
#include <cstdlib>
#include <chrono>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

namespace ovrn {

class Hmd final : public TrackedDevice, public vr::IVRDisplayComponent
{
public:
	//! SteamVR's verdict on the headset (user interaction, standby), for the presence log (ovrn_presence_log.hpp).
	void
	presenceEvent(PresenceLog::Event e)
	{
		m_presence.event(e, now_ns());
	}

	//! The presence log's lines due now (RunFrame thread; at_shutdown: what's left and the session totals).
	void
	presenceLog(int64_t now, bool at_shutdown = false)
	{
		for (const std::string &line : m_presence.tick(now, at_shutdown)) {
			log("%s\n", line.c_str());
		}
	}

	//! The headset's index in SteamVR, k_unTrackedDeviceIndexInvalid when not active.
	uint32_t
	objectIndex() const
	{
		return m_objectId.load();
	}

	//! SteamVR putting the headset in standby (it was idle: still and not worn, or the power menu). Logged only.
	void
	EnterStandby() override
	{
		presenceEvent(PresenceLog::DriverEnterStandby);
	}

	/*!
	 * See-through view on or off (RunFrame thread): as Sony's driver does in its see-through mode, ask SteamVR to
	 * pause the game and render less while the camera picture is all there is to see. Controllers stay SteamVR's to
	 * draw (Sony's driver draws them itself and sets Prop_DriverIsDrawingControllers_Bool; here SteamVR draws).
	 */
	void
	setSeeThrough(bool on)
	{
		if (m_camera) {
			m_camera->setRoomViewShown(on); // the full picture rate only while it's on screen
		}
		if (!active()) {
			return;
		}
		vr::VRProperties()->SetBoolProperty(m_container, vr::Prop_DriverRequestsApplicationPause_Bool, on);
		vr::VRProperties()->SetBoolProperty(m_container, vr::Prop_DriverRequestsReducedRendering_Bool, on);
	}

	/*!
	 * The headset's button to SteamVR as the headset's system button (RunFrame thread). Our compositor binding
	 * (resources/input/vrcompositor_binding_playstation_vr2_hmd.json, the headset profile's default_bindings) makes
	 * a press SteamVR's "Toggle Room View", as the button shows the cameras on a PS5. A double-click sent before
	 * (2026-11-18) did nothing: without a compositor binding for the headset, SteamVR binds its button to nothing.
	 * OVRN_HEAD_BUTTON=off: not reported.
	 */
	void
	updateInputs(int64_t now)
	{
		updateEyeTracking();
		if (m_proximityHandle != vr::k_ulInvalidInputComponentHandle) {
			const bool worn = m_backend->headWorn();
			m_presence.worn(worn, now);
			if (worn != m_wornSent || !m_wornSentOnce) {
				m_wornSent = worn;
				m_wornSentOnce = true;
				vr::VRDriverInput()->UpdateBooleanComponent(m_proximityHandle, worn, 0.0);
			}
		}
		if (m_buttonHandle == vr::k_ulInvalidInputComponentHandle) {
			return;
		}
		const bool down = m_backend->headButton();
		if (down != m_buttonSent) {
			m_buttonSent = down;
			vr::VRDriverInput()->UpdateBooleanComponent(m_buttonHandle, down, 0.0);
		}
	}


	explicit Hmd(Backend *backend)
	    : TrackedDevice(vr::TrackedDeviceClass_HMD, backend->headInfo().serial), m_backend(backend),
	      m_info(backend->headInfo()), m_display(backend->display()), m_direct(backend->directMode())
	{
		// OVRN_POSE_MODEL=predicted: the driver predicts (one refresh plus vsync-to-photons ahead, OVRN_PREDICT_MS)
		// and hands the pose over with zero velocity, as ALVR does. Default: SteamVR predicts (see updatePose).
		const char *model = getenv("OVRN_POSE_MODEL");
		m_driverPredicts = model != nullptr && strcmp(model, "predicted") == 0;
		if (m_driverPredicts) {
			predict_ns = (int64_t)((1.0 / m_display.refresh_hz + m_display.vsync_to_photons_s) * 1e9);
			const char *env = getenv("OVRN_PREDICT_MS");
			if (env != nullptr && env[0] != '\0') {
				predict_ns = (int64_t)(atof(env) * 1e6);
			}
		}
	}

	//! How far ahead of now the head pose is predicted by the driver (0 unless OVRN_POSE_MODEL=predicted).
	std::atomic<int64_t> predict_ns{0};

	/*!
	 * The head pose for now with its velocities and its exact age, so SteamVR computes the pose for the display
	 * time each application asks for (as a physical headset's driver does: SteamVR predicts).
	 *
	 * Why not ALVR's model (driver-predicted pose, zero velocity; ALVR's headset re-warps every frame, a wired headset
	 * under SteamVR without reprojection doesn't): with zero velocity SteamVR can only return the newest pose, so the
	 * pose depends on WHEN an application asks instead of the display time it asks for. Measured 2026-11-18 16:04 with
	 * that model (game frame log + gyro): the test game asks 1.7 ms later on every third frame (an extra
	 * xrLocateViews), so its poses jumped ahead and back by 1.7 ms of head motion on a 3-frame cycle: jitter against
	 * the true head p50 0.081 / p90 0.177 deg while turning, where prediction to the photon time from the same data
	 * gives 0.013 / 0.069 deg.
	 */
	void
	updatePose(int64_t now_ns) override
	{
		vr::DriverPose_t pose;
		int64_t frame_target = 0;
		if (directFramePose(now_ns, &pose, &frame_target)) {
			m_direct->logSubmittedPose(frame_target, pose);
		} else {
			pose = buildPose(now_ns + predict_ns.load(), !m_driverPredicts, now_ns);
		}
		m_presence.headPose(pose.poseIsValid && pose.result == vr::TrackingResult_Running_OK, now_ns);
		submitPose(pose);

		// The IPD dial: twice a second, new eye poses go to SteamVR when they changed.
		if (now_ns - m_eyeCheckNs > 500000000LL) {
			m_eyeCheckNs = now_ns;
			Pose eyes[2];
			if (m_backend->eyePoses(eyes)) {
				if (m_direct != nullptr) {
					struct xrt_pose x[2];
					for (int e = 0; e < 2; e++) {
						x[e].orientation = {(float)eyes[e].orientation.x, (float)eyes[e].orientation.y,
						                    (float)eyes[e].orientation.z, (float)eyes[e].orientation.w};
						x[e].position = {(float)eyes[e].position.x, (float)eyes[e].position.y,
						                 (float)eyes[e].position.z};
					}
					m_direct->setEyes(x);
				}
				// The pose thread: under withActive, so not after Deactivate.
				withActive([&](uint32_t id, vr::PropertyContainerHandle_t container) {
					const float ipd = (float)(eyes[1].position.x - eyes[0].position.x);
					vr::VRServerDriverHost()->SetDisplayEyeToHead(id, toMatrix(eyes[0]), toMatrix(eyes[1]));
					vr::VRProperties()->SetFloatProperty(container, vr::Prop_UserIpdMeters_Float,
					                                     ipd);
					log("ovrn: IPD %.1f mm\n", ipd * 1000.0f);
				});
			}
		}
	}

	void *
	GetComponent(const char *name) override
	{
		if (name != nullptr && std::string(name) == vr::IVRDisplayComponent_Version) {
			return static_cast<vr::IVRDisplayComponent *>(this);
		}
		if (name != nullptr && m_direct != nullptr && std::string(name) == vr::IVRDriverDirectModeComponent_Version) {
			log("ovrn: direct mode: SteamVR asked for the direct mode component\n");
			return static_cast<vr::IVRDriverDirectModeComponent *>(m_direct);
		}
		if (name != nullptr && m_camera && std::string(name) == vr::IVRCameraComponent_Version) {
			return static_cast<vr::IVRCameraComponent *>(m_camera.get());
		}
		return nullptr;
	}

	// IVRDisplayComponent

	void
	GetWindowBounds(int32_t *x, int32_t *y, uint32_t *w, uint32_t *h) override
	{
		*x = 0;
		*y = 0;
		*w = m_display.width;
		*h = m_display.height;
	}

	bool
	IsDisplayOnDesktop() override
	{
		return false;
	}

	//! In direct mode the frames come to the driver: SteamVR must not take the headset's display itself (as ALVR).
	bool
	IsDisplayRealDisplay() override
	{
		return m_direct == nullptr;
	}

	void
	GetRecommendedRenderTargetSize(uint32_t *w, uint32_t *h) override
	{
		*w = m_display.eye[0].render_w;
		*h = m_display.eye[0].render_h;
	}

	void
	GetEyeOutputViewport(vr::EVREye eye, uint32_t *x, uint32_t *y, uint32_t *w, uint32_t *h) override
	{
		const EyeInfo &e = m_display.eye[eye == vr::Eye_Left ? 0 : 1];
		*x = e.viewport_x;
		*y = e.viewport_y;
		*w = e.viewport_w;
		*h = e.viewport_h;
	}

	//! Tangents of the half angles; OpenVR's top is the negative one (up is -y in its clip space).
	void
	GetProjectionRaw(vr::EVREye eye, float *left, float *right, float *top, float *bottom) override
	{
		const Fov &f = m_display.eye[eye == vr::Eye_Left ? 0 : 1].fov;
		*left = tanf(f.left);
		*right = tanf(f.right);
		*top = tanf(-f.up);
		*bottom = tanf(-f.down);
	}

	vr::DistortionCoordinates_t
	ComputeDistortion(vr::EVREye eye, float u, float v) override
	{
		DistortionUv d;
		if (!m_backend->distortion(eye == vr::Eye_Left ? 0 : 1, u, v, &d)) {
			d = {{u, v}, {u, v}, {u, v}};
		}
		vr::DistortionCoordinates_t c;
		c.rfRed[0] = d.r[0];
		c.rfRed[1] = d.r[1];
		c.rfGreen[0] = d.g[0];
		c.rfGreen[1] = d.g[1];
		c.rfBlue[0] = d.b[0];
		c.rfBlue[1] = d.b[1];
		return c;
	}

	bool
	ComputeInverseDistortion(vr::HmdVector2_t *, vr::EVREye, uint32_t, float, float) override
	{
		return false;
	}

protected:
	//! The camera's threads, stream and SteamVR frame queue end here, not when Cleanup destroys the headset.
	void
	deactivate() override
	{
		if (m_camera) {
			m_camera->shutdown();
		}
		if (m_direct != nullptr) {
			m_direct->stop();
		}
	}

	vr::EVRInitError
	activate(uint32_t object_id) override
	{
		vr::CVRPropertyHelpers *props = vr::VRProperties();
		setIdentity(m_info, m_backend->universe());
		// The driver's own chaperone file (rooms drawn in Room Setup are kept there), as Sony's driver does.
		const std::string chaperone_path = m_backend->chaperonePath();
		if (!chaperone_path.empty()) {
			props->SetStringProperty(m_container, vr::Prop_DriverProvidedChaperonePath_String, chaperone_path.c_str());
		}
		const std::string chaperone = m_backend->chaperoneJson();
		if (!chaperone.empty()) {
			props->SetStringProperty(m_container, vr::Prop_DriverProvidedChaperoneJson_String, chaperone.c_str());
			props->SetBoolProperty(m_container, vr::Prop_DriverProvidedChaperoneVisibility_Bool, true);
		}
		props->SetFloatProperty(m_container, vr::Prop_DisplayFrequency_Float, (float)m_display.refresh_hz);
		props->SetFloatProperty(m_container, vr::Prop_SecondsFromVsyncToPhotons_Float,
		                        (float)m_display.vsync_to_photons_s);
		if (m_display.photons_to_vblank_s >= 0.0) {
			props->SetFloatProperty(m_container, vr::Prop_SecondsFromPhotonsToVblank_Float,
			                        (float)m_display.photons_to_vblank_s);
		}
		if (m_display.distortion_mesh_resolution > 0) {
			props->SetInt32Property(m_container, vr::Prop_DistortionMeshResolution_Int32,
			                        m_display.distortion_mesh_resolution);
		}
		const double ipd = m_display.eye[1].eye_in_head.position.x - m_display.eye[0].eye_in_head.position.x;
		props->SetFloatProperty(m_container, vr::Prop_UserIpdMeters_Float, (float)ipd);
		props->SetFloatProperty(m_container, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.0f);
		props->SetBoolProperty(m_container, vr::Prop_IsOnDesktop_Bool, false);
		props->SetBoolProperty(m_container, vr::Prop_HasDisplayComponent_Bool, true);
		// Inside-out tracking: cameras on the headset looking at the world (as Sony's driver reports it; documented
		// effect: SteamVR's wording in its UI, e.g. Room Setup).
		props->SetInt32Property(m_container, vr::Prop_HmdTrackingStyle_Int32, vr::HmdTrackingStyle_InsideOutCameras);
		/*
		 * SteamVR's in-headset playspace setup (as on Steam Frame): its dashboard offers "Playspace Setup" and "Adjust
		 * Floor Height" only for a headset reporting SupportsInHMDRoomSetup_Bool (property 11003 in SteamVR 2.19's
		 * dashboard, systemui.js; not in the published openvr_driver.h 2.15.6). SteamVR ignores an unknown property.
		 * OVRN_IN_HMD_ROOM_SETUP=0 doesn't report it.
		 */
		{
			const char *e = getenv("OVRN_IN_HMD_ROOM_SETUP");
			if (e == nullptr || strcmp(e, "0") != 0) {
				constexpr auto kSupportsInHMDRoomSetup = (vr::ETrackedDeviceProperty)11003;
				props->SetBoolProperty(m_container, kSupportsInHMDRoomSetup, true);
				/*
				 * 11003 only shows the buttons. The dashboard's setRoomSetupStep runs every step but recenter only
				 * for a headset with VRLinkClientHMDSupportsRoomSetupRequests_Bool (11004) or with the
				 * steamvr.inHMDRoomSetupDebug setting, else "HMD doesnt support setRoomSetupStep" and nothing
				 * happens: with the setting removed (2026-11-19), Adjust Floor and Playspace Setup did nothing,
				 * recenter still worked. The dashboard treats 11004 specially only when SteamVR is itself a VR
				 * Link server (it forwards the step to the client), which a wired PS VR2 never is.
				 */
				constexpr auto kSupportsRoomSetupRequests = (vr::ETrackedDeviceProperty)11004;
				props->SetBoolProperty(m_container, kSupportsRoomSetupRequests, true);
				log("ovrn: in-headset playspace setup offered to SteamVR (SupportsInHMDRoomSetup_Bool, "
				    "VRLinkClientHMDSupportsRoomSetupRequests_Bool; OVRN_IN_HMD_ROOM_SETUP=0 off)\n");
			}
		}

		vr::HmdMatrix34_t eye[2];
		for (int e = 0; e < 2; e++) {
			eye[e] = toMatrix(m_display.eye[e].eye_in_head);
		}
		vr::VRServerDriverHost()->SetDisplayEyeToHead(object_id, eye[0], eye[1]);
		loadHiddenArea();
		setupDirectMode();
		setupCamera();
		setupHeadButton();
		setupProximity();
		setupEyeTracking();
		log("ovrn: headset %s (%s), %ux%u at %.2f Hz, render %ux%u per eye, IPD %.1f mm, vsync-to-photons %.5f s, "
		    "head pose: %s, backend %s\n",
		    m_info.model.c_str(), m_info.serial.c_str(), m_display.width, m_display.height, m_display.refresh_hz,
		    m_display.eye[0].render_w, m_display.eye[0].render_h, ipd * 1000.0, m_display.vsync_to_photons_s,
		    m_direct != nullptr ? "Monado's prediction for each frame's display time, zero velocity (direct mode)"
		    : m_driverPredicts  ? "predicted by the driver, zero velocity (OVRN_POSE_MODEL=predicted)"
		                        : "for now with velocities and its age, SteamVR predicts",
		    m_backend->name());
		return vr::VRInitError_None;
	}

private:
	/*!
	 * The head pose at @p at_ns as SteamVR takes it: with its velocities and age (@p with_velocity, SteamVR predicts
	 * from it), else zero velocity (a pose already predicted for its display time). Any thread: the last position is
	 * kept under its own mutex.
	 */
	vr::DriverPose_t
	buildPose(int64_t at_ns, bool with_velocity, int64_t now_ns)
	{
		TrackedPose t;
		vr::DriverPose_t pose = identityPose();
		pose.deviceIsConnected = true;
		setWorld(pose, m_backend->worldFromDriver());
		if (!m_backend->headPose(at_ns, &t) || !t.orientation_valid) {
			pose.poseIsValid = false;
			pose.result = vr::TrackingResult_Uninitialized;
			return pose;
		}
		pose.poseIsValid = true;
		pose.result = t.orientation_tracked ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange;
		pose.qRotation = {t.pose.orientation.w, t.pose.orientation.x, t.pose.orientation.y, t.pose.orientation.z};
		Vec3 position;
		{
			std::lock_guard<std::mutex> lock(m_lastPositionMutex);
			if (t.position_valid) {
				m_lastPosition = t.pose.position;
			}
			// Without a position the last known one is kept (not the origin: the view would jump there).
			position = m_lastPosition;
		}
		pose.vecPosition[0] = position.x;
		pose.vecPosition[1] = position.y;
		pose.vecPosition[2] = position.z;
		if (with_velocity) {
			pose.vecVelocity[0] = t.linear_velocity.x;
			pose.vecVelocity[1] = t.linear_velocity.y;
			pose.vecVelocity[2] = t.linear_velocity.z;
			const Vec3 w = rotateInverse(t.pose.orientation, t.angular_velocity);
			pose.vecAngularVelocity[0] = w.x;
			pose.vecAngularVelocity[1] = w.y;
			pose.vecAngularVelocity[2] = w.z;
			// The pose is for now_ns; SteamVR takes it as valid at the moment of the call plus this offset.
			pose.poseTimeOffset = (double)(now_ns - ovrn::now_ns()) * 1e-9;
		}
		return pose;
	}

	/*!
	 * Direct mode: one head pose per frame, Monado's prediction for the display time of the frame being rendered
	 * (zero velocity), the same pose for that frame from every thread (OvrnFramePose: the game's pose and SteamVR's
	 * layer label must match, or Monado re-warps from the wrong pose). False when not in direct mode or no frame
	 * target near now (before the first frame): the usual pose then.
	 */
	bool
	directFramePose(int64_t now_ns, vr::DriverPose_t *out, int64_t *out_target)
	{
		if (m_direct == nullptr) {
			return false;
		}
		const int64_t at = m_direct->displayTarget();
		if (at <= now_ns - 30'000'000LL || at >= now_ns + 100'000'000LL) {
			return false;
		}
		*out = m_framePose.get(at, [&] { return buildPose(at, false, now_ns); }, out_target);
		return true;
	}

	/*!
	 * Driver direct mode (DriverKit's setting): SteamVR renders into images the driver allocates and hands each
	 * finished frame over (IVRDriverDirectModeComponent, paced by the driver's vsync events), and Monado's compositor
	 * shows them on the headset. Before PostPresent's vsync event the new frame's pose is submitted, so SteamVR has it
	 * when it wakes the game (Valve's direct-mode sequence).
	 */
	void
	setupDirectMode()
	{
		if (m_direct == nullptr) {
			return;
		}
		vr::CVRPropertyHelpers *props = vr::VRProperties();
		props->SetBoolProperty(m_container, vr::Prop_HasDriverDirectModeComponent_Bool, true);
		props->SetBoolProperty(m_container, vr::Prop_DriverDirectModeSendsVsyncEvents_Bool, true);
		m_direct->on_frame_start = [this](int64_t display_ns) {
			withActive([&](uint32_t id, vr::PropertyContainerHandle_t) {
				int64_t used = display_ns;
				const vr::DriverPose_t pose =
				    m_framePose.get(display_ns, [&] { return buildPose(display_ns, false, now_ns()); }, &used);
				m_direct->logSubmittedPose(used, pose);
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id, pose, sizeof(vr::DriverPose_t));
			});
		};
		m_direct->start((float)m_display.refresh_hz, getenv("OVRN_DIRECT_MODE_LOG"));
	}

	/*!
	 * The passthrough cameras (SteamVR's Room View and the camera in Room Setup), when the backend has them. Their
	 * calibration comes with the headset's calibration block; waits up to 3 s for it. OVRN_CAMERA=0: no camera.
	 */
	/*!
	 * The headset's wear sensor as /proximity (Sony's headset profile and legacy binding: /user/head/proximity) and
	 * Prop_ContainsProximitySensor_Bool, as Sony's driver reports them. Games read it for "headset on"; AMID EVIL VR
	 * asked for it and got "Invalid input type button::click for path (/user/head/proximity)" (2026-11-19).
	 */
	/*!
	 * The headset's eye tracking as SteamVR's eye-tracking input, as Valve's driver API and the working references
	 * (Pimax-EyeTracker-SteamVR, PSVR2Toolkit) do: Prop_SupportsXrEyeGazeInteraction_Bool and the component
	 * "/eyetracking" (exactly that path), which SteamVR's OpenXR runtime exposes as XR_EXT_eye_gaze_interaction.
	 * Only when the backend really tracks the eyes (no capability is reported otherwise).
	 */
	void
	setupEyeTracking()
	{
		if (!m_backend->hasEyeTracking() || vr::VRDriverInput() == nullptr) {
			return;
		}
		if (vr::VRDriverInput()->CreateEyeTrackingComponent(m_container, "/eyetracking", &m_eyeHandle) !=
		    vr::VRInputError_None) {
			m_eyeHandle = vr::k_ulInvalidInputComponentHandle;
			log("ovrn: SteamVR refused the eye-tracking component\n");
			return;
		}
		vr::VRProperties()->SetBoolProperty(m_container, vr::Prop_SupportsXrEyeGazeInteraction_Bool, true);
		log("ovrn: eye tracking as SteamVR's /eyetracking input (XR_EXT_eye_gaze_interaction)\n");
	}

	/*!
	 * The latest gaze to SteamVR (RunFrame thread): head-space unit direction as vGazeTarget, origin at the head
	 * (no per-eye origin from this headset yet), all three flags from the headset's validity, as both references do.
	 * A sample whose packet is older than kEyeStaleNs is reported as not valid, never as current gaze. The time
	 * offset says how old the sample is (negative, seconds).
	 */
	void
	updateEyeTracking()
	{
		if (m_eyeHandle == vr::k_ulInvalidInputComponentHandle) {
			return;
		}
		static constexpr int64_t kEyeStaleNs = 100'000'000; // 100 ms; the headset streams gaze at ~120 Hz
		Backend::EyeGaze g{};
		vr::VREyeTrackingData_t d{};
		d.vGazeTarget = {0.0f, 0.0f, -1.0f};
		double offset_s = 0.0;
		if (m_backend->eyeGaze(&g)) {
			const bool fresh = g.packet_age_ns >= 0 && g.packet_age_ns < kEyeStaleNs;
			const bool ok = g.valid && fresh && std::isfinite(g.dir.x) && std::isfinite(g.dir.y) &&
			                std::isfinite(g.dir.z);
			d.bActive = d.bValid = d.bTracked = ok;
			if (ok) {
				d.vGazeTarget = {(float)g.dir.x, (float)g.dir.y, (float)g.dir.z};
				const double age_s = (double)((int64_t)os_monotonic_get_ns() - g.sample_ns) / 1e9;
				offset_s = (age_s > 0.0 && age_s < 0.1) ? -age_s : 0.0;
			}
			m_eyeFresh += ok ? 1 : 0;
			m_eyeHadSample = true;
			if (ok && !m_eyeLoggedFirst) {
				m_eyeLoggedFirst = true;
				log("ovrn: eye tracking: first valid gaze (yaw %+.1f, pitch %+.1f deg)\n", gazeYawDeg(d.vGazeTarget),
				    gazePitchDeg(d.vGazeTarget));
			}
		}
		m_eyeUpdates++;
		vr::VRDriverInput()->UpdateEyeTrackingComponent(m_eyeHandle, &d, offset_s);

		// Every 30 s: how much of the gaze SteamVR got was valid, and the latest direction (+yaw right, +pitch up).
		const int64_t now = (int64_t)os_monotonic_get_ns();
		if (m_eyeLogNs == 0) {
			m_eyeLogNs = now;
		} else if (now - m_eyeLogNs >= 30'000'000'000LL) {
			const uint64_t n = m_eyeUpdates - m_eyeUpdatesAtLog, v = m_eyeFresh - m_eyeFreshAtLog;
			if (!m_eyeHadSample) {
				log("ovrn: eye tracking: no gaze from the headset in the last 30 s\n");
			} else {
				log("ovrn: eye tracking: %.0f%% of %llu gaze updates valid in 30 s; last %s yaw %+.1f, pitch %+.1f deg\n",
				    n ? 100.0 * (double)v / (double)n : 0.0, (unsigned long long)n, d.bValid ? "valid" : "(not valid)",
				    gazeYawDeg(d.vGazeTarget), gazePitchDeg(d.vGazeTarget));
			}
			m_eyeLogNs = now;
			m_eyeUpdatesAtLog = m_eyeUpdates;
			m_eyeFreshAtLog = m_eyeFresh;
		}
	}

	static float
	gazeYawDeg(const vr::HmdVector3_t &t)
	{
		return std::atan2(t.v[0], -t.v[2]) * 57.2958f;
	}

	static float
	gazePitchDeg(const vr::HmdVector3_t &t)
	{
		return std::atan2(t.v[1], std::sqrt(t.v[0] * t.v[0] + t.v[2] * t.v[2])) * 57.2958f;
	}

	void
	setupProximity()
	{
		if (!m_backend->hasProximity() || vr::VRDriverInput() == nullptr) {
			return;
		}
		vr::VRProperties()->SetBoolProperty(m_container, vr::Prop_ContainsProximitySensor_Bool, true);
		if (vr::VRDriverInput()->CreateBooleanComponent(m_container, "/proximity", &m_proximityHandle) !=
		    vr::VRInputError_None) {
			m_proximityHandle = vr::k_ulInvalidInputComponentHandle;
			return;
		}
		log("ovrn: headset wear sensor as /proximity\n");
	}

	void
	setupHeadButton()
	{
		const char *mode = getenv("OVRN_HEAD_BUTTON");
		if ((mode != nullptr && strcmp(mode, "off") == 0) || vr::VRDriverInput() == nullptr) {
			return;
		}
		if (vr::VRDriverInput()->CreateBooleanComponent(m_container, "/input/system/click", &m_buttonHandle) !=
		    vr::VRInputError_None) {
			m_buttonHandle = vr::k_ulInvalidInputComponentHandle;
			return;
		}
		log("ovrn: headset button: SteamVR's Toggle Room View (our compositor binding)\n");
	}

	void
	setupCamera()
	{
		/*
		 * Driver direct mode: no camera for SteamVR. With a direct-mode driver SteamVR's compositor always uses its
		 * Facet renderer (vrcompositor 2.18.4 forces useFacetRenderer on when its IVRDriverDirectInternal interface
		 * is there, whatever the setting), and Facet aborts on SteamVR's tracked-camera code: "ASSERT NOIMPL at
		 * facetrenderer.cpp:3572" < 1 s after Room View was created, 2026-11-24 11:37:30 and 11:37:54 (-203);
		 * SteamVR-for-Linux #909, the same assert with a PS VR2: Facet lacks the event queries the camera uses.
		 */
		if (m_direct != nullptr) {
			log("ovrn: headset cameras: off in direct mode (SteamVR's Facet renderer, which direct mode uses, can't "
			    "show its camera views: SteamVR-for-Linux #909)\n");
			return;
		}
		std::string source, invalid;
		const CameraMode mode = camera_mode(&source, &invalid);
		if (!invalid.empty()) {
			log("ovrn: headset cameras setting '%s' is not on, noimages or off: using on\n", invalid.c_str());
		}
		if (mode == CameraMode::Off) {
			log("ovrn: headset cameras: off (%s): no camera for SteamVR\n", source.c_str());
			return;
		}
		if (mode == CameraMode::NoImages) {
			log("ovrn: headset cameras: noimages (%s): SteamVR's camera set up, no picture sent\n", source.c_str());
		}
		if (!m_backend->hasCameras()) {
			return;
		}
		CameraInfo info;
		bool have = m_backend->cameraInfo(&info);
		for (int i = 0; !have && i < 30; i++) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			have = m_backend->cameraInfo(&info);
		}
		if (!have) {
			log("ovrn: no passthrough cameras (the backend has none, or their calibration didn't arrive)\n");
			return;
		}
		info.no_images = mode == CameraMode::NoImages;
		m_camera.reset(new Camera(m_backend, info));
		m_camera->setProperties(m_container);
	}

	vr::VRInputComponentHandle_t m_buttonHandle = vr::k_ulInvalidInputComponentHandle;
	bool m_buttonSent = false;
	vr::VRInputComponentHandle_t m_proximityHandle = vr::k_ulInvalidInputComponentHandle;
	bool m_wornSent = false;
	PresenceLog m_presence; //!< wear sensor, head tracking and SteamVR's standby verdicts, logged every 30 s
	bool m_wornSentOnce = false;
	vr::VRInputComponentHandle_t m_eyeHandle = vr::k_ulInvalidInputComponentHandle;
	uint64_t m_eyeUpdates = 0; //!< RunFrame thread: gaze updates sent, and how many carried valid gaze
	uint64_t m_eyeFresh = 0;
	bool m_eyeHadSample = false, m_eyeLoggedFirst = false;
	int64_t m_eyeLogNs = 0;
	uint64_t m_eyeUpdatesAtLog = 0, m_eyeFreshAtLog = 0;
	std::unique_ptr<Camera> m_camera;

	//! The hidden-area mask from the backend's file, handed to SteamVR as it is (render target UV).
	void
	loadHiddenArea()
	{
		if (m_info.hidden_area_file.empty()) {
			return;
		}
		FILE *f = fopen(m_info.hidden_area_file.c_str(), "r");
		if (f == nullptr) {
			log("ovrn: no hidden-area mask (%s not found)\n", m_info.hidden_area_file.c_str());
			return;
		}
		std::vector<vr::HmdVector2_t> verts[2][3];
		char line[256];
		while (fgets(line, sizeof line, f) != nullptr) {
			int eye, type;
			float u, v;
			if (sscanf(line, "ham_vert %d %d %f %f", &eye, &type, &u, &v) == 4 && eye >= 0 && eye < 2 &&
			    type >= 0 && type < 3) {
				verts[eye][type].push_back({{u, v}});
			}
		}
		fclose(f);
		for (int e = 0; e < 2; e++) {
			if (verts[e][0].size() < 3 || verts[e][0].size() % 3 != 0) {
				log("ovrn: hidden-area mask for eye %d in %s is not a triangle list, not used\n", e,
				    m_info.hidden_area_file.c_str());
				continue;
			}
			double area = 0;
			for (size_t i = 0; i + 2 < verts[e][0].size(); i += 3) {
				const float *A = verts[e][0][i].v, *B = verts[e][0][i + 1].v, *C = verts[e][0][i + 2].v;
				area += std::fabs((B[0] - A[0]) * (C[1] - A[1]) - (C[0] - A[0]) * (B[1] - A[1])) / 2;
			}
			for (int t = 0; t < 3; t++) {
				if (!verts[e][t].empty()) {
					vr::VRHiddenArea()->SetHiddenArea((vr::EVREye)e, (vr::EHiddenAreaMeshType)t, verts[e][t].data(),
					                                  (uint32_t)verts[e][t].size());
				}
			}
			log("ovrn: hidden-area mask, eye %d: %zu triangles, %.1f%% of the render target (%s)\n", e,
			    verts[e][0].size() / 3, area * 100.0, m_info.hidden_area_file.c_str());
		}
	}

	static vr::HmdMatrix34_t
	toMatrix(const Pose &p)
	{
		const double x = p.orientation.x, y = p.orientation.y, z = p.orientation.z, w = p.orientation.w;
		vr::HmdMatrix34_t m;
		m.m[0][0] = (float)(1 - 2 * (y * y + z * z));
		m.m[0][1] = (float)(2 * (x * y - z * w));
		m.m[0][2] = (float)(2 * (x * z + y * w));
		m.m[1][0] = (float)(2 * (x * y + z * w));
		m.m[1][1] = (float)(1 - 2 * (x * x + z * z));
		m.m[1][2] = (float)(2 * (y * z - x * w));
		m.m[2][0] = (float)(2 * (x * z - y * w));
		m.m[2][1] = (float)(2 * (y * z + x * w));
		m.m[2][2] = (float)(1 - 2 * (x * x + y * y));
		m.m[0][3] = (float)p.position.x;
		m.m[1][3] = (float)p.position.y;
		m.m[2][3] = (float)p.position.z;
		return m;
	}

	Backend *const m_backend;
	const DeviceInfo m_info;
	const DisplayInfo m_display;
	bool m_driverPredicts = false;
	OvrnDirectMode *const m_direct; //!< the backend's, nullptr unless driver direct mode is on
	OvrnFramePose m_framePose;      //!< direct mode: the one head pose of each frame
	std::mutex m_lastPositionMutex;
	Vec3 m_lastPosition; //!< under m_lastPositionMutex (pose thread, direct mode's frame thread)
	int64_t m_eyeCheckNs = 0; //!< pose thread only
};

} // namespace ovrn
