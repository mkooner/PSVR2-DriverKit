// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The headset: a plain display device. SteamVR owns the display, composites, paces and reprojects; the
 *         driver describes the display and hands over head poses (after ALVR's HMD.cpp on Linux: IVRDisplayComponent,
 *         IsDisplayRealDisplay true, no direct mode component).
 * @ingroup st_ovrn
 */

#pragma once

#include "ovrn_camera.hpp"
#include "ovrn_device.hpp"

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
	/*!
	 * The headset's button to SteamVR (RunFrame thread). OVRN_HEAD_BUTTON:
	 *  - roomview (default): each press is a double-click of the headset's system button, the gesture SteamVR's
	 *    Room View setting ("double-click system button") toggles the camera view with, so the button toggles
	 *    passthrough as it does on a PS5;
	 *  - system: the headset's system button as it is;
	 *  - off: not reported.
	 */
	void
	updateInputs(int64_t now)
	{
		if (m_buttonHandle == vr::k_ulInvalidInputComponentHandle) {
			return;
		}
		const bool down = m_backend->headButton();
		if (!m_buttonDoubleClick) {
			if (down != m_buttonSent) {
				m_buttonSent = down;
				vr::VRDriverInput()->UpdateBooleanComponent(m_buttonHandle, down, 0.0);
			}
			return;
		}
		if (down && !m_buttonWasDown && m_clickStartNs == 0) {
			m_clickStartNs = now;
		}
		m_buttonWasDown = down;
		if (m_clickStartNs == 0) {
			return;
		}
		// down 0-60 ms, up 60-120, down 120-180, up after: two clicks well inside a double-click window.
		const int64_t t = now - m_clickStartNs;
		const bool want = (t < 60000000) || (t >= 120000000 && t < 180000000);
		if (want != m_buttonSent) {
			m_buttonSent = want;
			vr::VRDriverInput()->UpdateBooleanComponent(m_buttonHandle, want, 0.0);
		}
		if (t >= 240000000) {
			m_clickStartNs = 0;
		}
	}

	explicit Hmd(Backend *backend)
	    : TrackedDevice(vr::TrackedDeviceClass_HMD, backend->headInfo().serial), m_backend(backend),
	      m_info(backend->headInfo()), m_display(backend->display())
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
		TrackedPose t;
		vr::DriverPose_t pose = identityPose();
		pose.deviceIsConnected = true;
		setWorld(pose, m_backend->worldFromDriver());
		if (!m_backend->headPose(now_ns + predict_ns.load(), &t) || !t.orientation_valid) {
			pose.poseIsValid = false;
			pose.result = vr::TrackingResult_Uninitialized;
			submitPose(pose);
			return;
		}
		pose.poseIsValid = true;
		pose.result = t.orientation_tracked ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange;
		pose.qRotation = {t.pose.orientation.w, t.pose.orientation.x, t.pose.orientation.y, t.pose.orientation.z};
		if (t.position_valid) {
			m_lastPosition = t.pose.position;
		}
		// Without a position the last known one is kept (not the origin: the view would jump there).
		pose.vecPosition[0] = m_lastPosition.x;
		pose.vecPosition[1] = m_lastPosition.y;
		pose.vecPosition[2] = m_lastPosition.z;
		if (!m_driverPredicts) {
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
		submitPose(pose);

		// The IPD dial: twice a second, new eye poses go to SteamVR when they changed.
		if (now_ns - m_eyeCheckNs > 500000000LL) {
			m_eyeCheckNs = now_ns;
			Pose eyes[2];
			if (m_backend->eyePoses(eyes)) {
				const uint32_t id = m_objectId.load();
				if (id != vr::k_unTrackedDeviceIndexInvalid) {
					const float ipd = (float)(eyes[1].position.x - eyes[0].position.x);
					vr::VRServerDriverHost()->SetDisplayEyeToHead(id, toMatrix(eyes[0]), toMatrix(eyes[1]));
					vr::VRProperties()->SetFloatProperty(m_container, vr::Prop_UserIpdMeters_Float, ipd);
					log("ovrn: IPD %.1f mm\n", ipd * 1000.0f);
				}
			}
		}
	}

	void *
	GetComponent(const char *name) override
	{
		if (name != nullptr && std::string(name) == vr::IVRDisplayComponent_Version) {
			return static_cast<vr::IVRDisplayComponent *>(this);
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

	bool
	IsDisplayRealDisplay() override
	{
		return true;
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

		vr::HmdMatrix34_t eye[2];
		for (int e = 0; e < 2; e++) {
			eye[e] = toMatrix(m_display.eye[e].eye_in_head);
		}
		vr::VRServerDriverHost()->SetDisplayEyeToHead(object_id, eye[0], eye[1]);
		loadHiddenArea();
		setupCamera();
		setupHeadButton();
		log("ovrn: headset %s (%s), %ux%u at %.2f Hz, render %ux%u per eye, IPD %.1f mm, vsync-to-photons %.5f s, "
		    "head pose: %s, backend %s\n",
		    m_info.model.c_str(), m_info.serial.c_str(), m_display.width, m_display.height, m_display.refresh_hz,
		    m_display.eye[0].render_w, m_display.eye[0].render_h, ipd * 1000.0, m_display.vsync_to_photons_s,
		    m_driverPredicts ? "predicted by the driver, zero velocity (OVRN_POSE_MODEL=predicted)"
		                     : "for now with velocities and its age, SteamVR predicts",
		    m_backend->name());
		return vr::VRInitError_None;
	}

private:
	/*!
	 * The passthrough cameras (SteamVR's Room View and the camera in Room Setup), when the backend has them. Their
	 * calibration comes with the headset's calibration block; waits up to 3 s for it. OVRN_CAMERA=0: no camera.
	 */
	void
	setupHeadButton()
	{
		const char *mode = getenv("OVRN_HEAD_BUTTON");
		const std::string m = mode != nullptr ? mode : "roomview";
		if (m == "off" || vr::VRDriverInput() == nullptr) {
			return;
		}
		m_buttonDoubleClick = m != "system";
		if (vr::VRDriverInput()->CreateBooleanComponent(m_container, "/input/system/click", &m_buttonHandle) !=
		    vr::VRInputError_None) {
			m_buttonHandle = vr::k_ulInvalidInputComponentHandle;
			return;
		}
		log("ovrn: headset button: %s\n", m_buttonDoubleClick ? "toggles Room View (a system button double-click)"
		                                                        : "the headset's system button");
	}

	void
	setupCamera()
	{
		const char *env = getenv("OVRN_CAMERA");
		if (env != nullptr && strcmp(env, "0") == 0) {
			log("ovrn: passthrough cameras off (OVRN_CAMERA=0)\n");
			return;
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
		m_camera.reset(new Camera(m_backend, info));
		m_camera->setProperties(m_container);
	}

	vr::VRInputComponentHandle_t m_buttonHandle = vr::k_ulInvalidInputComponentHandle;
	bool m_buttonDoubleClick = true;
	bool m_buttonWasDown = false;
	bool m_buttonSent = false;
	int64_t m_clickStartNs = 0;
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
	Vec3 m_lastPosition;      //!< pose thread only
	int64_t m_eyeCheckNs = 0; //!< pose thread only
};

} // namespace ovrn
