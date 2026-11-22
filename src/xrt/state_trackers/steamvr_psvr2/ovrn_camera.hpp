// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The headset's passthrough cameras for SteamVR (Room View, Room Setup): the PS VR2's bottom camera pair,
 *         the pair Sony's driver shows, as one side-by-side greyscale NV12 frame.
 *
 * Two ways to SteamVR, both fed from the same frames:
 *  - the published IVRCameraComponent (openvr_driver.h): SteamVR's frame buffers (SetCameraFrameBuffering) or our
 *    own, GetVideoStreamFrame, and the sink callback;
 *  - SteamVR's internal camera frame queue "/lighthouse/camera/raw_frames" (ovrn_steamvr_internal.hpp), which is how
 *    PSVR2Toolkit gets the picture shown with SteamVR 2.x; used only when SteamVR hands the interfaces out.
 *
 * The headset driver calls back on its USB thread with the BC4 image pair; that only copies it. A worker thread
 * decodes, brightens (the frames average ~25/255; OVRN_CAMERA_GAMMA, default 0.5 as the hand tracking uses) and
 * delivers. Images are streamed only while SteamVR wants them, so the headset's camera mode is changed only then:
 * between StartVideoStream and StopVideoStream, or while the internal frame queue has a reader. SteamVR 2.19's
 * Room View and its settings page camera test never called StartVideoStream on this driver (2026-11-18 20:02-20:17,
 * the compositor set up Room View but no start ever came), so the queue's reader is watched as well (4x a second;
 * stopped 2 s after the last reader leaves, so a brief reconnect doesn't toggle the headset's camera mode).
 *
 * Lens model and frustum follow PSVR2Toolkit's hmd_device_camera.cpp (MIT): Extended_FTheta for the distorted
 * frame, and a 3.25x zoom (focal length / 3.25) for the undistorted one so it covers the cameras' wide view.
 * @ingroup st_ovrn
 */

#pragma once

#include "ovrn_backend.hpp"
#include "ovrn_log.hpp"
#include "ovrn_steamvr_internal.hpp"

#include "openvr_driver.h"

#define BCDEC_BC4BC5_PRECISE
#include "bcdec.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace ovrn {

class Camera final : public vr::IVRCameraComponent
{
public:
	static constexpr uint32_t kStride = 1024; //!< BC4 rows are 1024 pixels wide; the image is the left 1016
	static constexpr float kZoom = 3.25f;     //!< undistorted view: focal length / kZoom (PSVR2Toolkit's value)
	static constexpr double kRate = 59.94;    //!< mode 0x10 image pairs per second (the headset's camera rate)

	Camera(Backend *backend, const CameraInfo &info) : m_backend(backend), m_info(info)
	{
		/*
		 * Half size (CameraInfo::half, DriverKit's "Lighter camera picture"): each 2x2 pixels averaged, a quarter of
		 * the data. SteamVR's compositor copies every frame to the GPU through its transfer ring buffer while it
		 * shows the camera (a user's PC, 2026-11-21: 3.1 MB 60 times a second filled it every 1.43 s, and it once
		 * aborted on a full one). Pixel-based values SteamVR gets (frame size, intrinsics) are scaled with it; the
		 * normalized and angular ones (distortion, projection) don't change.
		 */
		m_scale = info.half ? 2 : 1;
		m_w = 2 * info.width / m_scale; // side by side
		m_h = info.height / m_scale;
		m_frameBytes = (size_t)m_w * m_h * 3 / 2;
		m_own[0].resize(m_frameBytes);
		m_own[1].resize(m_frameBytes);
		const char *g = getenv("OVRN_CAMERA_GAMMA");
		const double parsed = g != nullptr ? strtod(g, nullptr) : 0.0;
		const float gamma = parsed > 0.0 && parsed < 4.0 ? (float)parsed : 0.5f;
		for (int i = 0; i < 256; i++) {
			m_lut[i] = (uint8_t)lroundf(255.0f * powf(i / 255.0f, gamma));
		}
		m_gamma = gamma;
	}

	~Camera()
	{
		shutdown();
	}

	/*!
	 * Stops for good: the watcher, the stream and its worker, then SteamVR's frame queue. The headset calls it at
	 * its Deactivate, so nothing writes to SteamVR after that (OpenVR); a later StartVideoStream doesn't start it
	 * again. Once only (the destructor calls it too).
	 */
	void
	shutdown()
	{
		{
			std::lock_guard<std::mutex> l(m_watchMutex);
			m_watchQuit = true;
			m_watchCv.notify_all();
		}
		if (m_watcher.joinable()) {
			m_watcher.join();
		}
		{
			std::lock_guard<std::mutex> l(m_controlMutex);
			if (m_shutDown) {
				return;
			}
			m_shutDown = true;
			m_streamRequested = false;
			m_readerActive = false;
			stop();
		}
		if (m_queue != nullptr && m_queueHandle != vr::k_ulInvalidPropertyContainer) {
			m_queue->Destroy(m_queueHandle);
		}
		m_queue = nullptr;
		m_queueHandle = vr::k_ulInvalidPropertyContainer;
	}

	/*!
	 * The headset's camera properties, and SteamVR's internal frame queue if it offers one (HMD activation).
	 * Property values as PSVR2Toolkit sets them for Sony's driver, including the three version numbers it notes
	 * SteamVR needs before it shows a camera.
	 */
	void
	setProperties(vr::PropertyContainerHandle_t c)
	{
		vr::CVRPropertyHelpers *p = vr::VRProperties();
		p->SetBoolProperty(c, vr::Prop_HasCamera_Bool, true);
		p->SetBoolProperty(c, vr::Prop_HasCameraComponent_Bool, true);
		p->SetBoolProperty(c, vr::Prop_AllowCameraToggle_Bool, true);
		p->SetInt32Property(c, vr::Prop_NumCameras_Int32, 2);
		p->SetUint64Property(c, vr::Prop_FPGAVersion_Uint64, 0x104);
		p->SetUint64Property(c, vr::Prop_FirmwareVersion_Uint64, 0x56456BA0);
		p->SetUint64Property(c, vr::Prop_CameraFirmwareVersion_Uint64, 0x200040049);
		vr::HmdMatrix34_t toHead[2];
		for (int i = 0; i < 2; i++) {
			toHead[i] = matrix(m_info.cam_in_head[i]);
		}
		p->SetProperty(c, vr::Prop_CameraToHeadTransform_Matrix34, &toHead[0], sizeof(vr::HmdMatrix34_t),
		               vr::k_unHmdMatrix34PropertyTag);
		p->SetProperty(c, vr::Prop_CameraToHeadTransforms_Matrix34_Array, toHead, sizeof(toHead),
		               vr::k_unHmdMatrix34PropertyTag);
		p->SetInt32Property(c, vr::Prop_CameraFrameLayout_Int32,
		                    vr::EVRTrackedCameraFrameLayout_Stereo | vr::EVRTrackedCameraFrameLayout_HorizontalLayout);
		p->SetInt32Property(c, vr::Prop_CameraStreamFormat_Int32, vr::CVS_FORMAT_NV12);
		int32_t fn[2] = {vr::VRDistortionFunctionType_Extended_FTheta, vr::VRDistortionFunctionType_Extended_FTheta};
		p->SetProperty(c, vr::Prop_CameraDistortionFunction_Int32_Array, fn, sizeof(fn), vr::k_unInt32PropertyTag);
		double coeffs[2][vr::k_unMaxDistortionFunctionParameters];
		memcpy(coeffs, m_info.ftheta, sizeof(coeffs));
		p->SetProperty(c, vr::Prop_CameraDistortionCoefficients_Float_Array, coeffs, sizeof(coeffs),
		               vr::k_unDoublePropertyTag);
		vr::HmdVector4_t white[2] = {{{1, 1, 1, 1}}, {{1, 1, 1, 1}}};
		p->SetProperty(c, vr::Prop_CameraWhiteBalance_Vector4_Array, white, sizeof(white),
		               vr::k_unHmdVector4PropertyTag);
		p->SetFloatProperty(c, vr::Prop_CameraExposureTime_Float, (float)(1.0 / kRate));
		p->SetFloatProperty(c, vr::Prop_CameraGlobalGain_Float, 1.0f);

		vr::EVRInitError e = vr::VRInitError_None;
		m_queue = static_cast<steamvr::FrameQueue *>(
		    vr::VRDriverContext()->GetGenericInterface(steamvr::kFrameQueueVersion, &e));
		m_paths = static_cast<steamvr::Paths *>(
		    vr::VRDriverContext()->GetGenericInterface(steamvr::kPathsVersion, &e));
		if (m_queue != nullptr && m_paths != nullptr &&
		    m_queue->Create(&m_queueHandle, "/lighthouse/camera/raw_frames", (uint32_t)m_frameBytes, 512, 8) ==
		        steamvr::QueueResult::Ok) {
			steamvr::set_path(m_paths, m_queueHandle, "/width", (int32_t)m_w);
			steamvr::set_path(m_paths, m_queueHandle, "/height", (int32_t)m_h);
			steamvr::set_path(m_paths, m_queueHandle, "/format", (int32_t)vr::CVS_FORMAT_NV12);
			m_watcher = std::thread(&Camera::watchLoop, this);
		} else {
			m_queue = nullptr;
			m_queueHandle = vr::k_ulInvalidPropertyContainer;
		}
		float pitch[2], yaw[2];
		for (int i = 0; i < 2; i++) {
			view_angles(m_info.cam_in_head[i], &pitch[i], &yaw[i]);
		}
		const Pose &p0 = m_info.cam_in_head[0], &p1 = m_info.cam_in_head[1];
		const double baseline = std::sqrt((p1.position.x - p0.position.x) * (p1.position.x - p0.position.x) +
		                                  (p1.position.y - p0.position.y) * (p1.position.y - p0.position.y) +
		                                  (p1.position.z - p0.position.z) * (p1.position.z - p0.position.z));
		log("ovrn: passthrough cameras: %ux%u side by side (%s, %s), NV12, gamma %.2f; lens model fit %.2f / %.2f px "
		    "rms; camera 0 looks %.1f deg %s, %.1f deg %s, camera 1 %.1f deg %s, %.1f deg %s, %.0f mm apart (%s); "
		    "SteamVR's camera frame queue: %s\n",
		    m_w, m_h, m_scale == 2 ? "half size" : "full size", m_info.half_source.c_str(), m_gamma,
		    m_info.fit_rms_px[0], m_info.fit_rms_px[1], std::fabs(pitch[0]),
		    pitch[0] < 0 ? "down" : "up", std::fabs(yaw[0]), yaw[0] > 0 ? "left" : "right", std::fabs(pitch[1]),
		    pitch[1] < 0 ? "down" : "up", std::fabs(yaw[1]), yaw[1] > 0 ? "left" : "right", baseline * 1000.0,
		    m_info.tracker_poses ? "the controller tracker's camera poses, OVRN_CAMERA_POSES=tracker"
		                         : "measured placement",
		    m_queue != nullptr ? "yes (used)" : "not offered (published interface only)");
	}

	// IVRCameraComponent

	bool
	GetCameraFrameDimensions(vr::ECameraVideoStreamFormat, uint32_t *w, uint32_t *h) override
	{
		*w = m_w;
		*h = m_h;
		return true;
	}

	bool
	GetCameraFrameBufferingRequirements(int *count, uint32_t *bytes) override
	{
		*count = 3;
		*bytes = (uint32_t)m_frameBytes;
		return true;
	}

	bool
	SetCameraFrameBuffering(int count, void **buffers, uint32_t bytes) override
	{
		std::lock_guard<std::mutex> l(m_frameMutex);
		m_steamBuffers.clear();
		if (buffers != nullptr && count > 0 && bytes >= m_frameBytes) {
			m_steamBuffers.assign(buffers, buffers + count);
		}
		return true;
	}

	bool
	SetCameraVideoStreamFormat(vr::ECameraVideoStreamFormat format) override
	{
		return format == vr::CVS_FORMAT_NV12;
	}

	vr::ECameraVideoStreamFormat
	GetCameraVideoStreamFormat() override
	{
		return vr::CVS_FORMAT_NV12;
	}

	bool
	StartVideoStream() override
	{
		std::lock_guard<std::mutex> l(m_controlMutex);
		m_streamRequested = true;
		return update("SteamVR's StartVideoStream");
	}

	void
	StopVideoStream() override
	{
		std::lock_guard<std::mutex> l(m_controlMutex);
		m_streamRequested = false;
		update("SteamVR's StopVideoStream");
	}

	/*!
	 * One look at the internal queue's reader (the watcher thread calls it 4x a second; public for the tests):
	 * streams while a reader is there, stops @p linger_ns after the last one left.
	 */
	void
	readerCheck(bool has_reader, int64_t now, int64_t linger_ns = 2000000000)
	{
		std::lock_guard<std::mutex> l(m_controlMutex);
		if (has_reader) {
			m_lastReaderNs = now;
			if (!m_readerActive) {
				m_readerActive = true;
				update("a reader on SteamVR's camera frame queue");
			}
		} else if (m_readerActive && now - m_lastReaderNs >= linger_ns) {
			m_readerActive = false;
			update("no reader on SteamVR's camera frame queue");
		}
	}

	bool
	IsVideoStreamActive(bool *paused, float *elapsed) override
	{
		if (paused != nullptr) {
			*paused = !m_running.load();
		}
		if (elapsed != nullptr) {
			*elapsed = m_running.load() ? (float)((now_ns() - m_startNs) * 1e-9) : 0.0f;
		}
		return m_running.load();
	}

	const vr::CameraVideoStreamFrame_t *
	GetVideoStreamFrame() override
	{
		std::lock_guard<std::mutex> l(m_frameMutex);
		return m_haveFrame ? &m_frame : nullptr;
	}

	void
	ReleaseVideoStreamFrame(const vr::CameraVideoStreamFrame_t *) override
	{}

	bool
	SetAutoExposure(bool) override
	{
		return false;
	}

	bool
	PauseVideoStream() override
	{
		std::lock_guard<std::mutex> l(m_controlMutex);
		m_streamRequested = false;
		update("SteamVR's PauseVideoStream");
		return true;
	}

	bool
	ResumeVideoStream() override
	{
		std::lock_guard<std::mutex> l(m_controlMutex);
		m_streamRequested = true;
		return update("SteamVR's ResumeVideoStream");
	}

	bool
	GetCameraDistortion(uint32_t cam, float u, float v, float *out_u, float *out_v) override
	{
		if (cam >= 2) {
			return false;
		}
		// From the (zoomed) undistorted image to the camera's own image, both normalized 0..1 per camera.
		const double x = (u * m_info.width - cx(cam)) / m_info.fx[cam] * kZoom;
		const double y = (v * m_info.height - m_info.cy[cam]) / m_info.fy[cam] * kZoom;
		const double r = std::hypot(x, y);
		double s = 1.0;
		if (r > 1e-9) {
			const double t = std::atan(r);
			double rd = t;
			for (int k = 0; k < 4; k++) {
				rd += m_info.ftheta[cam][k] * std::pow(t, 2 * k + 3);
			}
			s = rd / r;
		}
		*out_u = (float)((x * s * m_info.fx[cam] + cx(cam)) / m_info.width);
		*out_v = (float)((y * s * m_info.fy[cam] + m_info.cy[cam]) / m_info.height);
		return true;
	}

	bool
	GetCameraProjection(uint32_t cam, vr::EVRTrackedCameraFrameType, float n, float f, vr::HmdMatrix44_t *out) override
	{
		if (cam >= 2) {
			return false;
		}
		const double fx = m_info.fx[cam] / kZoom, fy = m_info.fy[cam] / kZoom;
		// As PSVR2Toolkit's GetCameraProjection: the right edge over the side-by-side frame's double width.
		const double l = -cx(cam) * n / fx, r = (2.0 * m_info.width - cx(cam)) * n / fx;
		const double t = m_info.cy[cam] * n / fy, b = -(m_info.height - m_info.cy[cam]) * n / fy;
		memset(out, 0, sizeof(*out));
		out->m[0][0] = (float)(2 * n / (r - l));
		out->m[0][2] = (float)((r + l) / (r - l));
		out->m[1][1] = (float)(2 * n / (t - b));
		out->m[1][2] = (float)((t + b) / (t - b));
		out->m[2][2] = -(f + n) / (f - n);
		out->m[2][3] = -2 * f * n / (f - n);
		out->m[3][2] = -1;
		return true;
	}

	bool
	SetFrameRate(int, int) override
	{
		return false;
	}

	bool
	SetCameraVideoSinkCallback(vr::ICameraVideoSinkCallback *sink) override
	{
		std::lock_guard<std::mutex> l(m_frameMutex);
		m_sink = sink;
		return true;
	}

	bool
	GetCameraCompatibilityMode(vr::ECameraCompatibilityMode *) override
	{
		return false;
	}

	bool
	SetCameraCompatibilityMode(vr::ECameraCompatibilityMode) override
	{
		return false;
	}

	bool
	GetCameraFrameBounds(vr::EVRTrackedCameraFrameType, uint32_t *l, uint32_t *t, uint32_t *w, uint32_t *h) override
	{
		if (l != nullptr) {
			*l = 0;
		}
		if (t != nullptr) {
			*t = 0;
		}
		if (w != nullptr) {
			*w = m_w;
		}
		if (h != nullptr) {
			*h = m_h;
		}
		return true;
	}

	bool
	GetCameraIntrinsics(uint32_t cam,
	                    vr::EVRTrackedCameraFrameType type,
	                    vr::HmdVector2_t *focal,
	                    vr::HmdVector2_t *center,
	                    vr::EVRDistortionFunctionType *dist,
	                    double coeffs[vr::k_unMaxDistortionFunctionParameters]) override
	{
		if (cam >= 2) {
			return false;
		}
		const bool distorted = type == vr::VRTrackedCameraFrameType_Distorted;
		const double zoom = distorted ? 1.0 : kZoom;
		// In the delivered frame's pixels (half of them at half size).
		if (focal != nullptr) {
			focal->v[0] = (float)(m_info.fx[cam] / zoom / m_scale);
			focal->v[1] = (float)(m_info.fy[cam] / zoom / m_scale);
		}
		if (center != nullptr) {
			center->v[0] = (float)(cx(cam) / m_scale);
			center->v[1] = (float)(m_info.cy[cam] / m_scale);
		}
		if (dist != nullptr) {
			*dist = distorted ? vr::VRDistortionFunctionType_Extended_FTheta : vr::VRDistortionFunctionType_None;
		}
		if (coeffs != nullptr) {
			memcpy(coeffs, m_info.ftheta[cam], sizeof(double) * vr::k_unMaxDistortionFunctionParameters);
		}
		return true;
	}

	//! Frames delivered so far (tests and diagnostics).
	uint64_t
	frames() const
	{
		return m_sequence.load();
	}

	/*!
	 * Decode one BC4 image pair into an NV12 frame (side by side, brightened, grey chroma); at half size each 2x2
	 * pixels are averaged (in the camera's own values, then brightened). Public for the tests.
	 */
	void
	decode(const uint8_t *pair, uint8_t *nv12) const
	{
		const uint32_t w = m_info.width, h = m_info.height;
		const uint32_t ow = w / m_scale; // one camera's width in the delivered frame
		const size_t row_bytes = (kStride / 4) * BCDEC_BC4_BLOCK_SIZE;
		const size_t image_bytes = (size_t)kStride * h / 2;
		uint8_t block[16];
		for (int cam = 0; cam < 2; cam++) {
			const uint8_t *src = pair + cam * image_bytes;
			for (uint32_t by = 0; by < h / 4; by++) {
				for (uint32_t bx = 0; bx < w / 4; bx++) {
					bcdec_bc4(src + by * row_bytes + bx * BCDEC_BC4_BLOCK_SIZE, block, 4, 0);
					if (m_scale == 1) {
						for (int y = 0; y < 4; y++) {
							uint8_t *dst = nv12 + (size_t)(by * 4 + y) * m_w + cam * ow + bx * 4;
							for (int x = 0; x < 4; x++) {
								dst[x] = m_lut[block[y * 4 + x]];
							}
						}
						continue;
					}
					for (int y = 0; y < 2; y++) {
						uint8_t *dst = nv12 + (size_t)(by * 2 + y) * m_w + cam * ow + bx * 2;
						for (int x = 0; x < 2; x++) {
							const uint8_t *q = block + y * 8 + x * 2; // the 2x2 at (2x, 2y) of the 4x4 block
							const unsigned sum = q[0] + q[1] + q[4] + q[5];
							dst[x] = m_lut[(sum + 2) / 4];
						}
					}
				}
			}
		}
		memset(nv12 + (size_t)m_w * m_h, 128, (size_t)m_w * m_h / 2);
	}

private:
	/*!
	 * The image centre SteamVR is given for camera @p cam: camera 1's mirrored (width - cx), as PSVR2Toolkit gives
	 * Sony's camera to SteamVR ("the center seems to be from the right for the right eye", hmd_device_camera.cpp);
	 * without it the right camera's picture was laid out the wrong way round (2026-11-19). The lens model fit keeps
	 * the calibration's own centre, as there.
	 */
	double
	cx(uint32_t cam) const
	{
		return cam == 1 ? m_info.width - m_info.cx[1] : m_info.cx[0];
	}

	//! Where a camera looks (its -Z axis in the head, degrees): pitch positive up, yaw positive left.
	static void
	view_angles(const Pose &p, float *pitch, float *yaw)
	{
		const vr::HmdMatrix34_t m = matrix(p);
		const double fx = -m.m[0][2], fy = -m.m[1][2], fz = -m.m[2][2];
		*pitch = (float)(std::asin(std::max(-1.0, std::min(1.0, fy))) * 180.0 / M_PI);
		*yaw = (float)(std::atan2(-fx, -fz) * 180.0 / M_PI);
	}

	static vr::HmdMatrix34_t
	matrix(const Pose &p)
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

	//! Start or stop to match what SteamVR wants (call with m_controlMutex held).
	bool
	update(const char *why)
	{
		if (m_shutDown) {
			return false; // after the headset's Deactivate: no new stream
		}
		if (m_streamRequested || m_readerActive) {
			if (m_running.load()) {
				return true;
			}
			log("ovrn: passthrough wanted (%s)\n", why);
			return start();
		}
		if (m_running.load()) {
			log("ovrn: passthrough not wanted any more (%s)\n", why);
			stop();
		}
		return true;
	}

	void
	watchLoop()
	{
		std::unique_lock<std::mutex> l(m_watchMutex);
		while (!m_watchQuit) {
			bool has = false;
			const bool ok = m_queue->QueueHasReader(m_queueHandle, &has) == steamvr::QueueResult::Ok;
			l.unlock();
			readerCheck(ok && has, now_ns());
			l.lock();
			m_watchCv.wait_for(l, std::chrono::milliseconds(250), [&] { return m_watchQuit; });
		}
	}

	bool
	start()
	{
		if (m_running.exchange(true)) {
			return true;
		}
		m_startNs = now_ns();
		m_worker = std::thread(&Camera::workerLoop, this);
		const bool ok = m_backend->cameraStart(
		    [this](const uint8_t *pair, size_t size, int64_t exposure_ns) { onImage(pair, size, exposure_ns); });
		log("ovrn: passthrough %s\n", ok ? "on (camera images streaming)" : "can't start: the backend has no cameras");
		if (!ok) {
			stop();
		}
		return ok;
	}

	void
	stop()
	{
		if (!m_running.exchange(false)) {
			return;
		}
		m_backend->cameraStop();
		{
			std::lock_guard<std::mutex> l(m_inMutex);
			m_cv.notify_all();
		}
		if (m_worker.joinable()) {
			m_worker.join();
		}
		log("ovrn: passthrough off (%llu frames)\n", (unsigned long long)m_sequence.load());
	}

	//! USB thread: keep the newest pair for the worker (a pair still waiting is replaced, never queued up).
	void
	onImage(const uint8_t *pair, size_t size, int64_t exposure_ns)
	{
		const size_t need = (size_t)kStride * m_info.height; // two BC4 images
		if (size < need) {
			return;
		}
		std::lock_guard<std::mutex> l(m_inMutex);
		m_in.assign(pair, pair + need);
		m_inExposureNs = exposure_ns;
		m_haveIn = true;
		m_cv.notify_one();
	}

	void
	workerLoop()
	{
		std::vector<uint8_t> pair;
		while (true) {
			int64_t exposure = 0;
			{
				std::unique_lock<std::mutex> l(m_inMutex);
				m_cv.wait(l, [&] { return m_haveIn || !m_running.load(); });
				if (!m_running.load()) {
					return;
				}
				pair.swap(m_in);
				exposure = m_inExposureNs;
				m_haveIn = false;
			}
			deliver(pair.data(), exposure);
		}
	}

	void
	deliver(const uint8_t *pair, int64_t exposure_ns)
	{
		const uint64_t seq = m_sequence.load();
		/*
		 * Decoded once per frame: into SteamVR's internal frame queue's block when offered, and that copied into the
		 * published path's buffer (decoding both took 2 x 2.15 ms per frame, 26% of a core at 60 Hz; once plus the
		 * 0.16 ms copy, 14%). The block is released only after the copy, so it's read while still ours.
		 */
		vr::PropertyContainerHandle_t block = vr::k_ulInvalidPropertyContainer;
		const uint8_t *decoded = nullptr;
		if (m_queue != nullptr) {
			void *buffer = nullptr;
			if (m_queue->AcquireWriteOnlyBlock(m_queueHandle, &block, &buffer) ==
			        steamvr::QueueResult::Ok &&
			    buffer != nullptr) {
				decode(pair, static_cast<uint8_t *>(buffer));
				decoded = static_cast<const uint8_t *>(buffer);
				steamvr::set_path(m_paths, block, "/server_time_ticks", (uint64_t)exposure_ns);
				steamvr::set_path(m_paths, block, "/frame_sequence", (uint64_t)seq);
				steamvr::set_path(m_paths, block, "/frame_size", (int32_t)m_frameBytes);
				steamvr::set_path(m_paths, block, "/frame_time_monotonic", (double)(exposure_ns * 1e-9));
				steamvr::set_path(m_paths, block, "/delivery_rate", kRate);
				steamvr::set_path(m_paths, block, "/elapsed_time", 1.0 / kRate);
			} else {
				block = vr::k_ulInvalidPropertyContainer;
			}
		}
		// The published path: SteamVR's buffers if it gave us some, else ours; then the sink callback.
		vr::ICameraVideoSinkCallback *sink = nullptr;
		{
			std::lock_guard<std::mutex> l(m_frameMutex);
			uint8_t *target = nullptr;
			uint32_t index = 0, count = 0;
			if (!m_steamBuffers.empty()) {
				count = (uint32_t)m_steamBuffers.size();
				index = (uint32_t)(seq % count);
				target = static_cast<uint8_t *>(m_steamBuffers[index]);
			} else {
				count = 2;
				index = (uint32_t)(seq % 2);
				target = m_own[index].data();
			}
			if (decoded != nullptr) {
				memcpy(target, decoded, m_frameBytes);
			} else {
				decode(pair, target);
			}
			m_frame = vr::CameraVideoStreamFrame_t{};
			m_frame.m_nStreamFormat = vr::CVS_FORMAT_NV12;
			m_frame.m_nWidth = m_w;
			m_frame.m_nHeight = m_h;
			m_frame.m_nImageDataSize = (uint32_t)m_frameBytes;
			m_frame.m_nFrameSequence = (uint32_t)seq;
			m_frame.m_nBufferIndex = index;
			m_frame.m_nBufferCount = count;
			m_frame.m_flFrameElapsedTime = (exposure_ns - m_startNs) * 1e-9;
			m_frame.m_flFrameDeliveryRate = kRate;
			m_frame.m_flFrameCaptureTime_ServerRelative = (exposure_ns - now_ns()) * 1e-9;
			m_frame.m_nFrameCaptureTicks_ServerAbsolute = (uint64_t)exposure_ns;
			m_frame.m_pImageData = (uint64_t)(uintptr_t)target;
			m_haveFrame = true;
			sink = m_sink;
		}
		if (block != vr::k_ulInvalidPropertyContainer) {
			m_queue->ReleaseWriteOnlyBlock(m_queueHandle, block);
		}
		m_sequence.fetch_add(1);
		if (sink != nullptr) {
			sink->OnCameraVideoSinkCallback();
		}
	}

	Backend *const m_backend;
	const CameraInfo m_info;
	uint32_t m_w = 0, m_h = 0;
	uint32_t m_scale = 1; //!< 2 at half size
	size_t m_frameBytes = 0;
	uint8_t m_lut[256];
	float m_gamma = 0.5f;

	steamvr::FrameQueue *m_queue = nullptr;
	steamvr::Paths *m_paths = nullptr;
	vr::PropertyContainerHandle_t m_queueHandle = vr::k_ulInvalidPropertyContainer;

	std::mutex m_controlMutex; //!< start / stop, from SteamVR's calls and the queue watcher
	bool m_streamRequested = false;
	bool m_readerActive = false;
	bool m_shutDown = false; //!< m_controlMutex: shutdown() ran
	int64_t m_lastReaderNs = 0;

	std::mutex m_watchMutex;
	std::condition_variable m_watchCv;
	bool m_watchQuit = false;
	std::thread m_watcher;

	std::atomic<bool> m_running{false};
	int64_t m_startNs = 0;
	std::thread m_worker;
	std::atomic<uint64_t> m_sequence{0};

	std::mutex m_inMutex; //!< the pair waiting for the worker
	std::condition_variable m_cv;
	std::vector<uint8_t> m_in;
	int64_t m_inExposureNs = 0;
	bool m_haveIn = false;

	std::mutex m_frameMutex; //!< the published frame
	std::vector<void *> m_steamBuffers;
	std::vector<uint8_t> m_own[2];
	vr::CameraVideoStreamFrame_t m_frame = {};
	bool m_haveFrame = false;
	vr::ICameraVideoSinkCallback *m_sink = nullptr;
};

} // namespace ovrn
