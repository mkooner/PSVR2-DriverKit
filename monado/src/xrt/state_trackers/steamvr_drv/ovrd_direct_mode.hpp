// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Driver direct mode (OVRD_DIRECT_MODE=1): SteamVR's compositor hands its finished eye images to the driver
 *         instead of driving the display itself. The images SteamVR renders into are shared Vulkan images we allocate
 *         (as ALVR's Linux OvrDirectModeComponent does). With an output (setOutput, Monado's compositor running
 *         inside vrserver) they are imported into it as swapchains and every SteamVR frame is submitted as projection
 *         layers with the head pose SteamVR rendered it with: Monado's compositor re-warps them to the latest pose,
 *         distorts them and shows them on the headset, paced by its own frame timing. Without an output, SteamVR is
 *         paced on a steady clock at the display rate and the frames are only logged.
 *
 * Log (OVRD_DIRECT_MODE_LOG, csv), host times are CLOCK_MONOTONIC ns:
 *   C,host_ns,pid,vk_format,width,height,samples,handle0,handle1,handle2,dmabuf_bytes  (CreateSwapTextureSet)
 *   S,host_ns,layer,predict_s,tx,ty,tz,qx,qy,qz,qw,tex_l,tex_r,depth_l,depth_r,umin,vmin,umax,vmax,err_mm,err_deg
 *     (err: SteamVR's pose mapped back to the device's space vs the device's own pose at the predicted time)
 *   P,host_ns,sync_handle,layers,frame_id,display_ns,submitted,target_err_ns,gpu_wait_us,gpu_sync   (Present)
 *     gpu_sync: 0 not used, 1 SteamVR's writes finished (waited gpu_wait_us), 2 timed out, 3 no semaphore found,
 *     4 import/wait error; target_err_ns: SteamVR's prediction target - Monado's display time (measurement only)
 *   Q,host_ns,frames_to_throttle,extra_frames_to_predict,woke_ns                       (PostPresent, after pacing)
 *   X,host_ns,handle  /  Y,host_ns,pid                                                 (destroy set / all of a pid)
 *   R,host_ns,frame_id,display_ns,render px,py,pz,qx,qy,qz,qw,pred px,py,pz,qx,qy,qz,qw
 *     (layer 0 at submit, device space: the pose the game rendered with, and the device's own prediction for the
 *     frame's display time made now; analysis/shown_pose_error.py compares both with T)
 *   T,host_ns,px,py,pz,qx,qy,qz,qw   (the device's pose at host_ns: where the head was; every 2 ms from a sampler
 *     thread, plus at Present and PostPresent)
 */

#pragma once

#include "openvr_driver.h"
#include "ovrd_log.hpp"
#include "os/os_time.h"

#include "xrt/xrt_compositor.h"
#include "xrt/xrt_device.h"
#include "math/m_api.h"
#include "ovrd_steamvr_sync.hpp"
#include "ovrd_vk_wait.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

class OvrdDirectMode : public vr::IVRDriverDirectModeComponent
{
public:
	//! Where the frames go: Monado's compositor (xcn NULL: log only).
	struct Output
	{
		struct xrt_compositor_native *xcn = NULL;
		struct xrt_device *head = NULL;
		struct xrt_fov fov[2] = {};
		struct xrt_pose eye_in_head[2] = {XRT_POSE_IDENTITY, XRT_POSE_IDENTITY};
	};

	//! Maps a pose SteamVR reports (its world) back to the head device's own tracking space.
	std::function<struct xrt_pose(const struct xrt_pose &)> world_to_device;
	/*!
	 * Called in PostPresent once Monado's next frame is known, before the vsync event, with its display time: the
	 * driver submits the headset pose for it there, so SteamVR has it when it wakes the game (Valve's direct-mode
	 * sequence: "block until running start, sample and submit your tracked device poses, then call VsyncEvent").
	 */
	std::function<void(int64_t display_ns)> on_frame_start;
	//! The head device, for the pose check in the log (also without an output).
	struct xrt_device *check_head = NULL;

	/*!
	 * Wait for SteamVR's writes into the submitted images before handing them to Monado's compositor (default on,
	 * OVRD_DIRECT_MODE_GPU_SYNC): on NVIDIA nothing else orders Monado's reads after SteamVR's / the app's GPU
	 * writes, so a frame could be shown partly drawn, mixed with the image's previous content (ALVR PR #3425). The
	 * signal is SteamVR's per-texture timeline semaphore (ovrd_steamvr_sync.hpp).
	 */
	bool gpu_sync = true;

	/*!
	 * Diagnostics (OVRD_DIRECT_MODE_CAPTURE=<s>): once, <s> seconds after the first frame with two or more layers,
	 * the finished images of that frame's layers (left eye) are written next to the log as
	 * <log>-layer<N>.rgba ("w h vkformat" line, then raw RGBA bytes; analysis/layer_capture.py turns them into PNGs).
	 * 0: off.
	 */
	double capture_after_s = 0.0;

	//! Before start(): the compositor to show the frames with.
	void
	setOutput(const Output &out)
	{
		m_out = out;
	}

	bool
	hasOutput() const
	{
		return m_out.xcn != NULL;
	}

	/*!
	 * Monado's display time of the frame SteamVR renders next (0 until the first frame): the frame a game starts
	 * now is shown at it (99.9% of 5705 frames, 2026-11-18 10:01 session). For giving the game Monado's own head
	 * prediction for that moment, as on the OpenXR path.
	 */
	int64_t
	displayTarget() const
	{
		return m_displayAt.load();
	}

	//! The eye-to-head poses changed (IPD dial).
	void
	setEyes(const struct xrt_pose eye_in_head[2])
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_out.eye_in_head[0] = eye_in_head[0];
		m_out.eye_in_head[1] = eye_in_head[1];
	}

	~OvrdDirectMode()
	{
		stop();
	}

	//! Start the vsync clock; @p log_path may be NULL or empty (summaries only).
	void
	start(float display_hz, const char *log_path)
	{
		m_periodNs = (int64_t)llround(1e9 / (display_hz > 1.0f ? display_hz : 90.0f));
		if (log_path != NULL && log_path[0] != '\0') {
			m_log = fopen(log_path, "w");
			m_logPath = log_path;
			if (m_logPath.size() > 4 && m_logPath.compare(m_logPath.size() - 4, 4, ".csv") == 0) {
				m_logPath.resize(m_logPath.size() - 4);
			}
			if (m_log != NULL) {
				fprintf(m_log, "# direct mode, display period %lld ns\n", (long long)m_periodNs);
				ovrd_log("Direct mode: logging frames to %s\n", log_path);
			} else {
				ovrd_log("Direct mode: can't write %s\n", log_path);
			}
		}
		m_gridNs = os_monotonic_get_ns();
		m_summaryNs = m_gridNs;
		if (m_out.xcn != NULL && gpu_sync) {
			std::string why;
			if (m_vk.init(0, &why)) {
				ovrd_log("Direct mode: waiting for SteamVR's writes on %s (timeline semaphores)\n",
				         m_vk.gpuName().c_str());
			} else {
				ovrd_log("Direct mode: can't wait for SteamVR's writes (%s), frames unsynchronized\n", why.c_str());
			}
		}
		m_running = true;
		m_vsyncThread = std::thread(&OvrdDirectMode::vsyncLoop, this);
		if (m_log != NULL && check_head != NULL) {
			// The reference for "where the head was" at any display time: the device's own pose every 2 ms.
			m_truthThread = std::thread([this]() {
				while (m_running) {
					os_nanosleep(2000000);
					logTruth();
				}
			});
		}
		if (m_out.xcn == NULL) {
			ovrd_log("Direct mode: on, frames logged only, vsync every %.3f ms\n", m_periodNs / 1e6);
		} else {
			ovrd_log("Direct mode: on, frames shown by Monado's compositor\n");
		}
	}

	void
	stop()
	{
		if (m_running.exchange(false) && m_vsyncThread.joinable()) {
			m_vsyncThread.join();
		}
		if (m_truthThread.joinable()) {
			m_truthThread.join();
		}
		std::lock_guard<std::mutex> lock(m_mutex);
		for (auto &set : m_sets) {
			for (int i = 0; i < 3; i++) {
				if (set.second.fds[i] >= 0) {
					close(set.second.fds[i]);
				}
			}
			xrt_swapchain_reference(&set.second.xsc, NULL);
		}
		m_sets.clear();
		m_handleToSet.clear();
		m_vk.destroy();
		m_sync.forgetAll();
		if (m_log != NULL) {
			fclose(m_log);
			m_log = NULL;
		}
	}

	// IVRDriverDirectModeComponent

	void
	CreateSwapTextureSet(uint32_t unPid, const SwapTextureSetDesc_t *desc, SwapTextureSet_t *out) override
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		*out = {};
		Set set = {};
		set.pid = unPid;
		set.desc = *desc;
		// As ALVR: sampled + transfer source + input attachment, mutable format (sRGB views); no compute access,
		// sRGB formats can't be storage images.
		const uint32_t usage = 0x1 /* TRANSFER_SRC */ | 0x4 /* SAMPLED */ | 0x80 /* INPUT_ATTACHMENT */;
		const uint32_t create = 0x8 /* MUTABLE_FORMAT */;
		off_t dmabuf_bytes = -1;
		bool ok = true;
		for (int i = 0; i < 3 && ok; i++) {
			vr::SharedTextureHandle_t h = 0;
			ok = vr::VRIPCResourceManager()->NewSharedVulkanImage(desc->nFormat, desc->nWidth, desc->nHeight,
			                                                      true, false, false, 1, 1, create, usage, &h);
			if (!ok) {
				ovrd_log("Direct mode: NewSharedVulkanImage failed (format %u %ux%u)\n", desc->nFormat,
				         desc->nWidth, desc->nHeight);
				break;
			}
			set.handles[i] = h;
			uint64_t ipc = 0;
			int fd = -1;
			if (!vr::VRIPCResourceManager()->RefResource(h, &ipc) ||
			    !vr::VRIPCResourceManager()->ReceiveSharedFd(ipc, &fd)) {
				ovrd_log("Direct mode: no fd for shared image %d\n", i);
				ok = false;
				break;
			}
			set.fds[i] = fd;
			if (i == 0) {
				dmabuf_bytes = lseek(fd, 0, SEEK_END);
				lseek(fd, 0, SEEK_SET);
			}
		}
		if (!ok) {
			for (int i = 0; i < 3; i++) {
				if (set.handles[i] != 0) {
					vr::VRIPCResourceManager()->UnrefResource(set.handles[i]);
				}
				if (set.fds[i] >= 0) {
					close(set.fds[i]);
				}
			}
		} else {
			for (int i = 0; i < 3; i++) {
				out->rSharedTextureHandles[i] = set.handles[i];
				m_handleToSet[set.handles[i]] = set.handles[0];
			}
			importLocked(set);
			m_sets[set.handles[0]] = set;
		}
		ovrd_log("Direct mode: CreateSwapTextureSet pid %u format %u %ux%u x%u samples: %s, dma-buf %lld bytes\n",
		         unPid, desc->nFormat, desc->nWidth, desc->nHeight, desc->nSampleCount, ok ? "ok" : "FAILED",
		         (long long)dmabuf_bytes);
		if (m_log != NULL) {
			fprintf(m_log, "C,%lld,%u,%u,%u,%u,%u,%llu,%llu,%llu,%lld\n", (long long)os_monotonic_get_ns(),
			        unPid, desc->nFormat, desc->nWidth, desc->nHeight, desc->nSampleCount,
			        (unsigned long long)set.handles[0], (unsigned long long)set.handles[1],
			        (unsigned long long)set.handles[2], ok ? (long long)dmabuf_bytes : -1LL);
		}
	}

	void
	DestroySwapTextureSet(vr::SharedTextureHandle_t handle) override
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		auto it = m_handleToSet.find(handle);
		if (it != m_handleToSet.end()) {
			destroySetLocked(it->second);
		}
		if (m_log != NULL) {
			fprintf(m_log, "X,%lld,%llu\n", (long long)os_monotonic_get_ns(), (unsigned long long)handle);
		}
	}

	void
	DestroyAllSwapTextureSets(uint32_t unPid) override
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		std::vector<vr::SharedTextureHandle_t> ids;
		for (auto &set : m_sets) {
			if (set.second.pid == unPid) {
				ids.push_back(set.first);
			}
		}
		for (auto id : ids) {
			destroySetLocked(id);
		}
		if (m_log != NULL) {
			fprintf(m_log, "Y,%lld,%u\n", (long long)os_monotonic_get_ns(), unPid);
		}
	}

	void
	GetNextSwapTextureSetIndex(vr::SharedTextureHandle_t handles[2], uint32_t (*indices)[2]) override
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		for (int eye = 0; eye < 2; eye++) {
			auto it = m_handleToSet.find(handles[eye]);
			if (it == m_handleToSet.end()) {
				continue;
			}
			Set &set = m_sets[it->second];
			set.next = (set.next + 1) % 3;
			if (indices != NULL) {
				(*indices)[eye] = set.next;
			}
		}
	}

	void
	SubmitLayer(const SubmitLayerPerEye_t (&perEye)[2]) override
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		const int64_t now = os_monotonic_get_ns();
		const vr::HmdMatrix34_t &m = perEye[0].mHmdPose;
		float q[4];
		matrix_to_quat(m, q);
		struct xrt_pose world = {{q[0], q[1], q[2], q[3]}, {m.m[0][3], m.m[1][3], m.m[2][3]}};
		struct xrt_pose device = world_to_device ? world_to_device(world) : world;
		if (m_layers < kMaxLayers) {
			Layer &L = m_layerBuf[m_layers];
			L.head = device;
			for (int e = 0; e < 2; e++) {
				L.tex[e] = perEye[e].hTexture;
				L.bounds[e] = perEye[e].bounds;
			}
		}
		// Check the space mapping: the device's own pose at the time SteamVR predicted to.
		float err_mm = NAN, err_deg = NAN;
		if (check_head != NULL && m_layers == 0) {
			struct xrt_space_relation rel = {};
			const int64_t at = now + (int64_t)(perEye[0].flHmdPosePredictionTimeInSecondsFromNow * 1e9);
			if (xrt_device_get_tracked_pose(check_head, XRT_INPUT_GENERIC_HEAD_POSE, at, &rel) == XRT_SUCCESS) {
				struct xrt_vec3 d = {device.position.x - rel.pose.position.x,
				                     device.position.y - rel.pose.position.y,
				                     device.position.z - rel.pose.position.z};
				err_mm = 1000.0f * sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
				float dot = fabsf(device.orientation.x * rel.pose.orientation.x +
				                  device.orientation.y * rel.pose.orientation.y +
				                  device.orientation.z * rel.pose.orientation.z +
				                  device.orientation.w * rel.pose.orientation.w);
				err_deg = 2.0f * acosf(fminf(1.0f, dot)) * 57.29578f;
			}
		}
		if (m_log != NULL) {
			const vr::VRTextureBounds_t &b = perEye[0].bounds;
			fprintf(m_log,
			        "S,%lld,%d,%.6f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%llu,%llu,%llu,%llu,%.4f,%.4f,%.4f,%.4f,%.2f,%.3f\n",
			        (long long)now, m_layers, perEye[0].flHmdPosePredictionTimeInSecondsFromNow, m.m[0][3],
			        m.m[1][3], m.m[2][3], q[0], q[1], q[2], q[3], (unsigned long long)perEye[0].hTexture,
			        (unsigned long long)perEye[1].hTexture, (unsigned long long)perEye[0].hDepthTexture,
			        (unsigned long long)perEye[1].hDepthTexture, b.uMin, b.vMin, b.uMax, b.vMax, err_mm, err_deg);
		}
		if (m_layers == 0 && !std::isnan(err_mm)) {
			m_errMm.push_back(err_mm);
			m_errDeg.push_back(err_deg);
		}
		if (m_layers == 0) {
			m_predictS.push_back(perEye[0].flHmdPosePredictionTimeInSecondsFromNow);
			m_lastSubmitNs = now;
			m_lastPredictS = perEye[0].flHmdPosePredictionTimeInSecondsFromNow;
		}
		m_layers++;
	}

	void
	Present(vr::SharedTextureHandle_t syncTexture) override
	{
		logTruth();
		// Wait until SteamVR's (and the app's) GPU writes into this frame's images are done, outside the lock.
		int sync_state = 0;
		int64_t sync_wait_ns = 0;
		if (m_out.xcn != NULL && gpu_sync && m_vk.ready()) {
			waitForWrites(&sync_state, &sync_wait_ns);
			maybeCapture();
		}
		// The first frame (and one after a failed wait) waits for its Monado frame here, outside the lock.
		if (m_out.xcn != NULL && !m_waited) {
			waitFrame();
		}
		std::lock_guard<std::mutex> lock(m_mutex);
		const int64_t now = os_monotonic_get_ns();
		bool submitted = false;
		int64_t target_err = 0;
		if (sync_state != 0) {
			m_syncStates[sync_state]++;
			if (sync_state == 1) {
				m_syncWaitUs.push_back(sync_wait_ns / 1e3);
			}
		}
		if (m_out.xcn != NULL && m_waited) {
			// SteamVR's prediction target for this frame vs the time Monado shows it (measurement only).
			if (m_layers > 0 && m_lastSubmitNs != 0) {
				const int64_t target = m_lastSubmitNs + (int64_t)(m_lastPredictS * 1e9);
				target_err = target - m_displayNs;
				m_targetErrMs.push_back(target_err / 1e6);
			}
			submitted = submitLocked();
			m_waited = false;
			m_submitted += submitted ? 1 : 0;
		}
		if (m_log != NULL) {
			fprintf(m_log, "P,%lld,%llu,%d,%lld,%lld,%d,%lld,%.1f,%d\n", (long long)now,
			        (unsigned long long)syncTexture, m_layers, (long long)m_frameId, (long long)m_displayNs,
			        submitted ? 1 : 0, (long long)target_err, sync_wait_ns / 1e3, sync_state);
		}
		if (m_lastPresentNs != 0) {
			m_presentGapsMs.push_back((now - m_lastPresentNs) / 1e6);
		}
		m_lastPresentNs = now;
		m_lastPresentAt = now;
		m_frames++;
		m_layerSum += m_layers;
		m_layers = 0;
		summaryLocked(now);
	}

	void
	PostPresent(const Throttling_t *throttling) override
	{
		int64_t woke;
		if (m_out.xcn != NULL) {
			// Monado's compositor paces SteamVR: wait for the next frame's wake-up, then tell SteamVR a vsync
			// happened so it starts rendering the frame for it.
			waitFrame();
			woke = os_monotonic_get_ns();
			if (on_frame_start && m_waited) {
				on_frame_start(m_displayAt.load());
			}
			vr::VRServerDriverHost()->VsyncEvent(0.0);
		} else {
			// Hold SteamVR's compositor until the next tick of the display clock, as ALVR does: it then starts
			// its next frame one display period after the last.
			woke = waitNextTick();
		}
		logTruth();
		if (m_log != NULL) {
			std::lock_guard<std::mutex> lock(m_mutex);
			fprintf(m_log, "Q,%lld,%u,%u,%lld\n", (long long)os_monotonic_get_ns(),
			        throttling ? throttling->nFramesToThrottle : 0u,
			        throttling ? throttling->nAdditionalFramesToPredict : 0u, (long long)woke);
		}
	}

	void
	GetFrameTiming(vr::DriverDirectMode_FrameTiming *timing) override
	{
		if (timing == NULL || timing->m_nSize < sizeof(vr::DriverDirectMode_FrameTiming)) {
			return;
		}
		timing->m_nNumFramePresents = 1;
		timing->m_nNumMisPresented = 0;
		timing->m_nNumDroppedFrames = 0;
		timing->m_nReprojectionFlags = 0; // overlaps the throttle mask, must not be echoed back
	}

private:
	struct Set
	{
		uint32_t pid = 0;
		SwapTextureSetDesc_t desc = {};
		vr::SharedTextureHandle_t handles[3] = {0, 0, 0};
		int fds[3] = {-1, -1, -1};
		uint32_t next = 0;
		struct xrt_swapchain *xsc = NULL; //!< imported into Monado's compositor
	};

	static constexpr int kMaxLayers = 8;
	struct Layer
	{
		struct xrt_pose head;          //!< SteamVR's render pose, in the device's space
		vr::SharedTextureHandle_t tex[2];
		vr::VRTextureBounds_t bounds[2];
	};

	//! Import a new set's three images into Monado's compositor as one swapchain.
	void
	importLocked(Set &set)
	{
		if (m_out.xcn == NULL) {
			return;
		}
		struct xrt_swapchain_create_info info = {};
		info.bits = (enum xrt_swapchain_usage_bits)(XRT_SWAPCHAIN_USAGE_COLOR | XRT_SWAPCHAIN_USAGE_SAMPLED |
		                                            XRT_SWAPCHAIN_USAGE_TRANSFER_SRC |
		                                            XRT_SWAPCHAIN_USAGE_INPUT_ATTACHMENT |
		                                            XRT_SWAPCHAIN_USAGE_MUTABLE_FORMAT);
		info.format = set.desc.nFormat;
		info.sample_count = 1;
		info.width = set.desc.nWidth;
		info.height = set.desc.nHeight;
		info.face_count = 1;
		info.array_size = 1;
		info.mip_count = 1;
		// The sRGB / UNORM pair (Monado creates sRGB images as UNORM with sRGB views).
		uint32_t pair = 0;
		switch (set.desc.nFormat) {
		case 43: pair = 37; break; // R8G8B8A8_SRGB / UNORM
		case 37: pair = 43; break;
		case 50: pair = 44; break; // B8G8R8A8_SRGB / UNORM
		case 44: pair = 50; break;
		default: break;
		}
		info.formats[info.format_count++] = set.desc.nFormat;
		if (pair != 0) {
			info.formats[info.format_count++] = pair;
		}
		struct xrt_image_native natives[3] = {};
		for (int i = 0; i < 3; i++) {
			natives[i].handle = dup(set.fds[i]); // the compositor takes ownership
			natives[i].size = 0;
			natives[i].use_dedicated_allocation = true;
		}
		xrt_result_t xret = xrt_comp_import_swapchain(&m_out.xcn->base, &info, natives, 3, &set.xsc);
		if (xret != XRT_SUCCESS) {
			for (int i = 0; i < 3; i++) {
				close(natives[i].handle);
			}
			set.xsc = NULL;
		}
		ovrd_log("Direct mode: import into Monado's compositor (format %u %ux%u): %s (%d)\n", set.desc.nFormat,
		         set.desc.nWidth, set.desc.nHeight, xret == XRT_SUCCESS ? "ok" : "FAILED", (int)xret);
	}

	//! T row: the head device's own pose now (the reference for where the head really was).
	void
	logTruth()
	{
		if (m_log == NULL || check_head == NULL) {
			return;
		}
		const int64_t now = os_monotonic_get_ns();
		struct xrt_space_relation rel = {};
		if (xrt_device_get_tracked_pose(check_head, XRT_INPUT_GENERIC_HEAD_POSE, now, &rel) != XRT_SUCCESS) {
			return;
		}
		std::lock_guard<std::mutex> lock(m_mutex);
		if (m_log != NULL) {
			const struct xrt_pose &p = rel.pose;
			fprintf(m_log, "T,%lld,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f\n", (long long)now, p.position.x,
			        p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z,
			        p.orientation.w);
		}
	}

	/*!
	 * Waits until every image of the frame being presented has been written (SteamVR's per-texture timeline
	 * semaphore reaches the value of its last submitted write). @p state: 1 all done, 2 a wait timed out, 3 an image
	 * had no semaphore (yet), 4 import/wait error. Bounded by one frame period in total so a stuck signal can't stall
	 * SteamVR.
	 */
	void
	waitForWrites(int *state, int64_t *waited_ns)
	{
		std::vector<vr::SharedTextureHandle_t> textures;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			for (int l = 0; l < std::min(m_layers, kMaxLayers); l++) {
				for (int e = 0; e < 2; e++) {
					vr::SharedTextureHandle_t t = m_layerBuf[l].tex[e];
					if (t != 0 && std::find(textures.begin(), textures.end(), t) == textures.end()) {
						textures.push_back(t);
					}
				}
			}
		}
		const int64_t t0 = os_monotonic_get_ns();
		int result = 1;
		for (vr::SharedTextureHandle_t t : textures) {
			uint64_t sem = m_sync.semaphoreOf(t);
			uint64_t value = 0;
			if (sem == 0 || !m_sync.value(sem, &value)) {
				result = std::max(result, 3);
				continue;
			}
			// One frame period in total for all images, so a stuck write costs SteamVR at most one frame.
			const int64_t left = std::max<int64_t>(0, t0 + m_periodNs - (int64_t)os_monotonic_get_ns());
			OvrdVkWait::Result r =
			    m_vk.wait(sem, value, (uint64_t)left, [this, sem]() { return m_sync.dupFd(sem); });
			if (r == OvrdVkWait::kTimeout) {
				result = std::max(result, 2);
			} else if (r == OvrdVkWait::kError) {
				result = 4;
			}
		}
		*state = textures.empty() ? 0 : result;
		*waited_ns = os_monotonic_get_ns() - t0;
	}

	//! The layer capture (capture_after_s), on the frame's finished images.
	void
	maybeCapture()
	{
		if (capture_after_s <= 0.0 || m_captured || m_logPath.empty()) {
			return;
		}
		const int64_t now = os_monotonic_get_ns();
		struct Item
		{
			int layer;
			int fd;
			uint32_t w, h, format;
		};
		std::vector<Item> items;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			if (m_layers < 2) {
				return;
			}
			if (m_multiLayerSinceNs == 0) {
				m_multiLayerSinceNs = now;
			}
			if (now - m_multiLayerSinceNs < (int64_t)(capture_after_s * 1e9)) {
				return;
			}
			m_captured = true;
			for (int l = 0; l < std::min(m_layers, kMaxLayers); l++) {
				auto it = m_handleToSet.find(m_layerBuf[l].tex[0]);
				if (it == m_handleToSet.end()) {
					continue;
				}
				Set &set = m_sets[it->second];
				for (int i = 0; i < 3; i++) {
					if (set.handles[i] == m_layerBuf[l].tex[0] && set.fds[i] >= 0) {
						items.push_back({l, dup(set.fds[i]), set.desc.nWidth, set.desc.nHeight, set.desc.nFormat});
					}
				}
			}
		}
		for (const Item &it : items) {
			std::string path = m_logPath + "-layer" + std::to_string(it.layer) + ".rgba";
			std::string why;
			bool ok = m_vk.capture(it.fd, it.w, it.h, it.format, path.c_str(), &why);
			ovrd_log("Direct mode: captured layer %d (%ux%u format %u) to %s: %s\n", it.layer, it.w, it.h,
			         it.format, path.c_str(), ok ? "ok" : why.c_str());
		}
	}

	//! Monado's next frame: blocks until its wake-up time.
	void
	waitFrame()
	{
		int64_t id = 0, display = 0, period = 0;
		xrt_result_t xret = xrt_comp_wait_frame(&m_out.xcn->base, &id, &display, &period);
		std::lock_guard<std::mutex> lock(m_mutex);
		if (xret != XRT_SUCCESS) {
			m_waited = false;
			m_waitFails++;
			return;
		}
		m_frameId = id;
		m_displayNs = display;
		m_displayAt = display;
		m_monadoPeriodNs = period;
		m_waited = true;
	}

	//! Hand the frame's layers to Monado's compositor (the frame waited for last).
	bool
	submitLocked()
	{
		struct xrt_compositor *xc = &m_out.xcn->base;
		if (xrt_comp_begin_frame(xc, m_frameId) != XRT_SUCCESS) {
			return false;
		}
		struct xrt_layer_frame_data fd = {};
		fd.frame_id = m_frameId;
		fd.display_time_ns = m_displayNs;
		fd.env_blend_mode = XRT_BLEND_MODE_OPAQUE;
		xrt_comp_layer_begin(xc, &fd);
		int shown = 0;
		for (int l = 0; l < std::min(m_layers, kMaxLayers); l++) {
			const Layer &L = m_layerBuf[l];
			struct xrt_swapchain *xsc[XRT_MAX_VIEWS] = {};
			struct xrt_layer_data d = {};
			d.type = XRT_LAYER_PROJECTION;
			d.name = XRT_INPUT_GENERIC_HEAD_POSE;
			d.timestamp = m_displayNs;
			// SteamVR's first layer is the scene, later ones (dashboard, overlays) are blended over it.
			d.flags = l > 0 ? XRT_LAYER_COMPOSITION_BLEND_TEXTURE_SOURCE_ALPHA_BIT
			                : (enum xrt_layer_composition_flags)0;
			d.color_scale = {1.0f, 1.0f, 1.0f, 1.0f};
			d.color_bias = {0.0f, 0.0f, 0.0f, 0.0f};
			d.view_count = 2;
			bool ok = true;
			for (int e = 0; e < 2 && ok; e++) {
				auto it = m_handleToSet.find(L.tex[e]);
				if (it == m_handleToSet.end() || m_sets[it->second].xsc == NULL) {
					ok = false;
					break;
				}
				Set &set = m_sets[it->second];
				uint32_t index = 0;
				for (uint32_t i = 0; i < 3; i++) {
					if (set.handles[i] == L.tex[e]) {
						index = i;
					}
				}
				xsc[e] = set.xsc;
				vr::VRTextureBounds_t b = L.bounds[e];
				if (b.vMin > b.vMax) {
					std::swap(b.vMin, b.vMax);
					d.flip_y = true;
				}
				struct xrt_layer_projection_view_data &v = d.proj.v[e];
				v.sub.image_index = index;
				v.sub.array_index = 0;
				v.sub.norm_rect = {b.uMin, b.vMin, b.uMax - b.uMin, b.vMax - b.vMin};
				v.sub.rect.offset = {(int)lroundf(b.uMin * set.desc.nWidth), (int)lroundf(b.vMin * set.desc.nHeight)};
				v.sub.rect.extent = {(int)lroundf((b.uMax - b.uMin) * set.desc.nWidth),
				                     (int)lroundf((b.vMax - b.vMin) * set.desc.nHeight)};
				v.fov = m_out.fov[e];
				math_pose_transform(&L.head, &m_out.eye_in_head[e], &v.pose);
			}
			if (!ok) {
				continue;
			}
			if (l == 0 && m_log != NULL) {
				struct xrt_space_relation pred = {};
				xrt_device_get_tracked_pose(m_out.head, XRT_INPUT_GENERIC_HEAD_POSE, m_displayNs, &pred);
				const struct xrt_pose &r = L.head;
				const struct xrt_pose &q = pred.pose;
				fprintf(m_log,
				        "R,%lld,%lld,%lld,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f\n",
				        (long long)os_monotonic_get_ns(), (long long)m_frameId, (long long)m_displayNs,
				        r.position.x, r.position.y, r.position.z, r.orientation.x, r.orientation.y,
				        r.orientation.z, r.orientation.w, q.position.x, q.position.y, q.position.z,
				        q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w);
			}
			if (xrt_comp_layer_projection(xc, m_out.head, xsc, &d) == XRT_SUCCESS) {
				shown++;
			}
		}
		return xrt_comp_layer_commit(xc, XRT_GRAPHICS_SYNC_HANDLE_INVALID) == XRT_SUCCESS && shown > 0;
	}

	void
	destroySetLocked(vr::SharedTextureHandle_t id)
	{
		auto it = m_sets.find(id);
		if (it == m_sets.end()) {
			return;
		}
		for (int i = 0; i < 3; i++) {
			if (it->second.handles[i] != 0) {
				vr::VRIPCResourceManager()->UnrefResource(it->second.handles[i]);
				m_handleToSet.erase(it->second.handles[i]);
			}
			if (it->second.fds[i] >= 0) {
				close(it->second.fds[i]);
			}
		}
		xrt_swapchain_reference(&it->second.xsc, NULL);
		std::vector<uint64_t> textures(it->second.handles, it->second.handles + 3);
		m_vk.forget(m_sync.forget(textures));
		m_sets.erase(it);
	}

	//! Sleep until the next display-clock tick after now; returns the tick time.
	int64_t
	waitNextTick()
	{
		const int64_t now = os_monotonic_get_ns();
		const int64_t grid = m_gridNs;
		const int64_t tick = grid + ((now - grid) / m_periodNs + 1) * m_periodNs;
		os_nanosleep(tick - now);
		return tick;
	}

	/*!
	 * Tells SteamVR about each vsync of the steady display clock (Prop_DriverDirectModeSendsVsyncEvents_Bool).
	 * With Monado's compositor, PostPresent sends them paced by Monado's frames; this clock then only runs while
	 * no frame came for 100 ms (before SteamVR's first frame, or when it stalls), so SteamVR never waits forever
	 * for a vsync to start a frame.
	 */
	void
	vsyncLoop()
	{
		while (m_running) {
			waitNextTick();
			if (m_out.xcn != NULL && os_monotonic_get_ns() - m_lastPresentAt.load() < 100000000LL) {
				continue;
			}
			vr::VRServerDriverHost()->VsyncEvent(0.0);
			m_clockVsyncs++;
		}
	}

	void
	summaryLocked(int64_t now)
	{
		if (now - m_summaryNs < 10LL * 1000000000LL) {
			return;
		}
		const double secs = (now - m_summaryNs) / 1e9;
		auto pct = [](std::vector<double> v, double p) -> double {
			if (v.empty()) {
				return NAN;
			}
			std::sort(v.begin(), v.end());
			return v[std::min(v.size() - 1, (size_t)(p * (v.size() - 1) + 0.5))];
		};
		std::vector<double> pred_ms;
		for (float s : m_predictS) {
			pred_ms.push_back(s * 1e3);
		}
		ovrd_log("Direct mode (%.0f s): %d frames (%.1f/s), %.2f layers/frame; Present interval median %.2f p99 %.2f "
		         "ms; pose predicted median %.2f (%.2f-%.2f) ms ahead; %zu image sets; shown %d, wait failures %d; "
		         "pose vs device median %.1f mm / %.2f deg, p99 %.1f mm / %.2f deg; %d clock vsyncs; SteamVR's "
		         "target - display median %+.2f (%+.2f..%+.2f) ms; GPU sync: %d waited (median %.0f us, p99 %.0f "
		         "us), %d timed out, %d without semaphore, %d errors\n",
		         secs, m_frames, m_frames / secs, m_frames ? (double)m_layerSum / m_frames : 0.0,
		         pct(m_presentGapsMs, 0.5), pct(m_presentGapsMs, 0.99), pct(pred_ms, 0.5), pct(pred_ms, 0.01),
		         pct(pred_ms, 0.99), m_sets.size(), m_submitted, m_waitFails, pct(m_errMm, 0.5),
		         pct(m_errDeg, 0.5), pct(m_errMm, 0.99), pct(m_errDeg, 0.99), m_clockVsyncs.exchange(0),
		         pct(m_targetErrMs, 0.5), pct(m_targetErrMs, 0.01), pct(m_targetErrMs, 0.99), m_syncStates[1],
		         pct(m_syncWaitUs, 0.5), pct(m_syncWaitUs, 0.99), m_syncStates[2], m_syncStates[3],
		         m_syncStates[4]);
		m_targetErrMs.clear();
		m_syncWaitUs.clear();
		for (int &c : m_syncStates) {
			c = 0;
		}
		m_submitted = 0;
		m_waitFails = 0;
		m_errMm.clear();
		m_errDeg.clear();
		if (m_log != NULL) {
			fflush(m_log);
		}
		m_summaryNs = now;
		m_frames = 0;
		m_layerSum = 0;
		m_presentGapsMs.clear();
		m_predictS.clear();
	}

	static void
	matrix_to_quat(const vr::HmdMatrix34_t &m, float q[4])
	{
		const float tr = m.m[0][0] + m.m[1][1] + m.m[2][2];
		float x, y, z, w;
		if (tr > 0.0f) {
			float s = sqrtf(tr + 1.0f) * 2.0f;
			w = 0.25f * s;
			x = (m.m[2][1] - m.m[1][2]) / s;
			y = (m.m[0][2] - m.m[2][0]) / s;
			z = (m.m[1][0] - m.m[0][1]) / s;
		} else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
			float s = sqrtf(1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2.0f;
			w = (m.m[2][1] - m.m[1][2]) / s;
			x = 0.25f * s;
			y = (m.m[0][1] + m.m[1][0]) / s;
			z = (m.m[0][2] + m.m[2][0]) / s;
		} else if (m.m[1][1] > m.m[2][2]) {
			float s = sqrtf(1.0f + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2.0f;
			w = (m.m[0][2] - m.m[2][0]) / s;
			x = (m.m[0][1] + m.m[1][0]) / s;
			y = 0.25f * s;
			z = (m.m[1][2] + m.m[2][1]) / s;
		} else {
			float s = sqrtf(1.0f + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2.0f;
			w = (m.m[1][0] - m.m[0][1]) / s;
			x = (m.m[0][2] + m.m[2][0]) / s;
			y = (m.m[1][2] + m.m[2][1]) / s;
			z = 0.25f * s;
		}
		q[0] = x;
		q[1] = y;
		q[2] = z;
		q[3] = w;
	}

	std::mutex m_mutex;
	std::map<vr::SharedTextureHandle_t, Set> m_sets;                           //!< by the set's first handle
	std::map<vr::SharedTextureHandle_t, vr::SharedTextureHandle_t> m_handleToSet; //!< any handle -> first
	FILE *m_log = NULL;
	std::atomic<bool> m_running{false};
	std::thread m_vsyncThread;
	std::thread m_truthThread;
	int64_t m_periodNs = 11111111;
	int64_t m_gridNs = 0;
	int m_layers = 0;

	// 10 s summary
	int64_t m_summaryNs = 0;
	int64_t m_lastPresentNs = 0;
	int m_frames = 0;
	int m_layerSum = 0;
	std::vector<double> m_presentGapsMs;
	std::vector<float> m_predictS;
	std::vector<double> m_errMm, m_errDeg;
	int m_submitted = 0;
	int m_waitFails = 0;
	std::atomic<int64_t> m_lastPresentAt{0};
	std::atomic<int64_t> m_displayAt{0}; //!< displayTarget()
	std::string m_logPath;                //!< the log path without .csv (for captures)
	bool m_captured = false;
	int64_t m_multiLayerSinceNs = 0;
	ovrd_steamvr_sync::Sync m_sync;
	OvrdVkWait m_vk;
	int m_syncStates[5] = {0, 0, 0, 0, 0};
	std::vector<double> m_syncWaitUs;
	int64_t m_monadoPeriodNs = 0;
	int64_t m_lastSubmitNs = 0;              //!< layer 0's SubmitLayer time of the current frame
	float m_lastPredictS = 0.0f;             //!< and SteamVR's prediction (s from then)
	std::vector<double> m_targetErrMs;
	std::atomic<int> m_clockVsyncs{0}; //!< vsync events from the steady clock (not Monado's frames)

	// Output to Monado's compositor
	Output m_out;
	Layer m_layerBuf[kMaxLayers];
	bool m_waited = false; //!< a Monado frame was waited for and not submitted yet
	int64_t m_frameId = -1;
	int64_t m_displayNs = 0;
};
