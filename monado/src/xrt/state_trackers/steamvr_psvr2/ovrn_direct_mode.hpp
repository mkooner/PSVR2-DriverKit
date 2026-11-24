// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Driver direct mode (DriverKit's Direct mode setting, DIRECT_MODE=1 in its settings file, or
 *         OVRN_DIRECT_MODE=1; off by default; brought over from the old plugin's ovrd_direct_mode.hpp on 2026-11-24):
 *         SteamVR's compositor hands its finished eye images to the driver
 *         instead of driving the display itself. The images SteamVR renders into are shared Vulkan images we allocate
 *         (as ALVR's Linux OvrDirectModeComponent does). With an output (setOutput, Monado's compositor running
 *         inside vrserver) they are imported into it as swapchains and every SteamVR frame is submitted as projection
 *         layers with the head pose SteamVR rendered it with: Monado's compositor re-warps them to the latest pose,
 *         distorts them and shows them on the headset, paced by its own frame timing. Without an output, SteamVR is
 *         paced on a steady clock at the display rate and the frames are only logged.
 *
 * Log (OVRN_DIRECT_MODE_LOG, csv), host times are CLOCK_MONOTONIC ns:
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
 *   U,host_ns,target_ns,px,py,pz,qx,qy,qz,qw   (every head pose handed to SteamVR; target_ns 0: not a frame pose)
 *   T,host_ns,px,py,pz,qx,qy,qz,qw   (the device's pose at host_ns: where the head was; every 2 ms from a sampler
 *     thread, plus at Present and PostPresent)
 */

#pragma once

#include "openvr_driver.h"
#include "ovrn_log.hpp"
#include "os/os_time.h"

#include "xrt/xrt_compositor.h"
#include "xrt/xrt_device.h"
#include "math/m_api.h"
#include "ovrn_steamvr_sync.hpp"
#include "ovrn_vk_wait.hpp"
#include "ovrn_frame_recorder.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <mutex>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

/*!
 * One head pose per frame (direct mode): the pose for a display time is computed once, by whichever thread asks first
 * (frame start or the 1 ms pose thread), and the same pose is returned for that display time afterwards, so the game's
 * pose and SteamVR's layer label (mHmdPose) are the same pose.
 */
class OvrnFramePose
{
public:
	/*!
	 * The pose for @p display_ns; @p out_target gets the display time of the pose returned. A request for an older
	 * display time than the cached one (a thread that read the target just before the clock thread moved it on) gets
	 * the newer frame's pose, never a recomputed old one (audit 3: that sent SteamVR a stale pose after the new one).
	 */
	template <typename Compute>
	vr::DriverPose_t
	get(int64_t display_ns, Compute compute, int64_t *out_target = NULL)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (display_ns > m_target) {
			m_pose = compute();
			m_target = display_ns;
		}
		if (out_target != NULL) {
			*out_target = m_target;
		}
		return m_pose;
	}

private:
	std::mutex m_mutex;
	int64_t m_target = 0;
	vr::DriverPose_t m_pose = {};
};

class OvrnDirectMode : public vr::IVRDriverDirectModeComponent
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
	 * OVRN_DIRECT_MODE_GPU_SYNC): on NVIDIA nothing else orders Monado's reads after SteamVR's / the app's GPU
	 * writes, so a frame could be shown partly drawn, mixed with the image's previous content (ALVR PR #3425). The
	 * signal is SteamVR's per-texture timeline semaphore (ovrn_steamvr_sync.hpp).
	 */
	bool gpu_sync = true;

	/*!
	 * Diagnostics (OVRN_DIRECT_MODE_CAPTURE=<s>): once, <s> seconds after the first frame with two or more layers,
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

	//! A U row: the head pose handed to SteamVR (TrackedDevicePoseUpdated) for display time @p target_ns.
	void
	logSubmittedPose(int64_t target_ns, const vr::DriverPose_t &pose)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (target_ns != 0 && target_ns != m_hist[(m_histPos + kHist - 1) % kHist].target) {
			// A new frame pose: remember it, to recognise it in SteamVR's labels (SubmitLayer).
			Hist &h = m_hist[m_histPos];
			h.target = target_ns;
			for (int i = 0; i < 3; i++) {
				h.p[i] = pose.vecPosition[i];
			}
			h.q[0] = pose.qRotation.x;
			h.q[1] = pose.qRotation.y;
			h.q[2] = pose.qRotation.z;
			h.q[3] = pose.qRotation.w;
			m_histPos = (m_histPos + 1) % kHist;
			if (m_rec.on()) {
				OvrnFrameRecorder::Rec r = {};
				r.kind = 'U';
				r.t0 = os_monotonic_get_ns();
				r.t1 = target_ns;
				for (int i = 0; i < 3; i++) {
					r.f[i] = (float)h.p[i];
				}
				for (int i = 0; i < 4; i++) {
					r.f[3 + i] = (float)h.q[i];
				}
				m_rec.push(r);
			}
		}
		if (m_log != NULL) {
			const vr::HmdQuaternion_t &q = pose.qRotation;
			fprintf(m_log, "U,%lld,%lld,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f\n",
			        (long long)os_monotonic_get_ns(), (long long)target_ns, pose.vecPosition[0],
			        pose.vecPosition[1], pose.vecPosition[2], q.x, q.y, q.z, q.w);
		}
	}

	/*!
	 * The display time the frame a game starts now will really be shown at (0 until the first frame): Monado's
	 * display time of the current frame plus the measured delay of SteamVR's pipeline (m_delayNs, learned from every
	 * game frame: when Monado showed it minus the time its pose was for). 2026-11-18 12:01: game frames were shown
	 * 22.5 ms after the display time of the frame they started in.
	 */
	//! SteamVR's measured pipeline delay added to the pose target (for tests and logs).
	int64_t
	pipelineDelayNs() const
	{
		return m_delayNs.load();
	}

	int64_t
	displayTarget() const
	{
		return m_targetNs.load(); // fixed for the frame (set at its start), so one pose per frame
	}

	//! The eye-to-head poses changed (IPD dial).
	void
	setEyes(const struct xrt_pose eye_in_head[2])
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_out.eye_in_head[0] = eye_in_head[0];
		m_out.eye_in_head[1] = eye_in_head[1];
	}

	~OvrnDirectMode()
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
				ovrn::log("Direct mode: logging frames to %s\n", log_path);
			} else {
				ovrn::log("Direct mode: can't write %s\n", log_path);
			}
		}
		m_gridNs = os_monotonic_get_ns();
		m_summaryNs = m_gridNs;
		if (m_out.xcn != NULL && gpu_sync) {
			std::string why;
			if (m_vk.init(0, &why)) {
				ovrn::log("Direct mode: waiting for SteamVR's writes on %s (timeline semaphores)\n",
				         m_vk.gpuName().c_str());
			} else {
				ovrn::log("Direct mode: can't wait for SteamVR's writes (%s), frames unsynchronized\n", why.c_str());
			}
		}
		const char *frame_log = getenv("OVRN_FRAME_LOG");
		if (frame_log != NULL && frame_log[0] != '\0') {
			ovrn::log(m_rec.start(frame_log) ? "Direct mode: frame timing to %s (memory only on live threads)\n"
			                                : "Direct mode: can't write %s\n",
			         frame_log);
		}
		m_running = true;
		m_vsyncThread = std::thread(&OvrnDirectMode::vsyncLoop, this);
		if (m_out.xcn != NULL) {
			m_frameThread = std::thread(&OvrnDirectMode::frameLoop, this);
		}
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
			ovrn::log("Direct mode: on, frames logged only, vsync every %.3f ms\n", m_periodNs / 1e6);
		} else {
			ovrn::log("Direct mode: on, frames shown by Monado's compositor\n");
		}
	}

	void
	stop()
	{
		if (m_running.exchange(false) && m_vsyncThread.joinable()) {
			m_vsyncThread.join();
		}
		m_frameCv.notify_all();
		if (m_frameThread.joinable()) {
			m_frameThread.join();
		}
		m_rec.stop();
		if (m_truthThread.joinable()) {
			m_truthThread.join();
		}
		{
			std::lock_guard<std::mutex> comp_lock(m_compMutex);
			forgetShownLocked();
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
		set.compositor = isCompositorPid(unPid);
		set.uid = ++m_setUid;
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
				ovrn::log("Direct mode: NewSharedVulkanImage failed (format %u %ux%u)\n", desc->nFormat,
				         desc->nWidth, desc->nHeight);
				break;
			}
			set.handles[i] = h;
			uint64_t ipc = 0;
			int fd = -1;
			if (!vr::VRIPCResourceManager()->RefResource(h, &ipc) ||
			    !vr::VRIPCResourceManager()->ReceiveSharedFd(ipc, &fd)) {
				ovrn::log("Direct mode: no fd for shared image %d\n", i);
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
		ovrn::log("Direct mode: CreateSwapTextureSet pid %u format %u %ux%u x%u samples: %s, dma-buf %lld bytes\n",
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
		// SteamVR labels an application's image with the frame pose current when it composes, one frame pose newer
		// than the one the application drew with (2026-11-18 12:01: 92% of 432 turning frames, all identified
		// exactly). Give Monado the one before it, so the re-warp starts from the pose the image was drawn with.
		const struct xrt_pose label_device = world_to_device ? world_to_device(world) : world; // SteamVR's own
		int64_t pose_target = 0, label_target = 0;
		bool app_image = false;
		{
			auto it = m_handleToSet.find(perEye[0].hTexture);
			app_image = it != m_handleToSet.end() && !m_sets[it->second].compositor;
		}
		const int k = findHistLocked(world);
		if (k >= 0) {
			label_target = m_hist[k].target;
			const Hist &prev = m_hist[(k + kHist - 1) % kHist];
			if (label_fix && app_image && prev.target != 0 && prev.target < m_hist[k].target) {
				world = {{(float)prev.q[0], (float)prev.q[1], (float)prev.q[2], (float)prev.q[3]},
				         {(float)prev.p[0], (float)prev.p[1], (float)prev.p[2]}};
				pose_target = prev.target;
				m_labelFixed++;
			}
		} else if (app_image) {
			m_labelUnknown++;
		}
		if (m_rec.on()) {
			OvrnFrameRecorder::Rec r = {};
			r.kind = 'L';
			r.t0 = now;
			r.i = m_layers;
			r.t1 = label_target;
			r.t2 = pose_target;
			r.flags = app_image ? 1 : 0;
			m_rec.push(r);
		}
		struct xrt_pose device = world_to_device ? world_to_device(world) : world;
		if (m_layers < kMaxLayers) {
			Layer &L = m_layerBuf[m_layers];
			L.head = device;
			L.poseTarget = pose_target;
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
				const struct xrt_pose &lp = label_device;
				struct xrt_vec3 d = {lp.position.x - rel.pose.position.x, lp.position.y - rel.pose.position.y,
				                     lp.position.z - rel.pose.position.z};
				err_mm = 1000.0f * sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
				float dot = fabsf(lp.orientation.x * rel.pose.orientation.x + lp.orientation.y * rel.pose.orientation.y +
				                  lp.orientation.z * rel.pose.orientation.z + lp.orientation.w * rel.pose.orientation.w);
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

	/*!
	 * Hands the frame to the frame thread (frameLoop) and returns at once: SteamVR's submit thread is never held
	 * (ALVR PR #3425: holding it made vrcompositor's WaitForAcquire time out and render into the texture still being
	 * read). Only the values the frame's GPU writes will signal are read here, as they must be read at Present.
	 */
	void
	Present(vr::SharedTextureHandle_t syncTexture) override
	{
		logTruth();
		const int64_t t_in = os_monotonic_get_ns();
		if (m_out.xcn != NULL && gpu_sync && m_vk.ready()) {
			maybeCapture(); // opt-in (capture_after_s), blocks once
		}
		Pending p = {};
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			p.n = std::min(m_layers, kMaxLayers);
			for (int l = 0; l < p.n; l++) {
				p.layers[l] = m_layerBuf[l];
			}
		}
		if (m_out.xcn != NULL && gpu_sync && m_vk.ready()) {
			for (int l = 0; l < p.n; l++) {
				for (int e = 0; e < 2; e++) {
					const vr::SharedTextureHandle_t t = p.layers[l].tex[e];
					bool seen = false;
					for (int k = 0; k < p.nWaits; k++) {
						seen = seen || p.waits[k].texture == t;
					}
					if (t == 0 || seen || p.nWaits >= kMaxWaits) {
						continue;
					}
					Wait w = {};
					w.texture = t;
					w.sem = m_sync.semaphoreOf(t);
					if (w.sem == 0 || !m_sync.value(w.sem, &w.value)) {
						p.missing = true;
						continue;
					}
					p.waits[p.nWaits++] = w;
				}
			}
			p.sync = true;
		}
		std::lock_guard<std::mutex> lock(m_mutex);
		const int64_t now = os_monotonic_get_ns();
		p.valid = true;
		p.presentNs = now;
		p.syncTexture = syncTexture;
		p.layerCount = m_layers;
		p.lastSubmitNs = m_lastSubmitNs;
		p.predictS = m_lastPredictS;
		if (m_out.xcn != NULL) {
			if (m_pending.valid) {
				// The frame thread didn't take the previous frame yet: this newer one replaces it.
				m_replaced++;
				logPresentLocked(m_pending, -1, 0, false, 0, 0);
				OvrnFrameRecorder::Rec r = {};
				r.kind = 'R';
				r.t0 = m_pending.presentNs;
				m_rec.push(r);
			}
			m_pending = p;
			m_frameCv.notify_one();
		} else {
			logPresentLocked(p, m_frameId, m_displayNs, false, 0, 0);
		}
		if (m_lastPresentNs != 0) {
			m_presentGapsMs.push_back((now - m_lastPresentNs) / 1e6);
		}
		m_lastPresentNs = now;
		m_lastPresentAt = now;
		m_frames++;
		m_layerSum += m_layers;
		m_layers = 0;
		const int64_t call_ns = os_monotonic_get_ns() - t_in;
		m_presentCallUs.push_back(call_ns / 1e3);
		if (m_rec.on()) {
			OvrnFrameRecorder::Rec r = {};
			r.kind = 'P';
			r.t0 = now;
			r.layers = (uint8_t)std::min(p.layerCount, 255);
			r.i = (int32_t)std::min<int64_t>(call_ns, INT32_MAX);
			m_rec.push(r);
		}
		summaryLocked(now);
	}

	//! With Monado's compositor: holds until the next declared vsync (the clock thread sends the vsync events).
	void
	PostPresent(const Throttling_t *throttling) override
	{
		int64_t woke;
		if (m_out.xcn != NULL) {
			// Hold SteamVR's submit thread until the next declared vsync (+ throttled frames), measured from the
			// last declared one so a skipped tick doesn't lengthen it (ALVR's PaceAfterPresent); at most 2 periods.
			const int64_t period = m_gridPeriodNs.load() > 0 ? m_gridPeriodNs.load() : m_periodNs;
			const int64_t last = m_lastVsyncNs.load();
			const int64_t now = os_monotonic_get_ns();
			if (hold && last != 0) {
				const uint32_t thr = throttling ? throttling->nFramesToThrottle : 0;
				int64_t until = last + period * (1 + (int64_t)thr);
				until = std::min(until, now + 2 * period);
				if (until > now) {
					os_nanosleep(until - now);
				}
			}
			woke = os_monotonic_get_ns();
		} else {
			// Hold SteamVR's compositor until the next tick of the display clock, as ALVR does: it then starts
			// its next frame one display period after the last.
			woke = waitNextTick();
		}
		logTruth();
		if (m_rec.on()) {
			OvrnFrameRecorder::Rec r = {};
			r.kind = 'Q';
			r.t0 = os_monotonic_get_ns();
			r.i = throttling ? (int32_t)throttling->nFramesToThrottle : 0;
			r.j = throttling ? (int32_t)throttling->nAdditionalFramesToPredict : 0;
			m_rec.push(r);
		}
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
		bool compositor = false;          //!< created by vrcompositor (SteamVR draws it itself)
		bool copied = false;              //!< xsc is Monado's own swapchain, SteamVR's images are copied into it
		uint64_t uid = 0;                 //!< unique per set (copy image keys; pointers can be reused)
	};

	static constexpr int kMaxLayers = 8;
	struct Layer
	{
		struct xrt_pose head;          //!< SteamVR's render pose, in the device's space
		vr::SharedTextureHandle_t tex[2];
		vr::VRTextureBounds_t bounds[2];
		int64_t poseTarget; //!< the display time of the frame pose given as its pose (0: SteamVR's label kept)
	};
	static constexpr int kMaxWaits = 2 * kMaxLayers;
	struct Wait
	{
		vr::SharedTextureHandle_t texture;
		uint64_t sem;
		uint64_t value; //!< the value this frame's write signals (read at Present)
	};
	//! A presented frame, from Present to the frame thread.
	struct Pending
	{
		bool valid;
		Layer layers[kMaxLayers];
		int n;          //!< layers kept
		int layerCount; //!< layers SteamVR submitted
		Wait waits[kMaxWaits];
		int nWaits;
		bool sync;    //!< GPU sync on
		bool missing; //!< an image had no semaphore (yet)
		int64_t presentNs;
		vr::SharedTextureHandle_t syncTexture;
		int64_t lastSubmitNs;
		float predictS;
		int dstIndex[kMaxLayers][2]; //!< Monado swapchain image the copy went to (-1: SteamVR's image itself)
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
		if (copy_mode && (m_vk.ready() || test_copy)) {
			// Monado's own swapchain: SteamVR's images are copied into it each frame with the GPU synchronisation
			// the hardware needs (OvrnVkWait::copy), as Monado's own Vulkan clients hand images over.
			info.bits = (enum xrt_swapchain_usage_bits)(info.bits | XRT_SWAPCHAIN_USAGE_TRANSFER_DST);
			xrt_result_t cret;
			{
				std::lock_guard<std::mutex> comp_lock(m_compMutex);
				cret = xrt_comp_create_swapchain(&m_out.xcn->base, &info, &set.xsc);
			}
			set.copied = cret == XRT_SUCCESS && set.xsc != NULL;
			ovrn::log("Direct mode: Monado swapchain for copies (format %u %ux%u): %s (%d)\n", set.desc.nFormat,
			         set.desc.nWidth, set.desc.nHeight, set.copied ? "ok" : "FAILED", (int)cret);
			if (set.copied) {
				return;
			}
		}
		struct xrt_image_native natives[3] = {};
		for (int i = 0; i < 3; i++) {
			natives[i].handle = dup(set.fds[i]); // the compositor takes ownership
			natives[i].size = 0;
			natives[i].use_dedicated_allocation = true;
		}
		xrt_result_t xret;
		{
			std::lock_guard<std::mutex> comp_lock(m_compMutex);
			xret = xrt_comp_import_swapchain(&m_out.xcn->base, &info, natives, 3, &set.xsc);
		}
		if (xret != XRT_SUCCESS) {
			for (int i = 0; i < 3; i++) {
				close(natives[i].handle);
			}
			set.xsc = NULL;
		}
		ovrn::log("Direct mode: import into Monado's compositor (format %u %ux%u): %s (%d)\n", set.desc.nFormat,
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
	 * Waits until every image of @p p has been written (SteamVR's per-texture timeline semaphore reaches the value
	 * read at Present). @p state: 1 all done, 2 a wait timed out, 3 an image had no semaphore (yet), 4 import/wait
	 * error. Bounded by one frame period in total. Runs on the frame thread, never on SteamVR's.
	 */
	void
	waitForWrites(const Pending &p, int *state, int64_t *waited_ns)
	{
		const int64_t t0 = os_monotonic_get_ns();
		int result = p.missing ? 3 : 1;
		for (int k = 0; k < p.nWaits; k++) {
			const Wait &w = p.waits[k];
			const int64_t left = std::max<int64_t>(0, t0 + m_periodNs - (int64_t)os_monotonic_get_ns());
			const uint64_t sem = w.sem;
			OvrnVkWait::Result r =
			    m_vk.wait(sem, w.value, (uint64_t)left, [this, sem]() { return m_sync.dupFd(sem); });
			if (r == OvrnVkWait::kTimeout) {
				result = std::max(result, 2);
			} else if (r == OvrnVkWait::kError) {
				result = 4;
			}
		}
		*state = (!p.sync || (p.nWaits == 0 && !p.missing)) ? 0 : result;
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
			ovrn::log("Direct mode: captured layer %d (%ux%u format %u) to %s: %s\n", it.layer, it.w, it.h,
			         it.format, path.c_str(), ok ? "ok" : why.c_str());
		}
	}

	/*!
	 * The hand-over thread: gives SteamVR's newest frame to Monado. Per Monado frame: wait for its wake-up, take the
	 * newest presented frame (waiting for one at most until handoff_margin before its display time), wait for its GPU
	 * writes, copy it (copyLayers) and commit it. It doesn't pace SteamVR: Monado's app pacing moves this thread's
	 * wake-up with its own measured frame times (u_pacing_app.c), so SteamVR's vsync events and poses come from the
	 * steady clock thread (clockLoop) instead. A Monado frame without a usable new SteamVR frame gets the last frame
	 * shown again (repeatLast), which Monado re-warps to the head pose of the moment.
	 */
	void
	frameLoop()
	{
		struct xrt_compositor *xc = &m_out.xcn->base;
		while (m_running) {
			int64_t id = 0, display = 0, period = 0;
			xrt_result_t xret = xrt_comp_wait_frame(xc, &id, &display, &period);
			const int64_t woke = os_monotonic_get_ns();
			if (xret != XRT_SUCCESS) {
				{
					std::lock_guard<std::mutex> lock(m_mutex);
					m_waitFails++;
				}
				os_nanosleep(m_periodNs);
				continue;
			}
			{
				std::lock_guard<std::mutex> lock(m_mutex);
				m_frameId = id;
				m_displayNs = display;
				m_monadoPeriodNs = period;
			}
			// The display grid for the clock thread (Monado's display times are on its vblank grid).
			if (period > 0) {
				m_gridPeriodNs = period;
			}
			m_gridAnchorNs = display;
			m_displayAt = display;
			const int64_t vsync_ns = m_lastVsyncNs.load();
			if (test_handover_delay_ns > 0) {
				os_nanosleep(test_handover_delay_ns); // tests: a slow hand-over must not move SteamVR's clock
			}
			// The newest presented frame; if none is waiting, wait for one until handoff_margin before display.
			const int64_t deadline = display - handoff_margin_ns;
			Pending p = {};
			{
				std::unique_lock<std::mutex> lock(m_mutex);
				const auto until = std::chrono::steady_clock::now() +
				                   std::chrono::nanoseconds(std::max<int64_t>(0, deadline - os_monotonic_get_ns()));
				m_frameCv.wait_until(lock, until, [this]() { return m_pending.valid || !m_running; });
				if (m_pending.valid) {
					p = m_pending;
					m_pending.valid = false;
				}
			}
			if (!p.valid) {
				const bool repeated = repeatLast(id, display);
				std::lock_guard<std::mutex> lock(m_mutex);
				m_missed++;
				m_repeated += repeated ? 1 : 0;
				recordFrameLocked(woke, id, display, vsync_ns, p, 0, 0, 4);
				continue;
			}
			int sync_state = 0;
			int64_t sync_wait_ns = 0;
			if (p.sync) {
				waitForWrites(p, &sync_state, &sync_wait_ns);
			}
			if (sync_state == 2 || sync_state == 4) {
				// A write that didn't finish in time (or a failed wait) would be shown half drawn: the last frame is
				// shown again instead (audit 9). Without a semaphore yet (sync 3) there is nothing to wait on: shown
				// as is (a mitigation, as before).
				const bool repeated = repeatLast(id, display);
				std::lock_guard<std::mutex> lock(m_mutex);
				m_repeated += repeated ? 1 : 0;
				m_syncStates[sync_state]++;
				m_syncDropped++;
				logPresentLocked(p, id, display, false, 0, sync_wait_ns, sync_state);
				recordFrameLocked(woke, id, display, vsync_ns, p, sync_wait_ns, sync_state, 16);
				continue;
			}
			if (!copyLayers(p, sync_state)) {
				const bool repeated = repeatLast(id, display);
				std::lock_guard<std::mutex> lock(m_mutex);
				m_repeated += repeated ? 1 : 0;
				m_copyFails++;
				logPresentLocked(p, id, display, false, 0, sync_wait_ns, sync_state);
				recordFrameLocked(woke, id, display, vsync_ns, p, sync_wait_ns, sync_state, 8);
				continue;
			}
			// Build the frame under m_mutex, but hand it to Monado without it: layer_commit renders the whole frame
			// synchronously (comp_compositor.c), which must not block SteamVR's Present, the clock thread's pose
			// and vsync, or the pose thread (audit 1).
			Prepared prep;
			{
				std::lock_guard<std::mutex> lock(m_mutex);
				prepareLocked(p, id, display, &prep);
			}
			bool begun = false;
			const bool submitted = submitPrepared(prep, id, display, &begun);
			releasePrepared(&prep);
			// Nothing of the new frame could be shown (no layer usable): the last frame again, not a black one.
			const bool repeated = !submitted && !begun && repeatLast(id, display);
			std::lock_guard<std::mutex> lock(m_mutex);
			m_repeated += repeated ? 1 : 0;
			int64_t target_err = 0;
			if (p.layerCount > 0 && p.lastSubmitNs != 0) {
				// SteamVR's prediction target for this frame vs the time Monado shows it (measurement only).
				const int64_t target = p.lastSubmitNs + (int64_t)(p.predictS * 1e9);
				target_err = target - display;
				m_targetErrMs.push_back(target_err / 1e6);
			}
			if (sync_state != 0) {
				m_syncStates[sync_state]++;
				if (sync_state == 1) {
					m_syncWaitUs.push_back(sync_wait_ns / 1e3);
				}
			}
			m_submitted += submitted ? 1 : 0;
			if (submitted && p.n > 0 && p.layers[0].poseTarget != 0) {
				// When this image is shown minus the time its pose was for: steer the pose target onto it.
				const int64_t err = display - p.layers[0].poseTarget;
				m_delayErrMs.push_back(err / 1e6);
				m_recentErrMs.push_back(err / 1e6);
				if (m_recentErrMs.size() > 30) {
					m_recentErrMs.erase(m_recentErrMs.begin());
				}
				// Steer by the median of the last 9 errors (a single mislabelled frame can't jolt the target), a
				// quarter of it per frame: converges in ~10 frames (the old err/20 with a 0.25 ms cap per frame took
				// ~20 s at 90 Hz). The errors were measured against targets older than the latest correction, so the
				// window restarts after each step to avoid steering twice on the same error.
				m_steerErrs.push_back(err);
				if (m_steerErrs.size() >= 9) {
					std::vector<int64_t> v = m_steerErrs;
					std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
					const int64_t period = m_gridPeriodNs.load() > 0 ? m_gridPeriodNs.load() : m_periodNs;
					const int64_t med = std::max<int64_t>(-2 * period, std::min<int64_t>(2 * period, v[v.size() / 2]));
					const int64_t cap = 6 * period; // at most 6 display periods
					m_delayNs = std::max<int64_t>(0, std::min<int64_t>(cap, m_delayNs.load() + med / 2));
					m_steerErrs.clear();
				}
			}
			logPresentLocked(p, id, display, submitted, target_err, sync_wait_ns, sync_state);
			recordFrameLocked(woke, id, display, vsync_ns, p, sync_wait_ns, sync_state, submitted ? 1 : 0);
		}
	}

	//! F row (OVRN_FRAME_LOG): one Monado frame as the frame thread handled it.
	void
	recordFrameLocked(int64_t woke,
	                  int64_t id,
	                  int64_t display,
	                  int64_t vsync_ns,
	                  const Pending &p,
	                  int64_t wait_ns,
	                  int sync_state,
	                  int flags)
	{
		if (!m_rec.on()) {
			return;
		}
		OvrnFrameRecorder::Rec r = {};
		r.kind = 'F';
		r.t0 = woke;
		r.t1 = id;
		r.t2 = display;
		r.t3 = vsync_ns;
		r.t4 = p.valid ? p.presentNs : 0;
		r.i = (int32_t)(wait_ns / 1000);
		r.sync = (int8_t)sync_state;
		r.flags = (uint8_t)flags;
		r.layers = (uint8_t)(p.valid ? std::min(p.layerCount, 255) : 0);
		uint64_t *h[2] = {&r.h0, &r.h1};
		for (int l = 0; l < 2 && l < p.n; l++) {
			auto it = m_handleToSet.find(p.layers[l].tex[0]);
			*h[l] = it != m_handleToSet.end() ? it->second : 0;
		}
		m_rec.push(r);
	}

	//! P row for a presented frame (@p frame_id -1: replaced by a newer frame before it was shown).
	void
	logPresentLocked(const Pending &p,
	                 int64_t frame_id,
	                 int64_t display,
	                 bool submitted,
	                 int64_t target_err,
	                 int64_t sync_wait_ns,
	                 int sync_state = 0)
	{
		if (m_log != NULL) {
			fprintf(m_log, "P,%lld,%llu,%d,%lld,%lld,%d,%lld,%.1f,%d\n", (long long)p.presentNs,
			        (unsigned long long)p.syncTexture, p.layerCount, (long long)frame_id, (long long)display,
			        submitted ? 1 : 0, (long long)target_err, sync_wait_ns / 1e3, sync_state);
		}
	}

	/*!
	 * Tells Monado frame @p frame_id won't be submitted. Monado's multi compositor then drops the client's last
	 * delivered frame (multi_compositor_discard_frame -> multi_compositor_retire_delivered_locked), so that display
	 * period shows no layers at all: black. Only for when there is nothing to show yet (repeatLast).
	 */
	void
	discardFrameLocked(int64_t frame_id)
	{
		struct xrt_compositor *xc = &m_out.xcn->base;
		if (xrt_comp_begin_frame(xc, frame_id) == XRT_SUCCESS) {
			xrt_comp_discard_frame(xc, frame_id);
		}
	}

	//! A frame ready for Monado: its layers with references to their swapchains (built under m_mutex).
	struct Prepared
	{
		int n = 0;
		struct xrt_layer_data d[kMaxLayers];
		struct xrt_swapchain *xsc[kMaxLayers][XRT_MAX_VIEWS];
		//! Every layer is in Monado's own copy swapchains (copy mode): the images stay ours, so the frame can be
		//! shown again later. SteamVR's own images (no copy) are drawn into again by SteamVR: never repeated.
		bool repeatable = false;
	};

	/*!
	 * Frame @p frame_id with the last frame shown (its images and the head pose they were drawn with; Monado re-warps
	 * it to the pose of the moment), for a Monado frame SteamVR had no usable new frame for: a game that missed its
	 * frame, a write that didn't finish, a failed copy. Before 2026-11-24 such a frame was discarded, which Monado
	 * shows as black (discardFrameLocked): rare random black frames in direct mode (2026-11-24 11:50 session, 3-58
	 * "Monado frames without one" per 10 s). The image stays valid: it is in our copy swapchain, whose next copy
	 * goes to another image (3 per set, and Monado's wait_image waits while a delivered layer still uses one).
	 * False when there is no frame to repeat yet (then discarded, as at the start).
	 */
	bool
	repeatLast(int64_t frame_id, int64_t display_ns)
	{
		std::lock_guard<std::mutex> comp_lock(m_compMutex);
		struct xrt_compositor *xc = &m_out.xcn->base;
		if (m_last.n == 0) {
			discardFrameLocked(frame_id);
			return false;
		}
		if (xrt_comp_begin_frame(xc, frame_id) != XRT_SUCCESS) {
			return false;
		}
		struct xrt_layer_frame_data fd = {};
		fd.frame_id = frame_id;
		fd.display_time_ns = display_ns;
		fd.env_blend_mode = XRT_BLEND_MODE_OPAQUE;
		if (xrt_comp_layer_begin(xc, &fd) != XRT_SUCCESS) {
			xrt_comp_discard_frame(xc, frame_id);
			return false;
		}
		int shown = 0;
		for (int l = 0; l < m_last.n; l++) {
			struct xrt_layer_data d = m_last.d[l];
			d.timestamp = display_ns;
			struct xrt_swapchain *xsc[XRT_MAX_VIEWS] = {m_last.xsc[l][0], m_last.xsc[l][1]};
			if (xrt_comp_layer_projection(xc, m_out.head, xsc, &d) == XRT_SUCCESS) {
				shown++;
			}
		}
		return xrt_comp_layer_commit(xc, XRT_GRAPHICS_SYNC_HANDLE_INVALID) == XRT_SUCCESS && shown > 0;
	}

	//! The frame just shown becomes the one repeatLast shows (its own swapchain references); under m_compMutex.
	void
	rememberShownLocked(const Prepared &prep)
	{
		forgetShownLocked();
		if (!prep.repeatable) {
			return;
		}
		for (int l = 0; l < prep.n; l++) {
			m_last.d[l] = prep.d[l];
			for (int e = 0; e < 2; e++) {
				m_last.xsc[l][e] = NULL;
				xrt_swapchain_reference(&m_last.xsc[l][e], prep.xsc[l][e]);
			}
		}
		m_last.n = prep.n;
		m_last.repeatable = true;
	}

	void
	forgetShownLocked()
	{
		for (int l = 0; l < m_last.n; l++) {
			for (int e = 0; e < 2; e++) {
				xrt_swapchain_reference(&m_last.xsc[l][e], NULL);
			}
		}
		m_last.n = 0;
		m_last.repeatable = false;
	}

	//! Hands @p prep to Monado as frame @p frame_id (without m_mutex). Nothing to show: the frame is discarded, so
	//! Monado keeps showing the last one instead of an empty (black) frame (audit 8).
	bool
	submitPrepared(const Prepared &prep, int64_t frame_id, int64_t display_ns, bool *out_begun)
	{
		std::lock_guard<std::mutex> comp_lock(m_compMutex);
		struct xrt_compositor *xc = &m_out.xcn->base;
		*out_begun = false;
		if (prep.n == 0) {
			return false; // nothing usable: the caller shows the last frame again
		}
		if (xrt_comp_begin_frame(xc, frame_id) != XRT_SUCCESS) {
			return false;
		}
		*out_begun = true;
		struct xrt_layer_frame_data fd = {};
		fd.frame_id = frame_id;
		fd.display_time_ns = display_ns;
		fd.env_blend_mode = XRT_BLEND_MODE_OPAQUE;
		if (xrt_comp_layer_begin(xc, &fd) != XRT_SUCCESS) {
			xrt_comp_discard_frame(xc, frame_id);
			return false;
		}
		int shown = 0;
		for (int l = 0; l < prep.n; l++) {
			struct xrt_swapchain *xsc[XRT_MAX_VIEWS] = {prep.xsc[l][0], prep.xsc[l][1]};
			if (xrt_comp_layer_projection(xc, m_out.head, xsc, &prep.d[l]) == XRT_SUCCESS) {
				shown++;
			}
		}
		// Layers already given can't be taken back: commit even if one was refused.
		const bool ok = xrt_comp_layer_commit(xc, XRT_GRAPHICS_SYNC_HANDLE_INVALID) == XRT_SUCCESS && shown > 0;
		if (ok && shown == prep.n) {
			rememberShownLocked(prep);
		}
		return ok;
	}

	void
	releasePrepared(Prepared *prep)
	{
		std::lock_guard<std::mutex> comp_lock(m_compMutex);
		for (int l = 0; l < prep->n; l++) {
			for (int e = 0; e < 2; e++) {
				xrt_swapchain_reference(&prep->xsc[l][e], NULL);
			}
		}
		prep->n = 0;
	}

	//! Builds @p prep from @p p's layers (references the swapchains so a set destroyed meanwhile stays valid).
	void
	prepareLocked(const Pending &p, int64_t frame_id, int64_t display_ns, Prepared *prep)
	{
		prep->n = 0;
		prep->repeatable = true;
		for (int l = 0; l < std::min(p.n, max_layers); l++) {
			const Layer &L = p.layers[l];
			struct xrt_swapchain *xsc[XRT_MAX_VIEWS] = {};
			struct xrt_layer_data d = {};
			bool flip[2] = {false, false};
			d.type = XRT_LAYER_PROJECTION;
			d.name = XRT_INPUT_GENERIC_HEAD_POSE;
			d.timestamp = display_ns;
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
				if (set.copied) {
					if (p.dstIndex[l][e] < 0) {
						ok = false; // not copied this frame
						break;
					}
					index = (uint32_t)p.dstIndex[l][e];
				} else {
					prep->repeatable = false; // SteamVR's own image: it draws into it again
				}
				xsc[e] = set.xsc;
				vr::VRTextureBounds_t b = L.bounds[e];
				if (b.vMin > b.vMax) {
					std::swap(b.vMin, b.vMax);
					flip[e] = true;
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
			// flip_y is per layer in Monado, bounds are per eye in SteamVR (audit 12).
			if (ok && flip[0] != flip[1]) {
				if (!m_flipWarned) {
					ovrn::log("Direct mode: a layer flips one eye but not the other; not shown\n");
					m_flipWarned = true;
				}
				ok = false;
			}
			d.flip_y = flip[0];
			if (!ok) {
				continue;
			}
			if (l == 0 && m_log != NULL) {
				struct xrt_space_relation pred = {};
				xrt_device_get_tracked_pose(m_out.head, XRT_INPUT_GENERIC_HEAD_POSE, display_ns, &pred);
				const struct xrt_pose &r = L.head;
				const struct xrt_pose &q = pred.pose;
				fprintf(m_log,
				        "R,%lld,%lld,%lld,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f\n",
				        (long long)os_monotonic_get_ns(), (long long)frame_id, (long long)display_ns,
				        r.position.x, r.position.y, r.position.z, r.orientation.x, r.orientation.y,
				        r.orientation.z, r.orientation.w, q.position.x, q.position.y, q.position.z,
				        q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w);
			}
			const int k = prep->n++;
			prep->d[k] = d;
			for (int e = 0; e < 2; e++) {
				prep->xsc[k][e] = NULL;
				xrt_swapchain_reference(&prep->xsc[k][e], xsc[e]);
			}
		}
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
		if (it->second.copied) {
			std::vector<uint64_t> dst;
			for (uint64_t i = 0; i < 3; i++) {
				dst.push_back(dstKey(it->second.uid, (uint32_t)i));
			}
			m_vk.forgetImages(std::vector<uint64_t>(it->second.handles, it->second.handles + 3), dst);
		}
		{
			std::lock_guard<std::mutex> comp_lock(m_compMutex);
			xrt_swapchain_reference(&it->second.xsc, NULL);
		}
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
	 * With Monado's compositor, the frame thread sends them paced by Monado's frames; this clock then only runs
	 * while that thread sent none for 100 ms, so SteamVR never waits forever for a vsync to start a frame.
	 */
	/*!
	 * SteamVR's vsync clock with Monado's compositor (Valve's driver direct mode sequence, as ALVR's vsync.rs): a
	 * steady grid on Monado's display timing that doesn't follow the frames. Each declared vsync V is
	 * vsync_lead_ns before a Monado display time; at V - running_start_ns ("running start") the head pose for the
	 * display time the frame SteamVR starts now will be shown at is submitted, then VsyncEvent with the time until V.
	 */
	void
	clockLoop()
	{
		while (m_running) {
			const int64_t period = m_gridPeriodNs.load() > 0 ? m_gridPeriodNs.load() : m_periodNs;
			const int64_t anchor = m_gridAnchorNs.load();
			const int64_t now = os_monotonic_get_ns();
			int64_t v;
			if (anchor == 0) {
				v = now + running_start_ns + period; // no Monado frame yet: a provisional grid
			} else {
				const int64_t base = anchor - vsync_lead_ns;
				// The first grid vsync whose running start is after now: floor division, also when base is ahead
				// of now (truncation skipped a vsync there, audit 6).
				const int64_t x = now + running_start_ns - base;
				const int64_t n = (x >= 0 ? x / period : -((-x + period - 1) / period)) + 1;
				v = base + n * period;
			}
			const int64_t fire = v - running_start_ns;
			if (fire > now) {
				os_nanosleep(fire - now);
			}
			if (!m_running) {
				break;
			}
			if (m_delayNs.load() == 0 && anchor != 0) {
				m_delayNs = vsync_lead_ns + period; // first guess; then learned from when frames are shown
			}
			m_targetNs = v + m_delayNs.load();
			if (on_frame_start) {
				on_frame_start(displayTarget());
			}
			const int64_t t = os_monotonic_get_ns();
			vr::VRServerDriverHost()->VsyncEvent((double)(v - t) / 1e9);
			m_lastVsyncNs = v;
			m_lastFrameVsyncNs = t;
		}
	}

	void
	vsyncLoop()
	{
		if (m_out.xcn != NULL) {
			clockLoop();
			return;
		}
		while (m_running) {
			waitNextTick();
			if (m_out.xcn != NULL && os_monotonic_get_ns() - m_lastFrameVsyncNs.load() < 100000000LL) {
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
		ovrn::log("Direct mode (%.0f s): %d frames (%.1f/s), %.2f layers/frame; Present interval median %.2f p99 %.2f "
		         "ms; pose predicted median %.2f (%.2f-%.2f) ms ahead; %zu image sets; shown %d, wait failures %d; "
		         "pose vs device median %.1f mm / %.2f deg, p99 %.1f mm / %.2f deg; %d clock vsyncs; SteamVR's "
		         "target - display median %+.2f (%+.2f..%+.2f) ms; GPU sync: %d waited (median %.0f us, p99 %.0f "
		         "us), %d timed out, %d without semaphore, %d errors; Present call median %.0f us, max %.0f us; %d "
		         "frames replaced, %d Monado frames without one, %d shown again; labels corrected %d, unknown %d; pose target delay "
		         "%.2f ms, shown - pose time median %+.2f (p10 %+.2f, p90 %+.2f) ms; GPU copy median %.0f us, p99 %.0f "
		         "us, %d failed\n",
		         secs, m_frames, m_frames / secs, m_frames ? (double)m_layerSum / m_frames : 0.0,
		         pct(m_presentGapsMs, 0.5), pct(m_presentGapsMs, 0.99), pct(pred_ms, 0.5), pct(pred_ms, 0.01),
		         pct(pred_ms, 0.99), m_sets.size(), m_submitted, m_waitFails, pct(m_errMm, 0.5),
		         pct(m_errDeg, 0.5), pct(m_errMm, 0.99), pct(m_errDeg, 0.99), m_clockVsyncs.exchange(0),
		         pct(m_targetErrMs, 0.5), pct(m_targetErrMs, 0.01), pct(m_targetErrMs, 0.99), m_syncStates[1],
		         pct(m_syncWaitUs, 0.5), pct(m_syncWaitUs, 0.99), m_syncStates[2], m_syncStates[3],
		         m_syncStates[4], pct(m_presentCallUs, 0.5), pct(m_presentCallUs, 1.0), m_replaced, m_missed,
		         m_repeated, m_labelFixed, m_labelUnknown, m_delayNs.load() / 1e6, pct(m_delayErrMs, 0.5), pct(m_delayErrMs, 0.1),
		         pct(m_delayErrMs, 0.9), pct(m_copyUs, 0.5), pct(m_copyUs, 0.99), m_copyFails);
		m_copyUs.clear();
		m_copyFails = 0;
		m_labelFixed = 0;
		m_labelUnknown = 0;
		m_delayErrMs.clear();
		m_presentCallUs.clear();
		m_replaced = 0;
		m_missed = 0;
		m_repeated = 0;
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
	ovrn_steamvr_sync::Sync m_sync;
	OvrnVkWait m_vk;
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
	int64_t m_frameId = -1; //!< the frame thread's current Monado frame
	int64_t m_displayNs = 0;
	std::thread m_frameThread;
	std::condition_variable m_frameCv;
	Pending m_pending = {};              //!< the newest presented frame not taken by the frame thread yet
	std::atomic<int64_t> m_lastFrameVsyncNs{0};
	int m_replaced = 0;                  //!< presented frames replaced before the frame thread took them
	int m_missed = 0;                    //!< Monado frames SteamVR delivered no frame for in time
	int m_repeated = 0;                  //!< Monado frames given the last frame again (repeatLast)
	std::vector<double> m_presentCallUs; //!< time spent in Present (SteamVR's thread)
	OvrnFrameRecorder m_rec;             //!< OVRN_FRAME_LOG
	std::vector<double> m_copyUs;        //!< GPU copy time per frame
	int m_copyFails = 0;
	int m_syncDropped = 0; //!< frames not shown because their write didn't finish in time
	uint64_t m_lastDstKey = 0;           //!< for tests: the last copy destination of layer 0's left eye
	uint64_t m_lastSrcKey = 0;           //!< and its source (SteamVR's image)
	uint64_t m_setUid = 0;
	bool m_flipWarned = false;
	Prepared m_last;                     //!< the last frame shown, for repeatLast (under m_compMutex)
	std::mutex m_compMutex;              //!< serializes calls into Monado's compositor (never held with m_mutex
	                                     //!< taken after it: m_mutex -> m_compMutex only)

public:
	//! OVRN_DIRECT_MODE_COPY (default on): copy SteamVR's images into Monado's own swapchains (see importLocked).
	bool copy_mode = true;

	//! For tests: the first pixel (RGBA8) of the image Monado was last given for layer 0's left eye.
	bool
	readLastOutputPixel(uint8_t out[4])
	{
		uint64_t key;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			key = m_lastDstKey;
		}
		return key != 0 && m_vk.readFirstPixel(key, out);
	}

	//! For tests: the first pixel of SteamVR's image that was copied into it.
	bool
	readLastSourcePixel(uint8_t out[4])
	{
		uint64_t key;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			key = m_lastSrcKey;
		}
		return key != 0 && m_vk.readFirstSourcePixel(key, out);
	}

	//! For tests: the median of the last shown - pose time errors (ms), NAN if none.
	double
	recentDelayErrMs()
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		std::vector<double> v(m_recentErrMs.begin(), m_recentErrMs.end());
		if (v.empty()) {
			return NAN;
		}
		std::sort(v.begin(), v.end());
		return v[v.size() / 2];
	}

private:
	static uint64_t
	dstKey(uint64_t set_uid, uint32_t index)
	{
		return (set_uid << 2) | index;
	}

	/*!
	 * For copied sets: takes a Monado swapchain image for every image of @p p (Monado's acquire/wait/release), copies
	 * SteamVR's image into it on the GPU (OvrnVkWait::copy: waits on the writes, acquire/release barriers) and fills
	 * p.dstIndex. The GPU waits only on writes the CPU already saw finish (@p sync_state 1), so a stuck write can't
	 * stall the GPU. False if a copy failed (the frame isn't shown).
	 */
	bool
	copyLayers(Pending &p, int sync_state)
	{
		struct Item
		{
			int l, e;
			struct xrt_swapchain *xsc;
			OvrnVkWait::CopyJob job;
			uint32_t di;
			bool acquired;
			uint64_t uid;
			vr::SharedTextureHandle_t first; //!< the set's key in m_sets
		};

		std::vector<Item> items;
		for (int l = 0; l < kMaxLayers; l++) {
			p.dstIndex[l][0] = p.dstIndex[l][1] = -1;
		}
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			for (int l = 0; l < p.n; l++) {
				for (int e = 0; e < 2; e++) {
					auto it = m_handleToSet.find(p.layers[l].tex[e]);
					if (it == m_handleToSet.end() || !m_sets[it->second].copied) {
						continue;
					}
					Set &set = m_sets[it->second];
					int src_fd = -1;
					for (int i = 0; i < 3; i++) {
						if (set.handles[i] == p.layers[l].tex[e] && set.fds[i] >= 0) {
							src_fd = dup(set.fds[i]);
						}
					}
					Item item = {};
					item.l = l;
					item.e = e;
					item.uid = set.uid;
					item.first = it->second;
					xrt_swapchain_reference(&item.xsc, set.xsc);
					item.job = {p.layers[l].tex[e], src_fd, 0, -1, set.desc.nWidth, set.desc.nHeight,
					            set.desc.nFormat};
					items.push_back(item);
				}
			}
		}
		if (items.empty()) {
			return true;
		}
		bool ok = true;
		std::vector<OvrnVkWait::CopyJob> jobs;
		for (Item &it : items) {
			it.acquired = false;
			if (!ok || xrt_swapchain_acquire_image(it.xsc, &it.di) != XRT_SUCCESS) {
				ok = false;
				continue;
			}
			it.acquired = true; // released below whatever happens (audit 2: a failed wait leaked the index)
			if (xrt_swapchain_wait_image(it.xsc, m_periodNs, it.di) != XRT_SUCCESS) {
				ok = false;
				continue;
			}
			struct xrt_swapchain_native *xscn = (struct xrt_swapchain_native *)it.xsc;
			it.job.dst_key = dstKey(it.uid, it.di);
			it.job.dst_fd = dup(xscn->images[it.di].handle);
			jobs.push_back(it.job); // copy() now owns both fds
			it.job.src_fd = -1;
			it.job.dst_fd = -1;
		}
		int64_t gpu_ns = 0;
		if (ok) {
			std::vector<OvrnVkWait::CopyWait> waits;
			if (sync_state == 1) {
				for (int k = 0; k < p.nWaits; k++) {
					waits.push_back({p.waits[k].sem, p.waits[k].value});
				}
			}
			if (test_copy) {
				for (OvrnVkWait::CopyJob &j : jobs) { // tests: no GPU, the fds aren't images
					close(j.src_fd);
					close(j.dst_fd);
				}
				ok = test_copy();
			} else {
				ok = m_vk.copy(jobs, waits, (uint64_t)m_periodNs, &gpu_ns) == OvrnVkWait::kReached;
			}
		} else {
			for (OvrnVkWait::CopyJob &j : jobs) { // not handed to copy(): close both (audit 7)
				close(j.src_fd);
				close(j.dst_fd);
			}
		}
		for (Item &it : items) {
			if (it.job.src_fd >= 0) {
				close(it.job.src_fd);
			}
			if (it.acquired) {
				xrt_swapchain_release_image(it.xsc, it.di);
				if (ok) {
					p.dstIndex[it.l][it.e] = (int)it.di;
				}
			}
		}
		std::lock_guard<std::mutex> lock(m_mutex);
		for (Item &it : items) {
			auto st = m_sets.find(it.first);
			if (st == m_sets.end() || st->second.uid != it.uid) {
				// The set was destroyed during the copy: its images were re-cached under its keys (audit 4).
				std::vector<uint64_t> dst;
				for (uint32_t i = 0; i < 3; i++) {
					dst.push_back(dstKey(it.uid, i));
				}
				m_vk.forgetImages({it.job.src_key}, dst);
				ok = false;
			} else if (ok && it.l == 0 && it.e == 0) {
				m_lastDstKey = dstKey(it.uid, it.di);
				m_lastSrcKey = it.job.src_key;
			}
			std::lock_guard<std::mutex> comp_lock(m_compMutex);
			xrt_swapchain_reference(&it.xsc, NULL);
		}
		if (ok) {
			m_copyUs.push_back(gpu_ns / 1e3);
		}
		return ok;
	}
	static constexpr int kHist = 16;
	struct Hist
	{
		int64_t target = 0; //!< display time it was predicted for
		double p[3] = {0, 0, 0};
		double q[4] = {0, 0, 0, 1};
	};
	Hist m_hist[kHist];                  //!< the last frame poses handed to SteamVR (driver space)
	int m_histPos = 0;
	std::atomic<int64_t> m_delayNs{0};   //!< SteamVR's pipeline delay, added to the pose target
	std::atomic<int64_t> m_targetNs{0};  //!< displayTarget() of the current frame
	std::atomic<int64_t> m_gridAnchorNs{0}; //!< a recent Monado display time (its vblank grid)
	std::atomic<int64_t> m_gridPeriodNs{0}; //!< Monado's display period
	std::atomic<int64_t> m_lastVsyncNs{0};  //!< the last vsync declared to SteamVR

public:
	//! Declared vsync = Monado display time - this: SteamVR's frame must reach Monado before its compositor starts
	//! (~7 ms before display, measured 2026-11-18) after SteamVR composes (~2.2 ms after the event) and the GPU wait
	//! and copy (~2.3 ms). OVRN_DIRECT_MODE_VSYNC_LEAD_MS.
	int64_t vsync_lead_ns = 12000000;
	//! "Running start": how long before the declared vsync the pose and VsyncEvent are sent (ALVR: 2 ms).
	int64_t running_start_ns = 2000000;
	//! The hand-over thread waits for a new SteamVR frame until this long before Monado's display time.
	int64_t handoff_margin_ns = 8000000;
	//! PostPresent holds until the next declared vsync (OVRN_DIRECT_MODE_HOLD, default on).
	bool hold = true;
	//! OVRN_DIRECT_MODE_LAYERS: hand Monado only the first this many of SteamVR's layers (1: the scene only, no
	//! dashboard/overlay layer; for isolating the overlay).
	int max_layers = kMaxLayers;
	//! Tests only: delay the hand-over thread per frame.
	int64_t test_handover_delay_ns = 0;
	//! Tests only: stands in for the GPU copy into Monado's swapchains (copy mode without a GPU); returns whether the
	//! copy finished. Set before start().
	std::function<bool()> test_copy;

private:
	int m_labelFixed = 0, m_labelUnknown = 0;
	std::vector<double> m_delayErrMs;
	std::vector<double> m_recentErrMs; //!< tests: the last 30 shown - pose time errors
	std::vector<int64_t> m_steerErrs;  //!< errors since the last delay correction

public:
	//! OVRN_DIRECT_MODE_LABEL_FIX (default on): give Monado the pose an application's image was drawn with.
	bool label_fix = true;

private:
	/*!
	 * Which of the last frame poses @p world is (SteamVR passes our poses through unchanged, apart from float rounding
	 * in its 3x4 matrix: ~4e-8 in the quaternion dot product), -1 if none. The closest one within float tolerance.
	 */
	int
	findHistLocked(const struct xrt_pose &world) const
	{
		int best = -1;
		double best_score = 1e30;
		for (int n = 1; n <= kHist; n++) {
			const int k = (m_histPos + kHist - n) % kHist;
			const Hist &h = m_hist[k];
			if (h.target == 0) {
				continue;
			}
			const double dot = fabs(h.q[0] * world.orientation.x + h.q[1] * world.orientation.y +
			                        h.q[2] * world.orientation.z + h.q[3] * world.orientation.w);
			const double dx = h.p[0] - world.position.x, dy = h.p[1] - world.position.y,
			             dz = h.p[2] - world.position.z;
			const double d2 = dx * dx + dy * dy + dz * dz;
			if (1.0 - dot < 1e-6 && d2 < 1e-10) {
				const double score = (1.0 - dot) + d2 * 1e4;
				if (score < best_score) {
					best_score = score;
					best = k;
				}
			}
		}
		return best;
	}

	//! vrcompositor's own images keep SteamVR's label (it draws them itself, with that pose).
	static bool
	isCompositorPid(uint32_t pid)
	{
		char path[64], comm[64] = {};
		snprintf(path, sizeof(path), "/proc/%u/comm", pid);
		FILE *f = fopen(path, "r");
		if (f == NULL) {
			return false;
		}
		const bool ok = fgets(comm, sizeof(comm), f) != NULL;
		fclose(f);
		return ok && strncmp(comm, "vrcompositor", 12) == 0;
	}
};
