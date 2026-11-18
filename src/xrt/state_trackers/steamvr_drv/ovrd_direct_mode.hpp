// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Driver direct mode prototype (OVRD_DIRECT_MODE=1): SteamVR's compositor hands its finished frames to the
 *         driver instead of driving the display itself. This first step only allocates the images SteamVR renders
 *         into (shared Vulkan images, as ALVR's Linux OvrDirectModeComponent does), paces SteamVR on a steady clock
 *         at the display rate and logs every frame it receives. Nothing is shown in the headset yet: the next step
 *         hands these frames to Monado's compositor.
 *
 * Log (OVRD_DIRECT_MODE_LOG, csv), host times are CLOCK_MONOTONIC ns:
 *   C,host_ns,pid,vk_format,width,height,samples,handle0,handle1,handle2,dmabuf_bytes  (CreateSwapTextureSet)
 *   S,host_ns,layer,predict_s,tx,ty,tz,qx,qy,qz,qw,tex_l,tex_r,depth_l,depth_r,umin,vmin,umax,vmax
 *   P,host_ns,sync_handle,layers                                                     (Present)
 *   Q,host_ns,frames_to_throttle,extra_frames_to_predict,woke_ns                       (PostPresent, after pacing)
 *   X,host_ns,handle  /  Y,host_ns,pid                                                 (destroy set / all of a pid)
 */

#pragma once

#include "openvr_driver.h"
#include "ovrd_log.hpp"
#include "os/os_time.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include <unistd.h>

class OvrdDirectMode : public vr::IVRDriverDirectModeComponent
{
public:
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
			if (m_log != NULL) {
				fprintf(m_log, "# direct mode, display period %lld ns\n", (long long)m_periodNs);
				ovrd_log("Direct mode: logging frames to %s\n", log_path);
			} else {
				ovrd_log("Direct mode: can't write %s\n", log_path);
			}
		}
		m_gridNs = os_monotonic_get_ns();
		m_summaryNs = m_gridNs;
		m_running = true;
		m_vsyncThread = std::thread(&OvrdDirectMode::vsyncLoop, this);
		ovrd_log("Direct mode: on, vsync every %.3f ms\n", m_periodNs / 1e6);
	}

	void
	stop()
	{
		if (m_running.exchange(false) && m_vsyncThread.joinable()) {
			m_vsyncThread.join();
		}
		std::lock_guard<std::mutex> lock(m_mutex);
		for (auto &set : m_sets) {
			for (int i = 0; i < 3; i++) {
				if (set.second.fds[i] >= 0) {
					close(set.second.fds[i]);
				}
			}
		}
		m_sets.clear();
		m_handleToSet.clear();
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
		if (m_log != NULL) {
			const vr::VRTextureBounds_t &b = perEye[0].bounds;
			fprintf(m_log, "S,%lld,%d,%.6f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%llu,%llu,%llu,%llu,%.4f,%.4f,%.4f,%.4f\n",
			        (long long)now, m_layers, perEye[0].flHmdPosePredictionTimeInSecondsFromNow, m.m[0][3],
			        m.m[1][3], m.m[2][3], q[0], q[1], q[2], q[3], (unsigned long long)perEye[0].hTexture,
			        (unsigned long long)perEye[1].hTexture, (unsigned long long)perEye[0].hDepthTexture,
			        (unsigned long long)perEye[1].hDepthTexture, b.uMin, b.vMin, b.uMax, b.vMax);
		}
		if (m_layers == 0) {
			m_predictS.push_back(perEye[0].flHmdPosePredictionTimeInSecondsFromNow);
		}
		m_layers++;
	}

	void
	Present(vr::SharedTextureHandle_t syncTexture) override
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		const int64_t now = os_monotonic_get_ns();
		if (m_log != NULL) {
			fprintf(m_log, "P,%lld,%llu,%d\n", (long long)now, (unsigned long long)syncTexture, m_layers);
		}
		if (m_lastPresentNs != 0) {
			m_presentGapsMs.push_back((now - m_lastPresentNs) / 1e6);
		}
		m_lastPresentNs = now;
		m_frames++;
		m_layerSum += m_layers;
		m_layers = 0;
		summaryLocked(now);
	}

	void
	PostPresent(const Throttling_t *throttling) override
	{
		// Hold SteamVR's compositor until the next tick of the display clock, as ALVR does: it then starts its
		// next frame one display period after the last.
		const int64_t woke = waitNextTick();
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
	};

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

	//! Tells SteamVR about each vsync of the steady display clock (Prop_DriverDirectModeSendsVsyncEvents_Bool).
	void
	vsyncLoop()
	{
		while (m_running) {
			waitNextTick();
			vr::VRServerDriverHost()->VsyncEvent(0.0);
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
		         "ms; pose predicted median %.2f (%.2f-%.2f) ms ahead; %zu image sets\n",
		         secs, m_frames, m_frames / secs, m_frames ? (double)m_layerSum / m_frames : 0.0,
		         pct(m_presentGapsMs, 0.5), pct(m_presentGapsMs, 0.99), pct(pred_ms, 0.5), pct(pred_ms, 0.01),
		         pct(pred_ms, 0.99), m_sets.size());
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
};
