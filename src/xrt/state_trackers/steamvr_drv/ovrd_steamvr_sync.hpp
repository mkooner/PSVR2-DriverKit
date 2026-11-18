// Copyright 2026, fbinerd (ALVR PR #3425, alvr/server_openvr/cpp/platform/linux/SteamVrSync.h, MIT)
// Copyright 2026, Mehrab Kooner (adapted for the Monado SteamVR plugin)
// SPDX-License-Identifier: MIT
/*!
 * @file
 * @brief  Finds the GPU signal that marks the end of SteamVR's write into one of its shared textures, so driver direct
 *         mode (ovrd_direct_mode.hpp) hands a frame to Monado's compositor only once it is finished. Linux only.
 *
 * Why: on NVIDIA's Linux driver the dma-buf of a SteamVR texture carries no implicit fence, so a reader can't tell
 * when SteamVR's compositor (or the application) finished drawing it; reading too early shows part of the previous
 * frame (ALVR PR #3425: "the right eye ... on the previous frame ... and on the next one when SteamVR's next write
 * overlapped the read"). Steam Link's own driver (driver_vrlink.so) waits on a per-texture timeline semaphore; this
 * reaches the same semaphore.
 *
 * UNDOCUMENTED STEAMVR INTERFACES (not in openvr_driver.h; from ALVR's analysis of driver_vrlink.so, SteamVR 2.17.x;
 * checked here on SteamVR 2.19.3, 2026-11-18: names, +0x40 link and timeline values all as described):
 *  1. Every shared resource (image or semaphore) has a 0x48-byte state in POSIX shared memory, named
 *     "/u<uid>-Shm_<crc32 hex>" of "VR_CompositorResourceState_<handle hex>_0_<vrserver pid>".
 *  2. A driver runs inside vrserver, so getpid() is that pid.
 *  3. Image state +0x40: the handle of the image's timeline semaphore (0 until SteamVR associated one).
 *  4. Semaphore state +0x00: the timeline value of the last write submitted on its image; read at Present it is the
 *     value that write will signal.
 * The semaphore fd itself comes through the public IVRIPCResourceManagerClient (RefResource + ReceiveSharedFd), as an
 * OPAQUE_FD timeline semaphore. A SteamVR update may change any of this; every failure degrades to "no semaphore" and
 * the caller then hands the frame over unsynchronized, as before.
 */

#pragma once

#include "openvr_driver.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <map>
#include <mutex>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

namespace ovrd_steamvr_sync {

constexpr size_t kStateSize = 0x48;
constexpr size_t kImageSemaphoreOffset = 0x40;

//! CRC-32 (zlib, IEEE) as SteamVR's shared object manager names its memory with.
inline uint32_t
crc32(const char *data, size_t length)
{
	uint32_t crc = 0xffffffffu;
	for (size_t i = 0; i < length; ++i) {
		crc ^= (uint8_t)data[i];
		for (int bit = 0; bit < 8; ++bit) {
			crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
		}
	}
	return ~crc;
}

//! The shared memory name of a resource's state (for tests).
inline void
state_path(uint64_t handle, unsigned pid, unsigned uid, char *out, size_t out_size)
{
	char name[96];
	snprintf(name, sizeof(name), "VR_CompositorResourceState_%llx_0_%u", (unsigned long long)handle, pid);
	snprintf(out, out_size, "/u%u-Shm_%x", uid, crc32(name, strlen(name)));
}

class Sync
{
public:
	//! Handle of the semaphore SteamVR associated with this texture, 0 if none (yet). Cached once found.
	uint64_t
	semaphoreOf(uint64_t texture)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		auto it = m_semaphoreOf.find(texture);
		if (it != m_semaphoreOf.end()) {
			return it->second;
		}
		const volatile uint8_t *state = stateLocked(texture);
		if (state == NULL) {
			return 0;
		}
		uint64_t sem = __atomic_load_n((const uint64_t *)(state + kImageSemaphoreOffset), __ATOMIC_ACQUIRE);
		if (sem != 0) {
			m_semaphoreOf[texture] = sem;
		}
		return sem;
	}

	//! Timeline value of the last write submitted on the semaphore's image (read at Present).
	bool
	value(uint64_t sem, uint64_t *out)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		const volatile uint8_t *state = stateLocked(sem);
		if (state == NULL) {
			return false;
		}
		*out = __atomic_load_n((const uint64_t *)state, __ATOMIC_ACQUIRE);
		return true;
	}

	//! A new fd for the semaphore (the caller owns it), -1 on failure.
	int
	dupFd(uint64_t sem)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		auto it = m_fds.find(sem);
		if (it == m_fds.end()) {
			vr::IVRIPCResourceManagerClient *mgr = vr::VRIPCResourceManager();
			if (mgr == NULL) {
				return -1;
			}
			uint64_t ipc = 0;
			int fd = -1;
			if (!mgr->RefResource(sem, &ipc)) {
				return -1;
			}
			if (!mgr->ReceiveSharedFd(ipc, &fd) || fd < 0) {
				if (fd >= 0) {
					close(fd);
				}
				mgr->UnrefResource(sem);
				return -1;
			}
			it = m_fds.emplace(sem, fd).first;
		}
		return dup(it->second);
	}

	//! Drops what is kept for these textures (SteamVR destroyed them); returns their semaphores.
	std::vector<uint64_t>
	forget(const std::vector<uint64_t> &textures)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		std::vector<uint64_t> sems;
		for (uint64_t tex : textures) {
			auto s = m_semaphoreOf.find(tex);
			if (s != m_semaphoreOf.end()) {
				sems.push_back(s->second);
				unmapLocked(s->second);
				auto fd = m_fds.find(s->second);
				if (fd != m_fds.end()) {
					close(fd->second);
					if (vr::VRIPCResourceManager() != NULL) {
						vr::VRIPCResourceManager()->UnrefResource(s->second);
					}
					m_fds.erase(fd);
				}
				m_semaphoreOf.erase(s);
			}
			unmapLocked(tex);
		}
		return sems;
	}

	void
	forgetAll()
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		for (auto &fd : m_fds) {
			close(fd.second);
			if (vr::VRIPCResourceManager() != NULL) {
				vr::VRIPCResourceManager()->UnrefResource(fd.first);
			}
		}
		m_fds.clear();
		for (auto &s : m_states) {
			munmap((void *)s.second, kStateSize);
		}
		m_states.clear();
		m_semaphoreOf.clear();
	}

private:
	std::mutex m_mutex;
	std::map<uint64_t, const volatile uint8_t *> m_states;
	std::map<uint64_t, uint64_t> m_semaphoreOf;
	std::map<uint64_t, int> m_fds;

	void
	unmapLocked(uint64_t handle)
	{
		auto it = m_states.find(handle);
		if (it != m_states.end()) {
			munmap((void *)it->second, kStateSize);
			m_states.erase(it);
		}
	}

	const volatile uint8_t *
	stateLocked(uint64_t handle)
	{
		auto it = m_states.find(handle);
		if (it != m_states.end()) {
			return it->second;
		}
		char path[64];
		state_path(handle, (unsigned)getpid(), (unsigned)getuid(), path, sizeof(path));
		int fd = shm_open(path, O_RDONLY, 0);
		if (fd < 0) {
			return NULL; // not created yet, retried next time
		}
		void *map = mmap(NULL, kStateSize, PROT_READ, MAP_SHARED, fd, 0);
		close(fd);
		if (map == MAP_FAILED) {
			return NULL;
		}
		m_states[handle] = (const volatile uint8_t *)map;
		return (const volatile uint8_t *)map;
	}
};

} // namespace ovrd_steamvr_sync
