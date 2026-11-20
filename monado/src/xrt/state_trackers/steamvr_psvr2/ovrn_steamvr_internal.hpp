// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Two SteamVR 2.x interfaces Valve doesn't publish in openvr_driver.h, as far as the passthrough needs them:
 *         the frame queue SteamVR's camera view (Room View) reads ("/lighthouse/camera/raw_frames") and the path
 *         properties that describe the queue and each frame on it.
 *
 * The layouts (function order, argument and struct layout, version names) were referenced from PSVR2Toolkit's
 * implementation (github.com/BnuuySolutions/PSVR2Toolkit, MIT); the function order also matches Monado's own
 * declarations in drivers/steamvr_lh/interfaces (blockqueue.hpp, paths.hpp). This file is our own: only the calls our camera code
 * makes are named; the other functions stay as placeholders so every call lands in the slot SteamVR's object has
 * for it. Both version names are in SteamVR's vrserver. SteamVR returns null for them when it doesn't offer them;
 * the camera code then stays on the published IVRCameraComponent.
 * @ingroup st_ovrn
 */

#pragma once

#include "openvr_driver.h"

#include <cstddef>
#include <cstdint>

namespace ovrn::steamvr {

//! What SteamVR's frame queue calls return; 0 is success, the other values only mean a failure to us.
enum class QueueResult : int32_t
{
	Ok = 0,
};

/*!
 * The frame queue (version "IVRBlockQueue_005"): the driver creates a named queue of fixed-size blocks, fills one
 * block per camera frame and hands it over; SteamVR's camera view reads them.
 */
class FrameQueue
{
public:
	//! New queue at @p path with @p block_count blocks of @p data_bytes (+ @p header_bytes each).
	virtual QueueResult
	Create(vr::PropertyContainerHandle_t *out_queue,
	       const char *path,
	       uint32_t data_bytes,
	       uint32_t header_bytes,
	       uint32_t block_count,
	       uint32_t flags = 0) = 0;
	virtual void
	unused_connect() = 0;
	virtual QueueResult
	Destroy(vr::PropertyContainerHandle_t queue) = 0;
	//! A free block to write a frame into; @p out_buffer points at its data.
	virtual QueueResult
	AcquireWriteOnlyBlock(vr::PropertyContainerHandle_t queue,
	                      vr::PropertyContainerHandle_t *out_block,
	                      void **out_buffer) = 0;
	//! Hands the written block to the readers.
	virtual QueueResult
	ReleaseWriteOnlyBlock(vr::PropertyContainerHandle_t queue, vr::PropertyContainerHandle_t block) = 0;
	virtual void
	unused_wait_and_acquire_read() = 0;
	virtual void
	unused_acquire_read() = 0;
	virtual void
	unused_release_read() = 0;
	//! Whether anything (SteamVR's camera view) reads the queue now.
	virtual QueueResult
	QueueHasReader(vr::PropertyContainerHandle_t queue, bool *out_has_reader) = 0;
};

inline constexpr const char *kFrameQueueVersion = "IVRBlockQueue_005";

using PathHandle = uint64_t;

//! One property write for Paths::WritePathBatch (SteamVR's layout: 48 bytes on 64-bit, checked below).
struct PathWrite
{
	PathHandle path;
	vr::EPropertyWriteType write_type;
	vr::ETrackedPropertyError set_error;
	void *buffer;
	uint32_t buffer_bytes;
	vr::PropertyTypeTag_t tag;
	vr::ETrackedPropertyError error;
	char *path_string;
};

static_assert(sizeof(void *) != 8 || sizeof(PathWrite) == 48, "PathWrite must match SteamVR's layout");
static_assert(sizeof(void *) != 8 || offsetof(PathWrite, buffer) == 16, "PathWrite must match SteamVR's layout");
static_assert(sizeof(void *) != 8 || offsetof(PathWrite, path_string) == 40, "PathWrite must match SteamVR's layout");

/*!
 * Path properties (version "IVRPaths_001"): values such as "/width" or "/frame_sequence" set on a queue or a block,
 * by path name.
 */
class Paths
{
public:
	virtual void
	unused_read_path_batch() = 0;
	virtual vr::ETrackedPropertyError
	WritePathBatch(vr::PropertyContainerHandle_t container, PathWrite *writes, uint32_t count) = 0;
	virtual vr::ETrackedPropertyError
	StringToHandle(PathHandle *out_handle, const char *path) = 0;
	virtual void
	unused_handle_to_string() = 0;
};

inline constexpr const char *kPathsVersion = "IVRPaths_001";

namespace detail {
inline vr::PropertyTypeTag_t
tag_of(uint64_t)
{
	return vr::k_unUint64PropertyTag;
}
inline vr::PropertyTypeTag_t
tag_of(int32_t)
{
	return vr::k_unInt32PropertyTag;
}
inline vr::PropertyTypeTag_t
tag_of(double)
{
	return vr::k_unDoublePropertyTag;
}
} // namespace detail

//! Sets one path property (e.g. "/frame_sequence") on a queue or block; silently nothing without the interface.
template <typename T>
inline void
set_path(Paths *paths, vr::PropertyContainerHandle_t container, const char *path, T value)
{
	if (paths == nullptr || container == vr::k_ulInvalidPropertyContainer) {
		return;
	}
	PathHandle handle = 0;
	if (paths->StringToHandle(&handle, path) != vr::TrackedProp_Success) {
		return;
	}
	PathWrite w = {};
	w.path = handle;
	w.write_type = vr::PropertyWrite_Set;
	w.buffer = &value;
	w.buffer_bytes = sizeof(T);
	w.tag = detail::tag_of(value);
	paths->WritePathBatch(container, &w, 1);
}

} // namespace ovrn::steamvr
