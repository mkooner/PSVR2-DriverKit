// Copyright 2026, Bnuuy Solutions (PSVR2Toolkit, MIT License)
// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: MIT
/*!
 * @file
 * @brief  SteamVR interfaces that are NOT in Valve's published openvr_driver.h (checked against OpenVR SDK 2.15.6):
 *         the block queue SteamVR's camera view reads frames from ("/lighthouse/camera/raw_frames") and the path
 *         properties written on each block. Declarations as PSVR2Toolkit uses them with SteamVR 2.x
 *         (projects/psvr2_openvr_driver_ex/openvr_driver_internal.h, MIT); used only when SteamVR hands them out
 *         (GetGenericInterface returns null otherwise), next to the published IVRCameraComponent path.
 * @ingroup st_ovrn
 */

#pragma once

#include "openvr_driver.h"

namespace vr {
namespace internal {

enum EBlockQueueError
{
	BlockQueueError_None = 0,
	BlockQueueError_QueueAlreadyExists = 1,
	BlockQueueError_QueueNotFound = 2,
	BlockQueueError_BlockNotAvailable = 3,
	BlockQueueError_InvalidHandle = 4,
	BlockQueueError_InvalidParam = 5,
	BlockQueueError_ParamMismatch = 6,
	BlockQueueError_InternalError = 7,
	BlockQueueError_AlreadyInitialized = 8,
	BlockQueueError_OperationIsServerOnly = 9,
	BlockQueueError_TooManyConnections = 10,
};

enum EBlockQueueReadType
{
	BlockQueueRead_Latest = 0,
	BlockQueueRead_New = 1,
	BlockQueueRead_Next = 2,
};

class IVRBlockQueue
{
public:
	virtual EBlockQueueError
	Create(PropertyContainerHandle_t *pulQueueHandle,
	       const char *pchPath,
	       uint32_t unBlockDataSize,
	       uint32_t unBlockHeaderSize,
	       uint32_t unBlockCount,
	       uint32_t unFlags = 0) = 0;
	virtual EBlockQueueError
	Connect(PropertyContainerHandle_t *pulQueueHandle, const char *pchPath) = 0;
	virtual EBlockQueueError
	Destroy(PropertyContainerHandle_t ulQueueHandle) = 0;
	virtual EBlockQueueError
	AcquireWriteOnlyBlock(PropertyContainerHandle_t ulQueueHandle,
	                      PropertyContainerHandle_t *pulBlockHandle,
	                      void **ppvBuffer) = 0;
	virtual EBlockQueueError
	ReleaseWriteOnlyBlock(PropertyContainerHandle_t ulQueueHandle, PropertyContainerHandle_t ulBlockHandle) = 0;
	virtual EBlockQueueError
	WaitAndAcquireReadOnlyBlock(PropertyContainerHandle_t ulQueueHandle,
	                            PropertyContainerHandle_t *pulBlockHandle,
	                            void **ppvBuffer,
	                            EBlockQueueReadType eReadType,
	                            uint32_t unTimeoutMs) = 0;
	virtual EBlockQueueError
	AcquireReadOnlyBlock(PropertyContainerHandle_t ulQueueHandle,
	                     PropertyContainerHandle_t *pulBlockHandle,
	                     void **ppvBuffer,
	                     EBlockQueueReadType eReadType) = 0;
	virtual EBlockQueueError
	ReleaseReadOnlyBlock(PropertyContainerHandle_t ulQueueHandle, PropertyContainerHandle_t ulBlockHandle) = 0;
	virtual EBlockQueueError
	QueueHasReader(PropertyContainerHandle_t ulQueueHandle, bool *pbHasReaders) = 0;
};

static const char *const IVRBlockQueue_Version = "IVRBlockQueue_005";

typedef uint64_t PathHandle_t;

struct PathWrite_t
{
	PathHandle_t ulPath;
	EPropertyWriteType writeType;
	ETrackedPropertyError eSetError;
	void *pvBuffer;
	uint32_t unBufferSize;
	PropertyTypeTag_t unTag;
	ETrackedPropertyError eError;
	char *pszPath;
};

struct PathRead_t
{
	PathHandle_t ulPath;
	void *pvBuffer;
	uint32_t unBufferSize;
	PropertyTypeTag_t unTag;
	uint32_t unRequiredBufferSize;
	ETrackedPropertyError eError;
	char *pszPath;
};

class IVRPaths
{
public:
	virtual ETrackedPropertyError
	ReadPathBatch(PropertyContainerHandle_t ulRootHandle, PathRead_t *pBatch, uint32_t unBatchEntryCount) = 0;
	virtual ETrackedPropertyError
	WritePathBatch(PropertyContainerHandle_t ulRootHandle, PathWrite_t *pBatch, uint32_t unBatchEntryCount) = 0;
	virtual ETrackedPropertyError
	StringToHandle(PathHandle_t *pHandle, const char *pchPath) = 0;
	virtual ETrackedPropertyError
	HandleToString(PathHandle_t pHandle, char *pchBuffer, uint32_t unBufferSize, uint32_t *punBufferSizeUsed) = 0;
};

static const char *const IVRPaths_Version = "IVRPaths_001";

template <typename T> struct PathTag;
template <> struct PathTag<uint64_t>
{
	static constexpr PropertyTypeTag_t tag = k_unUint64PropertyTag;
};
template <> struct PathTag<int32_t>
{
	static constexpr PropertyTypeTag_t tag = k_unInt32PropertyTag;
};
template <> struct PathTag<double>
{
	static constexpr PropertyTypeTag_t tag = k_unDoublePropertyTag;
};

//! Write one path property (e.g. "/frame_sequence") on a block or queue.
template <typename T>
inline void
write_path(IVRPaths *paths, PropertyContainerHandle_t container, const char *path, T value)
{
	if (paths == nullptr || container == k_ulInvalidPropertyContainer) {
		return;
	}
	PathHandle_t handle;
	if (paths->StringToHandle(&handle, path) != TrackedProp_Success) {
		return;
	}
	PathWrite_t write = {};
	write.ulPath = handle;
	write.writeType = PropertyWrite_Set;
	write.pvBuffer = &value;
	write.unBufferSize = sizeof(T);
	write.unTag = PathTag<T>::tag;
	paths->WritePathBatch(container, &write, 1);
}

} // namespace internal
} // namespace vr
