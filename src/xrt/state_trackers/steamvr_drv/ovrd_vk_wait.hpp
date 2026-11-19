// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  CPU wait on SteamVR's per-texture timeline semaphores (ovrd_steamvr_sync.hpp) for driver direct mode: a
 *         minimal Vulkan device on the same GPU imports each semaphore's OPAQUE_FD once and waits for a value with
 *         vkWaitSemaphores. Also reads one shared image back to a file (capture(), diagnostics only).
 */

#pragma once

#include <vulkan/vulkan.h>

#include "os/os_time.h"

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <unistd.h>
#include <vector>
#include <cstdio>
#include <cstring>

class OvrdVkWait
{
public:
	enum Result
	{
		kReached = 0,  //!< the value was signalled (or already was)
		kTimeout = 1,  //!< not signalled within the timeout
		kError = 2,    //!< import or wait failed
	};

	~OvrdVkWait()
	{
		destroy();
	}

	//! Creates the device on the GPU with this vendor (0: the first discrete one). Returns false with a reason.
	bool
	init(uint32_t vendor_id, std::string *why)
	{
		VkApplicationInfo app = {};
		app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
		app.pApplicationName = "monado steamvr direct mode wait";
		app.apiVersion = VK_API_VERSION_1_2;
		VkInstanceCreateInfo ici = {};
		ici.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
		ici.pApplicationInfo = &app;
		if (vkCreateInstance(&ici, NULL, &m_inst) != VK_SUCCESS) {
			*why = "vkCreateInstance failed";
			return false;
		}
		uint32_t n = 0;
		vkEnumeratePhysicalDevices(m_inst, &n, NULL);
		std::vector<VkPhysicalDevice> devs(n);
		vkEnumeratePhysicalDevices(m_inst, &n, devs.data());
		VkPhysicalDevice phys = VK_NULL_HANDLE;
		for (VkPhysicalDevice d : devs) {
			VkPhysicalDeviceProperties p;
			vkGetPhysicalDeviceProperties(d, &p);
			bool match = vendor_id != 0 ? p.vendorID == vendor_id
			                            : p.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU;
			if (match && p.apiVersion >= VK_API_VERSION_1_2) {
				phys = d;
				m_gpuName = p.deviceName;
				break;
			}
		}
		if (phys == VK_NULL_HANDLE) {
			*why = "no matching Vulkan 1.2 GPU";
			return false;
		}
		float prio = 1.0f;
		VkDeviceQueueCreateInfo qci = {};
		qci.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
		qci.queueFamilyIndex = 0;
		qci.queueCount = 1;
		qci.pQueuePriorities = &prio;
		VkPhysicalDeviceVulkan12Features f12 = {};
		f12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
		f12.timelineSemaphore = VK_TRUE;
		const char *exts[] = {VK_KHR_EXTERNAL_SEMAPHORE_FD_EXTENSION_NAME, VK_KHR_EXTERNAL_MEMORY_FD_EXTENSION_NAME};
		VkDeviceCreateInfo dci = {};
		dci.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
		dci.pNext = &f12;
		dci.queueCreateInfoCount = 1;
		dci.pQueueCreateInfos = &qci;
		dci.enabledExtensionCount = 2;
		dci.ppEnabledExtensionNames = exts;
		if (vkCreateDevice(phys, &dci, NULL, &m_dev) != VK_SUCCESS) {
			*why = "vkCreateDevice failed (timeline semaphores / external semaphore fd)";
			return false;
		}
		m_phys = phys;
		vkGetDeviceQueue(m_dev, 0, 0, &m_queue);
		m_import = (PFN_vkImportSemaphoreFdKHR)vkGetDeviceProcAddr(m_dev, "vkImportSemaphoreFdKHR");
		m_wait = (PFN_vkWaitSemaphores)vkGetDeviceProcAddr(m_dev, "vkWaitSemaphores");
		m_getValue = (PFN_vkGetSemaphoreCounterValue)vkGetDeviceProcAddr(m_dev, "vkGetSemaphoreCounterValue");
		if (m_import == NULL || m_wait == NULL || m_getValue == NULL) {
			*why = "missing vkImportSemaphoreFdKHR / vkWaitSemaphores";
			return false;
		}
		return true;
	}

	/*!
	 * Reads a finished shared image (opaque fd from SteamVR's IPC resource manager, consumed) back and writes it to
	 * @p path as raw RGBA bytes with a one-line text header "w h vkformat". Diagnostics only: slow, blocks.
	 * The image is imported with the flags our CreateSwapTextureSet asks SteamVR for (+ colour attachment).
	 */
	bool
	capture(int fd, uint32_t w, uint32_t h, uint32_t format, const char *path, std::string *why)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		VkExternalMemoryImageCreateInfo ext = {};
		ext.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO;
		ext.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
		VkImageCreateInfo ici = {};
		ici.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
		ici.pNext = &ext;
		ici.flags = VK_IMAGE_CREATE_MUTABLE_FORMAT_BIT;
		ici.imageType = VK_IMAGE_TYPE_2D;
		ici.format = (VkFormat)format;
		ici.extent = {w, h, 1};
		ici.mipLevels = 1;
		ici.arrayLayers = 1;
		ici.samples = VK_SAMPLE_COUNT_1_BIT;
		ici.tiling = VK_IMAGE_TILING_OPTIMAL;
		ici.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT |
		            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
		ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		VkImage image = VK_NULL_HANDLE;
		VkDeviceMemory mem = VK_NULL_HANDLE, bmem = VK_NULL_HANDLE;
		VkBuffer buf = VK_NULL_HANDLE;
		VkCommandPool pool = VK_NULL_HANDLE;
		bool ok = false;
		do {
			if (vkCreateImage(m_dev, &ici, NULL, &image) != VK_SUCCESS) {
				*why = "vkCreateImage";
				break;
			}
			VkMemoryRequirements req;
			vkGetImageMemoryRequirements(m_dev, image, &req);
			VkImportMemoryFdInfoKHR imp = {};
			imp.sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_FD_INFO_KHR;
			imp.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
			imp.fd = fd;
			VkMemoryDedicatedAllocateInfo ded = {};
			ded.sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO;
			ded.pNext = &imp;
			ded.image = image;
			VkMemoryAllocateInfo mai = {};
			mai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
			mai.pNext = &ded;
			mai.allocationSize = req.size;
			mai.memoryTypeIndex = memoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
			if (vkAllocateMemory(m_dev, &mai, NULL, &mem) != VK_SUCCESS) {
				*why = "import memory";
				break;
			}
			fd = -1; // consumed
			vkBindImageMemory(m_dev, image, mem, 0);
			const VkDeviceSize size = (VkDeviceSize)w * h * 4;
			VkBufferCreateInfo bci = {};
			bci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
			bci.size = size;
			bci.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
			if (vkCreateBuffer(m_dev, &bci, NULL, &buf) != VK_SUCCESS) {
				*why = "vkCreateBuffer";
				break;
			}
			VkMemoryRequirements breq;
			vkGetBufferMemoryRequirements(m_dev, buf, &breq);
			VkMemoryAllocateInfo bai = {};
			bai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
			bai.allocationSize = breq.size;
			bai.memoryTypeIndex = memoryType(breq.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
			                                                          VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
			if (vkAllocateMemory(m_dev, &bai, NULL, &bmem) != VK_SUCCESS) {
				*why = "buffer memory";
				break;
			}
			vkBindBufferMemory(m_dev, buf, bmem, 0);
			VkCommandPoolCreateInfo pci = {};
			pci.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
			pci.queueFamilyIndex = 0;
			vkCreateCommandPool(m_dev, &pci, NULL, &pool);
			VkCommandBufferAllocateInfo cai = {};
			cai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
			cai.commandPool = pool;
			cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
			cai.commandBufferCount = 1;
			VkCommandBuffer cb;
			vkAllocateCommandBuffers(m_dev, &cai, &cb);
			VkCommandBufferBeginInfo cbi = {};
			cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
			vkBeginCommandBuffer(cb, &cbi);
			// As ASVR: acquire from the external owner, GENERAL as the superset of what the writer left.
			VkImageMemoryBarrier b = {};
			b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
			b.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
			b.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
			b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
			b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_EXTERNAL;
			b.dstQueueFamilyIndex = 0;
			b.image = image;
			b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
			vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, NULL,
			                     0, NULL, 1, &b);
			VkBufferImageCopy reg = {};
			reg.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
			reg.imageExtent = {w, h, 1};
			vkCmdCopyImageToBuffer(cb, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, buf, 1, &reg);
			// Release back to the external owner in the layout it had.
			b.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
			b.dstAccessMask = 0;
			b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
			b.newLayout = VK_IMAGE_LAYOUT_GENERAL;
			b.srcQueueFamilyIndex = 0;
			b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_EXTERNAL;
			vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, 0, 0,
			                     NULL, 0, NULL, 1, &b);
			vkEndCommandBuffer(cb);
			VkSubmitInfo si = {};
			si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
			si.commandBufferCount = 1;
			si.pCommandBuffers = &cb;
			if (vkQueueSubmit(m_queue, 1, &si, VK_NULL_HANDLE) != VK_SUCCESS || vkQueueWaitIdle(m_queue) != VK_SUCCESS) {
				*why = "submit";
				break;
			}
			void *data = NULL;
			vkMapMemory(m_dev, bmem, 0, size, 0, &data);
			FILE *f = fopen(path, "wb");
			if (f != NULL) {
				fprintf(f, "%u %u %u\n", w, h, format);
				fwrite(data, 1, (size_t)size, f);
				fclose(f);
				ok = true;
			} else {
				*why = "can't write the file";
			}
			vkUnmapMemory(m_dev, bmem);
		} while (false);
		if (fd >= 0) {
			close(fd);
		}
		if (pool != VK_NULL_HANDLE) {
			vkDestroyCommandPool(m_dev, pool, NULL);
		}
		if (buf != VK_NULL_HANDLE) {
			vkDestroyBuffer(m_dev, buf, NULL);
		}
		if (bmem != VK_NULL_HANDLE) {
			vkFreeMemory(m_dev, bmem, NULL);
		}
		if (image != VK_NULL_HANDLE) {
			vkDestroyImage(m_dev, image, NULL);
		}
		if (mem != VK_NULL_HANDLE) {
			vkFreeMemory(m_dev, mem, NULL);
		}
		return ok;
	}

	const std::string &
	gpuName() const
	{
		return m_gpuName;
	}

	bool
	ready() const
	{
		return m_dev != VK_NULL_HANDLE && m_import != NULL;
	}

	/*!
	 * Waits until the semaphore @p key (fd from @p get_fd on first use; the fd is consumed by the import) reaches
	 * @p value.
	 */
	template <typename GetFd>
	Result
	wait(uint64_t key, uint64_t value, uint64_t timeout_ns, GetFd get_fd)
	{
		VkSemaphore sem = VK_NULL_HANDLE;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			sem = semaphoreLocked(key, get_fd);
			if (sem == VK_NULL_HANDLE) {
				return kError;
			}
		}
		VkSemaphoreWaitInfo wi = {};
		wi.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
		wi.semaphoreCount = 1;
		wi.pSemaphores = &sem;
		wi.pValues = &value;
		VkResult r = m_wait(m_dev, &wi, timeout_ns);
		return r == VK_SUCCESS ? kReached : r == VK_TIMEOUT ? kTimeout : kError;
	}

	//! One image copy of copy(): SteamVR's image (src) into a Monado swapchain image (dst), same size and format.
	struct CopyJob
	{
		uint64_t src_key;
		int src_fd; //!< a dup, consumed (or closed) by copy(); only used the first time src_key is seen
		uint64_t dst_key;
		int dst_fd; //!< a dup, same
		uint32_t w, h, format;
	};
	//! A write copy() waits for on the GPU (already reached on the CPU: this only makes it visible to our reads).
	struct CopyWait
	{
		uint64_t key;
		uint64_t value;
	};

	/*!
	 * Copies SteamVR's finished images into Monado's swapchain images with the synchronisation the hardware needs:
	 * the submission waits on the writes' timeline semaphores (memory dependency), each source image is acquired
	 * from the external owner (SteamVR, GENERAL) and released back, and each destination is released to Monado's
	 * compositor in SHADER_READ_ONLY_OPTIMAL as Monado's own Vulkan clients do (comp_vk_client.c). Blocks until the
	 * GPU finished (bounded by @p timeout_ns); @p gpu_ns gets the time. Semaphores must already be imported (wait()).
	 */
	Result
	copy(const std::vector<CopyJob> &jobs, const std::vector<CopyWait> &waits, uint64_t timeout_ns, int64_t *gpu_ns)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		const int64_t t0 = (int64_t)os_monotonic_get_ns();
		*gpu_ns = 0;
		// Every fd handed in is consumed or closed here, whatever happens (audit 7).
		std::vector<int> fds_left;
		for (const CopyJob &j : jobs) {
			fds_left.push_back(j.src_fd);
			fds_left.push_back(j.dst_fd);
		}
		auto close_left = [&](size_t from_fd) {
			for (size_t k = from_fd; k < fds_left.size(); k++) {
				if (fds_left[k] >= 0) {
					close(fds_left[k]);
				}
			}
		};
		if (m_busy) {
			// The last copy's GPU work outlived its wait: don't reset the pool under it (audit 11).
			if (vkWaitForFences(m_dev, 1, &m_fence, VK_TRUE, timeout_ns) != VK_SUCCESS) {
				close_left(0);
				return kError;
			}
			m_busy = false;
		}
		if (m_pool == VK_NULL_HANDLE) {
			VkCommandPoolCreateInfo pci = {};
			pci.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
			pci.queueFamilyIndex = 0;
			VkFenceCreateInfo fci = {};
			fci.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
			if (vkCreateCommandPool(m_dev, &pci, NULL, &m_pool) != VK_SUCCESS ||
			    vkCreateFence(m_dev, &fci, NULL, &m_fence) != VK_SUCCESS) {
				close_left(0);
				return kError;
			}
		}
		std::vector<VkImage> src(jobs.size()), dst(jobs.size());
		for (size_t i = 0; i < jobs.size(); i++) {
			const CopyJob &j = jobs[i];
			// imageLocked consumes or closes the fd it is given
			src[i] = imageLocked(m_srcImgs, j.src_key, j.src_fd, j.w, j.h, j.format, false);
			fds_left[2 * i] = -1;
			dst[i] = imageLocked(m_dstImgs, j.dst_key, j.dst_fd, j.w, j.h, j.format, true);
			fds_left[2 * i + 1] = -1;
			if (src[i] == VK_NULL_HANDLE || dst[i] == VK_NULL_HANDLE) {
				close_left(2 * i + 2);
				return kError;
			}
		}
		vkResetCommandPool(m_dev, m_pool, 0);
		VkCommandBufferAllocateInfo cai = {};
		cai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		cai.commandPool = m_pool;
		cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		cai.commandBufferCount = 1;
		VkCommandBuffer cb = VK_NULL_HANDLE;
		if (vkAllocateCommandBuffers(m_dev, &cai, &cb) != VK_SUCCESS) {
			return kError;
		}
		VkCommandBufferBeginInfo cbi = {};
		cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
		cbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
		vkBeginCommandBuffer(cb, &cbi);
		const VkImageSubresourceRange range = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
		std::vector<VkImageMemoryBarrier> in, out;
		for (size_t i = 0; i < jobs.size(); i++) {
			VkImageMemoryBarrier b = {};
			b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
			b.subresourceRange = range;
			// SteamVR's image: acquire from the external owner, GENERAL as the superset of what the writer left.
			b.srcAccessMask = 0;
			b.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
			b.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
			b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
			b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_EXTERNAL;
			b.dstQueueFamilyIndex = 0;
			b.image = src[i];
			in.push_back(b);
			// Monado's image: overwritten whole, as comp_vk_client's acquire (UNDEFINED, no ownership transfer).
			b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
			b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
			b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
			b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
			b.image = dst[i];
			in.push_back(b);
			// Back: SteamVR's image to its owner in GENERAL; Monado's to its compositor in SHADER_READ_ONLY_OPTIMAL.
			b.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
			b.dstAccessMask = 0;
			b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
			b.newLayout = VK_IMAGE_LAYOUT_GENERAL;
			b.srcQueueFamilyIndex = 0;
			b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_EXTERNAL;
			b.image = src[i];
			out.push_back(b);
			b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
			b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
			b.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
			b.image = dst[i];
			out.push_back(b);
		}
		vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, NULL, 0,
		                     NULL, (uint32_t)in.size(), in.data());
		for (size_t i = 0; i < jobs.size(); i++) {
			VkImageCopy c = {};
			c.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
			c.dstSubresource = c.srcSubresource;
			c.extent = {jobs[i].w, jobs[i].h, 1};
			vkCmdCopyImage(cb, src[i], VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, dst[i],
			               VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &c);
		}
		vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, 0, 0, NULL, 0,
		                     NULL, (uint32_t)out.size(), out.data());
		vkEndCommandBuffer(cb);
		std::vector<VkSemaphore> sems;
		std::vector<uint64_t> values;
		std::vector<VkPipelineStageFlags> stages;
		for (const CopyWait &w : waits) {
			auto it = m_sems.find(w.key);
			if (it == m_sems.end()) {
				continue;
			}
			sems.push_back(it->second);
			values.push_back(w.value);
			stages.push_back(VK_PIPELINE_STAGE_TRANSFER_BIT);
		}
		VkTimelineSemaphoreSubmitInfo tsi = {};
		tsi.sType = VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO;
		tsi.waitSemaphoreValueCount = (uint32_t)values.size();
		tsi.pWaitSemaphoreValues = values.data();
		VkSubmitInfo si = {};
		si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
		si.pNext = &tsi;
		si.waitSemaphoreCount = (uint32_t)sems.size();
		si.pWaitSemaphores = sems.data();
		si.pWaitDstStageMask = stages.data();
		si.commandBufferCount = 1;
		si.pCommandBuffers = &cb;
		vkResetFences(m_dev, 1, &m_fence);
		if (vkQueueSubmit(m_queue, 1, &si, m_fence) != VK_SUCCESS) {
			return kError;
		}
		VkResult r = vkWaitForFences(m_dev, 1, &m_fence, VK_TRUE, timeout_ns);
		if (r == VK_TIMEOUT) {
			// The pool can't be reset while the GPU uses it: wait (only waits on writes already reached).
			r = vkWaitForFences(m_dev, 1, &m_fence, VK_TRUE, 1000000000ull);
			*gpu_ns = (int64_t)os_monotonic_get_ns() - t0;
			if (r != VK_SUCCESS) {
				m_busy = true; // still in flight: the next copy waits for it before reusing the pool
				return kError;
			}
			return kTimeout;
		}
		*gpu_ns = (int64_t)os_monotonic_get_ns() - t0;
		return r == VK_SUCCESS ? kReached : kError;
	}

	//! Drops cached images (their sets were destroyed).
	void
	forgetImages(const std::vector<uint64_t> &src_keys, const std::vector<uint64_t> &dst_keys)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		dropLocked(m_srcImgs, src_keys);
		dropLocked(m_dstImgs, dst_keys);
	}

	//! For tests: the first pixel of a copy destination (RGBA8), read back after the copies. False if unknown.
	bool
	readFirstPixel(uint64_t dst_key, uint8_t out[4])
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		auto it = m_dstImgs.find(dst_key);
		if (it == m_dstImgs.end()) {
			return false;
		}
		return readPixelLocked(it->second.image, out, false);
	}

	//! For tests: the first pixel of a copy source (SteamVR's image, as written).
	bool
	readFirstSourcePixel(uint64_t src_key, uint8_t out[4])
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		auto it = m_srcImgs.find(src_key);
		if (it == m_srcImgs.end()) {
			return false;
		}
		return readPixelLocked(it->second.image, out, true);
	}

private:
	struct Img
	{
		VkImage image = VK_NULL_HANDLE;
		VkDeviceMemory memory = VK_NULL_HANDLE;
	};

	template <typename GetFd>
	VkSemaphore
	semaphoreLocked(uint64_t key, GetFd get_fd)
	{
		VkSemaphore sem = VK_NULL_HANDLE;
		{
			auto it = m_sems.find(key);
			if (it != m_sems.end()) {
				sem = it->second;
			} else {
				int fd = get_fd();
				if (fd < 0) {
					return VK_NULL_HANDLE;
				}
				VkSemaphoreTypeCreateInfo tci = {};
		tci.sType = VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO;
				tci.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
				VkSemaphoreCreateInfo sci = {};
		sci.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
				sci.pNext = &tci;
				if (vkCreateSemaphore(m_dev, &sci, NULL, &sem) != VK_SUCCESS) {
					close(fd);
					return VK_NULL_HANDLE;
				}
				VkImportSemaphoreFdInfoKHR imp = {};
		imp.sType = VK_STRUCTURE_TYPE_IMPORT_SEMAPHORE_FD_INFO_KHR;
				imp.semaphore = sem;
				imp.handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT;
				imp.fd = fd;
				if (m_import(m_dev, &imp) != VK_SUCCESS) {
					close(fd); // not consumed on failure
					vkDestroySemaphore(m_dev, sem, NULL);
					return VK_NULL_HANDLE;
				}
				m_sems[key] = sem;
			}
		}
		return sem;
	}

	//! An imported image (cached by key; @p fd consumed when imported, else closed).
	VkImage
	imageLocked(std::map<uint64_t, Img> &cache, uint64_t key, int fd, uint32_t w, uint32_t h, uint32_t format, bool dst)
	{
		auto it = cache.find(key);
		if (it != cache.end()) {
			if (fd >= 0) {
				close(fd);
			}
			return it->second.image;
		}
		if (fd < 0) {
			return VK_NULL_HANDLE;
		}
		VkExternalMemoryImageCreateInfo ext = {};
		ext.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO;
		ext.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
		VkImageCreateInfo ici = {};
		ici.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
		ici.pNext = &ext;
		ici.flags = VK_IMAGE_CREATE_MUTABLE_FORMAT_BIT;
		ici.imageType = VK_IMAGE_TYPE_2D;
		ici.format = (VkFormat)format;
		ici.extent = {w, h, 1};
		ici.mipLevels = 1;
		ici.arrayLayers = 1;
		ici.samples = VK_SAMPLE_COUNT_1_BIT;
		ici.tiling = VK_IMAGE_TILING_OPTIMAL;
		ici.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
		            (dst ? VK_IMAGE_USAGE_TRANSFER_DST_BIT : VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT);
		ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		Img img;
		if (vkCreateImage(m_dev, &ici, NULL, &img.image) != VK_SUCCESS) {
			close(fd);
			return VK_NULL_HANDLE;
		}
		VkMemoryRequirements req;
		vkGetImageMemoryRequirements(m_dev, img.image, &req);
		VkImportMemoryFdInfoKHR imp = {};
		imp.sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_FD_INFO_KHR;
		imp.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
		imp.fd = fd;
		VkMemoryDedicatedAllocateInfo ded = {};
		ded.sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO;
		ded.pNext = &imp;
		ded.image = img.image;
		VkMemoryAllocateInfo mai = {};
		mai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		mai.pNext = &ded;
		mai.allocationSize = req.size;
		mai.memoryTypeIndex = memoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
		if (vkAllocateMemory(m_dev, &mai, NULL, &img.memory) != VK_SUCCESS) {
			close(fd);
			vkDestroyImage(m_dev, img.image, NULL);
			return VK_NULL_HANDLE;
		}
		vkBindImageMemory(m_dev, img.image, img.memory, 0);
		cache[key] = img;
		return img.image;
	}

	void
	dropLocked(std::map<uint64_t, Img> &cache, const std::vector<uint64_t> &keys)
	{
		for (uint64_t k : keys) {
			auto it = cache.find(k);
			if (it == cache.end()) {
				continue;
			}
			vkDestroyImage(m_dev, it->second.image, NULL);
			vkFreeMemory(m_dev, it->second.memory, NULL);
			cache.erase(it);
		}
	}

	bool
	readPixelLocked(VkImage image, uint8_t out[4], bool external)
	{
		if (m_busy && vkWaitForFences(m_dev, 1, &m_fence, VK_TRUE, 1000000000ull) != VK_SUCCESS) {
			return false;
		}
		m_busy = false;
		VkBufferCreateInfo bci = {};
		bci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bci.size = 4;
		bci.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
		VkBuffer buf = VK_NULL_HANDLE;
		VkDeviceMemory mem = VK_NULL_HANDLE;
		if (vkCreateBuffer(m_dev, &bci, NULL, &buf) != VK_SUCCESS) {
			return false;
		}
		VkMemoryRequirements req;
		vkGetBufferMemoryRequirements(m_dev, buf, &req);
		VkMemoryAllocateInfo mai = {};
		mai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		mai.allocationSize = req.size;
		mai.memoryTypeIndex =
		    memoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
		bool ok = vkAllocateMemory(m_dev, &mai, NULL, &mem) == VK_SUCCESS;
		if (ok) {
			vkBindBufferMemory(m_dev, buf, mem, 0);
			vkResetCommandPool(m_dev, m_pool, 0);
			VkCommandBufferAllocateInfo cai = {};
			cai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
			cai.commandPool = m_pool;
			cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
			cai.commandBufferCount = 1;
			VkCommandBuffer cb;
			vkAllocateCommandBuffers(m_dev, &cai, &cb);
			VkCommandBufferBeginInfo cbi = {};
			cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
			vkBeginCommandBuffer(cb, &cbi);
			VkImageMemoryBarrier b = {};
			b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
			b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
			b.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
			b.oldLayout = external ? VK_IMAGE_LAYOUT_GENERAL : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
			b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
			b.srcQueueFamilyIndex = external ? VK_QUEUE_FAMILY_EXTERNAL : VK_QUEUE_FAMILY_IGNORED;
			b.dstQueueFamilyIndex = external ? 0 : VK_QUEUE_FAMILY_IGNORED;
			b.image = image;
			vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, NULL,
			                     0, NULL, 1, &b);
			VkBufferImageCopy reg = {};
			reg.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
			reg.imageExtent = {1, 1, 1};
			vkCmdCopyImageToBuffer(cb, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, buf, 1, &reg);
			b.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
			b.dstAccessMask = 0;
			b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
			b.newLayout = external ? VK_IMAGE_LAYOUT_GENERAL : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
			b.srcQueueFamilyIndex = external ? 0 : VK_QUEUE_FAMILY_IGNORED;
			b.dstQueueFamilyIndex = external ? VK_QUEUE_FAMILY_EXTERNAL : VK_QUEUE_FAMILY_IGNORED;
			vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, 0, 0, NULL,
			                     0, NULL, 1, &b);
			vkEndCommandBuffer(cb);
			VkSubmitInfo si = {};
			si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
			si.commandBufferCount = 1;
			si.pCommandBuffers = &cb;
			ok = vkQueueSubmit(m_queue, 1, &si, VK_NULL_HANDLE) == VK_SUCCESS && vkQueueWaitIdle(m_queue) == VK_SUCCESS;
			if (ok) {
				void *data = NULL;
				vkMapMemory(m_dev, mem, 0, 4, 0, &data);
				memcpy(out, data, 4);
				vkUnmapMemory(m_dev, mem);
			}
		}
		vkDestroyBuffer(m_dev, buf, NULL);
		if (mem != VK_NULL_HANDLE) {
			vkFreeMemory(m_dev, mem, NULL);
		}
		return ok;
	}

public:

	//! Drops imported semaphores (SteamVR destroyed their textures).
	void
	forget(const std::vector<uint64_t> &keys)
	{
		// wait() waits outside the lock, so a forgotten semaphore may still be waited on: it is only taken out of
		// the map here and destroyed in destroy(), after the device is idle (audit 5).
		std::lock_guard<std::mutex> lock(m_mutex);
		for (uint64_t k : keys) {
			auto it = m_sems.find(k);
			if (it != m_sems.end()) {
				m_retiredSems.push_back(it->second);
				m_sems.erase(it);
			}
		}
	}

	void
	destroy()
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (m_dev != VK_NULL_HANDLE) {
			vkDeviceWaitIdle(m_dev); // before destroying anything the GPU may still use
			for (VkSemaphore s : m_retiredSems) {
				vkDestroySemaphore(m_dev, s, NULL);
			}
			m_retiredSems.clear();
		}
		for (auto &s : m_sems) {
			vkDestroySemaphore(m_dev, s.second, NULL);
		}
		m_sems.clear();
		if (m_dev != VK_NULL_HANDLE) {
			vkDeviceWaitIdle(m_dev);
			for (auto *cache : {&m_srcImgs, &m_dstImgs}) {
				for (auto &i : *cache) {
					vkDestroyImage(m_dev, i.second.image, NULL);
					vkFreeMemory(m_dev, i.second.memory, NULL);
				}
				cache->clear();
			}
			if (m_fence != VK_NULL_HANDLE) {
				vkDestroyFence(m_dev, m_fence, NULL);
				m_fence = VK_NULL_HANDLE;
			}
			if (m_pool != VK_NULL_HANDLE) {
				vkDestroyCommandPool(m_dev, m_pool, NULL);
				m_pool = VK_NULL_HANDLE;
			}
			vkDestroyDevice(m_dev, NULL);
			m_dev = VK_NULL_HANDLE;
		}
		if (m_inst != VK_NULL_HANDLE) {
			vkDestroyInstance(m_inst, NULL);
			m_inst = VK_NULL_HANDLE;
		}
	}

private:
	uint32_t
	memoryType(uint32_t bits, VkMemoryPropertyFlags want)
	{
		VkPhysicalDeviceMemoryProperties mp;
		vkGetPhysicalDeviceMemoryProperties(m_phys, &mp);
		for (uint32_t i = 0; i < mp.memoryTypeCount; i++) {
			if ((bits & (1u << i)) && (mp.memoryTypes[i].propertyFlags & want) == want) {
				return i;
			}
		}
		return 0;
	}

	std::mutex m_mutex;
	VkPhysicalDevice m_phys = VK_NULL_HANDLE;
	VkQueue m_queue = VK_NULL_HANDLE;
	VkInstance m_inst = VK_NULL_HANDLE;
	VkDevice m_dev = VK_NULL_HANDLE;
	PFN_vkImportSemaphoreFdKHR m_import = NULL;
	PFN_vkWaitSemaphores m_wait = NULL;
	PFN_vkGetSemaphoreCounterValue m_getValue = NULL;
	std::map<uint64_t, VkSemaphore> m_sems;
	std::string m_gpuName;
	std::map<uint64_t, Img> m_srcImgs; //!< SteamVR's images (copy sources), by texture handle
	std::map<uint64_t, Img> m_dstImgs; //!< Monado's swapchain images (copy destinations), by direct mode's key
	VkCommandPool m_pool = VK_NULL_HANDLE;
	VkFence m_fence = VK_NULL_HANDLE;
	bool m_busy = false;                     //!< the last copy's GPU work hadn't finished when copy() returned
	std::vector<VkSemaphore> m_retiredSems;  //!< forgotten semaphores, destroyed in destroy()
};
