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
		const char *exts[] = {VK_KHR_EXTERNAL_SEMAPHORE_FD_EXTENSION_NAME};
		VkDeviceCreateInfo dci = {};
		dci.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
		dci.pNext = &f12;
		dci.queueCreateInfoCount = 1;
		dci.pQueueCreateInfos = &qci;
		dci.enabledExtensionCount = 1;
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
			auto it = m_sems.find(key);
			if (it != m_sems.end()) {
				sem = it->second;
			} else {
				int fd = get_fd();
				if (fd < 0) {
					return kError;
				}
				VkSemaphoreTypeCreateInfo tci = {};
		tci.sType = VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO;
				tci.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
				VkSemaphoreCreateInfo sci = {};
		sci.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
				sci.pNext = &tci;
				if (vkCreateSemaphore(m_dev, &sci, NULL, &sem) != VK_SUCCESS) {
					close(fd);
					return kError;
				}
				VkImportSemaphoreFdInfoKHR imp = {};
		imp.sType = VK_STRUCTURE_TYPE_IMPORT_SEMAPHORE_FD_INFO_KHR;
				imp.semaphore = sem;
				imp.handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT;
				imp.fd = fd;
				if (m_import(m_dev, &imp) != VK_SUCCESS) {
					close(fd); // not consumed on failure
					vkDestroySemaphore(m_dev, sem, NULL);
					return kError;
				}
				m_sems[key] = sem;
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

	//! Drops imported semaphores (SteamVR destroyed their textures).
	void
	forget(const std::vector<uint64_t> &keys)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		for (uint64_t k : keys) {
			auto it = m_sems.find(k);
			if (it != m_sems.end()) {
				vkDestroySemaphore(m_dev, it->second, NULL);
				m_sems.erase(it);
			}
		}
	}

	void
	destroy()
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		for (auto &s : m_sems) {
			vkDestroySemaphore(m_dev, s.second, NULL);
		}
		m_sems.clear();
		if (m_dev != VK_NULL_HANDLE) {
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
};
