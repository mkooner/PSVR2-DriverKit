// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Offline check of the SteamVR plugin's driver direct mode with Monado's compositor (not a ctest: needs a
 *         GPU). A fake SteamVR allocates images the way SteamVR's NewSharedVulkanImage does (exportable Vulkan
 *         memory, opaque fds, dedicated allocations), clears each to a colour, and drives the plugin's
 *         OvrdDirectMode like vrcompositor (GetNextSwapTextureSetIndex, SubmitLayer, Present, PostPresent). The frames
 *         go into a real Monado compositor on the simulated headset: the null compositor by default, the main
 *         compositor in a window with --window.
 *
 * Run: ovrd_direct_mode_harness [--window] [seconds]
 */

#include "ovrd_direct_mode.hpp"

#include "xrt/xrt_instance.h"
#include "xrt/xrt_system.h"
#include "xrt/xrt_session.h"

#include <vulkan/vulkan.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <thread>

#include <cstdlib>
#include <cstring>
#include <map>
#include <vector>

using namespace vr;

namespace {

std::atomic<int> g_vsyncs{0};
// SteamVR's rule (measured 2026-11-18): it predicts poses to 17.63 ms after the vsync time the driver reports.
constexpr int64_t kSteamTargetAfterVsyncNs = 17630000;
std::atomic<int64_t> g_reportedVsyncNs{0};
// Order check: the frame's pose must be submitted (on_frame_start) before each vsync event of a new frame.
std::atomic<int64_t> g_poseForDisplay{0};   //!< display time of the pose submitted last
std::atomic<int> g_vsyncWithPose{0};        //!< vsync events preceded by a pose for a future display time
std::atomic<int> g_vsyncAfterFrame{0};      //!< vsync events sent from PostPresent (after a pose callback)

struct FakeLog : IVRDriverLog
{
	void
	Log(const char *m) override
	{
		fputs(m, stdout);
	}
};

struct FakeHost : IVRServerDriverHost
{
	bool TrackedDeviceAdded(const char *, ETrackedDeviceClass, ITrackedDeviceServerDriver *) override { return true; }
	void TrackedDevicePoseUpdated(uint32_t, const DriverPose_t &, uint32_t) override {}
	void
	VsyncEvent(double offset_s) override
	{
		g_vsyncs++;
		g_reportedVsyncNs = os_monotonic_get_ns() + (int64_t)(offset_s * 1e9);
		int64_t p = g_poseForDisplay.exchange(0);
		if (p != 0) {
			g_vsyncAfterFrame++;
			if (p > os_monotonic_get_ns()) {
				g_vsyncWithPose++;
			}
		}
	}
	void VendorSpecificEvent(uint32_t, EVREventType, const VREvent_Data_t &, double) override {}
	bool IsExiting() override { return false; }
	bool PollNextEvent(VREvent_t *, uint32_t) override { return false; }
	void GetRawTrackedDevicePoses(float, TrackedDevicePose_t *, uint32_t) override {}
	void RequestRestart(const char *, const char *, const char *, const char *) override {}
	uint32_t GetFrameTimings(Compositor_FrameTiming *, uint32_t) override { return 0; }
	void SetDisplayEyeToHead(uint32_t, const HmdMatrix34_t &, const HmdMatrix34_t &) override {}
	void SetDisplayProjectionRaw(uint32_t, const HmdRect2_t &, const HmdRect2_t &) override {}
	void SetRecommendedRenderTargetSize(uint32_t, uint32_t, uint32_t) override {}
};

//! The "SteamVR side": a Vulkan device that allocates exportable images, as NewSharedVulkanImage.
struct FakeSteamVk
{
	VkInstance inst = VK_NULL_HANDLE;
	VkPhysicalDevice phys = VK_NULL_HANDLE;
	VkDevice dev = VK_NULL_HANDLE;
	VkQueue queue = VK_NULL_HANDLE;
	uint32_t family = 0;
	VkCommandPool pool = VK_NULL_HANDLE;
	PFN_vkGetMemoryFdKHR getMemoryFd = nullptr;
	PFN_vkGetSemaphoreFdKHR getSemFd = nullptr;

	/*!
	 * As SteamVR (see ovrd_steamvr_sync.hpp): each image has a timeline semaphore and both have a 0x48-byte state in
	 * shared memory; image +0x40 = the semaphore's handle, semaphore +0x00 = the value of its last submitted write.
	 */
	struct Timeline
	{
		VkSemaphore sem = VK_NULL_HANDLE;
		uint64_t handle = 0;
		uint8_t *state = nullptr; //!< the semaphore's state (+0x00 value)
		uint64_t value = 0;
	};
	std::map<uint64_t, Timeline> timelines; //!< by image handle
	std::vector<std::string> shmNames;

	uint8_t *
	makeState(uint64_t handle)
	{
		char path[64];
		ovrd_steamvr_sync::state_path(handle, (unsigned)getpid(), (unsigned)getuid(), path, sizeof(path));
		int fd = shm_open(path, O_RDWR | O_CREAT | O_TRUNC, 0600);
		if (fd < 0 || ftruncate(fd, ovrd_steamvr_sync::kStateSize) != 0) {
			return nullptr;
		}
		void *m = mmap(nullptr, ovrd_steamvr_sync::kStateSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
		close(fd);
		shmNames.push_back(path);
		return m == MAP_FAILED ? nullptr : (uint8_t *)m;
	}

	bool
	addTimeline(uint64_t image)
	{
		VkExportSemaphoreCreateInfo exp = {};
		exp.sType = VK_STRUCTURE_TYPE_EXPORT_SEMAPHORE_CREATE_INFO;
		exp.handleTypes = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT;
		VkSemaphoreTypeCreateInfo tci = {};
		tci.sType = VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO;
		tci.pNext = &exp;
		tci.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
		VkSemaphoreCreateInfo sci = {};
		sci.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
		sci.pNext = &tci;
		Timeline t;
		if (vkCreateSemaphore(dev, &sci, nullptr, &t.sem) != VK_SUCCESS) {
			return false;
		}
		t.handle = image | (1ULL << 40);
		uint8_t *img_state = makeState(image);
		t.state = makeState(t.handle);
		if (img_state == nullptr || t.state == nullptr) {
			return false;
		}
		memset(img_state, 0, ovrd_steamvr_sync::kStateSize);
		memset(t.state, 0, ovrd_steamvr_sync::kStateSize);
		memcpy(img_state + ovrd_steamvr_sync::kImageSemaphoreOffset, &t.handle, 8);
		timelines[image] = t;
		return true;
	}

	//! A write into the image starts: its value goes into the state now, the GPU signals it @p delay_ns later.
	void
	beginWrite(uint64_t image, int64_t delay_ns, bool signal)
	{
		Timeline &t = timelines[image];
		t.value++;
		uint64_t v = t.value;
		__atomic_store_n((uint64_t *)t.state, v, __ATOMIC_RELEASE);
		if (!signal) {
			return;
		}
		VkDevice d = dev;
		VkSemaphore sem = t.sem;
		std::thread([d, sem, v, delay_ns]() {
			os_nanosleep(delay_ns);
			VkSemaphoreSignalInfo si = {};
			si.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SIGNAL_INFO;
			si.semaphore = sem;
			si.value = v;
			vkSignalSemaphore(d, &si);
		}).detach();
	}

	int
	semaphoreFd(uint64_t sem_handle)
	{
		for (auto &t : timelines) {
			if (t.second.handle == sem_handle) {
				VkSemaphoreGetFdInfoKHR gi = {};
				gi.sType = VK_STRUCTURE_TYPE_SEMAPHORE_GET_FD_INFO_KHR;
				gi.semaphore = t.second.sem;
				gi.handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT;
				int fd = -1;
				return getSemFd(dev, &gi, &fd) == VK_SUCCESS ? fd : -1;
			}
		}
		return -1;
	}

	struct Image
	{
		VkImage image;
		VkDeviceMemory memory;
		int fd;
	};
	std::map<uint64_t, Image> images;

	bool
	init()
	{
		VkApplicationInfo app = {VK_STRUCTURE_TYPE_APPLICATION_INFO};
		app.apiVersion = VK_API_VERSION_1_1;
		VkInstanceCreateInfo ici = {VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
		ici.pApplicationInfo = &app;
		if (vkCreateInstance(&ici, nullptr, &inst) != VK_SUCCESS) {
			return false;
		}
		uint32_t n = 0;
		vkEnumeratePhysicalDevices(inst, &n, nullptr);
		std::vector<VkPhysicalDevice> devs(n);
		vkEnumeratePhysicalDevices(inst, &n, devs.data());
		for (auto d : devs) {
			VkPhysicalDeviceProperties p;
			vkGetPhysicalDeviceProperties(d, &p);
			if (phys == VK_NULL_HANDLE || p.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
				phys = d;
			}
		}
		if (phys == VK_NULL_HANDLE) {
			return false;
		}
		uint32_t qn = 0;
		vkGetPhysicalDeviceQueueFamilyProperties(phys, &qn, nullptr);
		std::vector<VkQueueFamilyProperties> qf(qn);
		vkGetPhysicalDeviceQueueFamilyProperties(phys, &qn, qf.data());
		for (uint32_t i = 0; i < qn; i++) {
			if (qf[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
				family = i;
				break;
			}
		}
		float prio = 1.0f;
		VkDeviceQueueCreateInfo qci = {VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
		qci.queueFamilyIndex = family;
		qci.queueCount = 1;
		qci.pQueuePriorities = &prio;
		const char *exts[] = {VK_KHR_EXTERNAL_MEMORY_FD_EXTENSION_NAME, VK_KHR_EXTERNAL_SEMAPHORE_FD_EXTENSION_NAME};
		VkPhysicalDeviceVulkan12Features f12 = {};
		f12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
		f12.timelineSemaphore = VK_TRUE;
		VkDeviceCreateInfo dci = {VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
		dci.pNext = &f12;
		dci.queueCreateInfoCount = 1;
		dci.pQueueCreateInfos = &qci;
		dci.enabledExtensionCount = 2;
		dci.ppEnabledExtensionNames = exts;
		if (vkCreateDevice(phys, &dci, nullptr, &dev) != VK_SUCCESS) {
			return false;
		}
		vkGetDeviceQueue(dev, family, 0, &queue);
		getMemoryFd = (PFN_vkGetMemoryFdKHR)vkGetDeviceProcAddr(dev, "vkGetMemoryFdKHR");
		getSemFd = (PFN_vkGetSemaphoreFdKHR)vkGetDeviceProcAddr(dev, "vkGetSemaphoreFdKHR");
		VkCommandPoolCreateInfo pci = {VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
		pci.queueFamilyIndex = family;
		return vkCreateCommandPool(dev, &pci, nullptr, &pool) == VK_SUCCESS && getMemoryFd != nullptr;
	}

	//! An exportable image with the flags our CreateSwapTextureSet asks NewSharedVulkanImage for (plus
	//! COLOR_ATTACHMENT for "renderable" and TRANSFER_DST so it can be cleared here).
	bool
	create(uint64_t handle, uint32_t format, uint32_t w, uint32_t h, uint32_t flags, uint32_t usage, float shade)
	{
		VkExternalMemoryImageCreateInfo ext = {VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO};
		ext.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
		VkImageCreateInfo ici = {VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
		ici.pNext = &ext;
		ici.flags = flags;
		ici.imageType = VK_IMAGE_TYPE_2D;
		ici.format = (VkFormat)format;
		ici.extent = {w, h, 1};
		ici.mipLevels = 1;
		ici.arrayLayers = 1;
		ici.samples = VK_SAMPLE_COUNT_1_BIT;
		ici.tiling = VK_IMAGE_TILING_OPTIMAL;
		ici.usage = usage | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		Image img = {};
		if (vkCreateImage(dev, &ici, nullptr, &img.image) != VK_SUCCESS) {
			return false;
		}
		VkMemoryRequirements req;
		vkGetImageMemoryRequirements(dev, img.image, &req);
		VkPhysicalDeviceMemoryProperties mp;
		vkGetPhysicalDeviceMemoryProperties(phys, &mp);
		uint32_t type = 0;
		for (uint32_t i = 0; i < mp.memoryTypeCount; i++) {
			if ((req.memoryTypeBits & (1u << i)) &&
			    (mp.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)) {
				type = i;
				break;
			}
		}
		VkExportMemoryAllocateInfo exp = {VK_STRUCTURE_TYPE_EXPORT_MEMORY_ALLOCATE_INFO};
		exp.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
		VkMemoryDedicatedAllocateInfo ded = {VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO};
		ded.pNext = &exp;
		ded.image = img.image;
		VkMemoryAllocateInfo mai = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
		mai.pNext = &ded;
		mai.allocationSize = req.size;
		mai.memoryTypeIndex = type;
		if (vkAllocateMemory(dev, &mai, nullptr, &img.memory) != VK_SUCCESS ||
		    vkBindImageMemory(dev, img.image, img.memory, 0) != VK_SUCCESS) {
			return false;
		}
		VkMemoryGetFdInfoKHR gfi = {VK_STRUCTURE_TYPE_MEMORY_GET_FD_INFO_KHR};
		gfi.memory = img.memory;
		gfi.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
		if (getMemoryFd(dev, &gfi, &img.fd) != VK_SUCCESS) {
			return false;
		}
		clear(img.image, shade);
		images[handle] = img;
		return addTimeline(handle);
	}

	void
	clear(VkImage image, float shade)
	{
		VkCommandBufferAllocateInfo ai = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
		ai.commandPool = pool;
		ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		ai.commandBufferCount = 1;
		VkCommandBuffer cb;
		vkAllocateCommandBuffers(dev, &ai, &cb);
		VkCommandBufferBeginInfo bi = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
		vkBeginCommandBuffer(cb, &bi);
		VkImageSubresourceRange range = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
		VkImageMemoryBarrier b = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
		b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		b.image = image;
		b.subresourceRange = range;
		vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0,
		                     nullptr, 1, &b);
		VkClearColorValue c = {{shade, 0.3f, 1.0f - shade, 1.0f}};
		vkCmdClearColorImage(cb, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &c, 1, &range);
		// SteamVR's compositor leaves its images ready to be sampled.
		b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
		b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		b.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr,
		                     0, nullptr, 1, &b);
		vkEndCommandBuffer(cb);
		VkSubmitInfo si = {VK_STRUCTURE_TYPE_SUBMIT_INFO};
		si.commandBufferCount = 1;
		si.pCommandBuffers = &cb;
		vkQueueSubmit(queue, 1, &si, VK_NULL_HANDLE);
		vkQueueWaitIdle(queue);
		vkFreeCommandBuffers(dev, pool, 1, &cb);
	}

	void
	destroy()
	{
		for (auto &i : images) {
			close(i.second.fd);
			vkDestroyImage(dev, i.second.image, nullptr);
			vkFreeMemory(dev, i.second.memory, nullptr);
		}
		images.clear();
		vkDeviceWaitIdle(dev);
		for (auto &t : timelines) {
			vkDestroySemaphore(dev, t.second.sem, nullptr);
		}
		timelines.clear();
		for (auto &n : shmNames) {
			shm_unlink(n.c_str());
		}
		if (pool) {
			vkDestroyCommandPool(dev, pool, nullptr);
		}
		if (dev) {
			vkDestroyDevice(dev, nullptr);
		}
		if (inst) {
			vkDestroyInstance(inst, nullptr);
		}
	}
};

FakeSteamVk g_vk;
uint64_t g_nextHandle = 0x100;
int g_imageFails = 0;

struct FakeIpc : IVRIPCResourceManagerClient
{
	bool
	NewSharedVulkanImage(uint32_t format, uint32_t w, uint32_t h, bool, bool, bool, uint32_t, uint32_t, uint32_t flags,
	                     uint32_t usage, SharedTextureHandle_t *out) override
	{
		uint64_t handle = g_nextHandle++;
		if (!g_vk.create(handle, format, w, h, flags, usage, (handle % 3) / 2.0f)) {
			g_imageFails++;
			return false;
		}
		*out = handle;
		return true;
	}
	bool NewSharedVulkanBuffer(uint32_t, uint32_t, SharedTextureHandle_t *) override { return false; }
	bool NewSharedVulkanSemaphore(bool, SharedTextureHandle_t *) override { return false; }
	bool
	RefResource(SharedTextureHandle_t h, uint64_t *ipc) override
	{
		*ipc = h;
		return true;
	}
	bool UnrefResource(SharedTextureHandle_t) override { return true; }
	bool GetDmabufFormats(uint32_t *n, uint32_t *) override { *n = 0; return false; }
	bool GetDmabufModifiers(EVRApplicationType, uint32_t, uint32_t *n, uint64_t *) override { *n = 0; return false; }
	bool ImportDmabuf(EVRApplicationType, DmabufAttributes_t *, SharedTextureHandle_t *) override { return false; }
	bool
	ReceiveSharedFd(uint64_t ipc, int *fd) override
	{
		auto it = g_vk.images.find(ipc);
		*fd = it == g_vk.images.end() ? g_vk.semaphoreFd(ipc) : dup(it->second.fd);
		return *fd >= 0;
	}
};

FakeLog g_log;
FakeHost g_host;
FakeIpc g_ipc;

struct FakeContext : IVRDriverContext
{
	void *
	GetGenericInterface(const char *v, EVRInitError *e) override
	{
		if (e) {
			*e = VRInitError_None;
		}
		if (!strcmp(v, IVRServerDriverHost_Version)) {
			return &g_host;
		}
		if (!strcmp(v, IVRIPCResourceManagerClient_Version)) {
			return &g_ipc;
		}
		if (!strcmp(v, IVRDriverLog_Version)) {
			return &g_log;
		}
		if (e) {
			*e = VRInitError_Init_InterfaceNotFound;
		}
		return nullptr;
	}
	DriverHandle_t
	GetDriverHandle() override
	{
		return 1;
	}
};

int g_fails = 0;

void
check(bool ok, const char *what)
{
	printf("%s: %s\n", ok ? "PASS" : "FAIL", what);
	g_fails += ok ? 0 : 1;
}

} // namespace

int
main(int argc, char **argv)
{
	bool window = false;
	double seconds = 5.0;
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--window")) {
			window = true;
		} else if (!strcmp(argv[i], "--debug-image")) {
		} else {
			seconds = atof(argv[i]);
		}
	}
	setenv("SIMULATED_ENABLE", "true", 0);
	// --debug-image: Monado's main compositor (its real renderer) on the off-screen debug target, no window.
	bool debug_image = false;
	for (int i = 1; i < argc; i++) {
		debug_image = debug_image || !strcmp(argv[i], "--debug-image");
	}
	if (debug_image) {
		setenv("XRT_COMPOSITOR_FORCE_DEBUG_IMAGE", "1", 0);
	} else if (!window) {
		setenv("XRT_COMPOSITOR_NULL", "true", 0);
	}

	static FakeContext ctx;
	VRDriverContext() = &ctx;
	OpenVRInternal_ModuleServerDriverContext().Clear();
	s_pLogFile = &g_log;

	// Monado as the plugin creates it in direct mode.
	struct xrt_instance *xinst = NULL;
	struct xrt_system *xsys = NULL;
	struct xrt_system_devices *xsysd = NULL;
	struct xrt_space_overseer *xso = NULL;
	struct xrt_system_compositor *xsysc = NULL;
	struct xrt_session *xs = NULL;
	struct xrt_compositor_native *xcn = NULL;
	check(xrt_instance_create(NULL, &xinst) == XRT_SUCCESS, "Monado instance");
	check(xrt_instance_create_system(xinst, &xsys, &xsysd, &xso, &xsysc) == XRT_SUCCESS && xsysc != NULL,
	      "system with Monado's compositor (simulated headset)");
	if (g_fails) {
		return 1;
	}
	struct xrt_device *head = xsysd->static_roles.head;
	struct xrt_session_info xsi = {};
	check(xrt_system_create_session(xsys, &xsi, &xs, &xcn) == XRT_SUCCESS && xcn != NULL, "session");
	if (xsysc->xmcc != NULL) {
		xrt_syscomp_set_state(xsysc, &xcn->base, true, true, os_monotonic_get_ns());
		xrt_syscomp_set_z_order(xsysc, &xcn->base, 0);
	}
	struct xrt_begin_session_info begin = {};
	begin.view_type = XRT_VIEW_TYPE_STEREO;
	check(xrt_comp_begin_session(&xcn->base, &begin) == XRT_SUCCESS, "begin session");
	check(g_vk.init(), "fake SteamVR Vulkan device with exportable memory");
	if (g_fails) {
		return 1;
	}

	const char *log = "/tmp/ovrd_direct_mode_harness.csv";
	int frames = 0;
	int stuck_frames = 0;
	int target_ok = 0;
	int64_t last_target = 0;
	{
		OvrdDirectMode dm;
		OvrdDirectMode::Output out;
		out.xcn = xcn;
		out.head = head;
		out.fov[0] = head->hmd->distortion.fov[0];
		out.fov[1] = head->hmd->distortion.fov[1];
		out.eye_in_head[0].position.x = -0.032f;
		out.eye_in_head[1].position.x = 0.032f;
		dm.setOutput(out);
		dm.world_to_device = [](const struct xrt_pose &p) { return p; };
		dm.check_head = head;
		dm.on_frame_start = [](int64_t display_ns) { g_poseForDisplay = display_ns; };
		dm.capture_after_s = 0.5; // the frames below have two layers
		dm.start(90.0f, log);

		IVRDriverDirectModeComponent *c = &dm;
		// What SteamVR's compositor asked for in the 08:31 session: R8G8B8A8_SRGB 4584x4676, one set per eye.
		IVRDriverDirectModeComponent::SwapTextureSetDesc_t desc = {4584, 4676, 43, 1};
		IVRDriverDirectModeComponent::SwapTextureSet_t set[2];
		c->CreateSwapTextureSet(4242, &desc, &set[0]);
		c->CreateSwapTextureSet(4242, &desc, &set[1]);
		check(g_imageFails == 0 && set[1].rSharedTextureHandles[2] != 0, "6 exportable images allocated");

		const int64_t t0 = os_monotonic_get_ns();
		uint32_t idx[2] = {0, 0};
		while (os_monotonic_get_ns() - t0 < (int64_t)(seconds * 1e9)) {
			SharedTextureHandle_t hs[2] = {set[0].rSharedTextureHandles[0], set[1].rSharedTextureHandles[0]};
			c->GetNextSwapTextureSetIndex(hs, &idx);
			struct xrt_space_relation rel = {};
			xrt_device_get_tracked_pose(head, XRT_INPUT_GENERIC_HEAD_POSE, os_monotonic_get_ns() + 17000000, &rel);
			IVRDriverDirectModeComponent::SubmitLayerPerEye_t eyes[2] = {};
			for (int e = 0; e < 2; e++) {
				eyes[e].hTexture = set[e].rSharedTextureHandles[idx[e]];
				eyes[e].bounds = {0, 0, 1, 1};
				const xrt_quat &q = rel.pose.orientation;
				HmdMatrix34_t &m = eyes[e].mHmdPose;
				m.m[0][0] = 1 - 2 * (q.y * q.y + q.z * q.z);
				m.m[0][1] = 2 * (q.x * q.y - q.z * q.w);
				m.m[0][2] = 2 * (q.x * q.z + q.y * q.w);
				m.m[1][0] = 2 * (q.x * q.y + q.z * q.w);
				m.m[1][1] = 1 - 2 * (q.x * q.x + q.z * q.z);
				m.m[1][2] = 2 * (q.y * q.z - q.x * q.w);
				m.m[2][0] = 2 * (q.x * q.z - q.y * q.w);
				m.m[2][1] = 2 * (q.y * q.z + q.x * q.w);
				m.m[2][2] = 1 - 2 * (q.x * q.x + q.y * q.y);
				m.m[0][3] = rel.pose.position.x;
				m.m[1][3] = rel.pose.position.y;
				m.m[2][3] = rel.pose.position.z;
				eyes[e].flHmdPosePredictionTimeInSecondsFromNow = 0.017f;
			}
			// SteamVR draws into the images: its write is submitted now and finishes 6 ms later, i.e. 4 ms after
			// Present is called; in the last 30 frames it never signals (a stuck GPU write).
			const bool stuck = os_monotonic_get_ns() - t0 > (int64_t)(seconds * 1e9) - 30 * 50000000LL;
			for (int e = 0; e < 2; e++) {
				g_vk.beginWrite(eyes[e].hTexture, 6000000, !stuck);
			}
			stuck_frames += stuck ? 1 : 0;
			c->SubmitLayer(eyes);
			c->SubmitLayer(eyes); // a second layer (SteamVR's overlay), the same images here
			os_nanosleep(2000000); // SteamVR's compositor CPU work before Present
			c->Present(0);
			IVRDriverDirectModeComponent::Throttling_t thr = {0, 0};
			c->PostPresent(&thr);
			// The pose thread's target: the display time of the Monado frame just waited for (in PostPresent).
			if (dm.displayTarget() > os_monotonic_get_ns() && dm.displayTarget() != last_target) {
				target_ok++;
			}
			last_target = dm.displayTarget();
			frames++;
		}
		const double secs = (os_monotonic_get_ns() - t0) / 1e9;
		printf("%d frames in %.2f s (%.1f/s), %d vsync events\n", frames, secs, frames / secs, g_vsyncs.load());
		dm.stop();
	}

	// The log: every Present with its Monado frame, and whether it went to the compositor.
	FILE *f = fopen(log, "r");
	int presents = 0, submitted = 0, ok_pose = 0, poses = 0;
	long long last_s_t = 0;
	double last_pred = 0;
	std::vector<double> target_err_ms;
	int sync_states[5] = {0, 0, 0, 0, 0};
	std::vector<double> waits_ok, waits_stuck;
	int64_t last_frame = -1;
	int frame_steps_ok = 0;
	char line[512];
	while (f && fgets(line, sizeof(line), f)) {
		if (line[0] == 'P') {
			long long t, sync, id, disp, terr;
			double wait_us;
			int gsync;
			int layers, sub;
			if (sscanf(line, "P,%lld,%lld,%d,%lld,%lld,%d,%lld,%lf,%d", &t, &sync, &layers, &id, &disp, &sub, &terr,
			           &wait_us, &gsync) == 9) {
				sync_states[gsync]++;
				(gsync == 1 ? waits_ok : waits_stuck).push_back(wait_us);
				if (last_s_t != 0) {
					target_err_ms.push_back((last_s_t + last_pred * 1e9 - disp) / 1e6);
				}
				presents++;
				submitted += sub;
				frame_steps_ok += (last_frame < 0 || id > last_frame) ? 1 : 0;
				last_frame = id;
			}
		} else if (line[0] == 'S') {
			const char *p = strrchr(line, ',');
			const char *q = p;
			while (q > line && *(q - 1) != ',') {
				q--;
			}
			float err_mm = atof(q - 0 == p ? p : q);
			// err_mm is the second-to-last field
			char *fields[32];
			int nf = 0;
			char buf[512];
			strncpy(buf, line, sizeof(buf) - 1);
			buf[sizeof(buf) - 1] = 0;
			for (char *tok = strtok(buf, ","); tok && nf < 32; tok = strtok(NULL, ",")) {
				fields[nf++] = tok;
			}
			(void)err_mm;
			if (nf >= 4 && atoi(fields[2]) == 0) {
				last_s_t = atoll(fields[1]);
				last_pred = atof(fields[3]);
			}
			if (nf >= 21 && atoi(fields[2]) == 0) { // the pose check is done on layer 0
				poses++;
				ok_pose += atof(fields[19]) < 1.0 ? 1 : 0;
			}
		}
	}
	if (f) {
		fclose(f);
	}
	printf("log: %d presents, %d submitted to Monado, Monado frame ids increasing in %d; %d poses, %d within 1 mm\n",
	       presents, submitted, frame_steps_ok, poses, ok_pose);
	check(presents == frames && presents > 0, "every Present logged");
	check(target_ok >= frames - 2, "after each PostPresent the pose target is the new frame's (future) display time");
	printf("pose submitted before the vsync event: %d of %d frame vsyncs (pose for a future display time: %d)\n",
	       g_vsyncAfterFrame.load(), frames, g_vsyncWithPose.load());
	check(g_vsyncAfterFrame >= frames - 2 && g_vsyncWithPose >= frames - 2,
	      "every frame's pose (for its display time) is submitted before SteamVR is told about the vsync");
	check(submitted >= presents - 1, "every frame went to Monado's compositor (imports worked, layers accepted)");
	check(frame_steps_ok == presents, "each frame is a new Monado frame (SteamVR paced by Monado's frame timing)");
	check(ok_pose >= poses * 0.95, "pose check: SteamVR's pose maps back onto the device's own pose");
	{
		std::sort(waits_ok.begin(), waits_ok.end());
		std::sort(waits_stuck.begin(), waits_stuck.end());
		auto med = [](const std::vector<double> &v) { return v.empty() ? -1.0 : v[v.size() / 2]; };
		printf("GPU sync: %d frames waited for the write (median %.0f us), %d timed out (median %.0f us, %d stuck "
		       "frames), %d without semaphore, %d errors\n",
		       sync_states[1], med(waits_ok), sync_states[2], med(waits_stuck), stuck_frames, sync_states[3],
		       sync_states[4]);
		check(sync_states[1] >= presents - stuck_frames - 2, "every frame waited until SteamVR's write finished");
		check(med(waits_ok) > 3000 && med(waits_ok) < 5500,
		      "Present waited ~4 ms (the write finished 4 ms after Present), not 0 and not a timeout");
		check(sync_states[2] >= stuck_frames - 2 && med(waits_stuck) < 1.2 * 11.2e3,
		      "a write that never finishes times out within one frame instead of stalling");
		check(sync_states[3] == 0 && sync_states[4] == 0, "semaphores found and imported for every image");
	}
	{
		// The capture: layer 0's left image, cleared by the fake SteamVR to (shade, 0.3, 1 - shade, 1) in sRGB.
		const char *cap = "/tmp/ovrd_direct_mode_harness-layer0.rgba";
		FILE *cf = fopen(cap, "rb");
		unsigned w = 0, h = 0, fmt = 0;
		std::vector<uint8_t> px;
		if (cf != NULL && fscanf(cf, "%u %u %u\n", &w, &h, &fmt) == 3) {
			px.resize((size_t)w * h * 4);
			px.resize(fread(px.data(), 1, px.size(), cf));
		}
		if (cf != NULL) {
			fclose(cf);
		}
		bool uniform = px.size() == (size_t)w * h * 4 && !px.empty();
		for (size_t i = 0; uniform && i < px.size(); i += 4 * 9973) {
			uniform = px[i + 1] == px[1] && px[i + 3] == px[3];
		}
		// sRGB encoding of 0.3 is 149 (+-1)
		printf("capture: %ux%u format %u, first pixel %d %d %d %d\n", w, h, fmt, px.empty() ? -1 : px[0],
		       px.empty() ? -1 : px[1], px.empty() ? -1 : px[2], px.empty() ? -1 : px[3]);
		check(w == 4584 && h == 4676 && fmt == 43 && uniform && px[3] == 255 && std::abs(px[1] - 149) <= 1,
		      "layer capture reads back the colour SteamVR drew (green 0.3 -> sRGB 149, alpha 255)");
		remove(cap);
		remove("/tmp/ovrd_direct_mode_harness-layer1.rgba");
	}


	xrt_comp_end_session(&xcn->base);
	xrt_comp_native_destroy(&xcn);
	xrt_session_destroy(&xs);
	xrt_syscomp_destroy(&xsysc);
	xrt_space_overseer_destroy(&xso);
	xrt_system_devices_destroy(&xsysd);
	xrt_system_destroy(&xsys);
	xrt_instance_destroy(&xinst);
	g_vk.destroy();
	printf("%s\n", g_fails ? "FAILED" : "all checks passed");
	return g_fails ? 1 : 0;
}
