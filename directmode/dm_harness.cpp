// Offline check of the plugin's direct mode prototype (trial/src/xrt/state_trackers/steamvr_drv/ovrd_direct_mode.hpp):
// a fake SteamVR creates swap sets (memfd stands in for the dma-buf), runs a compositor loop the way vrcompositor
// drives a direct mode driver (GetNextSwapTextureSetIndex, SubmitLayer, Present, PostPresent) and checks the pacing,
// the vsync events and the frame log.
// Build: see build.sh. Run: ./dm_harness <log.csv>
#include "ovrd_direct_mode.hpp"

#include <sys/mman.h>
#include <cstring>
#include <cassert>

using namespace vr;

static std::atomic<int> g_vsyncs{0};

struct FakeLog : IVRDriverLog
{
	void Log(const char *m) override { fputs(m, stdout); }
};

struct FakeHost : IVRServerDriverHost
{
	bool TrackedDeviceAdded(const char *, ETrackedDeviceClass, ITrackedDeviceServerDriver *) override { return true; }
	void TrackedDevicePoseUpdated(uint32_t, const DriverPose_t &, uint32_t) override {}
	void VsyncEvent(double) override { g_vsyncs++; }
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

static std::atomic<int> g_live{0};
static std::atomic<uint64_t> g_next{1000};

struct FakeIpc : IVRIPCResourceManagerClient
{
	bool NewSharedVulkanImage(uint32_t, uint32_t w, uint32_t h, bool, bool, bool, uint32_t, uint32_t, uint32_t,
	                          uint32_t, SharedTextureHandle_t *out) override
	{
		*out = g_next++;
		g_live++;
		return true;
	}
	bool NewSharedVulkanBuffer(uint32_t, uint32_t, SharedTextureHandle_t *) override { return false; }
	bool NewSharedVulkanSemaphore(bool, SharedTextureHandle_t *) override { return false; }
	bool RefResource(SharedTextureHandle_t h, uint64_t *ipc) override { *ipc = h; return true; }
	bool UnrefResource(SharedTextureHandle_t) override { g_live--; return true; }
	bool GetDmabufFormats(uint32_t *n, uint32_t *) override { *n = 0; return false; }
	bool GetDmabufModifiers(EVRApplicationType, uint32_t, uint32_t *n, uint64_t *) override { *n = 0; return false; }
	bool ImportDmabuf(EVRApplicationType, DmabufAttributes_t *, SharedTextureHandle_t *) override { return false; }
	bool ReceiveSharedFd(uint64_t, int *fd) override
	{
		*fd = memfd_create("fake-dmabuf", 0);
		return *fd >= 0 && ftruncate(*fd, 3040 * 3100 * 4) == 0;
	}
};

static FakeLog g_log;
static FakeHost g_host;
static FakeIpc g_ipc;

struct FakeContext : IVRDriverContext
{
	void *GetGenericInterface(const char *v, EVRInitError *e) override
	{
		if (e) *e = VRInitError_None;
		if (!strcmp(v, IVRServerDriverHost_Version)) return &g_host;
		if (!strcmp(v, IVRIPCResourceManagerClient_Version)) return &g_ipc;
		if (!strcmp(v, IVRDriverLog_Version)) return &g_log;
		if (e) *e = VRInitError_Init_InterfaceNotFound;
		return nullptr;
	}
	DriverHandle_t GetDriverHandle() override { return 1; }
};

int main(int argc, char **argv)
{
	static FakeContext ctx;
	VRDriverContext() = &ctx;
	OpenVRInternal_ModuleServerDriverContext().Clear();
	s_pLogFile = &g_log;
	const char *log = argc > 1 ? argv[1] : "dm_harness.csv";
	int fails = 0;
	auto check = [&](bool ok, const char *what) { printf("%s: %s\n", ok ? "PASS" : "FAIL", what); fails += !ok; };

	{
		OvrdDirectMode dm;
		dm.start(89.91f, log);
		IVRDriverDirectModeComponent *c = &dm;
		IVRDriverDirectModeComponent::SwapTextureSetDesc_t desc = {3040, 3100, 43 /* R8G8B8A8_SRGB */, 1};
		IVRDriverDirectModeComponent::SwapTextureSet_t set[2];
		c->CreateSwapTextureSet(4242, &desc, &set[0]);
		c->CreateSwapTextureSet(4242, &desc, &set[1]);
		check(g_live == 6 && set[0].rSharedTextureHandles[2] != 0, "two swap sets of 3 images created");

		// Compositor loop for 12 s: two layers per frame (scene + an overlay).
		const int64_t t0 = os_monotonic_get_ns();
		int frames = 0;
		uint32_t idx[2] = {0, 0};
		std::vector<int64_t> presents;
		while (os_monotonic_get_ns() - t0 < 12000000000LL) {
			SharedTextureHandle_t hs[2] = {set[0].rSharedTextureHandles[0], set[1].rSharedTextureHandles[0]};
			c->GetNextSwapTextureSetIndex(hs, &idx);
			IVRDriverDirectModeComponent::SubmitLayerPerEye_t eyes[2] = {};
			for (int e = 0; e < 2; e++) {
				eyes[e].hTexture = set[e].rSharedTextureHandles[idx[e]];
				eyes[e].bounds = {0, 0, 1, 1};
				eyes[e].mHmdPose.m[0][0] = eyes[e].mHmdPose.m[1][1] = eyes[e].mHmdPose.m[2][2] = 1;
				eyes[e].mHmdPose.m[1][3] = 1.6f;
				eyes[e].flHmdPosePredictionTimeInSecondsFromNow = 0.0195f;
			}
			c->SubmitLayer(eyes);
			c->SubmitLayer(eyes);
			os_nanosleep(2000000); // compositor GPU work
			c->Present(0);
			presents.push_back(os_monotonic_get_ns());
			IVRDriverDirectModeComponent::Throttling_t thr = {0, 0};
			c->PostPresent(&thr);
			frames++;
		}
		const double secs = (os_monotonic_get_ns() - t0) / 1e9;
		std::vector<double> gaps;
		for (size_t i = 1; i < presents.size(); i++) gaps.push_back((presents[i] - presents[i - 1]) / 1e6);
		std::sort(gaps.begin(), gaps.end());
		printf("frames %d in %.2f s (%.2f/s), vsync events %d (%.2f/s), present gap median %.3f p1 %.3f p99 %.3f ms\n",
		       frames, secs, frames / secs, g_vsyncs.load(), g_vsyncs / secs, gaps[gaps.size() / 2],
		       gaps[gaps.size() / 100], gaps[gaps.size() * 99 / 100]);
		check(std::fabs(frames / secs - 89.91) < 0.5, "PostPresent paces the compositor at the display rate");
		check(std::fabs(g_vsyncs / secs - 89.91) < 0.5, "vsync events at the display rate");
		check(gaps[gaps.size() * 99 / 100] - gaps[gaps.size() / 100] < 1.5, "present interval steady (p1-p99 < 1.5 ms)");
		IVRDriverDirectModeComponent::SwapTextureSetDesc_t d2 = desc;
		IVRDriverDirectModeComponent::SwapTextureSet_t s3;
		c->CreateSwapTextureSet(5555, &d2, &s3);
		c->DestroySwapTextureSet(set[1].rSharedTextureHandles[2]); // any handle of a set destroys the whole set
		check(g_live == 6, "destroying a set by any of its handles frees its 3 images");
		c->DestroyAllSwapTextureSets(4242);
		check(g_live == 3, "DestroyAllSwapTextureSets frees only that process's sets");
		vr::DriverDirectMode_FrameTiming ft = {};
		ft.m_nSize = sizeof(ft);
		ft.m_nReprojectionFlags = 0x100;
		c->GetFrameTiming(&ft);
		check(ft.m_nReprojectionFlags == 0 && ft.m_nNumFramePresents == 1, "GetFrameTiming clears the reprojection flags");
		dm.stop();
		check(g_live == 3, "stop leaves SteamVR's handles alone (closes our fds only)");
	}
	// The log: every frame's 2 S lines, its P and Q.
	FILE *f = fopen(log, "r");
	int nc = 0, ns = 0, np = 0, nq = 0;
	char line[512];
	while (f && fgets(line, sizeof(line), f)) {
		nc += line[0] == 'C'; ns += line[0] == 'S'; np += line[0] == 'P'; nq += line[0] == 'Q';
	}
	if (f) fclose(f);
	printf("log: %d C, %d S, %d P, %d Q lines\n", nc, ns, np, nq);
	check(nc == 3 && ns == 2 * np && np == nq && np > 1000, "frame log complete");
	return fails ? 1 : 0;
}
