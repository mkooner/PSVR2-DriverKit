// Offline check of the plugin's direct mode prototype (trial/src/xrt/state_trackers/steamvr_psvr2/ovrn_direct_mode.hpp):
// a fake SteamVR creates swap sets (memfd stands in for the dma-buf), runs a compositor loop the way vrcompositor
// drives a direct mode driver (GetNextSwapTextureSetIndex, SubmitLayer, Present, PostPresent) and checks the pacing,
// the vsync events and the frame log; then the same with a fake Monado compositor as output, where SteamVR skips
// frames and copies fail: every display period must still get a frame (no discard = no black frame).
// Build: see build.sh. Run: ./dm_harness <log.csv>
#include "ovrn_direct_mode.hpp"

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


// A fake Monado compositor (direct mode's output): paces at 89.91 Hz and records what it is given.
struct FakeComp;
struct FakeSc
{
	struct xrt_swapchain_native n = {};
	uint32_t next = 0;
	FakeComp *fc = nullptr;
};
struct FakeComp
{
	struct xrt_compositor_native n = {};
	std::mutex m;
	int waits = 0, commits = 0, commitsWithLayers = 0, discards = 0, discardsAfterFirst = 0, created = 0;
	int liveSwapchains = 0, repeats = 0;
	int layersThisFrame = 0;
	int64_t next_ns = 0, id = 0;
	uint32_t lastIndex[2] = {99, 99};
	FakeComp()
	{
		struct xrt_compositor *xc = &n.base;
		xc->wait_frame = [](struct xrt_compositor *xc, int64_t *id, int64_t *disp, int64_t *period) {
			FakeComp *f = (FakeComp *)xc;
			const int64_t p = 11122000;
			int64_t now = os_monotonic_get_ns();
			if (f->next_ns == 0) f->next_ns = now;
			f->next_ns += p;
			if (f->next_ns > now) os_nanosleep(f->next_ns - now);
			std::lock_guard<std::mutex> l(f->m);
			f->waits++;
			*id = ++f->id;
			*disp = f->next_ns + 20000000;
			*period = p;
			return XRT_SUCCESS;
		};
		xc->begin_frame = [](struct xrt_compositor *xc, int64_t) {
			FakeComp *f = (FakeComp *)xc;
			std::lock_guard<std::mutex> l(f->m);
			f->layersThisFrame = 0;
			return XRT_SUCCESS;
		};
		xc->discard_frame = [](struct xrt_compositor *xc, int64_t) {
			FakeComp *f = (FakeComp *)xc;
			std::lock_guard<std::mutex> l(f->m);
			f->discards++;
			if (f->commitsWithLayers > 0) f->discardsAfterFirst++;
			return XRT_SUCCESS;
		};
		xc->layer_begin = [](struct xrt_compositor *, const struct xrt_layer_frame_data *) { return XRT_SUCCESS; };
		xc->layer_projection = [](struct xrt_compositor *xc, struct xrt_device *, struct xrt_swapchain **,
		                          const struct xrt_layer_data *d) {
			FakeComp *f = (FakeComp *)xc;
			std::lock_guard<std::mutex> l(f->m);
			if (f->layersThisFrame == 0) {
				// The same images as the previous frame: the last frame shown again.
				if (d->proj.v[0].sub.image_index == f->lastIndex[0] && d->proj.v[1].sub.image_index == f->lastIndex[1])
					f->repeats++;
				f->lastIndex[0] = d->proj.v[0].sub.image_index;
				f->lastIndex[1] = d->proj.v[1].sub.image_index;
			}
			f->layersThisFrame++;
			return XRT_SUCCESS;
		};
		xc->layer_commit = [](struct xrt_compositor *xc, xrt_graphics_sync_handle_t) {
			FakeComp *f = (FakeComp *)xc;
			std::lock_guard<std::mutex> l(f->m);
			f->commits++;
			if (f->layersThisFrame > 0) f->commitsWithLayers++;
			return XRT_SUCCESS;
		};
		xc->create_swapchain = [](struct xrt_compositor *xc, const struct xrt_swapchain_create_info *,
		                          struct xrt_swapchain **out) {
			FakeComp *f = (FakeComp *)xc;
			FakeSc *s = new FakeSc();
			s->fc = f;
			s->n.base.reference.count = 1;
			s->n.base.image_count = 3;
			for (int i = 0; i < 3; i++) s->n.images[i].handle = memfd_create("fake-monado-image", 0);
			s->n.base.destroy = [](struct xrt_swapchain *x) {
				FakeSc *s = (FakeSc *)x;
				for (int i = 0; i < 3; i++) close(s->n.images[i].handle);
				std::lock_guard<std::mutex> l(s->fc->m);
				s->fc->liveSwapchains--;
				delete s;
			};
			s->n.base.acquire_image = [](struct xrt_swapchain *x, uint32_t *i) {
				FakeSc *s = (FakeSc *)x;
				*i = s->next++ % 3;
				return XRT_SUCCESS;
			};
			s->n.base.wait_image = [](struct xrt_swapchain *, int64_t, uint32_t) { return XRT_SUCCESS; };
			s->n.base.release_image = [](struct xrt_swapchain *, uint32_t) { return XRT_SUCCESS; };
			std::lock_guard<std::mutex> l(f->m);
			f->created++;
			f->liveSwapchains++;
			*out = &s->n.base;
			return XRT_SUCCESS;
		};
	}
};

int main(int argc, char **argv)
{
	static FakeContext ctx;
	VRDriverContext() = &ctx;
	OpenVRInternal_ModuleServerDriverContext().Clear();
	const char *log = argc > 1 ? argv[1] : "dm_harness.csv";
	int fails = 0;
	auto check = [&](bool ok, const char *what) { printf("%s: %s\n", ok ? "PASS" : "FAIL", what); fails += !ok; };

	{
		OvrnDirectMode dm;
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
	// With an output (a fake Monado compositor): every display period must get a frame. SteamVR skips some Presents
	// and some copies fail; the driver must then show the last frame again, never discard (Monado shows a discarded
	// frame as black: multi_compositor_discard_frame retires the client's delivered frame), 2026-11-24.
	{
		FakeComp fc;
		OvrnDirectMode dm;
		dm.gpu_sync = false;
		std::atomic<int> copies{0};
		dm.test_copy = [&]() { return (++copies % 23) != 0; }; // every 23rd copy fails
		OvrnDirectMode::Output out;
		out.xcn = &fc.n;
		out.fov[0] = out.fov[1] = {-0.8f, 0.8f, 0.8f, -0.8f};
		dm.setOutput(out);
		dm.start(89.91f, nullptr);
		IVRDriverDirectModeComponent *c = &dm;
		IVRDriverDirectModeComponent::SwapTextureSetDesc_t desc = {3040, 3100, 43, 1};
		IVRDriverDirectModeComponent::SwapTextureSet_t set[2];
		c->CreateSwapTextureSet(4343, &desc, &set[0]);
		c->CreateSwapTextureSet(4343, &desc, &set[1]);
		check(fc.created == 2, "output: copy mode, one Monado swapchain per set");
		const int64_t t0 = os_monotonic_get_ns();
		uint32_t idx[2] = {0, 0};
		int n = 0, skipped = 0;
		while (os_monotonic_get_ns() - t0 < 6000000000LL) {
			SharedTextureHandle_t hs[2] = {set[0].rSharedTextureHandles[0], set[1].rSharedTextureHandles[0]};
			c->GetNextSwapTextureSetIndex(hs, &idx);
			IVRDriverDirectModeComponent::SubmitLayerPerEye_t eyes[2] = {};
			for (int e = 0; e < 2; e++) {
				eyes[e].hTexture = set[e].rSharedTextureHandles[idx[e]];
				eyes[e].bounds = {0, 0, 1, 1};
				eyes[e].mHmdPose.m[0][0] = eyes[e].mHmdPose.m[1][1] = eyes[e].mHmdPose.m[2][2] = 1;
				eyes[e].flHmdPosePredictionTimeInSecondsFromNow = 0.0195f;
			}
			c->SubmitLayer(eyes);
			// A game that misses frames: every 15th, and a burst of 6 once a second.
			const bool skip = (n % 15) == 7 || ((n % 90) >= 40 && (n % 90) < 46);
			os_nanosleep(2000000);
			if (!skip) {
				c->Present(0);
			} else {
				skipped++;
			}
			IVRDriverDirectModeComponent::Throttling_t thr = {0, 0};
			c->PostPresent(&thr);
			n++;
		}
		dm.stop();
		std::lock_guard<std::mutex> l(fc.m);
		printf("output: %d Monado frames, %d committed (%d with layers), %d discarded (%d after the first shown), "
		       "%d SteamVR frames skipped, %d copies (%d failed)\n",
		       fc.waits, fc.commits, fc.commitsWithLayers, fc.discards, fc.discardsAfterFirst, skipped, copies.load(),
		       copies.load() / 23);
		check(skipped > 50 && copies.load() / 23 > 5, "output: SteamVR skipped frames and copies failed");
		check(fc.discardsAfterFirst == 0, "output: no frame discarded once one was shown (no black frames)");
		check(fc.commits == fc.commitsWithLayers, "output: every committed frame has layers");
		check(fc.waits - fc.commits - fc.discards <= 2, "output: every Monado frame was committed or (before the first) discarded");
		check(fc.repeats > skipped / 2, "output: missed frames got the last frame again");
		check(fc.liveSwapchains == 0, "output: all Monado swapchains released at stop");
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
