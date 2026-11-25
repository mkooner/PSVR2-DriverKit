// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*
 * hand_sim: replays a recorded session through the real code, end to end, to predict what SteamVR gets from the
 * bare hands, and to check that prediction against what SteamVR really got in that session (vrhandlog).
 *
 *  - tracker: the driver's own hand tracking (psvr2_hand_tracking.c: Mercury + MediaPipe, the gesture measurements),
 *    fed the session's recorded camera transfers at their recorded times, in real time (sim_ht_side.c);
 *  - head: the session's own head poses (vrhandlog, SteamVR's world), so the joints come out where they did live;
 *  - SteamVR side: the plugin's HandDevice (ovrn_hand.hpp, compiled from the plugin source given at build time),
 *    behind a stand-in SteamVR, updated at the session's own SteamVR frame times.
 *
 * Usage: hand_sim recording.pv2c psvr2-calibration-<serial>.bin vrhandlog.csv out.csv
 * out.csv, one row per vrhandlog frame: t_s (vrhandlog's clock), then per hand (left, right): valid, px, py, pz
 * (SteamVR world), trigger value, trigger click, grip click, system click.
 */
#include "openvr_driver.h"

#include "ovrn_hand.hpp"

#include "sim_ht_side.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <time.h>
#include <vector>

using namespace vr;

namespace {

int64_t
mono_ns()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

// ---- the session's head poses (vrhandlog) ----

struct HeadRow
{
	double t_s, pred_s;
	bool valid;
	double q[4], p[3]; // x y z w; x y z
};
std::vector<HeadRow> g_head;
std::vector<double> g_frame_t; //!< vrhandlog frame times (its clock, s)
int64_t g_t0 = 0;              //!< vrhandlog's t0 (session monotonic ns)
int64_t g_replayToSession = 0; //!< session monotonic = replay monotonic + this

bool
load_vrhandlog(const char *path)
{
	FILE *f = fopen(path, "r");
	if (f == nullptr) {
		return false;
	}
	char line[8192];
	std::vector<std::string> cols;
	while (fgets(line, sizeof(line), f)) {
		if (line[0] == '#') {
			const char *k = strstr(line, "t0_mono_ns ");
			if (k != nullptr) {
				g_t0 = atoll(k + 11);
			}
			continue;
		}
		std::vector<std::string> v;
		char *save = nullptr;
		for (char *tok = strtok_r(line, ",\n", &save); tok; tok = strtok_r(nullptr, ",\n", &save)) {
			v.push_back(tok);
		}
		if (cols.empty()) {
			cols = v;
			continue;
		}
		if (v.size() != cols.size()) {
			continue;
		}
		auto col = [&](const char *name) {
			for (size_t i = 0; i < cols.size(); i++) {
				if (cols[i] == name) {
					return atof(v[i].c_str());
				}
			}
			return 0.0;
		};
		HeadRow r;
		r.t_s = col("t_s");
		r.pred_s = col("pred_s");
		r.valid = col("hmd_valid") != 0;
		r.q[0] = col("hmd_qx");
		r.q[1] = col("hmd_qy");
		r.q[2] = col("hmd_qz");
		r.q[3] = col("hmd_qw");
		r.p[0] = col("hmd_px");
		r.p[1] = col("hmd_py");
		r.p[2] = col("hmd_pz");
		g_frame_t.push_back(r.t_s);
		if (r.valid) {
			g_head.push_back(r);
		}
	}
	fclose(f);
	return g_t0 != 0 && !g_head.empty();
}

//! The head at a session time (its predicted pose times, interpolated); before / after the log: its first / last.
void
head_at_session(int64_t session_ns, double out[7])
{
	const double t = (session_ns - g_t0) / 1e9;
	auto at = [](const HeadRow &r) { return r.t_s + r.pred_s; };
	size_t lo = 0, hi = g_head.size() - 1;
	if (t <= at(g_head[lo])) {
		hi = lo;
	} else if (t >= at(g_head[hi])) {
		lo = hi;
	} else {
		while (hi - lo > 1) {
			const size_t mid = (lo + hi) / 2;
			(at(g_head[mid]) <= t ? lo : hi) = mid;
		}
	}
	const HeadRow &a = g_head[lo], &b = g_head[hi];
	const double span = at(b) - at(a);
	const double w = span > 1e-9 ? (t - at(a)) / span : 0.0;
	double q[4], dot = 0;
	for (int i = 0; i < 4; i++) {
		dot += a.q[i] * b.q[i];
	}
	const double sb = dot < 0 ? -1.0 : 1.0;
	double n = 0;
	for (int i = 0; i < 4; i++) {
		q[i] = a.q[i] * (1 - w) + sb * b.q[i] * w;
		n += q[i] * q[i];
	}
	n = std::sqrt(n);
	for (int i = 0; i < 3; i++) {
		out[i] = a.p[i] * (1 - w) + b.p[i] * w;
	}
	for (int i = 0; i < 4; i++) {
		out[3 + i] = q[i] / n;
	}
}

void
head_at_replay(int64_t replay_ns, double out[7])
{
	head_at_session(replay_ns + g_replayToSession, out);
}

// ---- stand-in SteamVR ----

std::mutex g_mutex;
std::map<uint32_t, DriverPose_t> g_lastPose; //!< by device id
double g_frameT = 0; //!< the SteamVR frame being updated (vrhandlog's clock), for the plugin log lines

struct Component
{
	uint64_t container;
	std::string path;
	float value = 0;
};
std::vector<Component> g_components; //!< handle = index + 1

struct FakeLog : IVRDriverLog
{
	void
	Log(const char *m) override
	{
		if (getenv("HAND_SIM_QUIET") == nullptr) {
			printf("[plugin %.3f] %s", g_frameT, m);
		}
	}
};

struct FakeHost : IVRServerDriverHost
{
	bool TrackedDeviceAdded(const char *, ETrackedDeviceClass, ITrackedDeviceServerDriver *) override { return true; }
	void
	TrackedDevicePoseUpdated(uint32_t which, const DriverPose_t &pose, uint32_t) override
	{
		std::lock_guard<std::mutex> l(g_mutex);
		g_lastPose[which] = pose;
	}
	void VsyncEvent(double) override {}
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

struct FakeProperties : IVRProperties
{
	ETrackedPropertyError
	ReadPropertyBatch(PropertyContainerHandle_t, PropertyRead_t *batch, uint32_t n) override
	{
		for (uint32_t i = 0; i < n; i++) {
			batch[i].eError = TrackedProp_UnknownProperty;
			batch[i].unRequiredBufferSize = 0;
		}
		return TrackedProp_Success;
	}
	ETrackedPropertyError
	WritePropertyBatch(PropertyContainerHandle_t, PropertyWrite_t *batch, uint32_t n) override
	{
		for (uint32_t i = 0; i < n; i++) {
			batch[i].eError = TrackedProp_Success;
		}
		return TrackedProp_Success;
	}
	const char *GetPropErrorNameFromEnum(ETrackedPropertyError) override { return "error"; }
	PropertyContainerHandle_t TrackedDeviceToPropertyContainer(TrackedDeviceIndex_t d) override { return 1000 + d; }
};

struct FakeSettings : IVRSettings
{
	const char *GetSettingsErrorNameFromEnum(EVRSettingsError) override { return "error"; }
	void SetBool(const char *, const char *, bool, EVRSettingsError *) override {}
	void SetInt32(const char *, const char *, int32_t, EVRSettingsError *) override {}
	void SetFloat(const char *, const char *, float, EVRSettingsError *) override {}
	void SetString(const char *, const char *, const char *, EVRSettingsError *) override {}
	bool
	GetBool(const char *, const char *, EVRSettingsError *e) override
	{
		if (e) {
			*e = VRSettingsError_UnsetSettingHasNoDefault;
		}
		return false;
	}
	int32_t
	GetInt32(const char *, const char *, EVRSettingsError *e) override
	{
		if (e) {
			*e = VRSettingsError_UnsetSettingHasNoDefault;
		}
		return 0;
	}
	float
	GetFloat(const char *, const char *, EVRSettingsError *e) override
	{
		if (e) {
			*e = VRSettingsError_UnsetSettingHasNoDefault;
		}
		return 0;
	}
	void
	GetString(const char *, const char *, char *value, uint32_t len, EVRSettingsError *e) override
	{
		if (len) {
			value[0] = 0;
		}
		if (e) {
			*e = VRSettingsError_UnsetSettingHasNoDefault;
		}
	}
	void RemoveSection(const char *, EVRSettingsError *) override {}
	void RemoveKeyInSection(const char *, const char *, EVRSettingsError *) override {}
};

struct FakeDriverManager : IVRDriverManager
{
	uint32_t GetDriverCount() const override { return 1; }
	uint32_t GetDriverName(DriverId_t, char *, uint32_t) override { return 0; }
	DriverHandle_t GetDriverHandle(const char *) override { return 1; }
	bool IsEnabled(DriverId_t) const override { return true; }
};

struct FakeResources : IVRResources
{
	uint32_t LoadSharedResource(const char *, char *, uint32_t) override { return 0; }
	uint32_t GetResourceFullPath(const char *, const char *, char *, uint32_t) override { return 0; }
};

struct FakeInput : IVRDriverInput
{
	EVRInputError
	add(PropertyContainerHandle_t c, const char *name, VRInputComponentHandle_t *h)
	{
		std::lock_guard<std::mutex> l(g_mutex);
		g_components.push_back({c, name});
		*h = g_components.size();
		return VRInputError_None;
	}
	EVRInputError
	set(VRInputComponentHandle_t h, float v)
	{
		std::lock_guard<std::mutex> l(g_mutex);
		if (h == 0 || h > g_components.size()) {
			return VRInputError_InvalidHandle;
		}
		g_components[h - 1].value = v;
		return VRInputError_None;
	}
	EVRInputError CreateBooleanComponent(PropertyContainerHandle_t c, const char *n, VRInputComponentHandle_t *h) override { return add(c, n, h); }
	EVRInputError UpdateBooleanComponent(VRInputComponentHandle_t h, bool v, double) override { return set(h, v ? 1.0f : 0.0f); }
	EVRInputError CreateScalarComponent(PropertyContainerHandle_t c, const char *n, VRInputComponentHandle_t *h, EVRScalarType, EVRScalarUnits) override { return add(c, n, h); }
	EVRInputError UpdateScalarComponent(VRInputComponentHandle_t h, float v, double) override { return set(h, v); }
	EVRInputError CreateHapticComponent(PropertyContainerHandle_t c, const char *n, VRInputComponentHandle_t *h) override { return add(c, n, h); }
	EVRInputError CreateSkeletonComponent(PropertyContainerHandle_t c, const char *n, const char *, const char *, EVRSkeletalTrackingLevel, const VRBoneTransform_t *, uint32_t, VRInputComponentHandle_t *h) override { return add(c, n, h); }
	EVRInputError UpdateSkeletonComponent(VRInputComponentHandle_t, EVRSkeletalMotionRange, const VRBoneTransform_t *, uint32_t) override { return VRInputError_None; }
	EVRInputError CreatePoseComponent(PropertyContainerHandle_t c, const char *n, VRInputComponentHandle_t *h) override { return add(c, n, h); }
	EVRInputError UpdatePoseComponent(VRInputComponentHandle_t, const HmdMatrix34_t *, double) override { return VRInputError_None; }
	EVRInputError CreateEyeTrackingComponent(PropertyContainerHandle_t c, const char *n, VRInputComponentHandle_t *h) override { return add(c, n, h); }
	EVRInputError UpdateEyeTrackingComponent(VRInputComponentHandle_t, const VREyeTrackingData_t *, double) override { return VRInputError_None; }
};

FakeLog g_log;
FakeHost g_host;
FakeProperties g_properties;
FakeSettings g_settings;
FakeDriverManager g_driverManager;
FakeResources g_resources;
FakeInput g_input;

struct FakeContext : IVRDriverContext
{
	void *
	GetGenericInterface(const char *version, EVRInitError *error) override
	{
		if (error) {
			*error = VRInitError_None;
		}
		if (!strcmp(version, IVRServerDriverHost_Version)) return static_cast<IVRServerDriverHost *>(&g_host);
		if (!strcmp(version, IVRDriverLog_Version)) return static_cast<IVRDriverLog *>(&g_log);
		if (!strcmp(version, IVRProperties_Version)) return static_cast<IVRProperties *>(&g_properties);
		if (!strcmp(version, IVRDriverInput_Version)) return static_cast<IVRDriverInput *>(&g_input);
		if (!strcmp(version, IVRSettings_Version)) return static_cast<IVRSettings *>(&g_settings);
		if (!strcmp(version, IVRDriverManager_Version)) return static_cast<IVRDriverManager *>(&g_driverManager);
		if (!strcmp(version, IVRResources_Version)) return static_cast<IVRResources *>(&g_resources);
		if (error) {
			*error = VRInitError_Init_InterfaceNotFound;
		}
		return nullptr;
	}
	DriverHandle_t GetDriverHandle() override { return 1; }
};

float
component(uint64_t container, const char *path)
{
	std::lock_guard<std::mutex> l(g_mutex);
	for (const Component &c : g_components) {
		if (c.container == container && c.path == path) {
			return c.value;
		}
	}
	return -1;
}

// ---- the backend: the replayed tracker and the session's head ----

class ReplayBackend final : public ovrn::Backend
{
public:
	bool start(std::string *) override { return true; }
	void stop() override {}
	const char *name() const override { return "replay"; }
	ovrn::DeviceInfo
	headInfo() override
	{
		ovrn::DeviceInfo d;
		d.manufacturer = "Sony";
		d.model = "PlayStation VR2";
		d.serial = "16111237000976";
		return d;
	}
	ovrn::DisplayInfo display() override { return {}; }
	bool eyePoses(ovrn::Pose out[2]) override { (void)out; return false; }
	bool
	headPose(int64_t at_ns, ovrn::TrackedPose *out) override
	{
		double p[7];
		head_at_replay(at_ns, p);
		*out = ovrn::TrackedPose{};
		out->pose.position = {p[0], p[1], p[2]};
		out->pose.orientation = {p[3], p[4], p[5], p[6]};
		out->orientation_valid = out->orientation_tracked = out->position_valid = out->position_tracked = true;
		return true;
	}
	bool distortion(int, float, float, ovrn::DistortionUv *) override { return false; }
	bool hasController(ovrn::Hand) override { return false; }
	ovrn::DeviceInfo controllerInfo(ovrn::Hand) override { return {}; }
	bool controllerPose(ovrn::Hand, int64_t, ovrn::TrackedPose *) override { return false; }
	std::vector<ovrn::InputDesc> controllerInputDescs(ovrn::Hand, std::string *) override { return {}; }
	size_t controllerInputs(ovrn::Hand, float *, size_t) override { return 0; }
	void haptic(ovrn::Hand, float, float, float) override {}
	bool hasHandTracking() override { return true; }
	bool
	handJoints(ovrn::Hand hand, int64_t at_ns, ovrn::HandJoints *out) override
	{
		*out = ovrn::HandJoints();
		struct sim_joints s;
		if (!sim_ht_joints(hand == ovrn::Hand::Left ? 0 : 1, at_ns, &s)) {
			return true; // a tracker, no hand now
		}
		out->active = true;
		out->timestamp_ns = s.timestamp_ns;
		for (int i = 0; i < ovrn::HandJoints::kCount; i++) {
			out->joint[i].position = {s.pose[i][0], s.pose[i][1], s.pose[i][2]};
			out->joint[i].orientation = {s.pose[i][3], s.pose[i][4], s.pose[i][5], s.pose[i][6]};
			out->joint_valid[i] = s.valid[i] != 0;
			out->radius[i] = s.radius[i];
		}
		return true;
	}
	bool
	handGestures(ovrn::Hand hand, ovrn::gest::Sample *out) override
	{
		*out = ovrn::gest::Sample{};
		struct sim_gestures g;
		if (!sim_ht_gestures(hand == ovrn::Hand::Left ? 0 : 1, &g)) {
			return false;
		}
		// HAND_SIM_GESTURE_LOG=<file>: every gesture read: replay time, hand, valid, the measurement's frame time.
		static FILE *glog = getenv("HAND_SIM_GESTURE_LOG") ? fopen(getenv("HAND_SIM_GESTURE_LOG"), "w") : nullptr;
		if (glog != nullptr) {
			fprintf(glog, "%lld,%d,%d,%lld,%.5f,%d,%.4f,%u\n", (long long)mono_ns(), hand == ovrn::Hand::Left ? 0 : 1,
			        g.valid, (long long)g.timestamp_ns, g.pinch_distance_normalized, g.category, g.category_score,
			        (unsigned)g.extended_mask);
		}
		out->valid = g.valid != 0;
		out->timestamp_ns = g.timestamp_ns;
		out->confidence = g.confidence;
		out->pinch_distance_normalized = g.pinch_distance_normalized;
		out->extended_mask = g.extended_mask;
		out->category = g.category;
		out->category_score = g.category_score;
		return true;
	}
	ovrn::WorldTransform worldFromDriver() override { return {}; } // the head poses are already SteamVR's world
	uint64_t universe() override { return 0; }
	std::string chaperoneJson() override { return {}; }
	void runFrame(int64_t) override {}
};

} // namespace

int
main(int argc, char **argv)
{
	if (argc < 5) {
		fprintf(stderr, "usage: %s recording.pv2c psvr2-calibration-<serial>.bin vrhandlog.csv out.csv\n", argv[0]);
		return 1;
	}
	if (!load_vrhandlog(argv[3])) {
		fprintf(stderr, "can't read head poses from %s\n", argv[3]);
		return 1;
	}
	FakeContext ctx;
	if (vr::InitServerDriverContext(&ctx) != VRInitError_None) {
		return 1;
	}
	if (!sim_ht_init(argv[2], head_at_replay)) {
		return 1;
	}
	ReplayBackend backend;
	ovrn::HandDevice left(&backend, ovrn::Hand::Left), right(&backend, ovrn::Hand::Right);
	ovrn::HandDevice *hands[2] = {&left, &right};
	for (int i = 0; i < 2; i++) {
		if (hands[i]->Activate(1 + i) != VRInitError_None) {
			fprintf(stderr, "hand %d: Activate failed\n", i);
			return 1;
		}
	}

	FILE *in = fopen(argv[1], "rb");
	char magic[8];
	if (in == nullptr || fread(magic, 1, 8, in) != 8 || memcmp(magic, "PV2C", 4)) {
		fprintf(stderr, "not a PSVR2_RECORD_CAMERA file: %s\n", argv[1]);
		return 1;
	}
	FILE *out = fopen(argv[4], "w");
	fprintf(out, "t_s");
	for (const char *s : {"left", "right"}) {
		fprintf(out, ",%s_valid,%s_px,%s_py,%s_pz,%s_trigger,%s_trigger_click,%s_grip_click,%s_system_click", s, s, s,
		        s, s, s, s, s);
		fprintf(out, ",%s_qx,%s_qy,%s_qz,%s_qw,%s_mp_valid,%s_mp_category,%s_mp_score,%s_mp_pinch", s, s, s, s, s, s,
		        s, s);
	}
	fprintf(out, "\n");

	static uint8_t buf[1040640];
	int64_t first_rec = 0, start = 0;
	size_t next_frame = 0; // the next vrhandlog frame to sample
	int pushed = 0, sampled = 0;
	for (;;) {
		uint64_t ns;
		uint32_t len, dropped;
		if (fread(&ns, 8, 1, in) != 1 || fread(&len, 4, 1, in) != 1 || fread(&dropped, 4, 1, in) != 1 ||
		    len > sizeof(buf) || fread(buf, 1, len, in) != len) {
			break;
		}
		if (first_rec == 0) {
			first_rec = (int64_t)ns;
			// Mercury loads when the first frames come; give it time as driver_ht_replay does.
			for (int w = 0; w < 300; w++) {
				struct sim_joints s;
				sim_ht_joints(1, mono_ns(), &s);
				struct timespec d = {0, 20 * 1000 * 1000};
				nanosleep(&d, nullptr);
			}
			start = mono_ns();
			// The recording's frame times are the session's monotonic clock.
			g_replayToSession = first_rec - start;
			// Skip SteamVR frames before the recording.
			while (next_frame < g_frame_t.size() &&
			       g_t0 + (int64_t)(g_frame_t[next_frame] * 1e9) < first_rec) {
				next_frame++;
			}
		}
		const int64_t due = start + ((int64_t)ns - first_rec);
		for (;;) {
			const int64_t now = mono_ns();
			// SteamVR frames due by now: update the hands as the plugin does (pose, then RunFrame's inputs), sample.
			while (next_frame < g_frame_t.size() &&
			       g_t0 + (int64_t)(g_frame_t[next_frame] * 1e9) - g_replayToSession <= now) {
				const int64_t at = g_t0 + (int64_t)(g_frame_t[next_frame] * 1e9) - g_replayToSession;
				fprintf(out, "%.6f", g_frame_t[next_frame]);
				g_frameT = g_frame_t[next_frame];
				for (int h = 0; h < 2; h++) {
					hands[h]->updateInputs(at);
					hands[h]->updatePose(at);
					DriverPose_t p;
					{
						std::lock_guard<std::mutex> l(g_mutex);
						p = g_lastPose[1 + h];
					}
					const uint64_t c = 1000 + 1 + h;
					fprintf(out, ",%d,%.5f,%.5f,%.5f,%.3f,%d,%d,%d", p.poseIsValid ? 1 : 0, p.vecPosition[0],
					        p.vecPosition[1], p.vecPosition[2], component(c, "/input/trigger/value"),
					        component(c, "/input/trigger/click") > 0.5f, component(c, "/input/grip/click") > 0.5f,
					        component(c, "/input/system/click") > 0.5f);
					struct sim_gestures sg;
					const int have_g = sim_ht_gestures(h, &sg);
					fprintf(out, ",%.6f,%.6f,%.6f,%.6f,%d,%d,%.3f,%.4f", p.qRotation.x, p.qRotation.y, p.qRotation.z,
					        p.qRotation.w, have_g && sg.valid, have_g ? sg.category : -1, have_g ? sg.category_score : 0.0f,
					        have_g ? sg.pinch_distance_normalized : 0.0f);
				}
				fprintf(out, "\n");
				next_frame++;
				sampled++;
			}
			if (now >= due) {
				break;
			}
			struct timespec d = {0, 300 * 1000};
			nanosleep(&d, nullptr);
		}
		// HAND_SIM_KEEP_EVERY=N: give the tracker only every Nth transfer (to see what a lower frame rate does).
		static const int keep_every = getenv("HAND_SIM_KEEP_EVERY") ? atoi(getenv("HAND_SIM_KEEP_EVERY")) : 1;
		static int seen = 0;
		if (keep_every <= 1 || seen++ % keep_every == 0) {
			const int64_t push_ns = mono_ns();
			sim_ht_push(buf, len, push_ns);
			// HAND_SIM_PUSH_LOG=<file>: each transfer's index and the time it was given to the tracker.
			static FILE *plog = getenv("HAND_SIM_PUSH_LOG") ? fopen(getenv("HAND_SIM_PUSH_LOG"), "w") : nullptr;
			if (plog != nullptr) {
				fprintf(plog, "%d,%lld\n", pushed, (long long)push_ns);
			}
			pushed++;
		}
	}
	fclose(out);
	fprintf(stderr, "hand_sim: %d camera transfers, %d SteamVR frames sampled\n", pushed, sampled);
	left.Deactivate();
	right.Deactivate();
	sim_ht_destroy();
	return 0;
}
