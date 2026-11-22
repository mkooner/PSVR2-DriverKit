// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Offline check of the native PS VR2 SteamVR driver (st_ovrn): loads the built driver_monado_psvr2.so as
 *         vrserver does (dlopen + HmdDriverFactory) and plays SteamVR's side of the driver interface.
 *
 * What it can't see: anything real SteamVR does with the poses and properties (prediction, reprojection, the display).
 * It checks the driver's own behaviour: lifecycle, what it reports, pose cadence and content, clean shutdown.
 */

#include "openvr_driver.h"

#include <algorithm>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <dlfcn.h>
#include <sys/stat.h>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace vr;

namespace {

int g_fails = 0;

void
check(bool ok, const char *what)
{
	printf("%s: %s\n", ok ? "PASS" : "FAIL", what);
	g_fails += ok ? 0 : 1;
}

//! Whether a PS VR2 headset (054c:0cde) is on USB, read from sysfs (nothing is opened).
bool
psvr2_on_usb()
{
	for (int bus = 1; bus < 64; bus++) {
		for (int port = 0; port < 64; port++) {
			char p[96];
			snprintf(p, sizeof(p), "/sys/bus/usb/devices/%d-%d/idVendor", bus, port);
			FILE *v = fopen(p, "r");
			if (v == nullptr) {
				continue;
			}
			char vid[8] = {0}, pid[8] = {0};
			bool ok = fscanf(v, "%7s", vid) == 1;
			fclose(v);
			snprintf(p, sizeof(p), "/sys/bus/usb/devices/%d-%d/idProduct", bus, port);
			FILE *d = fopen(p, "r");
			if (d != nullptr) {
				ok = ok && fscanf(d, "%7s", pid) == 1;
				fclose(d);
			}
			if (ok && strcmp(vid, "054c") == 0 && strcmp(pid, "0cde") == 0) {
				return true;
			}
		}
	}
	return false;
}

int64_t
now_ns()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

struct Added
{
	std::string serial;
	ETrackedDeviceClass cls;
	ITrackedDeviceServerDriver *driver;
};

struct PoseLog
{
	std::vector<int64_t> times;
	DriverPose_t last = {};
	DriverPose_t prev = {}; //!< the one before, with their arrival times, to check velocities against the motion
	int64_t last_ns = 0, prev_ns = 0;
};

std::mutex g_mutex;
std::vector<Added> g_added;
std::map<uint32_t, PoseLog> g_poses;
std::map<uint32_t, int> g_eyeToHead;
HmdMatrix34_t g_eyeMatrix[2] = {};
const char *g_dumpPath = nullptr; //!< --real --dump <file>: the display as the driver describes it (vrdisplaydump format)
std::map<uint64_t, std::map<int, std::vector<uint8_t>>> g_props; //!< container -> property -> bytes

std::string g_logText; //!< everything the driver logged

struct FakeLog : IVRDriverLog
{
	void
	Log(const char *m) override
	{
		fputs(m, stdout);
		g_logText += m;
	}
};

struct FakeHost : IVRServerDriverHost
{
	bool
	TrackedDeviceAdded(const char *serial, ETrackedDeviceClass cls, ITrackedDeviceServerDriver *driver) override
	{
		std::lock_guard<std::mutex> l(g_mutex);
		g_added.push_back({serial, cls, driver});
		return true;
	}
	void
	TrackedDevicePoseUpdated(uint32_t which, const DriverPose_t &pose, uint32_t size) override
	{
		std::lock_guard<std::mutex> l(g_mutex);
		if (size != sizeof(DriverPose_t)) {
			g_fails++;
		}
		PoseLog &pl = g_poses[which];
		pl.times.push_back(now_ns());
		pl.prev = pl.last;
		pl.prev_ns = pl.last_ns;
		pl.last = pose;
		pl.last_ns = pl.times.back();
	}
	void VsyncEvent(double) override {}
	void VendorSpecificEvent(uint32_t, EVREventType, const VREvent_Data_t &, double) override {}
	bool IsExiting() override { return false; }
	bool PollNextEvent(VREvent_t *event, uint32_t size) override;
	void GetRawTrackedDevicePoses(float, TrackedDevicePose_t *, uint32_t) override {}
	void RequestRestart(const char *, const char *, const char *, const char *) override {}
	uint32_t GetFrameTimings(Compositor_FrameTiming *, uint32_t) override { return 0; }
	void
	SetDisplayEyeToHead(uint32_t which, const HmdMatrix34_t &left, const HmdMatrix34_t &right) override
	{
		std::lock_guard<std::mutex> l(g_mutex);
		g_eyeToHead[which]++;
		g_eyeMatrix[0] = left;
		g_eyeMatrix[1] = right;
	}
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
	WritePropertyBatch(PropertyContainerHandle_t container, PropertyWrite_t *batch, uint32_t n) override
	{
		std::lock_guard<std::mutex> l(g_mutex);
		for (uint32_t i = 0; i < n; i++) {
			const uint8_t *b = (const uint8_t *)batch[i].pvBuffer;
			g_props[container][batch[i].prop] = std::vector<uint8_t>(b, b + batch[i].unBufferSize);
			batch[i].eError = TrackedProp_Success;
		}
		return TrackedProp_Success;
	}
	const char *GetPropErrorNameFromEnum(ETrackedPropertyError) override { return "error"; }
	PropertyContainerHandle_t
	TrackedDeviceToPropertyContainer(TrackedDeviceIndex_t device) override
	{
		return 1000 + device;
	}
};

//! Boolean settings the driver wrote ("section/key"), as SteamVR's settings file would hold them.
std::map<std::string, bool> g_bools;
//! Integer settings the driver wrote ("section/key").
std::map<std::string, int32_t> g_ints;

struct FakeSettings : IVRSettings
{
	const char *GetSettingsErrorNameFromEnum(EVRSettingsError) override { return "error"; }
	void
	SetBool(const char *section, const char *key, bool value, EVRSettingsError *) override
	{
		std::lock_guard<std::mutex> l(g_mutex);
		g_bools[std::string(section) + "/" + key] = value;
	}
	void
	SetInt32(const char *section, const char *key, int32_t value, EVRSettingsError *) override
	{
		std::lock_guard<std::mutex> l(g_mutex);
		g_ints[std::string(section) + "/" + key] = value;
	}
	void SetFloat(const char *, const char *, float, EVRSettingsError *) override {}
	void SetString(const char *, const char *, const char *, EVRSettingsError *) override {}
	bool
	GetBool(const char *section, const char *key, EVRSettingsError *e) override
	{
		std::lock_guard<std::mutex> l(g_mutex);
		auto it = g_bools.find(std::string(section) + "/" + key);
		if (e != nullptr) {
			*e = it != g_bools.end() ? VRSettingsError_None : VRSettingsError_UnsetSettingHasNoDefault;
		}
		return it != g_bools.end() && it->second;
	}
	int32_t
	GetInt32(const char *section, const char *key, EVRSettingsError *e) override
	{
		std::lock_guard<std::mutex> l(g_mutex);
		auto it = g_ints.find(std::string(section) + "/" + key);
		if (e != nullptr) {
			*e = it != g_ints.end() ? VRSettingsError_None : VRSettingsError_UnsetSettingHasNoDefault;
		}
		return it != g_ints.end() ? it->second : 0;
	}
	float
	GetFloat(const char *, const char *, EVRSettingsError *e) override
	{
		if (e != nullptr) {
			*e = VRSettingsError_UnsetSettingHasNoDefault;
		}
		return 0;
	}
	void
	GetString(const char *, const char *, char *value, uint32_t len, EVRSettingsError *e) override
	{
		if (len > 0) {
			value[0] = '\0';
		}
		if (e != nullptr) {
			*e = VRSettingsError_UnsetSettingHasNoDefault;
		}
	}
	void RemoveSection(const char *, EVRSettingsError *) override {}
	void
	RemoveKeyInSection(const char *section, const char *key, EVRSettingsError *) override
	{
		std::lock_guard<std::mutex> l(g_mutex);
		g_bools.erase(std::string(section) + "/" + key);
		g_ints.erase(std::string(section) + "/" + key);
	}
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

//! Input components as SteamVR keeps them: created per container, updated by handle.
struct Component
{
	uint64_t container;
	std::string path;
	int kind; //!< 0 boolean, 1 scalar, 2 haptic
	float value;
	int updates;
	float min_value = 1e9f, max_value = -1e9f;
};
std::vector<Component> g_components; //!< handle = index + 1

//! A skeleton component: its updates per motion range, and the first problem seen in a pose (empty: all valid).
struct Skeleton
{
	PropertyContainerHandle_t container;
	std::string path, skeleton_path;
	EVRSkeletalTrackingLevel level = VRSkeletalTracking_Estimated;
	int with_controller = 0, without_controller = 0;
	std::string problem;
};
std::vector<Skeleton> g_skeletons; //!< handle = 100000 + index

//! The eye-tracking component (handle 200000): its path and the updates SteamVR got.
struct EyeComponent
{
	PropertyContainerHandle_t container = 0;
	std::string path;
	int updates = 0, valid = 0, invalid = 0, mixed_flags = 0;
	VREyeTrackingData_t last{}, last_valid{};
	double last_offset = 0, last_valid_offset = 0;
};
EyeComponent g_eye;

std::string
bone_problem(const VRBoneTransform_t *t, uint32_t n)
{
	if (n != 31) {
		return "bone count " + std::to_string(n);
	}
	for (uint32_t i = 0; i < n; i++) {
		const float q = t[i].orientation.w * t[i].orientation.w + t[i].orientation.x * t[i].orientation.x +
		                t[i].orientation.y * t[i].orientation.y + t[i].orientation.z * t[i].orientation.z;
		if (!std::isfinite(q) || std::fabs(q - 1.0f) > 2e-3f) {
			return "bone " + std::to_string(i) + " rotation";
		}
		for (int k = 0; k < 3; k++) {
			if (!std::isfinite(t[i].position.v[k]) || std::fabs(t[i].position.v[k]) > 0.25f) {
				return "bone " + std::to_string(i) + " position";
			}
		}
	}
	return {};
}

struct FakeInput : IVRDriverInput
{
	EVRInputError
	add(PropertyContainerHandle_t c, const char *name, int kind, VRInputComponentHandle_t *h)
	{
		std::lock_guard<std::mutex> l(g_mutex);
		g_components.push_back({c, name, kind, 0.0f, 0});
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
		g_components[h - 1].updates++;
		g_components[h - 1].min_value = std::min(g_components[h - 1].min_value, v);
		g_components[h - 1].max_value = std::max(g_components[h - 1].max_value, v);
		return VRInputError_None;
	}
	EVRInputError CreateBooleanComponent(PropertyContainerHandle_t c, const char *n, VRInputComponentHandle_t *h) override { return add(c, n, 0, h); }
	EVRInputError UpdateBooleanComponent(VRInputComponentHandle_t h, bool v, double) override { return set(h, v ? 1.0f : 0.0f); }
	EVRInputError CreateScalarComponent(PropertyContainerHandle_t c, const char *n, VRInputComponentHandle_t *h, EVRScalarType, EVRScalarUnits) override { return add(c, n, 1, h); }
	EVRInputError UpdateScalarComponent(VRInputComponentHandle_t h, float v, double) override { return set(h, v); }
	EVRInputError CreateHapticComponent(PropertyContainerHandle_t c, const char *n, VRInputComponentHandle_t *h) override { return add(c, n, 2, h); }
	EVRInputError
	CreateSkeletonComponent(PropertyContainerHandle_t c, const char *name, const char *skeleton, const char *, EVRSkeletalTrackingLevel level, const VRBoneTransform_t *, uint32_t, VRInputComponentHandle_t *h) override
	{
		std::lock_guard<std::mutex> l(g_mutex);
		g_skeletons.push_back({c, name, skeleton, level});
		*h = 100000 + g_skeletons.size() - 1;
		return VRInputError_None;
	}
	EVRInputError
	UpdateSkeletonComponent(VRInputComponentHandle_t h, EVRSkeletalMotionRange range, const VRBoneTransform_t *t, uint32_t n) override
	{
		std::lock_guard<std::mutex> l(g_mutex);
		if (h < 100000 || h - 100000 >= g_skeletons.size()) {
			return VRInputError_InvalidHandle;
		}
		Skeleton &s = g_skeletons[h - 100000];
		(range == VRSkeletalMotionRange_WithController ? s.with_controller : s.without_controller)++;
		if (s.problem.empty()) {
			s.problem = bone_problem(t, n);
		}
		return VRInputError_None;
	}
	EVRInputError
	CreatePoseComponent(PropertyContainerHandle_t c, const char *n, VRInputComponentHandle_t *h) override
	{
		return add(c, n, 3, h);
	}
	EVRInputError
	UpdatePoseComponent(VRInputComponentHandle_t h, const HmdMatrix34_t *m, double) override
	{
		return set(h, m != nullptr ? m->m[2][3] : 0.0f); // the offset's z (tip: ahead of the palm, negative)
	}
	EVRInputError
	CreateEyeTrackingComponent(PropertyContainerHandle_t c, const char *name, VRInputComponentHandle_t *h) override
	{
		std::lock_guard<std::mutex> l(g_mutex);
		g_eye.container = c;
		g_eye.path = name;
		*h = 200000;
		return VRInputError_None;
	}
	EVRInputError
	UpdateEyeTrackingComponent(VRInputComponentHandle_t h, const VREyeTrackingData_t *d, double offset) override
	{
		std::lock_guard<std::mutex> l(g_mutex);
		if (h != 200000) {
			return VRInputError_InvalidHandle;
		}
		g_eye.updates++;
		(d->bValid && d->bActive && d->bTracked ? g_eye.valid : g_eye.invalid)++;
		if (d->bValid != d->bActive || d->bValid != d->bTracked) {
			g_eye.mixed_flags++;
		}
		g_eye.last = *d;
		g_eye.last_offset = offset;
		if (d->bValid) {
			g_eye.last_valid = *d;
			g_eye.last_valid_offset = offset;
		}
		return VRInputError_None;
	}
};

std::vector<VREvent_t> g_events; //!< events for the driver's next PollNextEvent calls

bool
FakeHost::PollNextEvent(VREvent_t *event, uint32_t size)
{
	std::lock_guard<std::mutex> l(g_mutex);
	if (g_events.empty() || size < sizeof(VREvent_t)) {
		return false;
	}
	*event = g_events.front();
	g_events.erase(g_events.begin());
	return true;
}

FakeSettings g_settings;
FakeDriverManager g_driverManager;
FakeResources g_resources;
FakeInput g_input;
FakeLog g_log;
FakeHost g_host;
FakeProperties g_properties;

struct FakeContext : IVRDriverContext
{
	void *
	GetGenericInterface(const char *version, EVRInitError *error) override
	{
		if (error != nullptr) {
			*error = VRInitError_None;
		}
		if (!strcmp(version, IVRServerDriverHost_Version)) {
			return static_cast<IVRServerDriverHost *>(&g_host);
		}
		if (!strcmp(version, IVRDriverLog_Version)) {
			return static_cast<IVRDriverLog *>(&g_log);
		}
		if (!strcmp(version, IVRProperties_Version)) {
			return static_cast<IVRProperties *>(&g_properties);
		}
		if (!strcmp(version, IVRDriverInput_Version)) {
			return static_cast<IVRDriverInput *>(&g_input);
		}
		if (!strcmp(version, IVRSettings_Version)) {
			return static_cast<IVRSettings *>(&g_settings);
		}
		if (!strcmp(version, IVRDriverManager_Version)) {
			return static_cast<IVRDriverManager *>(&g_driverManager);
		}
		if (!strcmp(version, IVRResources_Version)) {
			return static_cast<IVRResources *>(&g_resources);
		}
		if (error != nullptr) {
			*error = VRInitError_Init_InterfaceNotFound;
		}
		return nullptr;
	}
	DriverHandle_t GetDriverHandle() override { return 1; }
};

template <typename T>
bool
prop(uint32_t device, int id, T *out)
{
	std::lock_guard<std::mutex> l(g_mutex);
	auto c = g_props.find(1000 + device);
	if (c == g_props.end() || c->second.find(id) == c->second.end() || c->second[id].size() != sizeof(T)) {
		return false;
	}
	memcpy(out, c->second[id].data(), sizeof(T));
	return true;
}

//! A string property ("" if not set).
std::string
sprop(uint32_t device, int id)
{
	std::lock_guard<std::mutex> l(g_mutex);
	auto c = g_props.find(1000 + device);
	if (c == g_props.end() || c->second.find(id) == c->second.end()) {
		return {};
	}
	const std::vector<uint8_t> &b = c->second[id];
	return std::string(b.begin(), std::find(b.begin(), b.end(), (uint8_t)0));
}

size_t
pose_count(uint32_t device)
{
	std::lock_guard<std::mutex> l(g_mutex);
	return g_poses[device].times.size();
}

void
sleep_ms(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

} // namespace

int
main(int argc, char **argv)
{
	// --monado: Monado's simulated headset through the Monado backend (no hardware is opened); default: the driver's
	// own simulated backend.
	// --real: the Monado backend on whatever headset is plugged in (opens it over USB; no display, no SteamVR).
	const bool real = argc > 1 && !strcmp(argv[1], "--real");
	if (real && argc > 3 && !strcmp(argv[2], "--dump")) {
		g_dumpPath = argv[3];
	}
	// --room: Monado's simulated headset with the room calibration on, against a temporary SteamVR config folder
	// (XDG_CONFIG_HOME points at a scratch folder, so the user's SteamVR files are never touched).
	const bool room = argc > 1 && !strcmp(argv[1], "--room");
	// --no-headset: the Monado backend with no PS VR2 plugged in (skipped if one is): the driver must refuse to start
	// rather than give SteamVR Monado's fallback "Simulated HMD".
	const bool no_headset = argc > 1 && !strcmp(argv[1], "--no-headset");
	const bool monado = real || room || no_headset || (argc > 1 && !strcmp(argv[1], "--monado"));
	// --predicted: the driver-predicted, zero-velocity pose model (OVRN_POSE_MODEL=predicted).
	const bool predicted = argc > 1 && !strcmp(argv[1], "--predicted");
	if (predicted) {
		setenv("OVRN_POSE_MODEL", "predicted", 1);
	}
	// The room calibration prepares a chaperone file: not the user's from a test (a fixed play area instead, or with
	// --room a scratch config folder).
	std::string room_dir, room_steam_config;
	if (room) {
		char tmpl[] = "/tmp/ovrn_room_XXXXXX";
		if (mkdtemp(tmpl) == nullptr) {
			printf("FAIL: no scratch folder\n");
			return 1;
		}
		room_dir = tmpl;
		room_steam_config = room_dir + "/steamconfig";
		mkdir((room_dir + "/openvr").c_str(), 0755);
		mkdir(room_steam_config.c_str(), 0755);
		// DriverKit's settings file, which the driver reads itself (hand tracking switched on there).
		mkdir((room_dir + "/PSVR2-DriverKit").c_str(), 0755);
		FILE *settings = fopen((room_dir + "/PSVR2-DriverKit/settings.conf").c_str(), "w");
		fprintf(settings, "# test\nHAND_TRACKING=1\n");
		fclose(settings);
		FILE *f = fopen((room_dir + "/openvr/openvrpaths.vrpath").c_str(), "w");
		fprintf(f, "{\"config\":[\"%s\"],\"version\":1}\n", room_steam_config.c_str());
		fclose(f);
		// As on a real PC: SteamVR's file with another headset's room (universe 1234), and the driver's old file
		// (until 2026-11-19 it kept the rooms there) with a room drawn in Room Setup, to be moved into SteamVR's.
		const char *quad = "[[[-2,0,-1],[-2,2.43,-1],[-2,2.43,1],[-2,0,1]],[[-2,0,1],[-2,2.43,1],[2,2.43,1],[2,0,1]],"
		                   "[[2,0,1],[2,2.43,1],[2,2.43,-1],[2,0,-1]],[[2,0,-1],[2,2.43,-1],[-2,2.43,-1],[-2,0,-1]]]";
		auto universe = [&](const char *id) {
			return std::string("{\"universeID\":\"") + id + "\",\"play_area\":[4,2],\"collision_bounds\":" + quad +
			       ",\"seated\":{\"translation\":[0,0,0],\"yaw\":0},\"standing\":{\"translation\":[0,0,0],\"yaw\":0}}";
		};
		f = fopen((room_steam_config + "/chaperone_info.vrchap").c_str(), "w");
		fprintf(f, "{\"jsonid\":\"chaperone_info\",\"version\":5,\"universes\":[%s]}\n", universe("1234").c_str());
		fclose(f);
		mkdir((room_steam_config + "/monado_psvr2").c_str(), 0755);
		f = fopen((room_steam_config + "/monado_psvr2/chaperone_info.vrchap").c_str(), "w");
		fprintf(f, "{\"jsonid\":\"chaperone_info\",\"version\":5,\"universes\":[%s]}\n", universe("5022").c_str());
		fclose(f);
		setenv("XDG_CONFIG_HOME", room_dir.c_str(), 1);
		unsetenv("OVRD_ROOM");
	} else {
		setenv("OVRD_ROOM", "0", 1);
	}
	if (real || no_headset) {
		// nothing more: Monado probes the hardware (and isn't asked for its simulated system)
		unsetenv("SIMULATED_ENABLE");
	} else if (monado) {
		setenv("SIMULATED_ENABLE", "true", 1);
	} else {
		setenv("OVRN_BACKEND", "sim", 1);
	}
	if (argc > 1 && !strcmp(argv[1], "--late")) {
		setenv("OVRN_SIM_LATE_LEFT", "1", 1);
	}
	const bool hands = argc > 1 && !strcmp(argv[1], "--hands");
	if (hands) {
		setenv("OVRN_SIM_HANDS", "1", 1);
	}
	const char *path = OVRN_DRIVER_PATH;
	void *lib = dlopen(path, RTLD_NOW | RTLD_LOCAL);
	if (lib == nullptr) {
		printf("FAIL: dlopen %s: %s\n", path, dlerror());
		return 1;
	}
	typedef void *(*Factory)(const char *, int *);
	Factory factory = (Factory)dlsym(lib, "HmdDriverFactory");
	check(factory != nullptr, "the driver exports HmdDriverFactory");
	if (factory == nullptr) {
		return 1;
	}
	int rc = 0;
	check(factory("IVRNotAThing_001", &rc) == nullptr && rc == VRInitError_Init_InterfaceNotFound,
	      "an unknown interface is refused with InterfaceNotFound");
	IServerTrackedDeviceProvider *provider =
	    (IServerTrackedDeviceProvider *)factory(IServerTrackedDeviceProvider_Version, &rc);
	check(provider != nullptr, "the driver returns its IServerTrackedDeviceProvider");
	if (provider == nullptr) {
		return 1;
	}
	FakeContext context;

	if (no_headset) {
		if (psvr2_on_usb()) {
			printf("skipped: a PS VR2 is on USB (this mode needs it unplugged, it would open it)\n");
			return 0;
		}
		check(provider->Init(&context) != VRInitError_None, "no headset: Init refuses to start");
		{
			std::lock_guard<std::mutex> l(g_mutex);
			check(g_added.empty(), "no headset: no device is registered (no fallback Simulated HMD)");
			check(g_logText.find("ovrn: no PS VR2 headset found") != std::string::npos,
			      "no headset: the driver logs why it reports no headset");
		}
		printf("%s\n", g_fails ? "FAILED" : "all checks passed");
		return g_fails ? 1 : 0;
	}

	// --camera-off / --camera-noimages (DriverKit's CAMERA setting, for the Intel Arc compositor crash): off leaves the
	// user's SteamVR camera settings alone and offers SteamVR no camera; noimages sets SteamVR's camera up as "on" does.
	if (argc > 1 && (!strcmp(argv[1], "--camera-off") || !strcmp(argv[1], "--camera-noimages"))) {
		const bool off = !strcmp(argv[1], "--camera-off");
		setenv("OVRN_CAMERA_MODE", off ? "off" : "noimages", 1);
		{
			std::lock_guard<std::mutex> l(g_mutex);
			g_ints["camera/roomViewStyle"] = 2;
			g_bools["camera/enableCamera"] = false;
		}
		check(provider->Init(&context) == VRInitError_None, "Init succeeds");
		{
			std::lock_guard<std::mutex> l(g_mutex);
			if (off) {
				check(g_ints["camera/roomViewStyle"] == 2 && !g_bools["camera/enableCamera"] &&
				          !g_ints.count("camera/roomView") && !g_bools.count("driver_monado_psvr2/seeThroughSettingsSet"),
				      "camera off: the user's SteamVR camera settings left alone");
			} else {
				check(g_ints["camera/roomViewStyle"] == 4 && g_bools["camera/enableCamera"],
				      "camera noimages: SteamVR's camera set up as with on");
			}
		}
		std::vector<Added> added;
		{
			std::lock_guard<std::mutex> l(g_mutex);
			added = g_added;
		}
		for (uint32_t i = 0; i < added.size(); i++) {
			added[i].driver->Activate(i);
		}
		{
			std::lock_guard<std::mutex> l(g_mutex);
			check(g_logText.find(off ? "ovrn: headset cameras: off (OVRN_CAMERA_MODE)"
			                         : "ovrn: headset cameras: noimages (OVRN_CAMERA_MODE)") != std::string::npos,
			      "camera mode: logged at the headset's Activate");
		}
		provider->Cleanup();
		{
			std::lock_guard<std::mutex> l(g_mutex);
			check(g_ints["camera/roomViewStyle"] == 2 && !g_bools["camera/enableCamera"],
			      "camera mode: the user's camera settings as they were after Cleanup");
		}
		check(dlclose(lib) == 0, "the driver unloads");
		printf("%s\n", g_fails ? "FAILED" : "all checks passed");
		return g_fails ? 1 : 0;
	}

	// --see-through: Sony-style see-through. The user's camera settings (style 2 "Colored Ghosts", camera on, no
	// roomView key) are saved and the opaque view set for the session; SteamVR's Room View shown / hidden events turn
	// the game pause and reduced rendering on / off; Cleanup puts the user's settings back exactly.
	if (argc > 1 && !strcmp(argv[1], "--see-through")) {
		{
			std::lock_guard<std::mutex> l(g_mutex);
			g_ints["camera/roomViewStyle"] = 2;
			g_bools["camera/enableCamera"] = true;
		}
		check(provider->Init(&context) == VRInitError_None, "Init succeeds");
		{
			std::lock_guard<std::mutex> l(g_mutex);
			check(g_ints["camera/roomViewStyle"] == 4 && g_bools["camera/enableCamera"] && g_ints.count("camera/roomView") &&
			          g_ints["camera/roomView"] == 1,
			      "see-through: SteamVR's camera on with the opaque view for the session");
			check(g_bools["driver_monado_psvr2/seeThroughSettingsSet"], "see-through: noted in the driver's own section");
		}
		std::vector<Added> added;
		{
			std::lock_guard<std::mutex> l(g_mutex);
			added = g_added;
		}
		for (uint32_t i = 0; i < added.size(); i++) {
			added[i].driver->Activate(i);
		}
		bool pause = true, reduced = true;
		check(!prop(0, Prop_DriverRequestsApplicationPause_Bool, &pause) || !pause, "see-through: no pause at the start");
		for (int type : {(int)VREvent_RoomViewShown, (int)VREvent_RoomViewHidden}) {
			VREvent_t ev = {};
			ev.eventType = (uint32_t)type;
			{
				std::lock_guard<std::mutex> l(g_mutex);
				g_events.push_back(ev);
			}
			provider->RunFrame();
			const bool want = type == VREvent_RoomViewShown;
			check(prop(0, Prop_DriverRequestsApplicationPause_Bool, &pause) && pause == want &&
			          prop(0, Prop_DriverRequestsReducedRendering_Bool, &reduced) && reduced == want,
			      want ? "see-through on: SteamVR asked to pause the game and render less"
			           : "see-through off: the game runs again at full rendering");
		}
		provider->Cleanup();
		{
			std::lock_guard<std::mutex> l(g_mutex);
			check(g_ints.count("camera/roomViewStyle") && g_ints["camera/roomViewStyle"] == 2 && g_bools["camera/enableCamera"] &&
			          !g_ints.count("camera/roomView"),
			      "see-through: the user's camera settings put back exactly at Cleanup");
			bool notes = false;
			for (auto &kv : g_bools) {
				notes = notes || kv.first.find("driver_monado_psvr2/seeThrough") == 0;
			}
			for (auto &kv : g_ints) {
				notes = notes || kv.first.find("driver_monado_psvr2/seeThrough") == 0;
			}
			check(!notes, "see-through: the driver's notes taken out again");
		}
		check(dlclose(lib) == 0, "the driver unloads");
		printf("%s\n", g_fails ? "FAILED" : "all checks passed");
		return g_fails ? 1 : 0;
	}

	/*
	 * --hands: the controllers off and two tracked bare hands (the simulated backend's hand tracking). Each hand is
	 * its own device (playstation_vr2_hand) with a full skeleton updated every frame in both motion ranges, pinch
	 * on the trigger, its palm as the pose, which stays put while the head moves.
	 */
	if (hands) {
		check(provider->Init(&context) == VRInitError_None, "hands: Init succeeds");
		std::vector<Added> added;
		{
			std::lock_guard<std::mutex> l(g_mutex);
			added = g_added;
		}
		const bool three = added.size() == 3;
		check(three && added[0].cls == TrackedDeviceClass_HMD && added[1].cls == TrackedDeviceClass_Controller &&
		          added[2].cls == TrackedDeviceClass_Controller,
		      "hands: the headset and two hand devices (controller class), no Sense controllers");
		if (!three) {
			provider->Cleanup();
			printf("FAILED\n");
			return 1;
		}
		for (uint32_t i = 0; i < added.size(); i++) {
			check(added[i].driver->Activate(i) == VRInitError_None, "hands: Activate succeeds");
		}
		int32_t role_l = 0, role_r = 0;
		check(prop(1, Prop_ControllerRoleHint_Int32, &role_l) && role_l == TrackedControllerRole_LeftHand &&
		          prop(2, Prop_ControllerRoleHint_Int32, &role_r) && role_r == TrackedControllerRole_RightHand,
		      "hands: left and right hand roles");
		check(sprop(1, Prop_ControllerType_String) == "playstation_vr2_hand" &&
		          sprop(2, Prop_ControllerType_String) == "playstation_vr2_hand" &&
		          sprop(1, Prop_InputProfilePath_String) == "{monado_psvr2}/input/playstation_vr2_hand_profile.json",
		      "hands: their own device type and input profile (not another controller's)");
		check(added[1].serial != added[2].serial && added[1].serial.find("-hand-left") != std::string::npos,
		      "hands: their own serials");
		check(sprop(1, Prop_NamedIconPathDeviceReady_String) == "{vrlink}/icons/left_handtracking_ready.png" &&
		          sprop(2, Prop_NamedIconPathDeviceOff_String) == "{vrlink}/icons/right_handtracking_off.png",
		      "hands: SteamVR's hand tracking icons, not a controller's");
		check(sprop(1, Prop_RenderModelName_String) == "{vrlink}/rendermodels/shuttlecock",
		      "hands: the render model SteamVR's own tracked hands use (a pointer marker, no controller drawn)");
		DriverPose_t first = {};
		for (int i = 0; i < 40; i++) {
			provider->RunFrame();
			sleep_ms(3);
			if (i == 2) {
				std::lock_guard<std::mutex> l(g_mutex);
				first = g_poses[1].last;
			}
		}
		{
			std::lock_guard<std::mutex> l(g_mutex);
			bool paths = g_skeletons.size() == 2, full = true, valid = true, ranges = true;
			int updates[2] = {0, 0};
			for (const Skeleton &k : g_skeletons) {
				const bool left = k.container == 1001;
				paths = paths && k.path == (left ? "/input/skeleton/left" : "/input/skeleton/right") &&
				        k.skeleton_path == (left ? "/skeleton/hand/left" : "/skeleton/hand/right");
				full = full && k.level == VRSkeletalTracking_Full;
				valid = valid && k.problem.empty();
				ranges = ranges && k.with_controller == k.without_controller;
				updates[left ? 0 : 1] = k.with_controller;
				if (!k.problem.empty()) {
					printf("    skeleton %s: %s\n", k.path.c_str(), k.problem.c_str());
				}
			}
			printf("    skeleton updates: left %d, right %d (40 frames, the right hand lost for 5)\n", updates[0],
			       updates[1]);
			check(paths, "hands: each hand has its skeleton (/input/skeleton/left|right, /skeleton/hand/left|right)");
			check(full, "hands: the skeletons are full tracking (VRSkeletalTracking_Full)");
			check(valid, "hands: every skeleton pose has 31 valid bones");
			check(ranges && updates[0] >= 40 && updates[1] >= 35 && updates[1] < updates[0],
			      "hands: both motion ranges every frame the hand is tracked (none while it is lost)");
			float pinch_min = 1e9f, pinch_max = -1e9f, click_max = -1e9f;
			for (const Component &c : g_components) {
				if (c.container == 1001 && c.path == "/input/trigger/value") {
					pinch_min = c.min_value;
					pinch_max = c.max_value;
				}
				if (c.container == 1001 && c.path == "/input/trigger/click") {
					click_max = c.max_value;
				}
			}
			check(pinch_min < 0.1f && pinch_max > 0.99f && click_max == 1.0f,
			      "hands: a pinch (thumb tip on the index tip) pulls the trigger, open it is released");
			// As SteamVR's own tracked hands (vrlink): per-finger pinches, a system button, the driver's poses.
			auto has = [&](uint64_t container, const char *path, int kind) {
				for (const Component &c : g_components) {
					if (c.container == container && c.path == path && c.kind == kind) {
						return true;
					}
				}
				return false;
			};
			bool inputs = true, poses = true;
			for (uint64_t c : {1001ull, 1002ull}) {
				for (const char *p : {"/input/index_pinch/value", "/input/middle_pinch/value", "/input/ring_pinch/value",
				                      "/input/pinky_pinch/value", "/input/grip/value"}) {
					inputs = inputs && has(c, p, 1);
				}
				inputs = inputs && has(c, "/input/system/click", 0);
				for (const char *p : {"/pose/raw", "/pose/grip", "/pose/tip"}) {
					poses = poses && has(c, p, 3);
				}
			}
			check(inputs, "hands: per-finger pinches, the grip and a system button, as SteamVR's own tracked hands");
			check(poses, "hands: the driver gives /pose/raw, /pose/grip and /pose/tip (the dashboard's laser)");
			float tip_z = 0.0f;
			for (const Component &c : g_components) {
				if (c.container == 1001 && c.path == "/pose/tip") {
					tip_z = c.value;
				}
			}
			check(tip_z < -0.01f, "hands: the tip pose is ahead of the palm, along the hand");
			const DriverPose_t &last = g_poses[1].last;
			check(last.poseIsValid && last.deviceIsConnected && last.vecPosition[0] < 0.0 &&
			          std::fabs(last.vecPosition[1] - 1.2) < 0.15,
			      "hands: the left hand's pose is valid, at the left, where it is held");
			const double moved = std::hypot(last.vecPosition[0] - first.vecPosition[0],
			                                last.vecPosition[1] - first.vecPosition[1],
			                                last.vecPosition[2] - first.vecPosition[2]);
			check(first.poseIsValid && moved < 1e-6,
			      "hands: a hand held still stays put while the head moves (not coupled to the head)");
		}
		provider->Cleanup();
		check(dlclose(lib) == 0, "the driver unloads");
		printf("%s\n", g_fails ? "FAILED" : "all checks passed");
		return g_fails ? 1 : 0;
	}

	// --late: the simulated left controller appears only after the start (a Sense controller connecting late):
	// the driver adds it to SteamVR from RunFrame, and its poses flow once SteamVR activates it.
	if (argc > 1 && !strcmp(argv[1], "--late")) {
		check(provider->Init(&context) == VRInitError_None, "Init succeeds");
		std::vector<Added> added;
		{
			std::lock_guard<std::mutex> l(g_mutex);
			added = g_added;
		}
		check(added.size() == 2 && added[0].cls == TrackedDeviceClass_HMD && added[1].cls == TrackedDeviceClass_Controller,
		      "late left: the headset and the right controller only at the start");
		for (uint32_t i = 0; i < added.size(); i++) {
			check(added[i].driver->Activate(i) == VRInitError_None, "Activate succeeds");
		}
		for (int i = 0; i < 4; i++) {
			provider->RunFrame();
		}
		{
			std::lock_guard<std::mutex> l(g_mutex);
			check(g_added.size() == 2, "late left: nothing added before it connects");
		}
		for (int i = 0; i < 3; i++) {
			provider->RunFrame();
		}
		{
			std::lock_guard<std::mutex> l(g_mutex);
			added = g_added;
		}
		check(added.size() == 3 && added[2].cls == TrackedDeviceClass_Controller && added[2].serial != added[1].serial,
		      "late left: added to SteamVR once it connects (RunFrame), with its own serial");
		for (int i = 0; i < 3; i++) {
			provider->RunFrame();
		}
		{
			std::lock_guard<std::mutex> l(g_mutex);
			check(g_added.size() == 3, "late left: added once only");
		}
		if (added.size() == 3) {
			sleep_ms(30);
			check(pose_count(2) == 0, "late left: no pose before SteamVR activates it");
			check(added[2].driver->Activate(2) == VRInitError_None, "late left: Activate succeeds");
			int32_t role = 0;
			check(prop(2, Prop_ControllerRoleHint_Int32, &role) && role == TrackedControllerRole_LeftHand,
			      "late left: it is the left hand");
			sleep_ms(100);
			bool valid = false;
			{
				std::lock_guard<std::mutex> l(g_mutex);
				valid = g_poses[2].last.poseIsValid;
			}
			check(pose_count(2) > 50 && valid, "late left: its poses flow after activation");
		}
		provider->Cleanup();
		check(dlclose(lib) == 0, "the driver unloads");
		printf("%s\n", g_fails ? "FAILED" : "all checks passed");
		return g_fails ? 1 : 0;
	}

	// Real hardware: one round (Monado's PS VR2 driver can't re-claim the USB interfaces within one process; under
	// SteamVR every start is a new vrserver process).
	for (int round = 0; round < (real ? 1 : 5); round++) {
		printf("--- round %d (%s)\n", round + 1, round == 0 ? "first start" : "restart after Cleanup");
		{
			std::lock_guard<std::mutex> l(g_mutex);
			g_added.clear();
			g_poses.clear();
			g_props.clear();
			g_eyeToHead.clear();
			g_components.clear();
			g_skeletons.clear();
			g_eye = EyeComponent{};
			g_events.clear();
		}
		unsetenv("DISABLE_STEAMVR_LINUX_FIXES");
		// Odd rounds ask for async reprojection (OVRN_ASYNC=1, DriverKit's setting on); even rounds use the default.
		const bool asyncAsked = round % 2 == 1;
		if (asyncAsked) {
			setenv("OVRN_ASYNC", "1", 1);
		} else {
			unsetenv("OVRN_ASYNC");
		}
		{
			std::lock_guard<std::mutex> l(g_mutex);
			g_bools["steamvr/useFacetRenderer"] = true; // the user's own setting, turned on by hand
		}
		check(provider->Init(&context) == VRInitError_None, "Init succeeds");
		if (real) {
			std::lock_guard<std::mutex> l(g_mutex);
			if (g_added.empty()) {
				// Used to run on Monado's fallback Simulated HMD; with no PS VR2 the driver now starts nothing.
				printf("--real: no headset registered (is the PS VR2 plugged in?)\nFAILED\n");
				return 1;
			}
		}
		{
			const char *off = getenv("DISABLE_STEAMVR_LINUX_FIXES");
			std::lock_guard<std::mutex> l(g_mutex);
			check(off != nullptr && strcmp(off, "1") == 0,
			      "the SteamVRLinuxFixes layer is switched off for the compositor (environment it inherits)");
			if (asyncAsked) {
				check(g_bools.count("steamvr/enableLinuxVulkanAsync") && g_bools["steamvr/enableLinuxVulkanAsync"],
				      "OVRN_ASYNC=1: SteamVR's async reprojection is switched on before the compositor starts");
			} else {
				check(!g_bools.count("steamvr/enableLinuxVulkanAsync"),
				      "by default SteamVR's async reprojection setting is left alone");
			}
			check(g_bools.count("steamvr/useFacetRenderer") && !g_bools["steamvr/useFacetRenderer"] &&
			          g_bools["driver_monado_psvr2/facetRendererTurnedOffByDriver"],
			      "SteamVR's Facet renderer is off before the compositor starts (noted in our section)");
		}
		if (room) {
#ifdef OVRN_HARNESS_HAND_TRACKING
			check(g_logText.find("hand tracking (experimental) on: the cameras track bare hands") != std::string::npos,
			      "DriverKit's settings file reaches the driver however SteamVR starts (hand tracking on)");
#else
			check(g_logText.find("hand tracking (experimental) is on in DriverKit, but this driver was built without") !=
			          std::string::npos,
			      "DriverKit's setting reaches the driver; a build without hand tracking says it has none");
#endif
		} else if (monado && getenv("XDG_CONFIG_HOME") == nullptr &&
		           access((std::string(getenv("HOME") ? getenv("HOME") : "") + "/.config/PSVR2-DriverKit/settings.conf").c_str(), F_OK) != 0) {
			// (only without a real DriverKit settings file in the user's config)
			check(g_logText.find("ovrn: hand tracking (experimental) off") != std::string::npos,
			      "without DriverKit's setting hand tracking stays off");
		}
		check(provider->GetInterfaceVersions() != nullptr, "interface versions are reported");
		std::vector<Added> added;
		{
			std::lock_guard<std::mutex> l(g_mutex);
			added = g_added;
		}
		int hmds = 0, controllers = 0;
		for (const Added &a : added) {
			hmds += a.cls == TrackedDeviceClass_HMD;
			controllers += a.cls == TrackedDeviceClass_Controller;
		}
		// Monado's simulated system has a headset only; real hardware has the headset and whichever controllers are on.
		const size_t n_dev = real ? added.size() : (monado ? 1 : 3);
		check(hmds == 1 && controllers == (int)n_dev - 1 && controllers <= 2 && added.size() == n_dev &&
		          added[0].cls == TrackedDeviceClass_HMD,
		      real ? "one headset (first) and the connected controllers are registered"
		           : (monado ? "one headset is registered" : "one headset (first) and two controllers are registered"));
		bool distinct = !added.empty() && !added[0].serial.empty();
		for (size_t i = 1; i < added.size(); i++) {
			distinct = distinct && added[i].serial != added[i - 1].serial;
		}
		check(distinct, "the devices have distinct, non-empty serial numbers");
		if (added.size() != n_dev) {
			provider->Cleanup();
			return 1;
		}
		sleep_ms(50);
		check(pose_count(0) + pose_count(1) + pose_count(2) == 0, "no pose is sent before a device is activated");

		for (uint32_t i = 0; i < n_dev; i++) {
			check(added[i].driver->Activate(i) == VRInitError_None, "Activate succeeds");
		}
		ITrackedDeviceServerDriver *hmd = added[0].driver;
		float hz = 0, ipd = 0, v2p = -1;
		bool has_display = false, on_desktop = true;
		check(prop(0, Prop_DisplayFrequency_Float, &hz) && (monado ? (hz > 1.0f && hz < 1000.0f) : fabsf(hz - 90.0f) < 0.01f),
		      "display frequency is set");
		check(prop(0, Prop_UserIpdMeters_Float, &ipd) && ipd > 0.05f && ipd < 0.08f, "a plausible IPD is set");
		check(prop(0, Prop_SecondsFromVsyncToPhotons_Float, &v2p) && v2p >= 0.0f, "vsync-to-photons is set");
		{
			int32_t style = -1;
			check(prop(0, Prop_HmdTrackingStyle_Int32, &style) && style == HmdTrackingStyle_InsideOutCameras,
			      "the headset reports inside-out tracking (Prop_HmdTrackingStyle_Int32 = InsideOutCameras)");
			{
				// SteamVR's dashboard: 11003 shows Playspace Setup / Adjust Floor, 11004 lets their steps run
				bool shows = false, runs = false;
				check(prop(0, 11003, &shows) && shows && prop(0, 11004, &runs) && runs,
				      "in-headset playspace setup: SupportsInHMDRoomSetup_Bool (11003) and "
				      "VRLinkClientHMDSupportsRoomSetupRequests_Bool (11004) both true on the headset");
			}
		}
		if (monado && !room) {
			std::lock_guard<std::mutex> l(g_mutex);
			check(g_props[1000].count(Prop_DriverProvidedChaperoneJson_String) == 1,
			      "with the room calibration off a play area is given to SteamVR");
		}
		if (monado) {
			// Sony's icons in the driver's resources (copied by build-driver.sh / linked by launch-steamvr.sh): the
			// driver finds them next to itself and gives SteamVR the PS VR2 headset icons, without any variable.
			// Only a real PS VR2 gets them (the Monado mode without --real runs on a simulated headset).
			std::string so = path, icon, type;
			const std::string plugin = so.substr(0, so.rfind("/bin/linux64/"));
			struct stat st;
			const bool have = stat((plugin + "/resources/icons/playstation_vr2/headset_status_off@2x.png").c_str(), &st) == 0;
			{
				std::lock_guard<std::mutex> l(g_mutex);
				const auto &b = g_props[1000][Prop_NamedIconPathDeviceOff_String];
				icon.assign(b.begin(), b.end());
				icon = icon.c_str();
				const auto &t = g_props[1000][Prop_ControllerType_String];
				type.assign(t.begin(), t.end());
				type = type.c_str();
			}
			if (type == "playstation_vr2_hmd") {
				printf("    headset icon: %s (icons %s in %s/resources)\n", icon.empty() ? "(SteamVR's default)" : icon.c_str(),
				       have ? "present" : "absent", plugin.c_str());
				check(have == (icon.find("playstation_vr2/headset_status_off") != std::string::npos),
				      "the PS VR2 headset icons are used exactly when they're in the driver's resources");
			}
		}
		if (room) {
			std::string published;
			uint64_t universe = 0;
			{
				std::lock_guard<std::mutex> l(g_mutex);
				const auto &b = g_props[1000][Prop_DriverProvidedChaperonePath_String];
				published.assign(b.begin(), b.end());
				published = published.c_str(); // up to the terminating NUL
			}
			check(published.empty(),
			      "the driver leaves the rooms in SteamVR's own file (no Prop_DriverProvidedChaperonePath_String: "
			      "SteamVR failed to save into the driver's file)");
			check(prop(0, Prop_CurrentUniverseId_Uint64, &universe) && universe == 5023,
			      "the session universe (5023) is active until a floor capture anchors the room");
			const std::string global = room_steam_config + "/chaperone_info.vrchap";
			FILE *gf = fopen(global.c_str(), "r");
			std::string text;
			if (gf != nullptr) {
				char buf[4096];
				size_t n;
				while ((n = fread(buf, 1, sizeof(buf), gf)) > 0) {
					text.append(buf, n);
				}
				fclose(gf);
			}
			auto has = [&](const char *id) {
				return text.find(std::string("\"universeID\":\t\"") + id + "\"") != std::string::npos ||
				       text.find(std::string("\"universeID\":\"") + id + "\"") != std::string::npos;
			};
			check(has("5022") && has("5023"),
			      "SteamVR's file holds the room universe (5022) and the session universe (5023)");
			check(has("1234"), "another headset's room in SteamVR's file is left alone");
			// Both 4 x 2 m rooms (the other headset's and the moved one) are in SteamVR's file now.
			size_t rooms = 0;
			for (size_t at = text.find("[4, 2]"); at != std::string::npos; at = text.find("[4, 2]", at + 1)) {
				rooms++;
			}
			printf("    SteamVR's chaperone file: %zu bytes, %zu drawn 4 x 2 m rooms\n", text.size(), rooms);
			check(rooms == 2, "the room drawn in the driver's old file was moved into SteamVR's file");
			struct stat ost;
			check(stat((room_steam_config + "/monado_psvr2/chaperone_info.vrchap").c_str(), &ost) != 0 &&
			          stat((room_steam_config + "/monado_psvr2/chaperone_info.vrchap.moved-to-steamvr").c_str(), &ost) == 0,
			      "the driver's old file is renamed, so it is moved only once");
		}
				check(prop(0, Prop_HasDisplayComponent_Bool, &has_display) && has_display &&
		          prop(0, Prop_IsOnDesktop_Bool, &on_desktop) && !on_desktop,
		      "the headset is declared as a display that is not on the desktop");
		{
			std::lock_guard<std::mutex> l(g_mutex);
			check(g_eyeToHead[0] == 1, "eye-to-head transforms are given to SteamVR once");
		}
		int32_t role[3] = {0, 0, 0};
		if (!monado) {
			check(prop(1, Prop_ControllerRoleHint_Int32, &role[1]) &&
			          prop(2, Prop_ControllerRoleHint_Int32, &role[2]) &&
			          role[1] == TrackedControllerRole_LeftHand && role[2] == TrackedControllerRole_RightHand,
			      "the controllers are left and right hand");
		}

		IVRDisplayComponent *display = (IVRDisplayComponent *)hmd->GetComponent(IVRDisplayComponent_Version);
		check(display != nullptr, "the headset has a display component");
		check(hmd->GetComponent(IVRDriverDirectModeComponent_Version) == nullptr &&
		          hmd->GetComponent(IVRVirtualDisplay_Version) == nullptr,
		      "no direct mode or virtual display component: SteamVR owns the display");
		if (display != nullptr) {
			int32_t x, y;
			uint32_t w, h;
			display->GetWindowBounds(&x, &y, &w, &h);
			check(monado ? (w > 0 && h > 0) : (w == 4000 && h == 2040), "window bounds are the panel size");
			const uint32_t pw = w, ph = h;
			check(display->IsDisplayRealDisplay() && !display->IsDisplayOnDesktop(), "real display, not on desktop");
			uint32_t rw, rh;
			display->GetRecommendedRenderTargetSize(&rw, &rh);
			check(monado ? (rw > 0 && rh > 0) : (rw == 2000 && rh == 2040), "recommended render target per eye");
			uint32_t vx[2], vy, vw[2], vh;
			display->GetEyeOutputViewport(Eye_Left, &vx[0], &vy, &vw[0], &vh);
			display->GetEyeOutputViewport(Eye_Right, &vx[1], &vy, &vw[1], &vh);
			check(vx[0] + vw[0] <= pw && vx[1] + vw[1] <= pw && vx[1] >= vx[0] + vw[0] && vw[0] > 0 && vw[1] > 0 &&
			          vh > 0 && vh <= ph && (monado || (vx[1] == 2000 && vw[0] == 2000 && vh == 2040)),
			      "eye viewports are side by side inside the panel");
			float l, r, t, b;
			display->GetProjectionRaw(Eye_Left, &l, &r, &t, &b);
			check(l < 0 && r > 0 && t < 0 && b > 0 && (monado || (fabsf(l + 1.0f) < 0.01f && fabsf(b - 1.0f) < 0.01f)),
			      "projection: left and top negative, right and bottom positive (OpenVR convention)");
			DistortionCoordinates_t d = display->ComputeDistortion(Eye_Right, 0.3f, 0.7f);
			check(monado ? (std::isfinite(d.rfGreen[0]) && std::isfinite(d.rfRed[1]) && std::isfinite(d.rfBlue[0]))
			             : (d.rfGreen[0] == 0.3f && d.rfGreen[1] == 0.7f && d.rfRed[0] == 0.3f && d.rfBlue[1] == 0.7f),
			      monado ? "distortion comes from the device" : "distortion falls back to the identity");
		}

		if (real && display != nullptr && g_dumpPath != nullptr) {
			// The same lines as psvr2-test/tools/vrdisplaydump writes from a running SteamVR, straight from the driver
			// (SteamVR passes these through), for analysis/display_diff.py against a dump of Sony's driver.
			FILE *df = fopen(g_dumpPath, "w");
			if (df != nullptr) {
				uint32_t rw, rh;
				display->GetRecommendedRenderTargetSize(&rw, &rh);
				fprintf(df, "render_target %u %u\n", rw, rh);
				for (int eye = 0; eye < 2; eye++) {
					float l, r, t, b;
					display->GetProjectionRaw((EVREye)eye, &l, &r, &t, &b);
					fprintf(df, "projection_raw %d %.6f %.6f %.6f %.6f\n", eye, l, r, t, b);
					fprintf(df, "eye_to_head %d", eye);
					for (int a = 0; a < 3; a++) {
						for (int c = 0; c < 4; c++) {
							fprintf(df, " %.6f", g_eyeMatrix[eye].m[a][c]);
						}
					}
					fprintf(df, "\n");
					const int N = 33;
					for (int y = 0; y < N; y++) {
						for (int x = 0; x < N; x++) {
							const float u = (float)x / (N - 1), v = (float)y / (N - 1);
							DistortionCoordinates_t d = display->ComputeDistortion((EVREye)eye, u, v);
							fprintf(df, "distortion %d %.4f %.4f 1 %.6f %.6f %.6f %.6f %.6f %.6f\n", eye, u, v,
							        d.rfRed[0], d.rfRed[1], d.rfGreen[0], d.rfGreen[1], d.rfBlue[0], d.rfBlue[1]);
						}
					}
				}
				fclose(df);
				printf("REAL: display description written to %s\n", g_dumpPath);
			}
		}
		if (real && display != nullptr) {
			// What the driver tells SteamVR about the real headset, for comparison with the known values.
			float l, r, t, b;
			uint32_t rw, rh, vx, vy, vw, vh;
			float p2v = -1;
			int32_t mesh = 0;
			prop(0, Prop_SecondsFromPhotonsToVblank_Float, &p2v);
			prop(0, Prop_DistortionMeshResolution_Int32, &mesh);
			display->GetRecommendedRenderTargetSize(&rw, &rh);
			printf("REAL: %.3f Hz, IPD %.2f mm, vsync-to-photons %.5f, photons-to-vblank %.5f, mesh %d, render %ux%u\n",
			       hz, ipd * 1000.0f, v2p, p2v, mesh, rw, rh);
			for (int e = 0; e < 2; e++) {
				display->GetProjectionRaw((EVREye)e, &l, &r, &t, &b);
				display->GetEyeOutputViewport((EVREye)e, &vx, &vy, &vw, &vh);
				DistortionCoordinates_t c = display->ComputeDistortion((EVREye)e, 0.5f, 0.5f);
				DistortionCoordinates_t k = display->ComputeDistortion((EVREye)e, 0.1f, 0.1f);
				printf("REAL eye %d: viewport %u,%u %ux%u; projection L%.4f R%.4f T%.4f B%.4f; distortion centre "
				       "g(%.4f,%.4f), at 0.1,0.1 r(%.4f,%.4f) g(%.4f,%.4f) b(%.4f,%.4f)\n",
				       e, vx, vy, vw, vh, l, r, t, b, c.rfGreen[0], c.rfGreen[1], k.rfRed[0], k.rfRed[1],
				       k.rfGreen[0], k.rfGreen[1], k.rfBlue[0], k.rfBlue[1]);
			}
		}
		if (real) {
			// The headset needs a few seconds before it reports an orientation.
			for (int w = 0; w < 150; w++) {
				sleep_ms(100);
				std::lock_guard<std::mutex> l(g_mutex);
				if (g_poses[0].last.poseIsValid) {
					printf("REAL: first valid head pose after %.1f s\n", (w + 1) * 0.1);
					break;
				}
			}
		}
		if (!monado) {
			// Controllers of the simulated backend: 3 inputs and a haptic output each; a universe that changes after
			// 30 frames; a room transform 25 cm up.
			uint64_t universe0 = 0;
			prop(1, Prop_CurrentUniverseId_Uint64, &universe0);
			for (int i = 0; i < 40; i++) {
				provider->RunFrame();
			}
			size_t n_inputs[2] = {0, 0}, n_haptic[2] = {0, 0};
			float trigger = -1, stick_l = 0, stick_r = 0;
			int trigger_updates = 0;
			uint64_t haptic_handle = 0;
			{
				std::lock_guard<std::mutex> l(g_mutex);
				for (size_t k = 0; k < g_components.size(); k++) {
					const Component &c = g_components[k];
					const int which = c.container == 1001 ? 0 : (c.container == 1002 ? 1 : -1);
					if (which < 0) {
						continue;
					}
					(c.kind == 2 ? n_haptic : n_inputs)[which]++;
					if (c.kind == 2 && which == 1) {
						haptic_handle = k + 1;
					}
					if (c.path == "/input/trigger/value" && which == 0) {
						trigger = c.value;
						trigger_updates = c.updates;
					}
					if (c.path == "/input/stick/x") {
						(which == 0 ? stick_l : stick_r) = c.value;
					}
				}
			}
			check(n_inputs[0] == 3 && n_inputs[1] == 3 && n_haptic[0] == 1 && n_haptic[1] == 1,
			      "each controller has its input components and a haptic component");
			{
				std::lock_guard<std::mutex> l(g_mutex);
				bool paths = g_skeletons.size() == 2, valid = true, every_frame = true;
				for (const Skeleton &k : g_skeletons) {
					const bool left = k.container == 1001;
					paths = paths && k.path == (left ? "/input/skeleton/left" : "/input/skeleton/right") &&
					        k.skeleton_path == (left ? "/skeleton/hand/left" : "/skeleton/hand/right");
					valid = valid && k.problem.empty();
					every_frame = every_frame && k.with_controller >= 41 && k.without_controller >= 41;
					if (!k.problem.empty()) {
						printf("    skeleton %s: %s\n", k.path.c_str(), k.problem.c_str());
					}
				}
				check(paths, "each controller has a hand skeleton (/input/skeleton/left|right, as Sony's driver)");
				check(valid, "every skeleton pose SteamVR gets has 31 valid bones");
				check(every_frame, "both skeleton motion ranges are set at activation and every frame after");
			}
			{
				// Eye tracking: present only with a gaze source (OVRN_SIM_EYE=1, the test-only gaze).
				const bool sim_eye = getenv("OVRN_SIM_EYE") != nullptr && strcmp(getenv("OVRN_SIM_EYE"), "1") == 0;
				bool cap = false;
				const bool has_cap = prop(0, Prop_SupportsXrEyeGazeInteraction_Bool, &cap) && cap;
				std::lock_guard<std::mutex> l(g_mutex);
				if (!sim_eye) {
					check(!has_cap && g_eye.path.empty(),
					      "no eye-tracking capability or component without a gaze source");
				} else {
					check(has_cap && g_eye.path == "/eyetracking" && g_eye.container == 1000,
					      "eye tracking: Prop_SupportsXrEyeGazeInteraction_Bool and the /eyetracking component on the headset");
					const VREyeTrackingData_t &v = g_eye.last_valid;
					const float yaw = std::atan2(v.vGazeTarget.v[0], -v.vGazeTarget.v[2]) * 57.2958f;
					const float pitch = std::asin(v.vGazeTarget.v[1]) * 57.2958f;
					printf("    gaze: %d updates, %d valid, %d not valid; last valid yaw %.2f pitch %.2f deg, offset %.4f s\n",
					       g_eye.updates, g_eye.valid, g_eye.invalid, yaw, pitch, g_eye.last_valid_offset);
					check(g_eye.valid > 0 && std::fabs(yaw - 10.0f) < 0.01f && std::fabs(pitch - 5.0f) < 0.01f &&
					          g_eye.last_valid_offset < 0.0 && g_eye.last_valid_offset > -0.02,
					      "fresh gaze reaches SteamVR valid, unchanged (10 deg right, 5 up), as a few ms old");
					check(g_eye.invalid > 0 && !g_eye.last.bValid,
					      "gaze whose packet is 200 ms old reaches SteamVR as not valid (never stale)");
					check(g_eye.mixed_flags == 0, "bActive, bValid and bTracked always agree");
				}
			}
			check(trigger == 0.5f && trigger_updates >= 40 && stick_l == -1.0f && stick_r == 1.0f,
			      "input values reach SteamVR every frame, per hand");
			uint64_t universe1 = 0, universe_c = 0;
			check(prop(0, Prop_CurrentUniverseId_Uint64, &universe1) && prop(2, Prop_CurrentUniverseId_Uint64, &universe_c) &&
			          universe0 == 3 && universe1 == 2 && universe_c == 2,
			      "a universe change (room captured) reaches the headset and the controllers");
			// A haptic event for the right controller.
			VREvent_t ev = {};
			ev.eventType = VREvent_Input_HapticVibration;
			ev.data.hapticVibration.containerHandle = 1002;
			ev.data.hapticVibration.componentHandle = haptic_handle;
			ev.data.hapticVibration.fDurationSeconds = 0.1f;
			ev.data.hapticVibration.fAmplitude = 0.7f;
			{
				std::lock_guard<std::mutex> l(g_mutex);
				g_events.push_back(ev);
			}
			provider->RunFrame();
			{
				std::lock_guard<std::mutex> l(g_mutex);
				check(g_events.empty(), "haptic events are taken from SteamVR's queue");
			}
		}
		sleep_ms(real ? 3000 : 1000);
		if (real) {
			std::lock_guard<std::mutex> l(g_mutex);
			const DriverPose_t &q = g_poses[0].last;
			printf("REAL head pose: valid %d result %d pos %.3f %.3f %.3f rot %.4f %.4f %.4f %.4f\n", q.poseIsValid,
			       q.result, q.vecPosition[0], q.vecPosition[1], q.vecPosition[2], q.qRotation.x, q.qRotation.y,
			       q.qRotation.z, q.qRotation.w);
		}
		for (int i = 0; i < 20; i++) {
			provider->RunFrame();
		}
		std::map<uint32_t, PoseLog> poses;
		{
			std::lock_guard<std::mutex> l(g_mutex);
			poses = g_poses;
		}
		bool cadence = true;
		for (uint32_t i = 0; i < n_dev; i++) {
			const std::vector<int64_t> &t = poses[i].times;
			std::vector<double> gaps;
			for (size_t k = 1; k < t.size(); k++) {
				gaps.push_back((t[k] - t[k - 1]) / 1e6);
			}
			std::sort(gaps.begin(), gaps.end());
			const double med = gaps.empty() ? 0 : gaps[gaps.size() / 2];
			const double p99 = gaps.empty() ? 0 : gaps[(size_t)(0.99 * (gaps.size() - 1))];
			printf("device %u: %zu poses, interval median %.3f ms, p99 %.3f ms\n", i, t.size(), med, p99);
			cadence = cadence && (real || (t.size() > 950 && t.size() < 1100)) && fabs(med - 1.0) < 0.05 && p99 < 1.5;
		}
		check(cadence, "every device's pose is sent on a steady 1 ms grid");
		const DriverPose_t &hp = poses[0].last;
		const double qn = sqrt(hp.qRotation.w * hp.qRotation.w + hp.qRotation.x * hp.qRotation.x +
		                       hp.qRotation.y * hp.qRotation.y + hp.qRotation.z * hp.qRotation.z);
		check(hp.poseIsValid && hp.deviceIsConnected && hp.result == TrackingResult_Running_OK &&
		          fabs(qn - 1.0) < 1e-5 && (monado || fabs(hp.vecPosition[1] - 1.6) < 1e-9),
		      "head pose: valid, connected, unit rotation, the backend's position");
		check(hp.poseTimeOffset <= 0.0 && hp.poseTimeOffset > -0.001,
		      "head pose: its age is handed over (a time offset between -1 ms and 0)");
		if (!monado) {
			// The simulated head yaws at 0.5 rad/s with a 0.3 rad pitch and drifts at 0.1 m/s: extrapolating one pose
			// with its own velocities, as SteamVR does, must land on the next pose.
			const DriverPose_t &a = poses[0].prev, &b = poses[0].last;
			const double dt = ((double)poses[0].last_ns * 1e-9 + b.poseTimeOffset) -
			                  ((double)poses[0].prev_ns * 1e-9 + a.poseTimeOffset);
			const double wx = a.vecAngularVelocity[0], wy = a.vecAngularVelocity[1], wz = a.vecAngularVelocity[2];
			const double wn = sqrt(wx * wx + wy * wy + wz * wz), ang = wn * dt;
			// q_pred = q_a * exp(w_local * dt)
			const double s2 = wn > 1e-12 ? sin(ang / 2) / wn : 0, c2 = cos(ang / 2);
			const double dx = wx * s2, dy = wy * s2, dz = wz * s2, dw = c2;
			const HmdQuaternion_t &q = a.qRotation;
			const double px = q.w * dx + q.x * dw + q.y * dz - q.z * dy, py = q.w * dy - q.x * dz + q.y * dw + q.z * dx,
			             pz = q.w * dz + q.x * dy - q.y * dx + q.z * dw, pw = q.w * dw - q.x * dx - q.y * dy - q.z * dz;
			const double dot = fabs(px * b.qRotation.x + py * b.qRotation.y + pz * b.qRotation.z + pw * b.qRotation.w);
			const double err_deg = 2 * acos(fmin(1.0, dot)) * 57.29578;
			const double pos_err = fabs(a.vecPosition[0] + a.vecVelocity[0] * dt - b.vecPosition[0]);
			printf("head velocities: |w| %.3f rad/s (local %.3f %.3f %.3f), extrapolated %.2f ms -> %.5f deg and %.5f mm "
			       "from the next pose\n",
			       wn, wx, wy, wz, dt * 1e3, err_deg, pos_err * 1e3);
			check(fabs(wn - 0.5) < 1e-6 && fabs(wz) > 0.1 && err_deg < 0.002 && pos_err < 2e-5 && dt > 0.0005 &&
			          dt < 0.0015,
			      "head pose: velocities in the head's own frame, and extrapolating with them reaches the next pose");
		}
		check(hp.qWorldFromDriverRotation.w == 1.0 && hp.qDriverFromHeadRotation.w == 1.0,
		      "head pose: identity world-from-driver and driver-from-head rotations");
		if (!monado) {
			check(hp.vecWorldFromDriverTranslation[1] == 0.25 && poses[1].last.vecWorldFromDriverTranslation[1] == 0.25 &&
			          poses[2].last.vecWorldFromDriverTranslation[1] == 0.25,
			      "the room transform is in every device's pose, the same for all");
		}
		if (!monado) {
			check(poses[1].last.poseIsValid && poses[2].last.poseIsValid && poses[1].last.vecPosition[0] < 0 &&
			          poses[2].last.vecPosition[0] > 0,
			      "controller poses: valid, left on the left and right on the right");
		}
		DriverPose_t got = hmd->GetPose();
		check(got.poseIsValid, "GetPose returns a valid last pose");

		hmd->Deactivate();
		// Counted the moment Deactivate returns, no wait: OpenVR allows no call for the device after that, not even
		// one the pose thread had already started (Deactivate must wait for it).
		size_t h0, e0;
		{
			std::lock_guard<std::mutex> l(g_mutex);
			h0 = g_poses[0].times.size();
			e0 = g_eyeToHead[0];
		}
		const size_t c0 = pose_count(1);
		sleep_ms(120);
		size_t e1;
		{
			std::lock_guard<std::mutex> l(g_mutex);
			e1 = g_eyeToHead[0];
		}
		check(pose_count(0) == h0, "no pose for the headset once its Deactivate returned");
		check(e1 == e0, "no eye-to-head (IPD) update for the headset once its Deactivate returned");
		check(monado || pose_count(1) > c0 + 80, "the other devices keep sending poses");

		// SteamVR deactivates every device before Cleanup; the same for each of them.
		bool quiet = true;
		for (size_t i = 1; i < added.size(); i++) {
			added[i].driver->Deactivate();
			const size_t n = pose_count((uint32_t)i);
			sleep_ms(30);
			quiet = quiet && pose_count((uint32_t)i) == n;
		}
		check(quiet, "no pose for any device once its Deactivate returned");

		provider->Cleanup();
		{
			std::lock_guard<std::mutex> l(g_mutex);
			check(!g_bools.count("steamvr/enableLinuxVulkanAsync") && !g_bools.count("driver_monado_psvr2/asyncReprojectionSetByDriver"),
			      "Cleanup takes the async reprojection setting out again (other drivers keep SteamVR's default)");
			check(g_bools["steamvr/useFacetRenderer"] && !g_bools.count("driver_monado_psvr2/facetRendererTurnedOffByDriver"),
			      "Cleanup puts the user's Facet renderer setting back on");
		}
		const size_t total = pose_count(0) + pose_count(1) + pose_count(2);
		sleep_ms(100);
		check(pose_count(0) + pose_count(1) + pose_count(2) == total, "no pose is sent after Cleanup (threads stopped)");
	}

	check(dlclose(lib) == 0, "the driver unloads");
	printf("%s\n", g_fails ? "FAILED" : "all checks passed");
	return g_fails ? 1 : 0;
}
