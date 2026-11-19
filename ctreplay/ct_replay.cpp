// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//
// Replays a constellation tracker dataset (RECORD_DATASET=1) through the real tracker, offline and deterministic,
// to measure controller tracking changes without the headset.
//
// Inputs, all from one session:
//   - the dataset (per camera frame: blobs with the blob IDs the live blobwatch assigned, the camera's world pose),
//     camera calibrations and the controllers' LED models;
//   - the controllers' IMU recordings (PSSENSE_RECORD_FILE, sense-<stamp>-L/R.csv): the same host timestamps, gyro
//     and accel the driver pushed to the tracker, and the driver's fused orientation (for the tracker's IMU gates).
//
// The tracker runs single threaded (T_CONSTELLATION_TRACKER_FLAGS_DETERMINISTIC): each frame gets the fast search
// and, if that fails, the slow one right away, where live the slow search runs on its own thread. Blob labels are
// carried between frames by blob ID like psvr2_blobwatch does. IMU samples up to each frame's time are pushed first.
//
// Output: per device fix rate and optical gaps (count, time share, length distribution), jumps between consecutive
// fixes, and a CSV of fixes per frame time (for comparisons). Tracker options are environment variables as live.
//
// Usage: ct_replay dataset sense-L.csv sense-R.csv out.csv [--max-seconds S] [--imu-lag-ms MS] [--realtime]

#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"
#include "math/m_api.h"
#include "os/os_time.h"
#include "util/u_time.h"
#include "math/m_vec3.h"
#include "tracking/t_constellation.h"
#include "t_constellation_tracker.h"
#include "t_constellation_tracker_dataset.hpp"
#include "pssense/pssense_led_model.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <mutex>
#include <atomic>
extern "C" {
#include "t_blob_background.h"
}
#include <sstream>
#include <string>
#include <tuple>
#include <vector>
#include <unistd.h>

using namespace xrt::tracking::constellation;

namespace {

struct ImuRow
{
	int64_t t;
	double g[3], a[3];
	xrt_quat q;
};

std::vector<ImuRow>
load_imu(const char *path)
{
	std::vector<ImuRow> rows;
	std::ifstream f(path);
	std::string line;
	while (std::getline(f, line)) {
		if (line.empty() || line[0] == '#')
			continue;
		ImuRow r{};
		long long host, imu;
		if (sscanf(line.c_str(), "%lld,%lld,%lf,%lf,%lf,%lf,%lf,%lf,%f,%f,%f,%f", &host, &imu, &r.g[0], &r.g[1],
		           &r.g[2], &r.a[0], &r.a[1], &r.a[2], &r.q.x, &r.q.y, &r.q.z, &r.q.w) == 12) {
			r.t = host;
			rows.push_back(r);
		}
	}
	return rows;
}

//! The tracking origin: camera 0's world pose over time, from the recorded camera poses.
struct OriginSource
{
	t_constellation_tracker_tracking_source base;
	std::vector<std::pair<int64_t, xrt_pose>> poses; // sorted by time
};

void
origin_get(t_constellation_tracker_tracking_source *src, int64_t when_ns, xrt_space_relation *out)
{
	auto *o = (OriginSource *)src;
	*out = XRT_SPACE_RELATION_ZERO;
	if (o->poses.empty())
		return;
	auto it = std::lower_bound(o->poses.begin(), o->poses.end(), when_ns,
	                           [](const std::pair<int64_t, xrt_pose> &p, int64_t t) { return p.first < t; });
	xrt_pose p;
	if (it == o->poses.begin()) {
		p = it->second;
	} else if (it == o->poses.end()) {
		p = o->poses.back().second;
	} else {
		auto prev = it - 1;
		float f = (float)(when_ns - prev->first) / (float)std::max<int64_t>(1, it->first - prev->first);
		p.position = m_vec3_lerp(prev->second.position, it->second.position, f);
		math_quat_slerp(&prev->second.orientation, &it->second.orientation, f, &p.orientation);
	}
	out->pose = p;
	out->relation_flags = (xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
}

struct Fix
{
	int64_t t;
	uint32_t camera;
	xrt_pose pose;
};

//! Live IMU timing for the IMU gates (--imu-lag-ms): only samples that would have arrived, then extrapolated.
static int64_t g_imu_lag_ns = 0;

struct ReplayDevice
{
	t_constellation_tracker_device base;
	int index;
	const std::vector<ImuRow> *imu;
	xrt_imu_sink *imu_sink;
	t_constellation_device_id_t id;
	std::mutex lock; // the tracker's camera threads report concurrently in --realtime
	std::vector<Fix> fixes;
};

/*!
 * --synced-from-live: the recordings don't store the live LED sync state, so a device counts as synced while the live
 * tracker matched any of its LEDs in the last second (g_live_seen_ns, filled as frames are fed).
 */
static bool g_synced_from_live = false;
static std::atomic<int64_t> g_live_seen_ns[XRT_CONSTELLATION_MAX_DEVICES];
static std::atomic<int64_t> g_feed_ns{0};

bool
device_led_synced(t_constellation_tracker_device *dev)
{
	auto *d = (ReplayDevice *)dev;
	int64_t seen = g_live_seen_ns[d->index % XRT_CONSTELLATION_MAX_DEVICES].load();
	return seen != 0 && g_feed_ns.load() - seen < 1000000000LL;
}

void
device_push_sample(t_constellation_tracker_device *dev, t_constellation_tracker_sample *sample)
{
	auto *d = (ReplayDevice *)dev;
	if (sample->has_world_pose) {
		std::lock_guard<std::mutex> g(d->lock);
		d->fixes.push_back({sample->timestamp_ns, sample->camera_index, sample->world_pose});
	}
}

bool
device_imu_orientation(t_constellation_tracker_device *dev, int64_t t, xrt_quat *out)
{
	auto *d = (ReplayDevice *)dev;
	const auto &v = *d->imu;
	if (v.empty() || t < v.front().t || t > v.back().t + 50000000)
		return false;
	xrt_quat q;
	if (g_imu_lag_ns > 0) {
		// Live, the newest IMU sample at the frame's processing (~2 ms after its timestamp) is lag older: take it and
		// extrapolate with its gyro (body frame) to t, like m_relation_history's prediction.
		int64_t avail = t + 2 * U_TIME_1MS_IN_NS - g_imu_lag_ns;
		auto it = std::upper_bound(v.begin(), v.end(), avail, [](int64_t tt, const ImuRow &r) { return tt < r.t; });
		if (it == v.begin())
			return false;
		--it;
		double dt = std::max<double>(0.0, (t - it->t) / 1e9);
		xrt_vec3 w = {(float)it->g[0], (float)it->g[1], (float)it->g[2]};
		float ang = sqrtf(w.x * w.x + w.y * w.y + w.z * w.z) * (float)dt;
		xrt_quat dq = XRT_QUAT_IDENTITY;
		if (ang > 1e-6f) {
			xrt_vec3 axis = {w.x, w.y, w.z};
			math_vec3_normalize(&axis);
			math_quat_from_angle_vector(ang, &axis, &dq);
		}
		math_quat_rotate(&it->q, &dq, &q);
	} else {
	auto it = std::lower_bound(v.begin(), v.end(), t, [](const ImuRow &r, int64_t tt) { return r.t < tt; });
	if (it == v.end())
		it = v.end() - 1;
	q = it->q;
	if (it != v.begin()) {
		auto prev = it - 1;
		float f = (float)(t - prev->t) / (float)std::max<int64_t>(1, it->t - prev->t);
		math_quat_slerp(&prev->q, &it->q, std::clamp(f, 0.0f, 1.0f), &q);
	}
	}
	// Into the LED model's IMU frame, as pssense_get_imu_orientation.
	xrt_quat corr;
	xrt_vec3 x = {1, 0, 0};
	const char *c = getenv("PSSENSE_IMU_REF_CORRECTION_DEG");
	float deg = c ? (float)atof(c) : PSSENSE_IMU_REF_CORRECTION_DEG_DEFAULT;
	math_quat_from_angle_vector(deg * (float)M_PI / 180.0f, &x, &corr);
	math_quat_rotate(&q, &corr, out);
	return true;
}

//! Carries blob labels between frames by blob ID, like psvr2_blobwatch.
struct LabelBlobwatch
{
	t_blobwatch base;
	std::mutex lock; // marks come from the tracker's threads in --realtime
	std::map<uint32_t, std::map<uint32_t, std::pair<t_constellation_device_id_t, t_constellation_led_id_it>>>
	    labels; // camera -> blob id -> label
};

void
bw_mark(t_blobwatch *tbw, const t_blob_observation *tbo, t_constellation_device_id_t device_id)
{
	auto *bw = (LabelBlobwatch *)tbw;
	std::lock_guard<std::mutex> g(bw->lock);
	auto &cam = bw->labels[(uint32_t)tbo->id];
	for (auto it = cam.begin(); it != cam.end();) {
		if (it->second.first == device_id)
			it = cam.erase(it);
		else
			++it;
	}
	for (uint32_t i = 0; i < tbo->num_blobs; i++) {
		const t_blob &b = tbo->blobs[i];
		if (b.matched_device_id == device_id)
			cam[b.blob_id] = {b.matched_device_id, b.matched_device_led_id};
	}
}

void
bw_destroy(t_blobwatch *)
{}

} // namespace

int
main(int argc, char **argv)
{
	if (argc < 5) {
		fprintf(stderr, "usage: %s dataset sense-L.csv sense-R.csv out.csv [--max-seconds S]\n", argv[0]);
		return 1;
	}
	double max_seconds = 1e9;
	bool realtime = false;
	bool gate_lag = false; // --gate-imu-lag: the IMU gates see the same lagged IMU as the fusion (live) // --realtime: the tracker's own threads, frames fed at the recorded pace (live timing)
	const char *head_rot_record = nullptr;
	// --background: drop background blobs with the background map (t_blob_background) before the tracker.
	t_blob_background *bg_map = nullptr;
	uint64_t bg_blobs_in = 0, bg_blobs_out = 0, bg_removed = 0, bg_led_removed = 0, bg_live_leds = 0;
	std::vector<double> bg_us;
	bool bg_only = false;
	xrt_quat head_rot = XRT_QUAT_IDENTITY;
	int64_t imu_lag_ns = 0; // --imu-lag-ms: IMU samples reach the tracker this much after their timestamp (Bluetooth)
	for (int i = 5; i < argc; i++) {
		if (!strcmp(argv[i], "--max-seconds"))
			max_seconds = atof(argv[i + 1]);
		else if (!strcmp(argv[i], "--synced-from-live"))
			g_synced_from_live = true;
		else if (!strcmp(argv[i], "--background"))
			bg_map = t_blob_background_create();
		else if (!strcmp(argv[i], "--bg-only")) {
			// Background map statistics only, no tracking (fast, for tuning).
			bg_map = t_blob_background_create();
			bg_only = true;
		}
		else if (!strcmp(argv[i], "--realtime"))
			realtime = true;
		else if (!strcmp(argv[i], "--gate-imu-lag"))
			gate_lag = true;
		else if (!strcmp(argv[i], "--imu-lag-ms"))
			imu_lag_ns = (int64_t)(atof(argv[i + 1]) * 1e6);
		else if (!strcmp(argv[i], "--head-rot") && i + 2 < argc) {
			// --head-rot <record.csv> qx,qy,qz,qw: test a camera mount change, every camera pose becomes
			// head * E * head^-1 * recorded (E in the head frame, head poses from the record file's O lines).
			head_rot_record = argv[i + 1];
			sscanf(argv[i + 2], "%f,%f,%f,%f", &head_rot.x, &head_rot.y, &head_rot.z, &head_rot.w);
			math_quat_normalize(&head_rot);
		}
	}

	if (gate_lag)
		g_imu_lag_ns = imu_lag_ns;
	DatasetReader reader(argv[1]);
	std::vector<ImuRow> imu[2] = {load_imu(argv[2]), load_imu(argv[3])};
	if (reader.mosaics.size() != 1 || reader.samples.empty()) {
		fprintf(stderr, "expected one mosaic with samples\n");
		return 1;
	}
	auto &samples = reader.samples;
	std::stable_sort(samples.begin(), samples.end(), [](const CameraSample &a, const CameraSample &b) {
		return a.timestamp_ns < b.timestamp_ns;
	});
	const size_t ncam = reader.mosaics[0].camera_calibrations.size();

	if (head_rot_record != nullptr) {
		std::map<int64_t, xrt_pose> head;
		FILE *rf = fopen(head_rot_record, "r");
		char line[512];
		while (rf && fgets(line, sizeof(line), rf)) {
			long long t;
			xrt_pose p;
			if (sscanf(line, "O,%lld,%f,%f,%f,%f,%f,%f,%f", &t, &p.position.x, &p.position.y, &p.position.z,
			           &p.orientation.x, &p.orientation.y, &p.orientation.z, &p.orientation.w) == 8)
				head[t] = p;
		}
		if (rf)
			fclose(rf);
		size_t changed = 0, dropped = 0;
		xrt_pose E = {head_rot, {0, 0, 0}};
		for (auto &s : samples) {
			if (!s.Txr_world_cam.has_value())
				continue;
			auto it = head.find(s.timestamp_ns);
			if (it == head.end()) {
				s.Txr_world_cam.reset();
				dropped++;
				continue;
			}
			xrt_pose hinv, a, b, c;
			math_pose_invert(&it->second, &hinv);
			math_pose_transform(&hinv, &*s.Txr_world_cam, &a); // camera in head
			math_pose_transform(&E, &a, &b);
			math_pose_transform(&it->second, &b, &c);
			*s.Txr_world_cam = c;
			changed++;
		}
		printf("--head-rot: %zu camera poses changed, %zu without a head pose dropped\n", changed, dropped);
	}

	// Camera poses in camera 0's frame, from a frame where every camera has a recorded pose.
	std::vector<xrt_pose> pose_in_origin(ncam, (xrt_pose)XRT_POSE_IDENTITY);
	{
		std::map<int64_t, std::map<uint32_t, xrt_pose>> by_t;
		for (auto &s : samples)
			if (s.Txr_world_cam.has_value())
				by_t[s.timestamp_ns][s.camera_index] = *s.Txr_world_cam;
		bool done = false;
		for (auto &[t, m] : by_t) {
			if (m.size() == ncam && m.count(0)) {
				xrt_pose inv0;
				math_pose_invert(&m[0], &inv0);
				for (uint32_t c = 0; c < ncam; c++)
					math_pose_transform(&inv0, &m[c], &pose_in_origin[c]);
				done = true;
				break;
			}
		}
		if (!done) {
			fprintf(stderr, "no frame with all camera poses\n");
			return 1;
		}
	}
	if (getenv("CT_REPLAY_DUMP_BLOBS")) {
		// Every blob: camera, frame time, blob id, distorted centre, size, for background audits.
		for (auto &s : samples)
			for (uint32_t i = 0; i < s.blob_count; i++) {
				const t_blob &b = s.blobs[i];
				printf("BLOB,%u,%lld,%u,%.1f,%.1f,%.2f,%.2f,%.1f,%d,%.5f,%.5f\n", s.camera_index,
				       (long long)s.timestamp_ns, b.blob_id, b.center_distorted.x, b.center_distorted.y, b.size.x,
				       b.size.y, b.brightness, (int)b.matched_device_id, b.center_homogenized.x,
				       b.center_homogenized.y);
			}
		return 0;
	}
	if (getenv("CT_REPLAY_DUMP_BLOB_COUNTS")) {
		// Per camera frame: time and blob count, for startup timelines (LED sync calibration shows as on/off counts).
		for (auto &s : samples)
			printf("BLOBS,%u,%lld,%u\n", s.camera_index, (long long)s.timestamp_ns, (unsigned)s.blob_count);
		return 0;
	}
	if (getenv("CT_REPLAY_DUMP_CAM_POSES")) {
		// Recorded camera world poses, to recover each camera's mount against the record file's head poses.
		for (auto &s : samples)
			if (s.Txr_world_cam.has_value())
				printf("CAMPOSE,%u,%lld,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f\n", s.camera_index, (long long)s.timestamp_ns,
				       s.Txr_world_cam->position.x, s.Txr_world_cam->position.y, s.Txr_world_cam->position.z,
				       s.Txr_world_cam->orientation.x, s.Txr_world_cam->orientation.y,
				       s.Txr_world_cam->orientation.z, s.Txr_world_cam->orientation.w);
		return 0;
	}
	OriginSource origin;
	origin.base.get_tracked_pose = origin_get;
	for (auto &s : samples) {
		if (!s.Txr_world_cam.has_value())
			continue;
		xrt_pose inv_rel, p0;
		math_pose_invert(&pose_in_origin[s.camera_index], &inv_rel);
		math_pose_transform(&*s.Txr_world_cam, &inv_rel, &p0);
		if (origin.poses.empty() || origin.poses.back().first < s.timestamp_ns)
			origin.poses.push_back({s.timestamp_ns, p0});
	}

	if (getenv("CT_REPLAY_DUMP")) {
		for (auto &s : samples) {
			if (s.blob_count >= 6) {
				const auto &c = reader.mosaics[0].camera_calibrations[s.camera_index];
				printf("cam %u calib %ux%u fx %.1f cx %.1f model %d dist0 %.4f\n", s.camera_index,
				       c.image_size_pixels.w, c.image_size_pixels.h, c.intrinsics[0][0], c.intrinsics[0][2],
				       (int)c.distortion_model, c.distortion_parameters_as_array[0]);
				for (uint32_t i = 0; i < 4; i++) {
					const t_blob &b = s.blobs[i];
					printf("  blob %u dist (%.1f,%.1f) undist (%.1f,%.1f) homog (%.4f,%.4f,%.4f) size (%.1f,%.1f) "
					       "bright %.2f\n",
					       b.blob_id, b.center_distorted.x, b.center_distorted.y, b.center_undistorted.x,
					       b.center_undistorted.y, b.center_homogenized.x, b.center_homogenized.y,
					       b.center_homogenized.z, b.size.x, b.size.y, b.brightness);
				}
				break;
			}
		}
		{
			// Gate check on live accepted fixes of device 0: angle between the optical up and the replay IMU up.
			ReplayDevice tmp{};
			tmp.imu = &imu[0];
			int n = 0;
			double sum = 0;
			for (auto &s : samples) {
				for (uint32_t k = 0; k < s.device_count; k++) {
					auto &st = s.device_states[k];
					if (st.device_id != 0 || !st.found_pose.has_value() || !s.Txr_world_cam.has_value())
						continue;
					xrt_quat q_imu;
					if (!device_imu_orientation(&tmp.base, s.timestamp_ns, &q_imu))
						continue;
					xrt_pose Tcv_world_cam;
					math_pose_convert_from_opencv(&*s.Txr_world_cam, &Tcv_world_cam);
					xrt_quat inv;
					math_quat_invert(&Tcv_world_cam.orientation, &inv);
					xrt_vec3 up_cam = {0, 1, 0};
					math_quat_rotate_vec3(&inv, &up_cam, &up_cam);
					xrt_quat iq;
					math_quat_invert(&q_imu, &iq);
					xrt_vec3 up_dev = {0, 1, 0};
					math_quat_rotate_vec3(&iq, &up_dev, &up_dev);
					xrt_vec3 up_dev_in_cam;
					math_quat_rotate_vec3(&st.found_pose->Tcv_cam_device.orientation, &up_dev, &up_dev_in_cam);
					double d = up_dev_in_cam.x * up_cam.x + up_dev_in_cam.y * up_cam.y + up_dev_in_cam.z * up_cam.z;
					double ang = acos(std::clamp(d, -1.0, 1.0)) * 180 / M_PI;
					if (n < 5)
						printf("live fix t %lld: gate angle %.1f deg (q_imu %.3f %.3f %.3f %.3f)\n",
						       (long long)s.timestamp_ns, ang, q_imu.x, q_imu.y, q_imu.z, q_imu.w);
					sum += ang;
					n++;
				}
			}
			printf("%d live fixes, mean gate angle %.1f deg\n", n, n ? sum / n : 0);
		}
		for (auto &dd : reader.devices) {
			printf("device %d: %u leds, led0 pos (%.4f,%.4f,%.4f) normal (%.2f,%.2f,%.2f) r %.4f vis %.2f id %d\n",
			       (int)dd.id, dd.led_model.led_count, dd.leds[0].position.x, dd.leds[0].position.y,
			       dd.leds[0].position.z, dd.leds[0].normal.x, dd.leds[0].normal.y, dd.leds[0].normal.z,
			       dd.leds[0].radius_m, dd.leds[0].visibility_angle, (int)dd.leds[0].id);
		}
		return 0;
	}
	if (getenv("CT_REPLAY_CHECK_POSES")) {
		double maxpos = 0, maxang = 0;
		int n = 0;
		for (auto &s : samples) {
			if (!s.Txr_world_cam.has_value())
				continue;
			xrt_space_relation o;
			origin_get(&origin.base, s.timestamp_ns, &o);
			xrt_pose cam;
			math_pose_transform(&o.pose, &pose_in_origin[s.camera_index], &cam);
			xrt_vec3 d = cam.position;
			math_vec3_subtract(&s.Txr_world_cam->position, &d);
			double dp = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
			xrt_quat inv, dq;
			math_quat_invert(&s.Txr_world_cam->orientation, &inv);
			math_quat_rotate(&inv, &cam.orientation, &dq);
			double ang = 2 * acos(std::min(1.0, (double)fabsf(dq.w))) * 180 / M_PI;
			maxpos = std::max(maxpos, dp);
			maxang = std::max(maxang, ang);
			if (n++ < 3)
				printf("cam %u: pos diff %.4f m, rot diff %.2f deg\n", s.camera_index, dp, ang);
		}
		printf("%d samples: max pos diff %.4f m, max rot diff %.2f deg\n", n, maxpos, maxang);
		return 0;
	}
	t_constellation_tracker_params params = {};
	params.flags = realtime ? T_CONSTELLATION_TRACKER_FLAGS_NONE : T_CONSTELLATION_TRACKER_FLAGS_DETERMINISTIC;
	params.num_mosaics = 1;
	auto &mosaic = params.mosaics[0];
	mosaic.tracking_origin = &origin.base;
	mosaic.num_cameras = ncam;
	for (size_t c = 0; c < ncam; c++) {
		mosaic.cameras[c].calibration = reader.mosaics[0].camera_calibrations[c];
		mosaic.cameras[c].pose_in_origin = pose_in_origin[c];
		mosaic.cameras[c].has_concrete_pose = true;
	}
	xrt_frame_context xfctx = {};
	t_constellation_tracker *tracker = nullptr;
	if (t_constellation_tracker_create(&xfctx, &params, &tracker) != 0) {
		fprintf(stderr, "tracker create failed\n");
		return 1;
	}

	std::vector<ReplayDevice *> devs;
	for (auto &dd : reader.devices) {
		auto *d = new ReplayDevice{};
		d->index = (int)devs.size();
		d->imu = &imu[std::min<int>(d->index, 1)];
		d->base.push_constellation_tracker_sample = device_push_sample;
		d->base.get_imu_orientation = device_imu_orientation;
		if (g_synced_from_live)
			d->base.is_led_synced = device_led_synced;
		t_constellation_tracker_device_params dp = {};
		dp.led_model = dd.led_model;
		/*
		 * The recorder saves the tracker's copy of the model, already flipped into OpenCV axes (Device constructor);
		 * the tracker flips what it's given, so flip it back to OpenXR axes (the flip is its own inverse). Without
		 * this every replay pose came out 180 deg about the device X axis from live and failed the gravity gate.
		 */
		for (auto &led : dd.leds) {
			math_vec3_convert_from_opencv(&led.position, &led.position);
			math_vec3_convert_from_opencv(&led.normal, &led.normal);
		}
		dp.led_model.leds = dd.leds.data();
		// The dataset doesn't store the match parameters; the driver's.
		dp.led_model.match_parameters = DEFAULT_MATCH_PARAMETERS;
		dp.imu_calibration = PSSENSE_IMU_CALIBRATION;
		if (t_constellation_tracker_add_device(tracker, &dp, &d->base, &d->id) < 0) {
			fprintf(stderr, "add device failed\n");
			return 1;
		}
		d->imu_sink = dp.imu_sink;
		devs.push_back(d);
	}

	LabelBlobwatch bw;
	bw.base.mark_blob_device = bw_mark;
	bw.base.destroy = bw_destroy;

	size_t imu_next[2] = {0, 0};
	const int64_t t0 = samples.front().timestamp_ns;
	std::vector<t_blob> blobs;
	std::vector<double> frame_us; // per camera frame: processing time in the push (deterministic: fast + slow)
	const bool dump_frame_times = getenv("CT_REPLAY_DUMP_FRAME_TIMES") != nullptr;
	size_t frames = 0;
	const int64_t wall0 = os_monotonic_get_ns();
	for (auto &s : samples) {
		if (realtime) {
			int64_t due = wall0 + (s.timestamp_ns - t0);
			int64_t now = os_monotonic_get_ns();
			if (due > now)
				os_nanosleep(due - now);
		}
		if ((s.timestamp_ns - t0) / 1e9 > max_seconds)
			break;
		// IMU first, up to this frame's time.
		for (size_t di = 0; di < devs.size() && di < 2; di++) {
			auto &v = *devs[di]->imu;
			while (imu_next[di] < v.size() && v[imu_next[di]].t + imu_lag_ns <= s.timestamp_ns) {
				const ImuRow &r = v[imu_next[di]++];
				xrt_imu_sample is = {};
				is.timestamp_ns = r.t;
				is.accel_m_s2 = {r.a[0], r.a[1], r.a[2]};
				is.gyro_rad_secs = {r.g[0], r.g[1], r.g[2]};
				if (devs[di]->imu_sink)
					xrt_sink_push_imu(devs[di]->imu_sink, &is);
			}
		}
		blobs.assign(s.blobs, s.blobs + s.blob_count);
		// Live tracker's labels, before the replay's own replace them (to check no controller LED is dropped).
		std::vector<bool> live_led(blobs.size());
		for (size_t i = 0; i < blobs.size(); i++)
			live_led[i] = blobs[i].matched_device_id != XRT_CONSTELLATION_INVALID_DEVICE_ID;
		std::unique_lock<std::mutex> label_lock(bw.lock);
		auto &cam_labels = bw.labels[s.camera_index];
		for (auto &b : blobs) {
			// The dataset stores the homogenized centre's x and y only; the blobwatch always sets z = 1.
			b.center_homogenized.z = 1.0f;
			auto it = cam_labels.find(b.blob_id);
			if (it != cam_labels.end()) {
				b.matched_device_id = it->second.first;
				b.matched_device_led_id = it->second.second;
			} else {
				b.matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
				b.matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
			}
		}
		label_lock.unlock();
		bg_blobs_in += blobs.size();
		if (bg_map != nullptr && s.Txr_world_cam.has_value()) {
			std::vector<uint8_t> is_bg(blobs.size() + 1);
			int64_t q0 = os_monotonic_get_ns();
			t_blob_background_process(bg_map, s.camera_index, s.timestamp_ns, &s.Txr_world_cam->orientation,
			                          blobs.data(), (uint32_t)blobs.size(), (bool *)is_bg.data());
			bg_us.push_back((os_monotonic_get_ns() - q0) / 1000.0);
			size_t w = 0;
			for (size_t i = 0; i < blobs.size(); i++) {
				if (is_bg[i]) {
					bg_removed++;
					bg_led_removed += live_led[i] ? 1 : 0;
					continue;
				}
				blobs[w++] = blobs[i];
			}
			blobs.resize(w);
		}
		bg_live_leds += std::count(live_led.begin(), live_led.end(), true);
		g_feed_ns = s.timestamp_ns;
		for (const auto &b : std::vector<t_blob>(s.blobs, s.blobs + s.blob_count))
			if (b.matched_device_id >= 0 && b.matched_device_id < XRT_CONSTELLATION_MAX_DEVICES)
				g_live_seen_ns[b.matched_device_id] = s.timestamp_ns;
		bg_blobs_out += blobs.size();
		if (bg_only) {
			frames++;
			continue;
		}
		t_blob_observation tbo = {};
		tbo.source = &bw.base;
		tbo.id = s.camera_index;
		tbo.sequence_id = s.sequence_id;
		tbo.timestamp_ns = s.timestamp_ns;
		tbo.blobs = blobs.data();
		tbo.num_blobs = (uint32_t)blobs.size();
		t_blob_sink *sink = params.mosaics[0].cameras[s.camera_index].blob_sink;
		int64_t p0 = os_monotonic_get_ns();
		sink->push_blobs(sink, &tbo);
		frame_us.push_back((os_monotonic_get_ns() - p0) / 1000.0);
		if (dump_frame_times) {
			// Per camera frame: what made it slow (blob counts before / after the background map, how many of them
			// the live tracker had matched to LEDs) next to its processing time.
			printf("FT,%u,%lld,%u,%zu,%zu,%.0f\n", s.camera_index, (long long)s.timestamp_ns, (unsigned)s.blob_count,
			       blobs.size(), (size_t)std::count(live_led.begin(), live_led.end(), true), frame_us.back());
		}
		frames++;
	}

	if (realtime)
		os_nanosleep(500 * U_TIME_1MS_IN_NS); // let the threads finish the last frames
	{
		double bg_mean = 0, bg_max = 0;
		for (double v : bg_us) {
			bg_mean += v;
			bg_max = std::max(bg_max, v);
		}
		printf("blobs to the tracker: %.1f per camera frame of %.1f (%s); live controller LED blobs dropped: %llu of %llu "
		       "(%.2f%%); background map %.1f us per frame (max %.0f)\n",
		       frames ? (double)bg_blobs_out / frames : 0.0, frames ? (double)bg_blobs_in / frames : 0.0,
		       bg_map ? "background map on" : "off", (unsigned long long)bg_led_removed,
		       (unsigned long long)bg_live_leds, bg_live_leds ? 100.0 * bg_led_removed / bg_live_leds : 0.0,
		       bg_us.empty() ? 0.0 : bg_mean / bg_us.size(), bg_max);
	}
	for (auto *d : devs)
		d->lock.lock(); // no more fixes while reporting (released by _exit)

	// Report.
	std::vector<int64_t> frame_times;
	for (auto &s : samples)
		if ((s.timestamp_ns - t0) / 1e9 <= max_seconds)
			frame_times.push_back(s.timestamp_ns);
	std::sort(frame_times.begin(), frame_times.end());
	frame_times.erase(std::unique(frame_times.begin(), frame_times.end()), frame_times.end());

	{
		// Every fix with its camera (out.csv.fixes), for per-camera jump and cross-camera checks.
		std::string fp = std::string(argv[4]) + ".fixes";
		FILE *ff = fopen(fp.c_str(), "w");
		if (ff) {
			fprintf(ff, "device,t_ns,camera,x,y,z,qx,qy,qz,qw\n");
			for (auto *d : devs)
				for (auto &f : d->fixes)
					fprintf(ff, "%d,%lld,%u,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f\n", d->index, (long long)f.t,
					        f.camera, f.pose.position.x, f.pose.position.y, f.pose.position.z, f.pose.orientation.x,
					        f.pose.orientation.y, f.pose.orientation.z, f.pose.orientation.w);
			fclose(ff);
		}
	}
	FILE *csv = fopen(argv[4], "w");
	fprintf(csv, "t_ns");
	for (auto *d : devs)
		fprintf(csv, ",fix%d,x%d,y%d,z%d,cam%d,ncam%d", d->index, d->index, d->index, d->index, d->index, d->index);
	fprintf(csv, "\n");
	std::vector<std::map<int64_t, xrt_pose>> fix_at(devs.size());
	std::vector<std::map<int64_t, std::pair<uint32_t, int>>> fix_cam(devs.size()); // last camera, cameras solving
	for (size_t di = 0; di < devs.size(); di++)
		for (auto &f : devs[di]->fixes) {
			fix_at[di][f.t] = f.pose;
			auto &fc = fix_cam[di][f.t];
			fc.first = f.camera;
			fc.second++;
		}
	for (int64_t t : frame_times) {
		fprintf(csv, "%lld", (long long)t);
		for (size_t di = 0; di < devs.size(); di++) {
			auto it = fix_at[di].find(t);
			if (it != fix_at[di].end())
				fprintf(csv, ",1,%.4f,%.4f,%.4f,%u,%d", it->second.position.x, it->second.position.y,
				        it->second.position.z, fix_cam[di][t].first, fix_cam[di][t].second);
			else
				fprintf(csv, ",0,,,,,");
		}
		fprintf(csv, "\n");
	}
	fclose(csv);

	// Up direction of each fix vs the device's IMU (the gravity gate's test), to spot flipped LED correspondences.
	for (size_t di = 0; di < devs.size(); di++) {
		int over = 0, n = 0;
		for (auto &f : devs[di]->fixes) {
			xrt_quat q_imu;
			if (!device_imu_orientation(&devs[di]->base, f.t, &q_imu))
				continue;
			xrt_vec3 up = {0, 1, 0}, a, b;
			xrt_quat inv;
			math_quat_invert(&f.pose.orientation, &inv);
			math_quat_rotate_vec3(&inv, &up, &a);
			math_quat_invert(&q_imu, &inv);
			math_quat_rotate_vec3(&inv, &up, &b);
			double ang = acos(std::clamp((double)(a.x * b.x + a.y * b.y + a.z * b.z), -1.0, 1.0)) * 180 / M_PI;
			over += ang > 35;
			n++;
		}
		printf("device %zu: %d of %d fixes more than 35 deg from the IMU up\n", di, over, n);
	}
	// Replay vs live pose for the same device, camera and frame.
	{
		std::map<std::tuple<int, uint32_t, int64_t>, xrt_pose> live; // device, camera, t -> Tcv_cam_device
		std::map<std::pair<uint32_t, int64_t>, xrt_pose> camw;
		for (auto &s : samples) {
			if (s.Txr_world_cam.has_value())
				camw[{s.camera_index, s.timestamp_ns}] = *s.Txr_world_cam;
			for (uint32_t k = 0; k < s.device_count; k++)
				if (s.device_states[k].found_pose.has_value())
					live[{(int)s.device_states[k].device_id, s.camera_index, s.timestamp_ns}] =
					    s.device_states[k].found_pose->Tcv_cam_device;
		}
		for (size_t di = 0; di < devs.size(); di++) {
			int n = 0;
			for (auto &f : devs[di]->fixes) {
				auto it = live.find({(int)di, f.camera, f.t});
				auto cw = camw.find({f.camera, f.t});
				if (it == live.end() || cw == camw.end())
					continue;
				xrt_pose inv_cam, xr_cam_dev, cv_cam_dev;
				math_pose_invert(&cw->second, &inv_cam);
				math_pose_transform(&inv_cam, &f.pose, &xr_cam_dev);
				math_pose_convert_from_opencv(&xr_cam_dev, &cv_cam_dev); // the axis flip is its own inverse
				xrt_quat inv, dq;
				math_quat_invert(&it->second.orientation, &inv);
				math_quat_rotate(&inv, &cv_cam_dev.orientation, &dq);
				xrt_vec3 d = cv_cam_dev.position;
				math_vec3_subtract(&it->second.position, &d);
				if (n++ < 4)
					printf("dev %zu cam %u: live->replay rotation (%.3f %.3f %.3f %.3f) = %.1f deg, pos diff %.3f m\n",
					       di, f.camera, dq.x, dq.y, dq.z, dq.w,
					       2 * acos(std::min(1.0, (double)fabsf(dq.w))) * 180 / M_PI,
					       sqrt(d.x * d.x + d.y * d.y + d.z * d.z));
			}
			printf("dev %zu: %d frames solved both live and in replay\n", di, n);
		}
	}
	if (!realtime && !frame_us.empty()) {
		std::vector<double> v = frame_us;
		std::sort(v.begin(), v.end());
		double sum = 0;
		for (double x : v)
			sum += x;
		auto pct = [&](double q) { return v[std::min(v.size() - 1, (size_t)(q * v.size()))]; };
		printf("per camera frame processing: mean %.0f us, p50 %.0f, p95 %.0f, p99 %.0f, max %.0f us\n",
		       sum / v.size(), pct(0.5), pct(0.95), pct(0.99), v.back());
	}
	printf("%zu camera frames over %.1f s\n", frames, (frame_times.back() - frame_times.front()) / 1e9);
	for (size_t di = 0; di < devs.size(); di++) {
		auto &m = fix_at[di];
		if (m.empty()) {
			printf("device %zu: no fixes\n", di);
			continue;
		}
		int64_t first = m.begin()->first, last = m.rbegin()->first;
		size_t in_span = 0, fixed = 0;
		int gaps = 0, g01 = 0, g03 = 0, g1 = 0;
		double gap_time = 0, max_jump = 0;
		int jumps = 0;
		int64_t prev_t = -1;
		xrt_pose prev_p{};
		for (int64_t t : frame_times) {
			if (t < first || t > last)
				continue;
			in_span++;
			auto it = m.find(t);
			if (it == m.end())
				continue;
			fixed++;
			if (prev_t >= 0) {
				double g = (t - prev_t) / 1e9;
				if (g > 0.1) {
					gaps++;
					gap_time += g;
					if (g < 0.3)
						g01++;
					else if (g < 1)
						g03++;
					else
						g1++;
				}
				xrt_vec3 d = it->second.position;
				math_vec3_subtract(&prev_p.position, &d);
				double dist = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
				// Faster than 4 m/s plus 2 cm between fixes: a jump, not motion.
				if (dist > 0.02 + 4.0 * g) {
					jumps++;
					max_jump = std::max(max_jump, dist);
				}
			}
			prev_t = t;
			prev_p = it->second;
		}
		double span = (last - first) / 1e9;
		printf("device %zu: first fix %.2f s; fixes in %.0f%% of frame times; %d gaps >0.1 s (%.1f s, %.0f%% of %.0f "
		       "s): %d <0.3 s, %d 0.3-1 s, %d >1 s; %d jumps (max %.2f m)\n",
		       di, (first - frame_times.front()) / 1e9, 100.0 * fixed / std::max<size_t>(1, in_span), gaps,
		       gap_time, 100.0 * gap_time / std::max(1e-9, span), span, g01, g03, g1, jumps, max_jump);
	}
	// The tracker's threads may still be running: leave without destructors (the dataset is still referenced).
	fflush(stdout);
	_exit(0);
}
