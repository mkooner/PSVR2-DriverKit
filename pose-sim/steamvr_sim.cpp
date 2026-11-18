// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//
// What SteamVR shows from our head pose, replayed from a PSVR2_RECORD_FILE recording.
//
// The driver (ovrd_driver.cpp CDeviceDriver_Monado::GetPose, ~1 kHz) asks the headset for the pose "now"
// (poseTimeOffset 0) and hands SteamVR that pose plus the pose filter's linear velocity and its angular velocity
// (the latest raw gyro sample). SteamVR extrapolates to photon time with them. This replays the recording through the
// real psvr2_pose_filter and the real clock conversion (m_clock_offset_a2b), samples the driver's output on a 90 Hz
// frame grid, extrapolates it SteamVR-style over H, and reports:
//   jitter : the frame-to-frame change of the shown rotation rate and velocity (deg/frame^2, mm/frame^2), still and
//            moving frames separately; real head motion barely changes between 11 ms frames, noise does.
//   error  : the extrapolated pose against the driver's own "now" pose when that time comes (rotation deg, mm).
// for H = 0 / 11 / 22 / 33 ms, and for alternative angular velocity sources (analysis only, the driver is unchanged):
//   raw    : the latest gyro sample (what the driver sends)
//   avgN   : mean of the last N ms of gyro
//
// Usage: steamvr_sim record.csv [frames.csv]   (frames to stdout without a path)
#include "psvr2_imu_time.h"
#include "math/m_api.h"
#include "math/m_vec3.h"
#include "math/m_clock_tracking.h"
#include "util/u_time.h"
#include "psvr2_pose_filter.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

struct Event
{
	int64_t host;
	int type; // 0 imu, 1 slam
	size_t idx;
};
struct Imu
{
	int64_t vts;
	xrt_vec3 gyro;
	xrt_vec3 accel;
};

static xrt_quat
extrapolate(const xrt_quat &q, const xrt_vec3 &w_world, double h)
{
	xrt_vec3 aa = m_vec3_mul_scalar(w_world, (float)h);
	xrt_quat dq;
	math_quat_exp_so3(&aa, &dq);
	xrt_quat out;
	math_quat_rotate(&dq, &q, &out);
	math_quat_normalize(&out);
	return out;
}

static double
angle_deg(const xrt_quat &a, const xrt_quat &b)
{
	double d = fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
	return 2 * acos(std::min(1.0, d)) * 180 / M_PI;
}

// rotation vector from a to b (world frame), rad
static xrt_vec3
rotvec(const xrt_quat &a, const xrt_quat &b)
{
	xrt_quat ai, d;
	math_quat_invert(&a, &ai);
	math_quat_rotate(&b, &ai, &d);
	if (d.w < 0) {
		d.x = -d.x, d.y = -d.y, d.z = -d.z, d.w = -d.w;
	}
	xrt_vec3 v;
	math_quat_ln_so3(&d, &v);
	return v;
}

struct Stat
{
	std::vector<double> v;
	void
	add(double x)
	{
		v.push_back(x);
	}
	double
	rms() const
	{
		double s = 0;
		for (double x : v)
			s += x * x;
		return v.empty() ? 0 : sqrt(s / v.size());
	}
	double
	pct(double p) const
	{
		if (v.empty())
			return 0;
		std::vector<double> c = v;
		size_t k = std::min(c.size() - 1, (size_t)(p / 100 * c.size()));
		std::nth_element(c.begin(), c.begin() + k, c.end());
		return c[k];
	}
};

int
main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "usage: %s record.csv [frames.csv]\n", argv[0]);
		return 1;
	}
	const char *csv_path = argc > 2 ? argv[2] : nullptr;
	std::vector<Imu> imu;
	std::vector<std::pair<int64_t, xrt_pose>> slam;
	std::vector<Event> ev;
	FILE *f = fopen(argv[1], "r");
	if (!f) {
		perror(argv[1]);
		return 1;
	}
	char line[512];
	while (fgets(line, sizeof(line), f)) {
		long long host, vts;
		float v[9];
		if (line[0] == 'I' && sscanf(line, "I,%lld,%lld,%f,%f,%f,%f,%f,%f", &host, &vts, &v[0], &v[1], &v[2], &v[3],
		                             &v[4], &v[5]) >= 5) {
			// SIM_IMU_TIME=1: the driver's sample time fix (psvr2_imu_time.h); recordings hold the raw VTS.
			static psvr2_imu_time itime = {};
			if (getenv("SIM_IMU_TIME") && atoi(getenv("SIM_IMU_TIME")) != 0)
				vts = psvr2_imu_time_fix(&itime, vts);
			imu.push_back({vts, {v[0], v[1], v[2]}, {v[3], v[4], v[5]}});
			ev.push_back({host, 0, imu.size() - 1});
		} else if (line[0] == 'S' && sscanf(line, "S,%lld,%lld,%f,%f,%f,%f,%f,%f,%f", &host, &vts, &v[0], &v[1],
		                                    &v[2], &v[3], &v[4], &v[5], &v[6]) == 9) {
			slam.push_back({vts, xrt_pose{{v[3], v[4], v[5], v[6]}, {v[0], v[1], v[2]}}});
			ev.push_back({host, 1, slam.size() - 1});
		}
	}
	fclose(f);
	std::stable_sort(ev.begin(), ev.end(), [](const Event &a, const Event &b) { return a.host < b.host; });
	if (ev.empty()) {
		return 1;
	}

	psvr2_pose_filter_params params;
	psvr2_pose_filter_default_params(&params);
	if (getenv("SIM_ORIENT_TAU"))
		params.orient_tau_s = atof(getenv("SIM_ORIENT_TAU"));
	if (getenv("SIM_POS_TAU"))
		params.pos_tau_s = atof(getenv("SIM_POS_TAU"));
	if (getenv("SIM_POS_SNAP"))
		params.pos_snap_m = atof(getenv("SIM_POS_SNAP"));
	if (getenv("SIM_ORIENT_SNAP_DEG"))
		params.orient_snap_rad = (float)(atof(getenv("SIM_ORIENT_SNAP_DEG")) * M_PI / 180.0);
	if (getenv("SIM_POS_BLEND_MS"))
		params.pos_blend_ns = (time_duration_ns)(atof(getenv("SIM_POS_BLEND_MS")) * 1e6);
	// SIM_ACCEL=1: accelerometer-aided position (accel_enable); SIM_GRAVITY_TAU: its gravity/bias average (s).
	if (getenv("SIM_ACCEL"))
		params.accel_enable = atoi(getenv("SIM_ACCEL")) != 0;
	// SIM_GRAVITY_PAUSE_DEG=30: pause the gravity average above this turn rate (driver PSVR2_GRAVITY_PAUSE_DEG).
	if (getenv("SIM_GRAVITY_PAUSE_DEG"))
		params.accel_gravity_pause_rad_s = (float)(atof(getenv("SIM_GRAVITY_PAUSE_DEG")) * M_PI / 180.0);
	// SIM_DUMP_RESIDUALS=file: one line per SLAM update (vts, rate rad/s, residual m, reset, skipped).
	FILE *resid_csv = getenv("SIM_DUMP_RESIDUALS") ? fopen(getenv("SIM_DUMP_RESIDUALS"), "w") : nullptr;
	// SIM_GYRO_BIAS_TAU: gyro bias estimation time constant (s), 0 off.
	if (getenv("SIM_GYRO_BIAS_TAU"))
		params.gyro_bias_tau_s = atof(getenv("SIM_GYRO_BIAS_TAU"));
	if (getenv("SIM_GRAVITY_TAU"))
		params.accel_gravity_tau_s = atof(getenv("SIM_GRAVITY_TAU"));
	if (getenv("SIM_POS_CONFIRM"))
		params.pos_confirm_jumps = atoi(getenv("SIM_POS_CONFIRM")) != 0;
	// SIM_LEVER="x,y,z": the filter's pivot_offset (head frame, metres; the driver default is 0,0,-0.10)
	if (getenv("SIM_LEVER"))
		sscanf(getenv("SIM_LEVER"), "%f,%f,%f", &params.pivot_offset.x, &params.pivot_offset.y,
		       &params.pivot_offset.z);
	static psvr2_pose_filter filt;
	psvr2_pose_filter_init(&filt, &params);
	// As the driver: SLAM positions go through the reset carry-over before the filter (SIM_CARRY=0 to skip).
	psvr2_slam_carry carry;
	psvr2_slam_carry_init(&carry, !(getenv("SIM_CARRY") && atoi(getenv("SIM_CARRY")) == 0));
	time_duration_ns hw2mono = 0;

	const int64_t frame_ns = 11122334; // 89.91 Hz
	const int wms[] = {0, 5, 10, 20};
	const int NW = 4;

	struct Frame
	{
		int64_t t;
		xrt_quat q;
		xrt_vec3 p, v;
		xrt_vec3 w[4];
		double speed; // deg/s
		bool valid;
		xrt_vec3 slam_p; // the headset's SLAM position at this frame's time (interpolated, offline reference)
		bool have_slam;
	};
	std::vector<Frame> frames;
	std::deque<std::pair<int64_t, xrt_vec3>> recent; // world-frame gyro history for averaging (by vts)

	int64_t next_frame = ev.front().host + 15000000000LL; // skip startup (SLAM resets)
	size_t imu_seen = 0;
	for (size_t i = 0; i < ev.size(); i++) {
		const Event &e = ev[i];
		while (next_frame <= e.host) {
			Frame fr{};
			fr.t = next_frame;
			xrt_space_relation r;
			if (imu_seen > 200 && psvr2_pose_filter_get(&filt, next_frame - hw2mono, &r)) {
				fr.valid = true;
				fr.q = r.pose.orientation;
				fr.p = r.pose.position;
				fr.v = r.linear_velocity;

				fr.w[0] = r.angular_velocity;
				fr.speed = m_vec3_len(r.angular_velocity) * 180 / M_PI;
				int64_t vts_q = next_frame - hw2mono;
				auto it = std::lower_bound(slam.begin(), slam.end(), vts_q,
				                           [](const std::pair<int64_t, xrt_pose> &a, int64_t v) { return a.first < v; });
				if (it != slam.begin() && it != slam.end() && it->first - (it - 1)->first < 40000000) {
					double u = double(vts_q - (it - 1)->first) / double(it->first - (it - 1)->first);
					xrt_vec3 a = (it - 1)->second.position, b = it->second.position;
					fr.slam_p = m_vec3_add(a, m_vec3_mul_scalar(m_vec3_sub(b, a), (float)u));
					fr.have_slam = true;
				}
				for (int k = 1; k < NW; k++) {
					xrt_vec3 s = {0, 0, 0};
					int n = 0;
					int64_t newest = recent.empty() ? 0 : recent.back().first;
					for (auto it = recent.rbegin(); it != recent.rend(); ++it) {
						if (newest - it->first > (int64_t)wms[k] * 1000000)
							break;
						s = m_vec3_add(s, it->second);
						n++;
					}
					// mean gyro in the body frame, rotated to world by the current orientation (as the driver does)
					xrt_vec3 body = n ? m_vec3_mul_scalar(s, 1.0f / n) : xrt_vec3{0, 0, 0};
					math_quat_rotate_vec3(&fr.q, &body, &fr.w[k]);
				}
			}
			frames.push_back(fr);
			next_frame += frame_ns;
		}
		if (e.type == 0) {
			const Imu &s = imu[e.idx];
			// SIM_DEQUANT=1: the headset's vts ticks every 1 ms, so every second 2 kHz sample repeats the previous
			// timestamp (and the filter drops it); stamp it 0.5 ms later instead.
			int64_t vts = s.vts;
			static int64_t last_vts = 0, last_raw = 0;
			if (getenv("SIM_DEQUANT") && atoi(getenv("SIM_DEQUANT")) && vts == last_raw)
				vts = last_vts + 500000;
			last_raw = s.vts;
			last_vts = vts;
			m_clock_offset_a2b(2000.0f, s.vts, e.host, &hw2mono);
			psvr2_pose_filter_push_gyro(&filt, vts, &s.gyro);
			psvr2_pose_filter_push_accel(&filt, vts, &s.accel);
			recent.push_back({s.vts, s.gyro});
			while (!recent.empty() && s.vts - recent.front().first > 30000000)
				recent.pop_front();
			imu_seen++;
		} else {
			xrt_pose sp = slam[e.idx].second;
			sp.position = psvr2_slam_carry_push(&carry, slam[e.idx].first, &sp.position, nullptr, nullptr);
			psvr2_pose_filter_update upd{};
			psvr2_pose_filter_push_slam(&filt, slam[e.idx].first, &sp, &upd);
			if (resid_csv) {
				fprintf(resid_csv, "%lld,%.4f,%.6f,%d,%d\n", (long long)slam[e.idx].first, upd.rate_rad_s,
				        upd.pos_residual_m, (int)upd.position_reset, (int)upd.position_skipped);
			}
		}
	}

	fprintf(stderr, "%s: %zu frames (%.0f s), SLAM %zu\n", argv[1], frames.size(), frames.size() * frame_ns * 1e-9,
	        slam.size());
	// Per-frame dump of what the driver hands SteamVR; analysed by analysis/jitter/shown_jitter.py.
	FILE *csv = csv_path ? fopen(csv_path, "w") : stdout;
	fprintf(csv, "t_s,valid,qx,qy,qz,qw,px,py,pz,vx,vy,vz,wx,wy,wz,a5x,a5y,a5z,a10x,a10y,a10z,a20x,a20y,a20z,"
	             "have_slam,sx,sy,sz\n");
	for (const Frame &fr : frames) {
		fprintf(csv, "%.6f,%d,%.7f,%.7f,%.7f,%.7f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f", (fr.t - frames[0].t) * 1e-9,
		        (int)fr.valid, fr.q.x, fr.q.y, fr.q.z, fr.q.w, fr.p.x, fr.p.y, fr.p.z, fr.v.x, fr.v.y, fr.v.z);
		for (int k = 0; k < NW; k++)
			fprintf(csv, ",%.6f,%.6f,%.6f", fr.w[k].x, fr.w[k].y, fr.w[k].z);
		fprintf(csv, ",%d,%.6f,%.6f,%.6f\n", (int)fr.have_slam, fr.slam_p.x, fr.slam_p.y, fr.slam_p.z);
	}
	if (csv != stdout)
		fclose(csv);
	if (resid_csv) fclose(resid_csv);
	fprintf(stderr, "filter: accel %s, have_accel %d, jumps skipped %u, gyro bias (%.3f, %.3f, %.3f) deg/s\n",
	        params.accel_enable ? "on" : "off", (int)filt.have_accel, filt.jumps_skipped, filt.gyro_bias.x * 57.29578,
	        filt.gyro_bias.y * 57.29578, filt.gyro_bias.z * 57.29578);
	return 0;
}
