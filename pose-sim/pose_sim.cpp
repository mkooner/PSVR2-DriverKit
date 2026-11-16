// PS VR2 head pose jitter simulator.
//
// Generates a known head trajectory, feeds simulated headset data (2kHz gyro in USB packets with receive jitter,
// SLAM poses with noise and latency) through:
//   OLD  : the current driver pipeline, using Monado's real code (m_clock_offset_a2b, m_ff_vec3_f32 gyro fifo,
//          m_relation_history with motion estimation, t_apply_dead_reckoning)
//   NEW  : psvr2_pose_filter + m_clock_windowed_skew_tracker
// and queries both like a compositor (90Hz, predicting ahead), measuring error vs ground truth and jitter
// (frame-to-frame error that the true motion doesn't have).
//
// Usage: pose_sim [slam_hz] [prediction_ms] [seconds]

#include "math/m_api.h"
#include "math/m_vec3.h"
#include "math/m_filter_fifo.h"
#include "math/m_relation_history.h"
#include "math/m_clock_tracking.h"
#include "tracking/t_dead_reckoning.h"
#include "util/u_time.h"

#include "psvr2_pose_filter.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
#include <algorithm>
#include <string>

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kImuHz = 2000.0;
static constexpr int64_t kImuPeriodNs = 500000;

struct Truth
{
	xrt_pose pose;
};

// Head motion: yaw/pitch/roll sinusoids plus a position sway, typical-ish of looking around.
static double g_motion_scale = 1.0;

static xrt_pose
truth_at(double t)
{
	const double k = g_motion_scale;
	double yaw = k * (0.7 * sin(2 * kPi * 0.6 * t) + 0.3 * sin(2 * kPi * 1.7 * t + 0.4));
	double pitch = k * 0.25 * sin(2 * kPi * 0.45 * t + 1.0);
	double roll = k * 0.08 * sin(2 * kPi * 0.9 * t + 2.0);
	xrt_vec3 euler = {(float)pitch, (float)yaw, (float)roll};
	xrt_pose p;
	math_quat_from_euler_angles(&euler, &p.orientation);
	p.position = {(float)(k * 0.08 * sin(2 * kPi * 0.5 * t)), (float)(1.6 + k * 0.03 * sin(2 * kPi * 0.8 * t + 0.5)),
	              (float)(k * 0.05 * sin(2 * kPi * 0.35 * t + 1.3))};
	return p;
}

static xrt_vec3
body_gyro_at(double t)
{
	const double h = 1e-5;
	xrt_pose a = truth_at(t - h), b = truth_at(t + h);
	xrt_quat ainv, d;
	math_quat_invert(&a.orientation, &ainv);
	math_quat_rotate(&ainv, &b.orientation, &d); // body frame delta
	xrt_vec3 aa;
	math_quat_ln_so3(&d, &aa);
	return m_vec3_mul_scalar(aa, (float)(1.0 / (2 * h)));
}

static double
angle_between(const xrt_quat &a, const xrt_quat &b)
{
	double d = fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
	d = std::min(1.0, d);
	return 2.0 * acos(d);
}

struct Event
{
	int64_t host_ns;
	int type; // 0 imu packet, 1 slam, 2 query
	size_t idx;
};

struct Metrics
{
	std::vector<double> ang_err, pos_err, ang_jit, pos_jit;
	void
	print(const char *name) const
	{
		auto rms = [](const std::vector<double> &v) {
			double s = 0;
			for (double x : v)
				s += x * x;
			return v.empty() ? 0.0 : sqrt(s / v.size());
		};
		auto p99 = [](std::vector<double> v) {
			if (v.empty())
				return 0.0;
			std::sort(v.begin(), v.end());
			return v[(size_t)(v.size() * 0.99)];
		};
		printf("  %-26s err: %6.3f deg %6.2f mm | jitter: %6.4f deg (p99 %6.4f) %6.3f mm (p99 %6.3f)\n", name,
		       rms(ang_err) * 180 / kPi, rms(pos_err) * 1000, rms(ang_jit) * 180 / kPi,
		       p99(ang_jit) * 180 / kPi, rms(pos_jit) * 1000, p99(pos_jit) * 1000);
	}
};

// Jitter: second difference of (output - truth) orientation/position error across consecutive queries.
struct JitterTracker
{
	bool have1 = false, have2 = false;
	xrt_vec3 e1{}, e2{}; // rotation error (axis-angle) history
	xrt_vec3 p1{}, p2{};
	void
	push(const xrt_pose &out, const xrt_pose &truth, Metrics &m)
	{
		xrt_quat tinv, err;
		math_quat_invert(&truth.orientation, &tinv);
		math_quat_rotate(&out.orientation, &tinv, &err);
		if (err.w < 0) {
			err.x = -err.x;
			err.y = -err.y;
			err.z = -err.z;
			err.w = -err.w;
		}
		xrt_vec3 e;
		math_quat_ln_so3(&err, &e);
		xrt_vec3 p = m_vec3_sub(out.position, truth.position);
		if (have2) {
			xrt_vec3 d2 = m_vec3_add(m_vec3_sub(e, m_vec3_mul_scalar(e1, 2)), e2);
			xrt_vec3 q2 = m_vec3_add(m_vec3_sub(p, m_vec3_mul_scalar(p1, 2)), p2);
			m.ang_jit.push_back(m_vec3_len(d2));
			m.pos_jit.push_back(m_vec3_len(q2));
		}
		e2 = e1;
		p2 = p1;
		e1 = e;
		p1 = p;
		have2 = have1;
		have1 = true;
	}
};

int
main(int argc, char **argv)
{
	// Replay mode: pose_sim --replay <PSVR2_RECORD_FILE csv> [prediction_ms]
	const bool replay = argc > 2 && std::string(argv[1]) == "--replay";
	const char *replay_path = replay ? argv[2] : nullptr;
	if (replay) {
		argv++;
		argc--;
	}
	double slam_hz = (argc > 1 && !replay) ? atof(argv[1]) : 60.0;
	double prediction_ms = argc > 2 ? atof(argv[2]) : 20.0;
	double seconds = argc > 3 ? atof(argv[3]) : 30.0;
	if (getenv("SIM_MOTION_SCALE"))
		g_motion_scale = atof(getenv("SIM_MOTION_SCALE"));

	// Simulation knobs (headset values unknown, plausible defaults)
	int imu_per_packet = replay ? 1 : 4;       // IMU samples per USB status packet
	const double gyro_noise = 0.004;           // rad/s rms per sample
	const xrt_vec3 gyro_bias = {0.004f, -0.003f, 0.002f}; // rad/s (~0.2 deg/s)
	const double slam_ang_noise = 0.05 * kPi / 180; // rad rms
	const double slam_pos_noise = 0.0007;      // m rms
	const double slam_latency_ms = 8.0;        // SLAM pose arrives this long after its timestamp
	const double usb_base_ms = 0.25, usb_jitter_mean_ms = 0.4, usb_spike_prob = 0.02, usb_spike_ms = 3.0;
	const int64_t vts_to_host = 123456789000LL; // true clock offset

	std::mt19937_64 rng(42);
	std::normal_distribution<double> n01(0, 1);
	std::exponential_distribution<double> jit(1.0 / usb_jitter_mean_ms);
	std::uniform_real_distribution<double> u01(0, 1);

	// --- Generate data in VTS time
	struct ImuSample
	{
		int64_t vts;
		xrt_vec3 gyro;
	};
	std::vector<ImuSample> imu;
	std::vector<std::pair<int64_t, xrt_pose>> slam;
	std::vector<Event> events;
	int64_t start = 1000000000LL, end = start + (int64_t)(seconds * 1e9);
	if (replay) {
		FILE *f = fopen(replay_path, "r");
		if (!f) {
			perror(replay_path);
			return 1;
		}
		char line[512];
		int64_t first_host = 0, last_host = 0;
		while (fgets(line, sizeof(line), f)) {
			long long host, vts;
			float v[7];
			if (line[0] == 'I' &&
			    sscanf(line, "I,%lld,%lld,%f,%f,%f", &host, &vts, &v[0], &v[1], &v[2]) == 5) {
				imu.push_back({vts, {v[0], v[1], v[2]}});
				events.push_back({host, 0, imu.size() - 1});
			} else if (line[0] == 'S' && sscanf(line, "S,%lld,%lld,%f,%f,%f,%f,%f,%f,%f", &host, &vts, &v[0],
			                                    &v[1], &v[2], &v[3], &v[4], &v[5], &v[6]) == 9) {
				xrt_pose p = {{v[3], v[4], v[5], v[6]}, {v[0], v[1], v[2]}};
				slam.push_back({vts, p});
				events.push_back({host, 1, slam.size() - 1});
			} else {
				continue;
			}
			if (first_host == 0)
				first_host = host;
			last_host = host;
		}
		fclose(f);
		for (int64_t t = first_host + 2000000000LL; t < last_host - 100000000LL; t += 11111111) {
			events.push_back({t, 2, 0});
		}
		std::stable_sort(events.begin(), events.end(),
		                 [](const Event &a, const Event &b) { return a.host_ns < b.host_ns; });
		seconds = (last_host - first_host) * 1e-9;
		slam_hz = slam.size() > 1 ? (slam.size() - 1) / ((slam.back().first - slam.front().first) * 1e-9) : 0;
		printf("Replaying %s: %zu IMU samples, %zu SLAM poses (%.1f Hz), %.1f s\n", replay_path, imu.size(),
		       slam.size(), slam_hz, seconds);
		printf("  (no ground truth: err columns are meaningless, compare the jitter columns)\n");
		g_motion_scale = 0.0;
	}
	for (int64_t t = start; !replay && t < end; t += kImuPeriodNs) {
		xrt_vec3 g = body_gyro_at(t * 1e-9);
		g = m_vec3_add(g, gyro_bias);
		g.x += gyro_noise * n01(rng);
		g.y += gyro_noise * n01(rng);
		g.z += gyro_noise * n01(rng);
		imu.push_back({t, g});
	}
	int64_t slam_period = (int64_t)(1e9 / slam_hz);
	for (int64_t t = start + 5000000; !replay && t < end; t += slam_period) {
		xrt_pose p = truth_at(t * 1e-9);
		xrt_vec3 n = {(float)(slam_ang_noise * n01(rng)), (float)(slam_ang_noise * n01(rng)),
		              (float)(slam_ang_noise * n01(rng))};
		xrt_quat dq;
		math_quat_exp_so3(&n, &dq);
		math_quat_rotate(&dq, &p.orientation, &p.orientation);
		p.position.x += slam_pos_noise * n01(rng);
		p.position.y += slam_pos_noise * n01(rng);
		p.position.z += slam_pos_noise * n01(rng);
		slam.push_back({t, p});
	}

	// --- Events in host time
	for (size_t i = imu_per_packet - 1; !replay && i < imu.size(); i += imu_per_packet) {
		double delay_ms = usb_base_ms + jit(rng) + (u01(rng) < usb_spike_prob ? usb_spike_ms : 0);
		events.push_back({imu[i].vts + vts_to_host + (int64_t)(delay_ms * 1e6), 0, i});
	}
	for (size_t i = 0; !replay && i < slam.size(); i++) {
		events.push_back({slam[i].first + vts_to_host + (int64_t)(slam_latency_ms * 1e6), 1, i});
	}
	for (int64_t t = start + 2000000000LL; !replay && t < end - 100000000LL; t += 11111111) {
		events.push_back({t + vts_to_host, 2, 0});
	}
	std::stable_sort(events.begin(), events.end(),
	                 [](const Event &a, const Event &b) { return a.host_ns < b.host_ns; });

	// --- Pipelines
	// OLD
	m_ff_vec3_f32 *ff_gyro = nullptr;
	m_ff_vec3_f32_alloc(&ff_gyro, 1024);
	m_relation_history *hist = nullptr;
	m_relation_history_create(&hist);
	time_duration_ns hw2mono_ema = 0;
	// NEW clock
	m_clock_windowed_skew_tracker *skew = m_clock_windowed_skew_tracker_alloc(2048);
	// NEW filters (one with the old clock, one with the new, to separate the effects)
	psvr2_pose_filter_params params;
	psvr2_pose_filter_default_params(&params);
	if (getenv("SIM_ORIENT_TAU"))
		params.orient_tau_s = atof(getenv("SIM_ORIENT_TAU"));
	if (getenv("SIM_POS_TAU"))
		params.pos_tau_s = atof(getenv("SIM_POS_TAU"));
	static psvr2_pose_filter filt_a, filt_b;
	psvr2_pose_filter_init(&filt_a, &params);
	psvr2_pose_filter_init(&filt_b, &params);

	Metrics m_old, m_new_ema, m_new;
	JitterTracker j_old, j_new_ema, j_new;
	double clk_err_ema = 0, clk_err_skew = 0;
	int clk_n = 0;
	int samples_seen = 0;

	for (const Event &ev : events) {
		if (ev.type == 0) {
			size_t last = ev.idx;
			for (int k = imu_per_packet - 1; k >= 0; k--) {
				const ImuSample &s = imu[last - k];
				// Driver: estimated_sample_time = received - (n - 1 - i) * period
				int64_t est = ev.host_ns - k * kImuPeriodNs;
				m_clock_offset_a2b((float)kImuHz, s.vts, est, &hw2mono_ema);
				m_clock_windowed_skew_tracker_push(skew, est, s.vts);
				m_ff_vec3_f32_push(ff_gyro, &s.gyro, s.vts);
				psvr2_pose_filter_push_gyro(&filt_a, s.vts, &s.gyro);
				psvr2_pose_filter_push_gyro(&filt_b, s.vts, &s.gyro);
				samples_seen++;
			}
		} else if (ev.type == 1) {
			const auto &sp = slam[ev.idx];
			xrt_space_relation rel{};
			rel.pose = sp.second;
			rel.relation_flags = (xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
			                                                XRT_SPACE_RELATION_POSITION_VALID_BIT |
			                                                XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
			                                                XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
			m_relation_history_push_with_motion_estimation(hist, &rel, sp.first);
			psvr2_pose_filter_push_slam(&filt_a, sp.first, &sp.second, nullptr);
			psvr2_pose_filter_push_slam(&filt_b, sp.first, &sp.second, nullptr);
		} else {
			if (samples_seen < 100)
				continue;
			int64_t at_host = ev.host_ns + (int64_t)(prediction_ms * 1e6);
			// Replay has no ground truth: compare against a constant pose so jitter is the output's own.
			xrt_pose truth = replay ? xrt_pose{{0, 0, 0, 1}, {0, 0, 0}} : truth_at((at_host - vts_to_host) * 1e-9);

			// Clock estimates
			timepoint_ns skew_vts = 0;
			bool have_skew = m_clock_windowed_skew_tracker_to_remote(skew, at_host, &skew_vts);
			clk_err_ema += pow((double)((at_host - hw2mono_ema) - (at_host - vts_to_host)), 2);
			if (have_skew)
				clk_err_skew += pow((double)(skew_vts - (at_host - vts_to_host)), 2);
			clk_n++;

			// OLD: hmd_get_raw_tracker_pose
			{
				int64_t at_hw = at_host - hw2mono_ema;
				xrt_space_relation latest{}, out{};
				int64_t latest_ts;
				if (m_relation_history_get_latest(hist, &latest_ts, &latest)) {
					if (at_hw <= latest_ts) {
						m_relation_history_get(hist, at_hw, &out);
					} else if (!t_apply_dead_reckoning(ff_gyro, NULL, NULL, at_hw, &latest, latest_ts,
					                                   &out)) {
						out = latest;
					}
					xrt_pose o = out.pose;
					m_old.ang_err.push_back(angle_between(o.orientation, truth.orientation));
					m_old.pos_err.push_back(m_vec3_len(m_vec3_sub(o.position, truth.position)));
					j_old.push(o, truth, m_old);
				}
			}
			// NEW filter, old clock
			{
				xrt_space_relation out;
				if (psvr2_pose_filter_get(&filt_a, at_host - hw2mono_ema, &out)) {
					m_new_ema.ang_err.push_back(angle_between(out.pose.orientation, truth.orientation));
					m_new_ema.pos_err.push_back(m_vec3_len(m_vec3_sub(out.pose.position, truth.position)));
					j_new_ema.push(out.pose, truth, m_new_ema);
				}
			}
			// NEW filter, new clock
			if (have_skew) {
				xrt_space_relation out;
				if (psvr2_pose_filter_get(&filt_b, skew_vts, &out)) {
					m_new.ang_err.push_back(angle_between(out.pose.orientation, truth.orientation));
					m_new.pos_err.push_back(m_vec3_len(m_vec3_sub(out.pose.position, truth.position)));
					j_new.push(out.pose, truth, m_new);
				}
			}
		}
	}

	printf("SLAM %.0f Hz, prediction %.0f ms, %.0f s, %zu queries\n", slam_hz, prediction_ms, seconds,
	       m_old.ang_err.size());
	if (!replay) {
		printf("  clock error rms: EMA %.3f ms, windowed %.3f ms\n", sqrt(clk_err_ema / clk_n) / 1e6,
		       sqrt(clk_err_skew / clk_n) / 1e6);
	}
	m_old.print("OLD (current driver)");
	m_new_ema.print("NEW filter, old clock");
	m_new.print("NEW filter + windowed clock");
	return 0;
}
