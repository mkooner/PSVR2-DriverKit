// Copyright 2020-2021, Collabora, Ltd.
// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  EuRoC playback functionality
 * @author Mateo de Mayo <mateo.demayo@collabora.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_euroc
 */

#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"
#include "xrt/xrt_data_source.h"

#include "os/os_threading.h"

#include "util/u_debug.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_var.h"
#include "util/u_sink.h"
#include "util/u_json.hpp"
#include "util/u_data_source.h"

#include "tracking/t_frame_cv_mat_wrapper.hpp"
#include "tracking/t_tracking.h"
#include "tracking/t_euroc_recorder.h"

#include "math/m_api.h"
#include "math/m_filter_fifo.h"

#include "euroc_driver.h"
#include "euroc_interface.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdint.h>
#include <stdio.h>
#include <fstream>
#include <future>
#include <thread>
#include <optional>


//! @see euroc_player_playback_config
DEBUG_GET_ONCE_LOG_OPTION(euroc_log, "EUROC_LOG", U_LOGGING_WARN)
DEBUG_GET_ONCE_OPTION(gt_device_name, "EUROC_GT_DEVICE_NAME", nullptr)
DEBUG_GET_ONCE_OPTION(cam_count, "EUROC_CAM_COUNT", nullptr)
DEBUG_GET_ONCE_OPTION(imu_count, "EUROC_IMU_COUNT", nullptr)
DEBUG_GET_ONCE_OPTION(color, "EUROC_COLOR", nullptr)
DEBUG_GET_ONCE_OPTION(gt, "EUROC_GT", nullptr)
DEBUG_GET_ONCE_OPTION(skip_first, "EUROC_SKIP_FIRST", "0%")
DEBUG_GET_ONCE_FLOAT_OPTION(scale, "EUROC_SCALE", 1.0f)
DEBUG_GET_ONCE_BOOL_OPTION(max_speed, "EUROC_MAX_SPEED", false)
DEBUG_GET_ONCE_FLOAT_OPTION(speed, "EUROC_SPEED", 1.0f)
DEBUG_GET_ONCE_BOOL_OPTION(paused, "EUROC_PAUSED", false)
DEBUG_GET_ONCE_BOOL_OPTION(send_all_imus_first, "EUROC_SEND_ALL_IMUS_FIRST", false)
DEBUG_GET_ONCE_BOOL_OPTION(use_source_ts, "EUROC_USE_SOURCE_TS", false)
DEBUG_GET_ONCE_BOOL_OPTION(play_from_start, "EUROC_PLAY_FROM_START", false)
DEBUG_GET_ONCE_BOOL_OPTION(print_progress, "EUROC_PRINT_PROGRESS", false)
DEBUG_GET_ONCE_FLOAT_OPTION(cam_delay_ms, "EUROC_CAM_DELAY_MS", 0.0f)
DEBUG_GET_ONCE_FLOAT_OPTION(imu_delay_ms, "EUROC_IMU_DELAY_MS", 0.0f)

#define EUROC_PLAYER_STR "Euroc Player"

using std::find_if;
using std::ifstream;
using std::is_same_v;
using std::launch;
using std::max_element;
using std::optional;
using std::pair;
using std::stof;
using std::string;
using std::to_string;
using std::vector;
using xrt::auxiliary::util::json::JSONNode;

struct img_sample
{
	timepoint_ns timestamp_ns;
	string path;
};

template <typename T> struct stream_state
{
	ssize_t latest_sent{-1};
	//! Samples are delivered this long after their timestamp; the timestamp itself is left untouched.
	time_duration_ns delay_ns{0};
	vector<T> samples{}; //!< List of all image names to read from the dataset per cameras

	optional<T *>
	peekNext(timepoint_ns playback_ns)
	{
		if (this->samples.size() == 0) {
			return std::nullopt;
		}

		if (this->latest_sent == (static_cast<ssize_t>(this->samples.size()) - 1)) {
			return std::nullopt;
		}

		auto &next = this->samples[this->latest_sent + 1];
		if (playback_ns >= next.timestamp_ns + this->delay_ns) {
			return &next;
		}
		return std::nullopt;
	}

	optional<T *>
	getNext(timepoint_ns playback_ns)
	{
		auto next = this->peekNext(playback_ns);
		if (next.has_value()) {
			this->latest_sent++;
		}
		return next;
	}

	optional<timepoint_ns>
	nextTimestamp()
	{
		if (this->samples.size() == 0) {
			return std::nullopt;
		}

		if (this->latest_sent == (static_cast<ssize_t>(this->samples.size()) - 1)) {
			return std::nullopt;
		}

		return this->samples[this->latest_sent + 1].timestamp_ns + this->delay_ns;
	}

	void
	skipUntil(timepoint_ns ts)
	{
		while (latest_sent + 1 < static_cast<ssize_t>(samples.size()) &&
		       samples[latest_sent + 1].timestamp_ns < ts) {
			latest_sent++;
		}
	}
};

using camera_stream = stream_state<img_sample>;
using imu_stream = stream_state<xrt_imu_sample>;
using gt_stream = stream_state<xrt_pose_sample>;

enum euroc_player_ui_state : std::uint8_t
{
	UNINITIALIZED = 0,
	NOT_STREAMING,
	STREAM_PLAYING,
	STREAM_PAUSED,
	STREAM_ENDED,
};

/*!
 * Euroc player is in charge of the playback of a particular dataset.
 *
 * @implements xrt_ds
 * @implements xrt_frame_node
 */
struct euroc_player
{
	xrt_ds base;
	xrt_frame_node node;

	// Sinks
	xrt_frame_sink cam_sinks[XRT_TRACKING_MAX_CAMS]; //!< Intermediate sink for each camera frames
	xrt_imu_sink imu_sinks[XRT_TRACKING_MAX_IMUS];   //!< Intermediate sink for IMU samples
	xrt_tracking_sinks in_sinks;                     //!< Pointers to intermediate sinks
	xrt_tracking_sinks out_sinks;                    //!< Pointers to downstream sinks

	u_logging_level log_level;             //!< Log messages with this priority and onwards
	euroc_player_dataset_info dataset;     //!< Contains information about the source dataset
	euroc_player_playback_config playback; //!< Playback information. Prefer to fill it before stream start
	xrt_ds_frame_mode mode;                //!< The only fs mode the euroc dataset provides
	bool is_running;                       //!< Set only at start, stop and end of frameserver stream
	os_thread_helper play_thread;

	timepoint_ns playback_ns;   //!< The playback timestamp of the stream.
	vector<imu_stream> imus;    //!< The list of IMUs in this dataset.
	vector<camera_stream> cams; //!< The list of cameras in this dataset.
	gt_stream gt;               //!< The state of the ground truth player.

	// Timestamp correction fields (can be disabled through `use_source_ts`)
	timepoint_ns base_ns; //!< First sample timestamp, stream timestamps are relative to this

	// UI related fields
	euroc_player_ui_state ui_state;
	u_var_button start_btn;
	u_var_button pause_btn;
	char progress_text[128];
	u_sink_debug ui_cam_sinks[XRT_TRACKING_MAX_CAMS]; //!< Sinks to display cam frames in UI
	m_ff_vec3_f32 *gyro_ffs[XRT_TRACKING_MAX_IMUS];   //!< Used for displaying IMU data
	m_ff_vec3_f32 *accel_ffs[XRT_TRACKING_MAX_IMUS];  //!< Same as `gyro_ff`

	static euroc_player *
	fromXds(xrt_ds *xds)
	{
		return reinterpret_cast<euroc_player *>(xds);
	}
};

static void
euroc_player_start_btn_cb(void *ptr);
static void
euroc_player_set_ui_state(euroc_player *ep, euroc_player_ui_state state);


// Euroc functionality

//! Parse and load all IMU samples into `samples`, assumes data.csv is well formed
//! If `read_n` > 0, read at most that amount of samples
//! Returns whether the appropriate data.csv file could be opened
static bool
euroc_player_preload_imu_data(const string &dataset_path, imu_stream &imu, uint32_t imu_id, int64_t read_n = -1)
{
	string imu_name = "imu" + to_string(imu_id);
	string csv_filename = dataset_path + "/mav0/" + imu_name + "/data.csv";
	ifstream fin{csv_filename};
	if (!fin.is_open()) {
		return false;
	}

	auto &samples = imu.samples;

	constexpr int COLUMN_COUNT = 6; // EuRoC imu columns: ts wx wy wz ax ay az
	string line;
	getline(fin, line); // Skip header line

	while (getline(fin, line) && read_n-- != 0) {
		timepoint_ns timestamp;
		double v[COLUMN_COUNT];
		size_t i = 0;
		size_t j = line.find(',');
		timestamp = stoll(line.substr(i, j));
		for (size_t k = 0; k < COLUMN_COUNT; k++) {
			i = j;
			j = line.find(',', i + 1);
			v[k] = stod(line.substr(i + 1, j));
		}

		xrt_imu_sample sample{timestamp, {v[3], v[4], v[5]}, {v[0], v[1], v[2]}};
		samples.push_back(sample);
	}
	return true;
}

string
slurp(ifstream &in)
{
	std::ostringstream sstr;
	sstr << in.rdbuf();
	return sstr.str();
}

static void
euroc_player_load_imu_led_model(const string &dataset_path, euroc_player_dataset_imu &imu, uint32_t imu_id)
{
	string imu_name = "led_model" + to_string(imu_id);
	string imu_path = dataset_path + "/mav0/" + imu_name;
	string csv_filename = imu_path + "/data.csv";
	string json_filename = imu_path + "/sensor.json";

	imu.has_led_model = false;

	ifstream fin{csv_filename};
	if (!fin.is_open()) {
		return;
	}

	ifstream fin_json{json_filename};
	if (!fin_json.is_open()) {
		return;
	}
	string json_contents = slurp(fin_json);

	try {
		JSONNode sensor{json_contents};
		if (sensor["sensor_type"].asString() != EUROC_SENSOR_TYPE_LED_MODEL) {
			U_LOG_E("Invalid sensor type %s, expected led_model_mnd",
			        sensor["sensor_type"].asString().c_str());
			return;
		}

		imu.led_model.match_parameters.min_leds_for_correspondence_search_without_prior =
		    sensor["min_leds_for_correspondence_search_without_prior"].asInt();
		imu.led_model.match_parameters.min_leds_for_correspondence_search_with_prior =
		    sensor["min_leds_for_correspondence_search_with_prior"].asInt();
	} catch (std::exception &e) {
		U_LOG_E("Got exception parsing led model sensor info: %s", e.what());
		return;
	}

	// #pos_x [m],pos_y [m],pos_z [m],norm_x [unit],norm_y [unit],norm_z [unit],radius [m],visibility_angle [rad],id
	constexpr int COLUMN_COUNT = 9; // EuRoC imu columns: ts wx wy wz ax ay az
	string line;
	getline(fin, line); // Skip header line

	while (getline(fin, line)) {
		double v[COLUMN_COUNT];
		size_t i = 0;
		size_t j = line.find(',');
		v[0] = stod(line.substr(i, j));
		for (size_t k = 1; k < COLUMN_COUNT; k++) {
			i = j;
			j = line.find(',', i + 1);
			v[k] = stod(line.substr(i + 1, j));
		}

		t_constellation_tracker_led led;
		led.position.x = v[0];
		led.position.y = v[1];
		led.position.z = v[2];
		led.normal.x = v[3];
		led.normal.y = v[4];
		led.normal.z = v[5];
		led.radius_m = v[6];
		led.visibility_angle = v[7];
		led.id = static_cast<t_constellation_led_id_it>(v[8]);
		imu.led_storage[imu.led_model.led_count++] = led;
	}
	imu.led_model.leds = imu.led_storage;
	imu.has_led_model = true;
}

/*!
 * Parse and load ground truth device name and trajectory into `gtdev` and
 * `trajectory` respectively
 *
 * @param[in] dataset_path
 * @param[in, out] gtdev The name of the groundtruth device found in the dataset if any or `nullptr`.
 * Groundtruth data can come from different devices, so we use the first of:
 * 1. The value prespecified in `gtdev`
 * 2. vicon0: found in euroc "vicon room" datasets
 * 3. mocap0: found in TUM-VI datasets with euroc format
 * 4. state_groundtruth_estimate0: found in euroc as a postprocessed ground truth (we only use first 7 columns)
 * 5. leica0: found in euroc "machine hall" datasets, only positional ground truth
 * @param[out] gt The read ground truth object
 * @param[in] read_n If > 0, read at most that amount of gt poses
 *
 * @returns Whether the appropriate data.csv file could be opened
 */
static bool
euroc_player_preload_gt_data(const string &dataset_path, const char *&gtdev, gt_stream &gt, int64_t read_n = -1)
{
	vector<const char *> gt_devices = {"vicon0", "mocap0", "state_groundtruth_estimate0", "leica0"};
	if (gtdev != nullptr && !string(gtdev).empty()) {
		gt_devices.insert(gt_devices.begin(), gtdev);
	}

	ifstream fin;
	for (const char *device : gt_devices) {
		string csv_filename = dataset_path + "/mav0/" + device + "/data.csv";
		fin = ifstream{csv_filename};
		if (fin.is_open()) {
			gtdev = device;
			break;
		}
	}

	if (!fin.is_open()) {
		return false;
	}

	constexpr int COLUMN_COUNT = 7; // EuRoC groundtruth columns: ts px py pz qw qx qy qz
	string line;
	getline(fin, line); // Skip header line

	auto &trajectory = gt.samples;

	while (getline(fin, line) && read_n-- != 0) {
		timepoint_ns timestamp;
		float v[COLUMN_COUNT] = {0, 0, 0, 1, 0, 0, 0}; // Set identity orientation for leica0
		size_t i = 0;
		size_t j = line.find(',');
		timestamp = stoll(line.substr(i, j));
		for (size_t k = 0; k < COLUMN_COUNT && j != string::npos; k++) {
			i = j;
			j = line.find(',', i + 1);
			v[k] = stof(line.substr(i + 1, j));
		}

		xrt_pose pose = {{v[4], v[5], v[6], v[3]}, {v[0], v[1], v[2]}};
		trajectory.push_back({timestamp, pose});
	}
	return true;
}

//! Parse and load image names and timestamps into `samples`
//! If read_n > 0, read at most that amount of samples
//! Returns whether the appropriate data.csv file could be opened
static bool
euroc_player_preload_img_data(const string &dataset_path, camera_stream &cam, uint32_t cam_id, int64_t read_n = -1)
{
	// Parse image data, assumes data.csv is well formed
	string cam_name = "cam" + to_string(cam_id);
	string imgs_path = dataset_path + "/mav0/" + cam_name + "/data";
	string csv_filename = dataset_path + "/mav0/" + cam_name + "/data.csv";
	ifstream fin{csv_filename};
	if (!fin.is_open()) {
		return false;
	}

	auto &samples = cam.samples;

	string line;
	getline(fin, line); // Skip header line
	while (getline(fin, line) && read_n-- != 0) {
		size_t i = line.find(',');
		timepoint_ns timestamp = stoll(line.substr(0, i));
		string img_name_tail = line.substr(i + 1);

		// Standard euroc datasets use CRLF line endings, so let's remove the extra '\r'
		if (img_name_tail.back() == '\r') {
			img_name_tail.pop_back();
		}

		string img_name = std::string(imgs_path).append("/").append(img_name_tail);
		img_sample sample{timestamp, img_name};
		samples.push_back(sample);
	}
	return true;
}

static void
euroc_player_load_cam_calib(const string &dataset_path, euroc_player_dataset_camera &cam, uint32_t cam_id)
{
	// Parse image data, assumes data.csv is well formed
	string cam_name = "cam" + to_string(cam_id);
	string json_filename = dataset_path + "/mav0/" + cam_name + "/sensor.json";
	ifstream fin{json_filename};
	if (!fin.is_open()) {
		cam.calib_valid = false;
		return;
	}

	string contents = slurp(fin);
	JSONNode node{contents};

	try {
		t_camera_calibration calib{};
		xrt_pose pose XRT_POSE_IDENTITY;

		if (node["sensor_type"].asString() != EUROC_SENSOR_TYPE_CAMERA) {
			throw std::runtime_error("Invalid sensor type " + node["sensor_type"].asString("INVALID"));
		}

		auto T_BS = node["T_BS"].asObject();
		if (T_BS["cols"].asInt() != 4 || T_BS["rows"].asInt() != 4 || !T_BS["data"].isArray()) {
			throw std::runtime_error("Invalid T_BS");
		}

		xrt_matrix_4x4 mat;
		auto T_BS_data = T_BS["data"].asArray();
		for (uint32_t i = 0; i < 16; i++) {
			mat.v[i] = T_BS_data[i].asDouble();
		}
		math_matrix_4x4_transpose(&mat, &mat); // row major (EuRoC) -> col major (Monado)
		math_pose_from_isometry(&mat, &pose);

		auto resolution = node["resolution"].asArray();
		calib.image_size_pixels.w = resolution[0].asInt(-1);
		calib.image_size_pixels.h = resolution[1].asInt(-1);
		if (calib.image_size_pixels.w == -1 || calib.image_size_pixels.h == -1) {
			throw std::runtime_error("Missing image size (pixels)");
		}

		calib.distortion_model = t_euroc_model_to_distortion_model(node["distortion_model"].asString().c_str());

		auto intrinsics = node["intrinsics"].asArray();
		t_camera_calibration_fill_intrinsics( //
		    &calib,                           //
		    intrinsics[0].asDouble(),         //
		    intrinsics[1].asDouble(),         //
		    intrinsics[2].asDouble(),         //
		    intrinsics[3].asDouble());        //

		auto distortion_coefficients = node["distortion_coefficients"].asArray();
		for (uint32_t i = 0;
		     i < distortion_coefficients.size() && i < ARRAY_SIZE(calib.distortion_parameters_as_array); i++) {
			calib.distortion_parameters_as_array[i] = distortion_coefficients[i].asDouble();
		}

		cam.calib = calib;
		cam.pose = pose;
		cam.calib_valid = true;
	} catch (std::exception &e) {
		U_LOG_E("Failed to load EuRoC camera calibration, got %s", e.what());
		cam.calib_valid = false;
	}
}

//! Trims cameras sequences so that they all start and end at the same sample
//! Note that this function does not guarantee that the dataset is free of framedrops
//! and it assumes it is properly formatted with monotonically increasing timestamps.
static void
euroc_player_match_cams_seqs(euroc_player *ep)
{
	// Find newest first timestamp and oldest last timestamp
	timepoint_ns first_ts = INT64_MIN;
	timepoint_ns last_ts = INT64_MAX;
	for (const camera_stream &cam : ep->cams) {
		auto &imgs = cam.samples;
		EUROC_ASSERT(!imgs.empty(), "Camera with no samples");

		timepoint_ns cam_first_ts = imgs.front().timestamp_ns;
		if (cam_first_ts > first_ts) {
			first_ts = cam_first_ts;
		}

		timepoint_ns cam_last_ts = imgs.back().timestamp_ns;
		if (cam_last_ts < last_ts) {
			last_ts = cam_last_ts;
		}
	}

	auto is_first = [first_ts](const img_sample &s) { return s.timestamp_ns == first_ts; };
	auto is_last = [last_ts](const img_sample &s) { return s.timestamp_ns == last_ts; };

	for (camera_stream &cam : ep->cams) {
		auto &imgs = cam.samples;
		auto new_first = find_if(imgs.begin(), imgs.end(), is_first);
		auto new_last = find_if(imgs.begin(), imgs.end(), is_last);
		EUROC_ASSERT_(new_first != imgs.end() && new_last != imgs.end());
		imgs.assign(new_first, new_last + 1);
	}
}

static void
euroc_player_preload(euroc_player *ep)
{
	for (uint32_t i = 0; i < ep->imus.size(); i++) {
		auto &imu = ep->imus[i];
		imu = imu_stream{};
		imu.delay_ns = ep->playback.imu_delay_ns;
		euroc_player_preload_imu_data(ep->dataset.path, imu, i);
	}

	for (size_t i = 0; i < ep->cams.size(); i++) {
		auto &cam = ep->cams[i];
		cam = camera_stream{};
		cam.delay_ns = ep->playback.cam_delay_ns;
		euroc_player_preload_img_data(ep->dataset.path, cam, i);
	}

	euroc_player_match_cams_seqs(ep);

	if (ep->dataset.has_gt) {
		ep->gt = gt_stream{};
		euroc_player_preload_gt_data(ep->dataset.path, ep->dataset.gt_device_name, ep->gt);
	}

	ep->base_ns = INT64_MAX;
	for (auto &cam : ep->cams) {
		ep->base_ns = MIN(ep->base_ns, cam.samples.front().timestamp_ns);
	}
	for (auto &imu : ep->imus) {
		ep->base_ns = MIN(ep->base_ns, imu.samples.front().timestamp_ns);
	}
}

//! Skips the first seconds of the dataset as specified by the user
static void
euroc_player_user_skip(euroc_player *ep)
{
	float skip_first_s = 0;
	if (ep->playback.skip_perc) {
		float skip_percentage = ep->playback.skip_first;
		timepoint_ns last_ts =
		    MAX(ep->cams[0].samples.back().timestamp_ns, ep->imus[0].samples.back().timestamp_ns);
		double dataset_length_s = (last_ts - ep->base_ns) / U_TIME_1S_IN_NS;
		skip_first_s = dataset_length_s * skip_percentage / 100.0f;
	} else {
		skip_first_s = ep->playback.skip_first;
	}

	time_duration_ns skip_first_ns = skip_first_s * U_TIME_1S_IN_NS;
	timepoint_ns skipped_ts = ep->base_ns + skip_first_ns;
	ep->playback_ns = skipped_ts;

	for (auto &imu : ep->imus) {
		imu.skipUntil(skipped_ts);
	}
	for (auto &cam : ep->cams) {
		cam.skipUntil(skipped_ts);
	}
	ep->gt.skipUntil(skipped_ts);
}

/*!
 * Determine and fill attributes of the dataset pointed by `path`
 * Assertion fails if `path` does not point to an euroc dataset
 */
static void
euroc_player_fill_dataset_info(const char *path, euroc_player_dataset_info *dataset)
{
	(void)snprintf(dataset->path, sizeof(dataset->path), "%s", path);
	camera_stream cam{};
	imu_stream imu{};
	gt_stream gt{};

	uint32_t i = 0;
	bool has_camera = euroc_player_preload_img_data(dataset->path, cam, i, 1);
	if (has_camera) {
		euroc_player_load_cam_calib(dataset->path, dataset->cams[i], i);
	}
	while (has_camera) {
		has_camera = euroc_player_preload_img_data(dataset->path, cam, ++i, 0);
		if (has_camera) {
			euroc_player_load_cam_calib(dataset->path, dataset->cams[i], i);
		}
	}
	uint32_t cam_count = i;
	EUROC_ASSERT(cam_count <= XRT_TRACKING_MAX_CAMS, "Increase XRT_TRACKING_MAX_CAMS (dataset with %u cams)",
	             cam_count);

	i = 0;
	bool has_imu = euroc_player_preload_imu_data(dataset->path, imu, i, 0);
	if (has_imu) {
		euroc_player_load_imu_led_model(dataset->path, dataset->imus[i], i);
	}
	while (has_imu) {
		has_imu = euroc_player_preload_imu_data(dataset->path, imu, ++i, 0);
		if (has_imu) {
			euroc_player_load_imu_led_model(dataset->path, dataset->imus[i], i);
		}
	}
	uint32_t imu_count = i;
	EUROC_ASSERT(imu_count <= XRT_TRACKING_MAX_IMUS, "Increase XRT_TRACKING_MAX_IMUS (dataset with %u imus)",
	             imu_count);

	bool has_gt = euroc_player_preload_gt_data(dataset->path, dataset->gt_device_name, gt, 0);
	bool is_valid_dataset = cam_count > 0 && imu_count > 0;
	EUROC_ASSERT(is_valid_dataset, "Invalid dataset %s", path);

	cv::Mat first_cam0_img = cv::imread(cam.samples[0].path, cv::IMREAD_ANYCOLOR);
	dataset->cam_count = cam_count;
	dataset->imu_count = imu_count;
	dataset->is_colored = first_cam0_img.channels() == 3;
	dataset->has_gt = has_gt;
	dataset->width = first_cam0_img.cols;
	dataset->height = first_cam0_img.rows;
}


// Playback functionality

static timepoint_ns
condition_sample_ts(euroc_player *ep, timepoint_ns ns)
{
	if (ep->playback.use_source_ts) {
		return ns;
	}

	// If we aren't using source timestamps, map them relative to the base timestamp;
	return ns - ep->base_ns;
}

static void
euroc_player_load_next_frame(euroc_player *ep, uint32_t cam_index, xrt_frame *&xf)
{
	auto &cam = ep->cams[cam_index];

	using xrt::auxiliary::tracking::FrameMat;
	img_sample sample = cam.samples[cam.latest_sent];
	ep->playback.scale = CLAMP(ep->playback.scale, 1.0 / 16, 4);

	// Load will be influenced by these playback options
	bool allow_color = ep->playback.color;
	float scale = ep->playback.scale;

	// Load image from disk
	timepoint_ns timestamp = condition_sample_ts(ep, sample.timestamp_ns);
	string img_name = sample.path;
	EUROC_TRACE(ep, "cam%d img t = %ld filename = %s", cam_index, timestamp, img_name.c_str());
	cv::ImreadModes read_mode = allow_color ? cv::IMREAD_ANYCOLOR : cv::IMREAD_GRAYSCALE;
	cv::Mat img = cv::imread(img_name, read_mode); // If colored, reads in BGR order

	if (scale != 1.0) {
		cv::Mat tmp;
		cv::resize(img, tmp, cv::Size(), scale, scale);
		img = tmp;
	}

	// Create xrt_frame, it will be freed by FrameMat destructor
	EUROC_ASSERT(xf == nullptr || xf->reference.count > 0, "Must be given a valid or nullptr frame ptr");
	EUROC_ASSERT(timestamp >= 0, "Unexpected negative timestamp");
	//! @todo Not using xrt_stereo_format because we use two sinks. It would
	//! probably be better to refactor everything to use stereo frames instead.
	FrameMat::Params params{XRT_STEREO_FORMAT_NONE, static_cast<uint64_t>(timestamp)};
	auto wrap = img.channels() == 3 ? FrameMat::wrapR8G8B8 : FrameMat::wrapL8;
	wrap(img, &xf, params);

	// Fields that aren't set by FrameMat
	xf->owner = ep;
	xf->source_timestamp = sample.timestamp_ns;
	xf->source_sequence = cam.latest_sent;
	xf->source_id = ep->base.source_id;
}

static void
euroc_player_push_pending_cams(euroc_player *ep)
{
	bool more_frames = true;
	while (more_frames) {
		more_frames = false;

		for (uint32_t i = 0; i < ep->cams.size(); i++) {
			auto &cam = ep->cams[i];

			while (auto next = cam.getNext(ep->playback_ns)) {
				xrt_frame *frame;
				euroc_player_load_next_frame(ep, i, frame);

				// Push the frame
				xrt_sink_push_frame(ep->in_sinks.cams[i], frame);

				// Drop our reference to the frame
				xrt_frame_reference(&frame, nullptr);

				if (i == 0) {
					size_t fcount = cam.samples.size();
					size_t img_seq = cam.latest_sent;
					(void)snprintf(ep->progress_text, sizeof(ep->progress_text),
					               "Playback %.2f%% - Frame %" PRId64 "/%" PRId64 " - IMU %" PRId64
					               "/%" PRId64,
					               float(img_seq) / float(fcount) * 100, img_seq, fcount,
					               ep->imus[0].latest_sent, ep->imus[0].samples.size());

					if (ep->playback.print_progress) {
						printf("%s\r", ep->progress_text);
						(void)fflush(stdout);
					}
				}
			}

			auto next = cam.peekNext(ep->playback_ns);
			if (!more_frames && next.has_value()) {
				more_frames = true;
			}
		}
	}
}

static void
euroc_player_push_pending_imus(euroc_player *ep)
{
	// Iterate all IMU samples
	for (uint32_t i = 0; i < ep->imus.size(); i++) {
		auto &imu = ep->imus[i];

		// Send all pending IMU data
		while (auto next = imu.getNext(ep->playback_ns)) {
			auto sample = *next.value();
			sample.timestamp_ns = condition_sample_ts(ep, sample.timestamp_ns);
			xrt_sink_push_imu(ep->in_sinks.imus[i], &sample);
		}
	}
}

static void
euroc_player_push_all_gt(euroc_player *ep)
{
	if (!ep->out_sinks.gt) {
		return;
	}

	for (xrt_pose_sample &sample : ep->gt.samples) {
		sample.timestamp_ns = condition_sample_ts(ep, sample.timestamp_ns);
		xrt_sink_push_pose(ep->out_sinks.gt, &sample);
	}
}

//! Gets the next sample timestamp, or INT64_MAX if none
timepoint_ns
euroc_player_get_next_sample_ts(euroc_player *ep)
{
	timepoint_ns next_sample_euroc_ts = INT64_MAX;
	for (auto &imu : ep->imus) {
		auto next = imu.nextTimestamp();
		if (next.has_value()) {
			next_sample_euroc_ts = MIN(next_sample_euroc_ts, next.value());
		}
	}
	for (auto &cam : ep->cams) {
		auto next = cam.nextTimestamp();
		if (next.has_value()) {
			next_sample_euroc_ts = MIN(next_sample_euroc_ts, next.value());
		}
	}
	return next_sample_euroc_ts;
}

void
euroc_player_sleep_until_next_sample(euroc_player *ep)
{
	using std::chrono::nanoseconds;
	using timepoint_ch = std::chrono::time_point<std::chrono::steady_clock>;
	using std::this_thread::sleep_until;

	timepoint_ns next_sample_euroc_ts = euroc_player_get_next_sample_ts(ep);
	if (next_sample_euroc_ts == INT64_MAX) {
		return;
	}

	timepoint_ns next_sample_mono_ts = (next_sample_euroc_ts - ep->playback_ns) / ep->playback.speed;
	timepoint_ch next_sample_chrono_tp{nanoseconds{next_sample_mono_ts}};
	sleep_until(next_sample_chrono_tp);
}

static void *
euroc_player_stream(void *ptr)
{
	xrt_ds *xds = (xrt_ds *)ptr;
	euroc_player *ep = euroc_player::fromXds(xds);
	EUROC_INFO(ep, "Starting euroc playback");

	euroc_player_preload(ep);
	euroc_player_user_skip(ep);

	// Push all IMU samples now if requested
	if (ep->playback.send_all_imus_first) {
		for (uint32_t i = 0; i < ep->imus.size(); i++) {
			auto &imu = ep->imus[i];
			while (auto src_sample = imu.getNext(INT64_MAX)) {
				xrt_imu_sample sample = *src_sample.value();
				sample.timestamp_ns = condition_sample_ts(ep, sample.timestamp_ns);
				xrt_sink_push_imu(ep->in_sinks.imus[i], &sample);
			}
		}
	}

	// Push ground truth trajectory now if available (and not disabled)
	if (ep->playback.gt) {
		euroc_player_push_all_gt(ep);
	}

	timepoint_ns last = os_monotonic_get_ns();
	while (os_thread_helper_is_running(&ep->play_thread)) {
		if (euroc_player_get_next_sample_ts(ep) == INT64_MAX) {
			break;
		}

		if (!ep->playback.max_speed) {
			euroc_player_sleep_until_next_sample(ep);
		}

		timepoint_ns now = os_monotonic_get_ns();
		time_duration_ns mono_elapsed = (now - last);
		time_duration_ns euroc_elapsed = mono_elapsed * ep->playback.speed;
		last = now;

		if (ep->playback.paused) {
			constexpr int64_t PAUSE_POLL_INTERVAL_NS = 15L * U_TIME_1MS_IN_NS;
			os_nanosleep(PAUSE_POLL_INTERVAL_NS);
			continue;
		}


		if (!ep->playback.max_speed) {
			ep->playback_ns += euroc_elapsed;
		} else {
			ep->playback_ns = euroc_player_get_next_sample_ts(ep);
		}

		euroc_player_push_pending_imus(ep);
		euroc_player_push_pending_cams(ep);
	}

	ep->is_running = false;

	EUROC_INFO(ep, "Euroc dataset playback finished");
	euroc_player_set_ui_state(ep, STREAM_ENDED);

	return nullptr;
}


// Data source functionality

static xrt_result_t
euroc_player_enumerate_frame_modes(xrt_ds *xds, xrt_ds_frame_mode **out_modes, uint32_t *out_count)
{
	euroc_player *ep = euroc_player::fromXds(xds);

	// Should be freed by caller
	xrt_ds_frame_mode *modes = U_TYPED_ARRAY_CALLOC(xrt_ds_frame_mode, 1);
	if (modes == nullptr) {
		EUROC_ERROR(ep, "Unable to calloc euroc playback modes");
		return XRT_ERROR_ALLOCATION;
	}

	// At first, it would sound like a good idea to list all possible playback
	// modes here, however it gets more troublesome than it is worth, and there
	// doesn't seem to be a good reason to use this feature here. Having said
	// that, a basic frame mode will be provided, which consists of only the original
	// properties of the dataset, and ignores the other playback options that can
	// be tweaked in the UI.
	modes[0] = ep->mode;

	*out_modes = modes;
	*out_count = 1;

	return XRT_SUCCESS;
}

static xrt_result_t
euroc_player_configure_capture(xrt_ds *xds, xrt_ds_frame_capture_parameters *cp)
{
	// Noop, this is a playback routine, there's no parameters to support.
	return XRT_SUCCESS;
}

#define DEFINE_RECEIVE_CAM(cam_id)                                                                                     \
	static void receive_cam##cam_id(xrt_frame_sink *sink, xrt_frame *xf)                                           \
	{                                                                                                              \
		euroc_player *ep = container_of(sink, euroc_player, cam_sinks[cam_id]);                                \
		EUROC_TRACE(ep, "cam%d img t=%ld source_t=%ld", cam_id, xf->timestamp, xf->source_timestamp);          \
		u_sink_debug_push_frame(&ep->ui_cam_sinks[cam_id], xf);                                                \
		if (ep->out_sinks.cams[cam_id]) {                                                                      \
			xrt_sink_push_frame(ep->out_sinks.cams[cam_id], xf);                                           \
		}                                                                                                      \
	}

XRT_TRACKING_FOR_EACH_CAM(DEFINE_RECEIVE_CAM)

#undef DEFINE_RECEIVE_CAM

#define DEFINE_RECEIVE_IMU(imu_id)                                                                                     \
	static void receive_imu##imu_id(xrt_imu_sink *sink, xrt_imu_sample *s)                                         \
	{                                                                                                              \
		euroc_player *ep = container_of(sink, euroc_player, imu_sinks[imu_id]);                                \
                                                                                                                       \
		timepoint_ns ts = s->timestamp_ns;                                                                     \
		xrt_vec3_f64 a = s->accel_m_s2;                                                                        \
		xrt_vec3_f64 w = s->gyro_rad_secs;                                                                     \
                                                                                                                       \
		const xrt_vec3 gyro{(float)w.x, (float)w.y, (float)w.z};                                               \
		const xrt_vec3 accel{(float)a.x, (float)a.y, (float)a.z};                                              \
		m_ff_vec3_f32_push(ep->gyro_ffs[imu_id], &gyro, ts);                                                   \
		m_ff_vec3_f32_push(ep->accel_ffs[imu_id], &accel, ts);                                                 \
                                                                                                                       \
		EUROC_TRACE(ep, "imu t=%ld ax=%f ay=%f az=%f wx=%f wy=%f wz=%f", ts, a.x, a.y, a.z, w.x, w.y, w.z);    \
		if (ep->out_sinks.imus[imu_id]) {                                                                      \
			xrt_sink_push_imu(ep->out_sinks.imus[imu_id], s);                                              \
		}                                                                                                      \
	}

XRT_TRACKING_FOR_EACH_IMU(DEFINE_RECEIVE_IMU)

#undef DEFINE_RECEIVE_IMU

/*!
 * This is the @ref xrt_ds stream start method, however as the euroc playback
 * is heavily customizable, it will be managed through the UI. So, unless
 * EUROC_PLAY_FROM_START is set, this will not start outputting frames until
 * the user clicks the start button.
 */
static xrt_result_t
euroc_player_frame_stream_start(xrt_ds *xds,
                                xrt_frame_sink *xs,
                                xrt_ds_capture_type capture_type,
                                uint32_t descriptor_index)
{
	euroc_player *ep = euroc_player::fromXds(xds);

	if (xs == nullptr && capture_type == XRT_DS_CAPTURE_TYPE_TRACKING) {
		EUROC_INFO(ep, "Starting Euroc Player in tracking mode");
		if (ep->out_sinks.cams[0] == nullptr) {
			EUROC_WARN(ep, "No cam0 sink provided, will keep running but tracking is unlikely to work");
		}
		if (ep->playback.play_from_start) {
			euroc_player_start_btn_cb(ep);
		}
	} else if (xs != nullptr && capture_type == XRT_DS_CAPTURE_TYPE_CALIBRATION) {
		EUROC_INFO(ep, "Starting Euroc Player in calibration mode, will stream only cam0 frames right away");
		ep->out_sinks.cams[0] = xs;
		euroc_player_start_btn_cb(ep);
	} else {
		EUROC_ERROR(ep, "Unsupported stream configuration xs=%p capture_type=%d", (void *)xs, capture_type);
		return XRT_ERROR_INVALID_ARGUMENT;
	}

	ep->is_running = true;

	return XRT_SUCCESS;
}

static xrt_result_t
euroc_player_tracking_streams_start(xrt_ds *xds, xrt_tracking_sinks *sinks)
{
	euroc_player *ep = euroc_player::fromXds(xds);
	ep->out_sinks = *sinks;
	return euroc_player_frame_stream_start(xds, nullptr, XRT_DS_CAPTURE_TYPE_TRACKING, 0);
}

static xrt_result_t
euroc_player_stream_stop(xrt_ds *xds)
{
	euroc_player *ep = euroc_player::fromXds(xds);
	ep->is_running = false;

	// Destroy also stops the thread.
	os_thread_helper_destroy(&ep->play_thread);

	return XRT_SUCCESS;
}

static xrt_result_t
euroc_player_is_running(xrt_ds *xds, bool *out_is_running)
{
	euroc_player *ep = euroc_player::fromXds(xds);
	(*out_is_running) = ep->is_running;
	return XRT_SUCCESS;
}


// Frame node functionality

static void
euroc_player_break_apart(xrt_frame_node *node)
{
	euroc_player *ep = container_of(node, euroc_player, node);
	euroc_player_stream_stop(&ep->base);
}

static void
euroc_player_destroy(xrt_frame_node *node)
{
	euroc_player *ep = container_of(node, euroc_player, node);

	u_var_remove_root(ep);
	for (uint32_t i = 0; i < ep->dataset.cam_count; i++) {
		u_sink_debug_destroy(&ep->ui_cam_sinks[i]);
	}
	for (uint32_t i = 0; i < ep->dataset.imu_count; i++) {
		m_ff_vec3_f32_free(&ep->gyro_ffs[i]);
		m_ff_vec3_f32_free(&ep->accel_ffs[i]);
	}

	delete ep;
}


// UI functionality

static void
euroc_player_set_ui_state(euroc_player *ep, euroc_player_ui_state state)
{
	// -> UNINITIALIZED -> NOT_STREAMING -> STREAM_PLAYING <-> STREAM_PAUSED
	//                                              └> STREAM_ENDED <┘
	euroc_player_ui_state prev_state = ep->ui_state;
	if (state == NOT_STREAMING) {
		EUROC_ASSERT_(prev_state == UNINITIALIZED);
		ep->pause_btn.disabled = true;
		snprintf(ep->progress_text, sizeof(ep->progress_text), "Stream has not started");
	} else if (state == STREAM_PLAYING) {
		EUROC_ASSERT_(prev_state == NOT_STREAMING || prev_state == STREAM_PAUSED);
		ep->start_btn.disabled = true;
		ep->pause_btn.disabled = false;
		snprintf(ep->pause_btn.label, sizeof(ep->pause_btn.label), "Pause");
	} else if (state == STREAM_PAUSED) {
		EUROC_ASSERT_(prev_state == STREAM_PLAYING);
		snprintf(ep->pause_btn.label, sizeof(ep->pause_btn.label), "Resume");
	} else if (state == STREAM_ENDED) {
		EUROC_ASSERT_(prev_state == STREAM_PLAYING || prev_state == STREAM_PAUSED);
		ep->pause_btn.disabled = true;
	} else {
		EUROC_ASSERT(false, "Unexpected UI state transition from %d to %d", prev_state, state);
	}
	ep->ui_state = state;
}

static void
euroc_player_start_btn_cb(void *ptr)
{
	euroc_player *ep = (euroc_player *)ptr;

	int ret = 0;
	ret |= os_thread_helper_init(&ep->play_thread);
	ret |= os_thread_helper_start(&ep->play_thread, euroc_player_stream, ep);
	EUROC_ASSERT(ret == 0, "Thread launch failure");

	euroc_player_set_ui_state(ep, STREAM_PLAYING);
}

static void
euroc_player_pause_btn_cb(void *ptr)
{
	//! @note: if you have groundtruth, pausing will unsync it from the tracker.

	euroc_player *ep = (euroc_player *)ptr;
	ep->playback.paused = !ep->playback.paused;

	euroc_player_set_ui_state(ep, ep->playback.paused ? STREAM_PAUSED : STREAM_PLAYING);
}

static void
euroc_player_setup_gui(euroc_player *ep)
{
	// Set sinks to display in UI
	for (uint32_t i = 0; i < ep->dataset.cam_count; i++) {
		u_sink_debug_init(&ep->ui_cam_sinks[i]);
	}
	for (uint32_t i = 0; i < ep->dataset.imu_count; i++) {
		m_ff_vec3_f32_alloc(&ep->gyro_ffs[i], 1000);
		m_ff_vec3_f32_alloc(&ep->accel_ffs[i], 1000);
	}

	// Set button callbacks
	ep->start_btn.cb = euroc_player_start_btn_cb;
	ep->start_btn.ptr = ep;
	ep->pause_btn.cb = euroc_player_pause_btn_cb;
	ep->pause_btn.ptr = ep;
	euroc_player_set_ui_state(ep, NOT_STREAMING);

	// Add UI wigets
	u_var_add_root(ep, "Euroc Player", false);
	u_var_add_ro_text(ep, ep->dataset.path, "Dataset");
	u_var_add_ro_text(ep, ep->progress_text, "Progress");
	u_var_add_button(ep, &ep->start_btn, "Start");
	u_var_add_button(ep, &ep->pause_btn, "Pause");
	u_var_add_log_level(ep, &ep->log_level, "Log level");

	u_var_add_gui_header(ep, nullptr, "Playback Options");
	u_var_add_ro_text(ep, "Set these before starting the stream", "Note");
	u_var_add_u32(ep, &ep->playback.cam_count, "Use N cams (if available)");
	u_var_add_u32(ep, &ep->playback.imu_count, "Use N IMUs (if available)");
	u_var_add_bool(ep, &ep->playback.color, "Color (if available)");
	u_var_add_bool(ep, &ep->playback.gt, "Groundtruth (if available)");
	u_var_add_bool(ep, &ep->playback.skip_perc, "Skip percentage, otherwise skips seconds");
	u_var_add_f32(ep, &ep->playback.skip_first, "How much to skip");
	u_var_add_f32(ep, &ep->playback.scale, "Scale");
	u_var_add_bool(ep, &ep->playback.max_speed, "Max speed");
	u_var_add_f64(ep, &ep->playback.speed, "Speed");
	u_var_add_bool(ep, &ep->playback.send_all_imus_first, "Send all IMU samples first");
	u_var_add_bool(ep, &ep->playback.use_source_ts, "Use original timestamps");

	u_var_add_gui_header(ep, nullptr, "Streams");
	for (uint32_t i = 0; i < ep->dataset.imu_count; i++) {
		char label[] = "IMU(X) NNNNNNNNNN";
		(void)snprintf(label, sizeof(label), "IMU(G) %d", i);
		u_var_add_ro_ff_vec3_f32(ep, ep->gyro_ffs[i], label);
		(void)snprintf(label, sizeof(label), "IMU(A) %d", i);
		u_var_add_ro_ff_vec3_f32(ep, ep->accel_ffs[i], label);
	}
	for (uint32_t i = 0; i < ep->dataset.cam_count; i++) {
		char label[] = "Camera NNNNNNNNNN";
		(void)snprintf(label, sizeof(label), "Camera %d", i);
		u_var_add_sink_debug(ep, &ep->ui_cam_sinks[i], label);
	}
}

extern "C" void
euroc_player_fill_default_config_for(euroc_player_config *config, const char *dataset_path)
{
	euroc_player_dataset_info dataset = {};
	dataset.gt_device_name = debug_get_option_gt_device_name();
	euroc_player_fill_dataset_info(dataset_path, &dataset);

	euroc_player_playback_config playback = {};
	const char *cam_count = debug_get_option_cam_count();
	const char *imu_count = debug_get_option_imu_count();
	const char *color = debug_get_option_color();
	const char *gt = debug_get_option_gt();
	const char *skip_option = debug_get_option_skip_first();
	playback.cam_count = (int)debug_string_to_num(cam_count, dataset.cam_count);
	playback.imu_count = (int)debug_string_to_num(imu_count, dataset.imu_count);
	playback.color = color == nullptr ? dataset.is_colored : debug_string_to_bool(color);
	playback.gt = gt == nullptr ? dataset.has_gt : debug_string_to_bool(gt);
	playback.skip_perc = string(skip_option).back() == '%';
	playback.skip_first = stof(skip_option);
	playback.scale = debug_get_float_option_scale();
	playback.max_speed = debug_get_bool_option_max_speed();
	playback.speed = debug_get_float_option_speed();
	playback.paused = debug_get_bool_option_paused();
	playback.send_all_imus_first = debug_get_bool_option_send_all_imus_first();
	playback.use_source_ts = debug_get_bool_option_use_source_ts();
	playback.play_from_start = debug_get_bool_option_play_from_start();
	playback.print_progress = debug_get_bool_option_print_progress();
	playback.cam_delay_ns = (time_duration_ns)(debug_get_float_option_cam_delay_ms() * U_TIME_1MS_IN_NS);
	playback.imu_delay_ns = (time_duration_ns)(debug_get_float_option_imu_delay_ms() * U_TIME_1MS_IN_NS);

	config->log_level = debug_get_log_option_euroc_log();
	config->dataset = dataset;
	config->playback = playback;
}

// Euroc driver creation

extern "C" xrt_ds *
euroc_player_create(xrt_frame_context *xfctx, const char *path, euroc_player_config *config)
{
	// Value-initialize so all the C members are zeroed, can't use braces as that would define a new struct.
	euroc_player *ep = new euroc_player();

	euroc_player_config *default_config = nullptr;
	if (config == nullptr) {
		default_config = U_TYPED_CALLOC(euroc_player_config);
		euroc_player_fill_default_config_for(default_config, path);
		config = default_config;
	}

	ep->log_level = config->log_level;
	ep->dataset = config->dataset;
	ep->playback = config->playback;

	// Update the pointers now that we've copied it.
	for (auto &imu : ep->dataset.imus) {
		imu.led_model.leds = imu.led_storage;
	}

	if (default_config != nullptr) {
		free(default_config);
	}

	ep->mode = xrt_ds_frame_mode{
	    ep->dataset.width,
	    ep->dataset.height,
	    ep->dataset.is_colored ? XRT_FORMAT_R8G8B8 : XRT_FORMAT_R8,
	    // Stereo euroc *is* supported, but we don't expose that through the
	    // xrt_ds interface as it will be managed through two different sinks.
	    XRT_STEREO_FORMAT_NONE,
	};
	EUROC_INFO(ep, "dataset information\n\tpath: %s\n\tcam_count: %d, is_colored: %d, width: %d, height: %d",
	           ep->dataset.path, ep->dataset.cam_count, ep->dataset.is_colored, ep->dataset.width,
	           ep->dataset.height);

	// Using pointers to not mix vector with a C-compatible struct
	ep->gt = {};
	ep->imus = vector<imu_stream>(ep->dataset.imu_count);
	ep->cams = vector<camera_stream>(ep->dataset.cam_count);

	euroc_player_setup_gui(ep);

	ep->in_sinks.cam_count = ep->dataset.cam_count;

#define DEFINE_RECEIVE_CAM(cam_id)                                                                                     \
	ep->cam_sinks[cam_id].push_frame = receive_cam##cam_id;                                                        \
	ep->in_sinks.cams[cam_id] = &ep->cam_sinks[cam_id];
	XRT_TRACKING_FOR_EACH_CAM(DEFINE_RECEIVE_CAM)
#undef DEFINE_RECEIVE_CAM

	ep->in_sinks.imu_count = ep->dataset.imu_count;

#define DEFINE_RECEIVE_IMU(imu_id)                                                                                     \
	ep->imu_sinks[imu_id].push_imu = receive_imu##imu_id;                                                          \
	ep->in_sinks.imus[imu_id] = &ep->imu_sinks[imu_id];
	XRT_TRACKING_FOR_EACH_IMU(DEFINE_RECEIVE_IMU)
#undef DEFINE_RECEIVE_IMU

	xrt_ds *xds = &ep->base;
	u_data_source_populate_function_pointers(xds);
	xds->enumerate_frame_modes = euroc_player_enumerate_frame_modes;
	xds->configure_capture = euroc_player_configure_capture;
	xds->frame_stream_start = euroc_player_frame_stream_start;
	xds->tracking_streams_start = euroc_player_tracking_streams_start;
	xds->stream_stop = euroc_player_stream_stop;
	xds->is_running = euroc_player_is_running;

	(void)snprintf(xds->name, sizeof(xds->name), EUROC_PLAYER_STR);
	(void)snprintf(xds->product, sizeof(xds->product), EUROC_PLAYER_STR " Product");
	(void)snprintf(xds->manufacturer, sizeof(xds->manufacturer), EUROC_PLAYER_STR " Manufacturer");
	(void)snprintf(xds->serial, sizeof(xds->serial), EUROC_PLAYER_STR " Serial");
	xds->source_id = 0xECD0FEED;

	xrt_frame_node *xfn = &ep->node;
	xfn->break_apart = euroc_player_break_apart;
	xfn->destroy = euroc_player_destroy;

	xrt_frame_context_add(xfctx, &ep->node);

	EUROC_DEBUG(ep, "Euroc player created");

	return xds;
}

void
euroc_player_get_dataset_info(xrt_ds *xds, const euroc_player_dataset_info **out_dataset_info)
{
	euroc_player *ep = euroc_player::fromXds(xds);

	(*out_dataset_info) = &ep->dataset;
}
