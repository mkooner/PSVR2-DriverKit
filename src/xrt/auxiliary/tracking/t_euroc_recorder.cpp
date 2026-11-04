// Copyright 2021, Collabora, Ltd.
// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  EuRoC dataset recorder utility.
 * @author Mateo de Mayo <mateo.demayo@collabora.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup aux_tracking
 */

#include "xrt/xrt_defines.h"
#include "xrt/xrt_tracking.h"
#include "xrt/xrt_config_have.h"

#include "math/m_api.h"

#include "os/os_time.h"

#include "util/u_frame.h"
#include "util/u_sink.h"
#include "util/u_var.h"
#include "util/u_debug.h"
#include "util/u_json.hpp"

#include "t_euroc_recorder.h"

#include <cassert>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <queue>
#include <span>
#include <string>

#ifdef XRT_HAVE_OPENCV
#include <opencv2/imgcodecs.hpp>
#endif


DEBUG_GET_ONCE_BOOL_OPTION(euroc_recorder_use_jpg, "EUROC_RECORDER_USE_JPG", false)

using std::lock_guard;
using std::mutex;
using std::ofstream;
using std::queue;
using std::string;
using std::to_string;
using std::vector;
using std::filesystem::create_directories;
using xrt::auxiliary::util::json::JSONBuilder;

union euroc_sensor_info {
	struct
	{
		t_camera_calibration calib;
	} camera;
	struct
	{
		t_constellation_tracker_led_model *led_model;
	} constellation;
	struct
	{
		//! May be null, in which case generic noise values are written.
		const xrt_imu_calibration *calibration;
	} imu;
};

static void
writeImuSensorCalibration(JSONBuilder &jb, const string &name, const xrt_imu_sensor_calibration &calib)
{
	// The noise density and random walk are standard EuRoC keys, the rest extend the schema in the same style.
	jb << name + "_noise_density" << calib.noise_density;                           // [ unit / sqrt(Hz) ]
	jb << name + "_random_walk" << calib.bias_random_walk;                          // [ unit / s / sqrt(Hz) ]
	jb << name + "_scale_factor_noise_density" << calib.scale_factor_noise_density; // [ 1 / sqrt(Hz) ]
	jb << name + "_cross_axis_noise_density" << calib.cross_axis_noise_density;     // [ 1 / sqrt(Hz) ]
	jb << name + "_axis_saturation" << calib.axis_saturation;                       // [ unit ]
	jb << name + "_saturated_noise_density" << calib.saturated_noise_density;       // [ unit / sqrt(Hz) ]
	jb << name + "_bias_initial_sigma" << calib.bias_initial_sigma;                 // [ unit ]
}

static void
writeSensorInfo(const string &data_path,
                const string &sensor_type,
                const string &sensor_comment,
                const xrt_pose &T_body_sensor,
                const union euroc_sensor_info &sensor_info)
{
	xrt_matrix_4x4 t_bs;
	math_matrix_4x4_isometry_from_pose(&T_body_sensor, &t_bs);
	math_matrix_4x4_transpose(&t_bs, &t_bs);

	JSONBuilder jb{};
	jb << "{";
	jb << "sensor_type" << sensor_type;
	jb << "comment" << sensor_comment;

	jb << "T_BS"
	   << "{";
	jb << "cols" << 4;
	jb << "rows" << 4;
	jb << "data"
	   << "[";
	for (uint32_t i = 0; i < ARRAY_SIZE(t_bs.v); i++) {
		jb << (double)t_bs.v[i];
	}
	jb << "]";
	jb << "}";

	if (sensor_type == EUROC_SENSOR_TYPE_IMU) {
		jb << "rate_hz" << 200;                           // reasonable guess until we have this data here
		const xrt_imu_calibration *calib = sensor_info.imu.calibration;
		if (calib != nullptr) {
			writeImuSensorCalibration(jb, "gyroscope", calib->gyro);
			writeImuSensorCalibration(jb, "accelerometer", calib->accel);
			jb << "accelerometer_scale_random_walk" << calib->accel_scale_random_walk; // [ 1 / sqrt(s) ]
			jb << "accelerometer_scale_initial_sigma" << calib->accel_scale_initial_sigma; // [ unitless ]
		} else {
			jb << "gyroscope_noise_density" << 1.6968e-04;    // [ rad / s / sqrt(Hz) ]
			jb << "gyroscope_random_walk" << 1.9393e-05;      // [ rad / s^2 / sqrt(Hz) ]
			jb << "accelerometer_noise_density" << 2.0000e-3; // [ m / s^2 / sqrt(Hz) ]
			jb << "accelerometer_random_walk" << 3.0000e-3;   // [ m / s^3 / sqrt(Hz) ]
		}
	} else if (sensor_type == EUROC_SENSOR_TYPE_CAMERA) {
		const auto &calib = sensor_info.camera.calib;

		jb << "rate_hz" << 20; // @todo
		jb << "resolution"
		   << "[" << (int)calib.image_size_pixels.w << (int)calib.image_size_pixels.h << "]";
		jb << "camera_model"
		   << "pinhole"; // technically wrong for fisheye models but this is convention
		// fx, fy, cx, cy
		jb << "intrinsics"
		   << "[" << calib.intrinsics[0][0] << calib.intrinsics[1][1] << calib.intrinsics[0][2]
		   << calib.intrinsics[1][2] << "]";
		switch (calib.distortion_model) {
		case T_DISTORTION_PINHOLE:
			jb << "distortion_model" << EUROC_DISTORTION_MODEL_RADTAN_4;
			jb << "distortion_coefficients"
			   << "[" << 0.0 << 0.0 << 0.0 << 0.0 << "]";
			break;
		default:
			jb << "distortion_model" << string{t_distortion_model_to_euroc(calib.distortion_model)};
			jb << "distortion_coefficients"
			   << "[";
			for (size_t i = 0; i < t_num_params_from_distortion_model(calib.distortion_model); i++) {
				jb << calib.distortion_parameters_as_array[i];
			}
			jb << "]";
			break;
		}
	} else if (sensor_type == EUROC_SENSOR_TYPE_LED_MODEL) {
		const auto &match = sensor_info.constellation.led_model->match_parameters;

		jb << "min_leds_for_correspondence_search_without_prior"
		   << (int)match.min_leds_for_correspondence_search_without_prior;
		jb << "min_leds_for_correspondence_search_with_prior"
		   << (int)match.min_leds_for_correspondence_search_with_prior;
	}
	jb << "}";

	ofstream out_stream{data_path + "/sensor.json"};
	out_stream << jb.getBuiltNode()->toString(false) << CSV_EOL;
}

static void
writeLedModel(const string &data_path, t_constellation_tracker_led_model *led_model)
{
	ofstream out_stream{data_path + "/data.csv"};
	out_stream << std::setprecision(std::numeric_limits<float>::max_digits10); // pos/norm

	out_stream << "#pos_x [m],pos_y [m],pos_z [m],norm_x [unit],norm_y [unit],norm_z [unit],radius "
	              "[m],visibility_angle [rad],id" CSV_EOL;

	for (uint32_t i = 0; i < led_model->led_count; i++) {
		const auto &led = led_model->leds[i];

		out_stream << led.position.x << ", ";
		out_stream << led.position.y << ", ";
		out_stream << led.position.z << ", ";
		out_stream << led.normal.x << ", ";
		out_stream << led.normal.y << ", ";
		out_stream << led.normal.z << ", ";
		out_stream << led.radius_m << ", ";
		out_stream << led.visibility_angle << ", ";
		out_stream << static_cast<unsigned int>(led.id) << CSV_EOL;
	}
}

struct euroc_recorder
{
	xrt_frame_node node;
	string path_prefix; //!< Path for the dataset without datetime suffix
	string path;        //!< Full path of the current dataset being recorded or empty string if none
	euroc_recorder_settings settings;

	bool recording;             //!< Whether samples are being recorded
	u_var_button recording_btn; //!< UI button to start/stop `recording`

	bool use_jpg; //! Whether or not we should save images as .jpg files

	// Cloner sinks: copy frame to heap for quick release of the original
	xrt_tracking_sinks cloner_queues; //!< Queue sinks that write into cloner sinks
	xrt_imu_sink cloner_imu_sinks[XRT_TRACKING_MAX_IMUS];
	xrt_pose_sink cloner_gt_sink;
	xrt_frame_sink cloner_sinks[XRT_TRACKING_MAX_CAMS];

	// Writer sinks: write copied frame to disk
	xrt_tracking_sinks writer_queues; //!< Queue sinks that write into writer sinks
	xrt_imu_sink writer_imu_sinks[XRT_TRACKING_MAX_IMUS];
	xrt_pose_sink writer_gt_sink;
	xrt_frame_sink writer_sinks[XRT_TRACKING_MAX_CAMS];

	vector<queue<xrt_imu_sample>> imu_queues{}; //!< IMU pushes are delayed here until cam0 push
	vector<mutex> imu_queue_locks{}; //!< Lock for each imu_queues[i], vector doesn't need lock, it's only read

	queue<xrt_pose_sample> gt_queue{}; //!< GT pushes get saved here and are delayed until left_frame pushes
	mutex gt_queue_lock{};             //!< Lock for gt_queue

	// CSV file handles, ofstream implementation is already buffered.
	// Using pointers because of `container_of`
	ofstream *imus_csv[XRT_TRACKING_MAX_IMUS] = {};
	ofstream *gt_csv = nullptr;
	ofstream *cams_csv[XRT_TRACKING_MAX_CAMS] = {};

	void
	makeFiles()
	{
		string path = this->path;

		for (uint32_t i = 0; i < this->settings.imu_count; i++) {
			const string data_path = path + "/mav0/imu" + to_string(i);
			create_directories(data_path);
			//! @todo Implement IMU extrinsics & IMU intrinsics
			writeSensorInfo(           //
			    data_path,             //
			    EUROC_SENSOR_TYPE_IMU, //
			    "IMU " + to_string(i), //
			    XRT_POSE_IDENTITY,     //
			    {
			        .imu = {.calibration = this->settings.constellation_recording
				                           ? this->settings.constellation.imu_calibrations[i]
				                           : nullptr},
			    });
			this->imus_csv[i] = new ofstream{data_path + "/data.csv"};
			*this->imus_csv[i] << std::fixed << std::setprecision(CSV_PRECISION);
			*this->imus_csv[i]
			    << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
			       "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]" CSV_EOL;
		}

		for (uint32_t i = 0; i < this->settings.imu_count && this->settings.constellation_recording; i++) {
			const string data_path = path + "/mav0/led_model" + to_string(i);
			create_directories(data_path);
			writeSensorInfo(                 //
			    data_path,                   //
			    EUROC_SENSOR_TYPE_LED_MODEL, //
			    "LED Model " + to_string(i), //
			    XRT_POSE_IDENTITY,           //
			    {
			        .constellation =
			            {
			                .led_model = this->settings.constellation.led_models[i],
			            },
			    });
			writeLedModel(data_path, this->settings.constellation.led_models[i]);
		}

		if (this->settings.ground_truth_stream_count > 0) {
			const string data_path = path + "/mav0/gt";
			create_directories(data_path);
			this->gt_csv = new ofstream{path + "/mav0/gt/data.csv"};
			*this->gt_csv << std::fixed << std::setprecision(CSV_PRECISION);
			*this->gt_csv << "#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],"
			                 "q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z []" CSV_EOL;
		}

		for (uint32_t i = 0; i < this->settings.cam_count; i++) {
			const string cam_path = path + "/mav0/cam" + to_string(i);
			const string data_path = cam_path + "/data";
			create_directories(data_path);
			if (this->settings.camera_model_valid) {
				writeSensorInfo(              //
				    cam_path,                 //
				    EUROC_SENSOR_TYPE_CAMERA, //
				    "Camera " + to_string(i), //
				    this->settings.camera_model.poses[i],
				    {
				        .camera =
				            {
				                .calib = this->settings.camera_model.intrinsics[i],
				            },
				    });
			}
			this->cams_csv[i] = new ofstream{data_path + ".csv"};
			*this->cams_csv[i] << "#timestamp [ns],filename" CSV_EOL;
		}
	}

	void
	flush()
	{
		for (uint32_t i = 0; i < this->settings.imu_count; i++) {
			// Flush IMU samples
			vector<xrt_imu_sample> imu_samples;

			{ // Move samples out of imu_queue into vector to minimize mutex contention
				lock_guard lock{this->imu_queue_locks[i]};
				queue<xrt_imu_sample> &imu_queue = this->imu_queues[i];
				imu_samples.reserve(imu_queue.size());
				while (!imu_queue.empty()) {
					imu_samples.push_back(imu_queue.front());
					imu_queue.pop();
				}
			}

			// Write queued IMU samples to csv stream.
			for (xrt_imu_sample &sample : imu_samples) {
				xrt_sink_push_imu(&this->writer_imu_sinks[i], &sample);
			}
		}

		// Flush groundtruth samples
		vector<xrt_pose_sample> gt_samples;

		{ // Move samples out of gt_queue into vector to minimize mutex contention
			lock_guard lock{this->gt_queue_lock};
			gt_samples.reserve(this->gt_queue.size());
			while (!this->gt_queue.empty()) {
				gt_samples.push_back(this->gt_queue.front());
				this->gt_queue.pop();
			}
		}

		// Write queued gt samples to csv stream.
		for (xrt_pose_sample &sample : gt_samples) {
			xrt_sink_push_pose(&this->writer_gt_sink, &sample);
		}

		// Flush csv streams. Not necessary, doing it only to increase flush frequency
		for (uint32_t i = 0; i < this->settings.imu_count; i++) {
			this->imus_csv[i]->flush();
		}
		//! @todo multiple gt streams
		if (this->settings.ground_truth_stream_count > 0) {
			this->gt_csv->flush();
		}
		for (uint32_t i = 0; i < this->settings.cam_count; i++) {
			this->cams_csv[i]->flush();
		}
	}

	void
	saveImuSample(const xrt_imu_sample &sample, uint32_t imu_index)
	{
		timepoint_ns ts = sample.timestamp_ns;
		const xrt_vec3_f64 &a = sample.accel_m_s2;
		const xrt_vec3_f64 &w = sample.gyro_rad_secs;

		*this->imus_csv[imu_index] << ts << ",";
		*this->imus_csv[imu_index] << w.x << "," << w.y << "," << w.z << ",";
		*this->imus_csv[imu_index] << a.x << "," << a.y << "," << a.z << CSV_EOL;
	}

	void
	saveGroundTruth(const xrt_pose_sample &sample)
	{
		timepoint_ns ts = sample.timestamp_ns;
		const xrt_vec3 &p = sample.pose.position;
		const xrt_quat &o = sample.pose.orientation;

		*this->gt_csv << ts << ",";
		*this->gt_csv << p.x << "," << p.y << "," << p.z << ",";
		*this->gt_csv << o.w << "," << o.x << "," << o.y << "," << o.z << CSV_EOL;
	}

	void
	saveFrame(xrt_frame *frame, uint32_t cam_index)
	{
		string cam_name = "cam" + to_string(cam_index);
		uint64_t ts = frame->timestamp;

		assert(frame->format == XRT_FORMAT_L8 || frame->format == XRT_FORMAT_R8G8B8); // Only formats supported
		string file_extension = this->use_jpg ? ".jpg" : ".png";
		string filename = std::to_string(ts) + file_extension;
		string img_path = this->path + "/mav0/" + cam_name + "/data/" + filename;

#ifdef XRT_HAVE_OPENCV
		auto img_type = frame->format == XRT_FORMAT_L8 ? CV_8UC1 : CV_8UC3;
		cv::Mat img{(int)frame->height, (int)frame->width, img_type, frame->data, frame->stride};
		cv::imwrite(img_path, img);
#else
		U_LOG_W("OpenCV not built, unable to save %s", img_path.c_str());
#endif

		*this->cams_csv[cam_index] << ts << "," << filename << CSV_EOL;
	}
};


/*
 *
 * Writer sinks functionality
 *
 */

#define DEFINE_SAVE_IMU(imu_id)                                                                                        \
	extern "C" void euroc_recorder_save_imu##imu_id(xrt_imu_sink *sink, xrt_imu_sample *sample)                    \
	{                                                                                                              \
		euroc_recorder *er = container_of(sink, euroc_recorder, writer_imu_sinks[imu_id]);                     \
		er->saveImuSample(*sample, imu_id);                                                                    \
	}

XRT_TRACKING_FOR_EACH_IMU(DEFINE_SAVE_IMU)

#define USE_SAVE_IMU(imu_id) euroc_recorder_save_imu##imu_id,

static void (*euroc_recorder_save_imu[XRT_TRACKING_MAX_IMUS])(xrt_imu_sink *sink, xrt_imu_sample *sample) = {
    XRT_TRACKING_FOR_EACH_IMU(USE_SAVE_IMU)};

extern "C" void
euroc_recorder_save_gt(xrt_pose_sink *sink, xrt_pose_sample *sample)
{
	euroc_recorder *er = container_of(sink, euroc_recorder, writer_gt_sink);

	er->saveGroundTruth(*sample);
}

#define DEFINE_SAVE_CAM(cam_id)                                                                                        \
	extern "C" void euroc_recorder_save_cam##cam_id(xrt_frame_sink *sink, xrt_frame *frame)                        \
	{                                                                                                              \
		euroc_recorder *er = container_of(sink, euroc_recorder, writer_sinks[cam_id]);                         \
		if (cam_id == 0) {                                                                                     \
			er->flush();                                                                                   \
		}                                                                                                      \
		er->saveFrame(frame, cam_id);                                                                          \
	}

XRT_TRACKING_FOR_EACH_CAM(DEFINE_SAVE_CAM)

#define USE_SAVE_CAM(cam_id) euroc_recorder_save_cam##cam_id,

static void (*euroc_recorder_save_cam[XRT_TRACKING_MAX_CAMS])(xrt_frame_sink *sink, xrt_frame *frame) = {
    XRT_TRACKING_FOR_EACH_CAM(USE_SAVE_CAM)};


/*
 *
 * Cloner sinks functionality
 *
 */

static void
euroc_recorder_receive_imu_sample(euroc_recorder *er, xrt_imu_sample *sample, uint32_t i)
{
	// Contrary to frame sinks, we don't have separately threaded queues for IMU
	// sinks so we use an std::queue to temporarily store IMU samples, later we
	// write them to disk when writing left frames.

	assert(i < er->settings.imu_count);

	if (!er->recording) {
		return;
	}

	{
		lock_guard lock{er->imu_queue_locks[i]};
		er->imu_queues[i].push(*sample);
	}
}

#define DEFINE_RECEIVE_IMU(imu_id)                                                                                     \
	extern "C" void euroc_recorder_receive_imu##imu_id(xrt_imu_sink *sink, xrt_imu_sample *sample)                 \
	{                                                                                                              \
		euroc_recorder *er = container_of(sink, euroc_recorder, cloner_imu_sinks[imu_id]);                     \
		euroc_recorder_receive_imu_sample(er, sample, imu_id);                                                 \
	}

DEFINE_RECEIVE_IMU(0)
DEFINE_RECEIVE_IMU(1)
DEFINE_RECEIVE_IMU(2)

//! Be sure to define the same number of defined functions as
//! XRT_TRACKING_MAX_IMUS and to add them to to euroc_recorder_receive_imu
static void (*euroc_recorder_receive_imu[XRT_TRACKING_MAX_IMUS])(xrt_imu_sink *sink,
                                                                 xrt_imu_sample *sample) = {
    euroc_recorder_receive_imu0, //
    euroc_recorder_receive_imu1, //
    euroc_recorder_receive_imu2, //
};

extern "C" void
euroc_recorder_receive_gt(xrt_pose_sink *sink, xrt_pose_sample *sample)
{
	// This works similarly to euroc_recorder_receive_imu, read its comments
	euroc_recorder *er = container_of(sink, euroc_recorder, cloner_gt_sink);

	assert(er->settings.ground_truth_stream_count > 0);

	if (!er->recording) {
		return;
	}

	{
		lock_guard lock{er->gt_queue_lock};
		er->gt_queue.push(*sample);
	}
}


static void
euroc_recorder_receive_frame(euroc_recorder *er, xrt_frame *src_frame, uint32_t cam_index)
{
	assert(cam_index < er->settings.cam_count);

	if (!er->recording) {
		return;
	}

	// Let's clone the frame so that we can release the src_frame quickly
	xrt_frame *copy = nullptr;
	u_frame_clone(src_frame, &copy);

	xrt_sink_push_frame(er->writer_queues.cams[cam_index], copy);

	xrt_frame_reference(&copy, NULL);
}

#define DEFINE_RECEIVE_CAM(cam_id)                                                                                     \
	extern "C" void euroc_recorder_receive_cam##cam_id(xrt_frame_sink *sink, xrt_frame *frame)                     \
	{                                                                                                              \
		euroc_recorder *er = container_of(sink, euroc_recorder, cloner_sinks[cam_id]);                         \
		euroc_recorder_receive_frame(er, frame, cam_id);                                                       \
	}

XRT_TRACKING_FOR_EACH_CAM(DEFINE_RECEIVE_CAM)

#define USE_RECEIVE_CAM(cam_id) euroc_recorder_receive_cam##cam_id,

static void (*euroc_recorder_receive_cam[XRT_TRACKING_MAX_CAMS])(xrt_frame_sink *, xrt_frame *) = {
    XRT_TRACKING_FOR_EACH_CAM(USE_RECEIVE_CAM)};


/*
 *
 * Frame node functionality
 *
 */

extern "C" void
euroc_recorder_node_break_apart(xrt_frame_node *node)
{}

extern "C" void
euroc_recorder_node_destroy(xrt_frame_node *node)
{
	euroc_recorder *er = container_of(node, euroc_recorder, node);
	for (uint32_t i = 0; i < er->settings.imu_count; i++) {
		delete er->imus_csv[i];
	}
	//! @todo multiple gt streams
	if (er->settings.ground_truth_stream_count > 0) {
		delete er->gt_csv;
	}
	for (uint32_t i = 0; i < er->settings.cam_count; i++) {
		delete er->cams_csv[i];
	}
	delete er;
}


/*
 *
 * Exported functions
 *
 */

const char *
t_distortion_model_to_euroc(t_camera_distortion_model model)
{
	switch (model) {
	case T_DISTORTION_PINHOLE: return EUROC_DISTORTION_MODEL_RADTAN_4; break;
	case T_DISTORTION_OPENCV_RADTAN_5: return EUROC_DISTORTION_MODEL_RADTAN_5_MND; break;
	case T_DISTORTION_OPENCV_RADTAN_8: return EUROC_DISTORTION_MODEL_RADTAN_8_MND; break;
	case T_DISTORTION_OPENCV_RADTAN_14: return EUROC_DISTORTION_MODEL_RADTAN_14_MND; break;
	case T_DISTORTION_WMR: return EUROC_DISTORTION_MODEL_WMR_MND; break;
	case T_DISTORTION_FISHEYE_KB4: return EUROC_DISTORTION_MODEL_KB4_MND; break;
	case T_DISTORTION_RIFT_CV1: return EUROC_DISTORTION_MODEL_RIFT_CV1_MND; break;
	default: U_LOG_E("Invalid distortion_model! %d", model); return "INVALID";
	}
}

t_camera_distortion_model
t_euroc_model_to_distortion_model(const char *model_cstr)
{
	string model = model_cstr;
	if (model == EUROC_DISTORTION_MODEL_RADTAN_4)
		return T_DISTORTION_PINHOLE;
	if (model == EUROC_DISTORTION_MODEL_RADTAN_5_MND)
		return T_DISTORTION_OPENCV_RADTAN_5;
	if (model == EUROC_DISTORTION_MODEL_RADTAN_8_MND)
		return T_DISTORTION_OPENCV_RADTAN_8;
	if (model == EUROC_DISTORTION_MODEL_RADTAN_14_MND)
		return T_DISTORTION_OPENCV_RADTAN_14;
	if (model == EUROC_DISTORTION_MODEL_WMR_MND)
		return T_DISTORTION_WMR;
	if (model == EUROC_DISTORTION_MODEL_KB4_MND)
		return T_DISTORTION_FISHEYE_KB4;
	if (model == EUROC_DISTORTION_MODEL_RIFT_CV1_MND)
		return T_DISTORTION_RIFT_CV1;

	U_LOG_E("Invalid distortion_model! %s", model.c_str());
	return T_DISTORTION_PINHOLE;
}

extern "C" xrt_tracking_sinks *
euroc_recorder_create(xrt_frame_context *xfctx, const euroc_recorder_settings *settings)
{
	euroc_recorder *er = new euroc_recorder{};

	// Only 0 or 1 right now.
	assert(settings->ground_truth_stream_count <= 1);

	er->settings = *settings;
	er->path_prefix = settings->record_path == nullptr ? "euroc_recording" : settings->record_path;
	er->path = settings->record_path == nullptr ? "" : settings->record_path;

	xrt_frame_node *xfn = &er->node;
	xfn->break_apart = euroc_recorder_node_break_apart;
	xfn->destroy = euroc_recorder_node_destroy;
	xrt_frame_context_add(xfctx, xfn);

	er->use_jpg = debug_get_bool_option_euroc_recorder_use_jpg();

	// Setup sink pipeline

	// We expose a "cloner" sink that will clone frames in memory so that original
	// frames can be released as soon as possible. Not doing this could result in
	// frame queues from the user being filled up. After that we write to disk. We
	// also put queues between these to support sensors streaming on different
	// threads.
	// cloner_queue -> cloner_sink (clone ) -> writer_queue -> writer_sink (write to disk)

	er->cloner_queues.cam_count = settings->cam_count;
	er->writer_queues.cam_count = settings->cam_count;
	for (uint32_t i = 0; i < settings->cam_count; i++) {

		// If any of these asserts failed see docs on euroc_recorder_receive/save_cam
		assert(euroc_recorder_receive_cam[i] != nullptr);
		assert(euroc_recorder_save_cam[i] != nullptr);

		u_sink_queue_create(xfctx, 0, &er->cloner_sinks[i], &er->cloner_queues.cams[i]);
		er->cloner_sinks[i].push_frame = euroc_recorder_receive_cam[i];
		u_sink_queue_create(xfctx, 0, &er->writer_sinks[i], &er->writer_queues.cams[i]);
		er->writer_sinks[i].push_frame = euroc_recorder_save_cam[i];
	}

	er->cloner_queues.imu_count = settings->imu_count;
	er->writer_queues.imu_count = settings->imu_count;
	er->imu_queues = vector<queue<xrt_imu_sample>>(settings->imu_count);
	er->imu_queue_locks = vector<mutex>(settings->imu_count);
	for (uint32_t i = 0; i < settings->imu_count; i++) {
		assert(euroc_recorder_receive_imu[i] != nullptr);
		assert(euroc_recorder_save_imu[i] != nullptr);
		er->cloner_queues.imus[i] = &er->cloner_imu_sinks[i];
		er->cloner_imu_sinks[i].push_imu = euroc_recorder_receive_imu[i];
		er->writer_queues.imus[i] = nullptr; // We use a std::queue instead
		er->writer_imu_sinks[i].push_imu = euroc_recorder_save_imu[i];
	}

	er->cloner_queues.gt = &er->cloner_gt_sink;
	er->cloner_gt_sink.push_pose = euroc_recorder_receive_gt;
	er->writer_queues.gt = nullptr; // We use a std::queue instead
	er->writer_gt_sink.push_pose = euroc_recorder_save_gt;

	xrt_tracking_sinks *public_sinks = &er->cloner_queues;

	if (settings->record_from_start) {
		euroc_recorder_start(public_sinks);
	}

	return public_sinks;
}

extern "C" void
euroc_recorder_start(xrt_tracking_sinks *er_sinks)
{
	euroc_recorder *er = container_of(er_sinks, euroc_recorder, cloner_queues);

	if (er->recording) {
		U_LOG_W("We are already recording; unable to start.");
		return;
	}

	// Create dataset directories with current datetime suffix
	time_t seconds = os_realtime_get_ns() / U_TIME_1S_IN_NS;
	constexpr size_t size = sizeof("YYYYMMDDHHmmss");
	char datetime[size] = {0};
	(void)strftime(datetime, size, "%Y%m%d%H%M%S", localtime(&seconds));
	string default_path = er->path_prefix + "_" + datetime;
	er->path = default_path;

	er->makeFiles();
	er->recording = true;
}

extern "C" void
euroc_recorder_stop(xrt_tracking_sinks *er_sinks)
{
	euroc_recorder *er = container_of(er_sinks, euroc_recorder, cloner_queues);

	if (!er->recording) {
		U_LOG_W("We are already not recording; unable to stop.");
		return;
	}

	er->path = "";
	er->recording = false;
	er->flush();
}

static void
euroc_recorder_btn_cb(void *ptr)
{
	euroc_recorder *er = (euroc_recorder *)ptr;

	if (er->recording) {
		euroc_recorder_stop(&er->cloner_queues);
		(void)snprintf(er->recording_btn.label, sizeof(er->recording_btn.label), "Record EuRoC dataset");
	} else {
		euroc_recorder_start(&er->cloner_queues);
		(void)snprintf(er->recording_btn.label, sizeof(er->recording_btn.label), "Stop recording");
	}
}

extern "C" void
euroc_recorder_add_ui(xrt_tracking_sinks *er_sinks, void *root, const char *prefix)
{
	euroc_recorder *er = container_of(er_sinks, euroc_recorder, cloner_queues);
	er->recording_btn.cb = euroc_recorder_btn_cb;
	er->recording_btn.ptr = er;

	char tmp[256];
	(void)snprintf(tmp, sizeof(tmp), "%s%s", prefix, er->recording ? "Stop recording" : "Record EuRoC dataset");
	u_var_add_button(root, &er->recording_btn, tmp);
}
