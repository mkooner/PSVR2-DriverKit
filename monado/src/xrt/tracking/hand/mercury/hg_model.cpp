// Copyright 2021-2022, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Mercury ML models!
 * @author Moshi Turner <moshiturner@protonmail.com>
 * @ingroup drv_ht
 */

// Many C api things were stolen from here (MIT license):
// https://github.com/microsoft/onnxruntime-inference-examples/blob/main/c_cxx/fns_candy_style_transfer/fns_candy_style_transfer.c
#include "hg_sync.hpp"
#include "hg_gesture_geometry.hpp"
#include "hg_stereographic_unprojection.hpp"
#include <atomic>
#include <limits>
#include "hg_image_math.inl"
#include "hg_numerics_checker.hpp"
#include "hg_custom_gesture_weights.h"


#include <filesystem>
#include <array>

namespace xrt::tracking::hand::mercury {

using namespace xrt::auxiliary::onnx;

//! MediaPipe hand landmark model's input side (pixels).
constexpr int MP_SIZE = 224;

static cv::Matx23f
blackbar(const cv::Mat &in, enum t_camera_orientation rot, cv::Mat &out, xrt_size out_size)
{
	// Easy to think about, always right, but pretty slow:
	// Get a matrix from the original to the scaled down / blackbar'd image, then get one that goes back.
	// Then just warpAffine() it.
	// Easy in programmer time - never have to worry about off by one, special cases. We can come back and optimize
	// later.
	bool swapped_wh = false;
	float in_w, in_h;

	switch (rot) {
	case CAMERA_ORIENTATION_90:
	case CAMERA_ORIENTATION_270:
		// Swap width and height
		in_w = in.rows;
		in_h = in.cols;
		swapped_wh = true;
		break;
	default:
		in_w = in.cols;
		in_h = in.rows;
		break;
	}

	// Figure out from the rotation and frame sizes if the black bars need to be on top and bottom, or on left and
	// right?
	float scale_down_w = (float)out_size.w / in_w; // 128/1280 = 0.1
	float scale_down_h = (float)out_size.h / in_h; // 128/800 =  0.16

	float scale_down = fmin(scale_down_w, scale_down_h); // 0.1

	float width_inside, height_inside;

	if (swapped_wh) {
		width_inside = (float)in.rows * scale_down;
		height_inside = (float)in.cols * scale_down;
	} else {
		width_inside = (float)in.cols * scale_down;
		height_inside = (float)in.rows * scale_down;
	}

	float translate_x = (out_size.w - width_inside) / 2;  // should be 0 for 1280x800
	float translate_y = (out_size.h - height_inside) / 2; // should be (1280-800)/2 = 240

	cv::Matx23f go;
	cv::Point2f center(in.rows / 2, in.cols / 2);

	switch (rot) {
	case CAMERA_ORIENTATION_0:
		// clang-format off
			go(0,0) = scale_down;  go(0,1) = 0.0f;          go(0,2) = translate_x;
			go(1,0) = 0.0f;        go(1,1) = scale_down;    go(1,2) = translate_y;
		// clang-format on
		break;
	case CAMERA_ORIENTATION_90:
		// clang-format off
			go(0,0) = 0.0f;         go(0,1) = scale_down;   go(0,2) = translate_x;
			go(1,0) = -scale_down;  go(1,1) = 0.0f;         go(1,2) = translate_y+out_size.h-1;
		// clang-format on
		break;
	case CAMERA_ORIENTATION_180:
		// clang-format off
			go(0,0) = -scale_down;  go(0,1) = 0.0f;         go(0,2) = translate_x+out_size.w-1;
			go(1,0) = 0.0f;         go(1,1) = -scale_down;  go(1,2) = -translate_y+out_size.h-1;
		// clang-format on
		break;
	case CAMERA_ORIENTATION_270:
		// clang-format off
			go(0,0) = 0.0f;        go(0,1) = -scale_down;   go(0,2) = -translate_x+out_size.w-1;
			go(1,0) = scale_down;  go(1,1) = 0.0f;          go(1,2) = translate_y;
		// clang-format on
		break;
	}

	cv::warpAffine(in, out, go, cv::Size(out_size.w, out_size.h));

	// Return the inverse affine transform by passing
	// through a 3x3 rotation matrix
	cv::Mat e = cv::Mat::eye(3, 3, CV_32F);
	cv::Mat tmp = e(cv::Rect(0, 0, 3, 2));
	cv::Mat(go).copyTo(tmp);

	e = e.inv();
	cv::Matx23f ret = e(cv::Rect(0, 0, 3, 2));

	return ret;
}

static inline int
argmax(const float *data, int size)
{
	float max_value = data[0];
	int out_idx = 0;

	for (int i = 1; i < size; i++) {
		if (data[i] > max_value) {
			max_value = data[i];
			out_idx = i;
		}
	}
	return out_idx;
}

static bool
hand_depth_center_of_mass(struct HandTracking *hgt, float data[22], float *out_depth, float *out_confidence)
{
	float avg_location_px_coord = 0;
	float sum = 0;

	for (int i = 0; i < 22; i++) {
		data[i] = fmin(1.0, fmax(data[i], 0.0));
		sum += data[i];
		avg_location_px_coord += data[i] * (float)i;
	}

	if (sum < 1e-5) {
		HG_DEBUG(hgt, "All depth outputs were zero!");
		return false;
	}

	avg_location_px_coord /= sum;

	// std::cout << avg_location_px_coord << std::endl;

	// bounds check
	if (avg_location_px_coord < 0 || avg_location_px_coord > 21) {
		HG_DEBUG(hgt, "Very bad! avg_location_px_coord was %f", avg_location_px_coord);
		for (int i = 0; i < 22; i++) {
			HG_DEBUG(hgt, "%f", data[i]);
		}

		return false;
	}

	// nan check
	if (avg_location_px_coord != avg_location_px_coord) {
		HG_DEBUG(hgt, "Very bad! avg_location_px_coord was not a number: %f", avg_location_px_coord);
		for (int i = 0; i < 22; i++) {
			HG_DEBUG(hgt, "%f", data[i]);
		}
		*out_depth = 0;
		*out_confidence = 0;
		return false;
	}
	// printf("%f %d\n", avg_location_px_coord, (int)avg_location_px_coord);
	*out_confidence = data[(int)avg_location_px_coord];



	float depth_value = avg_location_px_coord + 0.5;
	depth_value /= 22;
	depth_value -= 0.5;
	depth_value *= 2 * 1.5;

	*out_depth = depth_value;
	return true;
}

static bool
refine_center_of_distribution(struct HandTracking *hgt, //
                              const float *data,        //
                              int coarse_x,             //
                              int coarse_y,             //
                              int w,                    //
                              int h,                    //
                              float *out_refined_x,     //
                              float *out_refined_y)
{
	// Be VERY suspicious of this function, it's probably not centering correctly.
	float sum_of_values = 0;
	float sum_of_values_times_locations_x = 0;
	float sum_of_values_times_locations_y = 0;

	int max_kern_width = 10;


	//! @todo this is not good and has at least one edge case, make it more readable and link to a jupyter notebook
	int kern_width_x = std::max(0, std::min(coarse_x, std::min(max_kern_width, abs(coarse_x - w) - 1)));
	int kern_width_y = std::max(0, std::min(coarse_y, std::min(max_kern_width, abs(coarse_y - h) - 1)));
	int min_x = coarse_x - kern_width_x;
	int max_x = coarse_x + kern_width_x;

	int min_y = coarse_y - kern_width_y;
	int max_y = coarse_y + kern_width_y;


	for (int y = min_y; y <= max_y; y++) {
		for (int x = min_x; x <= max_x; x++) {
			int acc = (y * w) + x;
			float val = data[acc];
			sum_of_values += val;
			sum_of_values_times_locations_y += val * ((float)y + 0.5);
			sum_of_values_times_locations_x += val * ((float)x + 0.5);
		}
	}

	if (sum_of_values == 0) {
		// Edge case, will fix soon
		*out_refined_x = coarse_x;
		*out_refined_y = coarse_y;
		HG_DEBUG(hgt, "Failed! %d %d %d %d %d", coarse_x, coarse_y, w, h, max_kern_width);
		return false;
	}
	*out_refined_x = sum_of_values_times_locations_x / sum_of_values;
	*out_refined_y = sum_of_values_times_locations_y / sum_of_values;
	return true;
}

static bool
normalizeGrayscaleImage(cv::Mat &data_in, cv::Mat &data_out)
{
	data_in.convertTo(data_out, CV_32FC1, 1 / 255.0);

	cv::Mat mean;
	cv::Mat stddev;
	cv::meanStdDev(data_out, mean, stddev);

	if (stddev.at<double>(0, 0) == 0) {
		U_LOG_W("Got image with zero standard deviation!");
		return false;
	}

	data_out *= 0.25 / stddev.at<double>(0, 0);

	// Calculate it again; mean has changed. Yes we don't need to but it's easy
	//! @todo optimize
	cv::meanStdDev(data_out, mean, stddev);
	data_out += (0.5 - mean.at<double>(0, 0));
	return true;
}

//! Most zoomed-in tiles a frame (detection_tiles up to 4 per side), each with its own detector input.
static constexpr int kMaxDetectionTiles = 16;

void
setup_ort_api(HandTracking *hgt, onnx_state *state, const std::filesystem::path &path)
{
	state->wrap = std::make_unique<OnnxWrapper>(hgt->log_level, path.string(), "monado_ht");
}

void
setup_model_image_input(HandTracking *hgt, onnx_state *state, const char *name, int64_t w, int64_t h)
{
	OnnxWrapper &wrap = *state->wrap;

	model_input_wrap inputimg = {};
	inputimg.name = name;
	inputimg.dimensions[0] = 1;
	inputimg.dimensions[1] = 1;
	inputimg.dimensions[2] = h;
	inputimg.dimensions[3] = w;
	inputimg.num_dimensions = 4;
	size_t data_size = w * h * sizeof(float);
	inputimg.data = (float *)malloc(data_size);

	ORT_SAFE(wrap, CreateTensorWithDataAsOrtValue(wrap.meminfo,                        //
	                                              inputimg.data,                       //
	                                              data_size,                           //
	                                              inputimg.dimensions,                 //
	                                              inputimg.num_dimensions,             //
	                                              ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, //
	                                              &inputimg.tensor));

	assert(inputimg.tensor);
	int is_tensor;
	ORT_SAFE(wrap, IsTensor(inputimg.tensor, &is_tensor));
	assert(is_tensor);

	state->wraps.push_back(inputimg);
}

void
init_hand_detection(HandTracking *hgt, onnx_state *wrap)
{
	std::filesystem::path path = hgt->models_folder;

	path /= "grayscale_detection_160x160.onnx";

	wrap->wraps.clear();

	setup_ort_api(hgt, wrap, path);

	setup_model_image_input(hgt, wrap, "inputImg", kDetectionInputSize, kDetectionInputSize);
	// One more input per zoomed-in tile, so they can run at the same time on the one session (Run is thread-safe).
	for (int i = 0; i < kMaxDetectionTiles; i++) {
		setup_model_image_input(hgt, wrap, "inputImg", kDetectionInputSize, kDetectionInputSize);
	}
}


/*!
 * One detector pass on @p img (a whole view, or a crop of one): each hand's confidence, and its centre and size in
 * @p img's pixels. @p binned_out: the detector's input image, if wanted.
 */
static void
detect_in(HandTracking *hgt,
          onnx_state *state,
          int input_idx,
          const cv::Mat &img,
          enum t_camera_orientation rot,
          float out_conf[2],
          xrt_vec2 out_center[2],
          float out_size[2],
          cv::Mat *binned_out,
          bool mirror_pass)
{
	OnnxWrapper &wrap = *state->wrap;

	cv::Mat binned_uint8;
	xrt_size desired_bin_size;
	desired_bin_size.h = kDetectionInputSize;
	desired_bin_size.w = kDetectionInputSize;
	cv::Matx23f go_back = blackbar(img, rot, binned_uint8, desired_bin_size);

	cv::Mat binned_float_wrapper_mat(cv::Size(kDetectionInputSize, kDetectionInputSize), CV_32FC1, //
	                                 state->wraps[input_idx].data, kDetectionInputSize * sizeof(float));
	normalizeGrayscaleImage(binned_uint8, binned_float_wrapper_mat);

	const OrtValue *inputs[] = {state->wraps[input_idx].tensor};
	const char *input_names[] = {state->wraps[input_idx].name};
	OrtValue *output_tensors[] = {nullptr, nullptr, nullptr, nullptr};
	const char *output_names[] = {"hand_exists", "cx", "cy", "size"};
	{
		XRT_TRACE_IDENT(model);
		static_assert(ARRAY_SIZE(input_names) == ARRAY_SIZE(inputs));
		static_assert(ARRAY_SIZE(output_names) == ARRAY_SIZE(output_tensors));
		ORT_SAFE(wrap, Run(wrap.session, nullptr, input_names, inputs, ARRAY_SIZE(input_names), output_names,
		                   ARRAY_SIZE(output_names), output_tensors));
	}
	float *hand_exists = nullptr, *cx = nullptr, *cy = nullptr, *sizee = nullptr;
	ORT_SAFE(wrap, GetTensorMutableData(output_tensors[0], (void **)&hand_exists));
	ORT_SAFE(wrap, GetTensorMutableData(output_tensors[1], (void **)&cx));
	ORT_SAFE(wrap, GetTensorMutableData(output_tensors[2], (void **)&cy));
	ORT_SAFE(wrap, GetTensorMutableData(output_tensors[3], (void **)&sizee));

	// Local copies, so the mirrored pass below can replace the left hand.
	float e_local[2] = {hand_exists[0], hand_exists[1]};
	float cx_local[2] = {cx[0], cx[1]};
	float cy_local[2] = {cy[0], cy[1]};
	float size_local[2] = {sizee[0], sizee[1]};

	if (mirror_pass && hgt->tuneable_values.detection_mirror_left) {
		// Same image mirrored: its right hand is our left hand, at the mirrored x (outputs are in [-1, 1]).
		cv::Mat mirrored;
		cv::flip(binned_uint8, mirrored, 1);
		normalizeGrayscaleImage(mirrored, binned_float_wrapper_mat);
		OrtValue *m_out[] = {nullptr, nullptr, nullptr, nullptr};
		ORT_SAFE(wrap, Run(wrap.session, nullptr, input_names, inputs, ARRAY_SIZE(input_names), output_names,
		                   ARRAY_SIZE(output_names), m_out));
		float *me = nullptr, *mx = nullptr, *my = nullptr, *ms = nullptr;
		ORT_SAFE(wrap, GetTensorMutableData(m_out[0], (void **)&me));
		ORT_SAFE(wrap, GetTensorMutableData(m_out[1], (void **)&mx));
		ORT_SAFE(wrap, GetTensorMutableData(m_out[2], (void **)&my));
		ORT_SAFE(wrap, GetTensorMutableData(m_out[3], (void **)&ms));
		if (me[1] > e_local[0]) {
			e_local[0] = me[1];
			cx_local[0] = -mx[1];
			cy_local[0] = my[1];
			size_local[0] = ms[1];
		}
		for (size_t i = 0; i < ARRAY_SIZE(m_out); i++) {
			wrap.api->ReleaseValue(m_out[i]);
		}
	}

	for (int hand_idx = 0; hand_idx < 2; hand_idx++) {
		out_conf[hand_idx] = e_local[hand_idx];
		// [-1, 1] -> detector input pixels -> pixels of img
		xrt_vec2 pt = {};
		pt.x = math_map_ranges(cx_local[hand_idx], -1, 1, 0, kDetectionInputSize);
		pt.y = math_map_ranges(cy_local[hand_idx], -1, 1, 0, kDetectionInputSize);
		constexpr float fac = 2.0f;
		float size = size_local[hand_idx] * kDetectionInputSize * fac;
		size *= m_vec2_len({go_back(0, 0), go_back(0, 1)});
		out_center[hand_idx] = transformVecBy2x3(pt, go_back);
		out_size[hand_idx] = size;
	}
	for (size_t i = 0; i < ARRAY_SIZE(output_tensors); i++) {
		wrap.api->ReleaseValue(output_tensors[i]);
	}
	if (binned_out != nullptr) {
		*binned_out = binned_uint8;
	}
}

struct detection_tile_job
{
	ht_view *view;
	int input_idx;
	int x0, y0, tw, th;
	float conf[2];
	xrt_vec2 center[2];
	float size[2];
};

static void
run_detection_tile(void *ptr)
{
	detection_tile_job *job = (detection_tile_job *)ptr;
	ht_view *view = job->view;
	HandTracking *hgt = view->hgt;
	job->conf[0] = job->conf[1] = 0.0f;
	try {
		cv::Mat crop = view->run_model_on_this(cv::Rect(job->x0, job->y0, job->tw, job->th));
		// No mirrored pass on the crops: the left hand wasn't missed for being a left hand (mirrored, it wasn't
		// found as a right hand either), only for being small; it halves their cost.
		detect_in(hgt, &view->detection, job->input_idx, crop, view->camera_info.camera_orientation, job->conf,
		          job->center, job->size, nullptr, hgt->tuneable_values.detection_tile_mirror);
	} catch (const std::exception &e) {
		HG_ERROR(hgt, "Exception while running a detection tile: %s", e.what());
		job->conf[0] = job->conf[1] = 0.0f;
	}
}

static void
run_hand_detection_unsafe(hand_detection_run_info *info)
{
	ht_view *view = info->view;
	HandTracking *hgt = view->hgt;
	onnx_state *state = &view->detection;
	cv::Mat &orig_data = view->run_model_on_this;
	const enum t_camera_orientation rot = view->camera_info.camera_orientation;

	float conf[2];
	xrt_vec2 center[2];
	float size[2];
	cv::Mat binned_uint8;
	detect_in(hgt, state, 0, orig_data, rot, conf, center, size, &binned_uint8, true);
	for (int h = 0; h < 2; h++) {
		detection_log("D,%lld,%d,-1,%d,%.4f,%.1f,%.1f,%.1f\n", (long long)hgt->current_frame_timestamp, view->view, h,
		              conf[h], center[h].x, center[h].y, size[h]);
	}

	// Offline diagnosis: MERCURY_DUMP_DIR saves every 30th detector input per view (PGM, as the model gets it).
	{
		static const char *dump = getenv("MERCURY_DUMP_DIR");
		static std::atomic<uint32_t> count[2];
		const int vi = view == &hgt->views[0] ? 0 : 1;
		const uint32_t c = count[vi]++;
		if (dump != nullptr && c % 30 == 0) {
			char path[1024];
			snprintf(path, sizeof(path), "%s/det_%05u_view%d.pgm", dump, c, vi);
			FILE *f = fopen(path, "wb");
			if (f != nullptr) {
				fprintf(f, "P5\n%d %d\n255\n", binned_uint8.cols, binned_uint8.rows);
				for (int y = 0; y < binned_uint8.rows; y++) {
					fwrite(binned_uint8.ptr(y), 1, binned_uint8.cols, f);
				}
				fclose(f);
			}
		}
	}

	/*
	 * Zoomed-in tiles: the whole view shrunk to the detector's 160x160 leaves a hand at arm's length only ~20 pixels
	 * across, too small to be found (2026-11-20 PS VR2 recording: only the hand nearest a camera, the right hand in
	 * the right camera, was ever found). When a hand isn't confidently found in the whole view, the detector also
	 * looks at overlapping crops (detection_tiles per side, each crop detection_tile_fraction of the view), and the
	 * most confident result per hand is kept, mapped back into the whole view. Only detection_tiles_per_frame crops a
	 * frame, cycling: all nine every frame took ~40 ms instead of ~8 (a third of the frames skipped); spread out, a
	 * missing hand is searched in full every nine frames (0.15 s at 60 Hz) for one or two more detector runs a frame.
	 */
	const int tiles = hgt->tuneable_values.detection_tiles;
	if (tiles >= 2 && (conf[0] < hgt->tuneable_values.acquire_confidence ||
	                   conf[1] < hgt->tuneable_values.acquire_confidence)) {
		const int W = orig_data.cols, H = orig_data.rows;
		const int tw = (int)(W * hgt->tuneable_values.detection_tile_fraction);
		const int th = (int)(H * hgt->tuneable_values.detection_tile_fraction);
		const int per_frame = hgt->tuneable_values.detection_tiles_per_frame > 0
		                          ? hgt->tuneable_values.detection_tiles_per_frame
		                          : tiles * tiles;
		/*
		 * The tiles of a frame run in parallel, each on its own detector input (all nine every frame took ~20 ms
		 * one after another; 1 per frame found the left hand in 27% of the labelled frames, all nine in 41%).
		 */
		const int n = std::min({per_frame, tiles * tiles, kMaxDetectionTiles});
		detection_tile_job jobs[kMaxDetectionTiles];
		for (int k = 0; k < n; k++) {
			const int t = (int)(view->tile_cursor++ % (uint32_t)(tiles * tiles));
			const int ix = t % tiles, iy = t / tiles;
			jobs[k] = {};
			jobs[k].view = view;
			jobs[k].input_idx = 1 + k;
			jobs[k].x0 = (W - tw) * ix / (tiles - 1);
			jobs[k].y0 = (H - th) * iy / (tiles - 1);
			jobs[k].tw = tw;
			jobs[k].th = th;
		}
		const uint64_t tiles_start_ns = os_monotonic_get_ns();
		if (n > 1 && view->tile_group != nullptr) {
			for (int k = 0; k < n; k++) {
				u_worker_group_push(view->tile_group, run_detection_tile, &jobs[k]);
			}
			u_worker_group_wait_all(view->tile_group);
		} else {
			for (int k = 0; k < n; k++) {
				run_detection_tile(&jobs[k]);
			}
		}
		// Every 10 s at debug level: how many tiles ran and their time per frame, to keep their cost in view.
		view->tile_stats.frames++;
		view->tile_stats.tiles += (uint32_t)n;
		view->tile_stats.ns += os_monotonic_get_ns() - tiles_start_ns;
		if (view->tile_stats.since_ns == 0) {
			view->tile_stats.since_ns = tiles_start_ns;
		} else if (tiles_start_ns - view->tile_stats.since_ns > 10ULL * U_TIME_1S_IN_NS) {
			HG_DEBUG(hgt, "View %d detection tiles: %u frames searched, %u tiles, %.2f ms per frame (%s)", view->view,
			         view->tile_stats.frames, view->tile_stats.tiles,
			         view->tile_stats.ns / 1e6 / view->tile_stats.frames,
			         n > 1 && view->tile_group != nullptr ? "parallel" : "one after another");
			view->tile_stats = {};
			view->tile_stats.since_ns = tiles_start_ns;
		}
		/*
		 * Development: MERCURY_TILE_DUMP=<dir> saves the view with every tile (thin grey) and each tile's detection
		 * box per hand above min_detection_confidence (left white, right black; thicker for the one kept), to check the
		 * tiles' detections sit on the hands. At most 400 images.
		 */
		static const char *tile_dump = getenv("MERCURY_TILE_DUMP");
		static std::atomic<int> tile_dumped{0};
		bool tile_hit = false;
		for (int k = 0; k < n && tile_dump != nullptr; k++) {
			tile_hit = tile_hit || jobs[k].conf[0] > hgt->tuneable_values.min_detection_confidence.val ||
			           jobs[k].conf[1] > hgt->tuneable_values.min_detection_confidence.val;
		}
		const float conf_before[2] = {conf[0], conf[1]};
		int kept[2] = {-1, -1};
		for (int k = 0; k < n; k++) {
			for (int h = 0; h < 2; h++) {
				if (jobs[k].conf[h] > 0.1f) {
					detection_log("D,%lld,%d,%d,%d,%.4f,%.1f,%.1f,%.1f\n", (long long)hgt->current_frame_timestamp,
					              view->view, k, h, jobs[k].conf[h],
					              jobs[k].center[h].x + jobs[k].x0, jobs[k].center[h].y + jobs[k].y0, jobs[k].size[h]);
				}
			}
		}
		for (int k = 0; k < n; k++) {
			for (int h = 0; h < 2; h++) {
				if (jobs[k].conf[h] > conf[h] &&
				    jobs[k].conf[h] >= hgt->tuneable_values.detection_tile_min_confidence) {
					conf[h] = jobs[k].conf[h];
					center[h] = {jobs[k].center[h].x + jobs[k].x0, jobs[k].center[h].y + jobs[k].y0};
					size[h] = jobs[k].size[h];
					kept[h] = k;
				}
			}
		}
		if (tile_hit && tile_dumped < 400) {
			const int idx = tile_dumped++;
			cv::Mat img;
			cv::cvtColor(orig_data, img, cv::COLOR_GRAY2BGR);
			for (int k = 0; k < n; k++) {
				cv::rectangle(img, cv::Rect(jobs[k].x0, jobs[k].y0, jobs[k].tw, jobs[k].th), cv::Scalar(90, 90, 90), 1);
			}
			for (int k = 0; k < n; k++) {
				for (int h = 0; h < 2; h++) {
					if (jobs[k].conf[h] <= hgt->tuneable_values.min_detection_confidence.val) {
						continue;
					}
					const float cx = jobs[k].center[h].x + jobs[k].x0, cy = jobs[k].center[h].y + jobs[k].y0;
					const float half = jobs[k].size[h] / 2.0f;
					const cv::Scalar col = h == 0 ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 0);
					cv::rectangle(img, cv::Rect((int)(cx - half), (int)(cy - half), (int)(2 * half), (int)(2 * half)), col,
					              kept[h] == k ? 3 : 1);
					char txt[32];
					snprintf(txt, sizeof(txt), "%s %.2f", h == 0 ? "L" : "R", jobs[k].conf[h]);
					cv::putText(img, txt, cv::Point((int)(cx - half), (int)(cy - half) - 3), cv::FONT_HERSHEY_SIMPLEX, 0.45,
					            col, 1);
				}
			}
			char path[1024];
			snprintf(path, sizeof(path), "%s/tile_%04d_v%d_%lld_whole_L%.2f_R%.2f.png", tile_dump, idx, view->view,
			         (long long)hgt->current_frame_timestamp, conf_before[0], conf_before[1]);
			cv::imwrite(path, img);
		}
	}

	for (int hand_idx = 0; hand_idx < 2; hand_idx++) {
		hand_region_of_interest &output = info->outputs[hand_idx];
		output.found = conf[hand_idx] > hgt->tuneable_values.min_detection_confidence.val;
		if (output.found) {
			output.hand_detection_confidence = conf[hand_idx];
			output.center_px = center[hand_idx];
			output.size_px = size[hand_idx];
			if (hgt->debug_scribble) {
				handSquare(view->debug_out_to_this, output.center_px, output.size_px, PINK);
			}
		}
		if (hgt->debug_scribble) {
			// note: this will multiply the model outputs by 255, don't do anything with them after this.
			int top_of_rect_y = kVisSpacerSize;
			int left_of_rect_x = kVisSpacerSize + ((kKeypointInputSize + kVisSpacerSize) * 4);
			int start_y = top_of_rect_y + ((kDetectionInputSize + kVisSpacerSize) * view->view);
			cv::Rect p = cv::Rect(left_of_rect_x, start_y, kDetectionInputSize, kDetectionInputSize);
			binned_uint8.copyTo(hgt->visualizers.mat(p));
		}
	}
}
void
run_hand_detection(void *ptr)
{
	XRT_TRACE_MARKER();

	hand_detection_run_info *info = (hand_detection_run_info *)ptr;
	try {
		run_hand_detection_unsafe(info);
	} catch (const std::exception &e) {
		HG_ERROR(info->view->hgt, "Exception while running hand detection: %s", e.what());

		// Report nothing found if hand detection hits an exception
		for (auto &output : info->outputs) {
			output.found = false;
		}
	}
}

void
init_keypoint_estimation_mediapipe(HandTracking *hgt, onnx_state *state)
{
	{
		// MediaPipe's hand landmark model: input_1 [1, 3, 224, 224] (RGB 0..1), outputs 21 x (x, y, z) in input
		// pixels, hand presence, handedness and 21 metric points.
		std::filesystem::path path = hgt->models_folder;
		path /= "hand_landmarks_detector.onnx";
		state->wraps.clear();
		state->wrap = std::make_unique<OnnxWrapper>(hgt->log_level, path.string(), "monado_ht");
		OnnxWrapper &wrap = *state->wrap;
		model_input_wrap inputimg = {};
		inputimg.name = "input_1";
		inputimg.dimensions[0] = 1;
		inputimg.dimensions[1] = 3;
		inputimg.dimensions[2] = MP_SIZE;
		inputimg.dimensions[3] = MP_SIZE;
		inputimg.num_dimensions = 4;
		const size_t bytes = 3 * MP_SIZE * MP_SIZE * sizeof(float);
		inputimg.data = (float *)malloc(bytes);
		ORT_SAFE(wrap, CreateTensorWithDataAsOrtValue(wrap.meminfo, inputimg.data, bytes, inputimg.dimensions,
		                                              inputimg.num_dimensions,
		                                              ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &inputimg.tensor));
		state->wraps.push_back(inputimg);
	}
}

//! An ONNX input of @p dims (float) owned by @p state.
static void
add_float_input(OnnxWrapper &wrap, onnx_state *state, const char *name, std::initializer_list<int64_t> dims)
{
	model_input_wrap in = {};
	in.name = name;
	size_t count = 1;
	for (int64_t d : dims) {
		in.dimensions[in.num_dimensions++] = d;
		count *= (size_t)d;
	}
	in.data = (float *)calloc(count, sizeof(float));
	ORT_SAFE(wrap, CreateTensorWithDataAsOrtValue(wrap.meminfo, in.data, count * sizeof(float), in.dimensions,
	                                              in.num_dimensions, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &in.tensor));
	state->wraps.push_back(in);
}

void
init_gesture_models(HandTracking *hgt)
{
	std::filesystem::path embedder = hgt->models_folder, classifier = hgt->models_folder;
	embedder /= "gesture_embedder.onnx";
	classifier /= "canned_gesture_classifier.onnx";
	hgt->gesture_embedder.wraps.clear();
	hgt->gesture_embedder.wrap = std::make_unique<OnnxWrapper>(hgt->log_level, embedder.string(), "monado_ht");
	add_float_input(*hgt->gesture_embedder.wrap, &hgt->gesture_embedder, "hand", {1, 21, 3});
	add_float_input(*hgt->gesture_embedder.wrap, &hgt->gesture_embedder, "handedness", {1, 1});
	add_float_input(*hgt->gesture_embedder.wrap, &hgt->gesture_embedder, "world_hand", {1, 21, 3});
	hgt->gesture_classifier.wraps.clear();
	hgt->gesture_classifier.wrap = std::make_unique<OnnxWrapper>(hgt->log_level, classifier.string(), "monado_ht");
	add_float_input(*hgt->gesture_classifier.wrap, &hgt->gesture_classifier, "hand_embedding", {1, 128});
}

/*!
 * MediaPipe's LandmarksToMatrixCalculator with object normalization (origin landmark 0), as its gesture recognizer
 * graph runs it (mediapipe/tasks/cc/vision/gesture_recognizer/calculators/landmarks_to_matrix_calculator.cc):
 * subtract the wrist, divide by the larger of the x and y extents (+1e-5). Its max_x / max_y start at
 * numeric_limits<float>::min() (the smallest positive float), kept so results match. The rotation step there undoes
 * the crop's turn, which our crop-space landmarks never had.
 */
static void
mediapipe_object_normalize(const float in[63], float out[63])
{
	float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::min();
	float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::min();
	for (int i = 0; i < 21; i++) {
		for (int c = 0; c < 3; c++) {
			out[i * 3 + c] = in[i * 3 + c] - in[c];
		}
		min_x = fminf(min_x, out[i * 3]), max_x = fmaxf(max_x, out[i * 3]);
		min_y = fminf(min_y, out[i * 3 + 1]), max_y = fmaxf(max_y, out[i * 3 + 1]);
	}
	const float scale = fmaxf(max_x - min_x, max_y - min_y) + 1e-5f;
	for (int i = 0; i < 63; i++) {
		out[i] /= scale;
	}
}

//! The camera direction (Mercury's view frame) of a MediaPipe crop pixel (u, v), the crop never mirrored.
static Eigen::Vector3f
mediapipe_crop_direction(const Eigen::Quaternionf &rot, float radius, float u, float v)
{
	const float s = (float)MP_SIZE;
	return rot * stereographic_unprojection(-radius + 2.0f * radius * u / s, radius - 2.0f * radius * v / s);
}

void
compute_hand_gestures(HandTracking *hgt, int hand, struct t_hand_gestures *out, int only_view, float *embedding_out)
{
	*out = {};
	int best = -1;
	for (int v = 0; v < 2; v++) {
		if (only_view >= 0 && v != only_view) {
			continue;
		}
		const HandTracking::mp_view_output &o = hgt->mp_outputs[hand][v];
		if (o.valid && o.presence >= hgt->tuneable_values.mediapipe_presence &&
		    (best < 0 || o.presence > hgt->mp_outputs[hand][best].presence)) {
			best = v;
		}
	}
	if (best < 0) {
		return;
	}
	const HandTracking::mp_view_output &o = hgt->mp_outputs[hand][best];

	// Geometry on the hand-centred metric landmarks.
	gesture::Landmarks lm;
	for (int i = 0; i < 21; i++) {
		lm.p[i] = {o.world[i * 3], o.world[i * 3 + 1], o.world[i * 3 + 2]};
	}
	lm.valid = true;
	const gesture::HandGeometry geo = gesture::measure(lm);
	if (!geo.valid) {
		return;
	}

	/*
	 * Gesture Recognizer inputs as MediaPipe's gesture graph gets them: the landmarks projected back onto the camera
	 * image (LandmarkProjectionCalculator), normalised to the image; z as the landmark graph's (/ 224 / 0.4,
	 * kLandmarksNormalizeZ) times the crop's size in the image; the world landmarks turned back to the image's
	 * orientation (WorldLandmarkProjectionCalculator). The gesture graph turns them only by the caller's image
	 * rotation (the whole image, none here), not the hand's tracking rect, so the hand is as the camera sees it, not
	 * upright. Then object-normalised (LandmarksToMatrixCalculator).
	 */
	const ht_view &view = hgt->views[best];
	t_camera_model_params dist = view.hgdist;
	const float W = (float)view.run_model_on_this.cols, H = (float)view.run_model_on_this.rows;
	const auto to_image = [&](float u, float v, float *x, float *y) {
		const Eigen::Vector3f d = mediapipe_crop_direction(o.rot, o.radius, u, v);
		return t_camera_models_project(&dist, d.x(), -d.y(), -d.z(), x, y);
	};
	float img[63], world_img[63], norm[63], world_norm[63];
	for (int i = 0; i < 21; i++) {
		float x, y;
		if (!to_image(o.landmarks[i * 3], o.landmarks[i * 3 + 1], &x, &y)) {
			return;
		}
		img[i * 3] = x / W;
		img[i * 3 + 1] = y / H;
	}
	{
		// The crop's scale and turn in the image, from its centre: one crop pixel along x and along y.
		const float c = MP_SIZE / 2.0f;
		float x0, y0, x1, y1, x2, y2;
		if (!to_image(c, c, &x0, &y0) || !to_image(c + 1, c, &x1, &y1) || !to_image(c, c + 1, &x2, &y2)) {
			return;
		}
		const float ex_x = x1 - x0, ex_y = y1 - y0, ey_x = x2 - x0, ey_y = y2 - y0;
		const float scale = sqrtf(ex_x * ex_x + ex_y * ex_y); // image pixels per crop pixel
		const float ex_n = scale > 1e-6f ? 1.0f / scale : 0.0f;
		const float ey_l = sqrtf(ey_x * ey_x + ey_y * ey_y), ey_n = ey_l > 1e-6f ? 1.0f / ey_l : 0.0f;
		for (int i = 0; i < 21; i++) {
			img[i * 3 + 2] = o.landmarks[i * 3 + 2] * scale / (0.4f * W);
			const float wx = o.world[i * 3], wy = o.world[i * 3 + 1];
			world_img[i * 3] = wx * ex_x * ex_n + wy * ey_x * ey_n;
			world_img[i * 3 + 1] = wx * ex_y * ex_n + wy * ey_y * ey_n;
			world_img[i * 3 + 2] = o.world[i * 3 + 2];
		}
	}
	mediapipe_object_normalize(img, norm);
	mediapipe_object_normalize(world_img, world_norm);
	onnx_state &e = hgt->gesture_embedder;
	memcpy(e.wraps[0].data, norm, sizeof(norm));
	e.wraps[1].data[0] = o.handedness;
	memcpy(e.wraps[2].data, world_norm, sizeof(world_norm));
	OnnxWrapper &ew = *e.wrap;
	const OrtValue *e_in[] = {e.wraps[0].tensor, e.wraps[1].tensor, e.wraps[2].tensor};
	const char *e_names[] = {e.wraps[0].name, e.wraps[1].name, e.wraps[2].name};
	OrtValue *e_out = nullptr;
	const char *e_out_name = "Identity";
	ORT_SAFE(ew, Run(ew.session, nullptr, e_names, e_in, 3, &e_out_name, 1, &e_out));
	float *embedding = nullptr;
	ORT_SAFE(ew, GetTensorMutableData(e_out, (void **)&embedding));
	if (embedding_out != nullptr) {
		memcpy(embedding_out, embedding, 128 * sizeof(float));
	}
	// Development (MERCURY_DETECTION_LOG, per-view calls only): "E" lines with everything Hand Landmarker gives for
	// this hand in this view: frame time ns, hand, view, presence, right-hand score, then the 21 world landmarks (m,
	// hand-centred, 63), the 21 crop landmarks (crop pixels and z, 63), the gesture embedding (128).
	if (only_view >= 0 && getenv("MERCURY_DETECTION_LOG") != nullptr) {
		char line[16384];
		int n = snprintf(line, sizeof(line), "E,%lld,%d,%d,%.4f,%.4f", (long long)hgt->current_frame_timestamp,
		                 hand, best, o.presence, o.handedness);
		for (int i = 0; i < 63 && n < (int)sizeof(line) - 32; i++) {
			n += snprintf(line + n, sizeof(line) - n, ",%.5f", o.world[i]);
		}
		for (int i = 0; i < 63 && n < (int)sizeof(line) - 32; i++) {
			n += snprintf(line + n, sizeof(line) - n, ",%.2f", o.landmarks[i]);
		}
		for (int i = 0; i < 128 && n < (int)sizeof(line) - 32; i++) {
			n += snprintf(line + n, sizeof(line) - n, ",%.5f", embedding[i]);
		}
		detection_log("%s\n", line);
	}
	onnx_state &c = hgt->gesture_classifier;
	memcpy(c.wraps[0].data, embedding, 128 * sizeof(float));
	ew.api->ReleaseValue(e_out);
	OnnxWrapper &cw = *c.wrap;
	const OrtValue *c_in[] = {c.wraps[0].tensor};
	const char *c_names[] = {c.wraps[0].name};
	OrtValue *c_out = nullptr;
	ORT_SAFE(cw, Run(cw.session, nullptr, c_names, c_in, 1, &e_out_name, 1, &c_out));
	float *scores = nullptr;
	ORT_SAFE(cw, GetTensorMutableData(c_out, (void **)&scores));
	int top = 0;
	for (int i = 1; i < T_HAND_GESTURE_COUNT; i++) {
		top = scores[i] > scores[top] ? i : top;
	}
	const bool scores_ok = std::isfinite(scores[top]);
	out->category = (enum t_hand_gesture_category)top;
	out->category_score = scores_ok ? scores[top] : 0.0f;
	cw.api->ReleaseValue(c_out);

	out->confidence = o.presence;
	out->pinch_distance_normalized = geo.pinch_distance_normalized;
	out->extended_mask = geo.extended_mask;
	out->valid = scores_ok;
}

/*!
 * The custom mapping's joint measures of one camera's MediaPipe world landmarks (as psvr2-test gesture_map.py's
 * joints_feats): thumb tip to each fingertip and each fingertip to the wrist (index, middle, ring, little; / palm width,
 * index to little finger knuckle), then the cosine of the bend at 10 finger joints.
 */
static void
custom_gesture_joint_measures(const float *world, float out[18])
{
	const auto p = [&](int i) { return Eigen::Vector3f(world[i * 3], world[i * 3 + 1], world[i * 3 + 2]); };
	const float palm = (p(5) - p(17)).norm() + 1e-6f;
	int n = 0;
	for (int tip : {8, 12, 16, 20}) {
		out[n++] = (p(4) - p(tip)).norm() / palm;
		out[n++] = (p(tip) - p(0)).norm() / palm;
	}
	static const int bends[10][3] = {{5, 6, 7},    {6, 7, 8},    {9, 10, 11},  {10, 11, 12}, {13, 14, 15},
	                                 {14, 15, 16}, {17, 18, 19}, {18, 19, 20}, {1, 2, 3},    {2, 3, 4}};
	for (const auto &b : bends) {
		const Eigen::Vector3f u = p(b[0]) - p(b[1]), v = p(b[2]) - p(b[1]);
		out[n++] = u.dot(v) / (u.norm() * v.norm() + 1e-9f);
	}
}

void
compute_custom_gesture(HandTracking *hgt, int hand, struct t_hand_gestures *out)
{
	out->custom_valid = false;
	if (!hgt->tuneable_values.custom_gestures) {
		return;
	}
	static_assert(HG_CUSTOM_GESTURE_FEATURES == 2 * 128 + 2 + 2 * 18 + 2, "feature layout");
	static_assert(HG_CUSTOM_GESTURE_CLASSES == T_HAND_CUSTOM_COUNT, "classes");
	float x[HG_CUSTOM_GESTURE_FEATURES] = {};
	bool have[2] = {false, false};
	for (int v = 0; v < 2; v++) {
		const HandTracking::mp_view_output &o = hgt->mp_outputs[hand][v];
		if (!o.valid || o.presence < hgt->tuneable_values.mediapipe_presence) {
			continue;
		}
		struct t_hand_gestures g;
		float emb[128];
		memset(emb, 0, sizeof(emb));
		compute_hand_gestures(hgt, hand, &g, v, emb);
		if (!g.valid) {
			continue;
		}
		have[v] = true;
		memcpy(&x[v * 128], emb, sizeof(emb));
		custom_gesture_joint_measures(o.world, &x[2 * 128 + 2 + v * 18]);
	}
	const int64_t now = hgt->current_frame_timestamp;
	if (!have[0] && !have[1]) {
		return;
	}
	x[256] = have[0] ? 1.0f : 0.0f, x[257] = have[1] ? 1.0f : 0.0f;
	x[HG_CUSTOM_GESTURE_FEATURES - 2] = x[256], x[HG_CUSTOM_GESTURE_FEATURES - 1] = x[257];

	const hg_custom_gesture_model &m = hg_custom_gesture_models[hand];
	float z[HG_CUSTOM_GESTURE_CLASSES];
	for (int c = 0; c < HG_CUSTOM_GESTURE_CLASSES; c++) {
		float acc = m.bias[c];
		const float *w = &m.weight[c * HG_CUSTOM_GESTURE_FEATURES];
		for (int i = 0; i < HG_CUSTOM_GESTURE_FEATURES; i++) {
			acc += w[i] * (x[i] - m.mean[i]) / m.scale[i];
		}
		z[c] = acc;
	}

	// The mean score over this hand's last 10 frames with a reading (restarted after 100 ms without one), as fitted.
	if (hgt->custom_count[hand] > 0 && now - hgt->custom_last_ns[hand] > 100 * U_TIME_1MS_IN_NS) {
		hgt->custom_count[hand] = 0;
	}
	hgt->custom_last_ns[hand] = now;
	memcpy(hgt->custom_logits[hand][hgt->custom_next[hand]], z, sizeof(z));
	hgt->custom_next[hand] = (hgt->custom_next[hand] + 1) % 10;
	hgt->custom_count[hand] = std::min(hgt->custom_count[hand] + 1, 10);
	float mean[HG_CUSTOM_GESTURE_CLASSES] = {};
	for (int k = 0; k < hgt->custom_count[hand]; k++) {
		const int slot = (hgt->custom_next[hand] - 1 - k + 10) % 10;
		for (int c = 0; c < HG_CUSTOM_GESTURE_CLASSES; c++) {
			mean[c] += hgt->custom_logits[hand][slot][c] / (float)hgt->custom_count[hand];
		}
	}
	int top = 0;
	float mx = mean[0];
	for (int c = 1; c < HG_CUSTOM_GESTURE_CLASSES; c++) {
		if (mean[c] > mean[top]) {
			top = c;
		}
		mx = std::max(mx, mean[c]);
	}
	float sum = 0.0f;
	for (int c = 0; c < HG_CUSTOM_GESTURE_CLASSES; c++) {
		out->custom_scores[c] = expf(mean[c] - mx);
		sum += out->custom_scores[c];
	}
	for (int c = 0; c < HG_CUSTOM_GESTURE_CLASSES; c++) {
		out->custom_scores[c] /= sum;
	}
	out->custom = (enum t_hand_custom_gesture)top;
	out->custom_valid = std::isfinite(sum) && sum > 0.0f;
}

void
init_keypoint_estimation(HandTracking *hgt, onnx_state *state)
{
	std::filesystem::path path = hgt->models_folder;

	path /= "grayscale_keypoint_jan18.onnx";

	state->wraps.clear();

	state->wrap = std::make_unique<OnnxWrapper>(hgt->log_level, path.string(), "monado_ht");

	OnnxWrapper &wrap = *state->wrap;

	// size_t input_size = wrap->input_shape[0] * wrap->input_shape[1] * wrap->input_shape[2] *
	// wrap->input_shape[3];

	// wrap->data = (float *)malloc(input_size * sizeof(float));
	{
		model_input_wrap inputimg = {};
		inputimg.name = "inputImg";
		inputimg.dimensions[0] = 1;
		inputimg.dimensions[1] = 1;
		inputimg.dimensions[2] = 128;
		inputimg.dimensions[3] = 128;
		inputimg.num_dimensions = 4;

		inputimg.data = (float *)malloc(128 * 128 * sizeof(float)); // SORRY IM BUSY



		ORT_SAFE(wrap, CreateTensorWithDataAsOrtValue(wrap.meminfo,                        //
		                                              inputimg.data,                       //
		                                              128 * 128 * sizeof(float),           //
		                                              inputimg.dimensions,                 //
		                                              inputimg.num_dimensions,             //
		                                              ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, //
		                                              &inputimg.tensor));

		assert(inputimg.tensor);
		int is_tensor;
		ORT_SAFE(wrap, IsTensor(inputimg.tensor, &is_tensor));
		assert(is_tensor);

		state->wraps.push_back(inputimg);
	}

	{
		model_input_wrap inputimg = {};
		inputimg.name = "lastKeypoints";
		inputimg.dimensions[0] = 1;
		inputimg.dimensions[1] = 42;
		inputimg.num_dimensions = 2;

		inputimg.data = (float *)malloc(1 * 42 * sizeof(float)); // SORRY IM BUSY



		ORT_SAFE(wrap, CreateTensorWithDataAsOrtValue(wrap.meminfo,                        //
		                                              inputimg.data,                       //
		                                              1 * 42 * sizeof(float),              //
		                                              inputimg.dimensions,                 //
		                                              inputimg.num_dimensions,             //
		                                              ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, //
		                                              &inputimg.tensor));

		assert(inputimg.tensor);
		int is_tensor;
		ORT_SAFE(wrap, IsTensor(inputimg.tensor, &is_tensor));
		assert(is_tensor);
		state->wraps.push_back(inputimg);
	}

	{
		model_input_wrap inputimg = {};
		inputimg.name = "useLastKeypoints";
		inputimg.dimensions[0] = 1;
		inputimg.num_dimensions = 1;

		inputimg.data = (float *)malloc(1 * sizeof(float)); // SORRY IM BUSY



		ORT_SAFE(wrap, CreateTensorWithDataAsOrtValue(wrap.meminfo,                        //
		                                              inputimg.data,                       //
		                                              1 * sizeof(float),                   //
		                                              inputimg.dimensions,                 //
		                                              inputimg.num_dimensions,             //
		                                              ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, //
		                                              &inputimg.tensor));

		assert(inputimg.tensor);
		int is_tensor;
		ORT_SAFE(wrap, IsTensor(inputimg.tensor, &is_tensor));
		assert(is_tensor);
		state->wraps.push_back(inputimg);
	}
}

enum xrt_hand_joint joints_ml_to_xr[21]{
    XRT_HAND_JOINT_WRIST,               //
    XRT_HAND_JOINT_THUMB_METACARPAL,    //
    XRT_HAND_JOINT_THUMB_PROXIMAL,      //
    XRT_HAND_JOINT_THUMB_DISTAL,        //
    XRT_HAND_JOINT_THUMB_TIP,           //
                                        //
    XRT_HAND_JOINT_INDEX_PROXIMAL,      //
    XRT_HAND_JOINT_INDEX_INTERMEDIATE,  //
    XRT_HAND_JOINT_INDEX_DISTAL,        //
    XRT_HAND_JOINT_INDEX_TIP,           //
                                        //
    XRT_HAND_JOINT_MIDDLE_PROXIMAL,     //
    XRT_HAND_JOINT_MIDDLE_INTERMEDIATE, //
    XRT_HAND_JOINT_MIDDLE_DISTAL,       //
    XRT_HAND_JOINT_MIDDLE_TIP,          //
                                        //
    XRT_HAND_JOINT_RING_PROXIMAL,       //
    XRT_HAND_JOINT_RING_INTERMEDIATE,   //
    XRT_HAND_JOINT_RING_DISTAL,         //
    XRT_HAND_JOINT_RING_TIP,            //
                                        //
    XRT_HAND_JOINT_LITTLE_PROXIMAL,     //
    XRT_HAND_JOINT_LITTLE_INTERMEDIATE, //
    XRT_HAND_JOINT_LITTLE_DISTAL,       //
    XRT_HAND_JOINT_LITTLE_TIP,          //
};

void
make_keypoint_heatmap_output(int camera_idx, int hand_idx, int grid_pt_x, int grid_pt_y, float *plane, cv::Mat &out)
{
	int root_x = 8 + ((1 + 2 * hand_idx) * (128 + 8));
	int root_y = 8 + ((2 * camera_idx) * (128 + 8));

	int org_x = (root_x) + (grid_pt_x * 25);
	int org_y = (root_y) + (grid_pt_y * 25);
	cv::Rect p = cv::Rect(org_x, org_y, 22, 22);


	cv::Mat start(cv::Size(22, 22), CV_32FC1, plane, 22 * sizeof(float));
	start *= 255.0;

	start.copyTo(out(p));
}

void
make_keypoint_depth_heatmap_output(int camera_idx, //
                                   int hand_idx,   //
                                   int grid_pt_x,  //
                                   int grid_pt_y,
                                   float *plane, //
                                   cv::Mat &out)
{
	int root_x = 8 + ((1 + 2 * hand_idx) * (128 + 8));
	int root_y = 8 + ((1 + 2 * camera_idx) * (128 + 8));

	int org_x = (root_x) + (grid_pt_x * 25);
	int org_y = (root_y) + (grid_pt_y * 25);


	cv::Rect p = cv::Rect(org_x, org_y, 22, 22);


	cv::Mat start(cv::Size(22, 22), CV_32FC1);
	float *ptr = (float *)start.data; // Cope cope cope cope cope


	for (int i = 0; i < 22; i++) {
		for (int j = 0; j < 22; j++) {
			ptr[(i * 22) + j] = plane[i];
		}
	}

	start *= 255.0;

	start.copyTo(out(p));
}

void
set_predicted_zero(float *data)
{
	for (int i = 0; i < 42; i++) {
		data[i] = 0.0f;
	}
}

/*!
 * The turn (twist, radians) that brings the direction from the wrist (0) to the middle finger's knuckle (9) to
 * straight up in a crop made with @p current_twist, given those two points in that crop's stereographic frame
 * (x right, y up, before any mirroring). A point at angle a in the crop lands at a + (twist change).
 */
static float
upright_twist(float current_twist, xrt_vec2 wrist_sg, xrt_vec2 middle_sg)
{
	const float dx = middle_sg.x - wrist_sg.x, dy = middle_sg.y - wrist_sg.y;
	if (dx * dx + dy * dy < 1e-12f) {
		return current_twist;
	}
	return current_twist + (float)M_PI_2 - atan2f(dy, dx);
}

/*!
 * Runs MediaPipe's landmark model on the crop @p instr describes; the points come back in Mercury's scaled
 * stereographic form (x, y in -1..1 of the crop, mirrored like the crop), with depth relative to the middle knuckle
 * divided by the wrist-to-knuckle length. Returns the hand presence.
 */
static float
mediapipe_run(keypoint_estimation_run_info &info, const projection_instructions &instr, MLOutput2D &px_coord,
              cv::Mat &crop)
{
	onnx_state *state = &info.view->keypoint_mp[info.hand_idx];
	OnnxWrapper &wrap = *state->wrap;
	struct HandTracking *hgt = info.view->hgt;
	const int view_idx = info.view->view;

	stereographic_project_image_sized(info.view->hgdist, instr, hgt->views[view_idx].run_model_on_this, MP_SIZE,
	                                  crop);

	// Contrast stretch (1st to 99th percentile over the crop); the same gray in all three channels. (Measured over the
	// crop's middle only, or CLAHE first: the dim left hand was rejected more, 51-52% vs 35%; 2026-11-24.)
	{
		XRT_TRACE_IDENT(convert_format);
		int hist[256] = {0};
		const int n = MP_SIZE * MP_SIZE;
		const uint8_t *px = crop.ptr<uint8_t>(0);
		for (int i = 0; i < n; i++) {
			hist[px[i]]++;
		}
		int lo = 0, hi = 255, acc = 0;
		for (int v = 0; v < 256; v++) {
			acc += hist[v];
			if (acc >= n / 100) {
				lo = v;
				break;
			}
		}
		acc = 0;
		for (int v = 255; v >= 0; v--) {
			acc += hist[v];
			if (acc >= n / 100) {
				hi = v;
				break;
			}
		}
		const float scale = 1.0f / fmaxf((float)(hi - lo), 1.0f);
		float *data = state->wraps[0].data;
		for (int i = 0; i < n; i++) {
			const float v = fminf(fmaxf((px[i] - lo) * scale, 0.0f), 1.0f);
			data[i] = v, data[n + i] = v, data[2 * n + i] = v;
		}
	}

	const OrtValue *inputs[] = {state->wraps[0].tensor};
	const char *input_names[] = {state->wraps[0].name};
	// Identity: 21 x (x, y, z) in input pixels; Identity_1: presence; Identity_2: right-hand score; Identity_3: 21
	// hand-centred metric points.
	OrtValue *output_tensors[] = {nullptr, nullptr, nullptr, nullptr};
	const char *output_names[] = {"Identity", "Identity_1", "Identity_2", "Identity_3"};
	{
		XRT_TRACE_IDENT(model);
		ORT_SAFE(wrap, Run(wrap.session, nullptr, input_names, inputs, 1, output_names, 4, output_tensors));
	}
	float *pts = nullptr, *presence = nullptr, *handedness = nullptr, *world = nullptr;
	ORT_SAFE(wrap, GetTensorMutableData(output_tensors[0], (void **)&pts));
	ORT_SAFE(wrap, GetTensorMutableData(output_tensors[1], (void **)&presence));
	ORT_SAFE(wrap, GetTensorMutableData(output_tensors[2], (void **)&handedness));
	ORT_SAFE(wrap, GetTensorMutableData(output_tensors[3], (void **)&world));
	const float present = presence[0];
	{
		// Kept for the gesture measurements (compute_hand_gestures); this job owns this [hand][view] slot.
		HandTracking::mp_view_output &o = hgt->mp_outputs[info.hand_idx][view_idx];
		o.rot = instr.rot_quat;
		o.radius = instr.stereographic_radius;
		o.presence = present;
		o.handedness = handedness[0];
		memcpy(o.landmarks, pts, sizeof(o.landmarks));
		memcpy(o.world, world, sizeof(o.world));
		o.valid = true;
	}

	const float s = (float)MP_SIZE;
	const float len = sqrtf((pts[27] - pts[0]) * (pts[27] - pts[0]) + (pts[28] - pts[1]) * (pts[28] - pts[1]) +
	                        (pts[29] - pts[2]) * (pts[29] - pts[2]));
	const float z9 = pts[29];
	for (int i = 0; i < 21; i++) {
		const float u = pts[i * 3], v = pts[i * 3 + 1], z = pts[i * 3 + 2];
		// x: -1 at the crop's left edge (+1 when mirrored), y: +1 at its top (see stereographic_project_image_sized)
		px_coord[i].pos_2d.x = instr.flip ? 1.0f - 2.0f * u / s : -1.0f + 2.0f * u / s;
		px_coord[i].pos_2d.y = 1.0f - 2.0f * v / s;
		px_coord[i].confidence_xy = hgt->tuneable_values.mediapipe_xy_conf;
		px_coord[i].depth_relative_to_midpxm = len > 1e-3f ? (z - z9) / len : 0.0f;
		px_coord[i].confidence_depth = hgt->tuneable_values.mediapipe_depth_conf;
	}
	for (size_t i = 0; i < ARRAY_SIZE(output_tensors); i++) {
		wrap.api->ReleaseValue(output_tensors[i]);
	}
	return present;
}

void
mediapipe_remember(HandTracking *hgt)
{
	for (int hand = 0; hand < 2; hand++) {
		for (int view = 0; view < 2; view++) {
			const HandTracking::mp_view_output &o = hgt->mp_outputs[hand][view];
			HandTracking::mp_previous &p = hgt->mp_prev[hand][view];
			p.valid = o.valid && o.presence >= hgt->tuneable_values.mediapipe_presence;
			if (!p.valid) {
				continue;
			}
			p.timestamp = hgt->current_frame_timestamp;
			for (int i = 0; i < 21; i++) {
				p.dirs.col(i) = mediapipe_crop_direction(o.rot, o.radius, o.landmarks[i * 3], o.landmarks[i * 3 + 1]);
			}
		}
	}
}

//! Mercury's keypoint step with MediaPipe's landmark model (hg_tuneable_values::mediapipe_keypoints).
static void
run_keypoint_estimation_mediapipe(keypoint_estimation_run_info &info, bool for_fit = true)
{
	struct HandTracking *hgt = info.view->hgt;
	const int view_idx = info.view->view;
	const int hand_idx = info.hand_idx;
	// For the gestures only (for_fit false), the 3D hand's keypoints stay Mercury's: write the view's outputs to a
	// scratch copy (Mercury's job for this view runs at the same time); mp_outputs is this job's alone.
	one_frame_one_view scratch;
	one_frame_one_view &this_output = for_fit ? hgt->keypoint_outputs[hand_idx].views[view_idx] : scratch;
	MLOutput2D &px_coord = this_output.keypoints_in_scaled_stereographic;
	t_camera_model_params dist = info.view->hgdist;
	projection_instructions instr(info.view->hgdist);
	cv::Mat crop;
	float present = 0.0f;

	// Only for tracked hands (pose-predicted regions); detection boxes go to Mercury's model.
	/*
	 * As MediaPipe's own tracking (hand_landmarker_graph: the next frame's rect from this frame's landmarks): from
	 * MediaPipe's landmarks of the last frame in this view when it saw the hand there; else from the tracked hand.
	 */
	Eigen::Array<float, 3, 21> keypoints_in_camera;
	const HandTracking::mp_previous &prev = hgt->mp_prev[hand_idx][view_idx];
	if (prev.valid && hgt->current_frame_timestamp - prev.timestamp <= 50 * U_TIME_1MS_IN_NS) {
		keypoints_in_camera = prev.dirs;
	} else if (view_idx == 0) {
		keypoints_in_camera = hgt->pose_predicted_keypoints[hand_idx];
	} else {
		for (int i = 0; i < 21; i++) {
			Eigen::Quaternionf ori = map_quat(hgt->left_in_right.orientation);
			Eigen::Vector3f tmp = hgt->pose_predicted_keypoints[hand_idx].col(i);
			tmp = ori * tmp;
			tmp += map_vec3(hgt->left_in_right.position);
			keypoints_in_camera.col(i) = tmp;
		}
	}
	/*
	 * The hand with the camera's up, then turned as MediaPipe's HandLandmarksToRectCalculator does: the wrist to the
	 * mean of the index, middle and ring knuckles (middle weighted 1/2, the others 1/4) straight up. Never mirrored:
	 * MediaPipe's model takes left and right hands as they are (no flip in its graph); Mercury's own model is the one
	 * that wants one handedness. (make_projection_instructions gives y negated: undone here.)
	 */
	hand21_2d pred;
	make_projection_instructions(dist, false, 1.0f, 0.0f, keypoints_in_camera, instr, pred);
	const xrt_vec2 knuckles = {0.25f * pred[5].pos_2d.x + 0.5f * pred[9].pos_2d.x + 0.25f * pred[13].pos_2d.x,
	                           -(0.25f * pred[5].pos_2d.y + 0.5f * pred[9].pos_2d.y + 0.25f * pred[13].pos_2d.y)};
	const float twist = upright_twist(0.0f, {pred[0].pos_2d.x, -pred[0].pos_2d.y}, knuckles);
	/*
	 * MediaPipe's own tracking crop (hand_landmarks_detector_graph: HandLandmarksToRect on the palm and the base
	 * finger joints, then RectTransformation scale 2.0, shift_y -0.1): the box around landmarks 0, 1, 2, 3, 5, 6,
	 * 9, 10, 13, 14, 17, 18 in the turned frame, twice that size, moved a tenth of its side toward the fingers. The
	 * other joints are set to the wrist so make_projection_instructions boxes only these (its palm-size floor uses
	 * 0, 5, 9, 17, kept). A box around the whole hand made the fingers much smaller in the crop: offline, pinch
	 * barely separated from an open hand (median thumb-index / palm width 0.94 vs 1.23).
	 */
	Eigen::Array<float, 3, 21> partial = keypoints_in_camera;
	for (int i : {4, 7, 8, 11, 12, 15, 16, 19, 20}) {
		partial.col(i) = keypoints_in_camera.col(0);
	}
	make_projection_instructions(dist, false, hgt->tuneable_values.mediapipe_box_fac, twist, partial, instr, pred);
	{
		// The tenth-of-the-side shift toward the fingers: up in the turned crop (+y, stereographic).
		const Eigen::Vector3f moved =
		    instr.rot_quat * stereographic_unprojection(0.0f, 0.2f * instr.stereographic_radius);
		instr.rot_quat = direction(moved, twist);
	}
	present = mediapipe_run(info, instr, px_coord, crop);

	xrt::auxiliary::math::map_quat(this_output.look_dir) = instr.rot_quat;
	this_output.stereographic_radius = instr.stereographic_radius;

	HG_TRACE(hgt, "mediapipe presence view %d hand %d: %f", view_idx, hand_idx, present);
	this_output.active = present >= hgt->tuneable_values.mediapipe_presence;
	if (!this_output.active) {
		hgt->mp_rejected_views[hand_idx]++;
	}
	this_output.hand_score = 1.0f; // MediaPipe's presence decides its views (keypoint_hand_confidence is Mercury's)

	// No finger curls from this model: a variance so large the fitting ignores them.
	for (int i = 0; i < 5; i++) {
		this_output.curls[i].value = 0.0f;
		this_output.curls[i].variance = 1e6f;
	}

	if (hgt->debug_scribble) {
		int root_x = 8 + ((2 * hand_idx) * (128 + 8));
		int root_y = 8 + (2 * view_idx * (128 + 8));
		cv::Mat small;
		cv::resize(crop, small, cv::Size(128, 128));
		small.copyTo(hgt->visualizers.mat(cv::Rect(root_x, root_y, 128, 128)));
	}

	// Development: MERCURY_MEDIAPIPE_DUMP=<dir> writes every 4th crop as a PGM (named by frame time), to check the turn
	// and box.
	static const char *dump = getenv("MERCURY_MEDIAPIPE_DUMP");
	static std::atomic<int> dumped{0};
	if (dump != nullptr) {
		const int k = dumped++;
		if (k % 4 == 0 && k < 4 * 12000) {
			char path[1024];
			// The landmarks' centre in the crop (0 = middle, crop pixels): how well the crop was placed.
			const HandTracking::mp_view_output &mo = hgt->mp_outputs[hand_idx][view_idx];
			float mx = 0, my = 0;
			for (int i = 0; i < 21; i++) {
				mx += mo.landmarks[i * 3] / 21.0f, my += mo.landmarks[i * 3 + 1] / 21.0f;
			}
			snprintf(path, sizeof(path), "%s/mp_%lld_h%d_v%d_dx%+04d_dy%+04d_p%03d.png", dump,
			         (long long)hgt->current_frame_timestamp, hand_idx, view_idx, (int)lroundf(mx - MP_SIZE / 2.0f),
			         (int)lroundf(my - MP_SIZE / 2.0f), (int)(present * 100));
			// The crop with MediaPipe's 21 landmarks: bones grey, thumb tip (4) red, index tip (8) green.
			cv::Mat vis;
			cv::cvtColor(crop, vis, crop.channels() == 1 ? cv::COLOR_GRAY2BGR : cv::COLOR_RGB2BGR);
			if (vis.depth() != CV_8U) {
				vis.convertTo(vis, CV_8U, 255.0);
			}
			static const int bones[20][2] = {{0, 1},  {1, 2},   {2, 3},   {3, 4},   {0, 5},   {5, 6},   {6, 7},
			                                 {7, 8},  {0, 9},   {9, 10},  {10, 11}, {11, 12}, {0, 13}, {13, 14},
			                                 {14, 15}, {15, 16}, {0, 17}, {17, 18}, {18, 19}, {19, 20}};
			auto lp = [&](int i) { return cv::Point2f(mo.landmarks[i * 3], mo.landmarks[i * 3 + 1]); };
			for (const auto &bn : bones) {
				cv::line(vis, lp(bn[0]), lp(bn[1]), cv::Scalar(160, 160, 160), 1);
			}
			cv::circle(vis, lp(4), 4, cv::Scalar(0, 0, 255), -1);
			cv::circle(vis, lp(8), 4, cv::Scalar(0, 255, 0), -1);
			cv::imwrite(path, vis);
		}
	}
}

static void
run_keypoint_estimation_unsafe(keypoint_estimation_run_info &info)
{
	// MediaPipe for tracked hands; a fresh detection box goes to Mercury's own model, which picks hands up from
	// those far more reliably (offline, PS VR2: MediaPipe kept 25% of detection crops vs 91% of tracked ones).
	// When MediaPipe doesn't see a hand in a tracked crop, Mercury's model gets the same frame: a hand MediaPipe
	// doubts in one frame (faint, at a camera's edge) was otherwise lost whole and slow to be found again (offline:
	// left hand 152 such losses vs 34 with Mercury's model alone).
	// With mediapipe_fit off, Mercury's model alone gives the keypoints for the 3D hand; MediaPipe runs for the
	// gestures as its own job (run_gesture_mediapipe), at the same time.
	if (info.view->hgt->tuneable_values.mediapipe_keypoints && info.view->hgt->tuneable_values.mediapipe_fit &&
	    info.view->regions_of_interest_this_frame[info.hand_idx].provenance != ROIProvenance::HAND_DETECTION) {
		run_keypoint_estimation_mediapipe(info);
		if (info.view->hgt->keypoint_outputs[info.hand_idx].views[info.view->view].active) {
			return;
		}
	}

	onnx_state *state = &info.view->keypoint[info.hand_idx];
	OnnxWrapper &wrap = *state->wrap;

	struct HandTracking *hgt = info.view->hgt;

	int view_idx = info.view->view;
	int hand_idx = info.hand_idx;
	one_frame_one_view &this_output = hgt->keypoint_outputs[hand_idx].views[view_idx];
	MLOutput2D &px_coord = this_output.keypoints_in_scaled_stereographic;
	// Factor out starting here

	hand_region_of_interest &output = info.view->regions_of_interest_this_frame[hand_idx];

	cv::Mat data_128x128_uint8;

	projection_instructions instr(info.view->hgdist);
	instr.rot_quat = Eigen::Quaternionf::Identity();
	instr.stereographic_radius = 0.4;



	t_camera_model_params dist = info.view->hgdist;


	float twist = 0;

	if (output.provenance == ROIProvenance::HAND_DETECTION) {

		xrt_vec3 center;

		xrt_vec3 edges[4];


		t_camera_models_unproject_and_flip(&hgt->views[view_idx].hgdist, output.center_px.x, output.center_px.y,
		                                   &center.x, &center.y, &center.z);

		xrt_vec2 r = {output.size_px / 2, 0};
		xrt_vec2 d = {0, output.size_px / 2};
		xrt_vec2 v;

		// note! we do not need to rotate this! it's *already* in camera space.
		int acc_idx = 0;
		v = output.center_px + r + d;
		t_camera_models_unproject_and_flip(&hgt->views[view_idx].hgdist, v.x, v.y, &edges[acc_idx].x,
		                                   &edges[acc_idx].y, &edges[acc_idx].z);
		acc_idx++;

		v = output.center_px - r + d;
		t_camera_models_unproject_and_flip(&hgt->views[view_idx].hgdist, v.x, v.y, &edges[acc_idx].x,
		                                   &edges[acc_idx].y, &edges[acc_idx].z);
		acc_idx++;

		v = output.center_px - r - d;
		t_camera_models_unproject_and_flip(&hgt->views[view_idx].hgdist, v.x, v.y, &edges[acc_idx].x,
		                                   &edges[acc_idx].y, &edges[acc_idx].z);
		acc_idx++;

		v = output.center_px + r - d;
		t_camera_models_unproject_and_flip(&hgt->views[view_idx].hgdist, v.x, v.y, &edges[acc_idx].x,
		                                   &edges[acc_idx].y, &edges[acc_idx].z);


		float angle = 0;

		for (int i = 0; i < 4; i++) {
			angle = fmaxf(angle, m_vec3_angle(center, edges[i]));
		}


		make_projection_instructions_angular(center, hand_idx, angle,
		                                     hgt->tuneable_values.after_detection_fac.val, twist, instr);

		state->wraps[2].data[0] = 0.0f;
		set_predicted_zero(state->wraps[1].data);
	} else {
		Eigen::Array<float, 3, 21> keypoints_in_camera;


		if (view_idx == 0) {
			keypoints_in_camera = hgt->pose_predicted_keypoints[hand_idx];
		} else {
			for (int i = 0; i < 21; i++) {

				Eigen::Quaternionf ori = map_quat(hgt->left_in_right.orientation);
				Eigen::Vector3f tmp = hgt->pose_predicted_keypoints[hand_idx].col(i);

				tmp = ori * tmp;
				tmp += map_vec3(hgt->left_in_right.position);
				keypoints_in_camera.col(i) = tmp;
			}
		}

		hand21_2d bleh;

		make_projection_instructions(dist, hand_idx, hgt->tuneable_values.dyn_radii_fac.val, twist,
		                             keypoints_in_camera, instr, bleh);

		if (hgt->tuneable_values.enable_pose_predicted_input) {
			for (int ml_joint_idx = 0; ml_joint_idx < 21; ml_joint_idx++) {
				float *data = state->wraps[1].data;
				data[(ml_joint_idx * 2) + 0] = bleh[ml_joint_idx].pos_2d.x;
				data[(ml_joint_idx * 2) + 1] = bleh[ml_joint_idx].pos_2d.y;
				// data[(ml_joint_idx * 2) + 2] = bleh[ml_joint_idx].depth_relative_to_midpxm;
			}


			state->wraps[2].data[0] = 1.0f;
		} else {
			state->wraps[2].data[0] = 0.0f;
			set_predicted_zero(state->wraps[1].data);
		}
	}

	stereographic_project_image(dist, instr, hgt->views[view_idx].run_model_on_this,
	                            &hgt->views[view_idx].debug_out_to_this, info.hand_idx ? RED : YELLOW,
	                            data_128x128_uint8);


	xrt::auxiliary::math::map_quat(this_output.look_dir) = instr.rot_quat;
	this_output.stereographic_radius = instr.stereographic_radius;

	bool is_hand = true;

	{
		XRT_TRACE_IDENT(convert_format);

		// here!
		cv::Mat data_128x128_float(cv::Size(128, 128), CV_32FC1, state->wraps[0].data, 128 * sizeof(float));

		is_hand = is_hand && normalizeGrayscaleImage(data_128x128_uint8, data_128x128_float);
	}


	// Ending here


	const OrtValue *inputs[] = {state->wraps[0].tensor, state->wraps[1].tensor, state->wraps[2].tensor};
	const char *input_names[] = {state->wraps[0].name, state->wraps[1].name, state->wraps[2].name};
	static_assert(ARRAY_SIZE(input_names) == ARRAY_SIZE(inputs));

	OrtValue *output_tensors[] = {nullptr, nullptr, nullptr, nullptr};
	const char *output_names[] = {"heatmap_xy", "heatmap_depth", "scalar_extras", "curls"};
	static_assert(ARRAY_SIZE(output_names) == ARRAY_SIZE(output_tensors));

	{
		XRT_TRACE_IDENT(model);
		ORT_SAFE(wrap, Run(wrap.session, nullptr, input_names, inputs, ARRAY_SIZE(input_names), output_names,
		                   ARRAY_SIZE(output_names), output_tensors));
	}

	// To here

	// Interpret model outputs!


	float *out_data = nullptr;


	ORT_SAFE(wrap, GetTensorMutableData(output_tensors[0], (void **)&out_data));

	// I don't know why this was added
	// float *confidences = info.view->keypoint_outputs.views[hand_idx].confidences;

	// This was added for debug scribbling, and should be added back at some point.
	// xrt_vec2 *keypoints_global = px_coord.kps;

	size_t plane_size = 22 * 22;

	for (int i = 0; i < 21; i++) {
		float *data = &out_data[i * plane_size];


		// This will be optimized out if nan checking is disabled in hg_numerics_checker
		for (size_t x = 0; x < plane_size; x++) {
			CHECK_NOT_NAN(data[x]);
		}


		int out_idx = argmax(data, 22 * 22);
		int row = out_idx / 22;
		int col = out_idx % 22;

		xrt_vec2 loc = {};

		// This is a good start but rethink it. Maybe fail if less than 18/21 joints failed?
		// is_hand = is_hand &&
		refine_center_of_distribution(hgt, data, col, row, 22, 22, &loc.x, &loc.y);

		if (hand_idx == 0) {
			px_coord[i].pos_2d.x = math_map_ranges(loc.x, 0, 22, -1, 1);
		} else {
			px_coord[i].pos_2d.x = math_map_ranges(loc.x, 0, 22, 1, -1);
		}

		//!@todo when you change this to have +Z-forward
		// note note note the flip!!!
		px_coord[i].pos_2d.y = math_map_ranges(loc.y, 0, 22, 1, -1);

		px_coord[i].confidence_xy = data[out_idx];
	}


	float *out_data_depth = nullptr;
	ORT_SAFE(wrap, GetTensorMutableData(output_tensors[1], (void **)&out_data_depth));

	for (int joint_idx = 0; joint_idx < 21; joint_idx++) {
		float *p_ptr = &out_data_depth[(joint_idx * 22)];


		float depth = 0;
		float confidence = 0;

		// This function can fail
		if (hand_depth_center_of_mass(hgt, p_ptr, &depth, &confidence)) {

			px_coord[joint_idx].depth_relative_to_midpxm = depth;
			px_coord[joint_idx].confidence_depth = confidence;
		} else {
			px_coord[joint_idx].depth_relative_to_midpxm = 0;
			px_coord[joint_idx].confidence_depth = 0;
		}
	}

	float *out_data_extras = nullptr;
	ORT_SAFE(wrap, GetTensorMutableData(output_tensors[2], (void **)&out_data_extras));

	float is_hand_explicit = out_data_extras[0];

	is_hand_explicit = (1.0) / (1.0 + expf(-is_hand_explicit));

	// When the model is sure, it's _really_ sure.
	// Index was fine with 0.99.
	// North Star seemed to need 0.97.
	HG_TRACE(hgt, "is_hand view %d hand %d: %f", view_idx, (int)info.hand_idx, is_hand_explicit);
	this_output.hand_score = is_hand_explicit;
	if (is_hand_explicit < hgt->tuneable_values.keypoint_hand_confidence) {
		HG_DEBUG(hgt, "Not hand! %f", is_hand_explicit);
		is_hand = false;
	}

	this_output.active = is_hand;


	float *out_data_curls = nullptr;
	ORT_SAFE(wrap, GetTensorMutableData(output_tensors[3], (void **)&out_data_curls));

	for (int i = 0; i < 5; i++) {
		float curl = out_data_curls[i];
		float variance = out_data_curls[5 + i];

		// Next two lines directly correspond to py_training/settings.py
		// We don't want it to be negative
		variance = fabsf(variance);
		// We don't want it to be possible to be zero
		variance += 0.01;

		this_output.curls[i].value = curl;
		this_output.curls[i].variance = variance;
	}



	if (hgt->debug_scribble) {
		int data_acc_idx = 0;

		int root_x = 8 + ((2 * hand_idx) * (128 + 8));
		int root_y = 8 + (2 * info.view->view * (128 + 8));

		cv::Rect p = cv::Rect(root_x, root_y, 128, 128);

		data_128x128_uint8.copyTo(hgt->visualizers.mat(p));

		make_keypoint_heatmap_output(info.view->view, hand_idx, 0, 0, out_data + (data_acc_idx * plane_size),
		                             hgt->visualizers.mat);
		make_keypoint_depth_heatmap_output(info.view->view, hand_idx, 0, 0,
		                                   out_data_depth + (data_acc_idx * 22), hgt->visualizers.mat);

		data_acc_idx++;

		for (int finger = 0; finger < 5; finger++) {
			for (int joint = 0; joint < 4; joint++) {

				make_keypoint_heatmap_output(info.view->view, hand_idx, 1 + joint, finger,
				                             out_data + (data_acc_idx * plane_size),
				                             hgt->visualizers.mat);
				make_keypoint_depth_heatmap_output(info.view->view, hand_idx, 1 + joint, finger,
				                                   out_data_depth + (data_acc_idx * 22),
				                                   hgt->visualizers.mat);
				data_acc_idx++;
			}
		}

		// Hand existence
		char amt[5];

		snprintf(amt, ARRAY_SIZE(amt), "%.2f", is_hand_explicit);

		cv::Point2i text_origin;

		text_origin.x = root_x + 128 + 2;
		text_origin.y = root_y + 60;

		// Clear out what was there before
		cv::rectangle(hgt->visualizers.mat, cv::Rect(text_origin - cv::Point2i{0, 25}, cv::Size{30, 30}), {255},
		              cv::FILLED);


		cv::putText(hgt->visualizers.mat, amt, text_origin, cv::FONT_HERSHEY_SIMPLEX, 0.3, {0, 0, 0});

		// Curls
		cv::rectangle(hgt->visualizers.mat, cv::Rect(cv::Point2i{root_x, root_y + 128 + 22}, cv::Size{128, 60}),
		              {255}, cv::FILLED);
		for (int i = 0; i < 5; i++) {
			int r = 15;

			cv::Point2i center = {root_x + r + (20 * i), root_y + 128 + 60};

			cv::circle(hgt->visualizers.mat, center, 1, {0}, 1);

			float c = this_output.curls[i].value * 0.3;
			int x = cos(c) * r;
			// Remember, OpenCV has (0,0) at top left
			int y = -sin(c) * r;

			cv::Point2i pt2 = {x, y};
			pt2 += center;
			cv::circle(hgt->visualizers.mat, pt2, 1, {0}, 1);
			cv::line(hgt->visualizers.mat, center, pt2, {0}, 1);
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(output_tensors); i++) {
		wrap.api->ReleaseValue(output_tensors[i]);
	}
}

void
run_gesture_mediapipe(void *ptr)
{
	XRT_TRACE_MARKER();
	keypoint_estimation_run_info info = *(keypoint_estimation_run_info *)ptr;
	try {
		run_keypoint_estimation_mediapipe(info, false);
	} catch (const std::exception &e) {
		HG_ERROR(info.view->hgt, "Exception in MediaPipe's gesture run: %s", e.what());
		info.view->hgt->mp_outputs[info.hand_idx][info.view->view].valid = false;
	}
}

void
run_keypoint_estimation(void *ptr)
{
	XRT_TRACE_MARKER();
	keypoint_estimation_run_info info = *(keypoint_estimation_run_info *)ptr;

	try {
		run_keypoint_estimation_unsafe(info);
	} catch (const std::exception &e) {
		HG_ERROR(info.view->hgt, "Exception while running keypoint estimation: %s", e.what());

		// Report not active if keypoint estimation hits an exception
		info.view->hgt->keypoint_outputs[info.hand_idx].views[info.view->view].active = false;
	}
}

void
release_onnx_state(onnx_state *state)
{
	if (!state->wrap) {
		return; // never loaded (MediaPipe off)
	}
	OnnxWrapper &wrap = *state->wrap;

	for (model_input_wrap &a : state->wraps) {
		wrap.api->ReleaseValue(a.tensor);
		free(a.data);
	}
}

} // namespace xrt::tracking::hand::mercury
