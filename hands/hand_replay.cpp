// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//
// Offline Mercury hand tracking prototype for the PS VR2: replays a PSVR2_RECORD_CAMERA recording (bottom camera pair,
// camera mode 0x10, two 1016x1016 BC4 images per transfer) through Monado's Mercury hand tracker, with the stereo
// calibration from hands/extract_stereo_calib.py, and reports per-frame detections and timing.
//
// The images are the calibrated 1016x1016 images (--crop N shifts the principal point if that turns out wrong);
// --swap-views if image 1 turns out to be the left camera.
//
// Usage: hand_replay recording.pv2c stereo.json out.csv [--crop N] [--swap-views] [--max-frames N]
//                    [--dump-pgm dir]  (write the decoded image pairs of every 30th frame, to look at them)
//                    [--rectify FOV_DEG [--rect-size N]] (unwarp each image to a pinhole view of that field of view before
//                    Mercury, and give Mercury the matching pinhole calibration; same camera axes, so the stereo
//                    extrinsics are unchanged) [--gamma G] (brighten: out = 255 * (in/255)^G)

#include "xrt/xrt_frame.h"
#include "util/u_frame.h"
#include "util/u_file.h"
#include "util/u_json.h"
#include "os/os_time.h"
#include "math/m_api.h"
#include "tracking/t_hand_tracking.h"
#include "tracking/t_tracking.h"
#include "tracking/t_camera_models.h"
#include "hg_interface.h"

#define BCDEC_BC4BC5_PRECISE
#include "bcdec.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

// Mode 0x10 transfer: 256 byte header, then two BC4 images of 1024 (stride) x 1016 rows each, back to back:
// (1040640 - 256) / 2 = 520192 = 1024 * 1016 / 2. 1016x1016 is also the calibrated image size, so the full image is
// the calibrated one; the protocol note's "1000x1000 + 24 padding" is approximate.
static constexpr int kW = 1016, kH = 1016, kStride = 1024;

static bool
load_calib(const char *path, int crop, bool swap, t_stereo_camera_calibration **out)
{
	FILE *f = fopen(path, "rb");
	if (!f)
		return false;
	std::string s;
	char buf[4096];
	size_t n;
	while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
		s.append(buf, n);
	fclose(f);
	cJSON *j = cJSON_Parse(s.c_str());
	if (!j)
		return false;
	t_stereo_camera_calibration *c = nullptr;
	t_stereo_camera_calibration_alloc(&c, T_DISTORTION_PSVR2);
	cJSON *views = cJSON_GetObjectItem(j, "views");
	for (int i = 0; i < 2; i++) {
		cJSON *v = cJSON_GetArrayItem(views, swap ? 1 - i : i);
		t_camera_calibration &cc = c->view[i];
		cc.image_size_pixels = {kW, kH};
		cc.distortion_model = T_DISTORTION_PSVR2;
		cJSON *K = cJSON_GetObjectItem(v, "K");
		for (int r = 0; r < 3; r++)
			for (int k = 0; k < 3; k++)
				cc.intrinsics[r][k] = cJSON_GetArrayItem(cJSON_GetArrayItem(K, r), k)->valuedouble;
		cc.intrinsics[0][2] -= crop; // principal point in the cropped image
		cc.intrinsics[1][2] -= crop;
		cJSON *d = cJSON_GetObjectItem(v, "dist");
		for (int k = 0; k < 20; k++)
			cc.psvr2.params[k] = cJSON_GetArrayItem(d, k)->valuedouble;
	}
	// x_right = R x_left + T (OpenCV). If the views are swapped, invert.
	double R[3][3], T[3];
	cJSON *jr = cJSON_GetObjectItem(j, "R"), *jt = cJSON_GetObjectItem(j, "T");
	for (int r = 0; r < 3; r++) {
		T[r] = cJSON_GetArrayItem(jt, r)->valuedouble;
		for (int k = 0; k < 3; k++)
			R[r][k] = cJSON_GetArrayItem(cJSON_GetArrayItem(jr, r), k)->valuedouble;
	}
	if (swap) {
		double Ri[3][3], Ti[3];
		for (int r = 0; r < 3; r++)
			for (int k = 0; k < 3; k++)
				Ri[r][k] = R[k][r];
		for (int r = 0; r < 3; r++)
			Ti[r] = -(Ri[r][0] * T[0] + Ri[r][1] * T[1] + Ri[r][2] * T[2]);
		memcpy(R, Ri, sizeof(R));
		memcpy(T, Ti, sizeof(T));
	}
	for (int r = 0; r < 3; r++) {
		c->camera_translation[r] = T[r];
		for (int k = 0; k < 3; k++)
			c->camera_rotation[r][k] = R[r][k];
	}
	cJSON_Delete(j);
	*out = c;
	return true;
}

//! Decodes the two BC4 images of a mode 0x10 transfer.
static void
decode_pair(const uint8_t *transfer, xrt_frame *out[2])
{
	const uint8_t *src = transfer + 256;
	const size_t row_blocks = kStride / 4;
	const size_t img_size = BCDEC_BC4_COMPRESSED_SIZE(kStride, kH);
	for (int i = 0; i < 2; i++) {
		xrt_frame *f = out[i];
		for (int by = 0; by < kH / 4; by++) {
			for (int bx = 0; bx < kW / 4; bx++) {
				const uint8_t *block = src + (by * row_blocks + bx) * BCDEC_BC4_BLOCK_SIZE;
				bcdec_bc4(block, f->data + (by * 4) * f->stride + bx * 4, (int)f->stride, 0);
			}
		}
		src += img_size;
	}
}

//! Pinhole unwarp of one 1016x1016 PS VR2 image: per output pixel, the source position (bilinear), -1 = outside.
struct Rectifier
{
	int n = 0;
	std::vector<float> mx, my;

	void
	init(const t_camera_calibration &src, float fov_deg, int size)
	{
		t_camera_model_params mp;
		t_camera_model_params_from_t_camera_calibration(&src, &mp);
		n = size;
		mx.assign((size_t)n * n, -1);
		my.assign((size_t)n * n, -1);
		float f = (n / 2.0f) / tanf(fov_deg * (float)M_PI / 360.0f);
		for (int y = 0; y < n; y++)
			for (int x = 0; x < n; x++) {
				float sx, sy;
				if (t_camera_models_project(&mp, (x - n / 2.0f) / f, (y - n / 2.0f) / f, 1.0f, &sx, &sy) &&
				    sx >= 0 && sy >= 0 && sx < kW - 1 && sy < kH - 1) {
					mx[(size_t)y * n + x] = sx;
					my[(size_t)y * n + x] = sy;
				}
			}
	}

	void
	apply(const xrt_frame *in, xrt_frame *out, const uint8_t lut[256]) const
	{
		for (int y = 0; y < n; y++)
			for (int x = 0; x < n; x++) {
				size_t i = (size_t)y * n + x;
				uint8_t v = 0;
				if (mx[i] >= 0) {
					int x0 = (int)mx[i], y0 = (int)my[i];
					float fx = mx[i] - x0, fy = my[i] - y0;
					const uint8_t *p = in->data + (size_t)y0 * in->stride + x0;
					size_t st = in->stride;
					v = (uint8_t)((p[0] * (1 - fx) + p[1] * fx) * (1 - fy) + (p[st] * (1 - fx) + p[st + 1] * fx) * fy +
					              0.5f);
				}
				out->data[(size_t)y * out->stride + x] = lut[v];
			}
	}
};

static void
write_pgm(const std::string &path, const xrt_frame *f)
{
	FILE *o = fopen(path.c_str(), "wb");
	if (!o)
		return;
	fprintf(o, "P5\n%u %u\n255\n", f->width, f->height);
	for (uint32_t y = 0; y < f->height; y++)
		fwrite(f->data + y * f->stride, 1, f->width, o);
	fclose(o);
}

int
main(int argc, char **argv)
{
	if (argc < 4) {
		fprintf(stderr, "usage: %s recording.pv2c stereo.json out.csv [--crop N] [--swap-views] [--max-frames N] "
		                "[--dump-pgm dir]\n",
		        argv[0]);
		return 1;
	}
	int crop = 0, max_frames = -1;
	bool swap = false;
	const char *dump_dir = nullptr;
	float rect_fov = 0, gamma = 1;
	int rect_size = 640, start_frame = 0, index = 0, overlay_every = 15;
	const char *overlay_dir = nullptr;
	std::vector<int> overlay_list; // --overlay-frames a,b,c: only these frame indices
	for (int i = 4; i < argc; i++) {
		if (!strcmp(argv[i], "--crop") && i + 1 < argc)
			crop = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--swap-views"))
			swap = true;
		else if (!strcmp(argv[i], "--max-frames") && i + 1 < argc)
			max_frames = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--dump-pgm") && i + 1 < argc)
			dump_dir = argv[++i];
		else if (!strcmp(argv[i], "--rectify") && i + 1 < argc)
			rect_fov = atof(argv[++i]);
		else if (!strcmp(argv[i], "--rect-size") && i + 1 < argc)
			rect_size = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--overlay") && i + 1 < argc)
			overlay_dir = argv[++i];
		else if (!strcmp(argv[i], "--overlay-frames") && i + 1 < argc) {
			for (char *t = strtok(argv[++i], ","); t; t = strtok(nullptr, ","))
				overlay_list.push_back(atoi(t));
		} else if (!strcmp(argv[i], "--overlay-every") && i + 1 < argc)
			overlay_every = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--start-frame") && i + 1 < argc)
			start_frame = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--gamma") && i + 1 < argc)
			gamma = atof(argv[++i]);
	}

	t_stereo_camera_calibration *calib = nullptr;
	if (!load_calib(argv[2], crop, swap, &calib)) {
		fprintf(stderr, "can't read %s\n", argv[2]);
		return 1;
	}
	uint8_t lut[256];
	for (int i = 0; i < 256; i++)
		lut[i] = (uint8_t)std::lround(255.0 * std::pow(i / 255.0, (double)gamma));
	Rectifier rect[2];
	int out_w = kW, out_h = kH;
	if (rect_fov > 0) {
		out_w = out_h = rect_size;
		float f = (rect_size / 2.0f) / tanf(rect_fov * (float)M_PI / 360.0f);
		for (int i = 0; i < 2; i++) {
			rect[i].init(calib->view[i], rect_fov, rect_size);
			// What Mercury sees now: a distortion-free pinhole camera (RT8 with zero coefficients).
			t_camera_calibration &cc = calib->view[i];
			cc.image_size_pixels = {(uint32_t)rect_size, (uint32_t)rect_size};
			cc.distortion_model = T_DISTORTION_OPENCV_RADTAN_8;
			memset(cc.distortion_parameters_as_array, 0, sizeof(cc.distortion_parameters_as_array));
			memset(cc.intrinsics, 0, sizeof(cc.intrinsics));
			cc.intrinsics[0][0] = cc.intrinsics[1][1] = f;
			cc.intrinsics[0][2] = cc.intrinsics[1][2] = rect_size / 2.0;
			cc.intrinsics[2][2] = 1;
		}
		printf("rectifying to a %.0f deg %dx%d pinhole view, gamma %.2f\n", rect_fov, rect_size, rect_size, gamma);
	}
	char models[1024];
	if (u_file_get_hand_tracking_models_dir(models, sizeof(models)) < 0) {
		fprintf(stderr, "no hand-tracking models directory\n");
		return 1;
	}
	t_hand_tracking_create_info info = {};
	for (int i = 0; i < 2; i++) {
		// The unwarped pinhole view is filled edge to edge (at fields of view inside the lens), so no vignette.
		info.cams_info.views[i].boundary_type = rect_fov > 0 ? HT_IMAGE_BOUNDARY_NONE : HT_IMAGE_BOUNDARY_CIRCLE;
		info.cams_info.views[i].boundary.circle.normalized_center = {0.5f, 0.5f};
		info.cams_info.views[i].boundary.circle.normalized_radius = 0.55f; // PSVR2HT's value for these cameras
		info.cams_info.views[i].camera_orientation = CAMERA_ORIENTATION_0;
	}
	t_hand_tracking_sync *ht = t_hand_tracking_sync_mercury_create(calib, info, models);
	if (!ht) {
		fprintf(stderr, "Mercury failed to start\n");
		return 1;
	}

	FILE *in = fopen(argv[1], "rb");
	char magic[4];
	uint32_t version = 0;
	if (!in || fread(magic, 1, 4, in) != 4 || memcmp(magic, "PV2C", 4) || fread(&version, 4, 1, in) != 1) {
		fprintf(stderr, "%s is not a PSVR2_RECORD_CAMERA file\n", argv[1]);
		return 1;
	}
	FILE *csv = fopen(argv[3], "w");
	fprintf(csv, "host_ns,frame_counter,dropped_before,process_ms,left_active,lx,ly,lz,right_active,rx,ry,rz,mean_level,lpinch_mm,rpinch_mm\n");

	std::vector<uint8_t> transfer;
	int frames = 0, left = 0, right = 0, skipped = 0;
	double total_ms = 0, max_ms = 0;
	uint64_t first_ns = 0, last_ns = 0;
	for (;;) {
		uint64_t host_ns;
		uint32_t length, dropped;
		if (fread(&host_ns, 8, 1, in) != 1 || fread(&length, 4, 1, in) != 1 || fread(&dropped, 4, 1, in) != 1)
			break;
		transfer.resize(length);
		if (fread(transfer.data(), 1, length, in) != length)
			break;
		if (length != 1040640) { // only mode 0x10 BC4 bottom pairs; the recorder keeps every transfer size
			skipped++;
			continue;
		}
		if (index++ < start_frame)
			continue;
		if (max_frames >= 0 && frames >= max_frames)
			break;

		xrt_frame *views[2] = {nullptr, nullptr};
		for (int i = 0; i < 2; i++) {
			u_frame_create_one_off(XRT_FORMAT_L8, kW, kH, &views[i]);
			views[i]->timestamp = (int64_t)host_ns;
		}
		xrt_frame *ordered[2] = {views[0], views[1]};
		decode_pair(transfer.data(), ordered);
		if (swap) {
			std::swap(views[0], views[1]);
		}
		if (rect_fov > 0 || gamma != 1) {
			for (int i = 0; i < 2; i++) {
				xrt_frame *o = nullptr;
				u_frame_create_one_off(XRT_FORMAT_L8, out_w, out_h, &o);
				o->timestamp = views[i]->timestamp;
				if (rect_fov > 0) {
					rect[i].apply(views[i], o, lut);
				} else {
					for (int y = 0; y < kH; y++)
						for (int x = 0; x < kW; x++)
							o->data[y * o->stride + x] = lut[views[i]->data[y * views[i]->stride + x]];
				}
				xrt_frame_reference(&views[i], nullptr);
				views[i] = o;
			}
		}
		double level = 0;
		for (int y = 0; y < out_h; y += 8)
			for (int x = 0; x < out_w; x += 8)
				level += views[0]->data[y * views[0]->stride + x];
		level /= ((out_w + 7) / 8) * ((out_h + 7) / 8);
		if (dump_dir && frames % 30 == 0) {
			write_pgm(std::string(dump_dir) + "/f" + std::to_string(frames) + "-L.pgm", views[0]);
			write_pgm(std::string(dump_dir) + "/f" + std::to_string(frames) + "-R.pgm", views[1]);
		}

		xrt_hand_joint_set l = {}, r = {};
		int64_t ts = 0;
		auto t0 = std::chrono::steady_clock::now();
		if (getenv("HAND_REPLAY_MARK_FRAMES"))
			fprintf(stderr, "FRAME %d\n", index - 1);
		t_ht_sync_process(ht, views[0], views[1], &l, &r, &ts);
		double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
		total_ms += ms;
		max_ms = std::fmax(max_ms, ms);
		if (overlay_dir && rect_fov > 0 && (overlay_list.empty() ? frames % overlay_every == 0
		                                                        : std::find(overlay_list.begin(), overlay_list.end(),
		                                                                    index - 1) != overlay_list.end()) && (l.is_active || r.is_active)) {
			// Project both hands' joints (camera 0, OpenXR axes) into both unwarped views: left hand red, right
			// green; side by side PPM.
			int n = out_w;
			std::vector<uint8_t> img((size_t)n * 2 * n * 3);
			for (int v = 0; v < 2; v++)
				for (int y = 0; y < n; y++)
					for (int x = 0; x < n; x++) {
						uint8_t g = views[v]->data[y * views[v]->stride + x];
						uint8_t *px = &img[((size_t)y * 2 * n + v * n + x) * 3];
						px[0] = px[1] = px[2] = g;
					}
			const t_stereo_camera_calibration *c = calib;
			for (int h = 0; h < 2; h++) {
				const xrt_hand_joint_set &js = h ? r : l;
				if (!js.is_active)
					continue;
				for (int jn = 0; jn < XRT_HAND_JOINT_COUNT; jn++) {
					xrt_pose wp;
					math_pose_transform(&js.hand_pose.pose,
					                    &js.values.hand_joint_set_default[jn].relation.pose, &wp);
					double X0[3] = {wp.position.x, -wp.position.y, -wp.position.z}; // OpenCV axes
					for (int v = 0; v < 2; v++) {
						double X[3];
						if (v == 0) {
							memcpy(X, X0, sizeof(X));
						} else {
							for (int a = 0; a < 3; a++)
								X[a] = c->camera_rotation[a][0] * X0[0] + c->camera_rotation[a][1] * X0[1] +
								       c->camera_rotation[a][2] * X0[2] + c->camera_translation[a];
						}
						if (X[2] <= 0.01)
							continue;
						const t_camera_calibration &cc = c->view[v];
						int u = (int)lround(cc.intrinsics[0][0] * X[0] / X[2] + cc.intrinsics[0][2]);
						int w = (int)lround(cc.intrinsics[1][1] * X[1] / X[2] + cc.intrinsics[1][2]);
						for (int dy = -3; dy <= 3; dy++)
							for (int dx = -3; dx <= 3; dx++) {
								int xx = u + dx, yy = w + dy;
								if (xx < 0 || yy < 0 || xx >= n || yy >= n)
									continue;
								uint8_t *px = &img[((size_t)yy * 2 * n + v * n + xx) * 3];
								px[0] = h ? 0 : 255;
								px[1] = h ? 255 : 0;
								px[2] = 0;
							}
					}
				}
			}
			std::string path = std::string(overlay_dir) + "/o" + std::to_string(index - 1) + ".ppm";
			FILE *o = fopen(path.c_str(), "wb");
			fprintf(o, "P6\n%d %d\n255\n", 2 * n, n);
			fwrite(img.data(), 1, img.size(), o);
			fclose(o);
		}
		xrt_vec3 lw = l.values.hand_joint_set_default[XRT_HAND_JOINT_WRIST].relation.pose.position;
		xrt_vec3 rw = r.values.hand_joint_set_default[XRT_HAND_JOINT_WRIST].relation.pose.position;
		// Header bytes 12-13: the headset's frame counter (u16), 1 per transfer; 8-11 a microsecond clock.
		unsigned counter = transfer[12] | (transfer[13] << 8);
		auto pinch_mm = [](const xrt_hand_joint_set &js) {
			if (!js.is_active)
				return -1.0;
			xrt_vec3 a = js.values.hand_joint_set_default[XRT_HAND_JOINT_THUMB_TIP].relation.pose.position;
			xrt_vec3 b = js.values.hand_joint_set_default[XRT_HAND_JOINT_INDEX_TIP].relation.pose.position;
			return 1000.0 * std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
		};
		fprintf(csv, "%llu,%u,%u,%.2f,%d,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%.1f,%.1f,%.1f\n", (unsigned long long)host_ns, counter, dropped,
		        ms, (int)l.is_active, lw.x, lw.y, lw.z, (int)r.is_active, rw.x, rw.y, rw.z, level, pinch_mm(l), pinch_mm(r));
		left += l.is_active;
		right += r.is_active;
		if (!first_ns)
			first_ns = host_ns;
		last_ns = host_ns;
		frames++;
		xrt_frame_reference(&views[0], nullptr);
		xrt_frame_reference(&views[1], nullptr);
	}
	fclose(in);
	fclose(csv);
	double secs = (last_ns - first_ns) * 1e-9;
	printf("%d frames over %.1f s (%.1f /s); left hand found in %.0f%%, right in %.0f%%; Mercury %.1f ms mean, %.1f "
	       "max per frame\n",
	       frames, secs, secs > 0 ? frames / secs : 0, frames ? 100.0 * left / frames : 0,
	       frames ? 100.0 * right / frames : 0, frames ? total_ms / frames : 0, max_ms);
	if (skipped)
		printf("%d transfers of other sizes skipped (not mode 0x10 BC4)\n", skipped);
	t_ht_sync_destroy(&ht);
	t_stereo_camera_calibration_reference(&calib, nullptr);
	return 0;
}
