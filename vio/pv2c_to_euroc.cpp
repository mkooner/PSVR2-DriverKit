// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// PS VR2 recording -> EuRoC dataset for Basalt / monado-cli slambatch.
// Camera: mode 0x10 transfers (camera-*.pv2c): 256 B header (u32@8 VTS us), two BC4 1024-stride x 1016 images
// (image 0 = left = cam0, image 1 = cam1). IMU: record-*.csv I lines (VTS ns, gyro rad/s, accel m/s^2, driver head
// axes), with the duplicate-VTS fix of psvr2_imu_time.h. Timestamps: VTS ns (shared clock).
// Usage: pv2c_to_euroc camera.pv2c record.csv out_dir [max_frames]
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>
#include <cmath>

static void bc4_decode(const uint8_t *src, int w, int h, int stride, cv::Mat &out) {
	out.create(h, w, CV_8UC1);
	for (int by = 0; by < h / 4; by++)
		for (int bx = 0; bx < stride / 4; bx++) {
			const uint8_t *b = src + (by * (stride / 4) + bx) * 8;
			int a0 = b[0], a1 = b[1], pal[8] = {a0, a1};
			if (a0 > a1) for (int i = 1; i < 7; i++) pal[1 + i] = ((7 - i) * a0 + i * a1) / 7;
			else { for (int i = 1; i < 5; i++) pal[1 + i] = ((5 - i) * a0 + i * a1) / 5; pal[6] = 0; pal[7] = 255; }
			uint64_t bits = 0; for (int i = 0; i < 6; i++) bits |= (uint64_t)b[2 + i] << (8 * i);
			for (int p = 0; p < 16; p++) {
				int x = bx * 4 + (p & 3), y = by * 4 + (p >> 2);
				if (x < w) out.at<uint8_t>(y, x) = (uint8_t)pal[(bits >> (3 * p)) & 7];
			}
		}
}

int main(int argc, char **argv) {
	if (argc < 4) { fprintf(stderr, "usage: pv2c_to_euroc camera.pv2c record.csv out_dir [max_frames]\n"); return 1; }
	std::string out = argv[3]; long max_frames = argc > 4 ? atol(argv[4]) : -1;
	// PV2C_GAMMA=g: brighten dark images (out = 255 * (in/255)^g, g < 1) before writing.
	double gamma = getenv("PV2C_GAMMA") ? atof(getenv("PV2C_GAMMA")) : 1.0;
	// PV2C_SKIP=n: leave out the first n camera transfers (e.g. while the headset is being put on).
	long skip = getenv("PV2C_SKIP") ? atol(getenv("PV2C_SKIP")) : 0;
	cv::Mat lut(1, 256, CV_8U);
	for (int i = 0; i < 256; i++) lut.at<uint8_t>(i) = (uint8_t)std::lround(255.0 * std::pow(i / 255.0, gamma));
	namespace fs = std::filesystem;
	for (const char *d : {"/mav0/cam0/data", "/mav0/cam1/data", "/mav0/imu0"}) fs::create_directories(out + d);
	// Pass 1: camera timestamps (the IMU is written only from 20 ms before the first frame: Basalt's bounded IMU
	// queues fill while it waits for the first frame and the pipeline deadlocks).
	long long cam_first = 0, cam_last = 0;
	{
		FILE *f1 = fopen(argv[1], "rb"); char h1[8]; fread(h1, 1, 8, f1);
		std::vector<uint8_t> b(256); long n1 = 0; uint32_t lv = 0; long long hi = 0; bool any = false;
		while (max_frames < 0 || n1 < max_frames) {
			uint64_t ns; uint32_t len, dropped;
			if (fread(&ns, 8, 1, f1) != 1 || fread(&len, 4, 1, f1) != 1 || fread(&dropped, 4, 1, f1) != 1) break;
			n1++;
			if (len < 256 || fread(b.data(), 1, 256, f1) != 256) break;
			fseek(f1, len - 256, SEEK_CUR);
			if (len != 1040640) continue;
			if (n1 <= skip) continue;
			uint32_t v; memcpy(&v, &b[8], 4);
			if (any && v < lv) hi += (1LL << 32);
			lv = v; long long ts = (hi + v) * 1000LL;
			if (!any) cam_first = ts;
			cam_last = ts; any = true;
		}
		fclose(f1);
	}
	const long long imu_from = cam_first - 20000000LL, imu_to = cam_last + 20000000LL;
	// IMU
	std::ifstream rf(argv[2]); std::string l;
	FILE *imu = fopen((out + "/mav0/imu0/data.csv").c_str(), "w");
	fprintf(imu, "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n");
	bool have = false; long long last_raw = 0, last_out = 0; long n_imu = 0, n_dup = 0;
	long long imu_first = 0, imu_last = 0;
	while (std::getline(rf, l)) {
		if (l.rfind("I,", 0) != 0) continue;
		long long host, raw; double g[3], a[3];
		if (sscanf(l.c_str(), "I,%lld,%lld,%lf,%lf,%lf,%lf,%lf,%lf", &host, &raw, &g[0], &g[1], &g[2], &a[0], &a[1], &a[2]) != 8) continue;
		long long t = raw;
		if (have && raw == last_raw) { t = last_out + 500000; n_dup++; }
		if (have && t <= last_out) continue; // out of order: drop
		have = true; last_raw = raw; last_out = t;
		if (t < imu_from || t > imu_to) continue;
		fprintf(imu, "%lld,%.9f,%.9f,%.9f,%.7f,%.7f,%.7f\n", t, g[0], g[1], g[2], a[0], a[1], a[2]);
		if (!imu_first) imu_first = t; imu_last = t; n_imu++;
	}
	fclose(imu);
	// Cameras
	FILE *f = fopen(argv[1], "rb"); char hdr[8]; fread(hdr, 1, 8, f);
	FILE *c0 = fopen((out + "/mav0/cam0/data.csv").c_str(), "w"), *c1 = fopen((out + "/mav0/cam1/data.csv").c_str(), "w");
	fprintf(c0, "#timestamp [ns],filename\n"); fprintf(c1, "#timestamp [ns],filename\n");
	std::vector<uint8_t> buf; long n = 0, kept = 0; long long last_ts = 0;
	uint32_t last_vts = 0; long long vts_hi = 0;
	while (max_frames < 0 || n < max_frames) {
		uint64_t ns; uint32_t len, dropped;
		if (fread(&ns, 8, 1, f) != 1 || fread(&len, 4, 1, f) != 1 || fread(&dropped, 4, 1, f) != 1) break;
		buf.resize(len); if (fread(buf.data(), 1, len, f) != len) break;
		n++;
		if (len != 1040640 || n <= skip) continue;
		uint32_t vts_us; memcpy(&vts_us, &buf[8], 4);
		if (kept && vts_us < last_vts) vts_hi += (1LL << 32); // 32-bit us wraps every 71 min
		last_vts = vts_us;
		long long ts = (vts_hi + vts_us) * 1000LL;
		if (ts <= last_ts || ts < imu_first || ts > imu_last) continue;
		last_ts = ts;
		for (int c = 0; c < 2; c++) {
			cv::Mat m; bc4_decode(&buf[256 + c * 520192], 1016, 1016, 1024, m);
			if (gamma != 1.0) cv::LUT(m, lut, m);
			std::string name = std::to_string(ts) + ".png";
			cv::imwrite(out + "/mav0/cam" + std::to_string(c) + "/data/" + name, m, {cv::IMWRITE_PNG_COMPRESSION, 1});
			fprintf(c ? c1 : c0, "%lld,%s\n", ts, name.c_str());
		}
		kept++;
	}
	fclose(f); fclose(c0); fclose(c1);
	fprintf(stderr, "IMU %ld samples (%ld duplicate VTS spread to 500 us), %.1f s; frames %ld written of %ld read\n", n_imu, n_dup,
	        (imu_last - imu_first) * 1e-9, kept, n);
	return 0;
}
