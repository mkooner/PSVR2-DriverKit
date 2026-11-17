// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//
// Unwarps recorded PS VR2 bottom camera images (PGM, 1016x1016, T_DISTORTION_PSVR2) into a plain pinhole view with
// the calibration from extract_stereo_calib.py, to test whether Mercury's detector does better on rectified images
// (PSVR2HT rectifies before Mercury).
// Usage: rectify_probe stereo.json view(0|1) fov_deg pitch_deg out_size in.pgm out.pgm [in.pgm out.pgm ...]
//   pitch_deg tilts the virtual camera down (+) about its x axis.
#include "util/u_json.h"
#include "tracking/t_tracking.h"
#include "tracking/t_camera_models.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

static bool
rd(const char *p, int &w, int &h, std::vector<uint8_t> &px)
{
	std::ifstream f(p, std::ios::binary);
	std::string m;
	int mx;
	if (!(f >> m >> w >> h >> mx))
		return false;
	f.get();
	px.resize((size_t)w * h);
	f.read((char *)px.data(), px.size());
	return true;
}

int
main(int argc, char **argv)
{
	if (argc < 8) {
		fprintf(stderr, "usage: %s stereo.json view fov pitch size in.pgm out.pgm...\n", argv[0]);
		return 1;
	}
	std::ifstream jf(argv[1]);
	std::string js((std::istreambuf_iterator<char>(jf)), {});
	cJSON *j = cJSON_Parse(js.c_str());
	cJSON *v = cJSON_GetArrayItem(cJSON_GetObjectItem(j, "views"), atoi(argv[2]));
	t_camera_calibration cc = {};
	cc.image_size_pixels = {1016, 1016};
	cc.distortion_model = T_DISTORTION_PSVR2;
	cJSON *K = cJSON_GetObjectItem(v, "K");
	for (int r = 0; r < 3; r++)
		for (int k = 0; k < 3; k++)
			cc.intrinsics[r][k] = cJSON_GetArrayItem(cJSON_GetArrayItem(K, r), k)->valuedouble;
	cJSON *d = cJSON_GetObjectItem(v, "dist");
	for (int k = 0; k < 20; k++)
		cc.psvr2.params[k] = cJSON_GetArrayItem(d, k)->valuedouble;
	t_camera_model_params mp;
	t_camera_model_params_from_t_camera_calibration(&cc, &mp);

	float fov = atof(argv[3]) * M_PI / 180, pitch = atof(argv[4]) * M_PI / 180;
	int N = atoi(argv[5]);
	float f = (N / 2.0f) / tanf(fov / 2);
	// Map: output pixel -> ray (OpenCV axes: x right, y down, z forward), tilted down by pitch -> source pixel.
	std::vector<float> mapx((size_t)N * N), mapy((size_t)N * N);
	float cp = cosf(pitch), sp = sinf(pitch);
	for (int y = 0; y < N; y++)
		for (int x = 0; x < N; x++) {
			float rx = (x - N / 2.0f) / f, ry = (y - N / 2.0f) / f, rz = 1;
			float ry2 = ry * cp + rz * sp, rz2 = -ry * sp + rz * cp; // looking further down for pitch > 0
			float sx = -1, sy = -1;
			if (!t_camera_models_project(&mp, rx, ry2, rz2, &sx, &sy))
				sx = sy = -1;
			mapx[(size_t)y * N + x] = sx;
			mapy[(size_t)y * N + x] = sy;
		}
	for (int a = 6; a + 1 < argc; a += 2) {
		int w, h;
		std::vector<uint8_t> in;
		if (!rd(argv[a], w, h, in))
			continue;
		std::vector<uint8_t> out((size_t)N * N, 0);
		for (size_t i = 0; i < out.size(); i++) {
			float sx = mapx[i], sy = mapy[i];
			if (sx < 0 || sy < 0 || sx >= w - 1 || sy >= h - 1)
				continue;
			int x0 = (int)sx, y0 = (int)sy;
			float fx = sx - x0, fy = sy - y0;
			const uint8_t *p = &in[(size_t)y0 * w + x0];
			out[i] = (uint8_t)((p[0] * (1 - fx) + p[1] * fx) * (1 - fy) + (p[w] * (1 - fx) + p[w + 1] * fx) * fy + 0.5f);
		}
		FILE *o = fopen(argv[a + 1], "wb");
		fprintf(o, "P5\n%d %d\n255\n", N, N);
		fwrite(out.data(), 1, out.size(), o);
		fclose(o);
	}
	return 0;
}
