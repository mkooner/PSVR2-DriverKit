// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//
// Runs Mercury's hand detection model on PGM images with Mercury's own preprocessing (letterbox to 160x160, then
// normalise to std 0.25 / mean 0.5, as hg_model.cpp blackbar() + normalizeGrayscaleImage()) and prints the raw
// per-hand confidences. Variants: as is, gamma 0.5 (brightened), and mirrored, to see what the model reacts to.
// Usage: detect_probe model.onnx image.pgm...
#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <fstream>
#include <string>

static cv::Mat
read_pgm(const char *p)
{
	std::ifstream f(p, std::ios::binary);
	std::string m;
	int w, h, mx;
	f >> m >> w >> h >> mx;
	f.get();
	cv::Mat img(h, w, CV_8UC1);
	f.read((char *)img.data, w * h);
	return img;
}

static void
run(Ort::Session &s, const cv::Mat &in, const char *tag, const char *name)
{
	const int N = 160;
	float sc = std::min((float)N / in.cols, (float)N / in.rows);
	cv::Matx23f go(sc, 0, (N - in.cols * sc) / 2, 0, sc, (N - in.rows * sc) / 2);
	cv::Mat small, f;
	cv::warpAffine(in, small, go, cv::Size(N, N));
	small.convertTo(f, CV_32FC1, 1 / 255.0);
	cv::Scalar mean, sd;
	cv::meanStdDev(f, mean, sd);
	f *= 0.25 / sd[0];
	cv::meanStdDev(f, mean, sd);
	f += 0.5 - mean[0];
	Ort::MemoryInfo mi = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
	int64_t dims[4] = {1, 1, N, N};
	Ort::Value t = Ort::Value::CreateTensor<float>(mi, (float *)f.data, N * N, dims, 4);
	const char *in_names[] = {"inputImg"}, *out_names[] = {"hand_exists", "cx", "cy", "size"};
	auto o = s.Run(Ort::RunOptions{nullptr}, in_names, &t, 1, out_names, 4);
	float *e = o[0].GetTensorMutableData<float>(), *cx = o[1].GetTensorMutableData<float>(),
	      *cy = o[2].GetTensorMutableData<float>(), *sz = o[3].GetTensorMutableData<float>();
	printf("%-28s %-8s left %.3f (%.2f,%.2f s%.2f)  right %.3f (%.2f,%.2f s%.2f)\n", name, tag, e[0], cx[0], cy[0],
	       sz[0], e[1], cx[1], cy[1], sz[1]);
}

int
main(int argc, char **argv)
{
	Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "probe");
	Ort::SessionOptions so;
	Ort::Session s(env, argv[1], so);
	for (int i = 2; i < argc; i++) {
		cv::Mat img = read_pgm(argv[i]), g, m;
		const char *name = strrchr(argv[i], '/') ? strrchr(argv[i], '/') + 1 : argv[i];
		run(s, img, "as-is", name);
		img.convertTo(g, CV_32F, 1 / 255.0);
		cv::pow(g, 0.5, g);
		g.convertTo(g, CV_8U, 255);
		run(s, g, "gamma.5", name);
		for (double gm : {0.35, 0.65}) {
			cv::Mat g2;
			img.convertTo(g2, CV_32F, 1 / 255.0);
			cv::pow(g2, gm, g2);
			g2.convertTo(g2, CV_8U, 255);
			run(s, g2, gm < 0.5 ? "gamma.35" : "gamma.65", name);
		}
		cv::flip(img, m, 1);
		run(s, m, "mirror", name);
		cv::flip(g, m, 1);
		run(s, m, "mirror+g.5", name);
		// Zoomed tiles, square, 60% of the width, in the lower middle where hands are; brightened.
		for (double gm : {0.5, 0.35}) {
			cv::Mat b;
			img.convertTo(b, CV_32F, 1 / 255.0);
			cv::pow(b, gm, b);
			b.convertTo(b, CV_8U, 255);
			int t = img.cols * 6 / 10, y0 = img.rows * 3 / 10;
			if (y0 + t > img.rows)
				y0 = img.rows - t;
			run(s, b(cv::Rect(0, y0, t, t)).clone(), gm == 0.5 ? "tileL+g.5" : "tileL+g.35", name);
			run(s, b(cv::Rect(img.cols - t, y0, t, t)).clone(), gm == 0.5 ? "tileR+g.5" : "tileR+g.35", name);
		}
		cv::rotate(img, m, cv::ROTATE_180);
		run(s, m, "rot180", name);
		cv::rotate(img, m, cv::ROTATE_90_CLOCKWISE);
		run(s, m, "rot90", name);
		cv::Mat crop = img(cv::Rect(img.cols * 158 / 1016, img.rows * 100 / 1016, img.cols * 700 / 1016, img.rows * 700 / 1016)).clone(); // the middle, where the hands are
		run(s, crop, "crop", name);
		cv::Mat c2;
		crop.convertTo(c2, CV_32F, 1 / 255.0);
		cv::pow(c2, 0.5, c2);
		c2.convertTo(c2, CV_8U, 255);
		run(s, c2, "crop+g.5", name);
		cv::Mat clipped = cv::min(img, 80); // clip the bright monitors so they don't set the contrast
		run(s, clipped, "clip80", name);
		cv::Mat cc = cv::min(crop, 80);
		run(s, cc, "crop+clip", name);
		cv::Mat eq;
		cv::Ptr<cv::CLAHE> cl = cv::createCLAHE(3.0, cv::Size(8, 8));
		cl->apply(img, eq);
		run(s, eq, "clahe", name);
		cl->apply(crop, eq);
		run(s, eq, "crop+clahe", name);
		cv::Mat c3 = img(cv::Rect(img.cols * 208 / 1016, img.rows * 150 / 1016, img.cols * 600 / 1016, img.rows * 600 / 1016)).clone();
		c3.convertTo(c3, CV_32F, 1 / 255.0);
		cv::pow(c3, 0.5, c3);
		c3.convertTo(c3, CV_8U, 255);
		run(s, c3, "crop600+g.5", name);
	}
}
