// Cost of what the offline player adds (PNG read+decode) vs what the live driver does (BC4 decode), per frame pair.
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>
static void bc4(const uint8_t *src, cv::Mat &out) {
	out.create(1016, 1016, CV_8UC1);
	for (int by = 0; by < 254; by++) for (int bx = 0; bx < 256; bx++) {
		const uint8_t *b = src + (by * 256 + bx) * 8; int a0 = b[0], a1 = b[1], pal[8] = {a0, a1};
		if (a0 > a1) for (int i = 1; i < 7; i++) pal[1 + i] = ((7 - i) * a0 + i * a1) / 7;
		else { for (int i = 1; i < 5; i++) pal[1 + i] = ((5 - i) * a0 + i * a1) / 5; pal[6] = 0; pal[7] = 255; }
		uint64_t bits = 0; for (int i = 0; i < 6; i++) bits |= (uint64_t)b[2 + i] << (8 * i);
		for (int p = 0; p < 16; p++) { int x = bx * 4 + (p & 3), y = by * 4 + (p >> 2); if (x < 1016) out.at<uint8_t>(y, x) = pal[(bits >> (3 * p)) & 7]; }
	}
}
int main(int argc, char **argv) {
	std::ifstream csv(std::string(argv[1]) + "/mav0/cam0/data.csv"); std::string l; std::getline(csv, l);
	std::vector<std::string> names; while (std::getline(csv, l) && names.size() < 300) names.push_back(l.substr(l.find(',') + 1));
	auto t0 = std::chrono::steady_clock::now();
	for (auto &n : names) for (int c = 0; c < 2; c++) cv::Mat m = cv::imread(std::string(argv[1]) + "/mav0/cam" + std::to_string(c) + "/data/" + n, cv::IMREAD_GRAYSCALE);
	double png = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / names.size();
	FILE *f = fopen(argv[2], "rb"); char h[8]; fread(h, 1, 8, f); std::vector<uint8_t> buf; int n = 0; double tb = 0;
	while (n < 300) { uint64_t ns; uint32_t len, d; if (fread(&ns, 8, 1, f) != 1) break; fread(&len, 4, 1, f); fread(&d, 4, 1, f); buf.resize(len); fread(buf.data(), 1, len, f);
		auto a = std::chrono::steady_clock::now(); cv::Mat m0, m1; bc4(&buf[256], m0); bc4(&buf[256 + 520192], m1); tb += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count(); n++; }
	printf("per frame pair: PNG read+decode %.2f ms (offline player only), BC4 decode %.2f ms (live driver)\n", png, tb / n);
}
