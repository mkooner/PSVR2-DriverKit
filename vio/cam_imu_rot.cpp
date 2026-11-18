// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Camera-to-IMU rotation from data (hand-eye): camera rotation between frames (LK features -> kb4 bearings ->
// essential matrix) vs integrated gyro over the same interval; R_imu_cam solved by Kabsch on rotation vectors, for
// each of the two images in the mode 0x10 transfer, with a scan of the camera time offset.
// Usage: cam_imu_rot camera.pv2c record.csv cams.json-intrinsics(fx fy cx cy k1 k2 k3 k4 for img0) (same for img1)
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/geometry.hpp>
#include <opencv2/features.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
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

struct KB4 { double fx, fy, cx, cy, k[4]; };
static bool kb4_unproject(const KB4 &c, double u, double v, Eigen::Vector3d &out) {
	double mx = (u - c.cx) / c.fx, my = (v - c.cy) / c.fy, ru = std::hypot(mx, my);
	double th = ru; // invert d(th) = ru by Newton
	for (int i = 0; i < 20; i++) {
		double t2 = th * th, d = th * (1 + t2 * (c.k[0] + t2 * (c.k[1] + t2 * (c.k[2] + t2 * c.k[3]))));
		double dd = 1 + t2 * (3 * c.k[0] + t2 * (5 * c.k[1] + t2 * (7 * c.k[2] + t2 * 9 * c.k[3])));
		th -= (d - ru) / dd;
	}
	if (!(th >= 0 && th < 1.4)) return false;
	double s = ru > 1e-12 ? std::sin(th) / ru : 1;
	out = Eigen::Vector3d(mx * s, my * s, std::cos(th));
	return true;
}

int main(int argc, char **argv) {
	if (argc < 19) { fprintf(stderr, "usage: cam_imu_rot camera.pv2c record.csv fx fy cx cy k1 k2 k3 k4 (img0) fx fy cx cy k1 k2 k3 k4 (img1) [max_frames]\n"); return 1; }
	KB4 cam[2];
	for (int c = 0; c < 2; c++) {
		double *v[8] = {&cam[c].fx, &cam[c].fy, &cam[c].cx, &cam[c].cy, &cam[c].k[0], &cam[c].k[1], &cam[c].k[2], &cam[c].k[3]};
		for (int i = 0; i < 8; i++) *v[i] = atof(argv[3 + c * 8 + i]);
	}
	int max_frames = argc > 19 ? atoi(argv[19]) : 1800;
	// IMU: I,host,vts_ns,gx,gy,gz,... (driver head axes); drop duplicate vts (keep the first)
	std::vector<double> it; std::vector<Eigen::Vector3d> ig;
	{ std::ifstream f(argv[2]); std::string l;
	  while (std::getline(f, l)) if (l.rfind("I,", 0) == 0) {
		long long host, vts; double g[3];
		if (sscanf(l.c_str(), "I,%lld,%lld,%lf,%lf,%lf", &host, &vts, &g[0], &g[1], &g[2]) == 5) {
			double t = vts * 1e-9; if (!it.empty() && t <= it.back()) continue;
			it.push_back(t); ig.emplace_back(g[0], g[1], g[2]); } } }
	fprintf(stderr, "IMU %zu samples %.1f..%.1f s\n", it.size(), it.front(), it.back());
	auto gyro_rot = [&](double t0, double t1) {  // body-frame rotation increment
		Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
		size_t i = std::lower_bound(it.begin(), it.end(), t0) - it.begin();
		double t = t0;
		for (; i < it.size() && t < t1; i++) {
			double tn = std::min(it[i], t1); double dt = tn - t; if (dt <= 0) continue;
			Eigen::Vector3d w = ig[i] * dt; double a = w.norm();
			if (a > 1e-12) q = q * Eigen::Quaterniond(Eigen::AngleAxisd(a, w / a));
			t = tn;
		}
		return q;
	};
	// Images
	FILE *f = fopen(argv[1], "rb"); char hdr[8]; fread(hdr, 1, 8, f);
	std::vector<double> ft; std::vector<cv::Mat> imgs[2];
	std::vector<uint8_t> buf;
	int step = 2;
	for (int n = 0; n < max_frames; n++) {
		uint64_t ns; uint32_t len, dropped;
		if (fread(&ns, 8, 1, f) != 1 || fread(&len, 4, 1, f) != 1 || fread(&dropped, 4, 1, f) != 1) break;
		buf.resize(len); if (fread(buf.data(), 1, len, f) != len) break;
		if (len != 1040640 || n % step) continue;
		uint32_t vts_us; memcpy(&vts_us, &buf[8], 4);
		ft.push_back(vts_us * 1e-6);
		for (int c = 0; c < 2; c++) { cv::Mat m; bc4_decode(&buf[256 + c * 520192], 1016, 1016, 1024, m); imgs[c].push_back(m); }
	}
	fclose(f);
	fprintf(stderr, "frames %zu (every %d), %.1f..%.1f s\n", ft.size(), step, ft.front(), ft.back());
	// Camera rotation increments per image
	struct Pair { double t0, t1; Eigen::Vector3d rc; int inl; };
	std::vector<Pair> pairs[2];
	cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
	for (int c = 0; c < 2; c++) {
		long st_pairs = 0, st_feat = 0, st_few = 0, st_good = 0, st_track = 0, st_e = 0, st_inl = 0, st_lowinl = 0;
		for (size_t k = 0; k + 1 < ft.size(); k++) {
			cv::Mat a, b; clahe->apply(imgs[c][k], a); clahe->apply(imgs[c][k + 1], b);
			std::vector<cv::Point2f> p0, p1; cv::goodFeaturesToTrack(a, p0, 400, 0.01, 12);
			st_pairs++; st_feat += p0.size();
			if (p0.size() < 30) { st_few++; continue; }
			std::vector<uchar> st; std::vector<float> er;
			cv::calcOpticalFlowPyrLK(a, b, p0, p1, st, er, cv::Size(21, 21), 3);
			std::vector<cv::Point2f> back; std::vector<uchar> st2;
			cv::calcOpticalFlowPyrLK(b, a, p1, back, st2, er, cv::Size(21, 21), 3);
			std::vector<cv::Point2d> n0, n1;
			for (size_t i = 0; i < p0.size(); i++) {
				if (!st[i] || !st2[i] || cv::norm(back[i] - p0[i]) > 0.5) continue;
				Eigen::Vector3d r0, r1;
				if (!kb4_unproject(cam[c], p0[i].x, p0[i].y, r0) || !kb4_unproject(cam[c], p1[i].x, p1[i].y, r1)) continue;
				if (r0.z() < 0.35 || r1.z() < 0.35) continue; // keep within ~70 deg for the planar essential solve
				n0.emplace_back(r0.x() / r0.z(), r0.y() / r0.z()); n1.emplace_back(r1.x() / r1.z(), r1.y() / r1.z());
			}
			st_good += n0.size();
			if (n0.size() < 25) { st_track++; continue; }
			// Rotation-only model (head motion over 33 ms is almost pure rotation): RANSAC on 2-ray Kabsch, then
			// refine on the inliers. Bearings back to unit vectors.
			std::vector<Eigen::Vector3d> b0, b1;
			for (size_t i = 0; i < n0.size(); i++) {
				b0.push_back(Eigen::Vector3d(n0[i].x, n0[i].y, 1).normalized());
				b1.push_back(Eigen::Vector3d(n1[i].x, n1[i].y, 1).normalized());
			}
			auto kabsch = [&](const std::vector<int> &idx) {
				Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
				for (int i : idx) H += b0[i] * b1[i].transpose();
				Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
				Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
				D(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant() > 0 ? 1 : -1;
				return Eigen::Matrix3d(svd.matrixV() * D * svd.matrixU().transpose()); // b1 = R b0
			};
			const double thr = std::cos(0.4 * M_PI / 180);
			std::vector<int> best_inl; cv::RNG rng(1234 + k);
			for (int iter = 0; iter < 200; iter++) {
				int i = rng.uniform(0, (int)b0.size()), j = rng.uniform(0, (int)b0.size());
				if (i == j || b0[i].dot(b0[j]) > 0.98) continue;
				Eigen::Matrix3d R = kabsch({i, j});
				std::vector<int> inl;
				for (int m = 0; m < (int)b0.size(); m++) if ((R * b0[m]).dot(b1[m]) > thr) inl.push_back(m);
				if (inl.size() > best_inl.size()) best_inl = inl;
			}
			st_inl += best_inl.size();
			if ((int)best_inl.size() < 20 || best_inl.size() < 0.5 * b0.size()) { st_lowinl++; continue; }
			Eigen::Matrix3d Rk = kabsch(best_inl);
			// b1 = Rk b0: points in frame k+1 axes; the camera's pose increment is Rk^T.
			Eigen::AngleAxisd aa(Rk.transpose());
			int inl = (int)best_inl.size();
			pairs[c].push_back({ft[k], ft[k + 1], aa.axis() * aa.angle(), inl});
		}
		fprintf(stderr, "image %d: %zu frame pairs with a rotation; pairs %ld, features/pair %.0f, <30 features %ld, tracked+in-view/pair %.0f, <25 tracked %ld, no E %ld, inliers/pair %.0f, <20 inliers %ld\n",
		        c, pairs[c].size(), st_pairs, st_feat / (double)std::max(1L, st_pairs), st_few, st_good / (double)std::max(1L, st_pairs), st_track, st_e, st_inl / (double)std::max(1L, st_pairs), st_lowinl);
	}
	Eigen::Matrix3d Rsol[2];
	const double min_rot = getenv("CAM_IMU_MIN_ROT_DEG") ? atof(getenv("CAM_IMU_MIN_ROT_DEG")) * M_PI / 180 : 0.03;
	for (int c = 0; c < 2; c++) {
		double best = 1e9, best_off = 0; Eigen::Matrix3d bestR;
		for (double off_ms = -40; off_ms <= 40; off_ms += 1) {
			Eigen::Matrix3d H = Eigen::Matrix3d::Zero(); std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> v;
			for (auto &p : pairs[c]) {
				if (p.rc.norm() < min_rot) continue; // enough rotation to see the axis, enough to see the axis
				Eigen::Quaterniond qi = gyro_rot(p.t0 + off_ms * 1e-3, p.t1 + off_ms * 1e-3);
				Eigen::AngleAxisd ai(qi); Eigen::Vector3d ri = ai.axis() * ai.angle();
				v.emplace_back(ri, p.rc); H += p.rc * ri.transpose();
			}
			if (v.size() < 20) continue;
			// a_imu = R_ic a_cam  ->  Kabsch: maximise sum a_imu . R a_cam
			Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
			Eigen::Matrix3d D = Eigen::Matrix3d::Identity(); D(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant() > 0 ? 1 : -1;
			Eigen::Matrix3d R = svd.matrixV() * D * svd.matrixU().transpose();
			double err = 0; for (auto &e : v) err += (e.first - R * e.second).squaredNorm();
			err = std::sqrt(err / v.size());
			if (err < best) { best = err; best_off = off_ms; bestR = R; }
		}
		Rsol[c] = bestR;
		Eigen::Quaterniond q(bestR);
		int n = 0; for (auto &p : pairs[c]) n += p.rc.norm() >= min_rot;
		printf("image %d: %d rotations used; camera time offset %+.0f ms; residual %.3f deg per pair; R_imu_cam q(xyzw) %.6f %.6f %.6f %.6f\n", c, n, best_off,
		       best * 180 / M_PI, q.x(), q.y(), q.z(), q.w());
		Eigen::Vector3d fwd = bestR * Eigen::Vector3d(0, 0, 1);
		printf("   camera optical axis in IMU frame (x right, y up, z back): %.3f %.3f %.3f -> pitch %.1f deg, yaw %.1f deg\n", fwd.x(), fwd.y(), fwd.z(),
		       std::asin(fwd.y()) * 180 / M_PI, std::atan2(-fwd.x(), -fwd.z()) * 180 / M_PI);
	}
	if (getenv("CAM_IMU_ROT_OUT")) {
		FILE *o = fopen(getenv("CAM_IMU_ROT_OUT"), "w");
		for (int c = 0; c < 2; c++) for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) fprintf(o, "%.9f%c", Rsol[c](i, j), (c == 1 && i == 2 && j == 2) ? '\n' : ' ');
		fclose(o);
	}
	Eigen::AngleAxisd rel(Rsol[0].transpose() * Rsol[1]);
	printf("rotation between image 0 and image 1 cameras: %.2f deg (calibration cam0-cam1: 30.19 deg)\n", rel.angle() * 180 / M_PI);
	return 0;
}
