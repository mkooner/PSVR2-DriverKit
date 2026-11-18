// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Runs the driver's Basalt module (psvr2_basalt.c, the same code) on a recording in real time: IMU samples and camera
// transfers in arrival order, and for every SLAM pose Basalt's pose of that time as soon as Basalt has it (as the
// driver does). Writes the record with those as R lines (arrival = when they became available), for steamvr_sim
// SIM_REF. Usage: basalt_module_test record.csv camera.pv2c out.csv
#include "psvr2_basalt.h"
#include "os/os_time.h"
#include "xrt/xrt_frame.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>
struct Ev { long long t; int type; size_t idx; };
int main(int argc, char **argv) {
	if (argc < 4) { fprintf(stderr, "usage: basalt_module_test record.csv camera.pv2c out.csv\n"); return 1; }
	std::vector<std::string> lines; std::vector<uint8_t> block;
	struct Imu { long long vts; xrt_vec3 g, a; }; std::vector<Imu> imu;
	struct Slam { long long host, vts; xrt_pose p; }; std::vector<Slam> slam;
	{ std::ifstream f(argv[1]); std::string l;
	  while (std::getline(f, l)) { lines.push_back(l);
		long long h, v; float x[9]; int ty; unsigned sz; char hex[8192];
		if (l.rfind("I,", 0) == 0 && sscanf(l.c_str(), "I,%lld,%lld,%f,%f,%f,%f,%f,%f", &h, &v, &x[0], &x[1], &x[2], &x[3], &x[4], &x[5]) == 8)
			imu.push_back({v, {x[0], x[1], x[2]}, {x[3], x[4], x[5]}});
		else if (l.rfind("S,", 0) == 0 && sscanf(l.c_str(), "S,%lld,%lld,%f,%f,%f,%f,%f,%f,%f", &h, &v, &x[0], &x[1], &x[2], &x[3], &x[4], &x[5], &x[6]) == 9)
			slam.push_back({h, v, {{x[3], x[4], x[5], x[6]}, {x[0], x[1], x[2]}}});
		else if (block.empty() && l.rfind("V,", 0) == 0 && sscanf(l.c_str(), "V,%lld,%d,%u,%8191s", &h, &ty, &sz, hex) == 4 && ty == 17)
			for (unsigned i = 0; i + 1 < strlen(hex); i += 2) { unsigned b; sscanf(hex + i, "%2x", &b); block.push_back((uint8_t)b); }
	  } }
	std::sort(slam.begin(), slam.end(), [](const Slam &a, const Slam &b) { return a.vts < b.vts; });
	// Camera transfers (exposure time from the header, VTS us).
	FILE *cf = fopen(argv[2], "rb"); char mg[8]; fread(mg, 1, 8, cf);
	struct Cam { long long vts; long off; uint32_t len; }; std::vector<Cam> cams;
	for (;;) { uint64_t ns; uint32_t len, d; if (fread(&ns, 8, 1, cf) != 1 || fread(&len, 4, 1, cf) != 1 || fread(&d, 4, 1, cf) != 1) break;
		long off = ftell(cf); uint8_t h[16]; if (fread(h, 1, 16, cf) != 16) break; fseek(cf, off + len, SEEK_SET);
		if (len != 1040640) continue; uint32_t v; memcpy(&v, h + 8, 4); cams.push_back({(long long)v * 1000, off, len}); }
	fprintf(stderr, "%zu IMU, %zu SLAM, %zu camera transfers, block %zu bytes\n", imu.size(), slam.size(), cams.size(), block.size());
	struct xrt_frame_context xfctx = {};
	struct psvr2_basalt *b = psvr2_basalt_create(&xfctx, block.data(), (uint32_t)block.size());
	if (!b) { fprintf(stderr, "psvr2_basalt_create failed\n"); return 1; }
	// Arrival order: IMU ~2 ms after the sample, camera transfers ~33 ms after exposure (measured: image arrival
	// latency vs the headset's SLAM stream).
	std::vector<Ev> ev;
	for (size_t i = 0; i < imu.size(); i++) ev.push_back({imu[i].vts + 2000000, 0, i});
	for (size_t i = 0; i < cams.size(); i++) ev.push_back({cams[i].vts + 15700000, 1, i}); // measured: transfers arrive 15.7 ms after exposure
	std::stable_sort(ev.begin(), ev.end(), [](const Ev &a, const Ev &c) { return a.t < c.t; });
	std::vector<uint8_t> buf; size_t next_s = 0; std::vector<std::string> rlines; int got = 0, late = 0;
	long long t0 = ev.front().t; long long w0 = (long long)os_monotonic_get_ns();
	for (auto &e : ev) {
		long long due = w0 + (e.t - t0);
		long long now = (long long)os_monotonic_get_ns();
		if (due > now) std::this_thread::sleep_for(std::chrono::nanoseconds(due - now));
		if (e.type == 0) psvr2_basalt_push_imu(b, imu[e.idx].vts, &imu[e.idx].a, &imu[e.idx].g);
		else { buf.resize(cams[e.idx].len); fseek(cf, cams[e.idx].off, SEEK_SET); fread(buf.data(), 1, buf.size(), cf);
		       psvr2_basalt_push_transfer(b, cams[e.idx].vts, buf.data(), buf.size()); }
		// Referee: SLAM poses up to now, each as soon as Basalt has its time (give up after 150 ms).
		long long vnow = e.t - 2000000; // the data clock now
		while (next_s < slam.size() && slam[next_s].vts <= vnow) {
			xrt_pose rp;
			if (psvr2_basalt_ref_pose(b, slam[next_s].vts, &slam[next_s].p, &rp)) {
				long long arrival = slam[next_s].host + (vnow - slam[next_s].vts) - (slam[next_s].host - slam[next_s].vts - 0) * 0;
				// arrival on the record's host clock: SLAM's own arrival plus how much later Basalt had it
				long long slam_lat = 23100000; // measured: the headset's SLAM pose arrives 23.1 ms after its time
				arrival = slam[next_s].host + std::max(0LL, (vnow - slam[next_s].vts) - slam_lat);
				char l[256]; snprintf(l, sizeof l, "R,%lld,%lld,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f", arrival, slam[next_s].vts,
				                      rp.position.x, rp.position.y, rp.position.z, rp.orientation.x, rp.orientation.y, rp.orientation.z, rp.orientation.w);
				rlines.push_back(l); got++; next_s++;
			} else if (vnow - slam[next_s].vts > 150000000) { next_s++; late++; }
			else break;
		}
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(300));
	FILE *o = fopen(argv[3], "w");
	for (auto &l : lines) fprintf(o, "%s\n", l.c_str());
	for (auto &l : rlines) fprintf(o, "%s\n", l.c_str());
	fclose(o);
	fprintf(stderr, "referee poses: %d, SLAM poses without one (not started / not there in 150 ms): %d\n", got, late);
	xrt_frame_context_destroy_nodes(&xfctx);
	psvr2_basalt_destroy(&b);
	return 0;
}
