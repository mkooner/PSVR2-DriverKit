// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Replays a recorded run through the driver's own room capture code (ovrd_room.cpp, OvrdRoomCapture) with Create
// held all the time (the worst case): every FINAL left-controller pose from the combined log, the head from the
// record's SLAM poses, SLAM resets detected as psvr2.c does. Raw frame = logged poses + the 1.6 m tracking origin
// offset, as the SteamVR driver sees them.
//
// Usage: capture_replay <combined.log> <record.csv> <outline.csv> [hand L|R]
#include "ovrd_room.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

struct Head
{
	int64_t t;
	xrt_vec3 p;
};

int
main(int argc, char **argv)
{
	if (argc < 4) {
		fprintf(stderr, "usage: %s combined.log record.csv outline.csv [L|R]\n", argv[0]);
		return 1;
	}
	const char hand = argc > 4 ? argv[4][0] : 'L';
	const float offset_y = 1.6f; // PSVR2_FLOOR_HEIGHT default, the tracking origin's offset

	std::vector<xrt_vec3> outline;
	{
		std::ifstream f(argv[3]);
		std::string line;
		while (std::getline(f, line)) {
			char h;
			float x, y, z;
			if (sscanf(line.c_str(), "%c,%f,%f,%f", &h, &x, &y, &z) == 4 && h == hand) {
				outline.push_back(xrt_vec3{x, y, z});
			}
		}
	}

	std::vector<Head> heads;
	std::vector<int64_t> resets;
	{
		std::ifstream f(argv[2]);
		std::string line;
		xrt_vec3 prev = {0, 0, 0};
		bool have_prev = false;
		while (std::getline(f, line)) {
			long long t, vts;
			float x, y, z;
			if (line.rfind("S,", 0) == 0 && sscanf(line.c_str(), "S,%lld,%lld,%f,%f,%f", &t, &vts, &x, &y, &z) == 5) {
				float now_len = sqrtf(x * x + y * y + z * z);
				float old_len = sqrtf(prev.x * prev.x + prev.y * prev.y + prev.z * prev.z);
				if (have_prev && now_len < 0.001f && old_len > 0.005f) {
					resets.push_back(t);
				}
				prev = xrt_vec3{x, y, z};
				have_prev = true;
				heads.push_back(Head{t, xrt_vec3{x, y + offset_y, z}});
			}
		}
	}

	OvrdRoomCapture capture;
	std::ifstream f(argv[1]);
	std::string line;
	size_t hi = 0, ri = 0;
	int64_t last_reset = 0, first_t = 0;
	int captured = 0, rejected = 0, samples = 0;
	std::map<std::string, int> restarts;
	bool release = false;
	while (std::getline(f, line)) {
		const char *m = strstr(line.c_str(), "FINAL ");
		if (m == nullptr) {
			continue;
		}
		long long t;
		int h;
		unsigned flags;
		float px, py, pz, qx, qy, qz, qw;
		if (sscanf(m, "FINAL %lld %d flags %x pos %f %f %f rot %f %f %f %f", &t, &h, &flags, &px, &py, &pz, &qx,
		           &qy, &qz, &qw) != 10 ||
		    h != (hand == 'L' ? 0 : 1)) {
			continue;
		}
		if (first_t == 0) {
			first_t = t;
		}
		samples++;
		while (hi + 1 < heads.size() && heads[hi + 1].t <= t) {
			hi++;
		}
		while (ri < resets.size() && resets[ri] <= t) {
			last_reset = resets[ri++];
		}
		xrt_space_relation grip = {};
		grip.relation_flags = (enum xrt_space_relation_flags)(flags & ~XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
		grip.pose.position = xrt_vec3{px, py + offset_y, pz};
		grip.pose.orientation = xrt_quat{qx, qy, qz, qw};
		const xrt_vec3 *head = heads.empty() ? nullptr : &heads[hi].p;

		std::string log;
		OvrdRoomCapture::Result r = capture.feed(!release, grip, outline.data(), outline.size(), 0.006f, head,
		                                         last_reset, nullptr, t, &log);
		release = false;
		for (size_t p = 0; p < log.size();) {
			size_t nl = log.find('\n', p);
			std::string msg = log.substr(p, nl - p);
			if (msg.rfind("capture restarted", 0) == 0 || msg.rfind("capture cancelled", 0) == 0) {
				restarts[msg]++;
			} else {
				printf("  %7.1f s: %s\n", (t - first_t) / 1e9, msg.c_str());
			}
			p = nl == std::string::npos ? log.size() : nl + 1;
		}
		if (r == OvrdRoomCapture::Result::CAPTURED) {
			ovrd_room_yaw_pose c = capture.captured();
			printf("  %7.1f s: CAPTURED floor %.3f m below the head, at %.3f %.3f, heading %.1f deg (axis %d)\n",
			       (t - first_t) / 1e9, capture.captured_below_head(), c.position.x, c.position.z,
			       c.yaw * 57.29578f, (int)capture.captured_axis());
			captured++;
			release = true;
		} else if (r == OvrdRoomCapture::Result::REJECTED) {
			rejected++;
			release = true;
		}
	}
	printf("  %d %c samples, %zu SLAM resets; %d captured, %d rejected; restarts:", samples, hand, resets.size(),
	       captured, rejected);
	for (auto &kv : restarts) {
		printf(" [%s] x%d", kv.first.c_str(), kv.second);
	}
	printf("\n");
	return 0;
}
