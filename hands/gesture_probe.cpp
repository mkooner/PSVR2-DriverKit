// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//
// Offline check of the SteamVR plugin's hand gestures: runs the plugin's own gesture code (ovrn_hand_gestures.hpp,
// unchanged) on the joints driver_ht_replay dumps (HT_REPLAY_JOINTS=<csv>), so pinch / grab values can be compared
// with their thresholds without a headset. Distances and bends don't depend on the frame, so the head-frame joints
// give the same values as the palm-frame ones the plugin uses.
// Build: see build_gesture_probe.sh. Usage: gesture_probe joints.csv > gestures.csv
// Output per active hand query: t_s,hand,pinch_distance_m,pinch,grab,curl0..curl4,finger_pinch0..3,trigger,grip
#include "ovrn_hand_gestures.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace ovrn::skel;

int
main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "usage: %s joints.csv\n", argv[0]);
		return 1;
	}
	FILE *f = fopen(argv[1], "r");
	if (f == nullptr) {
		perror(argv[1]);
		return 1;
	}
	std::vector<char> line(1 << 16);
	bool trigger[2] = {false, false}, grip[2] = {false, false};
	printf("t,hand,pinch_d,pinch,grab,curl0,curl1,curl2,curl3,curl4,fp0,fp1,fp2,fp3,trigger,grip\n");
	while (fgets(line.data(), (int)line.size(), f) != nullptr) {
		std::vector<double> v;
		for (char *p = strtok(line.data(), ","); p != nullptr; p = strtok(nullptr, ",")) {
			v.push_back(atof(p));
		}
		if (v.size() < 3 + kJointCount * 8 || v[2] == 0) {
			continue;
		}
		const int hand = (int)v[1];
		Joints j;
		float radius[kJointCount];
		for (int k = 0; k < kJointCount; k++) {
			const double *q = &v[3 + k * 8];
			j.pos[k] = detail::V{(float)q[0], (float)q[1], (float)q[2]};
			j.rot[k] = detail::Q{(float)q[6], (float)q[3], (float)q[4], (float)q[5]};
			j.valid[k] = true;
			radius[k] = (float)q[7];
		}
		const Gestures g = gestures(j, radius);
		trigger[hand] = click(g.pinch, trigger[hand]);
		grip[hand] = click_grab(g.grab, grip[hand]);
		printf("%.4f,%d,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d\n", v[0], hand,
		       g.pinch_distance_m, g.pinch, g.grab, g.curl[0], g.curl[1], g.curl[2], g.curl[3], g.curl[4],
		       g.finger_pinch[0], g.finger_pinch[1], g.finger_pinch[2], g.finger_pinch[3], trigger[hand] ? 1 : 0,
		       grip[hand] ? 1 : 0);
	}
	fclose(f);
	return 0;
}
