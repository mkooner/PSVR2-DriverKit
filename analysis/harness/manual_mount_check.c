// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// The driver's PSVR2_CAM0_MOUNT path (psvr2_constellation_tracking.c: q = yaw * pitch * roll, then * Rx(180) for the
// OpenCV -> OpenXR camera axes), with Monado's math library, vs the mount measured from motion.
// Usage: manual_mount_check pitch yaw roll r00..r22
#include "math/m_api.h"
#include <stdio.h>
#include <stdlib.h>
int main(int argc, char **argv) {
	float p = atof(argv[1]), y = atof(argv[2]), r = atof(argv[3]); double M[9]; for (int i = 0; i < 9; i++) M[i] = atof(argv[4 + i]);
	struct xrt_quat q_yaw, q_pitch, q_roll, q_mount, q_cv;
	math_quat_from_angle_vector(DEG_TO_RAD(y), &(struct xrt_vec3){0, 1, 0}, &q_yaw);
	math_quat_from_angle_vector(DEG_TO_RAD(p), &(struct xrt_vec3){1, 0, 0}, &q_pitch);
	math_quat_from_angle_vector(DEG_TO_RAD(r), &(struct xrt_vec3){0, 0, 1}, &q_roll);
	math_quat_rotate(&q_yaw, &q_pitch, &q_mount); math_quat_rotate(&q_mount, &q_roll, &q_mount);
	math_quat_from_angle_vector((float)M_PI, &(struct xrt_vec3){1, 0, 0}, &q_cv);
	math_quat_rotate(&q_mount, &q_cv, &q_mount);
	struct xrt_matrix_3x3 R; math_matrix_3x3_from_quat(&q_mount, &R);
	double C[9] = {R.v[0], -R.v[1], -R.v[2], R.v[3], -R.v[4], -R.v[5], R.v[6], -R.v[7], -R.v[8]};
	double t = 0; for (int i = 0; i < 9; i++) t += C[i] * M[i];
	printf("PSVR2_CAM0_MOUNT=\"%g %g %g\": %.2f deg from measured; optical axis pitch %+.1f yaw %+.1f\n", p, y, r,
	       acos(fmin(1, fmax(-1, (t - 1) / 2))) * 180 / M_PI, asin(C[5]) * 180 / M_PI, atan2(-C[2], -C[8]) * 180 / M_PI);
}
