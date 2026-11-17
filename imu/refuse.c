// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*
 * Re-run a recorded Sense IMU (sense-*-L/R.csv from PSSENSE_RECORD) through the driver's 3DoF fusion
 * (m_imu_3dof, as pssense_driver.c sets it up), optionally with pssense_imu_rest.h (gyro bias + initial leveling),
 * and write the same CSV with the new orientation, for ct_replay.
 *
 *   refuse <in.csv> <out.csv> <mode: 0 as before, 1 rest module, 2 rest + no 3dof gravity, 3 rest + 300 ms gravity>
 */
#include "math/m_imu_3dof.h"
#include "pssense/pssense_imu_rest.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

int
main(int argc, char **argv)
{
	if (argc < 4) {
		fprintf(stderr, "usage: %s in.csv out.csv use_rest\n", argv[0]);
		return 1;
	}
	FILE *in = fopen(argv[1], "r"), *out = fopen(argv[2], "w");
	if (!in || !out) {
		perror("open");
		return 1;
	}
	// 0: driver as before; 1: + rest module; 2: rest module, m_imu_3dof's own gravity correction off;
	// 3: rest module, gravity correction 300 ms instead of 20 ms
	int mode = atoi(argv[3]);
	int use_rest = mode >= 1;
	struct m_imu_3dof f;
	m_imu_3dof_init(&f, mode == 2 ? 0 : mode == 3 ? M_IMU_3DOF_USE_GRAVITY_DUR_300MS : M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
	struct pssense_imu_rest rest = {0};
	// GYRO_MATRIX=m00,m01,...,m22: test a gyro cross-axis correction (applied to the calibrated rate).
	float gm[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	const char *gms = getenv("GYRO_MATRIX");
	if (gms)
		sscanf(gms, "%f,%f,%f,%f,%f,%f,%f,%f,%f", &gm[0], &gm[1], &gm[2], &gm[3], &gm[4], &gm[5], &gm[6], &gm[7],
		       &gm[8]);
	char line[4096];
	long rows = 0;
	int64_t level_ns = -1, first_ns = -1;
	while (fgets(line, sizeof(line), in)) {
		if (line[0] < '0' || line[0] > '9') {
			fputs(line, out);
			continue;
		}
		int64_t host_ns, imu_ns;
		double v[10];
		if (sscanf(line, "%" SCNd64 ",%" SCNd64 ",%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &host_ns, &imu_ns, &v[0],
		           &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8], &v[9]) != 12) {
			fputs(line, out);
			continue;
		}
		if (first_ns < 0)
			first_ns = imu_ns;
		struct xrt_vec3 gyro = {gm[0] * (float)v[0] + gm[1] * (float)v[1] + gm[2] * (float)v[2],
		                        gm[3] * (float)v[0] + gm[4] * (float)v[1] + gm[5] * (float)v[2],
		                        gm[6] * (float)v[0] + gm[7] * (float)v[1] + gm[8] * (float)v[2]};
		struct xrt_vec3 accel = {(float)v[3], (float)v[4], (float)v[5]};
		if (use_rest) {
			bool was = rest.leveled;
			pssense_imu_rest_push(&rest, &f, imu_ns, &gyro, &accel);
			if (!was && rest.leveled)
				level_ns = imu_ns;
		}
		m_imu_3dof_update(&f, (uint64_t)imu_ns, &accel, &gyro);
		fprintf(out, "%" PRId64 ",%" PRId64 ",%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.7f,%.7f,%.7f,%.7f\n", host_ns, imu_ns,
		        gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, f.rot.x, f.rot.y, f.rot.z, f.rot.w);
		rows++;
	}
	fprintf(stderr, "%ld rows; rest windows %u; bias %.3f %.3f %.3f deg/s; leveled at %.2f s\n", rows,
	        rest.rest_windows, rest.bias.x * 57.29578f, rest.bias.y * 57.29578f, rest.bias.z * 57.29578f,
	        level_ns < 0 ? -1.0 : (level_ns - first_ns) / 1e9);
	m_imu_3dof_close(&f);
	fclose(out);
	return 0;
}
