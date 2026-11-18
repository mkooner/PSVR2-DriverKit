// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Runs the driver's own camera 0 mount code (psvr2_cam_mount.h) on a recorded calibration block and compares it with
// the mount measured from motion, quaternions only (no matrix storage-order assumptions).
// Usage: mount_check <block.bin> qx qy qz qw   (measured mount: OpenXR camera axes in the head frame)
#include "psvr2_cam_mount.h"
#include <stdio.h>
#include <stdlib.h>
static double qang(struct xrt_quat a, struct xrt_quat b) {
	double d = fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w); return 2 * acos(fmin(1, d)) * 180 / M_PI;
}
static void show(const char *name, struct xrt_quat q, struct xrt_quat m) {
	float p, y; psvr2_cam_mount_view_angles(&q, &p, &y);
	printf("%-48s %6.2f deg from measured; looks %4.1f %s, %4.1f %s\n", name, qang(q, m), fabs(p), p < 0 ? "down" : "up  ",
	       fabs(y), y > 0 ? "left" : "right");
}
int main(int argc, char **argv) {
	FILE *f = fopen(argv[1], "rb"); static uint8_t b[4096]; size_t n = fread(b, 1, sizeof b, f); fclose(f);
	struct xrt_quat m = {atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5])}; math_quat_normalize(&m);
	show("measured from motion", m, m);
	struct xrt_quat q;
	psvr2_cam0_mount_from_block(b, (uint32_t)n, -15.0f, true, &q); show("-15 tilt, transposed (old driver, LEGACY=1)", q, m);
	psvr2_cam0_mount_from_block(b, (uint32_t)n, -15.0f, false, &q); show("-15 tilt, as stored (driver now)", q, m);
	psvr2_cam0_mount_from_block(b, (uint32_t)n, 15.0f, true, &q); show("+15 tilt, transposed", q, m);
	psvr2_cam0_mount_from_block(b, (uint32_t)n, 15.0f, false, &q); show("+15 tilt, as stored", q, m);
	psvr2_cam0_mount_from_angles(-29, 17, 0, &q); show("fallback estimate (-29, 17, 0)", q, m);
	struct xrt_quat rec = {-0.0003f, 0.1276f, -0.0348f, 0.9912f}; show("recorded by the tracker (19:12 dataset)", rec, m);
}
