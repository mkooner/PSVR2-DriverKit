// Standalone harness: runs the PS VR2 driver's camera-chain code (copied from
// psvr2_parse_gen_calibration_block, psvr2_constellation_tracking.c @ 4d525e6bb) with Monado's own math library on a
// recorded calibration block, and prints each camera's pose_in_origin. Variant flags change one convention at a time.
//
// usage: chain <block.bin> <mount_transpose 0/1> <relation_transpose 0/1, 2 = F O^T F, F t> <invert_rule 0=from>to 1=never 2=always>
//              <cv_flip 0=none 1=Rx180 2=Ry180>
#include "math/m_api.h"
#include "math/m_mathinclude.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

int
main(int argc, char **argv)
{
	FILE *f = fopen(argv[1], "rb");
	uint8_t data[2048];
	if (!f || fread(data, 1, sizeof(data), f) != sizeof(data)) {
		return 1;
	}
	fclose(f);
	int mount_transpose = atoi(argv[2]), rel_transpose = atoi(argv[3]), invert_rule = atoi(argv[4]),
	    cv_flip = atoi(argv[5]);

	uint8_t ndist = data[15], nrel = data[16];
	const uint8_t *rel = data + 24 + ndist * 228;

	struct xrt_matrix_4x4 poses[4] = {0};
	int parent_ids[4] = {-1, 0, 0, 0};

	struct xrt_matrix_3x3 r = {0};
	memcpy(r.v, data + 0x6f0, 36);
	if (mount_transpose) {
		math_matrix_3x3_transpose(&r, &r);
	}
	struct xrt_quat q_r, q_pitch15, q_mount, q_flip;
	math_quat_from_matrix_3x3(&r, &q_r);
	math_quat_normalize(&q_r);
	math_quat_from_angle_vector((float)(-15.0 * M_PI / 180.0), &(struct xrt_vec3){1, 0, 0}, &q_pitch15);
	math_quat_rotate(&q_pitch15, &q_r, &q_mount);
	if (cv_flip) {
		math_quat_from_angle_vector((float)M_PI, cv_flip == 1 ? &(struct xrt_vec3){1, 0, 0} : &(struct xrt_vec3){0, 1, 0},
		                            &q_flip);
		math_quat_rotate(&q_mount, &q_flip, &q_mount);
	}
	struct xrt_matrix_3x3 m3;
	math_matrix_3x3_from_quat(&q_mount, &m3);
	math_matrix_4x4_isometry_from_rt(&m3, &(struct xrt_vec3){0, 0, 0}, &poses[0]);

	struct xrt_matrix_4x4 scratch;
	for (int i = 0; i < nrel; i++) {
		const uint8_t *o = rel + i * 52;
		uint8_t to = o[0], from = o[1];
		struct xrt_matrix_3x3 orientation;
		memcpy(orientation.v, o + 4, 36);
		struct xrt_vec3 translation;
		memcpy(&translation, o + 40, 12);
		if (rel_transpose) {
			math_matrix_3x3_transpose(&orientation, &orientation);
		}
		if (rel_transpose == 2) { // the driver's convention since chain_unify.py: F O^T F, F t
			orientation.v[1] = -orientation.v[1];
			orientation.v[2] = -orientation.v[2];
			orientation.v[3] = -orientation.v[3];
			orientation.v[6] = -orientation.v[6];
			translation.y = -translation.y;
			translation.z = -translation.z;
		}
		math_matrix_4x4_isometry_from_rt(&orientation, &translation, &scratch);
		int invert = invert_rule == 0 ? (from > to) : (invert_rule == 2);
		if (invert) {
			// as built: store at the larger id
			int child = from > to ? from : to, par = from > to ? to : from;
			math_matrix_4x4_inverse(&scratch, &poses[child]);
			parent_ids[child] = par;
		} else {
			int child = from > to ? from : to, par = from > to ? to : from;
			if (invert_rule == 0) { // as built: the non-inverted relation is stored at `to`
				child = to;
				par = from;
			}
			memcpy(&poses[child], &scratch, sizeof(scratch));
			parent_ids[child] = par;
		}
	}
	for (int i = 0; i < 4; i++) {
		struct xrt_matrix_4x4 result = poses[i];
		int p = parent_ids[i], guard = 0;
		while (p != -1 && guard++ < 8) {
			math_matrix_4x4_multiply(&poses[p], &result, &result);
			p = parent_ids[p];
		}
		struct xrt_pose pose;
		math_pose_from_isometry(&result, &pose);
		printf("%d %f %f %f %f %f %f %f\n", i, pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
		       pose.orientation.y, pose.orientation.z, pose.orientation.w);
	}
	return 0;
}
