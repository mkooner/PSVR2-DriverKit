// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Prints the Sense controllers' LED outline in the grip frame, the same steps as pssense_get_grip_outline
// (pssense_driver.c), from the driver's own LED model header. Output: hand,x,y,z per LED.
#include "math/m_api.h"
#include "xrt/xrt_tracking.h"
#include "tracking/t_constellation.h"
#include "../../../trial/src/xrt/drivers/pssense/pssense_led_model.h"
#include <stdio.h>

static struct xrt_quat qx(float a) { return (struct xrt_quat){sinf(a * 0.5f), 0, 0, cosf(a * 0.5f)}; }

int main(void)
{
	for (int hand = 0; hand < 2; hand++) {
		float sx = hand == 0 ? 1.0f : -1.0f;
		const struct t_constellation_tracker_led *leds = hand == 0 ? pssense_left_leds : pssense_right_leds;
		size_t n = hand == 0 ? ARRAY_SIZE(pssense_left_leds) : ARRAY_SIZE(pssense_right_leds);
		struct xrt_pose T_led_imu = {qx(DEG_TO_RAD(PSSENSE_IMU_ANGLE_DEG_DEFAULT)), hand == 0 ? T_led_imu_left : T_led_imu_right};
		struct xrt_pose T_imu_led;
		math_pose_invert(&T_led_imu, &T_imu_led);
		struct xrt_pose model = {qx(DEG_TO_RAD(-39)), {-sx * .01432f, .007713f, .10399f}};
		struct xrt_pose xrgrip = {qx(DEG_TO_RAD(20.6)), {sx * .007f, -.00182941f, .1019482f}};
		struct xrt_pose grip_root, grip_model, grip_imu;
		math_pose_invert(&xrgrip, &grip_root);
		math_pose_transform(&grip_root, &model, &grip_model);
		math_pose_transform(&grip_model, &T_led_imu, &grip_imu);
		for (size_t i = 0; i < n; i++) {
			// As pssense_create: emission offset (x mirrored for the left hand), then LED -> IMU space.
			struct xrt_vec3 e = pssense_led_emission_offsets[i];
			if (hand == 0) e.x = -e.x;
			struct xrt_vec3 p = leds[i].position;
			math_vec3_accum(&e, &p);
			struct xrt_vec3 imu, g;
			math_pose_transform_point(&T_imu_led, &p, &imu);
			math_pose_transform_point(&grip_imu, &imu, &g);
			printf("%s,%.5f,%.5f,%.5f\n", hand == 0 ? "L" : "R", g.x, g.y, g.z);
		}
	}
	return 0;
}
