// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PlayStation Sense controller LED models. Data extracted from Sony's PC driver.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_pssense
 */

#pragma once

#include "xrt/xrt_tracking.h"

#include "tracking/t_constellation.h"

#include "math/m_api.h"


static const float LED_RADIUS_M = 0.006f; // 6mm
static const float LED_VISIBILITY_ANGLE = DEG_TO_RAD(90);
static const struct t_constellation_tracker_led_model_match_parameters DEFAULT_MATCH_PARAMETERS = {
    .min_leds_for_correspondence_search_without_prior = 7,
    .min_leds_for_correspondence_search_with_prior = 5,
};

/*!
 * Rotation on X axis from LED -> IMU, in degrees (the default for PSSENSE_IMU_ANGLE_DEG): upstream's value.
 *
 * A refit to -79.77 (matching the optical orientation to the driver's 3DoF IMU orientation) was wrong: on hardware
 * (01:45 vs 01:47 runs) -50.27 sat better in the hand and tracked better (45/40 vs 32/31 solves/s, 33/34% vs 47/45%
 * of the time position-held, 7 vs 17 fusion overrides). The 3DoF orientation itself is ~28 deg off the LED model's IMU
 * frame about X; see PSSENSE_IMU_REF_CORRECTION_DEG.
 */
#define PSSENSE_IMU_ANGLE_DEG_DEFAULT (-50.2703f)

/*!
 * Rotation about X applied to the driver's 3DoF IMU orientation before the tracker compares poses with it (gravity
 * and gyro gates, in-search check), degrees (PSSENSE_IMU_REF_CORRECTION_DEG). With -50.27 the optical orientation
 * disagreed with the uncorrected 3DoF orientation by 29.9 / 26.7 deg (left / right, 01:47 run), close to the 35 deg
 * gravity gate; a pure X rotation of +30.0 / +26.5 deg brings that to 7.0 / 6.5 deg (96% / 84% within 20 deg).
 */
/*
 * 2026-11-19: that ~30 deg was the PS VR2 camera rig's mount error (the tracker's camera 0 looked level instead of
 * 28.9 deg down, see PSVR2_CAM_ALIGN in psvr2_constellation_tracking.c). With the rig aligned, 28.25 made the optical
 * and IMU up disagree by 30.3 deg (median, both controllers) next to the 35 deg gravity gate and tracking fell apart
 * in motion; with 0 they agree to 2.5 deg (median, 3.9 p90), better than the old 3.7 / 6.5 deg, and the accelerometer
 * at rest matches the optical up to 0.9-1.3 deg (desk recording 20:58, replayed with the aligned rig).
 */
#define PSSENSE_IMU_REF_CORRECTION_DEG_DEFAULT (0.0f)

//! The offset from the LED origin -> IMU origin, for the left controller
static struct xrt_vec3 T_led_imu_left = {
    .x = 0.0093727f,
    .y = -0.0122481f,
    .z = -0.0060039f,
};
//! The offset from the LED origin -> IMU origin, for the right controller
static struct xrt_vec3 T_led_imu_right = {
    .x = -0.0200727f,
    .y = -0.0122481f,
    .z = -0.0060039f,
};

/*!
 * IMU noise model for the PS VR2 controllers.
 */
static const struct xrt_imu_calibration PSSENSE_IMU_CALIBRATION = {
    .gyro =
        {
            // 3e-4 (rad/s)^2 per sample
            .noise_density = 1.5e-3,
            // 0.5% scale factor and 0.25% cross axis error per sample
            .scale_factor_noise_density = 4.3301270e-4,
            .cross_axis_noise_density = 2.1650635e-4,
            // 1900 deg/s, a saturated axis gets (100 deg/s)^2 per sample
            .axis_saturation = 33.1612587,
            .saturated_noise_density = 0.15114995,
            // (0.001 deg/s)^2 per second
            .bias_random_walk = 1.745e-5,
            // 0.025 deg/s on factory calibrated samples
            .bias_initial_sigma = 4.363e-4,
        },
    .accel =
        {
            // 1.08e-4 G^2 per sample
            .noise_density = 8.8259848e-3,
            .scale_factor_noise_density = 4.3301270e-4,
            .cross_axis_noise_density = 2.1650635e-4,
            // 7.8 G, a saturated axis gets (2.5 G)^2 per sample
            .axis_saturation = 76.49187,
            .saturated_noise_density = 2.1232020,
            // The PS VR2 driver's fusion has no accelerometer bias random walk, this is Monado's default
            .bias_random_walk = 1e-3,
            // 1 mg on factory calibrated samples
            .bias_initial_sigma = 9.807e-3,
        },
    // The PS VR2 driver's fusion doesn't estimate the accelerometer scale, these are Monado's defaults
    .accel_scale_random_walk = 1e-5,
    .accel_scale_initial_sigma = 0.05,
};

/*
 * Intercepted at address 0xdde10 in the driver, within a vector in the first
 * parameter object.
 */
static const struct t_constellation_tracker_led pssense_left_leds[] = {
    {
        .position = {-0.012054, -0.055213, 0.038152},
        .normal = {-0.176485, -0.811967, 0.556384},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 0,
    },
    {
        .position = {-0.033088, -0.055236, 0.020190},
        .normal = {-0.426232, -0.891845, -0.151455},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 1,
    },
    {
        .position = {-0.053695, -0.031009, 0.028350},
        .normal = {-0.788493, -0.456350, 0.412339},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 2,
    },
    {
        .position = {-0.066986, -0.010505, 0.008679},
        .normal = {-0.980055, -0.153898, 0.125726},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 3,
    },
    {
        .position = {-0.063505, 0.024601, 0.004185},
        .normal = {-0.930796, 0.360502, 0.060482},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 4,
    },
    {
        .position = {-0.047803, 0.044966, 0.018502},
        .normal = {-0.701817, 0.660177, 0.267618},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 5,
    },
    {
        .position = {0.045472, -0.027042, 0.043022},
        .normal = {0.667492, -0.396842, 0.630056},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 6,
    },
    {
        .position = {0.059535, -0.013977, 0.028536},
        .normal = {0.961512, -0.273488, -0.026435},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 7,
    },
    {
        .position = {0.061060, 0.014608, 0.024725},
        .normal = {0.984435, 0.154641, -0.083507},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 8,
    },
    {
        .position = {0.050994, 0.027255, 0.036184},
        .normal = {0.748123, 0.399672, 0.529693},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 9,
    },
    {
        .position = {0.048475, 0.043025, 0.018776},
        .normal = {0.795992, 0.580182, -0.172584},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 10,
    },
    {
        .position = {0.030569, 0.053446, 0.029350},
        .normal = {0.448555, 0.783814, 0.429458},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 11,
    },
    {
        .position = {0.018498, 0.063913, 0.011230},
        .normal = {0.346695, 0.893406, -0.285708},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 12,
    },
    {
        .position = {-0.002748, 0.064260, 0.022718},
        .normal = {-0.040150, 0.942362, 0.332178},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 13,
    },
    {
        .position = {-0.029078, 0.013435, -0.060735},
        .normal = {-0.417589, 0.258817, -0.870996},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 14,
    },
    {
        .position = {0.001067, 0.013435, -0.067327},
        .normal = {0.014114, 0.258800, -0.965828},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 15,
    },
    {
        .position = {0.031783, 0.013435, -0.059270},
        .normal = {0.458628, 0.258803, -0.850106},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 16,
    },
};

static const struct t_constellation_tracker_led pssense_right_leds[] = {
    {
        .position = {0.012054, -0.055213, 0.038152},
        .normal = {0.176485, -0.811967, 0.556384},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 0,
    },
    {
        .position = {0.033088, -0.055236, 0.020190},
        .normal = {0.426232, -0.891845, -0.151455},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 1,
    },
    {
        .position = {0.053695, -0.031009, 0.028350},
        .normal = {0.788493, -0.456350, 0.412339},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 2,
    },
    {
        .position = {0.066986, -0.010505, 0.008679},
        .normal = {0.980055, -0.153898, 0.125726},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 3,
    },
    {
        .position = {0.063505, 0.024601, 0.004185},
        .normal = {0.930796, 0.360502, 0.060482},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 4,
    },
    {
        .position = {0.047803, 0.044966, 0.018502},
        .normal = {0.701817, 0.660177, 0.267618},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 5,
    },
    {
        .position = {-0.045472, -0.027042, 0.043022},
        .normal = {-0.667492, -0.396842, 0.630056},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 6,
    },
    {
        .position = {-0.059535, -0.013977, 0.028536},
        .normal = {-0.961512, -0.273488, -0.026435},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 7,
    },
    {
        .position = {-0.061060, 0.014608, 0.024725},
        .normal = {-0.984435, 0.154641, -0.083507},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 8,
    },
    {
        .position = {-0.050994, 0.027255, 0.036184},
        .normal = {-0.748123, 0.399672, 0.529693},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 9,
    },
    {
        .position = {-0.048475, 0.043025, 0.018776},
        .normal = {-0.795992, 0.580182, -0.172584},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 10,
    },
    {
        .position = {-0.030569, 0.053446, 0.029350},
        .normal = {-0.448555, 0.783814, 0.429458},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 11,
    },
    {
        .position = {-0.018498, 0.063913, 0.011230},
        .normal = {-0.346695, 0.893406, -0.285708},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 12,
    },
    {
        .position = {0.002748, 0.064260, 0.022718},
        .normal = {0.040150, 0.942362, 0.332178},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 13,
    },
    {
        .position = {0.029078, 0.013435, -0.060735},
        .normal = {0.417589, 0.258817, -0.870996},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 14,
    },
    {
        .position = {-0.001067, 0.013435, -0.067327},
        .normal = {-0.014114, 0.258800, -0.965828},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 15,
    },
    {
        .position = {-0.031783, 0.013435, -0.059270},
        .normal = {-0.458628, 0.258803, -0.850106},
        .radius_m = LED_RADIUS_M,
        .visibility_angle = LED_VISIBILITY_ANGLE,
        .id = 16,
    },
};

/*!
 * Sony forgot to factor in the plastic when computing the original
 * model, this is a fixup on top of that.
 *
 * This applies specifically to the right controller, and has X inverted for the left controller.
 *
 * Computed via a bundle adjustment of a large controller tracking dataset with CV1.
 */
static const struct xrt_vec3 pssense_led_emission_offsets[] = {
    {0.000262, -0.001698, 0.000803},   // LED  0: +1.87 mm along normal
    {0.000599, -0.001296, -0.000481},  // LED  1: +1.48 mm along normal
    {0.001550, -0.001087, 0.000603},   // LED  2: +1.97 mm along normal
    {0.001588, -0.000457, -0.000046},  // LED  3: +1.62 mm along normal
    {0.001498, 0.000488, -0.000025},   // LED  4: +1.57 mm along normal
    {0.001238, 0.000888, 0.000272},    // LED  5: +1.53 mm along normal
    {-0.000983, -0.000921, 0.001300},  // LED  6: +1.84 mm along normal
    {-0.001169, -0.000671, -0.000025}, // LED  7: +1.31 mm along normal
    {-0.001315, -0.000090, -0.000021}, // LED  8: +1.28 mm along normal
    {-0.001246, 0.000856, 0.000855},   // LED  9: +1.73 mm along normal
    {-0.001071, 0.000414, -0.000405},  // LED 10: +1.16 mm along normal
    {-0.000537, 0.001162, 0.000544},   // LED 11: +1.39 mm along normal
    {-0.000372, 0.000718, -0.000602},  // LED 12: +0.94 mm along normal
    {0.000399, 0.001257, 0.000289},    // LED 13: +1.30 mm along normal
    {0.000331, 0.000033, -0.001030},   // LED 14: +1.04 mm along normal
    {-0.000125, 0.000117, -0.001062},  // LED 15: +1.06 mm along normal
    {-0.000648, 0.000286, -0.000970},  // LED 16: +1.20 mm along normal
};
