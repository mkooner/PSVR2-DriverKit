// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS VR2 headset IMU sample times.
 *
 * The IMU records carry the VTS time in microseconds, but it only advances about once a millisecond while the IMU
 * runs at 2 kHz: in a recording (15:48) 49% of the samples had the same VTS as the one before, 46% were 998-999us
 * later and 5% 499-500us. The pose filter integrates a sample only if it is newer than its state, so it dropped every
 * second gyro sample and held the other for the whole millisecond. Two samples on one VTS are the two halves of that
 * millisecond: the second one gets the VTS plus half a period, which restores the 500us spacing.
 *
 * @ingroup drv_psvr2
 */

#pragma once

#include "util/u_time.h"

#include <stdbool.h>
#include <stdint.h>

//! The IMU period, 2 kHz.
#define PSVR2_IMU_PERIOD_NS (500 * 1000)

struct psvr2_imu_time
{
	bool have;
	int64_t last_raw_ns;
	int64_t last_out_ns;
};

//! The sample's time from its VTS time @p raw_ns (see the file comment).
static inline int64_t
psvr2_imu_time_fix(struct psvr2_imu_time *t, int64_t raw_ns)
{
	int64_t out = raw_ns;
	if (t->have && raw_ns == t->last_raw_ns) {
		out = t->last_out_ns + PSVR2_IMU_PERIOD_NS;
	}
	// Never at or before the previous sample (a clock step back is the caller's to handle, keep the order).
	if (t->have && out <= t->last_out_ns && raw_ns >= t->last_raw_ns) {
		out = t->last_out_ns + 1;
	}
	t->have = true;
	t->last_raw_ns = raw_ns;
	t->last_out_ns = out;
	return out;
}
