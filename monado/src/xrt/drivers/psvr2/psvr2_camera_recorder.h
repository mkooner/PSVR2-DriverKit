// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Opt-in recording of the PS VR2 camera image transfers, for offline work on the images (hand tracking).
 *
 * The USB thread copies a transfer into a free ring slot (or drops it when the writer is behind), a writer thread
 * appends the slots to one file, so the USB thread never waits on the disk. File format, little endian:
 *   file:   "PV2C" u32 version(1)
 *   record: u64 host_ns (arrival, os_monotonic), u32 transfer_length, u32 dropped_before, transfer bytes
 * The transfer bytes are exactly what img_xfer_cb received (256 byte header, then the images).
 * @ingroup drv_psvr2
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct psvr2_camera_recorder;

/*!
 * Start recording to @p path, at most @p max_hz transfers per second (0: every transfer), each up to
 * @p max_transfer_bytes. NULL if the file can't be opened.
 */
struct psvr2_camera_recorder *
psvr2_camera_recorder_create(const char *path, float max_hz, size_t max_transfer_bytes);

//! Offer one transfer (USB thread). Copies it if the rate allows and a slot is free; never blocks on the disk.
void
psvr2_camera_recorder_push(struct psvr2_camera_recorder *rec, int64_t host_ns, const uint8_t *data, size_t length);

//! Stop, write what's queued and close the file.
void
psvr2_camera_recorder_destroy(struct psvr2_camera_recorder **rec_ptr);

#ifdef __cplusplus
}
#endif
