// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Per-frame record of what the main compositor shows (XRT_COMPOSITOR_FRAME_LOG=path, off by default), for
 *         measuring on-screen jitter: which client frame each displayed frame used, the head pose it re-warped to,
 *         when it was presented, and every vblank. psvr2-test/analysis/screen_jitter.py reads it.
 *
 * Rows (CLOCK_MONOTONIC ns):
 *   W,now_ns,frame_id,predicted_display_ns,layer_count,layer0_timestamp_ns,head px,py,pz,qx,qy,qz,qw
 *       renderer: the re-warp target (head pose for the predicted display time, device space) and layer 0's
 *       timestamp (the client frame's display time: identifies the client frame shown, repeats included)
 *   P,now_ns,frame_id,desired_present_ns    the frame handed to the display (vkQueuePresentKHR)
 *   B,vblank_ns                              a vblank (display-control event thread)
 * @ingroup comp_main
 */

#pragma once

#include "xrt/xrt_defines.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void
comp_frame_log_render(int64_t frame_id,
                      int64_t predicted_display_ns,
                      uint32_t layer_count,
                      int64_t layer0_timestamp_ns,
                      const struct xrt_pose *head);

void
comp_frame_log_present(int64_t frame_id, int64_t desired_present_ns);

void
comp_frame_log_vblank(int64_t vblank_ns);

#ifdef __cplusplus
}
#endif
