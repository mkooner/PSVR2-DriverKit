// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  See comp_frame_log.h.
 * @ingroup comp_main
 */

#include "comp_frame_log.h"

#include "os/os_threading.h"
#include "os/os_time.h"
#include "util/u_debug.h"

#include <pthread.h>
#include <stdio.h>

DEBUG_GET_ONCE_OPTION(comp_frame_log, "XRT_COMPOSITOR_FRAME_LOG", NULL)

static struct
{
	struct os_mutex mutex;
	FILE *file;
	int state; // 0 not opened yet, 1 open, -1 off
	unsigned rows;
} g_log;

static pthread_once_t g_once = PTHREAD_ONCE_INIT;

//! Opens the log once, whichever thread (renderer, present, vblank) comes first; off when no path is set.
static void
open_once(void)
{
	const char *path = debug_get_option_comp_frame_log();
	if (path == NULL || path[0] == '\0' || os_mutex_init(&g_log.mutex) != 0) {
		g_log.state = -1;
		return;
	}
	g_log.file = fopen(path, "w");
	if (g_log.file == NULL) {
		g_log.state = -1;
		return;
	}
	fprintf(g_log.file, "# main compositor frame log (comp_frame_log.h)\n");
	g_log.state = 1;
}

static FILE *
lock_file(void)
{
	pthread_once(&g_once, open_once);
	if (g_log.state < 0) {
		return NULL;
	}
	os_mutex_lock(&g_log.mutex);
	return g_log.file;
}

static void
unlock_file(void)
{
	if (++g_log.rows % 500 == 0) {
		fflush(g_log.file);
	}
	os_mutex_unlock(&g_log.mutex);
}

void
comp_frame_log_render(int64_t frame_id,
                      int64_t predicted_display_ns,
                      uint32_t layer_count,
                      int64_t layer0_timestamp_ns,
                      const struct xrt_pose *head)
{
	FILE *f = lock_file();
	if (f == NULL) {
		return;
	}
	fprintf(f, "W,%lld,%lld,%lld,%u,%lld,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f\n", (long long)os_monotonic_get_ns(),
	        (long long)frame_id, (long long)predicted_display_ns, layer_count, (long long)layer0_timestamp_ns,
	        head->position.x, head->position.y, head->position.z, head->orientation.x, head->orientation.y,
	        head->orientation.z, head->orientation.w);
	unlock_file();
}

void
comp_frame_log_present(int64_t frame_id, int64_t desired_present_ns)
{
	FILE *f = lock_file();
	if (f == NULL) {
		return;
	}
	fprintf(f, "P,%lld,%lld,%lld\n", (long long)os_monotonic_get_ns(), (long long)frame_id,
	        (long long)desired_present_ns);
	unlock_file();
}

void
comp_frame_log_vblank(int64_t vblank_ns)
{
	FILE *f = lock_file();
	if (f == NULL) {
		return;
	}
	fprintf(f, "B,%lld\n", (long long)vblank_ns);
	unlock_file();
}
