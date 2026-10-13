// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  EuRoC dataset playback scene with no downstream consumers.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup gui
 */

#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_data_source.h"
#include "xrt/xrt_tracking.h"

#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_pretty_print.h"

#include "b_system_devices.h"

#include "gui_common.h"

#ifdef XRT_BUILD_DRIVER_EUROC
#include "euroc/euroc_interface.h"
#endif


#ifdef XRT_BUILD_DRIVER_EUROC

DEBUG_GET_ONCE_OPTION(euroc_path, "EUROC_PATH", NULL)

void
gui_scene_euroc_playback(struct gui_program *p, const char *path)
{
	if (path == NULL) {
		path = debug_get_option_euroc_path();
	}

	if (path == NULL) {
		U_LOG_E("No EuRoC dataset given, pass it as an argument or set EUROC_PATH");
		gui_scene_main_menu(p);
		return;
	}

	// Placeholder so that gui_scene_debug doesn't try to probe, also owns the player's frame context.
	struct b_system_devices *bsysd = b_system_devices_allocate();
	struct xrt_system_devices *xsysd = &bsysd->base;

	// Aborts if path isn't a valid EuRoC dataset.
	struct xrt_ds *xds = euroc_player_create(&bsysd->xfctx, path, NULL);

	// No downstream consumers, the player still feeds its own UI sinks and IMU plots.
	struct xrt_tracking_sinks sinks = {0};
	xrt_result_t xret = xrt_ds_tracking_streams_start(xds, &sinks);
	if (xret != XRT_SUCCESS) {
		U_LOG_E("Failed to start EuRoC playback: %s", u_str_xrt_result(xret));
		xrt_system_devices_destroy(&xsysd);
		gui_scene_main_menu(p);
		return;
	}

	p->xsysd = xsysd;

	gui_scene_debug(p);
}


#else /* XRT_BUILD_DRIVER_EUROC */


void
gui_scene_euroc_playback(struct gui_program *p, const char *path)
{
	(void)path;

	U_LOG_E("EuRoC driver not built");
	gui_scene_main_menu(p);
}


#endif /* XRT_BUILD_DRIVER_EUROC */
