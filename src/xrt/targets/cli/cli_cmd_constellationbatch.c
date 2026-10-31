// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  EuRoC datasets constellation tracker runner
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 */

#include "xrt/xrt_config_build.h"
#include "xrt/xrt_config_have.h"
#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_data_source.h"

#include <stdio.h>
#include <stdlib.h>

#define P(...) fprintf(stderr, __VA_ARGS__)

#if defined(XRT_MODULE_CONSTELLATION_TRACKING) && defined(XRT_BUILD_DRIVER_EUROC)

#include "euroc/euroc_interface.h"

#include "constellation/t_constellation_tracker.h"
#include "constellation/t_rift_blobwatch.h"

#include "os/os_time.h"

#include "util/u_misc.h"


static void
stub_push_constellation_tracker_sample(struct t_constellation_tracker_device *device,
                                       struct t_constellation_tracker_sample *sample)
{
	// Nothing consumes the samples, we just run the tracker.
}

static int
run_dataset(const char *dataset_path)
{
	struct euroc_player_config *ep_config = U_TYPED_CALLOC(struct euroc_player_config);
	euroc_player_fill_default_config_for(ep_config, dataset_path);

	// Feed everything as fast as possible, in order, from the single player thread.
	//! @todo: Enable max speed when constellation tracker has synchronous sensor fusion
	ep_config->playback.max_speed = true;
	ep_config->playback.use_source_ts = true;
	ep_config->playback.play_from_start = true;
	ep_config->playback.print_progress = false;
	ep_config->playback.color = false;
	ep_config->playback.gt = false;
	ep_config->playback.cam_delay_ns = 15L * U_TIME_1MS_IN_NS; // 15ms transfer latency for cameras
	ep_config->playback.imu_delay_ns = 2L * U_TIME_1MS_IN_NS;  // 2ms transfer latency for IMU

	struct xrt_frame_context xfctx = {0};
	struct xrt_ds *xds = euroc_player_create(&xfctx, dataset_path, ep_config);

	const struct euroc_player_dataset_info *info = NULL;
	euroc_player_get_dataset_info(xds, &info);
	struct t_constellation_tracker_params tracker_params = {
	    .flags = T_CONSTELLATION_TRACKER_FLAGS_DETERMINISTIC,
	    .num_mosaics = 1,
	};
	struct t_constellation_tracker_camera_mosaic *mosaic = &tracker_params.mosaics[0];
	for (uint32_t i = 0; i < info->cam_count && i < ep_config->playback.cam_count; i++) {
		if (!info->cams[i].calib_valid) {
			P("Camera %u in dataset has no valid calibration\n", i);
			xrt_frame_context_destroy_nodes(&xfctx);
			free(ep_config);
			return EXIT_FAILURE;
		}
		mosaic->cameras[mosaic->num_cameras++] = (struct t_constellation_tracker_camera){
		    .calibration = info->cams[i].calib,
		    .pose_in_origin = info->cams[i].pose,
		    .has_concrete_pose = true,
		};
	}

	struct t_constellation_tracker *tracker = NULL;
	if (t_constellation_tracker_create(&xfctx, &tracker_params, &tracker) != 0) {
		P("Failed to create constellation tracker\n");
		xrt_frame_context_destroy_nodes(&xfctx);
		free(ep_config);
		return EXIT_FAILURE;
	}

	struct xrt_tracking_sinks sinks = {0};

	// Devices have to be added before any data flows
	struct t_constellation_tracker_device devices[XRT_TRACKING_MAX_IMUS] = {0};
	for (uint32_t i = 0; i < info->imu_count && i < ep_config->playback.imu_count; i++) {
		if (!info->imus[i].has_led_model) {
			continue;
		}

		devices[i].push_constellation_tracker_sample = stub_push_constellation_tracker_sample;

		if (!info->imus[i].has_imu_calibration) {
			P("IMU %u in dataset has no IMU calibration\n", i);
			xrt_frame_context_destroy_nodes(&xfctx);
			free(ep_config);
			return EXIT_FAILURE;
		}

		struct t_constellation_tracker_device_params params = {
		    .led_model = info->imus[i].led_model,
		    .imu_calibration = info->imus[i].imu_calibration,
		};
		t_constellation_device_id_t id;
		if (t_constellation_tracker_add_device(tracker, &params, &devices[i], &id) != 0) {
			P("Failed to add IMU %u as a constellation device\n", i);
			xrt_frame_context_destroy_nodes(&xfctx);
			free(ep_config);
			return EXIT_FAILURE;
		}
		sinks.imus[i] = params.imu_sink;
	}
	sinks.imu_count = (int)info->imu_count;

	// Each camera's frames go straight into its blobwatch, which pushes blobs straight into the tracker
	for (uint32_t i = 0; i < mosaic->num_cameras; i++) {
		struct t_rift_blobwatch_params bw_params = {
		    .pixel_threshold = RIFT_BLOBWATCH_PIXEL_THRESHOLD_CV1,
		    .blob_required_threshold = RIFT_BLOBWATCH_BLOB_REQUIRED_THRESHOLD,
		    .max_match_dist = RIFT_BLOBWATCH_DEFAULT_MAX_MATCH_DIST,
		    .max_blob_width = RIFT_BLOBWATCH_DEFAULT_MAX_BLOB_WIDTH,
		    .camera_calibration = mosaic->cameras[i].calibration,
		};
		struct t_blobwatch *blobwatch = NULL;
		if (t_rift_blobwatch_create(&bw_params, &xfctx, mosaic->cameras[i].blob_sink, &sinks.cams[i],
		                            &blobwatch) != 0) {
			P("Failed to create blobwatch for camera %u\n", i);
			xrt_frame_context_destroy_nodes(&xfctx);
			free(ep_config);
			return EXIT_FAILURE;
		}
	}
	sinks.cam_count = (int)mosaic->num_cameras;

	xrt_ds_tracking_streams_start(xds, &sinks);

	bool running = true;
	while (running) {
		os_nanosleep(50 * U_TIME_1MS_IN_NS);
		if (xrt_ds_is_running(xds, &running) != XRT_SUCCESS) {
			break;
		}
	}

	xrt_frame_context_destroy_nodes(&xfctx);
	free(ep_config);
	return EXIT_SUCCESS;
}

#endif

int
cli_cmd_constellationbatch(int argc, const char **argv)
{
#if !defined(XRT_MODULE_CONSTELLATION_TRACKING)
	P("No constellation tracker built.\n");
	return EXIT_FAILURE;
#elif !defined(XRT_BUILD_DRIVER_EUROC)
	P("Euroc driver not built, can't reproduce datasets.\n");
	return EXIT_FAILURE;
#else
	// Do not count "monado-cli" and "constellationbatch" as args
	int nof_args = argc - 2;
	const char **args = &argv[2];

	if (nof_args == 0) {
		P("Runs EuRoC datasets through a deterministic, single-threaded constellation tracker.\n");
		P("Usage: %s %s <euroc_path>...\n", argv[0], argv[1]);
		return EXIT_FAILURE;
	}

	for (int i = 0; i < nof_args; i++) {
		int ret = run_dataset(args[i]);
		if (ret != EXIT_SUCCESS) {
			return ret;
		}
	}
	return EXIT_SUCCESS;
#endif
}
