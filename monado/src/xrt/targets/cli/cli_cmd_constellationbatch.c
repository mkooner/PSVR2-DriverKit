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
#include <string.h>

#define P(...) fprintf(stderr, __VA_ARGS__)

#if defined(XRT_MODULE_CONSTELLATION_TRACKING) && defined(XRT_BUILD_DRIVER_EUROC)

#include "euroc/euroc_interface.h"

#include "constellation/t_constellation_tracker.h"
#include "constellation/trace_log/t_constellation_tracker_trace_sink_csv.h"
#include "constellation/t_rift_blobwatch.h"

#include "os/os_time.h"

#include "util/u_misc.h"


//! Trace logging output for a single dataset run.
struct trace_output
{
	struct t_constellation_tracker_trace_sink_csv_files files;
	//! One sink per blobwatch, all sharing @ref files.
	struct t_constellation_trace_log_sink *sinks[XRT_TRACKING_MAX_CAMS];
};

static void
stub_push_constellation_tracker_sample(struct t_constellation_tracker_device *device,
                                       struct t_constellation_tracker_sample *sample)
{
	// Nothing consumes the samples, we just run the tracker.
}

//! Gets the last path component of @p path, ignoring trailing slashes.
static void
get_dataset_name(const char *path, char *out, size_t out_size)
{
	snprintf(out, out_size, "%s", path);

	size_t len = strlen(out);
	while (len > 1 && out[len - 1] == '/') {
		out[--len] = '\0';
	}

	const char *last_slash = strrchr(out, '/');
	if (last_slash != NULL) {
		memmove(out, last_slash + 1, strlen(last_slash + 1) + 1);
	}
}

static FILE *
open_trace_file(const char *trace_dir, const char *dataset_name, const char *suffix)
{
	char path[4096];
	snprintf(path, sizeof(path), "%s/%s_%s.csv", trace_dir, dataset_name, suffix);

	FILE *file = fopen(path, "w");
	if (file == NULL) {
		P("Failed to open trace file '%s'\n", path);
	}
	return file;
}

static int
trace_output_open(struct trace_output *out,
                  const char *trace_dir,
                  const char *dataset_path,
                  const t_constellation_device_id_t *device_ids,
                  uint32_t device_count)
{
	char dataset_name[1024];
	get_dataset_name(dataset_path, dataset_name, sizeof(dataset_name));

	out->files.timings = open_trace_file(trace_dir, dataset_name, "timings");
	if (out->files.timings == NULL) {
		return -1;
	}
	t_constellation_tracker_trace_sink_csv_write_timings_header(out->files.timings);

	for (uint32_t i = 0; i < device_count; i++) {
		t_constellation_device_id_t id = device_ids[i];
		if (id < 0 || id >= XRT_CONSTELLATION_MAX_DEVICES) {
			P("Device ID %d out of range for trace output, skipping\n", id);
			continue;
		}

		char suffix[64];
		snprintf(suffix, sizeof(suffix), "matching_device_%d", id);

		FILE *file = open_trace_file(trace_dir, dataset_name, suffix);
		if (file == NULL) {
			return -1;
		}
		t_constellation_tracker_trace_sink_csv_write_matching_header(file);
		out->files.device_matching[id] = file;
	}

	return 0;
}

//! Must be called after the frame context is destroyed, so no trace logs are still alive.
static void
trace_output_close(struct trace_output *out)
{
	for (uint32_t i = 0; i < ARRAY_SIZE(out->sinks); i++) {
		t_constellation_tracker_trace_sink_csv_destroy(&out->sinks[i]);
	}

	if (out->files.timings != NULL) {
		fclose(out->files.timings);
		out->files.timings = NULL;
	}

	for (uint32_t i = 0; i < ARRAY_SIZE(out->files.device_matching); i++) {
		if (out->files.device_matching[i] != NULL) {
			fclose(out->files.device_matching[i]);
			out->files.device_matching[i] = NULL;
		}
	}
}

static int
run_dataset(const char *dataset_path, const char *trace_dir)
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

	ep_config->playback.skip_first = 90;
	ep_config->playback.skip_perc = true;

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
	t_constellation_device_id_t device_ids[XRT_TRACKING_MAX_IMUS];
	uint32_t device_count = 0;
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
		device_ids[device_count++] = id;
		sinks.imus[i] = params.imu_sink;
	}
	sinks.imu_count = (int)info->imu_count;

	struct trace_output trace = {0};
	if (trace_dir != NULL && trace_output_open(&trace, trace_dir, dataset_path, device_ids, device_count) != 0) {
		xrt_frame_context_destroy_nodes(&xfctx);
		trace_output_close(&trace);
		free(ep_config);
		return EXIT_FAILURE;
	}

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
			trace_output_close(&trace);
			free(ep_config);
			return EXIT_FAILURE;
		}

		if (trace_dir != NULL) {
			trace.sinks[i] = t_constellation_tracker_trace_sink_csv_create(&trace.files, 0, i);
			t_rift_blobwatch_set_trace_log_sink(blobwatch, trace.sinks[i]);
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

	// Destroying the nodes releases every trace log, so the trace sinks can be safely destroyed after.
	xrt_frame_context_destroy_nodes(&xfctx);
	trace_output_close(&trace);
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

	const char *trace_dir = NULL;
	if (nof_args >= 2 && strcmp(args[0], "--trace-dir") == 0) {
		trace_dir = args[1];
		nof_args -= 2;
		args += 2;
	}

	if (nof_args == 0) {
		P("Runs EuRoC datasets through a deterministic, single-threaded constellation tracker.\n");
		P("Usage: %s %s [--trace-dir <dir>] <euroc_path>...\n", argv[0], argv[1]);
		P("  --trace-dir <dir>  Write trace log CSVs for each dataset into <dir>, named after the dataset.\n");
		return EXIT_FAILURE;
	}

	for (int i = 0; i < nof_args; i++) {
		int ret = run_dataset(args[i], trace_dir);
		if (ret != EXIT_SUCCESS) {
			return ret;
		}
	}
	return EXIT_SUCCESS;
#endif
}
