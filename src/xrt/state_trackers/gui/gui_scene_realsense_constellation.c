// Copyright 2019-2023, Collabora, Ltd.
// Copyright 2026, NVIDIA CORPORATION.
// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Small realsense constellation testing scene
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup gui
 */

#include "xrt/xrt_results.h"
#include "xrt/xrt_system.h"
#include "xrt/xrt_config_have.h"
#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_data_source.h"

#include "math/m_api.h"
#include "math/m_space.h"

#include "tracking/t_hand_tracking.h"
#include "tracking/t_tracking.h"
#include "tracking/t_euroc_recorder.h"
#include "tracking/t_constellation.h"

#include "constellation/t_rift_blobwatch.h"
#include "constellation/t_constellation_tracker.h"

#include "os/os_time.h"

#include "util/u_var.h"
#include "util/u_logging.h"
#include "util/u_sink.h"
#include "util/u_debug.h"
#include "util/u_pretty_print.h"
#include "util/u_builder_search.h"

#include "gui_common.h"
#include "gui_imgui.h"


#ifdef XRT_BUILD_DRIVER_REALSENSE
#include "realsense/rs_interface.h"
#endif

#ifdef XRT_BUILD_DRIVER_PSSENSE
#include "pssense/pssense_interface.h"
#endif


#if defined(XRT_BUILD_DRIVER_REALSENSE)


#define PSSENSE_CONTROLLER_COUNT 2


/*!
 * A GUI scene that owns the frame context for the realsense constellation test.
 * @implements gui_scene
 */
struct realsense_constellation_scene
{
	struct gui_scene base;

	struct xrt_frame_context xfctx;

	//! The PlayStation Sense controllers (left, right), if found. Their memory is owned by @ref xfctx.
	struct xrt_device *xdevs[PSSENSE_CONTROLLER_COUNT];

	struct u_sink_debug *debug_sinks;
	uint32_t debug_sink_count;
};

/*!
 * Looks for a PlayStation Sense controller with the given product ID, and if found creates it and adds it to the
 * constellation tracker.
 *
 * @return The controller, or NULL if it wasn't found or couldn't be set up. Its memory is owned by @p xfctx.
 */
static struct xrt_device *
create_pssense_controller(struct xrt_prober *xp,
                          struct xrt_prober_device **devices,
                          size_t device_count,
                          uint16_t product_id,
                          struct xrt_frame_context *xfctx,
                          struct t_constellation_tracker *constellation_tracker,
                          struct t_timing_event_source *timing_source)
{
	struct xrt_prober_device *xpdev = u_builder_find_prober_device( //
	    devices,                                                    //
	    device_count,                                               //
	    PSSENSE_VID,                                                //
	    product_id,                                                 //
	    XRT_BUS_TYPE_ANY);                                          //
	if (xpdev == NULL) {
		return NULL;
	}

	struct t_timing_event_sink *timing_sink = NULL;
	struct xrt_device *xdev = pssense_create(xp, xpdev, xfctx, &timing_sink);
	if (xdev == NULL) {
		U_LOG_W("Failed to create the PlayStation Sense controller (PID 0x%04x)", product_id);
		return NULL;
	}

	if (pssense_add_to_constellation_tracker(xdev, constellation_tracker) != 0) {
		U_LOG_W("Failed to add the PlayStation Sense controller (PID 0x%04x) to the constellation tracker",
		        product_id);
		// Still returned so the caller destroys it, its memory is owned by the frame context.
		return xdev;
	}

	if (timing_source != NULL && timing_sink != NULL) {
		t_timing_event_source_add_sink(timing_source, timing_sink);
	}

	return xdev;
}

//! Smallest the top down view will get in pixels, if the window is shrunk further it just scrolls.
#define VIEW_MIN_SIZE 100.0f

//! How many meters the view shows from the center to the nearest edge.
#define VIEW_HALF_EXTENT_METERS 1.0f

//! Length of the gizmo axes in meters.
#define GIZMO_AXIS_LENGTH 0.08f

static void
draw_gizmo(ImDrawList *dl, ImVec2 center, float pixels_per_meter, const struct xrt_pose *pose, const char *label)
{
	// Top down: tracking space +X is screen right, +Z is screen down (so -Z, forward, is up).
	ImVec2 origin = {center.x + pose->position.x * pixels_per_meter,
	                 center.y + pose->position.z * pixels_per_meter};

	// Axes are ABGR colors: X red, Y green, Z blue.
	const struct
	{
		struct xrt_vec3 dir;
		ImU32 color;
	} axes[] = {
	    {{1, 0, 0}, 0xff3030ff},
	    {{0, 1, 0}, 0xff30ff30},
	    {{0, 0, 1}, 0xffff7030},
	};

	for (uint32_t i = 0; i < ARRAY_SIZE(axes); i++) {
		struct xrt_vec3 dir;
		math_quat_rotate_vec3(&pose->orientation, &axes[i].dir, &dir);

		// Projected onto the XZ plane, an axis pointing up or down shows up short.
		ImVec2 end = {origin.x + dir.x * GIZMO_AXIS_LENGTH * pixels_per_meter,
		              origin.y + dir.z * GIZMO_AXIS_LENGTH * pixels_per_meter};
		ImDrawList_AddLine(dl, origin, end, axes[i].color, 2.0f);
	}

	ImVec2 text_pos = {origin.x + 6, origin.y + 6};
	ImDrawList_AddText_Vec2(dl, text_pos, 0xffffffff, label, NULL);
}

static void
scene_render(struct gui_scene *scene, struct gui_program *p)
{
	struct realsense_constellation_scene *rcs = (struct realsense_constellation_scene *)scene;

	static const char *labels[PSSENSE_CONTROLLER_COUNT] = {"L", "R"};

	igBegin("Controller poses (top down)", NULL, 0);

	// Fill whatever space the window has.
	ImVec2 size = igGetContentRegionAvail();
	size.x = size.x > VIEW_MIN_SIZE ? size.x : VIEW_MIN_SIZE;
	size.y = size.y > VIEW_MIN_SIZE ? size.y : VIEW_MIN_SIZE;

	float shortest_side = size.x < size.y ? size.x : size.y;
	float pixels_per_meter = shortest_side / 2 / VIEW_HALF_EXTENT_METERS;

	ImVec2 top_left = igGetCursorScreenPos();
	ImVec2 bottom_right = {top_left.x + size.x, top_left.y + size.y};
	ImVec2 center = {top_left.x + size.x / 2, top_left.y + size.y / 2};

	ImDrawList *dl = igGetWindowDrawList();
	ImDrawList_AddRectFilled(dl, top_left, bottom_right, 0xff202020, 0.0f, 0);
	ImDrawList_AddCircleFilled(dl, center, 3.0f, 0xff808080, 12);

	int64_t now_ns = os_monotonic_get_ns();
	for (uint32_t i = 0; i < PSSENSE_CONTROLLER_COUNT; i++) {
		if (rcs->xdevs[i] == NULL) {
			continue;
		}

		struct xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
		xrt_result_t xret =
		    xrt_device_get_tracked_pose(rcs->xdevs[i], XRT_INPUT_PSSENSE_GRIP_POSE, now_ns, &relation);

		const enum xrt_space_relation_flags needed =
		    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
		if (xret != XRT_SUCCESS || (relation.relation_flags & needed) != needed) {
			continue;
		}

		draw_gizmo(dl, center, pixels_per_meter, &relation.pose, labels[i]);
	}

	igDummy(size);

	igEnd();
}

static void
scene_destroy(struct gui_scene *scene, struct gui_program *p)
{
	struct realsense_constellation_scene *rcs = (struct realsense_constellation_scene *)scene;

	// Stops the controller threads, the memory itself is freed by the frame context.
	for (uint32_t i = 0; i < PSSENSE_CONTROLLER_COUNT; i++) {
		xrt_device_destroy(&rcs->xdevs[i]);
	}

	if (rcs->debug_sinks != NULL) {
		u_var_remove_root(rcs->debug_sinks);
	}

	// Destroys the sink chains feeding the debug sinks, so must come before destroying them.
	xrt_frame_context_destroy_nodes(&rcs->xfctx);

	for (uint32_t i = 0; i < rcs->debug_sink_count; i++) {
		u_sink_debug_destroy(&rcs->debug_sinks[i]);
	}
	free(rcs->debug_sinks);

	free(rcs);
}

void
gui_scene_realsense_constellation(struct gui_program *p)
{
	struct realsense_constellation_scene *rcs = U_TYPED_CALLOC(struct realsense_constellation_scene);
	rcs->base.render = scene_render;
	rcs->base.destroy = scene_destroy;

	struct xrt_frame_context *xfctx = &rcs->xfctx;

	// Pushed first so the frame context is cleaned up even if setup fails below.
	gui_scene_push_front(p, &rcs->base);

	struct rs_ds_context *context = rs_ds_context_create(xfctx);
	if (context == NULL) {
		U_LOG_E("Failed to create the RealSense data source context");
		gui_scene_delete_me(p, &rcs->base);
		return;
	}

	struct t_timing_event_source *timing_source = NULL;

	struct t_constellation_tracker_params constellation_tracker_params = {
	    .flags = T_CONSTELLATION_TRACKER_FLAGS_NONE,
	    .num_mosaics = 1,
	};

	struct t_constellation_tracker_camera_mosaic *mosaic = &constellation_tracker_params.mosaics[0];

	uint32_t sensor_count = rs_ds_context_sensor_count(context);
	for (uint32_t sensor_idx = 0; sensor_idx < sensor_count; sensor_idx++) {
		uint32_t ds_count = rs_ds_context_sensor_data_source_count(context, sensor_idx);
		for (uint32_t ds_idx = 0; ds_idx < ds_count; ds_idx++) {
			struct t_camera_calibration calib;
			struct xrt_ds *ds =
			    rs_ds_context_sensor_get_data_source(context, sensor_idx, ds_idx, &calib, &timing_source);
			if (ds == NULL) {
				continue;
			}

			if (mosaic->num_cameras >= XRT_TRACKING_MAX_CAMS) {
				U_LOG_W("Too many RealSense cameras, ignoring the rest");
				break;
			}

			mosaic->cameras[mosaic->num_cameras++] = (struct t_constellation_tracker_camera){
			    .calibration = calib,
			    .has_concrete_pose = false,
			    .pose_in_origin = XRT_POSE_IDENTITY,
			};
		}
	}

	if (mosaic->num_cameras == 0) {
		U_LOG_E("No RealSense cameras found");
		gui_scene_delete_me(p, &rcs->base);
		return;
	}

	struct t_constellation_tracker *constellation_tracker;
	int ret = t_constellation_tracker_create(xfctx, &constellation_tracker_params, &constellation_tracker);
	if (ret != 0) {
		U_LOG_E("Failed to create the constellation tracker: %d", ret);
		gui_scene_delete_me(p, &rcs->base);
		return;
	}

	struct xrt_prober_device **devices = NULL;
	size_t device_count = 0;
	xrt_result_t xret = xrt_prober_lock_list(p->xp, &devices, &device_count);
	if (xret != XRT_SUCCESS) {
		// Not fatal, we can still show the cameras without a controller.
		U_LOG_W("Failed to lock the prober device list, not looking for a controller: %d", xret);
	} else {
		static const uint16_t product_ids[PSSENSE_CONTROLLER_COUNT] = {PSSENSE_PID_LEFT, PSSENSE_PID_RIGHT};
		for (uint32_t i = 0; i < PSSENSE_CONTROLLER_COUNT; i++) {
			rcs->xdevs[i] = create_pssense_controller(p->xp, devices, device_count, product_ids[i], xfctx,
			                                          constellation_tracker, timing_source);
		}

		xrt_prober_unlock_list(p->xp, &devices);
	}

	struct u_sink_debug *debug_sinks = U_TYPED_ARRAY_CALLOC(struct u_sink_debug, mosaic->num_cameras);
	if (debug_sinks == NULL) {
		U_LOG_E("Failed to allocate debug sinks");
		gui_scene_delete_me(p, &rcs->base);
		return;
	}
	rcs->debug_sinks = debug_sinks;

	u_var_add_root(debug_sinks, "Realsense constellation tracker test", false);

	uint32_t streaming_count = 0;
	mosaic->num_cameras = 0;
	for (uint32_t sensor_idx = 0; sensor_idx < sensor_count; sensor_idx++) {
		uint32_t ds_count = rs_ds_context_sensor_data_source_count(context, sensor_idx);
		for (uint32_t ds_idx = 0; ds_idx < ds_count; ds_idx++) {
			struct t_camera_calibration calib;
			struct xrt_ds *ds =
			    rs_ds_context_sensor_get_data_source(context, sensor_idx, ds_idx, &calib, NULL);
			if (ds == NULL) {
				continue;
			}

			uint32_t camera_index = mosaic->num_cameras++;
			struct u_sink_debug *debug_sink = debug_sinks + camera_index;

			u_sink_debug_init(debug_sink);
			rcs->debug_sink_count = camera_index + 1;
			u_var_add_sink_debug(debug_sinks, debug_sink, "camera debug sink");

			struct t_blob_sink *blob_sink = mosaic->cameras[camera_index].blob_sink;
			u_sink_blob_visualizer_create(xfctx, blob_sink, debug_sink, calib.image_size_pixels.w,
			                              calib.image_size_pixels.h, &blob_sink);

			struct xrt_frame_sink *frame_sink;
			struct t_rift_blobwatch_params params = {
			    .pixel_threshold = RIFT_BLOBWATCH_PIXEL_THRESHOLD_DK2,
			    .blob_required_threshold = RIFT_BLOBWATCH_BLOB_REQUIRED_THRESHOLD,
			    .max_match_dist = RIFT_BLOBWATCH_DEFAULT_MAX_MATCH_DIST,
			    .max_blob_width = RIFT_BLOBWATCH_DEFAULT_MAX_BLOB_WIDTH,
			    .camera_calibration = calib,
			};

			struct t_blobwatch *blobwatch;

			ret = t_rift_blobwatch_create(&params, xfctx, blob_sink, &frame_sink, &blobwatch);
			if (ret != 0) {
				U_LOG_E("Failed to create blobwatch for camera %u: %d", camera_index, ret);
				continue;
			}

			u_sink_create_format_converter(xfctx, XRT_FORMAT_L8, frame_sink, &frame_sink);

			if (!u_sink_simple_queue_create(xfctx, frame_sink, &frame_sink)) {
				U_LOG_E("Failed to create frame queue for camera %u", camera_index);
				continue;
			}

			xret = xrt_ds_frame_stream_start(ds, frame_sink, XRT_DS_CAPTURE_TYPE_TRACKING, 0);
			if (xret != XRT_SUCCESS) {
				U_LOG_E("Failed to start frame stream for camera %u: %d", camera_index, xret);
				continue;
			}

			streaming_count++;
		}
	}

	if (streaming_count == 0) {
		U_LOG_E("No RealSense camera streams could be started");
		gui_scene_delete_me(p, &rcs->base);
		return;
	}

	gui_scene_debug(p);
}


#else /* XRT_BUILD_DRIVER_REALSENSE */


void
gui_scene_realsense_constellation(struct gui_program *p)
{
	// No-op
}


#endif /* XRT_BUILD_DRIVER_REALSENSE */
