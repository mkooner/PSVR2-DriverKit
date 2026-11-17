// Copyright 2025, Beyley Cardellio
// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS VR2 prober code.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup xrt_iface
 */

#include "psvr2/psvr2.h"
#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_system.h"

#include "util/u_debug.h"
#include "util/u_misc.h"
#include "util/u_logging.h"
#include "util/u_trace_marker.h"
#include "util/u_builder_search.h"

#include "target_builder_helpers.h"

#include "psvr2/psvr2_interface.h"

#ifdef XRT_BUILD_DRIVER_HANDTRACKING
#include "psvr2/psvr2_hand_tracking.h"
#include "ht_ctrl_emu/ht_ctrl_emu_interface.h"
#endif

#ifdef XRT_BUILD_DRIVER_PSSENSE
#include "pssense/pssense_interface.h"
#endif

/*
 *
 * Internal structures
 *
 */

struct psvr2_builder
{
	struct t_builder base;

#ifdef XRT_BUILD_DRIVER_PSSENSE
	struct xrt_device *pssense_left;
	struct xrt_device *pssense_right;

	struct t_timing_event_source *timing_event_source;

	struct t_constellation_tracker *constellation_tracker;
#endif
	enum u_logging_level log_level;
};

static struct psvr2_builder *
psvr2_builder(struct xrt_builder *xb)
{
	return (struct psvr2_builder *)xb;
}

/*
 *
 * Misc stuff.
 *
 */

DEBUG_GET_ONCE_LOG_OPTION(psvr2_log, "PSVR2_LOG", U_LOGGING_WARN)

#undef PSVR2_ERROR
#undef PSVR2_DEBUG
#define PSVR2_ERROR(p, ...) U_LOG_IFL_E(p->log_level, __VA_ARGS__)
#define PSVR2_DEBUG(p, ...) U_LOG_IFL_D(p->log_level, __VA_ARGS__)

static const char *driver_list[] = {
    "psvr2",
    "pssense",
};


/*
 *
 * Member functions.
 *
 */

static xrt_result_t
psvr2_estimate_system(struct xrt_builder *xb,
                      cJSON *config,
                      struct xrt_prober *xp,
                      struct xrt_builder_estimate *estimate)
{
	struct psvr2_builder *pb = psvr2_builder(xb);

	struct xrt_prober_device **xpdevs = NULL;
	size_t xpdev_count = 0;
	xrt_result_t xret = XRT_SUCCESS;

	U_ZERO(estimate);

	xret = xrt_prober_lock_list(xp, &xpdevs, &xpdev_count);
	U_LOG_CHK_AND_RET(pb->log_level, xret, "xrt_prober_lock_list");

	struct xrt_prober_device *dev =
	    u_builder_find_prober_device(xpdevs, xpdev_count, PSVR2_VID, PSVR2_PID, XRT_BUS_TYPE_USB);
	if (dev != NULL) {
		estimate->certain.head = true;
		estimate->certain.dof6 = true;
	}

#ifdef XRT_BUILD_DRIVER_PSSENSE
	struct xrt_prober_device *dev_controller_left =
	    u_builder_find_prober_device(xpdevs, xpdev_count, PSSENSE_VID, PSSENSE_PID_LEFT, XRT_BUS_TYPE_ANY);
	if (dev_controller_left != NULL) {
		estimate->certain.left = true;
	}

	struct xrt_prober_device *dev_controller_right =
	    u_builder_find_prober_device(xpdevs, xpdev_count, PSSENSE_VID, PSSENSE_PID_RIGHT, XRT_BUS_TYPE_ANY);
	if (dev_controller_right != NULL) {
		estimate->certain.right = true;
	}
#endif

	PSVR2_DEBUG(pb, "PSVR2 builder estimate: head %d, left %d, right %d", estimate->certain.head,
	            estimate->certain.left, estimate->certain.right);

	xret = xrt_prober_unlock_list(xp, &xpdevs);
	U_LOG_CHK_AND_RET(pb->log_level, xret, "xrt_prober_unlock_list");

	return xret;
}

static xrt_result_t
psvr2_open_system_impl(struct xrt_builder *xb,
                       cJSON *config,
                       struct xrt_prober *xp,
                       struct xrt_tracking_origin *origin,
                       struct xrt_system_devices *xsysd,
                       struct xrt_frame_context *xfctx,
                       struct t_builder_options *tbo)
{
	struct psvr2_builder *pb = psvr2_builder(xb);
	struct xrt_prober_device **xpdevs = NULL;
	size_t xpdev_count = 0;
	xrt_result_t xret = XRT_SUCCESS;

	DRV_TRACE_MARKER();

	xret = xrt_prober_lock_list(xp, &xpdevs, &xpdev_count);
	if (xret != XRT_SUCCESS) {
		goto unlock_and_fail;
	}

	struct xrt_device *head_xdev = NULL;
	struct xrt_prober_device *head_xpdev =
	    u_builder_find_prober_device(xpdevs, xpdev_count, PSVR2_VID, PSVR2_PID, XRT_BUS_TYPE_USB);
	if (head_xpdev != NULL) {
		head_xdev = psvr2_hmd_create(head_xpdev);
		if (head_xdev == NULL) {
			PSVR2_ERROR(psvr2_builder(xb), "PSVR2 HMD device creation failed");
			goto unlock_and_fail;
		}
		pb->timing_event_source = psvr2_hmd_get_timing_event_source((struct psvr2_hmd *)head_xdev);
		xsysd->static_xdevs[xsysd->static_xdev_count++] = head_xdev;
	}

	tbo->head = head_xdev;
	tbo->eyes = head_xdev;
	tbo->face = head_xdev;

	// Hand tracking: emulated hand controllers from the bottom cameras (PSVR2_HAND_TRACKING). Added after the
	// head: devices are destroyed in order, so the headset's USB thread (which feeds the hand tracker) has stopped
	// before the hand tracker goes.
	struct xrt_device *left_ht = NULL, *right_ht = NULL;
#ifdef XRT_BUILD_DRIVER_HANDTRACKING
	if (head_xdev != NULL && psvr2_hand_tracking_enabled()) {
		struct xrt_device *ht = psvr2_hand_tracking_create(head_xdev);
		struct xrt_device *two_hands[2] = {NULL, NULL};
		if (ht != NULL && cemu_devices_create(head_xdev, ht, two_hands) == 2) {
			psvr2_hmd_attach_hand_tracking(head_xdev, ht);
			left_ht = two_hands[0];
			right_ht = two_hands[1];
			xsysd->static_xdevs[xsysd->static_xdev_count++] = left_ht;
			xsysd->static_xdevs[xsysd->static_xdev_count++] = right_ht;
		} else {
			PSVR2_ERROR(pb, "PS VR2 hand tracking device creation failed");
			if (ht != NULL) {
				xrt_device_destroy(&ht);
			}
		}
	}
#endif
	tbo->hand_tracking.unobstructed.left = left_ht;
	tbo->hand_tracking.unobstructed.right = right_ht;
	// Hands take the controller roles when no Sense controller is there.
	tbo->left = left_ht;
	tbo->right = right_ht;

#ifdef XRT_BUILD_DRIVER_PSSENSE

	struct xrt_device *left_xdev = NULL;
	struct xrt_prober_device *left_xpdev =
	    u_builder_find_prober_device(xpdevs, xpdev_count, PSSENSE_VID, PSSENSE_PID_LEFT, XRT_BUS_TYPE_ANY);
	if (left_xpdev != NULL) {
		struct t_timing_event_sink *timing_sink;
		left_xdev = pssense_create(xp, left_xpdev, xfctx, &timing_sink);
		if (left_xdev == NULL) {
			PSVR2_ERROR(psvr2_builder(xb), "PS Sense left controller device creation failed");
		} else {
			xsysd->static_xdevs[xsysd->static_xdev_count++] = left_xdev;
			t_timing_event_source_add_sink(pb->timing_event_source, timing_sink);
			// pssense_add_to_constellation_tracker(left_xdev, pb->constellation_tracker);
		}
	}

	struct xrt_device *right_xdev = NULL;
	struct xrt_prober_device *right_xpdev =
	    u_builder_find_prober_device(xpdevs, xpdev_count, PSSENSE_VID, PSSENSE_PID_RIGHT, XRT_BUS_TYPE_ANY);
	if (right_xpdev != NULL) {
		struct t_timing_event_sink *timing_sink;
		right_xdev = pssense_create(xp, right_xpdev, xfctx, &timing_sink);
		if (right_xdev == NULL) {
			PSVR2_ERROR(psvr2_builder(xb), "PS Sense right controller device creation failed");
		} else {
			xsysd->static_xdevs[xsysd->static_xdev_count++] = right_xdev;
			t_timing_event_source_add_sink(pb->timing_event_source, timing_sink);
			// pssense_add_to_constellation_tracker(right_xdev, pb->constellation_tracker);
		}
	}

	if (left_xdev != NULL) {
		tbo->left = left_xdev;
	}
	if (right_xdev != NULL) {
		tbo->right = right_xdev;
	}
	((struct psvr2_hmd *)head_xdev)->pssense_left = left_xdev;
	((struct psvr2_hmd *)head_xdev)->pssense_right = right_xdev;
#endif

	xret = xrt_prober_unlock_list(xp, &xpdevs);
	if (xret != XRT_SUCCESS) {
		goto fail;
	}

	return XRT_SUCCESS;


unlock_and_fail:
	xret = xrt_prober_unlock_list(xp, &xpdevs);
	if (xret != XRT_SUCCESS) {
		return xret;
	}

	/* Fallthrough */
fail:
	return XRT_ERROR_DEVICE_CREATION_FAILED;
}

static void
psvr2_destroy(struct xrt_builder *xb)
{
	free(xb);
}


/*
 *
 * 'Exported' functions.
 *
 */

struct xrt_builder *
t_builder_psvr2_create(void)
{
	struct psvr2_builder *pb = U_TYPED_CALLOC(struct psvr2_builder);

	pb->log_level = debug_get_log_option_psvr2_log();

	// xrt_builder fields.
	pb->base.base.estimate_system = psvr2_estimate_system;
	pb->base.base.open_system = t_builder_open_system_static_roles;
	pb->base.base.destroy = psvr2_destroy;
	pb->base.base.identifier = "psvr2";
	pb->base.base.name = "PlayStation VR 2";
	pb->base.base.driver_identifiers = driver_list;
	pb->base.base.driver_identifier_count = ARRAY_SIZE(driver_list);

	// t_builder fields.
	pb->base.open_system_static_roles = psvr2_open_system_impl;

	return &pb->base.base;
}
