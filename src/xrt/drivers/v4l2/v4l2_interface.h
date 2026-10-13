// Copyright 2019-2022, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Header
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup drv_v4l2
 */

#pragma once

#include "xrt/xrt_data_source.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_v4l2 V4L2 data source driver
 * @ingroup drv
 *
 * @brief DataSource using the Video 4 Linux 2 framework.
 */


/*!
 * Descriptor of a v4l2 source.
 *
 * @ingroup drv_v4l2
 * @extends xrt_ds_frame_mode
 */
struct v4l2_source_descriptor
{
	struct xrt_ds_frame_mode base;

	char format_name[32];

	struct
	{
		uint32_t width;
		uint32_t height;
		uint32_t format;
		uint8_t extended_format;

		size_t size;
		size_t stride;
	} stream;

	/*!
	 * Offset from start off frame to start of pixels.
	 *
	 * Also known as crop_scanline_bytes_start.
	 *
	 * Special case for ps4 camera
	 */
	size_t offset;
	uint32_t rate;
};



/*!
 * Create a v4l2 data source
 *
 * @ingroup drv_v4l2
 */
struct xrt_ds *
v4l2_ds_create(struct xrt_frame_context *xfctx,
               const char *path,
               const char *product,
               const char *manufacturer,
               const char *serial);


#ifdef __cplusplus
}
#endif
