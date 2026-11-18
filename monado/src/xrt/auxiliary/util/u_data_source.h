// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Misc helpers for @ref xrt_ds implementations.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup aux_util
 */

#pragma once

#include "xrt/xrt_data_source.h"


#ifdef __cplusplus
extern "C" {
#endif


/*!
 * Populate the data source's function pointers with default implementations.
 *
 * Fills in all data source function pointers with not-implemented versions,
 * allowing implementations to override only the functions they implement.
 *
 * @param[in,out] xds Pointer to the data source to populate.
 * @ingroup aux_util
 */
void
u_data_source_populate_function_pointers(struct xrt_ds *xds);


#ifdef __cplusplus
}
#endif
