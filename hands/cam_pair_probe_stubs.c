// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Stand-ins for cam_pair_probe: the controller tracker's hooks, never reached without controllers.
#include <stdlib.h>
void *pssense_add_to_constellation_tracker(void) { abort(); }
void *psvr2_blobwatch_create(void) { abort(); }
void *t_constellation_tracker_create(void) { abort(); }
void *t_constellation_tracker_get_tracking_origin(void) { abort(); }
#ifndef HAVE_AUX_TRACKING // driver_ht_replay links the real one (aux_tracking)
void *t_camera_model_params_from_t_camera_calibration(void) { abort(); }
#endif
