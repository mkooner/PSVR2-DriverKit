// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Background map of fixed bright spots, see t_blob_background.h.
 * @ingroup aux_tracking
 */

#include "t_blob_background.h"

#include "math/m_api.h"
#include "math/m_vec3.h"
#include "util/u_misc.h"
#include "util/u_debug.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/*
 * Tunables, set from the PS VR2 recordings (psvr2-test/bgmap): with the head still, a fixed spot's blob stays within
 * ~1 px (~0.15 deg) between frames; head pose timing, pose noise and the parallax of near reflections while moving add
 * up to a degree or two, which TOL covers; the direction itself follows slowly (FOLLOW) for walking.
 */
#define BG_MAX_LANDMARKS 2048
DEBUG_GET_ONCE_FLOAT_OPTION(bg_tol_deg, "T_BLOB_BACKGROUND_TOL_DEG", 1.5f)
DEBUG_GET_ONCE_FLOAT_OPTION(bg_promote, "T_BLOB_BACKGROUND_PROMOTE", 0.6f)
DEBUG_GET_ONCE_FLOAT_OPTION(bg_demote, "T_BLOB_BACKGROUND_DEMOTE", 0.3f)
#define BG_TOL_RAD (debug_get_float_option_bg_tol_deg() * (float)M_PI / 180.0f)
#define BG_FOLLOW 0.2f
//! Presence: share of in-view frames with a blob on the direction, as a running average with this gain.
#define BG_PRESENCE_GAIN 0.1f
//! Background once present this much and seen at least BG_MIN_HITS times over BG_MIN_AGE_NS.
#define BG_PROMOTE debug_get_float_option_bg_promote()
#define BG_DEMOTE debug_get_float_option_bg_demote()
#define BG_MIN_HITS 12
#define BG_MIN_AGE_NS (300LL * 1000000LL)
//! Forgotten when this unlikely and not seen for BG_FORGET_NS.
#define BG_FORGET_PRESENCE 0.05f
#define BG_FORGET_NS (5LL * 1000000000LL)
//! A direction a controller LED was matched on can't become background for this long.
#define BG_VETO_NS (3LL * 1000000000LL)
//! Blobs this close (homogenized, ~0.1 = ~6 deg) to a matched blob in the same frame are the controller's too.
#define BG_VETO_RADIUS 0.10f
#define BG_MAX_CAMERAS 8
//! Grid over homogenized x, y in [-BG_GRID_RANGE, BG_GRID_RANGE]; a cell (0.0625) is wider than the tolerance.
#define BG_GRID 64
#define BG_GRID_RANGE 2.0f

static inline int
bg_cell_coord(float v)
{
	int c = (int)floorf((v + BG_GRID_RANGE) * (BG_GRID / (2.0f * BG_GRID_RANGE)));
	return c < 0 ? 0 : c >= BG_GRID ? BG_GRID - 1 : c;
}

struct bg_landmark
{
	struct xrt_vec3 dir; // unit, world
	float presence;
	uint32_t hits;
	int64_t first_ns, last_seen_ns, veto_until_ns;
	bool background;
	bool used; // slot in use
};

struct t_blob_background
{
	struct bg_landmark lm[BG_MAX_LANDMARKS];
	uint32_t num;
	//! Per camera: largest |x|, |y| of blob rays seen, for its field of view.
	float fov_x[BG_MAX_CAMERAS], fov_y[BG_MAX_CAMERAS];
	//! Scratch per frame.
	int32_t blob_lm[XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME];
	struct xrt_vec3 blob_dir[XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME];
	bool blob_veto[XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME];
	//! Image grid of this frame's blobs (homogenized coordinates), so a landmark only checks nearby blobs.
	uint16_t cell_start[BG_GRID * BG_GRID + 1];
	uint16_t cell_blobs[XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME];
	uint16_t blob_cell[XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME];
};

struct t_blob_background *
t_blob_background_create(void)
{
	return U_TYPED_CALLOC(struct t_blob_background);
}

void
t_blob_background_destroy(struct t_blob_background **bg_ptr)
{
	if (bg_ptr == NULL || *bg_ptr == NULL) {
		return;
	}
	free(*bg_ptr);
	*bg_ptr = NULL;
}

static int32_t
bg_alloc(struct t_blob_background *bg, int64_t now_ns)
{
	// A free slot, else the least present non-background one.
	int32_t worst = -1;
	float worst_p = 2.0f;
	for (uint32_t i = 0; i < BG_MAX_LANDMARKS; i++) {
		if (!bg->lm[i].used) {
			return (int32_t)i;
		}
		if (!bg->lm[i].background && bg->lm[i].presence < worst_p) {
			worst_p = bg->lm[i].presence;
			worst = (int32_t)i;
		}
	}
	(void)now_ns;
	return worst;
}

uint32_t
t_blob_background_process(struct t_blob_background *bg,
                          uint32_t camera_index,
                          int64_t timestamp_ns,
                          const struct xrt_quat *rot_world_cam,
                          const struct t_blob *blobs,
                          uint32_t num_blobs,
                          bool *out_background)
{
	if (num_blobs > XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME) {
		num_blobs = XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME;
	}
	uint32_t cam = camera_index < BG_MAX_CAMERAS ? camera_index : BG_MAX_CAMERAS - 1;
	const float cos_tol = cosf(BG_TOL_RAD);

	// Rays into the room, and which blobs belong to a matched controller.
	for (uint32_t b = 0; b < num_blobs; b++) {
		const struct t_blob *blob = &blobs[b];
		float hx = blob->center_homogenized.x, hy = blob->center_homogenized.y;
		out_background[b] = false;
		if (!isfinite(hx) || !isfinite(hy)) {
			// No ray (undistortion failed at the image edge): neither learned nor marked.
			bg->blob_dir[b] = (struct xrt_vec3){0, 0, 0};
			bg->blob_lm[b] = -1;
			bg->blob_veto[b] = true;
			continue;
		}
		if (fabsf(hx) > bg->fov_x[cam]) {
			bg->fov_x[cam] = fabsf(hx);
		}
		if (fabsf(hy) > bg->fov_y[cam]) {
			bg->fov_y[cam] = fabsf(hy);
		}
		// OpenCV camera axes (x right, y down, z forward) -> OpenXR (x right, y up, -z forward).
		struct xrt_vec3 ray = {hx, -hy, -1.0f};
		ray = m_vec3_normalize(ray);
		math_quat_rotate_vec3(rot_world_cam, &ray, &bg->blob_dir[b]);
		bg->blob_lm[b] = -1;
		bg->blob_veto[b] = blob->matched_device_id != XRT_CONSTELLATION_INVALID_DEVICE_ID;
		out_background[b] = false;
	}
	for (uint32_t b = 0; b < num_blobs; b++) {
		if (bg->blob_veto[b] || blobs[b].matched_device_id != XRT_CONSTELLATION_INVALID_DEVICE_ID) {
			continue;
		}
		for (uint32_t m = 0; m < num_blobs; m++) {
			if (blobs[m].matched_device_id == XRT_CONSTELLATION_INVALID_DEVICE_ID) {
				continue;
			}
			float dx = blobs[b].center_homogenized.x - blobs[m].center_homogenized.x;
			float dy = blobs[b].center_homogenized.y - blobs[m].center_homogenized.y;
			if (dx * dx + dy * dy < BG_VETO_RADIUS * BG_VETO_RADIUS) {
				bg->blob_veto[b] = true;
				break;
			}
		}
	}

	// Bucket the blobs (counting sort by cell).
	memset(bg->cell_start, 0, sizeof(bg->cell_start));
	for (uint32_t b = 0; b < num_blobs; b++) {
		float hx = blobs[b].center_homogenized.x, hy = blobs[b].center_homogenized.y;
		int cx = isfinite(hx) ? bg_cell_coord(hx) : 0, cy = isfinite(hy) ? bg_cell_coord(hy) : 0;
		bg->blob_cell[b] = (uint16_t)(cy * BG_GRID + cx);
		bg->cell_start[bg->blob_cell[b] + 1]++;
	}
	for (int c = 0; c < BG_GRID * BG_GRID; c++) {
		bg->cell_start[c + 1] += bg->cell_start[c];
	}
	{
		uint16_t fill[BG_GRID * BG_GRID];
		memcpy(fill, bg->cell_start, sizeof(fill));
		for (uint32_t b = 0; b < num_blobs; b++) {
			bg->cell_blobs[fill[bg->blob_cell[b]]++] = (uint16_t)b;
		}
	}

	// Associate each blob with the nearest landmark within the tolerance.
	struct xrt_quat inv;
	math_quat_invert(rot_world_cam, &inv);
	for (uint32_t i = 0; i < BG_MAX_LANDMARKS; i++) {
		struct bg_landmark *l = &bg->lm[i];
		if (!l->used) {
			continue;
		}
		// In this camera's view? (direction in camera axes, inside the field of view seen so far)
		struct xrt_vec3 c;
		math_quat_rotate_vec3(&inv, &l->dir, &c);
		if (c.z > -0.2f) {
			continue;
		}
		float px = c.x / -c.z, py = -c.y / -c.z;
		if (fabsf(px) > bg->fov_x[cam] * 0.95f || fabsf(py) > bg->fov_y[cam] * 0.95f) {
			continue;
		}
		int32_t best = -1;
		float best_dot = cos_tol;
		int gx = bg_cell_coord(px), gy = bg_cell_coord(py);
		for (int yy = gy - 1; yy <= gy + 1; yy++) {
			for (int xx = gx - 1; xx <= gx + 1; xx++) {
				if (xx < 0 || yy < 0 || xx >= BG_GRID || yy >= BG_GRID) {
					continue;
				}
				int cell = yy * BG_GRID + xx;
				for (uint16_t k = bg->cell_start[cell]; k < bg->cell_start[cell + 1]; k++) {
					uint16_t b = bg->cell_blobs[k];
					float d = m_vec3_dot(bg->blob_dir[b], l->dir);
					if (d > best_dot) {
						best_dot = d;
						best = (int32_t)b;
					}
				}
			}
		}
		bool present = best >= 0;
		if (present && bg->blob_veto[best]) {
			// A controller LED on this direction: not background, and not for a while.
			l->presence = 0.0f;
			l->background = false;
			l->veto_until_ns = timestamp_ns + BG_VETO_NS;
			bg->blob_lm[best] = (int32_t)i;
			continue;
		}
		l->presence += BG_PRESENCE_GAIN * ((present ? 1.0f : 0.0f) - l->presence);
		if (present) {
			l->hits++;
			l->last_seen_ns = timestamp_ns;
			struct xrt_vec3 d = bg->blob_dir[best];
			l->dir.x += BG_FOLLOW * (d.x - l->dir.x);
			l->dir.y += BG_FOLLOW * (d.y - l->dir.y);
			l->dir.z += BG_FOLLOW * (d.z - l->dir.z);
			l->dir = m_vec3_normalize(l->dir);
			if (bg->blob_lm[best] < 0) {
				bg->blob_lm[best] = (int32_t)i;
			}
		}
		if (!l->background && l->presence >= BG_PROMOTE && l->hits >= BG_MIN_HITS &&
		    timestamp_ns - l->first_ns >= BG_MIN_AGE_NS && timestamp_ns >= l->veto_until_ns) {
			l->background = true;
		} else if (l->background && l->presence < BG_DEMOTE) {
			l->background = false;
		}
		if (l->presence < BG_FORGET_PRESENCE && timestamp_ns - l->last_seen_ns > BG_FORGET_NS) {
			l->used = false;
			bg->num--;
		}
	}

	// New landmarks for unmatched blobs; mark the background ones.
	uint32_t marked = 0;
	for (uint32_t b = 0; b < num_blobs; b++) {
		int32_t i = bg->blob_lm[b];
		if (i < 0) {
			if (bg->blob_veto[b]) {
				continue;
			}
			i = bg_alloc(bg, timestamp_ns);
			if (i < 0) {
				continue;
			}
			struct bg_landmark *l = &bg->lm[i];
			if (!l->used) {
				bg->num++;
			}
			*l = (struct bg_landmark){
			    .dir = bg->blob_dir[b],
			    .presence = BG_PRESENCE_GAIN,
			    .hits = 1,
			    .first_ns = timestamp_ns,
			    .last_seen_ns = timestamp_ns,
			    .used = true,
			};
			continue;
		}
		if (!bg->blob_veto[b] && bg->lm[i].background) {
			out_background[b] = true;
			marked++;
		}
	}
	return marked;
}

void
t_blob_background_stats(struct t_blob_background *bg, uint32_t *out_landmarks, uint32_t *out_background)
{
	uint32_t n = 0, nb = 0;
	for (uint32_t i = 0; i < BG_MAX_LANDMARKS; i++) {
		if (bg->lm[i].used) {
			n++;
			nb += bg->lm[i].background ? 1 : 0;
		}
	}
	*out_landmarks = n;
	*out_background = nb;
}
