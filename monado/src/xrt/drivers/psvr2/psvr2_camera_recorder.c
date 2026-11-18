// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Opt-in recording of the PS VR2 camera image transfers, see psvr2_camera_recorder.h.
 * @ingroup drv_psvr2
 */

#include "psvr2_camera_recorder.h"

#include "os/os_threading.h"
#include "util/u_logging.h"
#include "util/u_misc.h"

#include <stdio.h>
#include <string.h>

#define RECORDER_SLOTS 32

struct recorder_slot
{
	bool full;
	int64_t host_ns;
	uint32_t length;
	uint32_t dropped_before;
	uint8_t *data;
};

struct psvr2_camera_recorder
{
	struct os_thread_helper thread;
	FILE *file;
	size_t max_bytes;
	int64_t min_interval_ns;
	int64_t last_accept_ns;

	struct recorder_slot slots[RECORDER_SLOTS];
	uint32_t write_index; // next slot the USB thread fills
	uint32_t read_index;  // next slot the writer writes
	uint32_t dropped;     // dropped since the last accepted transfer
	uint64_t written;
	uint64_t dropped_total;
};

static void *
recorder_thread(void *ptr)
{
	struct psvr2_camera_recorder *rec = (struct psvr2_camera_recorder *)ptr;
	os_thread_helper_lock(&rec->thread);
	for (;;) {
		struct recorder_slot *slot = &rec->slots[rec->read_index];
		if (!slot->full) {
			if (!os_thread_helper_is_running_locked(&rec->thread)) {
				break; // stopped and drained
			}
			os_thread_helper_wait_locked(&rec->thread);
			continue;
		}
		os_thread_helper_unlock(&rec->thread);

		uint64_t host = (uint64_t)slot->host_ns;
		fwrite(&host, sizeof(host), 1, rec->file);
		fwrite(&slot->length, sizeof(slot->length), 1, rec->file);
		fwrite(&slot->dropped_before, sizeof(slot->dropped_before), 1, rec->file);
		fwrite(slot->data, 1, slot->length, rec->file);

		os_thread_helper_lock(&rec->thread);
		slot->full = false;
		rec->read_index = (rec->read_index + 1) % RECORDER_SLOTS;
		rec->written++;
	}
	os_thread_helper_unlock(&rec->thread);
	return NULL;
}

struct psvr2_camera_recorder *
psvr2_camera_recorder_create(const char *path, float max_hz, size_t max_transfer_bytes)
{
	struct psvr2_camera_recorder *rec = U_TYPED_CALLOC(struct psvr2_camera_recorder);
	if (rec == NULL) {
		return NULL;
	}
	rec->file = fopen(path, "wb");
	if (rec->file == NULL) {
		U_LOG_E("PSVR2_RECORD_CAMERA: can't write %s", path);
		free(rec);
		return NULL;
	}
	fwrite("PV2C", 1, 4, rec->file);
	uint32_t version = 1;
	fwrite(&version, sizeof(version), 1, rec->file);

	rec->max_bytes = max_transfer_bytes;
	rec->min_interval_ns = max_hz > 0.0f ? (int64_t)(1e9 / max_hz) : 0;
	for (int i = 0; i < RECORDER_SLOTS; i++) {
		rec->slots[i].data = U_TYPED_ARRAY_CALLOC(uint8_t, max_transfer_bytes);
		if (rec->slots[i].data == NULL) {
			psvr2_camera_recorder_destroy(&rec);
			return NULL;
		}
	}
	if (os_thread_helper_init(&rec->thread) != 0 || os_thread_helper_start(&rec->thread, recorder_thread, rec) != 0) {
		psvr2_camera_recorder_destroy(&rec);
		return NULL;
	}
	U_LOG_I("Recording camera transfers to %s (at most %.0f/s)", path, max_hz);
	return rec;
}

void
psvr2_camera_recorder_push(struct psvr2_camera_recorder *rec, int64_t host_ns, const uint8_t *data, size_t length)
{
	if (rec == NULL || length > rec->max_bytes) {
		return;
	}
	// Rate limit (not a drop), with 10% slack so frames arriving at a multiple just under the interval aren't skipped.
	if (rec->min_interval_ns > 0 && host_ns - rec->last_accept_ns < rec->min_interval_ns * 9 / 10) {
		return;
	}
	os_thread_helper_lock(&rec->thread);
	struct recorder_slot *slot = &rec->slots[rec->write_index];
	if (slot->full) {
		rec->dropped++;
		rec->dropped_total++;
		os_thread_helper_unlock(&rec->thread);
		return;
	}
	os_thread_helper_unlock(&rec->thread);

	// The writer never touches a slot that isn't full, so this copy needs no lock.
	memcpy(slot->data, data, length);
	slot->length = (uint32_t)length;
	slot->host_ns = host_ns;

	os_thread_helper_lock(&rec->thread);
	slot->dropped_before = rec->dropped;
	rec->dropped = 0;
	slot->full = true;
	rec->write_index = (rec->write_index + 1) % RECORDER_SLOTS;
	rec->last_accept_ns = host_ns;
	os_thread_helper_signal_locked(&rec->thread);
	os_thread_helper_unlock(&rec->thread);
}

void
psvr2_camera_recorder_destroy(struct psvr2_camera_recorder **rec_ptr)
{
	struct psvr2_camera_recorder *rec = *rec_ptr;
	if (rec == NULL) {
		return;
	}
	// Stop: the writer drains the full slots, then exits.
	os_thread_helper_lock(&rec->thread);
	bool running = os_thread_helper_is_running_locked(&rec->thread);
	os_thread_helper_unlock(&rec->thread);
	if (running) {
		os_thread_helper_stop_and_wait(&rec->thread);
	}
	os_thread_helper_destroy(&rec->thread);
	if (rec->file != NULL) {
		U_LOG_I("Camera recording closed: %llu transfers written, %llu dropped (writer behind)",
		        (unsigned long long)rec->written, (unsigned long long)rec->dropped_total);
		fclose(rec->file);
	}
	for (int i = 0; i < RECORDER_SLOTS; i++) {
		free(rec->slots[i].data);
	}
	free(rec);
	*rec_ptr = NULL;
}
