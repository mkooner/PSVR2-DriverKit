// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  A frame-timing recorder that can't disturb the frames it records (direct mode, OVRD_FRAME_LOG).
 *
 * On 2026-11-18 our own measurement (2 ms pose sampler, per-frame file writes on live threads, recordings) caused
 * most of the head shake it was meant to find. This recorder only copies a small fixed-size record into memory on
 * the live threads (SteamVR's submit thread, the frame thread): no file I/O, no pose queries, no device locks. A
 * writer thread at idle priority turns the records into text once a second. If the writer falls behind (slow disk),
 * records beyond kMaxPending are dropped and counted, never waited for.
 *
 * Rows (CSV, times CLOCK_MONOTONIC ns):
 *   P,t,layers,call_ns                         Present on SteamVR's thread: when, layers, time spent in the call
 *   F,woke,frame_id,display,vsync,present,wait_us,sync,flags,layers,set0,set1
 *       one per Monado frame from the frame thread: Monado's wake-up, its frame id and display time, when SteamVR was
 *       told about the vsync, the Present shown in it (0: none), the GPU wait, the sync state (ovrd_direct_mode.hpp),
 *       flags (1 submitted, 2 the Present came before this vsync, 4 no frame in time), the layer count and the
 *       first image of each of the first two layers' sets
 *   R,t                                        a Present replaced by a newer one before the frame thread took it
 *   Q,t,throttle,extra_predict                 PostPresent and SteamVR's throttling values
 *   D,t,count                                  records dropped since the last D row (writer behind)
 *   U,t,target,px,py,pz,qx,qy,qz,qw            a frame pose handed to SteamVR (once per display target)
 *   L,t,layer,label_target,used_target,game    SubmitLayer: which of our frame poses SteamVR's label is (0: none
 *                                              of the last 16), the pose given to Monado instead, 1 if the image
 *                                              is the application's (labels of SteamVR's own images are kept)
 */

#pragma once

#include <atomic>
#include <cinttypes>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <pthread.h>
#include <sched.h>
#include <string>
#include <thread>
#include <vector>

class OvrdFrameRecorder
{
public:
	struct Rec
	{
		char kind;
		int8_t sync;
		uint8_t flags;
		uint8_t layers;
		int32_t i;   //!< wait_us (F), call_ns (P), throttle (Q)
		int32_t j;   //!< extra_predict (Q)
		int64_t t0;  //!< t (P, R, Q), woke (F)
		int64_t t1;  //!< frame_id (F)
		int64_t t2;  //!< display (F)
		int64_t t3;  //!< vsync (F)
		int64_t t4;  //!< present (F)
		uint64_t h0; //!< set0 (F)
		uint64_t h1; //!< set1 (F)
		float f[7];  //!< pose (U)
	};
	static constexpr size_t kMaxPending = 1 << 18; //!< ~45 min of frames at 90 Hz; beyond it records are dropped

	~OvrdFrameRecorder()
	{
		stop();
	}

	bool
	start(const char *path)
	{
		if (path == NULL || path[0] == '\0' || m_file != NULL) {
			return false;
		}
		m_file = fopen(path, "w");
		if (m_file == NULL) {
			return false;
		}
		fprintf(m_file, "# OVRD_FRAME_LOG (ovrd_frame_recorder.hpp)\n");
		m_buf.reserve(4096);
		m_on = true;
		m_thread = std::thread(&OvrdFrameRecorder::writerLoop, this);
		return true;
	}

	void
	stop()
	{
		if (!m_thread.joinable()) {
			return;
		}
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_on = false;
		}
		m_cv.notify_all();
		m_thread.join();
		if (m_file != NULL) {
			fclose(m_file);
			m_file = NULL;
		}
	}

	bool
	on() const
	{
		return m_on.load(std::memory_order_relaxed);
	}

	//! The only call on live threads: copies @p r into memory.
	void
	push(const Rec &r)
	{
		if (!m_on.load(std::memory_order_relaxed)) {
			return;
		}
		std::lock_guard<std::mutex> lock(m_mutex);
		if (m_buf.size() >= kMaxPending) {
			m_dropped++;
			return;
		}
		m_buf.push_back(r);
	}

	//! For tests: stall the writer for @p ns per batch (a slow disk).
	std::atomic<int64_t> test_write_delay_ns{0};
	std::atomic<uint64_t> written{0};

	uint64_t
	dropped() const
	{
		return m_droppedTotal.load();
	}

private:
	void
	writerLoop()
	{
		// Idle priority: the writer only runs when nothing else wants the CPU.
		struct sched_param sp = {};
		pthread_setschedparam(pthread_self(), SCHED_IDLE, &sp);
		std::vector<Rec> batch;
		batch.reserve(4096);
		for (;;) {
			uint64_t dropped = 0;
			bool on;
			{
				std::unique_lock<std::mutex> lock(m_mutex);
				m_cv.wait_for(lock, std::chrono::seconds(1), [this]() { return !m_on.load(); });
				on = m_on.load();
				batch.swap(m_buf); // m_buf gets the (empty, reserved) batch vector
				dropped = m_dropped;
				m_dropped = 0;
			}
			if (test_write_delay_ns.load() > 0) {
				std::this_thread::sleep_for(std::chrono::nanoseconds(test_write_delay_ns.load()));
			}
			for (const Rec &r : batch) {
				writeRow(r);
			}
			written += batch.size();
			if (dropped != 0) {
				m_droppedTotal += dropped;
				fprintf(m_file, "D,%" PRId64 ",%" PRIu64 "\n", (int64_t)0, dropped);
			}
			fflush(m_file);
			batch.clear();
			if (!on) {
				std::lock_guard<std::mutex> lock(m_mutex);
				for (const Rec &r : m_buf) {
					writeRow(r);
				}
				written += m_buf.size();
				m_buf.clear();
				fflush(m_file);
				return;
			}
		}
	}

	void
	writeRow(const Rec &r)
	{
		switch (r.kind) {
		case 'P': fprintf(m_file, "P,%" PRId64 ",%u,%d\n", r.t0, r.layers, r.i); break;
		case 'F':
			fprintf(m_file, "F,%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%d,%d,%u,%u,%" PRIu64
			                ",%" PRIu64 "\n",
			        r.t0, r.t1, r.t2, r.t3, r.t4, r.i, r.sync, r.flags, r.layers, r.h0, r.h1);
			break;
		case 'R': fprintf(m_file, "R,%" PRId64 "\n", r.t0); break;
		case 'Q': fprintf(m_file, "Q,%" PRId64 ",%d,%d\n", r.t0, r.i, r.j); break;
		case 'U':
			fprintf(m_file, "U,%" PRId64 ",%" PRId64 ",%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f\n", r.t0, r.t1, r.f[0],
			        r.f[1], r.f[2], r.f[3], r.f[4], r.f[5], r.f[6]);
			break;
		case 'L':
			fprintf(m_file, "L,%" PRId64 ",%d,%" PRId64 ",%" PRId64 ",%u\n", r.t0, r.i, r.t1, r.t2, r.flags);
			break;
		default: break;
		}
	}

	FILE *m_file = NULL;
	std::atomic<bool> m_on{false};
	std::thread m_thread;
	std::mutex m_mutex;
	std::condition_variable m_cv;
	std::vector<Rec> m_buf;
	uint64_t m_dropped = 0;
	std::atomic<uint64_t> m_droppedTotal{0};
};
