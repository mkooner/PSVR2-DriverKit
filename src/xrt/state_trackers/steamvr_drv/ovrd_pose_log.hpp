// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Records every headset pose the SteamVR plugin hands SteamVR (OVRD_POSE_LOG=path), for telling tracking
 *         jitter from compositor judder (psvr2-test/analysis/steamvr_timeline.py lines it up with tools/vrframetiming).
 *
 * The pose thread only copies a record into a ring (no locks, no I/O); a writer thread turns them into CSV every
 * 100 ms. If the writer falls behind, records are dropped and counted, never waited for.
 *
 * @ingroup st_ovrd
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>

class OvrdPoseLog
{
public:
	struct Record
	{
		int64_t submit_ns;   //!< CLOCK_MONOTONIC, when the pose was handed to SteamVR (its time: poseTimeOffset 0)
		int64_t get_pose_ns; //!< time spent getting the pose from the device (cost of GetPose)
		int64_t target_ns;   //!< the time the pose is for (= submit time with OVRD_POSE_MODE=steamvr)
		float nq[4], np[3];  //!< the pose for now (= the submitted one with OVRD_POSE_MODE=steamvr): the head's motion
		float q[4];          //!< qRotation x y z w (driver space)
		float p[3];          //!< vecPosition
		float v[3];          //!< vecVelocity (driver world space)
		float w[3];          //!< vecAngularVelocity (head-local, as sent)
		uint8_t valid, result, pos_tracked;
	};

	bool
	open(const char *path)
	{
		m_file = fopen(path, "w");
		if (m_file == nullptr) {
			return false;
		}
		fprintf(m_file, "# SteamVR plugin headset poses as submitted (CLOCK_MONOTONIC ns)\n");
		fprintf(m_file, "submit_ns,get_pose_us,valid,result,pos_tracked,qx,qy,qz,qw,px,py,pz,vx,vy,vz,wx,wy,wz,target_ns,nqx,nqy,nqz,nqw,npx,npy,npz\n");
		memset((void *)m_ring, 0, sizeof(m_ring)); // fault the ring's pages in now, not in the pose thread
		m_run = true;
		m_thread = std::thread(&OvrdPoseLog::writer, this);
		return true;
	}

	bool
	is_open() const
	{
		return m_file != nullptr;
	}

	//! Pose thread: copy only.
	void
	push(const Record &r)
	{
		uint64_t h = m_head.load(std::memory_order_relaxed);
		if (h - m_tail.load(std::memory_order_acquire) >= N) {
			m_dropped.fetch_add(1, std::memory_order_relaxed);
			return;
		}
		m_ring[h % N] = r;
		m_head.store(h + 1, std::memory_order_release);
	}

	void
	close()
	{
		if (m_file == nullptr) {
			return;
		}
		m_run = false;
		if (m_thread.joinable()) {
			m_thread.join();
		}
		flush();
		fprintf(m_file, "# dropped %llu records\n", (unsigned long long)m_dropped.load());
		fclose(m_file);
		m_file = nullptr;
	}

	~OvrdPoseLog()
	{
		close();
	}

private:
	static constexpr uint64_t N = 8192; // 8 s at 1 kHz

	void
	flush()
	{
		uint64_t t = m_tail.load(std::memory_order_relaxed);
		uint64_t h = m_head.load(std::memory_order_acquire);
		for (; t < h; t++) {
			const Record &r = m_ring[t % N];
			fprintf(m_file, "%lld,%.1f,%u,%u,%u,%.7f,%.7f,%.7f,%.7f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%lld,%.7f,%.7f,%.7f,%.7f,%.6f,%.6f,%.6f\n",
			        (long long)r.submit_ns, r.get_pose_ns / 1000.0, r.valid, r.result, r.pos_tracked, r.q[0],
			        r.q[1], r.q[2], r.q[3], r.p[0], r.p[1], r.p[2], r.v[0], r.v[1], r.v[2], r.w[0], r.w[1], r.w[2], (long long)r.target_ns, r.nq[0], r.nq[1], r.nq[2], r.nq[3], r.np[0], r.np[1], r.np[2]);
			m_tail.store(t + 1, std::memory_order_release);
		}
		fflush(m_file);
	}

	void
	writer()
	{
		while (m_run) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			flush();
		}
	}

	FILE *m_file = nullptr;
	Record m_ring[N];
	std::atomic<uint64_t> m_head{0}, m_tail{0}, m_dropped{0};
	std::atomic<bool> m_run{false};
	std::thread m_thread;
};
