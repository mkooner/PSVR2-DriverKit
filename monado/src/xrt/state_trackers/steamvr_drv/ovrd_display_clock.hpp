// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  SteamVR's vsync grid in CLOCK_MONOTONIC, from the compositor frame timings a driver can read
 *         (IVRServerDriverHost::GetFrameTimings), and the display time to predict a headset pose to.
 *
 * Each frame timing has its vsync time on SteamVR's clock (m_flSystemTimeInSeconds), and the newest entry appears
 * once its vsync has happened, so (now - newest t_s) over many reads is never below the clock offset and is close to it
 * on the read right after a vsync: the smallest value over the last two seconds is the offset (to the read interval,
 * ~1 ms at the pose thread's rate).
 *
 * Used for display-time poses (OVRD_POSE_MODE=display, as ALVR's driver does): the driver predicts the pose to when it
 * will be shown and sends no velocities, so the pose a frame gets doesn't depend on which target time SteamVR picks for
 * it. Shared with the offline test psvr2-test/analysis/harness/display_clock_test.cpp.
 *
 * @ingroup st_ovrd
 */

#pragma once

#include <cmath>
#include <cstdint>

class OvrdDisplayClock
{
public:
	//! @p period_ns the refresh period (Prop_DisplayFrequency).
	explicit OvrdDisplayClock(int64_t period_ns = 11122233) : m_period_ns(period_ns) {}

	void
	set_period(int64_t period_ns)
	{
		m_period_ns = period_ns;
	}

	/*!
	 * A read of the newest frame timing at @p now_ns (CLOCK_MONOTONIC): its vsync time @p t_s on SteamVR's clock and its
	 * frame index.
	 */
	void
	observe(int64_t now_ns, double t_s, uint32_t frame_index)
	{
		if (t_s <= 0.0) {
			return;
		}
		double cand = (double)now_ns * 1e-9 - t_s;
		if (!m_have || now_ns - m_window_start_ns > 2000000000LL) {
			// New two-second window; keep the last window's minimum so a window start doesn't lose it.
			m_prev_min = m_have ? m_cur_min : cand;
			m_cur_min = cand;
			m_window_start_ns = now_ns;
		} else if (cand < m_cur_min) {
			m_cur_min = cand;
		}
		m_have = true;
		m_last_t_s = t_s;
		m_last_frame = frame_index;
		m_reads++;
	}

	bool
	ready() const
	{
		return m_have && m_reads > 200; // ~0.2 s of reads at 1 kHz
	}

	//! SteamVR clock + offset = CLOCK_MONOTONIC (seconds).
	double
	offset_s() const
	{
		return m_cur_min < m_prev_min ? m_cur_min : m_prev_min;
	}

	//! The newest vsync in CLOCK_MONOTONIC ns.
	int64_t
	last_vsync_ns() const
	{
		return (int64_t)std::llround((m_last_t_s + offset_s()) * 1e9);
	}

	/*!
	 * The display time for a pose submitted at @p now_ns: the first vsync after now + @p cut_ns, plus @p after_vsync_ns
	 * (when that refresh's light reaches the eyes). The cut keeps the switch to the next refresh away from the moment the
	 * app asks for its pose, so the target doesn't flip between two refreshes on small timing differences.
	 */
	int64_t
	display_time_ns(int64_t now_ns, int64_t cut_ns, int64_t after_vsync_ns) const
	{
		int64_t v0 = last_vsync_ns();
		int64_t since = now_ns + cut_ns - v0;
		int64_t k = since < 0 ? 0 : since / m_period_ns + 1;
		return v0 + k * m_period_ns + after_vsync_ns;
	}

	uint32_t
	last_frame() const
	{
		return m_last_frame;
	}

private:
	int64_t m_period_ns;
	bool m_have = false;
	double m_cur_min = 0, m_prev_min = 0, m_last_t_s = 0;
	int64_t m_window_start_ns = 0;
	uint32_t m_last_frame = 0;
	uint64_t m_reads = 0;
};
