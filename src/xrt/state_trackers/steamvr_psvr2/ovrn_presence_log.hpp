// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Logging only: what decides whether SteamVR shows the headset's picture, summed up every 30 s.
 *
 * A player's display pulsed bright/dark (2026-11-23, Intel Arc A750) at 89 frames a second with nothing late, in
 * SteamVR Home as in games, where Sony's driver on the same PC doesn't. What our plugin hands SteamVR that can make it
 * dim, fade or blank the view, none of which the log showed after the first pose:
 *  - the wear sensor (/proximity, sent raw at every change): per the OpenVR wiki (Device sleep states) an HMD whose
 *    /proximity is false is idle once it is still for the HMD timeout (5 s by default), and SteamVR then puts it in
 *    standby; apps read the same value for "user present";
 *  - the head pose dropping out of Running_OK (SteamVR fades toward its grey background while tracking is lost);
 *  - SteamVR's own verdicts: the headset's user interaction ending or starting (VREvent_TrackedDeviceUserInteraction-
 *    Ended/Started), standby entered or left (VREvent_EnterStandbyMode/LeaveStandbyMode, EnterStandby on the driver).
 * One line per 30 s window in which any of these happened or the headset spent time not worn or not tracked, nothing
 * otherwise; a session total at shutdown. No behaviour changes.
 *
 * Threads: worn() and event()/tick() on SteamVR's RunFrame thread, headPose() on the pose thread; the mutex is only
 * taken when the pose's state changes, not per pose.
 * @ingroup st_ovrn
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

namespace ovrn {

class PresenceLog
{
public:
	//! SteamVR's verdicts, counted per window.
	enum Event
	{
		UserInteractionStarted = 0, //!< VREvent_TrackedDeviceUserInteractionStarted for the headset
		UserInteractionEnded,       //!< VREvent_TrackedDeviceUserInteractionEnded for the headset
		StandbyEntered,             //!< VREvent_EnterStandbyMode
		StandbyLeft,                //!< VREvent_LeaveStandbyMode
		DriverEnterStandby,         //!< EnterStandby called on our driver (provider or headset)
		DriverLeaveStandby,         //!< LeaveStandby called on our provider
		kEventCount,
	};

	static constexpr int64_t kWindowNs = 30LL * 1000000000LL;

	//! The wear sensor's value as sent to SteamVR (RunFrame thread).
	void
	worn(bool w, int64_t now)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		startWindow(now);
		if (!m_wornKnown) {
			m_wornKnown = true;
			m_worn = w;
			m_notWornSince = w ? -1 : now;
			m_firstWorn = w ? 1 : 0;
			return;
		}
		if (w == m_worn) {
			return;
		}
		m_worn = w;
		m_win.wornChanges++;
		if (!w) {
			m_notWornSince = now;
			return;
		}
		const int64_t spell = now - m_notWornSince;
		m_win.notWornSpells++;
		m_win.shortestNotWorn = m_win.notWornSpells == 1 ? spell : std::min(m_win.shortestNotWorn, spell);
		m_win.longestNotWorn = std::max(m_win.longestNotWorn, spell);
		m_win.notWornNs += now - std::max(m_notWornSince, m_windowStart);
		m_notWornSince = -1;
	}

	//! Whether the head pose sent now is valid and Running_OK (pose thread; locks only on a change).
	void
	headPose(bool ok, int64_t now)
	{
		if (m_poseKnownLocal && ok == m_poseOkLocal) {
			return;
		}
		const bool first = !m_poseKnownLocal;
		m_poseKnownLocal = true;
		m_poseOkLocal = ok;
		std::lock_guard<std::mutex> lock(m_mutex);
		startWindow(now);
		if (first) {
			m_badSince = ok ? -1 : now;
			return;
		}
		if (!ok) {
			m_win.poseDrops++;
			m_badSince = now;
			return;
		}
		const int64_t spell = now - m_badSince;
		m_win.longestBad = std::max(m_win.longestBad, spell);
		m_win.badNs += now - std::max(m_badSince, m_windowStart);
		m_badSince = -1;
	}

	//! One of SteamVR's verdicts (RunFrame thread, or SteamVR's thread calling EnterStandby).
	void
	event(Event e, int64_t now)
	{
		if (e < 0 || e >= kEventCount) {
			return;
		}
		std::lock_guard<std::mutex> lock(m_mutex);
		startWindow(now);
		m_win.events[e]++;
	}

	/*!
	 * The lines to log for now (RunFrame thread): the window's summary once 30 s have passed and something happened
	 * in it, the wear sensor's first value once, and at shutdown what's left plus the session totals.
	 */
	std::vector<std::string>
	tick(int64_t now, bool at_shutdown = false)
	{
		std::vector<std::string> out;
		std::lock_guard<std::mutex> lock(m_mutex);
		startWindow(now);
		if (m_firstWorn >= 0) {
			out.push_back(std::string("ovrn: headset wear sensor at the start: ") +
			              (m_firstWorn ? "worn" : "not worn (SteamVR may treat the headset as idle)"));
			m_firstWorn = -1;
		}
		if (!at_shutdown && now - m_windowStart < kWindowNs) {
			return out;
		}
		// Time still not worn / not tracked at the end of the window belongs to it; the rest goes to the next one.
		if (m_notWornSince >= 0) {
			m_win.notWornNs += now - std::max(m_notWornSince, m_windowStart);
		}
		if (m_badSince >= 0) {
			m_win.badNs += now - std::max(m_badSince, m_windowStart);
		}
		const double secs = (double)(now - m_windowStart) / 1e9;
		if (m_win.any()) {
			out.push_back(summary(m_win, at_shutdown ? "since the last summary" : "in the last 30 s", secs));
		}
		m_total.add(m_win);
		m_totalNs += now - m_windowStart;
		m_win = Window();
		m_windowStart = now;
		if (at_shutdown) {
			if (m_total.any()) {
				out.push_back(summary(m_total, "this session", (double)m_totalNs / 1e9));
			} else {
				out.push_back(std::string("ovrn: headset presence this session: ") +
				              (m_wornKnown ? "always worn" : "no wear sensor") + " and tracked, no standby");
			}
		}
		return out;
	}

private:
	struct Window
	{
		uint32_t wornChanges = 0, notWornSpells = 0, poseDrops = 0;
		int64_t shortestNotWorn = 0, longestNotWorn = 0, notWornNs = 0, longestBad = 0, badNs = 0;
		uint32_t events[kEventCount] = {};

		bool
		any() const
		{
			uint32_t e = 0;
			for (uint32_t v : events) {
				e += v;
			}
			return wornChanges || poseDrops || notWornNs > 0 || badNs > 0 || e;
		}

		void
		add(const Window &w)
		{
			if (w.notWornSpells > 0) {
				shortestNotWorn = notWornSpells == 0 ? w.shortestNotWorn : std::min(shortestNotWorn, w.shortestNotWorn);
			}
			wornChanges += w.wornChanges;
			notWornSpells += w.notWornSpells;
			poseDrops += w.poseDrops;
			longestNotWorn = std::max(longestNotWorn, w.longestNotWorn);
			notWornNs += w.notWornNs;
			longestBad = std::max(longestBad, w.longestBad);
			badNs += w.badNs;
			for (int i = 0; i < kEventCount; i++) {
				events[i] += w.events[i];
			}
		}
	};

	static std::string
	summary(const Window &w, const char *when, double secs)
	{
		char buf[640];
		snprintf(buf, sizeof(buf),
		         "ovrn: headset presence %s (%.0f s): wear sensor %u changes, %u not-worn spells (shortest %.3f s, "
		         "longest %.3f s), %.1f s not worn; head pose %u drops out of Running_OK (longest %.3f s), %.1f s not "
		         "tracked; SteamVR: user interaction ended %u / started %u, standby entered %u / left %u, driver "
		         "EnterStandby %u / LeaveStandby %u",
		         when, secs, w.wornChanges, w.notWornSpells, (double)w.shortestNotWorn / 1e9,
		         (double)w.longestNotWorn / 1e9, (double)w.notWornNs / 1e9, w.poseDrops, (double)w.longestBad / 1e9,
		         (double)w.badNs / 1e9, w.events[UserInteractionEnded], w.events[UserInteractionStarted],
		         w.events[StandbyEntered], w.events[StandbyLeft], w.events[DriverEnterStandby],
		         w.events[DriverLeaveStandby]);
		return buf;
	}

	void
	startWindow(int64_t now)
	{
		if (m_windowStart < 0) {
			m_windowStart = now;
		}
	}

	std::mutex m_mutex;
	int64_t m_windowStart = -1, m_totalNs = 0;
	Window m_win, m_total;
	bool m_wornKnown = false, m_worn = false;
	int m_firstWorn = -1; //!< the first value until logged: 1 worn, 0 not worn, -1 none / logged
	int64_t m_notWornSince = -1, m_badSince = -1;
	// Pose thread only.
	bool m_poseKnownLocal = false, m_poseOkLocal = false;
};

} // namespace ovrn
