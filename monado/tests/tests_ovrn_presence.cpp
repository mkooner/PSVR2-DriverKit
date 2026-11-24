// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Tests for the SteamVR plugin's headset presence log (ovrn_presence_log.hpp): wear-sensor spells, head pose
 *         drops and SteamVR's standby verdicts, summed per 30 s window and per session.
 */

#include "ovrn_presence_log.hpp"

#include "catch_amalgamated.hpp"

#include <string>

using ovrn::PresenceLog;

namespace {

constexpr int64_t MS = 1000000LL, S = 1000LL * MS;

bool
has(const std::vector<std::string> &lines, const std::string &text)
{
	for (const std::string &l : lines) {
		if (l.find(text) != std::string::npos) {
			return true;
		}
	}
	return false;
}

} // namespace

TEST_CASE("presence: a steady session logs only the first value and a quiet total")
{
	PresenceLog p;
	for (int64_t t = 0; t < 100 * S; t += 11 * MS) {
		p.worn(true, t);
		p.headPose(true, t);
		const auto lines = p.tick(t);
		if (t == 0) {
			REQUIRE(lines.size() == 1);
			CHECK(has(lines, "wear sensor at the start: worn"));
		} else {
			CHECK(lines.empty());
		}
	}
	const auto end = p.tick(100 * S, true);
	REQUIRE(end.size() == 1);
	CHECK(has(end, "always worn and tracked, no standby"));
}

TEST_CASE("presence: a flickering wear sensor is counted with its spells")
{
	PresenceLog p;
	p.worn(true, 0);
	p.headPose(true, 0);
	(void)p.tick(0);
	// 10 short "not worn" spells of 40 ms, one every second, then a 300 ms one.
	for (int i = 1; i <= 10; i++) {
		p.worn(false, i * S);
		p.worn(true, i * S + 40 * MS);
	}
	p.worn(false, 20 * S);
	p.worn(true, 20 * S + 300 * MS);
	CHECK(p.tick(29 * S).empty()); // the window isn't over yet
	const auto lines = p.tick(30 * S);
	REQUIRE(lines.size() == 1);
	CHECK(has(lines, "in the last 30 s"));
	CHECK(has(lines, "wear sensor 22 changes, 11 not-worn spells (shortest 0.040 s, longest 0.300 s), 0.7 s not worn"));
	CHECK(has(lines, "head pose 0 drops"));
	// Nothing in the next window: no line.
	CHECK(p.tick(60 * S).empty());
	const auto end = p.tick(61 * S, true);
	CHECK(has(end, "this session (61 s): wear sensor 22 changes, 11 not-worn spells"));
}

TEST_CASE("presence: not worn across a window boundary is split between the windows")
{
	PresenceLog p;
	p.worn(true, 0);
	(void)p.tick(0);
	p.worn(false, 25 * S);
	const auto first = p.tick(30 * S);
	CHECK(has(first, "0 not-worn spells (shortest 0.000 s, longest 0.000 s), 5.0 s not worn"));
	p.worn(true, 40 * S);
	const auto second = p.tick(60 * S);
	CHECK(has(second, "1 not-worn spells (shortest 15.000 s, longest 15.000 s), 10.0 s not worn"));
}

TEST_CASE("presence: a headset that starts not worn and stays so is reported")
{
	PresenceLog p;
	p.worn(false, 0);
	const auto start = p.tick(0);
	CHECK(has(start, "not worn (SteamVR may treat the headset as idle)"));
	const auto w = p.tick(30 * S);
	CHECK(has(w, "0 changes, 0 not-worn spells (shortest 0.000 s, longest 0.000 s), 30.0 s not worn"));
}

TEST_CASE("presence: head pose drops and SteamVR's verdicts")
{
	PresenceLog p;
	p.worn(true, 0);
	(void)p.tick(0);
	for (int64_t t = 0; t < 30 * S; t += 2 * MS) {
		// Two drops: 100 ms at 5 s, 250 ms at 12 s.
		const bool bad = (t >= 5 * S && t < 5 * S + 100 * MS) || (t >= 12 * S && t < 12 * S + 250 * MS);
		p.headPose(!bad, t);
	}
	p.event(PresenceLog::UserInteractionEnded, 6 * S);
	p.event(PresenceLog::StandbyEntered, 6 * S);
	p.event(PresenceLog::DriverEnterStandby, 6 * S);
	p.event(PresenceLog::StandbyLeft, 7 * S);
	p.event(PresenceLog::UserInteractionStarted, 7 * S);
	const auto lines = p.tick(30 * S);
	REQUIRE(lines.size() == 1);
	CHECK(has(lines, "head pose 2 drops out of Running_OK (longest 0.250 s), 0.3 s not tracked"));
	CHECK(has(lines, "user interaction ended 1 / started 1, standby entered 1 / left 1, driver EnterStandby 1 / "
	                 "LeaveStandby 0"));
}

TEST_CASE("presence: without a wear sensor the total says so")
{
	PresenceLog p;
	p.headPose(true, 0);
	CHECK(p.tick(0).empty());
	const auto end = p.tick(40 * S, true);
	REQUIRE(end.size() == 1);
	CHECK(has(end, "no wear sensor and tracked, no standby"));
}
