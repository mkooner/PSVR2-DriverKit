// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Tests for the gesture geometry (Mercury's hg_gesture_geometry.hpp) and the SteamVR plugin's gesture state
 *         machine (ovrn_gesture_state.hpp). All inputs here are hand-made test values (simple synthetic hands and
 *         measurement sequences), not tracking data; real-data checks are the replay runs on recordings.
 */

#include "hg_gesture_geometry.hpp"
#include "ovrn_gesture_state.hpp"

#include "catch_amalgamated.hpp"

#include <cmath>
#include <limits>

using namespace xrt::tracking::hand::mercury::gesture;
using namespace ovrn::gest;
using Catch::Approx;

namespace {

/*!
 * A synthetic open right hand in a plane (metres, x right, y up): palm 8 cm wide, straight fingers pointing up, the
 * thumb out to the side.
 */
Landmarks
open_hand()
{
	Landmarks lm;
	lm.valid = true;
	lm.p[WRIST] = {0.0f, 0.0f, 0.0f};
	// thumb out to the side
	lm.p[1] = {0.02f, 0.02f, 0};
	lm.p[2] = {0.045f, 0.035f, 0};
	lm.p[3] = {0.07f, 0.05f, 0};
	lm.p[4] = {0.095f, 0.065f, 0};
	// fingers straight up from knuckles 8 cm apart in total (index x=0.03 ... little x=-0.05)
	const float xs[4] = {0.03f, 0.003f, -0.024f, -0.05f};
	for (int f = 0; f < 4; f++) {
		const int mcp = 5 + 4 * f;
		for (int j = 0; j < 4; j++) {
			lm.p[mcp + j] = {xs[f], 0.08f + 0.025f * j, 0};
		}
	}
	return lm;
}

//! The open hand with the index fingertip brought to the thumb tip (a pinch).
Landmarks
pinching_hand()
{
	Landmarks lm = open_hand();
	lm.p[8] = {lm.p[4].x - 0.005f, lm.p[4].y, 0};
	return lm;
}

//! The open hand with fingers 1-4 curled back toward the palm.
Landmarks
fist_hand()
{
	Landmarks lm = open_hand();
	for (int f = 0; f < 4; f++) {
		const int mcp = 5 + 4 * f;
		const Vec3 k = lm.p[mcp];
		lm.p[mcp + 1] = {k.x, k.y + 0.02f, 0.02f};
		lm.p[mcp + 2] = {k.x, k.y, 0.035f};
		lm.p[mcp + 3] = {k.x, k.y - 0.02f, 0.025f};
	}
	return lm;
}

Sample
sample(int64_t ts_ms, float pinch, int category = None, float score = 0.9f, float confidence = 0.95f)
{
	Sample s;
	s.valid = true;
	s.timestamp_ns = ts_ms * 1'000'000LL;
	s.confidence = confidence;
	s.pinch_distance_normalized = pinch;
	s.extended_mask = 0x1f;
	s.category = category;
	s.category_score = score;
	return s;
}

//! Feeds one frame every 16 ms from @p t0_ms; returns the time after the last.
int64_t
feed(Machine &m, int64_t t0_ms, int frames, float pinch, int category = None, int *presses = nullptr,
     int *releases = nullptr)
{
	int64_t t = t0_ms;
	for (int i = 0; i < frames; i++, t += 16) {
		const Events e = m.update(sample(t, pinch, category), t * 1'000'000LL);
		if (presses != nullptr && e.pinch_pressed) {
			(*presses)++;
		}
		if (releases != nullptr && e.pinch_released) {
			(*releases)++;
		}
	}
	return t;
}

} // namespace

TEST_CASE("distance and angle helpers")
{
	CHECK(distance({0, 0, 0}, {3, 4, 0}).value == Approx(5.0f));
	CHECK(distance({0, 0, 0}, {3, 4, 0}).valid);
	CHECK(angleDeg({1, 0, 0}, {0, 0, 0}, {0, 1, 0}).value == Approx(90.0f));
	CHECK(angleDeg({1, 0, 0}, {0, 0, 0}, {-1, 0, 0}).value == Approx(180.0f));
	CHECK(angleDeg({1, 0, 0}, {0, 0, 0}, {1, 0, 0}).value == Approx(0.0f).margin(1e-3));
	// nearly parallel vectors whose cosine rounds past 1: clamped, still finite
	const Value tiny = angleDeg({1, 1e-8f, 0}, {0, 0, 0}, {1, 0, 0});
	CHECK(tiny.valid);
	CHECK(std::isfinite(tiny.value));
}

TEST_CASE("invalid input gives invalid results, never NaN")
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	const float inf = std::numeric_limits<float>::infinity();
	CHECK_FALSE(distance({nan, 0, 0}, {0, 0, 0}).valid);
	CHECK_FALSE(distance({0, 0, 0}, {inf, 0, 0}).valid);
	CHECK_FALSE(angleDeg({0, 0, 0}, {0, 0, 0}, {1, 0, 0}).valid); // zero-length side
	CHECK_FALSE(angleDeg({1, 0, 0}, {nan, 0, 0}, {0, 1, 0}).valid);
	Landmarks missing = open_hand();
	missing.valid = false;
	CHECK_FALSE(normalizedDistance(missing, 4, 8).valid);
	CHECK_FALSE(measure(missing).valid);
	Landmarks bad = open_hand();
	bad.p[8].x = nan;
	CHECK_FALSE(measure(bad).valid);
	CHECK_FALSE(normalizedDistance(open_hand(), 4, 21).valid);
	CHECK_FALSE(normalizedDistance(open_hand(), -1, 8).valid);
}

TEST_CASE("zero palm width is invalid")
{
	Landmarks lm = open_hand();
	lm.p[17] = lm.p[5];
	CHECK_FALSE(palmWidth(lm).value > kEpsilon);
	CHECK_FALSE(normalizedDistance(lm, 4, 8).valid);
	CHECK_FALSE(measure(lm).valid);
}

TEST_CASE("normalized pinch distance and finger extension on synthetic hands")
{
	const Landmarks open = open_hand();
	CHECK(palmWidth(open).value == Approx(0.08f));
	const Value open_pinch = normalizedDistance(open, THUMB_TIP, INDEX_TIP);
	REQUIRE(open_pinch.valid);
	CHECK(open_pinch.value > 0.6f);
	const Value pinch = normalizedDistance(pinching_hand(), THUMB_TIP, INDEX_TIP);
	REQUIRE(pinch.valid);
	CHECK(pinch.value < 0.1f);
	// distances scale with the hand: the normalised value doesn't
	Landmarks big = open;
	for (Vec3 &p : big.p) {
		p = {p.x * 1.3f, p.y * 1.3f, p.z * 1.3f};
	}
	CHECK(normalizedDistance(big, THUMB_TIP, INDEX_TIP).value == Approx(open_pinch.value));

	const HandGeometry go = measure(open);
	REQUIRE(go.valid);
	CHECK((go.extended_mask & 0x1e) == 0x1e); // all four fingers extended
	const HandGeometry gf = measure(fist_hand());
	REQUIRE(gf.valid);
	CHECK((gf.extended_mask & 0x1e) == 0); // none of the four fingers
}

TEST_CASE("pinch hysteresis and confirmation")
{
	Config c;
	c.pinch_on = 0.45f;
	c.pinch_off = 0.60f;
	c.confirm_frames = 2;
	Machine m(c);
	int presses = 0, releases = 0;
	int64_t t = feed(m, 1000, 5, 1.0f, None, &presses, &releases);
	CHECK_FALSE(m.state().pinch);
	// one frame below pinch_on is not enough
	t = feed(m, t, 1, 0.3f, None, &presses, &releases);
	CHECK_FALSE(m.state().pinch);
	t = feed(m, t, 1, 1.0f, None, &presses, &releases);
	// two in a row are
	t = feed(m, t, 2, 0.3f, None, &presses, &releases);
	CHECK(m.state().pinch);
	CHECK(presses == 1);
	// between the thresholds: stays on
	t = feed(m, t, 10, 0.52f, None, &presses, &releases);
	CHECK(m.state().pinch);
	// held: no further presses
	t = feed(m, t, 20, 0.3f, None, &presses, &releases);
	CHECK(presses == 1);
	// above pinch_off for two frames: off
	t = feed(m, t, 2, 0.7f, None, &presses, &releases);
	CHECK_FALSE(m.state().pinch);
	CHECK(releases == 1);
	// between the thresholds again: stays off
	feed(m, t, 10, 0.52f, None, &presses, &releases);
	CHECK_FALSE(m.state().pinch);
	CHECK(presses == 1);
}

TEST_CASE("the same camera frame again doesn't advance the confirmation")
{
	Machine m;
	const Sample s = sample(1000, 0.1f);
	for (int i = 0; i < 20; i++) {
		m.update(s, s.timestamp_ns);
	}
	CHECK_FALSE(m.state().pinch); // one frame, polled 20 times
	m.update(sample(1016, 0.1f), 1016'000'000LL);
	CHECK(m.state().pinch);
}

TEST_CASE("low confidence, invalid and old measurements let go at once")
{
	Machine m;
	int64_t t = feed(m, 1000, 3, 0.1f);
	REQUIRE(m.state().pinch);

	SECTION("low confidence")
	{
		const Events e = m.update(sample(t, 0.1f, None, 0.9f, 0.2f), t * 1'000'000LL);
		CHECK(e.pinch_released);
		CHECK_FALSE(m.state().pinch);
		CHECK_FALSE(m.state().valid);
		CHECK(m.takeStats().rejected_low_confidence == 1);
	}
	SECTION("invalid (tracking lost)")
	{
		Sample s = sample(t, 0.1f);
		s.valid = false;
		const Events e = m.update(s, t * 1'000'000LL);
		CHECK(e.pinch_released);
		CHECK_FALSE(m.state().pinch);
	}
	SECTION("NaN in the measurement")
	{
		Sample s = sample(t, std::numeric_limits<float>::quiet_NaN());
		CHECK(m.update(s, t * 1'000'000LL).pinch_released);
		CHECK_FALSE(m.state().valid);
	}
	SECTION("too old")
	{
		const Events e = m.update(sample(t - 16, 0.1f), (t + 500) * 1'000'000LL);
		CHECK(e.pinch_released);
		CHECK_FALSE(m.state().pinch);
		CHECK(m.takeStats().rejected_stale == 1);
	}
	SECTION("reset (hand not tracked, shutdown)")
	{
		CHECK(m.reset("hand not tracked").pinch_released);
		CHECK_FALSE(m.state().pinch);
		CHECK(m.pinchValue() == 0.0f);
	}
}

TEST_CASE("a pinch needs confirming again after a loss")
{
	Machine m;
	int64_t t = feed(m, 1000, 3, 0.1f);
	REQUIRE(m.state().pinch);
	Sample lost = sample(t, 0.1f);
	lost.valid = false;
	m.update(lost, t * 1'000'000LL);
	t += 16;
	m.update(sample(t, 0.1f), t * 1'000'000LL);
	CHECK_FALSE(m.state().pinch); // one frame after coming back
	t += 16;
	m.update(sample(t, 0.1f), t * 1'000'000LL);
	CHECK(m.state().pinch);
}

TEST_CASE("re-association: time going backwards or a gap starts fresh")
{
	Machine m;
	int64_t t = feed(m, 1000, 3, 0.1f);
	REQUIRE(m.state().pinch);
	SECTION("backwards")
	{
		const Events e = m.update(sample(t - 100, 0.1f), (t - 100) * 1'000'000LL);
		CHECK(e.pinch_released);
		CHECK_FALSE(m.state().pinch);
		CHECK(m.takeStats().reassociated == 1);
	}
	SECTION("a gap longer than max_age")
	{
		const int64_t later = t + 400;
		const Events e = m.update(sample(later, 0.1f), later * 1'000'000LL);
		CHECK(e.pinch_released);
		CHECK_FALSE(m.state().pinch); // this frame counts as the first after the gap
		CHECK(m.takeStats().reassociated == 1);
	}
}

TEST_CASE("left and right hands are independent")
{
	// Mercury keeps each hand in its own slot (0 left, 1 right) and measures each hand on its own crop, so the
	// plugin keeps one machine per hand: one hand's loss or pinch doesn't touch the other.
	Machine left, right;
	feed(left, 1000, 3, 0.1f);
	feed(right, 1000, 3, 1.0f);
	CHECK(left.state().pinch);
	CHECK_FALSE(right.state().pinch);
	left.reset("left hand lost");
	CHECK_FALSE(left.state().pinch);
	CHECK(right.state().valid);
}

TEST_CASE("fist: confirmed from the category, blocks the pinch, one press per hold")
{
	Config c;
	c.confirm_frames = 2;
	c.min_category_score = 0.6f;
	Machine m(c);
	int64_t t = 1000;
	int fist_presses = 0;
	for (int i = 0; i < 2; i++, t += 16) {
		// a fist brings the thumb to the index: pinch distance small
		fist_presses += m.update(sample(t, 0.1f, ClosedFist), t * 1'000'000LL).fist_pressed ? 1 : 0;
	}
	CHECK(m.state().fist);
	CHECK_FALSE(m.state().pinch);
	CHECK(m.pinchValue() == 0.0f);
	for (int i = 0; i < 30; i++, t += 16) {
		fist_presses += m.update(sample(t, 0.1f, ClosedFist), t * 1'000'000LL).fist_pressed ? 1 : 0;
	}
	CHECK(fist_presses == 1);
	CHECK_FALSE(m.state().pinch);
	// a low score doesn't count as a fist
	Machine weak(c);
	feed(weak, 1000, 5, 1.0f, ClosedFist);
	CHECK(weak.state().fist);
	Machine low(c);
	int64_t t2 = 1000;
	for (int i = 0; i < 5; i++, t2 += 16) {
		low.update(sample(t2, 1.0f, ClosedFist, 0.3f), t2 * 1'000'000LL);
	}
	CHECK_FALSE(low.state().fist);
}

TEST_CASE("deliberate pinch: the other fingers open; the trigger pinch doesn't need them by default")
{
	Machine d;
	int64_t t0 = 1000;
	for (int i = 0; i < 3; i++, t0 += 16) {
		Sample s = sample(t0, 0.1f);
		s.extended_mask = 0x02;
		d.update(s, t0 * 1'000'000LL);
	}
	CHECK(d.state().pinch);
	CHECK_FALSE(d.state().deliberatePinch);
	for (int i = 0; i < 2; i++, t0 += 16) {
		Sample s = sample(t0, 0.1f);
		s.extended_mask = 0x1e;
		d.update(s, t0 * 1'000'000LL);
	}
	CHECK(d.state().deliberatePinch);
	CHECK(d.reset("lost").pinch_released);
	CHECK_FALSE(d.state().deliberatePinch);
}

TEST_CASE("a pinch needs the other fingers open when configured")
{
	Config c;
	c.pinch_min_other_fingers = 2;
	Machine m(c);
	int64_t t = 1000;
	// thumb and index close, middle/ring/little curled (a pointing hand or a loose fist): no pinch
	for (int i = 0; i < 5; i++, t += 16) {
		Sample s = sample(t, 0.1f);
		s.extended_mask = 0x02; // only the index
		m.update(s, t * 1'000'000LL);
	}
	CHECK_FALSE(m.state().pinch);
	// the same with middle and ring straight: a pinch
	for (int i = 0; i < 2; i++, t += 16) {
		Sample s = sample(t, 0.1f);
		s.extended_mask = 0x0c;
		m.update(s, t * 1'000'000LL);
	}
	CHECK(m.state().pinch);
	// one straight finger keeps it (hysteresis), none lets go
	for (int i = 0; i < 4; i++, t += 16) {
		Sample s = sample(t, 0.1f);
		s.extended_mask = 0x04;
		m.update(s, t * 1'000'000LL);
	}
	CHECK(m.state().pinch);
	Events e;
	for (int i = 0; i < 2; i++, t += 16) {
		Sample s = sample(t, 0.1f);
		s.extended_mask = 0x00;
		e = m.update(s, t * 1'000'000LL);
	}
	CHECK_FALSE(m.state().pinch);
	CHECK(e.pinch_released);
	// 0: the finger rule off
	Config off;
	off.pinch_min_other_fingers = 0;
	Machine any(off);
	int64_t t2 = 1000;
	for (int i = 0; i < 2; i++, t2 += 16) {
		Sample s = sample(t2, 0.1f);
		s.extended_mask = 0x00;
		any.update(s, t2 * 1'000'000LL);
	}
	CHECK(any.state().pinch);
}

TEST_CASE("categories and point")
{
	Machine m;
	feed(m, 1000, 3, 1.0f, OpenPalm);
	CHECK(m.state().openPalm);
	CHECK_FALSE(m.state().victory);
	Machine v;
	feed(v, 1000, 3, 1.0f, Victory);
	CHECK(v.state().victory);
	Machine ily;
	feed(ily, 1000, 3, 1.0f, ILoveYou);
	CHECK(ily.state().iLoveYou);
	// point: index extended (bit 1), middle/ring/little not (bits 2-4); the thumb doesn't matter
	Machine p;
	int64_t t = 1000;
	for (int i = 0; i < 3; i++, t += 16) {
		Sample s = sample(t, 1.0f);
		s.extended_mask = 0x03;
		p.update(s, t * 1'000'000LL);
	}
	CHECK(p.state().point);
}

TEST_CASE("pinch value")
{
	Config c;
	c.pinch_on = 0.45f;
	c.pinch_off = 0.60f;
	Machine m(c);
	CHECK(m.pinchValue() == 0.0f); // nothing yet
	feed(m, 1000, 1, 0.3f);
	CHECK(m.pinchValue() == Approx(1.0f));
	feed(m, 2000, 1, 1.5f);
	CHECK(m.pinchValue() == Approx(0.0f));
}
