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
#include "ovrn_gesture_rules.hpp"
#include "ovrn_gesture_rules_default.hpp"

#include "catch_amalgamated.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

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

TEST_CASE("pinch hysteresis and confirmation (each change held 60 ms)")
{
	Config c;
	c.pinch_on = 0.45f;
	c.pinch_off = 0.60f;
	c.confirm_frames = 2;
	Machine m(c);
	int presses = 0, releases = 0;
	int64_t t = feed(m, 1000, 5, 1.0f, None, &presses, &releases);
	CHECK_FALSE(m.state().pinch);
	// a brief dip below pinch_on (two frames, 16 ms apart) is not enough
	t = feed(m, t, 2, 0.3f, None, &presses, &releases);
	CHECK_FALSE(m.state().pinch);
	t = feed(m, t, 1, 1.0f, None, &presses, &releases);
	// below for 60 ms is (frames at 0, 16 ... 64 ms)
	t = feed(m, t, 5, 0.3f, None, &presses, &releases);
	CHECK(m.state().pinch);
	CHECK(presses == 1);
	// between the thresholds: stays on
	t = feed(m, t, 10, 0.52f, None, &presses, &releases);
	CHECK(m.state().pinch);
	// held: no further presses
	t = feed(m, t, 20, 0.3f, None, &presses, &releases);
	CHECK(presses == 1);
	// above pinch_off for two frames: still on (a jump in the measurement)
	t = feed(m, t, 2, 0.7f, None, &presses, &releases);
	CHECK(m.state().pinch);
	t = feed(m, t, 1, 0.3f, None, &presses, &releases);
	// above pinch_off for 60 ms: off
	t = feed(m, t, 5, 0.7f, None, &presses, &releases);
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
	for (int64_t t = 1016; t <= 1064; t += 16) {
		m.update(sample(t, 0.1f), t * 1'000'000LL);
	}
	CHECK(m.state().pinch);
}

TEST_CASE("low confidence, invalid and old measurements let go at once")
{
	Machine m;
	int64_t t = feed(m, 1000, 6, 0.1f);
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
	int64_t t = feed(m, 1000, 6, 0.1f);
	REQUIRE(m.state().pinch);
	Sample lost = sample(t, 0.1f);
	lost.valid = false;
	m.update(lost, t * 1'000'000LL);
	t += 16;
	m.update(sample(t, 0.1f), t * 1'000'000LL);
	CHECK_FALSE(m.state().pinch); // one frame after coming back
	for (int i = 0; i < 4; i++) {
		t += 16;
		m.update(sample(t, 0.1f), t * 1'000'000LL);
	}
	CHECK(m.state().pinch); // 64 ms after coming back
}

TEST_CASE("re-association: time going backwards or a gap starts fresh")
{
	Machine m;
	int64_t t = feed(m, 1000, 6, 0.1f);
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
	feed(left, 1000, 6, 0.1f);
	feed(right, 1000, 6, 1.0f);
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

TEST_CASE("a pinch doesn't depend on the other fingers")
{
	Machine m;
	int64_t t = 1000;
	for (int i = 0; i < 6; i++, t += 16) {
		Sample s = sample(t, 0.1f);
		s.extended_mask = 0x00; // all curled but thumb and index close
		m.update(s, t * 1'000'000LL);
	}
	CHECK(m.state().pinch);
}

TEST_CASE("open palm")
{
	Machine m;
	feed(m, 1000, 3, 1.0f, OpenPalm);
	CHECK(m.state().openPalm);
	CHECK_FALSE(m.state().fist);
	Machine v;
	feed(v, 1000, 3, 1.0f, Victory);
	CHECK_FALSE(v.state().openPalm);
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

TEST_CASE("dashboard gesture: open palm toward the face, raised, held about 1 s (judged over a window)")
{
	constexpr int64_t ms = 1'000'000;
	// frames every 11 ms; open_every: Open_Palm on 1 of that many readings (MediaPipe's flicker), the rest "none"
	const auto run = [](SystemGesture &g, int64_t from, int64_t to, bool pose, int open_every, bool bad = false) {
		bool down = false;
		int k = 0;
		for (int64_t t = from; t < to; t += 11, k++) {
			down = g.update(t * ms, pose, true, open_every > 0 && k % open_every == 0, bad);
		}
		return down;
	};
	SECTION("a steady open palm held 1 s presses; down while held, up when the pose ends")
	{
		SystemGesture g;
		CHECK_FALSE(run(g, 0, 900, true, 1));
		CHECK(run(g, 900, 1100, true, 1));
		CHECK(run(g, 1100, 2000, true, 1));
		CHECK_FALSE(run(g, 2000, 2700, false, 0));
	}
	SECTION("MediaPipe's flicker (open on half the readings) still presses")
	{
		SystemGesture g;
		CHECK(run(g, 0, 1200, true, 2));
	}
	SECTION("too little open palm (1 in 4 readings) doesn't press")
	{
		SystemGesture g;
		CHECK_FALSE(run(g, 0, 2000, true, 4));
	}
	SECTION("a fist or pinch in the window doesn't press")
	{
		SystemGesture g;
		CHECK_FALSE(run(g, 0, 2000, true, 1, true));
	}
	SECTION("the pose must hold: open palm not toward the face (or too low) doesn't press")
	{
		SystemGesture g;
		CHECK_FALSE(run(g, 0, 2000, false, 1));
	}
	SECTION("one hold is one press; a second hold after letting go is another")
	{
		SystemGesture g;
		CHECK(run(g, 0, 1200, true, 1));
		CHECK(run(g, 1200, 3000, true, 1)); // still the same press
		CHECK_FALSE(run(g, 3000, 3700, false, 0));
		CHECK_FALSE(run(g, 3700, 4500, true, 1)); // the window still holds the let-go frames
		CHECK(run(g, 4500, 5000, true, 1));
	}
	SECTION("reset (hand lost) lets go and forgets the window")
	{
		SystemGesture g;
		CHECK(run(g, 0, 1200, true, 1));
		g.reset();
		CHECK_FALSE(g.down());
		CHECK_FALSE(run(g, 1200, 1700, true, 1));
	}
}

TEST_CASE("custom mapping: decides pinch and fist when valid, held 60 ms; point and claw do nothing")
{
	Config c;
	c.confirm_frames = 2;
	const auto custom = [](int64_t ts_ms, int g, float pinch_distance = 1.2f) {
		Sample s = sample(ts_ms, pinch_distance);
		s.custom_valid = true;
		s.custom = g;
		s.custom_scores[g] = 0.9f;
		return s;
	};
	// pinch: the mapping's pinch, not the distance (an open-looking distance), on after 60 ms, off after 60 ms
	Machine m(c);
	int64_t t = 1000;
	int presses = 0, releases = 0;
	for (int i = 0; i < 3; i++, t += 16) {
		presses += m.update(custom(t, CustomPinch), t * 1'000'000LL).pinch_pressed ? 1 : 0;
	}
	CHECK(presses == 0); // 32 ms asked for
	for (int i = 0; i < 10; i++, t += 16) {
		presses += m.update(custom(t, CustomPinch), t * 1'000'000LL).pinch_pressed ? 1 : 0;
	}
	CHECK(presses == 1);
	CHECK(m.state().pinch);
	CHECK(m.pinchValue() == 1.0f);
	for (int i = 0; i < 10; i++, t += 16) {
		releases += m.update(custom(t, CustomOpen, 0.1f), t * 1'000'000LL).pinch_released ? 1 : 0;
	}
	CHECK(releases == 1); // a small distance doesn't hold it on: the mapping said open
	CHECK_FALSE(m.state().pinch);

	// point and claw: neither pinch nor fist, whatever the distance
	for (int g : {CustomPoint, CustomClaw}) {
		Machine n(c);
		int64_t u = 1000;
		for (int i = 0; i < 20; i++, u += 16) {
			n.update(custom(u, g, 0.1f), u * 1'000'000LL);
		}
		CHECK_FALSE(n.state().pinch);
		CHECK_FALSE(n.state().fist);
	}

	// fist: from the mapping (not the category), blocks the pinch, value 0
	Machine f(c);
	int64_t v = 1000;
	int fist_presses = 0;
	for (int i = 0; i < 20; i++, v += 16) {
		fist_presses += f.update(custom(v, CustomFist, 0.1f), v * 1'000'000LL).fist_pressed ? 1 : 0;
	}
	CHECK(fist_presses == 1);
	CHECK(f.state().fist);
	CHECK_FALSE(f.state().pinch);
	CHECK(f.pinchValue() == 0.0f);

	// without the mapping the old rules still apply (category fist)
	Machine old(c);
	feed(old, 1000, 5, 1.0f, ClosedFist);
	CHECK(old.state().fist);
}

// ------------------------------------------------------------------------------------------- joint features and rules

namespace {

namespace rl = ovrn::gest::rules;

//! The same features psvr2-test hands/gesture_fit.py computes for these synthetic hands (copied from its output).
const float kOpenFeatures[kFeatureCount] = {180.0f, 180.0f, 180.0f, 180.0f, 180.0f, 180.0f, 180.0f, 180.0f, 1.8478f,
                                            1.9365f, 1.8779f, 1.7264f, 1.3877f, 1.6088f, 1.8650f, 2.1333f, 180.0f,
                                            0.8339f};
const float kFistFeatures[kFeatureCount] = {81.8699f, 116.5651f, 81.8699f, 116.5651f, 81.8699f, 116.5651f, 81.8699f,
                                            116.5651f, 0.8379f,  0.8128f,   0.8296f,  0.8693f,   0.8728f,  1.1933f,
                                            1.5213f,  1.8403f,   180.0f,   0.8339f};

const char *const kTestConfig = R"json({
 "smooth_frames": 1, "grace_ms": 100,
 "hands": {
  "left": {
   "fist": {"enabled": true, "output": "grip", "priority": 3, "min_confidence": 0.5, "activate_ms": 50,
            "release_ms": 50, "terms": [{"feature": "index_reach", "op": "<=", "on": 1.0, "off": 1.2}]},
   "point": {"enabled": true, "output": "block_pinch", "priority": 2, "activate_ms": 30, "release_ms": 30,
             "terms": [{"feature": "middle_reach", "op": "<=", "on": 1.0, "off": 1.1},
                       {"feature": "index_reach", "op": ">=", "on": 1.5, "off": 1.4}]},
   "thumbs_up": {"enabled": false, "terms": [], "status": "no recorded examples"}
  },
  "right": {}
 }
})json";

//! Feeds @p f every 16 ms for @p frames frames from @p t_ms; returns the time after.
int64_t
feedRules(rl::Engine &e, const float *f, int64_t t_ms, int frames, float conf = 0.9f, bool valid = true)
{
	rl::Change ch[8];
	for (int i = 0; i < frames; i++, t_ms += 16) {
		e.update(f, valid, conf, t_ms * 1'000'000LL, ch, 8);
	}
	return t_ms;
}

} // namespace

TEST_CASE("joint features match the fit tool's on synthetic hands; names round-trip; bad input is invalid")
{
	float f[kFeatureCount];
	REQUIRE(jointFeatures(open_hand(), f));
	for (int i = 0; i < kFeatureCount; i++) {
		CHECK(f[i] == Approx(kOpenFeatures[i]).margin(1e-3));
	}
	REQUIRE(jointFeatures(fist_hand(), f));
	for (int i = 0; i < kFeatureCount; i++) {
		CHECK(f[i] == Approx(kFistFeatures[i]).margin(1e-3));
	}
	for (int i = 0; i < kFeatureCount; i++) {
		CHECK(featureIndex(featureName(i)) == i);
	}
	CHECK(featureIndex("nope") == -1);
	Landmarks bad = open_hand();
	bad.p[8].x = std::numeric_limits<float>::quiet_NaN();
	CHECK_FALSE(jointFeatures(bad, f));
	Landmarks zero = open_hand();
	zero.p[17] = zero.p[5]; // palm width 0
	CHECK_FALSE(jointFeatures(zero, f));
}

TEST_CASE("gesture rules config: parse, defaults, errors; the built-in config parses")
{
	std::string err;
	const rl::Config c = rl::parse(kTestConfig, &err);
	REQUIRE(c.valid);
	REQUIRE(c.hands[0].gestures.size() == 3);
	CHECK(c.hands[1].gestures.empty());
	const rl::Gesture &fist = c.hands[0].gestures[0];
	CHECK(fist.id == "fist");
	CHECK(fist.output == rl::Output::Grip);
	CHECK(fist.activate_ns == 50'000'000);
	REQUIRE(fist.terms.size() == 1);
	CHECK(fist.terms[0].feature == F_INDEX_REACH);
	CHECK(fist.terms[0].below);
	CHECK(fist.terms[0].off == Approx(1.2f));
	CHECK_FALSE(rl::parse("{", &err).valid);
	CHECK_FALSE(rl::parse(R"({"hands": {"left": {"x": {"terms": [{"feature": "nope", "op": "<=", "on": 1}]}}}})", &err).valid);
	CHECK_FALSE(rl::parse(R"({"hands": {"left": {"x": {"terms": [{"feature": "index_pip", "op": "<="}]}}}})", &err).valid);
	const rl::Config builtin = rl::parse(rl::kDefaultConfig, &err);
	CHECK(builtin.valid);
}

TEST_CASE("gesture rules: activation after its hold time, hysteresis, release, one press")
{
	std::string err;
	const rl::Config c = rl::parse(kTestConfig, &err);
	rl::Engine e(c.hands[0], 1, 100'000'000);
	int64_t t = feedRules(e, kOpenFeatures, 1000, 5);
	CHECK(e.activeIndex() == -1);
	t = feedRules(e, kFistFeatures, t, 3); // 32 ms of fist: still a candidate
	CHECK(e.activeIndex() == -1);
	CHECK(e.phase(0) == rl::Phase::Candidate);
	t = feedRules(e, kFistFeatures, t, 3);
	CHECK(e.activeIndex() == 0);
	CHECK(e.activeOutput(rl::Output::Grip));
	CHECK(e.decides(rl::Output::Grip));
	// hysteresis: index reach 1.1 is above "on" (1.0) but inside "off" (1.2): stays active
	float f[kFeatureCount];
	std::copy(kFistFeatures, kFistFeatures + kFeatureCount, f);
	f[F_INDEX_REACH] = 1.1f;
	t = feedRules(e, f, t, 20);
	CHECK(e.activeIndex() == 0);
	// a one-frame dip past "off" doesn't release (release_ms 50)
	t = feedRules(e, kOpenFeatures, t, 1);
	CHECK(e.phase(0) == rl::Phase::Releasing);
	t = feedRules(e, kFistFeatures, t, 1);
	CHECK(e.phase(0) == rl::Phase::Active);
	// open for long enough: released
	t = feedRules(e, kOpenFeatures, t, 6);
	CHECK(e.activeIndex() == -1);
	// the same camera frame again doesn't advance anything
	rl::Change ch[8];
	CHECK(e.update(kFistFeatures, true, 0.9f, (t - 16) * 1'000'000LL, ch, 8) == 0);
}

TEST_CASE("gesture rules: tracking loss holds for the grace time, then lets go; found again starts fresh")
{
	std::string err;
	const rl::Config c = rl::parse(kTestConfig, &err);
	rl::Engine e(c.hands[0], 1, 100'000'000);
	int64_t t = feedRules(e, kFistFeatures, 1000, 10);
	REQUIRE(e.activeIndex() == 0);
	t = feedRules(e, kFistFeatures, t, 4, 0.9f, false); // 64 ms without a measurement: held
	CHECK(e.activeIndex() == 0);
	t = feedRules(e, kFistFeatures, t, 5, 0.9f, false); // past 100 ms: let go
	CHECK(e.activeIndex() == -1);
	// low confidence counts as lost too
	t = feedRules(e, kFistFeatures, t, 10);
	REQUIRE(e.activeIndex() == 0);
	t = feedRules(e, kFistFeatures, t, 10, 0.2f);
	CHECK(e.activeIndex() == -1);
	// found again after a gap: from idle, the hold time again
	t = feedRules(e, kFistFeatures, t + 500, 1);
	CHECK(e.activeIndex() == -1);
	t = feedRules(e, kFistFeatures, t, 5);
	CHECK(e.activeIndex() == 0);
	// reset lets go at once
	rl::Change ch[8];
	CHECK(e.reset(t * 1'000'000LL, "test", ch, 8) == 1);
	CHECK(e.activeIndex() == -1);
}

TEST_CASE("gesture rules: one gesture at a time, higher priority takes over; disabled and empty rules never fire")
{
	std::string err;
	const rl::Config c = rl::parse(kTestConfig, &err);
	rl::Engine e(c.hands[0], 1, 100'000'000);
	// a pose that is both a point (middle curled, index straight) and, with index curled, a fist: point first
	float point[kFeatureCount];
	std::copy(kFistFeatures, kFistFeatures + kFeatureCount, point);
	point[F_INDEX_REACH] = 1.8f;
	int64_t t = feedRules(e, point, 1000, 5);
	CHECK(e.activeIndex() == 1);
	CHECK(e.activeOutput(rl::Output::BlockPinch));
	// now a fist (priority 3 > 2): takes over after its hold time; never two active
	t = feedRules(e, kFistFeatures, t, 6);
	CHECK(e.activeIndex() == 0);
	CHECK(e.phase(1) == rl::Phase::Idle);
	// thumbs_up is disabled with no terms: never leaves idle
	CHECK(e.phase(2) == rl::Phase::Idle);
	// no gestures at all (the right hand here): nothing, no crash
	rl::Engine none(c.hands[1], 3, 100'000'000);
	feedRules(none, kFistFeatures, 1000, 10);
	CHECK(none.activeIndex() == -1);
	CHECK_FALSE(none.decides(rl::Output::Grip));
}

TEST_CASE("gesture rules smoothing: the mean of the last frames")
{
	std::string err;
	const rl::Config c = rl::parse(kTestConfig, &err);
	rl::Engine e(c.hands[0], 3, 100'000'000);
	float a[kFeatureCount], b[kFeatureCount];
	std::copy(kOpenFeatures, kOpenFeatures + kFeatureCount, a);
	std::copy(kOpenFeatures, kOpenFeatures + kFeatureCount, b);
	a[F_INDEX_REACH] = 0.9f;
	b[F_INDEX_REACH] = 1.5f;
	int64_t t = feedRules(e, a, 1000, 1);
	t = feedRules(e, b, t, 1);
	t = feedRules(e, a, t, 1);
	CHECK(e.smoothed()[F_INDEX_REACH] == Approx((0.9f + 1.5f + 0.9f) / 3.0f));
	t = feedRules(e, b, t, 1);
	CHECK(e.smoothed()[F_INDEX_REACH] == Approx((1.5f + 0.9f + 1.5f) / 3.0f));
}

TEST_CASE("the calibrated rules drive the state machine's fist and rule out the pinch")
{
	Config cfg;
	cfg.confirm_frames = 3;
	Machine m(cfg);
	int64_t t = 1000;
	// the rule says fist: on at once (its own hold time already passed), whatever the category / custom mapping says
	Sample s = sample(t, 1.2f);
	s.custom_valid = true;
	s.custom = CustomOpen;
	s.rule_fist_decides = true;
	s.rule_fist = true;
	CHECK(m.update(s, t * 1'000'000LL).fist_pressed);
	CHECK(m.state().fist);
	// the rule says no fist: off even if the custom mapping says fist
	t += 16;
	s = sample(t, 1.2f);
	s.custom_valid = true;
	s.custom = CustomFist;
	s.rule_fist_decides = true;
	s.rule_fist = false;
	CHECK(m.update(s, t * 1'000'000LL).fist_released);
	// an active point rules out the custom mapping's pinch
	Machine p(cfg);
	int64_t u = 1000;
	for (int i = 0; i < 20; i++, u += 16) {
		Sample q = sample(u, 1.2f);
		q.custom_valid = true;
		q.custom = CustomPinch;
		q.custom_scores[CustomPinch] = 0.9f;
		q.rule_block_pinch = true;
		p.update(q, u * 1'000'000LL);
	}
	CHECK_FALSE(p.state().pinch);
}

TEST_CASE("image pinch detector: averaged probability, on 0.6 / off 0.4 (hysteresis), held 30 ms, reset clears it")
{
	Config cfg;
	Machine m(cfg);
	const auto img = [](int64_t ts_ms, float p) {
		Sample s = sample(ts_ms, 1.5f); // an open-looking landmark distance: the image decides
		s.pinch_image_valid = true;
		s.pinch_image = p;
		return s;
	};
	int64_t t = 1000;
	int presses = 0, releases = 0;
	for (int i = 0; i < 10; i++, t += 16) {
		presses += m.update(img(t, 0.2f), t * 1'000'000LL).pinch_pressed ? 1 : 0;
	}
	CHECK(presses == 0);
	// probability 0.9: the 3-frame mean crosses 0.6 on the 2nd frame, then 30 ms held
	for (int i = 0; i < 6; i++, t += 16) {
		presses += m.update(img(t, 0.9f), t * 1'000'000LL).pinch_pressed ? 1 : 0;
	}
	CHECK(presses == 1);
	CHECK(m.state().pinch);
	CHECK(m.pinchValue() == 1.0f);
	// 0.5 is below "on" but above "off": stays on
	for (int i = 0; i < 10; i++, t += 16) {
		releases += m.update(img(t, 0.5f), t * 1'000'000LL).pinch_released ? 1 : 0;
	}
	CHECK(releases == 0);
	// 0.1: released after the hold
	for (int i = 0; i < 8; i++, t += 16) {
		releases += m.update(img(t, 0.1f), t * 1'000'000LL).pinch_released ? 1 : 0;
	}
	CHECK(releases == 1);
	CHECK_FALSE(m.state().pinch);
	// a single high frame doesn't click (the mean of 3 stays under 0.6)
	presses = 0;
	presses += m.update(img(t, 0.95f), t * 1'000'000LL).pinch_pressed ? 1 : 0;
	t += 16;
	for (int i = 0; i < 6; i++, t += 16) {
		presses += m.update(img(t, 0.1f), t * 1'000'000LL).pinch_pressed ? 1 : 0;
	}
	CHECK(presses == 0);
	// a fist still wins over the image pinch
	Machine f(cfg);
	int64_t u = 1000;
	for (int i = 0; i < 10; i++, u += 16) {
		Sample s = img(u, 0.95f);
		s.rule_fist_decides = true;
		s.rule_fist = true;
		f.update(s, u * 1'000'000LL);
	}
	CHECK(f.state().fist);
	CHECK_FALSE(f.state().pinch);
	// reset clears the history: the next high frame needs the mean again
	m.reset("test");
	t += 16;
	CHECK_FALSE(m.update(img(t, 0.9f), t * 1'000'000LL).pinch_pressed);
}

TEST_CASE("formula pinch rule decides the pinch when it can measure; the gesture mapping covers the rest")
{
	Config cfg;
	// the built-in config has the formula pinch on for both hands
	std::string err;
	const rl::Config builtin = rl::parse(rl::kDefaultConfig, &err);
	REQUIRE(builtin.valid);
	for (int h = 0; h < 2; h++) {
		rl::Engine e(builtin.hands[h], builtin.smooth_frames, builtin.grace_ns);
		CHECK(e.decides(rl::Output::Pinch));
		CHECK(e.decides(rl::Output::Grip));
	}
	// the rule decides: its pinch, not the mapping's
	Machine m(cfg);
	int64_t t = 1000;
	for (int i = 0; i < 5; i++, t += 16) {
		Sample s = sample(t, 1.2f);
		s.custom_valid = true;
		s.custom = CustomOpen; // the mapping says no pinch
		s.rule_pinch_decides = true;
		s.rule_pinch = true;
		m.update(s, t * 1'000'000LL);
	}
	CHECK(m.state().pinch);
	// no measurement: the mapping's pinch covers the frames
	Machine f(cfg);
	int64_t u = 1000;
	for (int i = 0; i < 10; i++, u += 16) {
		Sample s = sample(u, 1.2f);
		s.custom_valid = true;
		s.custom = CustomPinch;
		s.custom_scores[CustomPinch] = 0.9f;
		s.rule_pinch_decides = false;
		f.update(s, u * 1'000'000LL);
	}
	CHECK(f.state().pinch);
}

TEST_CASE("Hand Landmarker's own gestures: confirmed from its categories, low scores ignored, reset releases them")
{
	Config c;
	c.confirm_frames = 2;
	c.min_category_score = 0.6f;
	for (int cat : {PointingUp, ThumbUp, ThumbDown, Victory, ILoveYou, OpenPalm}) {
		Machine m(c);
		int64_t t = 1000;
		m.update(sample(t, 1.2f, cat, 0.9f), t * 1'000'000LL);
		CHECK_FALSE(m.state().category[cat]); // one frame: not yet confirmed
		t += 16;
		m.update(sample(t, 1.2f, cat, 0.9f), t * 1'000'000LL);
		CHECK(m.state().category[cat]);
		for (int other : {PointingUp, ThumbUp, ThumbDown, Victory, ILoveYou, OpenPalm}) {
			if (other != cat) {
				CHECK_FALSE(m.state().category[other]);
			}
		}
		m.reset("test");
		CHECK_FALSE(m.state().category[cat]);
		// a low score never counts
		Machine low(c);
		int64_t u = 1000;
		for (int i = 0; i < 5; i++, u += 16) {
			low.update(sample(u, 1.2f, cat, 0.4f), u * 1'000'000LL);
		}
		CHECK_FALSE(low.state().category[cat]);
	}
}
