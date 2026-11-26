// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Calibrated gesture rules on MediaPipe's joint features (Mercury's gesture::jointFeatures), per hand, from a
 *         JSON config fitted on labelled recordings (psvr2-test hands/gesture_fit.py), with a state machine.
 *
 * A gesture is a set of terms, each one feature above or below a threshold; it may switch on when every term holds at
 * its "on" threshold and stays on while every term holds at its looser "off" threshold (hysteresis). States: idle ->
 * candidate (the on condition holds) -> active after activate_ms -> releasing when the off condition fails -> idle
 * after release_ms (back to active if it holds again). Gestures are mutually exclusive: one active at a time; a
 * candidate of higher priority takes over when its activate time is up. Measurements missing or below the gesture's
 * minimum hand confidence: everything is kept for grace_ms (a dropped frame doesn't release a held gesture), then
 * released; after such a loss the hand starts from idle with fresh smoothing (re-found). The features are averaged
 * over the last smooth_frames measurements.
 *
 * Config ($XDG_CONFIG_HOME/monado/psvr2-gestures.json, else the built-in one, ovrn_gesture_rules_default.hpp; same
 * format): {"smooth_frames": N, "grace_ms": ms, "hands": {"left": {"<id>": {"enabled": bool, "output": "grip" |
 * "block_pinch" | "none", "priority": n, "min_confidence": c, "activate_ms": ms, "release_ms": ms, "status": "...",
 * "terms": [{"feature": "<name>", "op": "<=" | ">=", "on": x, "off": y}]}}, "right": {...}}}. A gesture with no terms
 * never activates (definitions waiting for recordings).
 *
 * @ingroup st_ovrn
 */

#pragma once

#include "hg_gesture_geometry.hpp"
#include "util/u_json.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace ovrn::gest::rules {

namespace geo = xrt::tracking::hand::mercury::gesture;
constexpr int kFeatures = geo::kFeatureCount;

//! What an active gesture does to the hand's inputs.
enum class Output : int
{
	None = 0,       //!< nothing (recognised only)
	Grip = 1,       //!< the grip (grab)
	BlockPinch = 2, //!< no pinch while it's active (a pose that brings thumb and index close without pinching)
	Pinch = 3,      //!< the pinch (trigger) itself
};

struct Term
{
	int feature = -1;
	bool below = true; //!< the gesture has values at or below the thresholds ("<="), else at or above (">=")
	float on = 0.0f;   //!< to switch on
	float off = 0.0f;  //!< to stay on (looser than on)
};

struct Gesture
{
	std::string id;
	bool enabled = false;
	Output output = Output::None;
	int priority = 0;
	float min_confidence = 0.5f;
	int64_t activate_ns = 50'000'000;
	int64_t release_ns = 50'000'000;
	std::string status; //!< how it was calibrated, or why it's off
	std::vector<Term> terms;
};

struct HandConfig
{
	std::vector<Gesture> gestures;
};

struct Config
{
	int smooth_frames = 1;
	int64_t grace_ns = 100'000'000;
	HandConfig hands[2]; //!< 0 left, 1 right
	std::string source;  //!< where it came from (file path, or "built in")
	bool valid = false;
};

//! Parses @p text (the config format above); @p err gets why on failure.
inline Config
parse(const char *text, std::string *err)
{
	Config c;
	cJSON *root = cJSON_Parse(text != nullptr ? text : "");
	if (root == nullptr) {
		*err = "not JSON";
		return c;
	}
	const auto num = [](const cJSON *o, const char *k, double d) {
		const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, k);
		return cJSON_IsNumber(v) ? v->valuedouble : d;
	};
	c.smooth_frames = (int)num(root, "smooth_frames", 1);
	c.smooth_frames = c.smooth_frames < 1 ? 1 : (c.smooth_frames > 30 ? 30 : c.smooth_frames);
	c.grace_ns = (int64_t)(num(root, "grace_ms", 100) * 1e6);
	const cJSON *hands = cJSON_GetObjectItemCaseSensitive(root, "hands");
	static const char *const names[2] = {"left", "right"};
	bool ok = true;
	for (int h = 0; h < 2 && ok; h++) {
		const cJSON *hand = cJSON_GetObjectItemCaseSensitive(hands, names[h]);
		const cJSON *g = nullptr;
		cJSON_ArrayForEach(g, hand)
		{
			if (!cJSON_IsObject(g) || g->string == nullptr) {
				continue;
			}
			Gesture d;
			d.id = g->string;
			d.enabled = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(g, "enabled"));
			const cJSON *out = cJSON_GetObjectItemCaseSensitive(g, "output");
			const std::string o = cJSON_IsString(out) ? out->valuestring : "none";
			d.output = o == "grip" ? Output::Grip
			           : o == "block_pinch" ? Output::BlockPinch
			           : o == "pinch" ? Output::Pinch
			                         : Output::None;
			d.priority = (int)num(g, "priority", 0);
			d.min_confidence = (float)num(g, "min_confidence", 0.5);
			d.activate_ns = (int64_t)(num(g, "activate_ms", 50) * 1e6);
			d.release_ns = (int64_t)(num(g, "release_ms", 50) * 1e6);
			const cJSON *st = cJSON_GetObjectItemCaseSensitive(g, "status");
			d.status = cJSON_IsString(st) ? st->valuestring : "";
			const cJSON *t = nullptr;
			cJSON_ArrayForEach(t, cJSON_GetObjectItemCaseSensitive(g, "terms"))
			{
				Term term;
				const cJSON *f = cJSON_GetObjectItemCaseSensitive(t, "feature");
				const cJSON *op = cJSON_GetObjectItemCaseSensitive(t, "op");
				term.feature = cJSON_IsString(f) ? geo::featureIndex(f->valuestring) : -1;
				if (term.feature < 0 || !cJSON_IsString(op)) {
					*err = "gesture " + d.id + ": unknown feature or op";
					ok = false;
					break;
				}
				term.below = std::string(op->valuestring) == "<=";
				term.on = (float)num(t, "on", NAN);
				term.off = (float)num(t, "off", term.on);
				if (!std::isfinite(term.on) || !std::isfinite(term.off)) {
					*err = "gesture " + d.id + ": threshold missing";
					ok = false;
					break;
				}
				d.terms.push_back(term);
			}
			if (!ok) {
				break;
			}
			c.hands[h].gestures.push_back(d);
		}
	}
	cJSON_Delete(root);
	c.valid = ok;
	return c;
}

enum class Phase : int
{
	Idle = 0,
	Candidate = 1,
	Active = 2,
	Releasing = 3,
};

inline const char *
phaseName(Phase p)
{
	switch (p) {
	case Phase::Idle: return "idle";
	case Phase::Candidate: return "candidate";
	case Phase::Active: return "active";
	case Phase::Releasing: return "releasing";
	}
	return "?";
}

//! What changed in one update.
struct Change
{
	int gesture = -1; //!< index into the hand's gestures (-1: none)
	Phase from = Phase::Idle, to = Phase::Idle;
	const char *reason = nullptr;
};

//! One hand's rules and states.
class Engine
{
public:
	Engine() = default;
	Engine(const HandConfig &hand, int smooth_frames, int64_t grace_ns)
	    : m_hand(hand), m_smooth(smooth_frames < 1 ? 1 : (smooth_frames > kMaxSmooth ? kMaxSmooth : smooth_frames)),
	      m_grace(grace_ns),
	      m_phase(hand.gestures.size(), Phase::Idle), m_since(hand.gestures.size(), 0)
	{}

	/*!
	 * One camera frame's measurement (@p features valid or not, the hand presence @p confidence) at @p t_ns. Fills
	 * @p changes (up to its capacity, returns how many happened).
	 */
	int
	update(const float *features, bool valid, float confidence, int64_t t_ns, Change *changes, int capacity)
	{
		int n = 0;
		if (t_ns == m_lastT) {
			return 0; // the same camera frame again (read every SteamVR frame)
		}
		m_lastT = t_ns;
		const auto note = [&](int g, Phase to, const char *why) {
			if (m_phase[g] != to && n < capacity) {
				changes[n++] = Change{g, m_phase[g], to, why};
			}
			m_phase[g] = to;
			m_since[g] = t_ns;
		};
		bool any_valid_gesture = false;
		for (const Gesture &g : m_hand.gestures) {
			any_valid_gesture = any_valid_gesture || (g.enabled && confidence >= g.min_confidence);
		}
		if (!valid || !any_valid_gesture) {
			// Tracking lost: hold everything for the grace time, then let go.
			if (m_lastValid != 0 && t_ns - m_lastValid > m_grace) {
				for (size_t g = 0; g < m_phase.size(); g++) {
					note((int)g, Phase::Idle, "tracking lost");
				}
				m_count = m_next = 0;
				m_lastValid = 0;
			}
			return n;
		}
		if (m_lastValid != 0 && t_ns - m_lastValid > m_grace) {
			// Found again after a gap: fresh start.
			for (size_t g = 0; g < m_phase.size(); g++) {
				note((int)g, Phase::Idle, "tracking found again");
			}
			m_count = m_next = 0;
		}
		m_lastValid = t_ns;

		// Smoothing: the mean of the last smooth_frames measurements (a fixed ring, no allocation per frame).
		for (int i = 0; i < kFeatures; i++) {
			m_hist[m_next][i] = features[i];
		}
		m_next = (m_next + 1) % m_smooth;
		m_count = m_count < m_smooth ? m_count + 1 : m_smooth;
		for (int i = 0; i < kFeatures; i++) {
			float s = 0.0f;
			for (int k = 0; k < m_count; k++) {
				s += m_hist[k][i];
			}
			m_smoothed[i] = s / (float)m_count;
		}

		int active = activeIndex();
		for (size_t gi = 0; gi < m_hand.gestures.size(); gi++) {
			const Gesture &g = m_hand.gestures[gi];
			const int i = (int)gi;
			const bool usable = g.enabled && !g.terms.empty() && confidence >= g.min_confidence;
			const bool on = usable && holds(g, true);
			const bool keep = usable && holds(g, false);
			switch (m_phase[i]) {
			case Phase::Idle:
				if (on) {
					note(i, Phase::Candidate, "on condition holds");
				}
				break;
			case Phase::Candidate:
				if (!on) {
					note(i, Phase::Idle, "on condition stopped holding");
				} else if (t_ns - m_since[i] >= g.activate_ns) {
					if (active >= 0 && active != i && m_hand.gestures[active].priority >= g.priority) {
						break; // another gesture of at least this priority is active: stay a candidate
					}
					if (active >= 0 && active != i) {
						note(active, Phase::Idle, "replaced by a higher-priority gesture");
					}
					note(i, Phase::Active, "held long enough");
					active = i;
				}
				break;
			case Phase::Active:
				if (!keep) {
					note(i, Phase::Releasing, "off condition failed");
				}
				break;
			case Phase::Releasing:
				if (keep) {
					m_phase[i] = Phase::Active; // the dip was shorter than release_ms: still on, no change
				} else if (t_ns - m_since[i] >= g.release_ns) {
					note(i, Phase::Idle, "released");
					if (active == i) {
						active = -1;
					}
				}
				break;
			}
		}
		return n;
	}

	//! Lets go of everything at once (e.g. the hand isn't tracked).
	int
	reset(int64_t t_ns, const char *why, Change *changes, int capacity)
	{
		int n = 0;
		for (size_t g = 0; g < m_phase.size(); g++) {
			if (m_phase[g] != Phase::Idle && n < capacity) {
				changes[n++] = Change{(int)g, m_phase[g], Phase::Idle, why};
			}
			m_phase[g] = Phase::Idle;
			m_since[g] = t_ns;
		}
		m_count = m_next = 0;
		m_lastValid = 0;
		m_lastT = 0;
		return n;
	}

	//! The active gesture (active or releasing), or -1.
	int
	activeIndex() const
	{
		for (size_t g = 0; g < m_phase.size(); g++) {
			if (m_phase[g] == Phase::Active || m_phase[g] == Phase::Releasing) {
				return (int)g;
			}
		}
		return -1;
	}

	//! Whether an enabled gesture with this output exists (the rules decide it).
	bool
	decides(Output o) const
	{
		for (const Gesture &g : m_hand.gestures) {
			if (g.enabled && !g.terms.empty() && g.output == o) {
				return true;
			}
		}
		return false;
	}

	//! Whether the active gesture has this output.
	bool
	activeOutput(Output o) const
	{
		const int a = activeIndex();
		return a >= 0 && m_hand.gestures[a].output == o;
	}

	const HandConfig &
	hand() const
	{
		return m_hand;
	}
	Phase
	phase(int g) const
	{
		return m_phase[g];
	}
	const float *
	smoothed() const
	{
		return m_smoothed;
	}

private:
	bool
	holds(const Gesture &g, bool on) const
	{
		for (const Term &t : g.terms) {
			const float x = m_smoothed[t.feature];
			const float th = on ? t.on : t.off;
			if (t.below ? !(x <= th) : !(x >= th)) {
				return false;
			}
		}
		return true;
	}

	HandConfig m_hand;
	int m_smooth = 1;
	int64_t m_grace = 100'000'000;
	std::vector<Phase> m_phase;
	std::vector<int64_t> m_since;
	static constexpr int kMaxSmooth = 30;
	float m_hist[kMaxSmooth][kFeatures] = {};
	int m_count = 0, m_next = 0;
	float m_smoothed[kFeatures] = {};
	int64_t m_lastValid = 0;
	int64_t m_lastT = 0;
};

//! The config file's path ($XDG_CONFIG_HOME or ~/.config, then monado/psvr2-gestures.json); empty without a home.
inline std::string
configPath()
{
	const char *xdg = getenv("XDG_CONFIG_HOME");
	const char *home = getenv("HOME");
	if (xdg != nullptr && xdg[0] != '\0') {
		return std::string(xdg) + "/monado/psvr2-gestures.json";
	}
	return home != nullptr ? std::string(home) + "/.config/monado/psvr2-gestures.json" : std::string();
}

/*!
 * The gesture config: the file at configPath() if it exists and parses, else the built-in one (@p note says which and
 * why).
 */
inline Config
loadConfig(const char *builtin, std::string *note)
{
	// OVRN_GESTURE_RULES=<file>: another config for a test (e.g. psvr2-test hands/configs/pinch-formula.json).
	const char *over = getenv("OVRN_GESTURE_RULES");
	const std::string path = over != nullptr && over[0] != '\0' ? std::string(over) : configPath();
	FILE *f = path.empty() ? nullptr : fopen(path.c_str(), "rb");
	if (f != nullptr) {
		std::string text;
		char buf[4096];
		size_t r;
		while ((r = fread(buf, 1, sizeof(buf), f)) > 0 && text.size() < (1u << 20)) {
			text.append(buf, r);
		}
		fclose(f);
		std::string err;
		Config c = parse(text.c_str(), &err);
		if (c.valid) {
			c.source = path;
			*note = "from " + path;
			return c;
		}
		*note = path + " not used (" + err + "); built-in config";
	} else {
		*note = "built-in config (no " + (path.empty() ? std::string("config file") : path) + ")";
	}
	std::string err;
	Config c = parse(builtin, &err);
	c.source = "built in";
	return c;
}

} // namespace ovrn::gest::rules
