// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The PS VR2 Sense controller's hand skeleton for SteamVR (/input/skeleton/left|right), driven by the Sense's
 *         finger sensors.
 *
 * Sony's driver gives each Sense controller a skeleton input; games built on Unreal's SteamVR plugin (AMID EVIL VR,
 * Hellsweeper VR) bind their hand animation to it and hung the GPU with our driver while it had none (their logs:
 * "skeleton/left ... missing path invalid device path for skeleton output", 2026-11-19). Every pose here has all 31
 * bones valid, from the first frame on.
 *
 * How the hand is made:
 *  - The hand itself is Valve's reference hand, read at startup from SteamVR's own resources
 *    (resources/skeletons/vr_glove_left|right_skeleton.glb, the skeleton SteamVR's glove model is skinned to), so the
 *    bones' lengths and frames are exactly the ones SteamVR and games expect. Without that file a plain built-in hand
 *    of the same layout is used, so the skeleton still works.
 *  - It is fitted to the Sense once, in the controller's grip pose (the pose this driver reports; OpenXR's grip: its
 *    Z axis runs through the tube the fingers make around the handle, the left palm faces +X, the right palm -X, the
 *    index finger toward -Z): the palm lies across the handle, the middle finger lands on the grip sensor, the
 *    fingers close until they touch the handle, the index fingertip rests on the trigger and the thumb on the stick
 *    and on each face button. The control positions come from Sony's Sense model (the controller's dimensions).
 *  - Every frame the Sense's sensors drive it: how near the index finger is to the trigger, trigger touch and pull;
 *    how near the middle finger is to the grip, grip touch and click; and which control the thumb touches. The fingers
 *    move smoothly between the fitted poses.
 *
 * The motion is modelled after ALVR's skeleton (github.com/alvr-org/ALVR, server_openvr Controller.cpp, MIT), our
 * reference implementation for SteamVR controller skeletons; the hand, its fit and the sensor-driven motion are our
 * own.
 * @ingroup st_ovrn
 */

#pragma once

#include "openvr_driver.h"

#include "util/u_json.h"

#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

namespace ovrn {
namespace skel {

constexpr uint32_t kBoneCount = 31; //!< OpenVR's hand skeleton (HandSkeletonBone_Count)

//! Which control the thumb is on. A: square (left) / cross (right); B: triangle (left) / circle (right).
enum class Thumb
{
	None,
	Stick,
	ButtonA,
	ButtonB,
};

//! The finger state the skeleton follows (the Sense's sensors).
struct HandInput
{
	float trigger = 0.0f;           //!< 0..1
	bool trigger_touch = false;     //!< index finger on the trigger
	float trigger_proximity = 0.0f; //!< 0..1, how near the index finger is to the trigger
	float grip = 0.0f;              //!< 0..1 (the Sense's L1/R1 is a click: 0 or 1)
	bool grip_touch = false;        //!< middle finger on the grip
	float grip_proximity = 0.0f;    //!< 0..1, how near the middle finger is to the grip
	Thumb thumb = Thumb::None;
};

namespace detail {

struct V
{
	float x = 0, y = 0, z = 0;
};
struct Q
{
	float w = 1, x = 0, y = 0, z = 0;
};

inline V
operator+(V a, V b)
{
	return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline V
operator-(V a, V b)
{
	return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline V
operator*(V a, float s)
{
	return {a.x * s, a.y * s, a.z * s};
}
inline float
dot(V a, V b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline V
cross(V a, V b)
{
	return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float
len(V a)
{
	return std::sqrt(dot(a, a));
}
inline V
unit(V a)
{
	const float n = len(a);
	return n > 1e-9f ? a * (1.0f / n) : V{1, 0, 0};
}
inline Q
operator*(Q a, Q b)
{
	return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
	        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
inline Q
normalized(Q q)
{
	const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (!(n > 1e-9f) || !std::isfinite(n)) {
		return Q{};
	}
	return {q.w / n, q.x / n, q.y / n, q.z / n};
}
inline V
rotate(Q q, V v)
{
	const V u{q.x, q.y, q.z};
	const V t = cross(u, v) * 2.0f;
	return v + t * q.w + cross(u, t);
}
inline Q
about(V axis, float angle)
{
	const V a = unit(axis);
	const float s = std::sin(angle * 0.5f);
	return {std::cos(angle * 0.5f), a.x * s, a.y * s, a.z * s};
}
//! The rotation whose matrix has the columns c0, c1, c2 (an orthonormal right-handed basis).
inline Q
from_columns(V c0, V c1, V c2)
{
	const float m[3][3] = {{c0.x, c1.x, c2.x}, {c0.y, c1.y, c2.y}, {c0.z, c1.z, c2.z}};
	const float tr = m[0][0] + m[1][1] + m[2][2];
	Q q;
	if (tr > 0) {
		const float s = std::sqrt(tr + 1.0f) * 2.0f;
		q = {0.25f * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s};
	} else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
		const float s = std::sqrt(1.0f + m[0][0] - m[1][1] - m[2][2]) * 2.0f;
		q = {(m[2][1] - m[1][2]) / s, 0.25f * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s};
	} else if (m[1][1] > m[2][2]) {
		const float s = std::sqrt(1.0f + m[1][1] - m[0][0] - m[2][2]) * 2.0f;
		q = {(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25f * s, (m[1][2] + m[2][1]) / s};
	} else {
		const float s = std::sqrt(1.0f + m[2][2] - m[0][0] - m[1][1]) * 2.0f;
		q = {(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25f * s};
	}
	return normalized(q);
}
inline float
deg(float d)
{
	return d * 3.14159265f / 180.0f;
}
inline float
clamp01(float v)
{
	return std::isfinite(v) ? (v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v)) : 0.0f;
}
//! 0 below @p a, 1 above @p b, smooth in between.
inline float
ramp(float v, float a, float b)
{
	const float t = clamp01((clamp01(v) - a) / (b - a));
	return t * t * (3.0f - 2.0f * t);
}

/*
 * OpenVR's bone order: 0 root, 1 wrist, 2-5 thumb (0, 1, 2, end), 6-10 index (metacarpal, 0, 1, 2, end), 11-15
 * middle, 16-20 ring, 21-25 pinky, 26-30 the aux bones (thumb..pinky).
 */
inline int
parent_of(int bone)
{
	if (bone <= 0) {
		return -1;
	}
	if (bone == 1 || bone >= 26) {
		return bone == 1 ? 0 : 0;
	}
	if (bone == 2 || bone == 6 || bone == 11 || bone == 16 || bone == 21) {
		return 1;
	}
	return bone - 1;
}

constexpr int kFingerFirst[5] = {2, 6, 11, 16, 21}; //!< first bone of each finger (thumb: thumb_0, others: metacarpal)
constexpr int kLastJoint[5] = {4, 9, 14, 19, 24};   //!< the last joint of each finger (what its aux bone follows)
constexpr int kTip[5] = {5, 10, 15, 20, 25};

inline std::vector<std::string>
bone_names(bool left)
{
	const std::string s = left ? "l" : "r";
	std::vector<std::string> n = {"Root", "wrist_" + s};
	n.push_back("finger_thumb_0_" + s);
	n.push_back("finger_thumb_1_" + s);
	n.push_back("finger_thumb_2_" + s);
	n.push_back("finger_thumb_" + s + "_end");
	for (const char *f : {"index", "middle", "ring", "pinky"}) {
		n.push_back(std::string("finger_") + f + "_meta_" + s);
		n.push_back(std::string("finger_") + f + "_0_" + s);
		n.push_back(std::string("finger_") + f + "_1_" + s);
		n.push_back(std::string("finger_") + f + "_2_" + s);
		n.push_back(std::string("finger_") + f + "_" + s + "_end");
	}
	for (const char *f : {"thumb", "index", "middle", "ring", "pinky"}) {
		n.push_back(std::string("finger_") + f + "_" + s + "_aux");
	}
	return n;
}

//! The hand at rest: each bone's position and rotation in its parent.
struct Rest
{
	V pos[kBoneCount];
	Q rot[kBoneCount];
};

//! Valve's reference hand from a SteamVR skeleton .glb (glTF binary: a JSON chunk with the bones as nodes).
inline bool
load_glb(const std::string &path, bool left, Rest &out, std::string *why)
{
	FILE *f = fopen(path.c_str(), "rb");
	if (f == nullptr) {
		*why = "can't open " + path;
		return false;
	}
	std::vector<unsigned char> data;
	unsigned char buf[65536];
	size_t n;
	while ((n = fread(buf, 1, sizeof(buf), f)) > 0 && data.size() < (32u << 20)) {
		data.insert(data.end(), buf, buf + n);
	}
	fclose(f);
	auto u32 = [&](size_t at) {
		return (uint32_t)data[at] | (uint32_t)data[at + 1] << 8 | (uint32_t)data[at + 2] << 16 |
		       (uint32_t)data[at + 3] << 24;
	};
	if (data.size() < 20 || u32(0) != 0x46546C67u || u32(16) != 0x4E4F534Au || 20 + (size_t)u32(12) > data.size()) {
		*why = path + " is not a glTF binary with a JSON chunk";
		return false;
	}
	cJSON *root = cJSON_ParseWithLength(reinterpret_cast<const char *>(&data[20]), u32(12));
	const cJSON *nodes = root != nullptr ? cJSON_GetObjectItemCaseSensitive(root, "nodes") : nullptr;
	if (!cJSON_IsArray(nodes)) {
		cJSON_Delete(root);
		*why = path + ": no nodes";
		return false;
	}
	auto numbers = [](const cJSON *node, const char *key, float *out, int count) {
		const cJSON *a = cJSON_GetObjectItemCaseSensitive(node, key);
		if (!cJSON_IsArray(a) || cJSON_GetArraySize(a) != count) {
			return false;
		}
		for (int i = 0; i < count; i++) {
			const cJSON *v = cJSON_GetArrayItem(a, i);
			if (!cJSON_IsNumber(v) || !std::isfinite(v->valuedouble)) {
				return false;
			}
			out[i] = (float)v->valuedouble;
		}
		return true;
	};
	const std::vector<std::string> names = bone_names(left);
	const cJSON *found[kBoneCount] = {};
	const cJSON *node = nullptr;
	cJSON_ArrayForEach(node, nodes)
	{
		const cJSON *name = cJSON_GetObjectItemCaseSensitive(node, "name");
		if (!cJSON_IsString(name)) {
			continue;
		}
		for (uint32_t i = 0; i < kBoneCount; i++) {
			if (names[i] == name->valuestring) {
				found[i] = node;
			}
		}
	}
	bool ok = true;
	float scale = 1.0f; // the bones are in the Root node's units (Valve's: centimetre-sized numbers, Root scale 100)
	for (uint32_t i = 0; i < kBoneCount && ok; i++) {
		if (found[i] == nullptr) {
			*why = path + ": no bone " + names[i];
			ok = false;
			break;
		}
		float t[3] = {0, 0, 0}, r[4] = {0, 0, 0, 1}, s[3] = {1, 1, 1};
		numbers(found[i], "translation", t, 3);
		numbers(found[i], "rotation", r, 4);
		if (i == 0 && numbers(found[i], "scale", s, 3)) {
			scale = s[0];
		}
		const float k = i == 0 ? 1.0f : scale;
		out.pos[i] = {t[0] * k, t[1] * k, t[2] * k};
		out.rot[i] = normalized(Q{r[3], r[0], r[1], r[2]});
	}
	cJSON_Delete(root);
	if (ok && !(scale > 0.0f && scale < 1000.0f)) {
		*why = path + ": implausible scale";
		ok = false;
	}
	return ok;
}

/*!
 * A plain hand of the same layout (bones along their x axis, curling about their z axis), for when SteamVR's
 * reference hand can't be read. Made for the left hand; the right hand is its mirror image, built the way Valve's
 * right hand relates to its left one (positions mirrored in x; bones hanging on the wrist or root turned
 * (w,x,y,z) -> (x,-w,z,-y), the others unchanged — exact for SteamVR's two reference hands).
 */
inline Rest
builtin_rest(bool left)
{
	Rest r;
	for (uint32_t i = 0; i < kBoneCount; i++) {
		r.pos[i] = {};
		r.rot[i] = Q{};
	}
	// left hand: fingers along +Z from the wrist, index toward +Y, palm toward +X (curling turns +Z toward +X)
	const float spread_y[4] = {0.022f, 0.006f, -0.010f, -0.024f}; // index..pinky knuckle side offsets
	const float meta_len[4] = {0.072f, 0.070f, 0.066f, 0.060f};
	const float lens[4][4] = {
	    {0.044f, 0.028f, 0.022f, 0}, {0.048f, 0.031f, 0.024f, 0}, {0.044f, 0.029f, 0.023f, 0}, {0.034f, 0.021f, 0.019f, 0}};
	const Q along_z = about({0, 1, 0}, deg(-90)) * about({1, 0, 0}, deg(-90)); // x -> +Z, z (curl axis) -> +Y
	for (int f = 0; f < 4; f++) {
		const int b = kFingerFirst[f + 1];
		const float fan = deg(6.0f * (1.5f - (float)f));
		r.pos[b] = {0.0f, spread_y[f] * 0.5f, 0.012f};
		r.rot[b] = about({1, 0, 0}, -fan) * along_z;
		r.pos[b + 1] = {meta_len[f], 0, 0};
		r.rot[b + 1] = about({0, 0, 1}, deg(10));
		r.pos[b + 2] = {lens[f][0], 0, 0};
		r.rot[b + 2] = about({0, 0, 1}, deg(8));
		r.pos[b + 3] = {lens[f][1], 0, 0};
		r.rot[b + 3] = about({0, 0, 1}, deg(5));
		r.pos[b + 4] = {lens[f][2], 0, 0};
	}
	r.pos[2] = {0.008f, 0.020f, 0.025f};
	r.rot[2] = about({1, 0, 0}, deg(-45)) * about({0, 1, 0}, deg(-20)) * along_z;
	r.pos[3] = {0.040f, 0, 0};
	r.rot[3] = about({0, 0, 1}, deg(15));
	r.pos[4] = {0.032f, 0, 0};
	r.rot[4] = about({0, 0, 1}, deg(10));
	r.pos[5] = {0.030f, 0, 0};
	if (!left) {
		for (uint32_t i = 1; i < kBoneCount; i++) {
			r.pos[i].x = -r.pos[i].x;
			const int p = parent_of((int)i);
			if (p <= 1 && i != 1) {
				const Q q = r.rot[i];
				r.rot[i] = Q{q.x, -q.w, q.z, -q.y};
			}
		}
	}
	return r;
}

//! Where SteamVR is installed: OVRN_STEAMVR_DIR (tests), else from the running vrserver (<SteamVR>/bin/linux64).
inline std::string
steamvr_dir()
{
	const char *e = getenv("OVRN_STEAMVR_DIR");
	if (e != nullptr && *e != '\0') {
		return e;
	}
	char exe[4096];
	const ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
	if (n <= 0) {
		return {};
	}
	exe[n] = '\0';
	std::string d(exe);
	for (int i = 0; i < 3; i++) {
		const size_t s = d.find_last_of('/');
		if (s == std::string::npos) {
			return {};
		}
		d.resize(s);
	}
	return d;
}

/*!
 * Valve's reference hand from SteamVR's resources, or the built-in hand if it can't be read; @p source says which.
 */
inline Rest
reference_rest(bool left, std::string *source)
{
	const std::string dir = steamvr_dir();
	const std::string path =
	    dir.empty() ? std::string() : dir + "/resources/skeletons/vr_glove_" + (left ? "left" : "right") + "_skeleton.glb";
	std::string why;
	Rest rest;
	if (!path.empty() && load_glb(path, left, rest, &why)) {
		*source = "SteamVR's reference hand (" + path + ")";
		return rest;
	}
	*source = "the built-in hand (" + (path.empty() ? std::string("SteamVR's folder not found") : why) + ")";
	return builtin_rest(left);
}

/*
 * The Sense's controls in its grip pose (metres, left controller; the right one mirrors x), from Sony's Sense model:
 * each control's position, turned into the grip pose's frame. Sony's model angles turn the opposite way to a
 * right-handed rotation: read that way the stick sits at the front end of the handle axis, the trigger under it and
 * the grip sensor on the middle finger's side, as on the controller.
 */
constexpr float kTrigger[3] = {-0.00200f, -0.01831f, -0.04489f};
constexpr float kStick[3] = {-0.00729f, 0.00466f, -0.04699f};
constexpr float kButtonA[3] = {0.01390f, 0.02003f, -0.03291f}; // square / cross
constexpr float kButtonB[3] = {0.01838f, 0.01598f, -0.04565f}; // triangle / circle
constexpr float kGripSensor[3] = {0.00914f, -0.00738f, -0.01334f};
constexpr float kFaceNormal[3] = {0.0f, 0.94883f, -0.31565f}; // out of the face the stick and buttons are on
constexpr float kHandleRadius = 0.019f;                        // the grip's radius at the fingers
constexpr float kFingerHalf = 0.009f;                          // bone line to finger surface
constexpr float kPalmHalf = 0.013f;                            // bone line to palm surface
constexpr float kPalmAt = 0.85f;      // the handle lies across the far palm (power grip), as a share wrist->knuckles
constexpr float kPalmTiltDeg = 20.0f; // ... at a slant: the wrist sits back
constexpr float kCurlMaxDeg[3] = {85.0f, 100.0f, 70.0f}; // full curl over the rest pose: knuckle, middle, last joint

inline V
sense_point(const float p[3], bool left)
{
	return {left ? p[0] : -p[0], p[1], p[2]};
}

/*!
 * How every finger is bent away from the rest pose (radians): per finger the spread at its first finger joint and the
 * bend of its three joints; the thumb's first joint also turns about its y axis (spread).
 */
struct Articulation
{
	float finger[5][4] = {}; //!< thumb, index, middle, ring, pinky: spread, joint 1, joint 2, joint 3

	static Articulation
	lerp(const Articulation &a, const Articulation &b, float t)
	{
		Articulation r;
		for (int f = 0; f < 5; f++) {
			for (int j = 0; j < 4; j++) {
				r.finger[f][j] = a.finger[f][j] + (b.finger[f][j] - a.finger[f][j]) * t;
			}
		}
		return r;
	}
};

//! The ratio curl of a finger (index..pinky): k of each joint's full curl.
inline void
set_curl(Articulation &a, int finger, float k)
{
	a.finger[finger][0] = 0.0f;
	for (int j = 0; j < 3; j++) {
		a.finger[finger][j + 1] = deg(kCurlMaxDeg[j]) * k;
	}
}

//! The bones' rotations in their parents for an articulation (the wrist's own rotation and position given).
inline void
articulate(const Rest &rest, const Articulation &a, V wrist_pos, Q wrist_rot, V pos[kBoneCount], Q rot[kBoneCount])
{
	for (uint32_t i = 0; i < kBoneCount; i++) {
		pos[i] = rest.pos[i];
		rot[i] = rest.rot[i];
	}
	pos[0] = {};
	rot[0] = Q{};
	pos[1] = wrist_pos;
	rot[1] = wrist_rot;
	const V Y{0, 1, 0}, Z{0, 0, 1};
	// thumb: thumb_0 spreads (y) and bends (z), thumb_1 and thumb_2 bend
	rot[2] = rest.rot[2] * about(Y, a.finger[0][0]) * about(Z, a.finger[0][1]);
	rot[3] = rest.rot[3] * about(Z, a.finger[0][2]);
	rot[4] = rest.rot[4] * about(Z, a.finger[0][3]);
	// the other fingers: the first finger joint (after the metacarpal) spreads and bends, the next two bend
	for (int f = 1; f < 5; f++) {
		const int b = kFingerFirst[f] + 1;
		rot[b] = rest.rot[b] * about(Y, a.finger[f][0]) * about(Z, a.finger[f][1]);
		rot[b + 1] = rest.rot[b + 1] * about(Z, a.finger[f][2]);
		rot[b + 2] = rest.rot[b + 2] * about(Z, a.finger[f][3]);
	}
}

//! Every bone's pose in the root (the controller's grip pose); the aux bones follow each finger's last joint.
inline void
forward(const V pos[kBoneCount], const Q rot[kBoneCount], V wpos[kBoneCount], Q wrot[kBoneCount])
{
	for (int i = 0; i < 26; i++) {
		const int p = parent_of(i);
		if (p < 0) {
			wpos[i] = pos[i];
			wrot[i] = rot[i];
		} else {
			wpos[i] = wpos[p] + rotate(wrot[p], pos[i]);
			wrot[i] = normalized(wrot[p] * rot[i]);
		}
	}
	for (int f = 0; f < 5; f++) {
		wpos[26 + f] = wpos[kLastJoint[f]];
		wrot[26 + f] = wrot[kLastJoint[f]];
	}
}

inline float
from_axis(V p)
{
	return std::sqrt(p.x * p.x + p.y * p.y);
}

/*!
 * The hand fitted to the Sense: the wrist's place, the finger poses that touch the controls, and how well they do.
 */
struct Fit
{
	V wrist_pos;
	Q wrist_rot;
	Articulation open;    //!< fingers off the controller
	Articulation held;    //!< index on the trigger, the others closed on the grip, thumb on the stick
	Articulation pressed; //!< trigger pulled and grip clicked
	float thumb[4][4] = {}; //!< thumb joints for Thumb::None (lifted), Stick, ButtonA, ButtonB
	float index_error_m = 0, thumb_error_m[4] = {};
};

class Fitter
{
public:
	Fitter(const Rest &rest, bool left) : m_rest(rest), m_left(left) {}

	Fit
	run()
	{
		Fit fit;
		place(fit);
		// index..pinky close until they touch the handle
		Articulation contact;
		float k[5] = {};
		for (int f = 1; f < 5; f++) {
			k[f] = closeOnHandle(fit, f);
			set_curl(contact, f, k[f]);
		}
		// along the handle: the middle fingertip on the grip sensor (the shift is along the axis: curls stay valid)
		{
			V wp[kBoneCount];
			Q wr[kBoneCount];
			pose(fit, contact, wp, wr);
			fit.wrist_pos.z += sense_point(kGripSensor, m_left).z - wp[kTip[2]].z;
		}
		fit.open = Articulation{};
		for (int f = 1; f < 5; f++) {
			set_curl(fit.open, f, 0.12f);
		}
		fit.held = contact;
		// the index fingertip pad rests on the trigger (its surface: a finger's half thickness out from it)
		const V trig = sense_point(kTrigger, m_left) + V{0, -kFingerHalf, 0};
		float idx[4] = {0, 0, 0, 0};
		const float lo[4] = {deg(-25), deg(-20), 0, 0}, hi[4] = {deg(25), deg(90), deg(100), deg(70)};
		fit.index_error_m = solve(fit, fit.held, 1, trig, lo, hi, idx);
		for (int j = 0; j < 4; j++) {
			fit.held.finger[1][j] = idx[j];
		}
		// the thumb on each control (its pad a finger's half thickness above the control), and lifted off
		const V n{kFaceNormal[0], kFaceNormal[1], kFaceNormal[2]};
		const V targets[4] = {sense_point(kStick, m_left) + n * 0.045f + V{0, 0, 0.01f}, // lifted: resting off the controls
		                      sense_point(kStick, m_left) + n * (0.012f + kFingerHalf * 0.5f),
		                      sense_point(kButtonA, m_left) + n * kFingerHalf,
		                      sense_point(kButtonB, m_left) + n * kFingerHalf};
		const float tlo[4] = {deg(-90), deg(-90), 0, 0}, thi[4] = {deg(90), deg(90), deg(60), deg(80)};
		for (int t = 0; t < 4; t++) {
			fit.thumb_error_m[t] = solve(fit, fit.held, 0, targets[t], tlo, thi, fit.thumb[t]);
		}
		for (int j = 0; j < 4; j++) {
			fit.held.finger[0][j] = fit.thumb[1][j];
			fit.open.finger[0][j] = fit.thumb[0][j];
		}
		// pulling the trigger bends the index a little further; clicking the grip squeezes the others
		fit.pressed = fit.held;
		fit.pressed.finger[1][1] += deg(10);
		fit.pressed.finger[1][2] += deg(15);
		fit.pressed.finger[1][3] += deg(10);
		for (int f = 2; f < 5; f++) {
			set_curl(fit.pressed, f, std::fmin(1.0f, k[f] + 0.06f));
		}
		return fit;
	}

private:
	void
	pose(const Fit &fit, const Articulation &a, V wp[kBoneCount], Q wr[kBoneCount]) const
	{
		V pos[kBoneCount];
		Q rot[kBoneCount];
		articulate(m_rest, a, fit.wrist_pos, fit.wrist_rot, pos, rot);
		forward(pos, rot, wp, wr);
	}

	/*
	 * The wrist's place: the palm faces the handle axis (left palm toward +X, right toward -X), the index side toward
	 * -Z, the handle across the far palm at the handle's radius, slanted so the wrist sits back.
	 */
	void
	place(Fit &fit) const
	{
		V wp[kBoneCount];
		Q wr[kBoneCount];
		Fit at_origin;
		pose(at_origin, Articulation{}, wp, wr);
		V knuckles{};
		for (int f = 1; f < 5; f++) {
			knuckles = knuckles + wp[kFingerFirst[f] + 1] * 0.25f;
		}
		const V wrist = wp[1];
		const V fdir = unit(knuckles - wrist);
		V across = wp[kFingerFirst[1] + 1] - wp[kFingerFirst[4] + 1];
		across = unit(across - fdir * dot(across, fdir));
		// the palm's side: where the middle fingertip goes when the finger curls a little
		Articulation bit;
		set_curl(bit, 2, 0.25f);
		V wp2[kBoneCount];
		Q wr2[kBoneCount];
		pose(at_origin, bit, wp2, wr2);
		V d = wp2[kTip[2]] - wp[kTip[2]];
		d = d - fdir * dot(d, fdir) - across * dot(d, across);
		const V palm = unit(d);
		const V tn{m_left ? 1.0f : -1.0f, 0, 0};
		const V ta{0, 0, -1};
		const Q src = from_columns(palm, across, cross(palm, across));
		const Q dst = from_columns(tn, ta, cross(tn, ta));
		const Q r = normalized(about(tn, deg(m_left ? kPalmTiltDeg : -kPalmTiltDeg)) * dst *
		                       Q{src.w, -src.x, -src.y, -src.z});
		const V contact = wrist + (knuckles - wrist) * kPalmAt + palm * kPalmHalf;
		fit.wrist_rot = r;
		fit.wrist_pos = tn * -kHandleRadius - rotate(r, contact);
	}

	//! The smallest curl at which finger @p f touches the handle (its middle joint, last joint or tip).
	float
	closeOnHandle(const Fit &fit, int f) const
	{
		for (int i = 0; i <= 100; i++) {
			Articulation a;
			set_curl(a, f, (float)i / 100.0f);
			V wp[kBoneCount];
			Q wr[kBoneCount];
			pose(fit, a, wp, wr);
			const int b = kFingerFirst[f];
			const float nearest =
			    std::fmin(std::fmin(from_axis(wp[b + 2]), from_axis(wp[b + 3])), from_axis(wp[b + 4]));
			if (nearest <= kHandleRadius + kFingerHalf) {
				return (float)i / 100.0f;
			}
		}
		return 1.0f;
	}

	/*!
	 * Finger @p f's joints (within lo..hi) that put its pad (halfway between its last joint and tip) nearest to
	 * @p target without its joints sinking into the handle: a coarse grid, then coordinate descent. Returns the miss.
	 */
	float
	solve(const Fit &fit, const Articulation &base, int f, V target, const float lo[4], const float hi[4],
	      float out[4]) const
	{
		auto cost = [&](const float x[4]) {
			Articulation a = base;
			for (int j = 0; j < 4; j++) {
				a.finger[f][j] = x[j];
			}
			V wp[kBoneCount];
			Q wr[kBoneCount];
			pose(fit, a, wp, wr);
			const V pad = (wp[kLastJoint[f]] + wp[kTip[f]]) * 0.5f;
			float c = len(pad - target);
			const int b = f == 0 ? 3 : kFingerFirst[f] + 2;
			for (int j = b; j <= kTip[f]; j++) {
				const float inside = kHandleRadius + kFingerHalf * 0.8f - from_axis(wp[j]);
				if (inside > 0 && wp[j].z > -0.06f && wp[j].z < 0.05f) {
					c += inside * 3.0f;
				}
			}
			return c;
		};
		float x[4];
		constexpr int kStarts = 8;
		float starts[kStarts][4] = {};
		float start_c[kStarts];
		for (float &c : start_c) {
			c = 1e9f;
		}
		const int steps = 6;
		int idx[4];
		for (idx[0] = 0; idx[0] < steps; idx[0]++) {
			for (idx[1] = 0; idx[1] < steps; idx[1]++) {
				for (idx[2] = 0; idx[2] < steps; idx[2]++) {
					for (idx[3] = 0; idx[3] < steps; idx[3]++) {
						for (int j = 0; j < 4; j++) {
							x[j] = lo[j] + (hi[j] - lo[j]) * (float)idx[j] / (float)(steps - 1);
						}
						const float c = cost(x);
						int worst = 0;
						for (int k = 1; k < kStarts; k++) {
							worst = start_c[k] > start_c[worst] ? k : worst;
						}
						if (c < start_c[worst]) {
							start_c[worst] = c;
							memcpy(starts[worst], x, sizeof(x));
						}
					}
				}
			}
		}
		float best[4] = {};
		float best_c = 1e9f;
		for (int k = 0; k < kStarts; k++) {
			float cur[4];
			memcpy(cur, starts[k], sizeof(cur));
			float cur_c = start_c[k];
			float step = deg(8);
			while (step > deg(0.05f)) {
				bool better = false;
				for (int j = 0; j < 4; j++) {
					for (float s : {step, -step}) {
						memcpy(x, cur, sizeof(x));
						x[j] = std::fmax(lo[j], std::fmin(hi[j], x[j] + s));
						const float c = cost(x);
						if (c < cur_c - 1e-7f) {
							cur_c = c;
							memcpy(cur, x, sizeof(cur));
							better = true;
						}
					}
				}
				if (!better) {
					step *= 0.5f;
				}
			}
			if (cur_c < best_c) {
				best_c = cur_c;
				memcpy(best, cur, sizeof(best));
			}
		}
		memcpy(out, best, sizeof(best));
		Articulation a = base;
		for (int j = 0; j < 4; j++) {
			a.finger[f][j] = best[j];
		}
		V wp[kBoneCount];
		Q wr[kBoneCount];
		pose(fit, a, wp, wr);
		return len((wp[kLastJoint[f]] + wp[kTip[f]]) * 0.5f - target);
	}

	const Rest &m_rest;
	const bool m_left;
};

} // namespace detail

/*!
 * One hand's skeleton. Call update() once per frame with the Sense's finger state, then pose() for each motion range.
 */
class HandSkeleton
{
public:
	static constexpr float kSmoothingS = 0.035f; //!< how quickly the fingers follow the sensors

	explicit HandSkeleton(bool left)
	{
		using namespace detail;
		m_rest = reference_rest(left, &m_source);
		m_fit = Fitter(m_rest, left).run();
		m_current = m_fit.held;
		m_currentFree = Articulation{};
	}

	//! Where the hand comes from and how well it fits the Sense, for the log.
	std::string
	describe() const
	{
		char b[256];
		snprintf(b, sizeof(b),
		         "index on the trigger %.0f mm, thumb on the stick %.0f / buttons %.0f, %.0f mm (lifted %.0f mm)",
		         m_fit.index_error_m * 1000, m_fit.thumb_error_m[1] * 1000, m_fit.thumb_error_m[2] * 1000,
		         m_fit.thumb_error_m[3] * 1000, m_fit.thumb_error_m[0] * 1000);
		return m_source + "; fitted to the Sense: " + b;
	}

	bool
	fromSteamVR() const
	{
		return m_source.rfind("SteamVR", 0) == 0;
	}

	void
	update(const HandInput &input)
	{
		using namespace detail;
		const int64_t now = now_ns();
		float dt = m_lastNs == 0 ? 1.0f : (float)(now - m_lastNs) * 1e-9f;
		m_lastNs = now;
		dt = dt < 0.0f ? 0.0f : (dt > 1.0f ? 1.0f : dt);
		const float a = 1.0f - std::exp(-dt / kSmoothingS);
		m_current = Articulation::lerp(m_current, target(input, true), a);
		m_currentFree = Articulation::lerp(m_currentFree, target(input, false), a);
	}

	//! All 31 bones for one motion range (with the controller in hand, or the hand as if it held none).
	void
	pose(bool with_controller, vr::VRBoneTransform_t out[kBoneCount]) const
	{
		using namespace detail;
		V pos[kBoneCount], wp[kBoneCount];
		Q rot[kBoneCount], wr[kBoneCount];
		articulate(m_rest, with_controller ? m_current : m_currentFree, m_fit.wrist_pos, m_fit.wrist_rot, pos, rot);
		forward(pos, rot, wp, wr);
		for (uint32_t i = 0; i < kBoneCount; i++) {
			const bool aux = i >= 26;
			V p = aux ? wp[i] : pos[i];
			Q q = normalized(aux ? wr[i] : rot[i]);
			if (aux || i == 1) { // the root's children: from the grip into the root
				p = m_gripPos + rotate(m_gripRot, p);
				q = normalized(m_gripRot * q);
			}
			out[i].position = {{p.x, p.y, p.z, 1.0f}};
			out[i].orientation = {q.w, q.x, q.y, q.z};
		}
	}

	/*!
	 * Where the grip (the frame the hand is fitted in) is in the skeleton's root, the controller pose the driver
	 * reports: the wrist and the aux bones (the root's children) are placed through it. Default: the same.
	 */
	void
	setGripInRoot(detail::V position, detail::Q orientation)
	{
		m_gripPos = position;
		m_gripRot = detail::normalized(orientation);
	}

	//! The fitted poses (for the tests).
	const detail::Fit &
	fit() const
	{
		return m_fit;
	}

private:
	static int64_t
	now_ns()
	{
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC, &ts);
		return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
	}

	//! The pose the sensors ask for (smoothed into m_current / m_currentFree by update()).
	detail::Articulation
	target(const HandInput &in, bool with_controller) const
	{
		using namespace detail;
		const float trigger = clamp01(in.trigger);
		const float grip = clamp01(in.grip);
		const float near_index = in.trigger_touch || trigger > 0.0f ? 1.0f : ramp(in.trigger_proximity, 0.15f, 0.85f);
		const float near_grip = in.grip_touch || grip > 0.0f ? 1.0f : ramp(in.grip_proximity, 0.15f, 0.85f);
		const int thumb = in.thumb == Thumb::Stick ? 1 : in.thumb == Thumb::ButtonA ? 2 : in.thumb == Thumb::ButtonB ? 3 : 0;
		Articulation a;
		if (with_controller) {
			const Articulation idle = Articulation::lerp(m_fit.open, m_fit.held, near_index);
			const Articulation pulled = Articulation::lerp(m_fit.held, m_fit.pressed, trigger);
			const Articulation &index = trigger > 0.0f ? pulled : idle;
			const Articulation others_idle = Articulation::lerp(m_fit.open, m_fit.held, near_grip);
			const Articulation others = Articulation::lerp(others_idle, m_fit.pressed, grip);
			for (int j = 0; j < 4; j++) {
				a.finger[1][j] = index.finger[1][j];
				for (int f = 2; f < 5; f++) {
					a.finger[f][j] = others.finger[f][j];
				}
				a.finger[0][j] = m_fit.thumb[thumb][j];
			}
		} else {
			// no controller in the hand: the same sensors open and close a fist
			set_curl(a, 1, 0.05f + 0.45f * near_index + 0.5f * trigger);
			for (int f = 2; f < 5; f++) {
				set_curl(a, f, 0.08f + 0.72f * near_grip + 0.2f * grip);
			}
			const float fold = thumb == 0 ? 0.0f : 0.7f;
			for (int j = 0; j < 4; j++) {
				a.finger[0][j] = m_fit.thumb[1][j] * fold;
			}
		}
		return a;
	}

	detail::Rest m_rest;
	detail::Fit m_fit;
	std::string m_source;
	detail::Articulation m_current, m_currentFree;
	int64_t m_lastNs = 0;
	detail::V m_gripPos;
	detail::Q m_gripRot;
};

} // namespace skel
} // namespace ovrn
