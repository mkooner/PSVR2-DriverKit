// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Room (playspace) calibration for the SteamVR driver, see ovrd_room.hpp.
 *
 * The chaperone file handling follows lbgos/rift-s-linux (ovrd_chaperone_origin.cpp, BSL-1.0): seed a plain entry
 * for the universe Room Setup draws in, write atomically and never over a concurrent SteamVR update.
 *
 * @ingroup st_ovrn
 */

#include "ovrn_room.hpp"

#include "math/m_api.h"
#include "util/u_json.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <memory>
#include <vector>

#ifdef __linux__
#include <sys/stat.h>
#include <unistd.h>
#endif


namespace {

using Json = std::unique_ptr<cJSON, decltype(&cJSON_Delete)>;

constexpr int ROOM_FILE_VERSION = 1;

std::string
read_file(const std::string &path, size_t maximum, bool *out_exists)
{
	std::ifstream file(path, std::ios::binary);
	if (out_exists != nullptr) {
		*out_exists = (bool)file;
	}
	if (!file) {
		return {};
	}
	file.seekg(0, std::ios::end);
	auto size = file.tellg();
	if (size < 0 || size > std::streamoff(maximum)) {
		return {};
	}
	std::string bytes(size_t(size), '\0');
	file.seekg(0);
	return file.read(bytes.data(), size) ? bytes : std::string{};
}

/*!
 * Write @p bytes to @p path atomically. If @p expected is given, the file must still hold it (or still not exist,
 * for an empty @p expected with @p expect_missing) when renaming, so a concurrent SteamVR write is never overwritten.
 */
bool
write_file_atomic(const std::string &path, const std::string &bytes, const std::string *expected, bool expect_missing)
{
#ifdef __linux__
	std::string temporary = path + ".monado-XXXXXX";
	int fd = mkstemp(temporary.data());
	if (fd < 0) {
		return false;
	}
	FILE *file = fdopen(fd, "wb");
	if (file == nullptr) {
		close(fd);
		unlink(temporary.c_str());
		return false;
	}
	bool written = fwrite(bytes.data(), 1, bytes.size(), file) == bytes.size();
	written = fflush(file) == 0 && written;
	written = fsync(fd) == 0 && written;
	written = fclose(file) == 0 && written;
	(void)chmod(temporary.c_str(), 0644);

	bool unchanged = true;
	if (expected != nullptr) {
		bool exists = false;
		std::string now = read_file(path, 4 * 1024 * 1024, &exists);
		unchanged = expect_missing ? !exists : (exists && now == *expected);
	}
	if (!written || !unchanged || rename(temporary.c_str(), path.c_str()) != 0) {
		unlink(temporary.c_str());
		return false;
	}
	return true;
#else
	(void)path;
	(void)bytes;
	(void)expected;
	(void)expect_missing;
	return false;
#endif
}

bool
finite3(const xrt_vec3 &v)
{
	return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

std::string
time_string()
{
	time_t now = time(nullptr);
	char buf[64];
	struct tm tm_now;
	localtime_r(&now, &tm_now);
	strftime(buf, sizeof(buf), "%a %b %d %H:%M:%S %Y", &tm_now);
	return buf;
}

Json
plain_universe(const char *id, float play_area)
{
	char bounds[1024] = "";
	if (play_area > 0.0f) {
		float h = play_area / 2.0f;
		// Four vertical quads, floor to 2.43 m, as SteamVR's own files store a rectangle.
		snprintf(bounds, sizeof(bounds),
		         "\"play_area\":[%.3f,%.3f],\"collision_bounds\":["
		         "[[-%.3f,0,-%.3f],[-%.3f,2.43,-%.3f],[-%.3f,2.43,%.3f],[-%.3f,0,%.3f]],"
		         "[[-%.3f,0,%.3f],[-%.3f,2.43,%.3f],[%.3f,2.43,%.3f],[%.3f,0,%.3f]],"
		         "[[%.3f,0,%.3f],[%.3f,2.43,%.3f],[%.3f,2.43,-%.3f],[%.3f,0,-%.3f]],"
		         "[[%.3f,0,-%.3f],[%.3f,2.43,-%.3f],[-%.3f,2.43,-%.3f],[-%.3f,0,-%.3f]]],",
		         play_area, play_area,   //
		         h, h, h, h, h, h, h, h, //
		         h, h, h, h, h, h, h, h, //
		         h, h, h, h, h, h, h, h, //
		         h, h, h, h, h, h, h, h);
	}
	std::string text = std::string("{\"universeID\":\"") + id + "\"," + bounds +
	                   "\"seated\":{\"translation\":[0,0,0],\"yaw\":0},"
	                   "\"standing\":{\"translation\":[0,0,0],\"yaw\":0},"
	                   "\"time\":\"" +
	                   time_string() + "\"}";
	return Json(cJSON_Parse(text.c_str()), cJSON_Delete);
}

const char *
universe_id(cJSON *entry)
{
	cJSON *id = cJSON_GetObjectItemCaseSensitive(entry, "universeID");
	if (cJSON_IsString(id)) {
		return id->valuestring;
	}
	return nullptr;
}

//! Universe entry equal to @p reference, ignoring "time".
bool
same_universe(cJSON *entry, cJSON *reference)
{
	Json a(cJSON_Duplicate(entry, true), cJSON_Delete);
	Json b(cJSON_Duplicate(reference, true), cJSON_Delete);
	if (!a || !b) {
		return false;
	}
	cJSON_DeleteItemFromObjectCaseSensitive(a.get(), "time");
	cJSON_DeleteItemFromObjectCaseSensitive(b.get(), "time");
	return cJSON_Compare(a.get(), b.get(), true);
}

bool
has_drawn_room(cJSON *entry)
{
	cJSON *bounds = cJSON_GetObjectItemCaseSensitive(entry, "collision_bounds");
	if (cJSON_IsArray(bounds) && cJSON_GetArraySize(bounds) > 0) {
		return true;
	}
	cJSON *area = cJSON_GetObjectItemCaseSensitive(entry, "play_area");
	if (cJSON_IsArray(area) && cJSON_GetArraySize(area) == 2) {
		cJSON *w = cJSON_GetArrayItem(area, 0);
		cJSON *h = cJSON_GetArrayItem(area, 1);
		return cJSON_IsNumber(w) && cJSON_IsNumber(h) && w->valuedouble > 0 && h->valuedouble > 0;
	}
	return false;
}

bool
read_vec3(cJSON *array, xrt_vec3 *out)
{
	if (!cJSON_IsArray(array) || cJSON_GetArraySize(array) != 3) {
		return false;
	}
	float v[3];
	for (int i = 0; i < 3; i++) {
		cJSON *n = cJSON_GetArrayItem(array, i);
		if (!cJSON_IsNumber(n)) {
			return false;
		}
		v[i] = (float)n->valuedouble;
	}
	*out = xrt_vec3{v[0], v[1], v[2]};
	return true;
}

} // namespace


/*
 *
 * Math.
 *
 */

ovrd_room_transform
ovrd_room_transform_identity(void)
{
	return ovrd_room_transform{XRT_QUAT_IDENTITY, xrt_vec3{0, 0, 0}};
}

xrt_quat
ovrd_room_quat_from_yaw(float yaw)
{
	return xrt_quat{0.0f, sinf(yaw * 0.5f), 0.0f, cosf(yaw * 0.5f)};
}

namespace {

xrt_vec3
axis_vector(ovrd_room_heading_axis axis)
{
	switch (axis) {
	case OVRD_ROOM_AXIS_UP: return xrt_vec3{0, 1, 0};
	case OVRD_ROOM_AXIS_RIGHT: return xrt_vec3{1, 0, 0};
	case OVRD_ROOM_AXIS_FORWARD:
	default: return xrt_vec3{0, 0, -1};
	}
}

float
level_fraction(const xrt_quat &orientation, ovrd_room_heading_axis axis)
{
	xrt_vec3 a = axis_vector(axis);
	xrt_vec3 f;
	math_quat_rotate_vec3(&orientation, &a, &f);
	return sqrtf(f.x * f.x + f.z * f.z);
}

} // namespace

float
ovrd_room_heading_of_axis(const xrt_quat &orientation, ovrd_room_heading_axis axis)
{
	xrt_vec3 a = axis_vector(axis);
	xrt_vec3 f;
	math_quat_rotate_vec3(&orientation, &a, &f);
	float horizontal = sqrtf(f.x * f.x + f.z * f.z);
	if (!(horizontal > 0.5f)) { // the axis is within 30 degrees of vertical (or NaN)
		return NAN;
	}
	return atan2f(-f.x, -f.z);
}

float
ovrd_room_heading(const xrt_quat &orientation)
{
	return ovrd_room_heading_of_axis(orientation, OVRD_ROOM_AXIS_FORWARD);
}

ovrd_room_heading_axis
ovrd_room_most_level_axis(const xrt_quat &orientation)
{
	ovrd_room_heading_axis best = OVRD_ROOM_AXIS_FORWARD;
	for (ovrd_room_heading_axis axis : {OVRD_ROOM_AXIS_UP, OVRD_ROOM_AXIS_RIGHT}) {
		if (level_fraction(orientation, axis) > level_fraction(orientation, best)) {
			best = axis;
		}
	}
	return best;
}

xrt_vec3
ovrd_room_apply(const ovrd_room_transform &T, const xrt_vec3 &p)
{
	xrt_vec3 r;
	math_quat_rotate_vec3(&T.rotation, &p, &r);
	return xrt_vec3{r.x + T.translation.x, r.y + T.translation.y, r.z + T.translation.z};
}

ovrd_room_transform
ovrd_room_invert(const ovrd_room_transform &T)
{
	ovrd_room_transform out;
	math_quat_invert(&T.rotation, &out.rotation);
	xrt_vec3 t;
	math_quat_rotate_vec3(&out.rotation, &T.translation, &t);
	out.translation = xrt_vec3{-t.x, -t.y, -t.z};
	return out;
}

ovrd_room_transform
ovrd_room_compose(const ovrd_room_transform &a, const ovrd_room_transform &b)
{
	ovrd_room_transform out;
	math_quat_rotate(&a.rotation, &b.rotation, &out.rotation);
	math_quat_normalize(&out.rotation);
	out.translation = ovrd_room_apply(a, b.translation);
	return out;
}

ovrd_room_transform
ovrd_room_transform_from_capture(const ovrd_room_yaw_pose &capture_raw,
                                 const ovrd_room_yaw_pose *saved,
                                 ovrd_room_yaw_pose *out_anchor)
{
	ovrd_room_yaw_pose anchor;
	if (saved == nullptr) {
		// The room frame is the raw frame with its floor moved to y = 0.
		anchor = ovrd_room_yaw_pose{xrt_vec3{capture_raw.position.x, 0.0f, capture_raw.position.z}, capture_raw.yaw};
	} else {
		anchor = *saved;
		anchor.position.y = 0.0f;
	}

	float turn = anchor.yaw - capture_raw.yaw;
	ovrd_room_transform T;
	T.rotation = ovrd_room_quat_from_yaw(turn);
	xrt_vec3 rotated;
	math_quat_rotate_vec3(&T.rotation, &capture_raw.position, &rotated);
	T.translation = xrt_vec3{anchor.position.x - rotated.x, anchor.position.y - rotated.y,
	                         anchor.position.z - rotated.z};

	if (out_anchor != nullptr) {
		*out_anchor = anchor;
	}
	return T;
}

float
ovrd_room_lowest_point(const xrt_pose &grip, const xrt_vec3 *outline, size_t count, float radius_m)
{
	float lowest = INFINITY;
	for (size_t i = 0; i < count; i++) {
		xrt_vec3 p;
		math_pose_transform_point(&grip, &outline[i], &p);
		lowest = fminf(lowest, p.y);
	}
	return lowest - radius_m;
}


/*
 *
 * The capture gesture.
 *
 */

void
OvrdRoomCapture::restart(const xrt_space_relation &grip, const ovrd_room_heading_axis *axis, int64_t now_ns)
{
	bool complained = m_complained;
	*this = OvrdRoomCapture{};
	m_active = true;
	m_complained = complained;
	m_start_ns = now_ns;
	m_start = grip.pose.position;
	m_axis = axis != nullptr ? *axis : ovrd_room_most_level_axis(grip.pose.orientation);
}

OvrdRoomCapture::Result
OvrdRoomCapture::feed(bool held,
                      const xrt_space_relation &grip,
                      const xrt_vec3 *outline,
                      size_t outline_count,
                      float outline_radius,
                      const xrt_vec3 *head,
                      int64_t last_reset_ns,
                      const ovrd_room_heading_axis *axis,
                      int64_t now_ns,
                      std::string *out_log)
{
	auto log = [&](const std::string &text) {
		if (out_log != nullptr) {
			*out_log += text;
			*out_log += "\n";
		}
	};
	char buf[256];

	if (!held) {
		if (m_active && !m_done) {
			snprintf(buf, sizeof(buf), "capture cancelled (Create let go after %.1f s)",
			         (double)(now_ns - m_start_ns) / 1e9);
			log(buf);
		}
		*this = OvrdRoomCapture{};
		return Result::NONE;
	}
	if (m_done) {
		return Result::NONE; // wait for Create to be let go
	}
	if (!m_active || outline_count == 0) {
		m_complained = false;
		restart(grip, axis, now_ns);
	}

	const int needed = XRT_SPACE_RELATION_POSITION_TRACKED_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
	bool tracked = ((int)grip.relation_flags & needed) == needed;
	xrt_vec3 w = grip.angular_velocity;
	float angular_speed = (grip.relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT) != 0
	                          ? sqrtf(w.x * w.x + w.y * w.y + w.z * w.z)
	                          : 0.0f;
	xrt_vec3 d = {grip.pose.position.x - m_start.x, grip.pose.position.y - m_start.y,
	              grip.pose.position.z - m_start.z};
	float drift = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);

	const char *problem = nullptr;
	if (last_reset_ns != 0 && now_ns - last_reset_ns < AFTER_RESET_NS) {
		problem = "the headset's tracking re-originated in the last 5 s (it's still settling)";
	} else if (!tracked) {
		problem = "the controller isn't position tracked (keep it in the headset's view)";
	} else if (angular_speed > MAX_ANGULAR_SPEED || drift > MAX_DRIFT_M) {
		problem = "the controller moved";
	}
	if (problem != nullptr) {
		// Start again from here; only complain once per hold.
		if (!m_complained) {
			log(std::string("capture restarted: ") + problem);
			m_complained = true;
		}
		restart(grip, axis, now_ns);
		return Result::NONE;
	}

	float heading = ovrd_room_heading_of_axis(grip.pose.orientation, m_axis);
	m_samples++;
	m_sum_lowest += ovrd_room_lowest_point(grip.pose, outline, outline_count, outline_radius);
	m_sum_x += grip.pose.position.x;
	m_sum_z += grip.pose.position.z;
	if (std::isfinite(heading)) {
		m_sum_sin += sin(heading);
		m_sum_cos += cos(heading);
		m_heading_samples++;
	}

	if (now_ns - m_start_ns < HOLD_NS || m_samples < 20) {
		return Result::NONE;
	}
	m_done = true;

	static const char *axis_names[] = {"forward", "up", "right"};
	float floor_y = (float)(m_sum_lowest / m_samples);
	if (m_heading_samples < m_samples / 2) {
		snprintf(buf, sizeof(buf),
		         "capture rejected: the controller is turned differently from when the room was set up (its %s axis "
		         "is near vertical); put it down the same way as then",
		         axis_names[m_axis]);
		log(buf);
		return Result::REJECTED;
	}
	if (head == nullptr || head->y - floor_y < MIN_BELOW_HEAD_M) {
		snprintf(buf, sizeof(buf), "capture rejected: the controller is only %.2f m below the head, not on the floor",
		         head != nullptr ? head->y - floor_y : 0.0f);
		log(buf);
		return Result::REJECTED;
	}

	m_result = ovrd_room_yaw_pose{{(float)(m_sum_x / m_samples), floor_y, (float)(m_sum_z / m_samples)},
	                              (float)atan2(m_sum_sin, m_sum_cos)};
	m_below_head = head->y - floor_y;
	return Result::CAPTURED;
}


/*
 *
 * Files.
 *
 */

std::string
ovrd_room_chaperone_path(void)
{
	const char *home = getenv("HOME");
	const char *config = getenv("XDG_CONFIG_HOME");
	if (home == nullptr && config == nullptr) {
		return {};
	}
	std::string registry_path =
	    (config != nullptr ? std::string(config) : std::string(home) + "/.config") + "/openvr/openvrpaths.vrpath";
	std::string bytes = read_file(registry_path, 64 * 1024, nullptr);
	Json registry(cJSON_Parse(bytes.c_str()), cJSON_Delete);
	cJSON *config_path = cJSON_GetArrayItem(cJSON_GetObjectItemCaseSensitive(registry.get(), "config"), 0);
	if (!cJSON_IsString(config_path) || config_path->valuestring[0] == '\0') {
		return {};
	}
	return std::string(config_path->valuestring) + "/chaperone_info.vrchap";
}

//! The driver's own rectangle in universe 2 (not a room drawn in Room Setup).
static bool
is_driver_rectangle(cJSON *entry)
{
	return cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(entry, "monado_rectangle"));
}

bool
ovrd_room_migrate_universe(const std::string &from, const std::string &to, const char *universe, std::string *out_note)
{
	auto note = [&](const std::string &s) {
		if (out_note != nullptr) {
			*out_note = s;
		}
		return false;
	};
	bool from_exists = false;
	const std::string source_text = read_file(from, 4 * 1024 * 1024, &from_exists);
	if (!from_exists || source_text.empty()) {
		return note("no old file");
	}
	Json source(cJSON_Parse(source_text.c_str()), cJSON_Delete);
	cJSON *source_universes = cJSON_GetObjectItemCaseSensitive(source.get(), "universes");
	if (!cJSON_IsArray(source_universes)) {
		return note("old file is not a chaperone_info document");
	}
	cJSON *drawn = nullptr;
	cJSON *entry = nullptr;
	cJSON_ArrayForEach(entry, source_universes)
	{
		const char *uid = universe_id(entry);
		if (uid != nullptr && strcmp(uid, universe) == 0 && has_drawn_room(entry) && !is_driver_rectangle(entry)) {
			drawn = entry;
		}
	}
	if (drawn == nullptr) {
		return note("no room drawn in the old file");
	}

	bool to_exists = false;
	const std::string target_text = read_file(to, 4 * 1024 * 1024, &to_exists);
	Json target(nullptr, cJSON_Delete);
	if (!to_exists) {
		target.reset(cJSON_Parse("{\"jsonid\":\"chaperone_info\",\"version\":5,\"universes\":[]}"));
	} else if (target_text.empty()) {
		return note("SteamVR's file unreadable or too large");
	} else {
		target.reset(cJSON_Parse(target_text.c_str()));
	}
	cJSON *id = cJSON_GetObjectItemCaseSensitive(target.get(), "jsonid");
	cJSON *universes = cJSON_GetObjectItemCaseSensitive(target.get(), "universes");
	if (!cJSON_IsString(id) || strcmp(id->valuestring, "chaperone_info") != 0 || !cJSON_IsArray(universes)) {
		return note("SteamVR's file is not a chaperone_info document; left untouched");
	}
	cJSON *existing = nullptr;
	cJSON_ArrayForEach(entry, universes)
	{
		const char *uid = universe_id(entry);
		if (uid != nullptr && strcmp(uid, universe) == 0) {
			existing = entry;
		}
	}
	if (existing != nullptr && has_drawn_room(existing) && !is_driver_rectangle(existing)) {
		return note("SteamVR's file already has a room drawn there; kept");
	}
	cJSON *copy = cJSON_Duplicate(drawn, true);
	if (copy == nullptr) {
		return note("internal: copying the room");
	}
	const bool ok = existing != nullptr ? cJSON_ReplaceItemViaPointer(universes, existing, copy)
	                                    : cJSON_AddItemToArray(universes, copy);
	if (!ok) {
		cJSON_Delete(copy);
		return note("internal: inserting the room");
	}
	char *printed = cJSON_Print(target.get());
	if (printed == nullptr) {
		return note("internal: printing SteamVR's file");
	}
	const std::string bytes(printed);
	cJSON_free(printed);
	if (!write_file_atomic(to, bytes, &target_text, !to_exists)) {
		return note("could not write SteamVR's file (or SteamVR changed it meanwhile)");
	}
	if (out_note != nullptr) {
		*out_note = "room copied";
	}
	return true;
}

/*!
 * Whether @p entry holds only the driver's default square of @p size (plain_universe), compared by its numbers:
 * SteamVR rewrites the whole file and drops keys of ours such as "monado_rectangle".
 */
static bool
is_default_square(cJSON *entry, float size)
{
	if (!(size > 0.0f)) {
		return false;
	}
	Json reference = plain_universe("x", size);
	cJSON *a = cJSON_GetObjectItemCaseSensitive(entry, "collision_bounds");
	cJSON *b = cJSON_GetObjectItemCaseSensitive(reference.get(), "collision_bounds");
	if (!cJSON_IsArray(a) || !cJSON_IsArray(b) || cJSON_GetArraySize(a) != cJSON_GetArraySize(b)) {
		return false;
	}
	for (int q = 0; q < cJSON_GetArraySize(b); q++) {
		cJSON *qa = cJSON_GetArrayItem(a, q), *qb = cJSON_GetArrayItem(b, q);
		if (!cJSON_IsArray(qa) || cJSON_GetArraySize(qa) != cJSON_GetArraySize(qb)) {
			return false;
		}
		for (int k = 0; k < cJSON_GetArraySize(qb); k++) {
			xrt_vec3 va, vb;
			if (!read_vec3(cJSON_GetArrayItem(qa, k), &va) || !read_vec3(cJSON_GetArrayItem(qb, k), &vb) ||
			    fabsf(va.x - vb.x) > 2e-3f || fabsf(va.y - vb.y) > 2e-3f || fabsf(va.z - vb.z) > 2e-3f) {
				return false;
			}
		}
	}
	return true;
}

bool
ovrd_room_universe_drawn(const std::string &path, const char *universe, float default_size)
{
	bool exists = false;
	const std::string text = read_file(path, 4 * 1024 * 1024, &exists);
	Json doc(cJSON_Parse(text.c_str()), cJSON_Delete);
	cJSON *universes = cJSON_GetObjectItemCaseSensitive(doc.get(), "universes");
	cJSON *entry = nullptr;
	cJSON_ArrayForEach(entry, universes)
	{
		const char *uid = universe_id(entry);
		if (uid != nullptr && strcmp(uid, universe) == 0) {
			return has_drawn_room(entry) && !is_driver_rectangle(entry) && !is_default_square(entry, default_size);
		}
	}
	return false;
}

bool
ovrd_room_move_universe(const std::string &path,
                        const char *from_universe,
                        const char *to_universe,
                        const struct xrt_vec3 &shift,
                        float default_size,
                        std::string *out_note)
{
	auto note = [&](const std::string &s) {
		if (out_note != nullptr) {
			*out_note = s;
		}
		return false;
	};
	// SteamVR writes the file now and then (Room Setup, origin changes): try again if it did meanwhile.
	for (int attempt = 0; attempt < 3; attempt++) {
		bool exists = false;
		const std::string text = read_file(path, 4 * 1024 * 1024, &exists);
		if (!exists || text.empty()) {
			return note("SteamVR's file missing or unreadable");
		}
		Json doc(cJSON_Parse(text.c_str()), cJSON_Delete);
		cJSON *universes = cJSON_GetObjectItemCaseSensitive(doc.get(), "universes");
		if (!cJSON_IsArray(universes)) {
			return note("SteamVR's file is not a chaperone_info document; left untouched");
		}
		cJSON *source = nullptr, *target = nullptr, *entry = nullptr;
		cJSON_ArrayForEach(entry, universes)
		{
			const char *uid = universe_id(entry);
			if (uid != nullptr && strcmp(uid, from_universe) == 0) {
				source = entry;
			} else if (uid != nullptr && strcmp(uid, to_universe) == 0) {
				target = entry;
			}
		}
		if (source == nullptr || !has_drawn_room(source) || is_driver_rectangle(source) ||
		    is_default_square(source, default_size)) {
			return note("no room drawn in this session's universe");
		}
		cJSON *copy = cJSON_Duplicate(source, true);
		if (copy == nullptr) {
			return note("internal: copying the room");
		}
		cJSON_ReplaceItemInObjectCaseSensitive(copy, "universeID", cJSON_CreateString(to_universe));
		for (const char *origin : {"standing", "seated", "setup_standing2"}) {
			cJSON *o = cJSON_GetObjectItemCaseSensitive(copy, origin);
			cJSON *t = cJSON_GetObjectItemCaseSensitive(o, "translation");
			xrt_vec3 v;
			if (!read_vec3(t, &v)) {
				continue;
			}
			const float moved[3] = {v.x - shift.x, v.y - shift.y, v.z - shift.z};
			for (int i = 0; i < 3; i++) {
				cJSON_SetNumberValue(cJSON_GetArrayItem(t, i), moved[i]);
			}
		}
		const bool ok = target != nullptr ? cJSON_ReplaceItemViaPointer(universes, target, copy)
		                                  : cJSON_AddItemToArray(universes, copy);
		if (!ok) {
			cJSON_Delete(copy);
			return note("internal: inserting the room");
		}
		char *printed = cJSON_Print(doc.get());
		if (printed == nullptr) {
			return note("internal: printing SteamVR's file");
		}
		const std::string bytes(printed);
		cJSON_free(printed);
		if (write_file_atomic(path, bytes, &text, false)) {
			if (out_note != nullptr) {
				*out_note = "room moved";
			}
			return true;
		}
	}
	return note("could not write SteamVR's file (SteamVR kept changing it)");
}

bool
ovrd_room_prepare_chaperone(const std::string &path,
                            float session_size,
                            bool *out_has_room,
                            std::string *out_error,
                            float room_size,
                            const char *room_universe,
                            const char *session_universe)
{
	auto fail = [&](const char *why) {
		if (out_error != nullptr) {
			*out_error = why;
		}
		return false;
	};
	if (out_has_room != nullptr) {
		*out_has_room = false;
	}

#ifdef __linux__
	// A driver-owned file lives in a folder of its own (SteamVR's config folder/<driver>): create it once.
	const size_t slash = path.rfind('/');
	if (slash != std::string::npos && slash > 0) {
		const std::string dir = path.substr(0, slash);
		struct stat st;
		if (stat(dir.c_str(), &st) != 0 && mkdir(dir.c_str(), 0755) != 0) {
			return fail("could not create the chaperone file's folder");
		}
	}
#endif
	bool exists = false;
	std::string original = read_file(path, 4 * 1024 * 1024, &exists);
	Json document(nullptr, cJSON_Delete);
	if (!exists) {
		document.reset(cJSON_Parse("{\"jsonid\":\"chaperone_info\",\"version\":5,\"universes\":[]}"));
	} else if (original.empty()) {
		return fail("chaperone file unreadable or too large");
	} else {
		document.reset(cJSON_Parse(original.c_str()));
	}
	cJSON *id = cJSON_GetObjectItemCaseSensitive(document.get(), "jsonid");
	cJSON *universes = cJSON_GetObjectItemCaseSensitive(document.get(), "universes");
	if (!cJSON_IsString(id) || strcmp(id->valuestring, "chaperone_info") != 0 || !cJSON_IsArray(universes)) {
		return fail("chaperone file is not a chaperone_info document; left untouched");
	}

	cJSON *session = nullptr;
	cJSON *room = nullptr;
	cJSON *entry = nullptr;
	cJSON_ArrayForEach(entry, universes)
	{
		const char *uid = universe_id(entry);
		if (uid == nullptr) {
			continue;
		}
		if (strcmp(uid, session_universe) == 0) {
			session = entry;
		} else if (strcmp(uid, room_universe) == 0) {
			room = entry;
		}
	}

	bool changed = false;

	Json origin = plain_universe(session_universe, session_size);
	if (!origin) {
		return fail("internal: session universe JSON");
	}
	if (session == nullptr || !same_universe(session, origin.get())) {
		bool ok = session != nullptr ? cJSON_ReplaceItemViaPointer(universes, session, origin.get())
		                             : cJSON_AddItemToArray(universes, origin.get());
		if (!ok) {
			return fail("internal: replacing universe 3");
		}
		origin.release();
		changed = true;
	}

	const bool drawn = room != nullptr && has_drawn_room(room) && !is_driver_rectangle(room);
	if (out_has_room != nullptr) {
		*out_has_room = drawn;
	}
	if (room_size > 0.0f && !drawn) {
		// The driver's rectangle, unless the same one is already there.
		Json rect = plain_universe(room_universe, room_size);
		if (!rect || cJSON_AddTrueToObject(rect.get(), "monado_rectangle") == nullptr) {
			return fail("internal: room rectangle JSON");
		}
		if (room == nullptr || !same_universe(room, rect.get())) {
			bool ok = room != nullptr ? cJSON_ReplaceItemViaPointer(universes, room, rect.get())
			                          : cJSON_AddItemToArray(universes, rect.get());
			if (!ok) {
				return fail("internal: writing the room rectangle");
			}
			rect.release();
			changed = true;
		}
	} else if (room == nullptr) {
		Json seed = plain_universe(room_universe, 0.0f);
		if (!seed || !cJSON_AddItemToArray(universes, seed.get())) {
			return fail("internal: seeding universe 2");
		}
		seed.release();
		changed = true;
	}

	if (!changed) {
		return true;
	}

	std::unique_ptr<char, decltype(&cJSON_free)> bytes(cJSON_Print(document.get()), cJSON_free);
	if (!bytes) {
		return fail("internal: printing chaperone JSON");
	}
	if (exists) {
		// Keep the file as it was before our first change, once.
		std::string backup = path + ".monado-backup";
		bool backup_exists = false;
		(void)read_file(backup, 4 * 1024 * 1024, &backup_exists);
		if (!backup_exists && !write_file_atomic(backup, original, nullptr, false)) {
			return fail("could not write the chaperone backup");
		}
	}
	if (!write_file_atomic(path, bytes.get(), &original, !exists)) {
		return fail("could not write the chaperone file (or SteamVR changed it meanwhile)");
	}
	return true;
}

bool
ovrd_room_load(const std::string &path, ovrd_room_anchor *out)
{
	bool exists = false;
	std::string bytes = read_file(path, 64 * 1024, &exists);
	if (!exists || bytes.empty()) {
		return false;
	}
	Json doc(cJSON_Parse(bytes.c_str()), cJSON_Delete);
	cJSON *id = cJSON_GetObjectItemCaseSensitive(doc.get(), "jsonid");
	cJSON *version = cJSON_GetObjectItemCaseSensitive(doc.get(), "version");
	cJSON *anchor = cJSON_GetObjectItemCaseSensitive(doc.get(), "anchor");
	if (!cJSON_IsString(id) || strcmp(id->valuestring, "monado_room") != 0 || !cJSON_IsNumber(version) ||
	    version->valueint != ROOM_FILE_VERSION || !cJSON_IsObject(anchor)) {
		return false;
	}
	ovrd_room_anchor room = {};
	cJSON *yaw = cJSON_GetObjectItemCaseSensitive(anchor, "yaw");
	if (!read_vec3(cJSON_GetObjectItemCaseSensitive(anchor, "position"), &room.anchor.position) ||
	    !cJSON_IsNumber(yaw)) {
		return false;
	}
	room.anchor.yaw = (float)yaw->valuedouble;
	cJSON *axis = cJSON_GetObjectItemCaseSensitive(anchor, "axis");
	room.axis = OVRD_ROOM_AXIS_FORWARD;
	if (cJSON_IsNumber(axis)) {
		if (axis->valueint < OVRD_ROOM_AXIS_FORWARD || axis->valueint > OVRD_ROOM_AXIS_RIGHT) {
			return false;
		}
		room.axis = (ovrd_room_heading_axis)axis->valueint;
	}
	if (!finite3(room.anchor.position) || !std::isfinite(room.anchor.yaw) ||
	    fabsf(room.anchor.position.x) > 1000.0f || fabsf(room.anchor.position.z) > 1000.0f) {
		return false;
	}
	room.anchor.position.y = 0.0f;
	cJSON *created = cJSON_GetObjectItemCaseSensitive(doc.get(), "created");
	cJSON *updated = cJSON_GetObjectItemCaseSensitive(doc.get(), "updated");
	cJSON *captures = cJSON_GetObjectItemCaseSensitive(doc.get(), "captures");
	room.created_unix_s = cJSON_IsNumber(created) ? (int64_t)created->valuedouble : 0;
	room.updated_unix_s = cJSON_IsNumber(updated) ? (int64_t)updated->valuedouble : 0;
	room.captures = cJSON_IsNumber(captures) && captures->valuedouble >= 0 ? (uint32_t)captures->valuedouble : 0;
	*out = room;
	return true;
}

bool
ovrd_room_save(const std::string &path, const ovrd_room_anchor &room)
{
	if (!finite3(room.anchor.position) || !std::isfinite(room.anchor.yaw)) {
		return false;
	}
	char text[1024];
	snprintf(text, sizeof(text),
	         "{\n"
	         "\t\"jsonid\": \"monado_room\",\n"
	         "\t\"version\": %d,\n"
	         "\t\"universe\": 2,\n"
	         "\t\"comment\": \"Where the floor capture lies in the room frame (SteamVR universe 2). Delete this "
	         "file to set up the room again.\",\n"
	         "\t\"anchor\": {\"position\": [%.6f, %.6f, %.6f], \"yaw\": %.7f, \"axis\": %d},\n"
	         "\t\"created\": %lld,\n"
	         "\t\"updated\": %lld,\n"
	         "\t\"captures\": %u\n"
	         "}\n",
	         ROOM_FILE_VERSION, room.anchor.position.x, 0.0, room.anchor.position.z, room.anchor.yaw,
	         (int)room.axis, (long long)room.created_unix_s, (long long)room.updated_unix_s, room.captures);
	return write_file_atomic(path, text, nullptr, false);
}


/*
 *
 * Checking a universe (diagnostics and tests).
 *
 */

namespace {

struct P2
{
	double x, z;
};

double
cross2(const P2 &o, const P2 &a, const P2 &b)
{
	return (a.x - o.x) * (b.z - o.z) - (a.z - o.z) * (b.x - o.x);
}

//! Proper or touching intersection of segments ab and cd.
bool
segments_cross(const P2 &a, const P2 &b, const P2 &c, const P2 &d)
{
	const double d1 = cross2(c, d, a), d2 = cross2(c, d, b), d3 = cross2(a, b, c), d4 = cross2(a, b, d);
	return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && d1 != 0 && d2 != 0 && d3 != 0 && d4 != 0;
}

bool
inside_polygon(const std::vector<P2> &poly, const P2 &p)
{
	bool in = false;
	for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
		if (((poly[i].z > p.z) != (poly[j].z > p.z)) &&
		    (p.x < (poly[j].x - poly[i].x) * (p.z - poly[i].z) / (poly[j].z - poly[i].z) + poly[i].x)) {
			in = !in;
		}
	}
	return in;
}

bool
read_origin(cJSON *entry, const char *name, ovrd_room_yaw_pose *out)
{
	cJSON *o = cJSON_GetObjectItemCaseSensitive(entry, name);
	cJSON *yaw = cJSON_GetObjectItemCaseSensitive(o, "yaw");
	xrt_vec3 t;
	if (!cJSON_IsObject(o) || !read_vec3(cJSON_GetObjectItemCaseSensitive(o, "translation"), &t) ||
	    !cJSON_IsNumber(yaw)) {
		return false;
	}
	*out = ovrd_room_yaw_pose{t, (float)yaw->valuedouble};
	return finite3(t) && std::isfinite(out->yaw);
}

void
check_entry(cJSON *entry, ovrd_room_universe_info *out)
{
	constexpr double TOL = 0.01; // 1 cm
	auto issue = [&](const std::string &s) { out->issues.push_back(s); };
	char buf[160];

	out->found = true;
	cJSON *time_item = cJSON_GetObjectItemCaseSensitive(entry, "time");
	if (cJSON_IsString(time_item)) {
		out->time = time_item->valuestring;
	}
	out->has_standing = read_origin(entry, "standing", &out->standing);
	if (!out->has_standing) {
		issue("no valid standing origin");
	}
	ovrd_room_yaw_pose seated;
	if (!read_origin(entry, "seated", &seated)) {
		issue("no valid seated origin");
	}
	ovrd_room_yaw_pose setup;
	out->has_setup_standing = read_origin(entry, "setup_standing2", &setup);

	cJSON *area = cJSON_GetObjectItemCaseSensitive(entry, "play_area");
	if (area != nullptr) {
		if (!cJSON_IsArray(area) || cJSON_GetArraySize(area) != 2 || !cJSON_IsNumber(cJSON_GetArrayItem(area, 0)) ||
		    !cJSON_IsNumber(cJSON_GetArrayItem(area, 1))) {
			issue("play_area is not [width, depth]");
		} else {
			out->play_width = (float)cJSON_GetArrayItem(area, 0)->valuedouble;
			out->play_depth = (float)cJSON_GetArrayItem(area, 1)->valuedouble;
			if (!(out->play_width > 0) || !(out->play_depth > 0) || !std::isfinite(out->play_width) ||
			    !std::isfinite(out->play_depth)) {
				issue("play_area width/depth not positive");
			}
		}
	}

	cJSON *bounds = cJSON_GetObjectItemCaseSensitive(entry, "collision_bounds");
	if (bounds == nullptr) {
		return;
	}
	if (!cJSON_IsArray(bounds)) {
		issue("collision_bounds is not an array");
		return;
	}
	std::vector<P2> start, end; // each wall's bottom edge on the floor
	int index = 0;
	cJSON *quad = nullptr;
	cJSON_ArrayForEach(quad, bounds)
	{
		xrt_vec3 v[4];
		bool ok = cJSON_IsArray(quad) && cJSON_GetArraySize(quad) == 4;
		for (int k = 0; ok && k < 4; k++) {
			ok = read_vec3(cJSON_GetArrayItem(quad, k), &v[k]) && finite3(v[k]);
		}
		if (!ok) {
			snprintf(buf, sizeof(buf), "wall %d: not 4 finite points", index);
			issue(buf);
			index++;
			continue;
		}
		// SteamVR's order: bottom, top above it, top above the other end, bottom.
		const bool vertical = std::fabs(v[0].x - v[1].x) < TOL && std::fabs(v[0].z - v[1].z) < TOL &&
		                      std::fabs(v[3].x - v[2].x) < TOL && std::fabs(v[3].z - v[2].z) < TOL;
		if (!vertical) {
			snprintf(buf, sizeof(buf), "wall %d: not a vertical quad (bottom, top, top, bottom)", index);
			issue(buf);
		}
		if (std::fabs(v[0].y) > TOL || std::fabs(v[3].y) > TOL) {
			snprintf(buf, sizeof(buf), "wall %d: bottom not on the floor (y %.3f, %.3f)", index, v[0].y, v[3].y);
			issue(buf);
		}
		if (!(v[1].y > v[0].y + 0.1f) || !(v[2].y > v[3].y + 0.1f)) {
			snprintf(buf, sizeof(buf), "wall %d: top not above the bottom", index);
			issue(buf);
		}
		const P2 a{v[0].x, v[0].z}, b{v[3].x, v[3].z};
		const double len = std::hypot(b.x - a.x, b.z - a.z);
		if (len < TOL) {
			snprintf(buf, sizeof(buf), "wall %d: zero length", index);
			issue(buf);
		}
		out->perimeter_m += (float)len;
		start.push_back(a);
		end.push_back(b);
		index++;
	}
	out->walls = start.size();
	if (start.size() < 3) {
		if (!start.empty()) {
			issue("fewer than 3 walls");
		}
		return;
	}
	bool closed = true;
	for (size_t i = 0; i < start.size(); i++) {
		const P2 &e = end[i], &s = start[(i + 1) % start.size()];
		if (std::hypot(e.x - s.x, e.z - s.z) > TOL) {
			snprintf(buf, sizeof(buf), "wall %zu doesn't end where wall %zu starts (%.3f m apart)", i,
			         (i + 1) % start.size(), std::hypot(e.x - s.x, e.z - s.z));
			issue(buf);
			closed = false;
		}
	}
	for (size_t i = 0; i < start.size(); i++) {
		for (size_t j = i + 1; j < start.size(); j++) {
			if (std::hypot(start[i].x - start[j].x, start[i].z - start[j].z) < TOL) {
				snprintf(buf, sizeof(buf), "corners %zu and %zu are the same point", i, j);
				issue(buf);
			}
		}
	}
	const size_t n = start.size();
	for (size_t i = 0; i < n; i++) {
		for (size_t j = i + 2; j < n; j++) {
			if (i == 0 && j == n - 1) {
				continue; // neighbours through the closing corner
			}
			if (segments_cross(start[i], end[i], start[j], end[j])) {
				snprintf(buf, sizeof(buf), "walls %zu and %zu cross", i, j);
				issue(buf);
			}
		}
	}
	if (!closed) {
		return;
	}
	double twice_area = 0;
	for (size_t i = 0; i < n; i++) {
		twice_area += start[i].x * start[(i + 1) % n].z - start[(i + 1) % n].x * start[i].z;
	}
	// Seen from above (+Y down onto the x-z plane, -Z forward): positive shoelace in (x, z) is clockwise.
	out->winding = twice_area > 0 ? -1 : twice_area < 0 ? 1 : 0;
	out->area_m2 = (float)std::fabs(twice_area / 2);
	if (out->play_width > 0 && out->play_depth > 0) {
		const double hw = out->play_width / 2 - TOL, hd = out->play_depth / 2 - TOL;
		const P2 corners[4] = {{-hw, -hd}, {hw, -hd}, {hw, hd}, {-hw, hd}};
		for (const P2 &c : corners) {
			if (!inside_polygon(start, c)) {
				issue("the play area reaches outside the boundary");
				break;
			}
		}
	}
}

} // namespace

bool
ovrd_room_check_universe(const std::string &universe_json, ovrd_room_universe_info *out)
{
	*out = ovrd_room_universe_info{};
	Json entry(cJSON_Parse(universe_json.c_str()), cJSON_Delete);
	if (!entry || !cJSON_IsObject(entry.get())) {
		return false;
	}
	check_entry(entry.get(), out);
	return true;
}

bool
ovrd_room_read_universe(const std::string &path, const char *id, ovrd_room_universe_info *out)
{
	*out = ovrd_room_universe_info{};
	bool exists = false;
	std::string bytes = read_file(path, 4 * 1024 * 1024, &exists);
	Json document(cJSON_Parse(bytes.c_str()), cJSON_Delete);
	cJSON *universes = cJSON_GetObjectItemCaseSensitive(document.get(), "universes");
	if (!exists || !cJSON_IsArray(universes)) {
		return false;
	}
	cJSON *entry = nullptr;
	cJSON_ArrayForEach(entry, universes)
	{
		const char *uid = universe_id(entry);
		if (uid != nullptr && strcmp(uid, id) == 0) {
			check_entry(entry, out);
			return true;
		}
	}
	return true;
}
