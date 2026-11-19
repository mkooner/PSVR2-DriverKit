// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Room (playspace) calibration for the SteamVR driver, see ovrd_room.hpp.
 *
 * The chaperone file handling follows lbgos/rift-s-linux (ovrd_chaperone_origin.cpp, BSL-1.0): seed a plain entry
 * for the universe Room Setup draws in, write atomically and never over a concurrent SteamVR update.
 *
 * @ingroup st_ovrd
 */

#include "ovrd_room.hpp"

#include "math/m_api.h"
#include "util/u_json.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <memory>

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
ovrd_room_prepare_chaperone(
    const std::string &path, float session_size, bool *out_has_room, std::string *out_error, float room_size)
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
		if (strcmp(uid, "3") == 0) {
			session = entry;
		} else if (strcmp(uid, "2") == 0) {
			room = entry;
		}
	}

	bool changed = false;

	Json origin = plain_universe("3", session_size);
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
		Json rect = plain_universe("2", room_size);
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
		Json seed = plain_universe("2", 0.0f);
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
