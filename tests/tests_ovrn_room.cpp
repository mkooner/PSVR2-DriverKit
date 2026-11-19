// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Tests for the SteamVR driver's room calibration (ovrd_room).
 */

#include "ovrn_room.hpp"

#include "math/m_api.h"
#include "util/u_json.h"

#include "catch_amalgamated.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <unistd.h>

using Catch::Approx;

namespace {

xrt_vec3
random_point(std::mt19937 &rng)
{
	std::uniform_real_distribution<float> d(-3.0f, 3.0f);
	return xrt_vec3{d(rng), d(rng), d(rng)};
}

ovrd_room_transform
random_transform(std::mt19937 &rng)
{
	std::uniform_real_distribution<float> yaw(-3.14159f, 3.14159f);
	return ovrd_room_transform{ovrd_room_quat_from_yaw(yaw(rng)), random_point(rng)};
}

void
check_vec(const xrt_vec3 &a, const xrt_vec3 &b, float margin = 1e-4f)
{
	CHECK(a.x == Approx(b.x).margin(margin));
	CHECK(a.y == Approx(b.y).margin(margin));
	CHECK(a.z == Approx(b.z).margin(margin));
}

float
wrap(float a)
{
	while (a > 3.14159265f) {
		a -= 6.2831853f;
	}
	while (a < -3.14159265f) {
		a += 6.2831853f;
	}
	return a;
}

std::string
temp_dir()
{
	char tmpl[] = "/tmp/ovrd_room_test_XXXXXX";
	char *dir = mkdtemp(tmpl);
	REQUIRE(dir != nullptr);
	return dir;
}

std::string
slurp(const std::string &path)
{
	std::ifstream f(path);
	std::stringstream s;
	s << f.rdbuf();
	return s.str();
}

void
spit(const std::string &path, const std::string &text)
{
	std::ofstream f(path);
	f << text;
}

cJSON *
find_universe(cJSON *doc, const char *id)
{
	cJSON *entry = nullptr;
	cJSON_ArrayForEach(entry, cJSON_GetObjectItem(doc, "universes"))
	{
		cJSON *u = cJSON_GetObjectItem(entry, "universeID");
		if (cJSON_IsString(u) && std::string(u->valuestring) == id) {
			return entry;
		}
	}
	return nullptr;
}

} // namespace


TEST_CASE("ovrd_room heading")
{
	for (float yaw : {0.0f, 0.5f, -1.2f, 2.9f, -3.0f}) {
		xrt_quat q = ovrd_room_quat_from_yaw(yaw);
		CHECK(wrap(ovrd_room_heading(q) - yaw) == Approx(0).margin(1e-5));
	}
	// Positive yaw turns left: facing -X.
	xrt_vec3 f;
	xrt_vec3 fwd = {0, 0, -1};
	xrt_quat left = ovrd_room_quat_from_yaw(1.5707963f);
	math_quat_rotate_vec3(&left, &fwd, &f);
	check_vec(f, xrt_vec3{-1, 0, 0});

	// Heading ignores pitch and roll up to ~60 degrees.
	xrt_quat yaw = ovrd_room_quat_from_yaw(0.7f);
	xrt_quat pitch = {sinf(0.4f), 0, 0, cosf(0.4f)}; // 46 degrees about X
	xrt_quat both;
	math_quat_rotate(&yaw, &pitch, &both);
	CHECK(ovrd_room_heading(both) == Approx(0.7f).margin(1e-5));

	// Pointing straight up or down: no heading.
	xrt_quat up = {sinf(0.785398f), 0, 0, cosf(0.785398f)}; // 90 degrees about X
	CHECK(std::isnan(ovrd_room_heading(up)));
}

TEST_CASE("ovrd_room heading of a controller resting on its ring")
{
	// A recorded rest: world up was (0, 0.27, -0.96) in the grip frame, so the forward axis points up.
	xrt_vec3 up_in_grip = {0.0f, 0.27f, -0.96f};
	math_vec3_normalize(&up_in_grip);
	// Orientation taking up_in_grip to world +Y, then a yaw.
	xrt_vec3 world_up = {0, 1, 0};
	xrt_quat tilt;
	math_quat_from_vec_a_to_vec_b(&up_in_grip, &world_up, &tilt);
	CHECK(std::isnan(ovrd_room_heading(tilt))); // forward can't give a heading here
	ovrd_room_heading_axis axis = ovrd_room_most_level_axis(tilt);
	CHECK(axis != OVRD_ROOM_AXIS_FORWARD);
	for (float yaw : {0.0f, 1.0f, -2.5f}) {
		xrt_quat yq = ovrd_room_quat_from_yaw(yaw);
		xrt_quat q;
		math_quat_rotate(&yq, &tilt, &q);
		float base = ovrd_room_heading_of_axis(tilt, axis);
		// Turning the resting controller by yaw turns the chosen axis' heading by the same amount.
		CHECK(wrap(ovrd_room_heading_of_axis(q, axis) - base - yaw) == Approx(0).margin(1e-4));
	}
}

TEST_CASE("ovrd_room transform composition and inversion")
{
	std::mt19937 rng(7);
	for (int i = 0; i < 200; i++) {
		ovrd_room_transform a = random_transform(rng);
		ovrd_room_transform b = random_transform(rng);
		xrt_vec3 p = random_point(rng);

		// (a * b)(p) == a(b(p))
		check_vec(ovrd_room_apply(ovrd_room_compose(a, b), p), ovrd_room_apply(a, ovrd_room_apply(b, p)));
		// inverse(a)(a(p)) == p, and a * inverse(a) == identity
		check_vec(ovrd_room_apply(ovrd_room_invert(a), ovrd_room_apply(a, p)), p);
		ovrd_room_transform id = ovrd_room_compose(a, ovrd_room_invert(a));
		check_vec(ovrd_room_apply(id, p), p);
		// Rotations stay about Y: heights are preserved apart from the shift.
		CHECK(ovrd_room_apply(a, p).y == Approx(p.y + a.translation.y).margin(1e-4));
	}
}

TEST_CASE("ovrd_room first capture defines the room frame")
{
	// Controller on the floor 1.37 m below the raw origin (raw floor guess was 1.6 m below the head).
	ovrd_room_yaw_pose capture = {{0.4f, -1.37f + 1.6f, -0.8f}, 0.3f};
	ovrd_room_yaw_pose anchor;
	ovrd_room_transform T = ovrd_room_transform_from_capture(capture, nullptr, &anchor);

	// The floor goes to y = 0, nothing else moves or turns.
	check_vec(ovrd_room_apply(T, capture.position), xrt_vec3{0.4f, 0.0f, -0.8f});
	check_vec(ovrd_room_apply(T, xrt_vec3{1, 2, 3}), xrt_vec3{1, 2 - 0.23f, 3});
	CHECK(anchor.position.y == 0.0f);
	CHECK(anchor.yaw == Approx(0.3f));
}

TEST_CASE("ovrd_room later capture recovers the room in a new session")
{
	std::mt19937 rng(11);
	std::uniform_real_distribution<float> yaw(-3.14159f, 3.14159f);
	std::uniform_real_distribution<float> shift(-2.0f, 2.0f);

	// Session 1: the room frame and where the controller lay.
	ovrd_room_yaw_pose capture1 = {{0.5f, 0.31f, -1.2f}, 0.9f};
	ovrd_room_yaw_pose anchor;
	ovrd_room_transform T1 = ovrd_room_transform_from_capture(capture1, nullptr, &anchor);

	for (int i = 0; i < 100; i++) {
		// Session 2 starts with the head somewhere else: raw2 = S * raw1 for an unknown yaw + shift S.
		ovrd_room_transform S = {ovrd_room_quat_from_yaw(yaw(rng)), {shift(rng), shift(rng), shift(rng)}};

		// The controller is put back in the same place, so its capture in raw2 is S applied to capture1.
		ovrd_room_yaw_pose capture2;
		capture2.position = ovrd_room_apply(S, capture1.position);
		capture2.yaw = wrap(capture1.yaw + ovrd_room_heading(S.rotation));

		ovrd_room_transform T2 = ovrd_room_transform_from_capture(capture2, &anchor, nullptr);

		// Every physical point must land at the same room coordinates as in session 1: T2 * S == T1.
		for (int k = 0; k < 5; k++) {
			xrt_vec3 p1 = random_point(rng);
			check_vec(ovrd_room_apply(T2, ovrd_room_apply(S, p1)), ovrd_room_apply(T1, p1), 2e-4f);
		}
	}
}

TEST_CASE("ovrd_room capture error sensitivity")
{
	// How far a boundary corner 2 m away moves for a 1 degree / 1 cm placement error.
	ovrd_room_yaw_pose capture = {{0, 0, 0}, 0};
	ovrd_room_yaw_pose anchor;
	ovrd_room_transform_from_capture(capture, nullptr, &anchor);

	ovrd_room_yaw_pose off = {{0.01f, 0, 0}, 0.0174533f};
	ovrd_room_transform T = ovrd_room_transform_from_capture(off, &anchor, nullptr);
	xrt_vec3 corner = ovrd_room_apply(T, xrt_vec3{2.0f, 0, 0});
	float moved = sqrtf((corner.x - 2.0f) * (corner.x - 2.0f) + corner.z * corner.z);
	CHECK(moved < 0.05f); // 3.5 cm from the turn + 1 cm shift
}

TEST_CASE("ovrd_room lowest point")
{
	xrt_vec3 outline[] = {{0, 0, 0}, {0, -0.05f, 0}, {0.02f, 0.01f, 0}};
	xrt_pose grip = {XRT_QUAT_IDENTITY, {0, 1.0f, 0}};
	CHECK(ovrd_room_lowest_point(grip, outline, 3, 0.006f) == Approx(1.0f - 0.05f - 0.006f));
	// Upside down, the other end is lowest.
	grip.orientation = xrt_quat{1, 0, 0, 0};
	CHECK(ovrd_room_lowest_point(grip, outline, 3, 0.0f) == Approx(1.0f - 0.01f));
}

TEST_CASE("ovrd_room chaperone file: the driver's rectangle in the room universe")
{
	std::string dir = temp_dir();
	std::string path = dir + "/chaperone_info.vrchap";
	bool has_room = true;
	std::string err;

	// An undrawn room (seeded without a play area) gets the rectangle; it doesn't count as a drawn room.
	REQUIRE(ovrd_room_prepare_chaperone(path, 2.0f, &has_room, &err));
	REQUIRE(ovrd_room_prepare_chaperone(path, 2.0f, &has_room, &err, 3.0f));
	CHECK_FALSE(has_room);
	cJSON *doc = cJSON_Parse(slurp(path).c_str());
	REQUIRE(doc != nullptr);
	cJSON *u2 = find_universe(doc, "2");
	REQUIRE(u2 != nullptr);
	CHECK(cJSON_GetArraySize(cJSON_GetObjectItem(u2, "collision_bounds")) == 4);
	CHECK(cJSON_GetArrayItem(cJSON_GetObjectItem(u2, "play_area"), 0)->valuedouble == Catch::Approx(3.0));
	cJSON_Delete(doc);

	// Unchanged on a second run; a new size replaces the driver's own rectangle.
	std::string before = slurp(path);
	REQUIRE(ovrd_room_prepare_chaperone(path, 2.0f, &has_room, &err, 3.0f));
	CHECK(slurp(path) == before);
	REQUIRE(ovrd_room_prepare_chaperone(path, 2.0f, &has_room, &err, 2.5f));
	CHECK_FALSE(has_room);
	doc = cJSON_Parse(slurp(path).c_str());
	u2 = find_universe(doc, "2");
	CHECK(cJSON_GetArrayItem(cJSON_GetObjectItem(u2, "play_area"), 0)->valuedouble == Catch::Approx(2.5));

	// A room drawn in Room Setup (SteamVR rewrites the entry without our mark) is never replaced.
	cJSON_DeleteItemFromObject(u2, "monado_rectangle");
	cJSON_ReplaceItemInObject(u2, "play_area", cJSON_Parse("[1.7,2.2]"));
	char *text = cJSON_Print(doc);
	FILE *f = fopen(path.c_str(), "w");
	fputs(text, f);
	fclose(f);
	cJSON_free(text);
	cJSON_Delete(doc);
	before = slurp(path);
	REQUIRE(ovrd_room_prepare_chaperone(path, 2.0f, &has_room, &err, 3.0f));
	CHECK(has_room);
	CHECK(slurp(path) == before);
}

TEST_CASE("ovrd_room chaperone file: fresh, existing room, idempotent, foreign files")
{
	std::string dir = temp_dir();
	std::string path = dir + "/chaperone_info.vrchap";

	SECTION("no file: universes 3 and 2 are created")
	{
		bool has_room = true;
		std::string err;
		REQUIRE(ovrd_room_prepare_chaperone(path, 2.0f, &has_room, &err));
		CHECK_FALSE(has_room);
		cJSON *doc = cJSON_Parse(slurp(path).c_str());
		REQUIRE(doc != nullptr);
		cJSON *u3 = find_universe(doc, "3");
		cJSON *u2 = find_universe(doc, "2");
		REQUIRE(u3 != nullptr);
		REQUIRE(u2 != nullptr);
		CHECK(cJSON_GetArraySize(cJSON_GetObjectItem(u3, "collision_bounds")) == 4);
		CHECK(cJSON_GetObjectItem(u2, "collision_bounds") == nullptr);
		CHECK(cJSON_GetObjectItem(u2, "standing") != nullptr);
		cJSON_Delete(doc);

		// A second run changes nothing.
		std::string before = slurp(path);
		REQUIRE(ovrd_room_prepare_chaperone(path, 2.0f, &has_room, &err));
		CHECK(slurp(path) == before);
	}

	SECTION("a drawn room in universe 2 and another driver's universe are kept, universe 3 is reset")
	{
		std::string original =
		    "{\"jsonid\":\"chaperone_info\",\"version\":5,\"universes\":["
		    "{\"universeID\":\"4\",\"play_area\":[1.8,1.9],\"standing\":{\"translation\":[-33.8,6.49,25.58],"
		    "\"yaw\":-1.72},\"seated\":{\"translation\":[0,0,0],\"yaw\":0},\"collision_bounds\":[]},"
		    "{\"universeID\":\"2\",\"play_area\":[2.5,3.0],\"standing\":{\"translation\":[0.1,0.02,-0.3],"
		    "\"yaw\":0.4},\"seated\":{\"translation\":[0,0,0],\"yaw\":0},\"collision_bounds\":[[[1,0,1],[1,2.43,1],"
		    "[1,2.43,-1],[1,0,-1]]]},"
		    "{\"universeID\":\"3\",\"standing\":{\"translation\":[0,0.3,0],\"yaw\":1.0},\"seated\":{\"translation\":"
		    "[0,0,0],\"yaw\":0}}]}";
		spit(path, original);
		bool has_room = false;
		std::string err;
		REQUIRE(ovrd_room_prepare_chaperone(path, 2.0f, &has_room, &err));
		CHECK(has_room);
		CHECK(slurp(path + ".monado-backup") == original);

		cJSON *doc = cJSON_Parse(slurp(path).c_str());
		REQUIRE(doc != nullptr);
		cJSON *orig = cJSON_Parse(original.c_str());
		CHECK(cJSON_Compare(find_universe(doc, "4"), find_universe(orig, "4"), true));
		CHECK(cJSON_Compare(find_universe(doc, "2"), find_universe(orig, "2"), true));
		cJSON *u3 = find_universe(doc, "3");
		REQUIRE(u3 != nullptr);
		cJSON *standing = cJSON_GetObjectItem(u3, "standing");
		CHECK(cJSON_GetObjectItem(standing, "yaw")->valuedouble == 0.0);
		CHECK(cJSON_GetArrayItem(cJSON_GetObjectItem(standing, "translation"), 1)->valuedouble == 0.0);
		cJSON_Delete(doc);
		cJSON_Delete(orig);
	}

	SECTION("a file that isn't a chaperone document is left alone")
	{
		spit(path, "{\"something\":\"else\"}");
		std::string err;
		CHECK_FALSE(ovrd_room_prepare_chaperone(path, 2.0f, nullptr, &err));
		CHECK(slurp(path) == "{\"something\":\"else\"}");
	}
}

TEST_CASE("ovrd_room room file round trip and validation")
{
	std::string dir = temp_dir();
	std::string path = dir + "/room.json";

	ovrd_room_anchor room = {{{0.25f, 0.0f, -1.5f}, -2.1f}, OVRD_ROOM_AXIS_UP, 1000, 2000, 3};
	REQUIRE(ovrd_room_save(path, room));
	ovrd_room_anchor loaded = {};
	REQUIRE(ovrd_room_load(path, &loaded));
	check_vec(loaded.anchor.position, room.anchor.position, 1e-5f);
	CHECK(loaded.anchor.yaw == Approx(room.anchor.yaw));
	CHECK(loaded.captures == 3);
	CHECK(loaded.axis == OVRD_ROOM_AXIS_UP);

	spit(path, "{\"jsonid\":\"monado_room\",\"version\":99,\"anchor\":{\"position\":[0,0,0],\"yaw\":0}}");
	CHECK_FALSE(ovrd_room_load(path, &loaded));
	spit(path, "{\"jsonid\":\"monado_room\",\"version\":1,\"anchor\":{\"position\":[0,0],\"yaw\":0}}");
	CHECK_FALSE(ovrd_room_load(path, &loaded));
	spit(path, "not json");
	CHECK_FALSE(ovrd_room_load(path, &loaded));
	CHECK_FALSE(ovrd_room_load(dir + "/missing.json", &loaded));
}

namespace {

//! A universe entry with the given floor polygon (x, z corners in order) as SteamVR stores walls: bottom, top, top,
//! bottom, 2.43 m high; each wall from corner i to corner i+1, closing back to the first.
std::string
universe_with_walls(const std::vector<std::pair<float, float>> &corners, float play_w, float play_d, bool close = true)
{
	std::ostringstream s;
	s << "{\"universeID\":\"5022\",\"play_area\":[" << play_w << "," << play_d << "],\"collision_bounds\":[";
	const size_t n = corners.size(), walls = close ? n : n - 1;
	for (size_t i = 0; i < walls; i++) {
		const auto &a = corners[i], &b = corners[(i + 1) % n];
		s << (i ? "," : "") << "[[" << a.first << ",0," << a.second << "],[" << a.first << ",2.43," << a.second
		  << "],[" << b.first << ",2.43," << b.second << "],[" << b.first << ",0," << b.second << "]]";
	}
	s << "],\"seated\":{\"translation\":[0,0,0],\"yaw\":0},\"standing\":{\"translation\":[0.5,0,-0.25],\"yaw\":"
	     "1.5708},\"time\":\"Wed Nov 18 20:00:00 2026\"}";
	return s.str();
}

bool
has_issue(const ovrd_room_universe_info &info, const char *text)
{
	for (const auto &i : info.issues) {
		if (i.find(text) != std::string::npos) {
			return true;
		}
	}
	return false;
}

} // namespace

TEST_CASE("ovrd_room boundary check: SteamVR's own 2 m rectangle is valid")
{
	// As SteamVR wrote it into chaperone_info.vrchap (2026-11-18).
	const std::string steamvr =
	    "{\"collision_bounds\":[[[-1,0,-1],[-1,2.43000007,-1],[-1,2.43000007,1],[-1,0,1]],[[-1,0,1],[-1,2.43000007,1],"
	    "[1,2.43000007,1],[1,0,1]],[[1,0,1],[1,2.43000007,1],[1,2.43000007,-1],[1,0,-1]],[[1,0,-1],[1,2.43000007,-1],"
	    "[-1,2.43000007,-1],[-1,0,-1]]],\"play_area\":[2,2],\"seated\":{\"translation\":[0,0,0],\"yaw\":0},\"standing\":"
	    "{\"translation\":[0,0,0],\"yaw\":0},\"time\":\"Wed Nov 18 15:58:17 2026\",\"universeID\":\"2\"}";
	ovrd_room_universe_info info;
	REQUIRE(ovrd_room_check_universe(steamvr, &info));
	CHECK(info.issues.empty());
	CHECK(info.walls == 4);
	CHECK(info.perimeter_m == Approx(8.0f));
	CHECK(info.area_m2 == Approx(4.0f));
	CHECK(info.winding == 1); // front-left, back-left, back-right, front-right: counter-clockwise seen from above
	CHECK(info.play_width == Approx(2.0f));
	CHECK(info.time == "Wed Nov 18 15:58:17 2026");
}

TEST_CASE("ovrd_room boundary check: a traced L-shaped room, either direction")
{
	// 2.5 x 2.5 m with a 0.5 x 1.5 m corner cut out (front right), a 0.8 m play area around the origin.
	std::vector<std::pair<float, float>> l = {{-1.5f, -1}, {-1.5f, 1.5f}, {1, 1.5f}, {1, 0.5f}, {0.5f, 0.5f}, {0.5f, -1}};
	ovrd_room_universe_info info;
	REQUIRE(ovrd_room_check_universe(universe_with_walls(l, 0.8f, 0.8f), &info));
	CHECK(info.issues.empty());
	CHECK(info.walls == 6);
	CHECK(info.area_m2 == Approx(2.5f * 2.5f - 0.5f * 1.5f));
	// The same room with a 1.2 m play area reaches into the cut-out.
	ovrd_room_universe_info big;
	REQUIRE(ovrd_room_check_universe(universe_with_walls(l, 1.2f, 1.2f), &big));
	CHECK(has_issue(big, "outside the boundary"));
	CHECK(info.has_standing);
	CHECK(info.standing.yaw == Approx(1.5708f));
	std::vector<std::pair<float, float>> r(l.rbegin(), l.rend());
	ovrd_room_universe_info back;
	REQUIRE(ovrd_room_check_universe(universe_with_walls(r, 0.8f, 0.8f), &back));
	CHECK(back.issues.empty());
	CHECK(back.winding == -info.winding);
}

TEST_CASE("ovrd_room boundary check: malformed geometry is reported")
{
	ovrd_room_universe_info info;
	const std::vector<std::pair<float, float>> square = {{-1, -1}, {-1, 1}, {1, 1}, {1, -1}};

	// Not closed.
	REQUIRE(ovrd_room_check_universe(universe_with_walls(square, 1, 1, false), &info));
	CHECK(has_issue(info, "doesn't end where"));
	CHECK(info.winding == 0);

	// Walls crossing (a bow tie).
	REQUIRE(ovrd_room_check_universe(universe_with_walls({{-1, -1}, {1, 1}, {1, -1}, {-1, 1}}, 0.2f, 0.2f), &info));
	CHECK(has_issue(info, "cross"));

	// A corner repeated (a zero-length wall).
	REQUIRE(ovrd_room_check_universe(universe_with_walls({{-1, -1}, {-1, 1}, {-1, 1}, {1, 1}, {1, -1}}, 1, 1), &info));
	CHECK(has_issue(info, "zero length"));
	CHECK(has_issue(info, "same point"));

	// Play area bigger than the room.
	REQUIRE(ovrd_room_check_universe(universe_with_walls(square, 3, 3), &info));
	CHECK(has_issue(info, "outside the boundary"));

	// A wall floating above the floor, a slanted quad, a quad of 3 points.
	REQUIRE(ovrd_room_check_universe(
	    "{\"collision_bounds\":[[[0,0.2,0],[0,2.43,0],[1,2.43,0],[1,0.2,0]],[[1,0,0],[1.5,2.43,0],[1,2.43,1],[1,0,1]],"
	    "[[1,0,1],[1,2.43,1],[0,0,1]]],\"seated\":{\"translation\":[0,0,0],\"yaw\":0},\"standing\":{\"translation\":["
	    "0,0,0],\"yaw\":0}}",
	    &info));
	CHECK(has_issue(info, "not on the floor"));
	CHECK(has_issue(info, "not a vertical quad"));
	CHECK(has_issue(info, "not 4 finite points"));

	// No standing origin.
	REQUIRE(ovrd_room_check_universe("{\"seated\":{\"translation\":[0,0,0],\"yaw\":0}}", &info));
	CHECK(has_issue(info, "standing"));
	CHECK_FALSE(ovrd_room_check_universe("[1,2]", &info));
}

TEST_CASE("ovrd_room chaperone file: a driver-owned file with its own universes")
{
	std::string dir = temp_dir();
	std::string path = dir + "/monado_psvr2/chaperone_info.vrchap"; // the folder doesn't exist yet
	bool has_room = true;
	std::string err;
	REQUIRE(ovrd_room_prepare_chaperone(path, 2.0f, &has_room, &err, 0.0f, "5022", "5023"));
	CHECK_FALSE(has_room);

	// The session universe gets the temporary square, the room universe only a plain origin (no made-up bounds).
	ovrd_room_universe_info session, room;
	REQUIRE(ovrd_room_read_universe(path, "5023", &session));
	CHECK(session.found);
	CHECK(session.walls == 4);
	CHECK(session.issues.empty());
	REQUIRE(ovrd_room_read_universe(path, "5022", &room));
	CHECK(room.found);
	CHECK(room.walls == 0);
	CHECK(room.play_width == 0.0f);
	CHECK(room.issues.empty());
	ovrd_room_universe_info none;
	REQUIRE(ovrd_room_read_universe(path, "2", &none));
	CHECK_FALSE(none.found);

	// A room drawn in Room Setup (SteamVR rewrites the room universe) is kept, and the session universe is reset.
	cJSON *doc = cJSON_Parse(slurp(path).c_str());
	REQUIRE(doc != nullptr);
	cJSON *u = find_universe(doc, "5022");
	REQUIRE(u != nullptr);
	cJSON *drawn = cJSON_Parse(universe_with_walls({{-2, -1}, {-2, 1}, {2, 1}, {2, -1}}, 3, 1.5f).c_str());
	cJSON_ReplaceItemInObject(drawn, "universeID", cJSON_CreateString("5022"));
	cJSON_ReplaceItemViaPointer(cJSON_GetObjectItem(doc, "universes"), u, drawn);
	char *text = cJSON_Print(doc);
	{
		std::ofstream f(path);
		f << text;
	}
	cJSON_free(text);
	cJSON_Delete(doc);
	REQUIRE(ovrd_room_prepare_chaperone(path, 2.0f, &has_room, &err, 0.0f, "5022", "5023"));
	CHECK(has_room);
	REQUIRE(ovrd_room_read_universe(path, "5022", &room));
	CHECK(room.walls == 4);
	CHECK(room.area_m2 == Approx(8.0f));
	CHECK(room.issues.empty());
}
