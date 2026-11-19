// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Simulated backend: a 2x(2000x2040) 90 Hz headset 1.6 m up and two controllers in front of it, all still.
 * @ingroup st_ovrn
 */

#include "ovrn_backend.hpp"

#include <cmath>

namespace ovrn {

namespace {

class SimBackend : public Backend
{
public:
	bool
	start(std::string *) override
	{
		return true;
	}
	void
	stop() override
	{}
	const char *
	name() const override
	{
		return "sim";
	}

	DeviceInfo
	headInfo() override
	{
		DeviceInfo d;
		d.manufacturer = "Monado";
		d.model = "Simulated headset";
		d.serial = "OVRN-SIM-HMD";
		return d;
	}

	DisplayInfo
	display() override
	{
		DisplayInfo d;
		d.width = 4000;
		d.height = 2040;
		d.refresh_hz = 90.0;
		d.vsync_to_photons_s = 0.0;
		for (int e = 0; e < 2; e++) {
			d.eye[e].viewport_x = e * 2000;
			d.eye[e].viewport_y = 0;
			d.eye[e].viewport_w = 2000;
			d.eye[e].viewport_h = 2040;
			d.eye[e].render_w = 2000;
			d.eye[e].render_h = 2040;
			d.eye[e].eye_in_head.position.x = e == 0 ? -0.0315 : 0.0315;
		}
		return d;
	}

	bool
	eyePoses(Pose[2]) override
	{
		return false;
	}

	//! Yaw at 0.5 rad/s with a fixed 0.3 rad pitch (so the head's own frame differs from the tracking frame),
	//! drifting sideways at 0.1 m/s.
	bool
	headPose(int64_t at_ns, TrackedPose *out) override
	{
		*out = TrackedPose();
		const double t = (double)(at_ns % 100000000000LL) * 1e-9;
		const double yaw = 0.5 * t, pitch = 0.3;
		const double cy = cos(yaw / 2), sy = sin(yaw / 2), cp = cos(pitch / 2), sp = sin(pitch / 2);
		out->pose.orientation = {cy * sp, sy * cp, -sy * sp, cy * cp}; // rot_y(yaw) * rot_x(pitch)
		out->angular_velocity = {0.0, 0.5, 0.0};
		out->linear_velocity = {0.1, 0.0, 0.0};
		out->pose.position.x = 0.1 * fmod(t, 10.0);
		out->pose.position.y = 1.6;
		out->orientation_valid = out->orientation_tracked = true;
		out->position_valid = out->position_tracked = true;
		return true;
	}

	bool
	distortion(int, float, float, DistortionUv *) override
	{
		return false;
	}

	bool
	hasController(Hand) override
	{
		return true;
	}

	DeviceInfo
	controllerInfo(Hand hand) override
	{
		DeviceInfo d;
		d.manufacturer = "Monado";
		d.model = "Simulated controller";
		d.serial = hand == Hand::Left ? "OVRN-SIM-L" : "OVRN-SIM-R";
		d.controller_type = "ovrn_sim_controller";
		return d;
	}

	std::vector<InputDesc>
	controllerInputDescs(Hand, std::string *out_haptic_path) override
	{
		*out_haptic_path = "/output/haptic";
		return {{"/input/trigger/value", InputKind::ScalarOneSided},
		        {"/input/a/click", InputKind::Boolean},
		        {"/input/stick/x", InputKind::ScalarTwoSided}};
	}

	//! A trigger half pulled, a button that is down in even seconds, a stick pushed left or right by hand.
	size_t
	controllerInputs(Hand hand, float *out, size_t max) override
	{
		if (max < 3) {
			return 0;
		}
		out[0] = 0.5f;
		out[1] = (now_ns() / 1000000000LL) % 2 == 0 ? 1.0f : 0.0f;
		out[2] = hand == Hand::Left ? -1.0f : 1.0f;
		return 3;
	}

	void
	haptic(Hand hand, float duration_s, float, float amplitude) override
	{
		haptics[hand == Hand::Left ? 0 : 1]++;
		last_haptic_duration = duration_s;
		last_haptic_amplitude = amplitude;
	}

	WorldTransform
	worldFromDriver() override
	{
		WorldTransform t;
		t.translation.y = 0.25; // a floor 25 cm below the tracking origin, to see the transform reach the poses
		return t;
	}

	uint64_t
	universe() override
	{
		return frames > 30 ? 2 : 3; // changes once, as a room capture does
	}

	std::string
	chaperoneJson() override
	{
		return std::string();
	}

	void
	runFrame(int64_t) override
	{
		frames++;
	}

	int haptics[2] = {0, 0};
	float last_haptic_duration = 0, last_haptic_amplitude = 0;
	int frames = 0;

	bool
	controllerPose(Hand hand, int64_t, TrackedPose *out) override
	{
		*out = TrackedPose();
		out->pose.position.x = hand == Hand::Left ? -0.2 : 0.2;
		out->pose.position.y = 1.2;
		out->pose.position.z = -0.4;
		out->orientation_valid = out->orientation_tracked = true;
		out->position_valid = out->position_tracked = true;
		return true;
	}
};

} // namespace

std::unique_ptr<Backend>
make_sim_backend()
{
	return std::unique_ptr<Backend>(new SimBackend());
}

} // namespace ovrn
