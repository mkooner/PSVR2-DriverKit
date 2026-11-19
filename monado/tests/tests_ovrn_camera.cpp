// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Tests for the SteamVR plugin's passthrough cameras (ovrn_camera.hpp, ovrn_camera_fit.hpp): the BC4 pair
 *         decode (layout, crop, brightening, grey chroma), the lens model fit, and the published frame path.
 *
 * With OVRN_CAMERA_TEST_RECORDING=<camera-*.pv2c> (PSVR2_RECORD_CAMERA) it also decodes the 100th recorded mode 0x10
 * transfer and writes it as OVRN_CAMERA_TEST_PGM (default /tmp/ovrn_camera_test.pgm), to look at.
 */

#include "ovrn_camera.hpp"
#include "ovrn_camera_fit.hpp"

#include "catch_amalgamated.hpp"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <thread>

using Catch::Approx;

namespace {

constexpr uint32_t W = 1016, H = 1016, STRIDE = 1024;
constexpr size_t IMAGE_BYTES = (size_t)STRIDE * H / 2;

//! A BC4 block: endpoints a > b (8-level), every pixel index 0 (= a) except the ones in @p ones (index 1 = b).
void
bc4_block(uint8_t *out, uint8_t a, uint8_t b, std::initializer_list<int> ones = {})
{
	out[0] = a;
	out[1] = b;
	uint64_t bits = 0;
	for (int p : ones) {
		bits |= (uint64_t)1 << (3 * p);
	}
	for (int i = 0; i < 6; i++) {
		out[2 + i] = (uint8_t)(bits >> (8 * i));
	}
}

//! Two BC4 images: left all @p left, right all @p right; left block (3, 2) has pixel (1, 2) at @p marker.
std::vector<uint8_t>
pair(uint8_t left, uint8_t right, uint8_t marker)
{
	std::vector<uint8_t> v(2 * IMAGE_BYTES);
	for (int cam = 0; cam < 2; cam++) {
		for (uint32_t by = 0; by < H / 4; by++) {
			for (uint32_t bx = 0; bx < STRIDE / 4; bx++) {
				uint8_t *blk = v.data() + cam * IMAGE_BYTES + ((size_t)by * (STRIDE / 4) + bx) * 8;
				if (cam == 0 && bx == 3 && by == 2) {
					bc4_block(blk, left, marker, {2 * 4 + 1});
				} else {
					bc4_block(blk, cam == 0 ? left : right, 0);
				}
			}
		}
	}
	return v;
}

struct FakeBackend : ovrn::Backend
{
	ovrn::CameraImageFn fn;
	int starts = 0, stops = 0;

	bool start(std::string *) override { return true; }
	void stop() override {}
	const char *name() const override { return "fake"; }
	ovrn::DeviceInfo headInfo() override { return {}; }
	ovrn::DisplayInfo display() override { return {}; }
	bool eyePoses(ovrn::Pose *) override { return false; }
	bool headPose(int64_t, ovrn::TrackedPose *) override { return false; }
	bool distortion(int, float, float, ovrn::DistortionUv *) override { return false; }
	bool hasController(ovrn::Hand) override { return false; }
	ovrn::DeviceInfo controllerInfo(ovrn::Hand) override { return {}; }
	bool controllerPose(ovrn::Hand, int64_t, ovrn::TrackedPose *) override { return false; }
	std::vector<ovrn::InputDesc> controllerInputDescs(ovrn::Hand, std::string *) override { return {}; }
	size_t controllerInputs(ovrn::Hand, float *, size_t) override { return 0; }
	void haptic(ovrn::Hand, float, float, float) override {}
	ovrn::WorldTransform worldFromDriver() override { return {}; }
	uint64_t universe() override { return 2; }
	std::string chaperoneJson() override { return {}; }
	void runFrame(int64_t) override {}
	bool cameraStart(ovrn::CameraImageFn f) override
	{
		fn = std::move(f);
		starts++;
		return true;
	}
	void cameraStop() override
	{
		fn = nullptr;
		stops++;
	}
};

ovrn::CameraInfo
info()
{
	ovrn::CameraInfo i;
	i.width = W;
	i.height = H;
	for (int c = 0; c < 2; c++) {
		i.fx[c] = i.fy[c] = 300;
		i.cx[c] = i.cy[c] = 508;
	}
	return i;
}

struct Sink : vr::ICameraVideoSinkCallback
{
	int calls = 0;
	void OnCameraVideoSinkCallback() override { calls++; }
};

} // namespace

TEST_CASE("ovrn camera: the BC4 pair is decoded side by side, cropped to 1016, chroma grey")
{
	setenv("OVRN_CAMERA_GAMMA", "1", 1);
	FakeBackend backend;
	ovrn::Camera cam(&backend, info());
	std::vector<uint8_t> nv12(2 * W * H * 3 / 2);
	std::vector<uint8_t> src = pair(200, 40, 90);
	cam.decode(src.data(), nv12.data());
	CHECK(nv12[0] == 200);
	CHECK(nv12[W - 1] == 200);         // the left image's last column (1015 of the 1024 stride)
	CHECK(nv12[W] == 40);              // the right image starts at 1016
	CHECK(nv12[2 * W - 1] == 40);
	CHECK(nv12[(size_t)(H - 1) * 2 * W] == 200); // last row
	// The marker: block (3, 2), pixel (1, 2) -> image (13, 10).
	CHECK(nv12[(size_t)10 * 2 * W + 13] == 90);
	CHECK(nv12[(size_t)10 * 2 * W + 12] == 200);
	CHECK(nv12[(size_t)9 * 2 * W + 13] == 200);
	// Chroma: grey.
	CHECK(nv12[(size_t)2 * W * H] == 128);
	CHECK(nv12.back() == 128);
}

TEST_CASE("ovrn camera: brightening (OVRN_CAMERA_GAMMA, default 0.5)")
{
	unsetenv("OVRN_CAMERA_GAMMA");
	FakeBackend backend;
	ovrn::Camera cam(&backend, info());
	std::vector<uint8_t> nv12(2 * W * H * 3 / 2);
	std::vector<uint8_t> src = pair(64, 0, 255);
	cam.decode(src.data(), nv12.data());
	CHECK(nv12[0] == 128); // 255 * sqrt(64 / 255)
	CHECK(nv12[W] == 0);
	CHECK(nv12[(size_t)10 * 2 * W + 13] == 255);
}

TEST_CASE("ovrn camera: the lens model fit recovers a fisheye model's coefficients")
{
	struct t_camera_calibration cc = {};
	cc.image_size_pixels.w = W;
	cc.image_size_pixels.h = H;
	cc.intrinsics[0][0] = cc.intrinsics[1][1] = 300;
	cc.intrinsics[0][2] = cc.intrinsics[1][2] = 508;
	cc.intrinsics[2][2] = 1;
	cc.distortion_model = T_DISTORTION_FISHEYE_KB4;
	cc.kb4 = {0.08, -0.02, 0.004, -0.0005};
	double k[8];
	const float rms = ovrn::fit_ftheta(cc, k);
	CHECK(rms < 0.05f);
	CHECK(k[0] == Approx(0.08).margin(2e-3));
	CHECK(k[1] == Approx(-0.02).margin(2e-3));
	CHECK(k[4] == 0.0);
}

TEST_CASE("ovrn camera: frames reach SteamVR through the published interface")
{
	unsetenv("OVRN_CAMERA_GAMMA");
	FakeBackend backend;
	ovrn::Camera cam(&backend, info());
	vr::IVRCameraComponent *c = &cam;
	uint32_t w = 0, h = 0;
	REQUIRE(c->GetCameraFrameDimensions(vr::CVS_FORMAT_NV12, &w, &h));
	CHECK(w == 2 * W);
	CHECK(h == H);
	CHECK(c->GetCameraVideoStreamFormat() == vr::CVS_FORMAT_NV12);
	CHECK(c->GetVideoStreamFrame() == nullptr); // nothing before the stream starts

	// SteamVR's own buffers.
	int count = 0;
	uint32_t bytes = 0;
	REQUIRE(c->GetCameraFrameBufferingRequirements(&count, &bytes));
	std::vector<std::vector<uint8_t>> buffers(count, std::vector<uint8_t>(bytes));
	std::vector<void *> ptrs;
	for (auto &b : buffers) {
		ptrs.push_back(b.data());
	}
	REQUIRE(c->SetCameraFrameBuffering(count, ptrs.data(), bytes));
	Sink sink;
	REQUIRE(c->SetCameraVideoSinkCallback(&sink));

	REQUIRE(c->StartVideoStream());
	CHECK(backend.starts == 1);
	bool paused = true;
	float elapsed = 0;
	CHECK(c->IsVideoStreamActive(&paused, &elapsed));
	CHECK_FALSE(paused);
	REQUIRE(backend.fn);
	std::vector<uint8_t> src = pair(200, 40, 90);
	backend.fn(src.data(), src.size(), ovrn::now_ns());
	for (int i = 0; i < 200 && cam.frames() < 1; i++) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	REQUIRE(cam.frames() == 1);
	CHECK(sink.calls == 1);
	const vr::CameraVideoStreamFrame_t *f = c->GetVideoStreamFrame();
	REQUIRE(f != nullptr);
	CHECK(f->m_nWidth == 2 * W);
	CHECK(f->m_nImageDataSize == bytes);
	CHECK(f->m_nFrameSequence == 0);
	const uint8_t *img = reinterpret_cast<const uint8_t *>(f->m_pImageData);
	CHECK(img == buffers[0].data()); // in SteamVR's buffer
	CHECK(img[0] == 226);  // 200 brightened: 255 * sqrt(200 / 255)
	CHECK(img[W] == 101);  // 40 brightened
	CHECK(img[(size_t)2 * W * H] == 128);

	c->StopVideoStream();
	CHECK(backend.stops == 1);
	CHECK_FALSE(c->IsVideoStreamActive(&paused, &elapsed));
}

TEST_CASE("ovrn camera: projection and intrinsics are consistent")
{
	FakeBackend backend;
	ovrn::Camera cam(&backend, info());
	vr::HmdMatrix44_t p;
	REQUIRE(cam.GetCameraProjection(0, vr::VRTrackedCameraFrameType_Undistorted, 0.1f, 100.0f, &p));
	// Centred camera: symmetric frustum, focal 300 / 3.25 over a 1016 px half-width of 508 -> 2 * f / w.
	CHECK(p.m[0][2] == Approx(0.0f).margin(1e-5));
	CHECK(p.m[0][0] == Approx(2.0 * (300.0 / 3.25) / 1016.0).epsilon(1e-4));
	vr::HmdVector2_t focal, centre;
	vr::EVRDistortionFunctionType type;
	double k[vr::k_unMaxDistortionFunctionParameters];
	REQUIRE(cam.GetCameraIntrinsics(1, vr::VRTrackedCameraFrameType_Distorted, &focal, &centre, &type, k));
	CHECK(focal.v[0] == Approx(300.0f));
	CHECK(type == vr::VRDistortionFunctionType_Extended_FTheta);
	// The centre maps to itself through the distortion.
	float u = 0, v = 0;
	REQUIRE(cam.GetCameraDistortion(0, 0.5f, 0.5f, &u, &v));
	CHECK(u == Approx(0.5f).margin(1e-4));
	CHECK(v == Approx(0.5f).margin(1e-4));
}

TEST_CASE("ovrn camera: a recorded frame (OVRN_CAMERA_TEST_RECORDING)")
{
	const char *path = getenv("OVRN_CAMERA_TEST_RECORDING");
	if (path == nullptr) {
		SUCCEED("no recording given");
		return;
	}
	std::ifstream f(path, std::ios::binary);
	REQUIRE(f.good());
	char magic[4];
	uint32_t version = 0;
	f.read(magic, 4);
	f.read(reinterpret_cast<char *>(&version), 4);
	REQUIRE(std::string(magic, 4) == "PV2C");
	std::vector<uint8_t> transfer;
	int found = 0;
	while (f.good() && found < 100) {
		uint64_t host = 0;
		uint32_t length = 0, dropped = 0;
		f.read(reinterpret_cast<char *>(&host), 8);
		f.read(reinterpret_cast<char *>(&length), 4);
		f.read(reinterpret_cast<char *>(&dropped), 4);
		if (!f.good()) {
			break;
		}
		transfer.resize(length);
		f.read(reinterpret_cast<char *>(transfer.data()), length);
		if (length == 256 + 2 * IMAGE_BYTES) {
			found++;
		}
	}
	REQUIRE(found == 100);
	FakeBackend backend;
	ovrn::Camera cam(&backend, info());
	std::vector<uint8_t> nv12(2 * W * H * 3 / 2);
	cam.decode(transfer.data() + 256, nv12.data());
	double sum = 0;
	for (size_t i = 0; i < (size_t)2 * W * H; i++) {
		sum += nv12[i];
	}
	const double mean = sum / (2.0 * W * H);
	const char *out = getenv("OVRN_CAMERA_TEST_PGM");
	std::ofstream pgm(out != nullptr ? out : "/tmp/ovrn_camera_test.pgm", std::ios::binary);
	pgm << "P5\n" << 2 * W << " " << H << "\n255\n";
	pgm.write(reinterpret_cast<const char *>(nv12.data()), (std::streamsize)2 * W * H);
	printf("recorded frame decoded: mean brightness %.1f / 255 (after gamma)\n", mean);
	CHECK(mean > 20.0);
}

TEST_CASE("ovrn camera: streams while SteamVR's frame queue has a reader (Room View never calls StartVideoStream)")
{
	FakeBackend backend;
	ovrn::Camera cam(&backend, info());
	const int64_t s = 1000000000;
	cam.readerCheck(false, 1 * s);
	CHECK(backend.starts == 0);
	cam.readerCheck(true, 2 * s);
	CHECK(backend.starts == 1);
	cam.readerCheck(true, 3 * s);
	cam.readerCheck(false, 4 * s); // 1 s after the last reader: keeps streaming
	CHECK(backend.stops == 0);
	cam.readerCheck(true, 4 * s + s / 2); // back again: no restart
	CHECK(backend.starts == 1);
	cam.readerCheck(false, 7 * s); // 2.5 s gone: stops
	CHECK(backend.stops == 1);

	// StartVideoStream keeps it on while the reader leaves, and the reader keeps it on after StopVideoStream.
	vr::IVRCameraComponent *c = &cam;
	REQUIRE(c->StartVideoStream());
	CHECK(backend.starts == 2);
	cam.readerCheck(true, 8 * s);
	c->StopVideoStream();
	CHECK(backend.stops == 1);
	cam.readerCheck(false, 11 * s);
	CHECK(backend.stops == 2);
}
