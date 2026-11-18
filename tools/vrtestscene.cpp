// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//
// Diagnostic scene for SteamVR image instability: shows where it is introduced, by what moves and what doesn't.
//
//   world-locked : floor grid, a wall grid 3 m ahead with solid squares (moves only if the head pose is wrong)
//   head-locked  : crosshair and stereo alignment target 1.5 m ahead, drawn in head space (no pose involved: if these
//                  shake, the instability is added after rendering: compositor, reprojection or display)
//   frame code   : 16 head-locked squares, the frame number in binary (white = 1), lower view; a repeated or skipped
//                  frame shows as a frozen or jumping code (readable from a slow-motion video)
//   eye markers  : red square top-left in the left eye, blue in the right (eye-specific instability)
//
// Logs per frame to the CSV: frame index, CLOCK_MONOTONIC after WaitGetPoses and after Submit, and the render pose.
//
// Build: g++ -O2 -o vrtestscene vrtestscene.cpp $(pkg-config --cflags --libs sdl2 glew gl)
//        -L$STEAMVR/bin/linux64 -lopenvr_api -Wl,-rpath,$STEAMVR/bin/linux64
// Usage: vrtestscene [frames.csv]   (SteamVR running; close the window or Ctrl+C to quit)
#include <openvr/openvr.h>
#include <GL/glew.h>
#include <SDL2/SDL.h>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <time.h>
#include <vector>

static std::atomic<bool> g_stop{false};
static void on_signal(int) { g_stop = true; }
static double mono() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }

struct M4 { float m[16]; }; // column-major
static M4 ident() { M4 r = {}; r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1; return r; }
static M4 mul(const M4 &a, const M4 &b) {
	M4 r = {};
	for (int c = 0; c < 4; c++) for (int rr = 0; rr < 4; rr++) for (int k = 0; k < 4; k++) r.m[c * 4 + rr] += a.m[k * 4 + rr] * b.m[c * 4 + k];
	return r;
}
static M4 from34(const vr::HmdMatrix34_t &x) {
	M4 r = ident();
	for (int rr = 0; rr < 3; rr++) for (int c = 0; c < 4; c++) r.m[c * 4 + rr] = x.m[rr][c];
	return r;
}
static M4 from44(const vr::HmdMatrix44_t &x) {
	M4 r;
	for (int rr = 0; rr < 4; rr++) for (int c = 0; c < 4; c++) r.m[c * 4 + rr] = x.m[rr][c];
	return r;
}
static M4 inv_rigid(const M4 &a) { // inverse of rotation + translation
	M4 r = ident();
	for (int rr = 0; rr < 3; rr++) for (int c = 0; c < 3; c++) r.m[c * 4 + rr] = a.m[rr * 4 + c];
	for (int rr = 0; rr < 3; rr++) r.m[12 + rr] = -(r.m[rr] * a.m[12] + r.m[4 + rr] * a.m[13] + r.m[8 + rr] * a.m[14]);
	return r;
}

struct V { float x, y, z, r, g, b; };
static void line(std::vector<V> &v, float x0, float y0, float z0, float x1, float y1, float z1, float r, float g, float b) {
	v.push_back({x0, y0, z0, r, g, b}); v.push_back({x1, y1, z1, r, g, b});
}
static void quad(std::vector<V> &v, float cx, float cy, float z, float hw, float hh, float r, float g, float b) {
	V a{cx - hw, cy - hh, z, r, g, b}, c{cx + hw, cy + hh, z, r, g, b}, b1{cx + hw, cy - hh, z, r, g, b}, d{cx - hw, cy + hh, z, r, g, b};
	v.insert(v.end(), {a, b1, c, a, c, d});
}

static const char *VS = "#version 330 core\nlayout(location=0) in vec3 p; layout(location=1) in vec3 c; uniform mat4 mvp; out vec3 vc;"
                        "void main(){ gl_Position = mvp * vec4(p,1); vc = c; }";
static const char *FS = "#version 330 core\nin vec3 vc; out vec4 o; void main(){ o = vec4(vc,1); }";

static GLuint shader() {
	auto sh = [](GLenum t, const char *s) { GLuint h = glCreateShader(t); glShaderSource(h, 1, &s, nullptr); glCompileShader(h); return h; };
	GLuint p = glCreateProgram();
	glAttachShader(p, sh(GL_VERTEX_SHADER, VS));
	glAttachShader(p, sh(GL_FRAGMENT_SHADER, FS));
	glLinkProgram(p);
	return p;
}

int main(int argc, char **argv) {
	std::signal(SIGINT, on_signal);
	std::signal(SIGTERM, on_signal);
	vr::EVRInitError e = vr::VRInitError_None;
	vr::IVRSystem *sys = vr::VR_Init(&e, vr::VRApplication_Scene);
	if (e != vr::VRInitError_None) { fprintf(stderr, "VR_Init: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(e)); return 1; }
	if (SDL_Init(SDL_INIT_VIDEO) != 0) { fprintf(stderr, "SDL: %s\n", SDL_GetError()); return 1; }
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_Window *win = SDL_CreateWindow("vrtestscene (close to quit)", 100, 100, 640, 320, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
	SDL_GLContext ctx = SDL_GL_CreateContext(win);
	SDL_GL_SetSwapInterval(0);
	glewExperimental = GL_TRUE;
	glewInit();
	uint32_t W, H;
	sys->GetRecommendedRenderTargetSize(&W, &H);
	GLuint fbo[2], tex[2], depth[2];
	glGenFramebuffers(2, fbo); glGenTextures(2, tex); glGenRenderbuffers(2, depth);
	for (int i = 0; i < 2; i++) {
		glBindTexture(GL_TEXTURE_2D, tex[i]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, W, H, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glBindRenderbuffer(GL_RENDERBUFFER, depth[i]);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, W, H);
		glBindFramebuffer(GL_FRAMEBUFFER, fbo[i]);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex[i], 0);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth[i]);
	}
	GLuint prog = shader(), vao, vbo;
	GLint mvp_loc = glGetUniformLocation(prog, "mvp");
	glGenVertexArrays(1, &vao); glGenBuffers(1, &vbo);
	glBindVertexArray(vao); glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glEnableVertexAttribArray(0); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(V), (void *)0);
	glEnableVertexAttribArray(1); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(V), (void *)12);

	// World-locked geometry (standing space): floor grid every 0.5 m, wall grid 3 m ahead every 0.25 m.
	std::vector<V> world_lines, world_tris;
	for (int i = -10; i <= 10; i++) {
		line(world_lines, i * 0.5f, 0, -5, i * 0.5f, 0, 5, 0.3f, 0.3f, 0.3f);
		line(world_lines, -5, 0, i * 0.5f, 5, 0, i * 0.5f, 0.3f, 0.3f, 0.3f);
	}
	for (int i = -8; i <= 8; i++) {
		line(world_lines, i * 0.25f, 0, -3, i * 0.25f, 3, -3, 0.0f, 0.8f, 0.3f);
		if (i >= 0) line(world_lines, -2, i * 0.25f + 0.0f, -3, 2, i * 0.25f, -3, 0.0f, 0.8f, 0.3f);
	}
	for (int i = 0; i < 4; i++) quad(world_tris, -0.75f + i * 0.5f, 1.6f, -2.99f, 0.08f, 0.08f, 1.0f, 0.9f, 0.2f);
	// Head-locked geometry (head space, -z forward): crosshair and stereo target 1.5 m ahead.
	std::vector<V> head_lines;
	const float z = -1.5f;
	line(head_lines, -0.08f, 0, z, 0.08f, 0, z, 1, 1, 1);
	line(head_lines, 0, -0.08f, z, 0, 0.08f, z, 1, 1, 1);
	for (int k = 1; k <= 3; k++) {
		float s = 0.03f * k;
		line(head_lines, -s, -s, z, s, -s, z, 1, 0.3f, 1); line(head_lines, s, -s, z, s, s, z, 1, 0.3f, 1);
		line(head_lines, s, s, z, -s, s, z, 1, 0.3f, 1); line(head_lines, -s, s, z, -s, -s, z, 1, 0.3f, 1);
	}

	FILE *log = argc > 1 ? fopen(argv[1], "w") : nullptr;
	if (log) fprintf(log, "frame,wait_done_s,submit_done_s,hmd_valid,qx_m00,m01,m02,m03,m10,m11,m12,m13,m20,m21,m22,m23\n");
	vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
	uint32_t frame = 0;
	while (!g_stop) {
		SDL_Event ev;
		while (SDL_PollEvent(&ev)) if (ev.type == SDL_QUIT) g_stop = true;
		vr::VRCompositor()->WaitGetPoses(poses, vr::k_unMaxTrackedDeviceCount, nullptr, 0);
		double t_wait = mono();
		M4 hmd = poses[0].bPoseIsValid ? from34(poses[0].mDeviceToAbsoluteTracking) : ident();
		for (int eye = 0; eye < 2; eye++) {
			M4 proj = from44(sys->GetProjectionMatrix((vr::EVREye)eye, 0.05f, 50.0f));
			M4 eye2head = from34(sys->GetEyeToHeadTransform((vr::EVREye)eye));
			M4 view_world = inv_rigid(mul(hmd, eye2head));
			M4 view_head = inv_rigid(eye2head);
			glBindFramebuffer(GL_FRAMEBUFFER, fbo[eye]);
			glViewport(0, 0, W, H);
			glClearColor(0.05f, 0.05f, 0.08f, 1); glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glEnable(GL_DEPTH_TEST);
			glUseProgram(prog);
			glBindVertexArray(vao);
			auto draw = [&](const std::vector<V> &v, GLenum mode, const M4 &m) {
				M4 mvp = mul(proj, m);
				glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, mvp.m);
				glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(V), v.data(), GL_STREAM_DRAW);
				glDrawArrays(mode, 0, (GLsizei)v.size());
			};
			draw(world_lines, GL_LINES, view_world);
			draw(world_tris, GL_TRIANGLES, view_world);
			glDisable(GL_DEPTH_TEST);
			draw(head_lines, GL_LINES, view_head);
			std::vector<V> code;
			for (int b = 0; b < 16; b++) {
				bool on = (frame >> (15 - b)) & 1;
				quad(code, -0.30f + b * 0.04f, -0.35f, z, 0.015f, 0.015f, on ? 1.f : 0.15f, on ? 1.f : 0.15f, on ? 1.f : 0.15f);
			}
			quad(code, -0.35f, 0.30f, z, 0.03f, 0.03f, eye == 0 ? 1.f : 0.f, 0.f, eye == 0 ? 0.f : 1.f);
			draw(code, GL_TRIANGLES, view_head);
		}
		glFlush();
		for (int eye = 0; eye < 2; eye++) {
			vr::Texture_t t = {(void *)(uintptr_t)tex[eye], vr::TextureType_OpenGL, vr::ColorSpace_Gamma};
			vr::VRCompositor()->Submit((vr::EVREye)eye, &t);
		}
		double t_sub = mono();
		if (log) {
			fprintf(log, "%u,%.6f,%.6f,%d", frame, t_wait, t_sub, (int)poses[0].bPoseIsValid);
			for (int a = 0; a < 3; a++) for (int c = 0; c < 4; c++) fprintf(log, ",%.6f", poses[0].mDeviceToAbsoluteTracking.m[a][c]);
			fprintf(log, "\n");
		}
		// Desktop window: the left eye, for reference.
		glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo[0]); glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
		glBlitFramebuffer(0, 0, W, H, 0, 0, 320, 320 * H / W, GL_COLOR_BUFFER_BIT, GL_LINEAR);
		SDL_GL_SwapWindow(win);
		frame++;
	}
	if (log) fclose(log);
	vr::VR_Shutdown();
	SDL_GL_DeleteContext(ctx);
	SDL_DestroyWindow(win);
	SDL_Quit();
	return 0;
}
