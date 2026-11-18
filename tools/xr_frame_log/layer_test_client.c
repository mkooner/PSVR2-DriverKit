// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
// Minimal headless OpenXR client (XR_MND_headless) running 300 frames, to check XR_APILAYER_psvr2_frame_log offline
// against monado-service with a simulated headset and the null compositor.
#include <openxr/openxr.h>
#include <stdio.h>
#include <string.h>
#define CK(x) do { XrResult r_ = (x); if (XR_FAILED(r_)) { printf("%s failed: %d\n", #x, r_); return 1; } } while (0)
int main(void) {
	const char *ext[] = {"XR_MND_headless"};
	XrInstanceCreateInfo ici = {XR_TYPE_INSTANCE_CREATE_INFO};
	strcpy(ici.applicationInfo.applicationName, "layer_test_client");
	ici.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
	ici.enabledExtensionCount = 1; ici.enabledExtensionNames = ext;
	XrInstance inst; CK(xrCreateInstance(&ici, &inst));
	XrSystemGetInfo sgi = {XR_TYPE_SYSTEM_GET_INFO}; sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
	XrSystemId sys; CK(xrGetSystem(inst, &sgi, &sys));
	XrSessionCreateInfo sci = {XR_TYPE_SESSION_CREATE_INFO}; sci.systemId = sys;
	XrSession s; CK(xrCreateSession(inst, &sci, &s));
	int running = 0, frames = 0;
	while (frames < 300) {
		XrEventDataBuffer ev = {XR_TYPE_EVENT_DATA_BUFFER};
		while (xrPollEvent(inst, &ev) == XR_SUCCESS) {
			if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
				XrEventDataSessionStateChanged *c = (XrEventDataSessionStateChanged *)&ev;
				if (c->state == XR_SESSION_STATE_READY) {
					XrSessionBeginInfo bi = {XR_TYPE_SESSION_BEGIN_INFO}; bi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
					CK(xrBeginSession(s, &bi)); running = 1;
				}
			}
			ev.type = XR_TYPE_EVENT_DATA_BUFFER;
		}
		if (!running) continue;
		XrFrameState fs = {XR_TYPE_FRAME_STATE};
		CK(xrWaitFrame(s, NULL, &fs));
		CK(xrBeginFrame(s, NULL));
		XrFrameEndInfo fe = {XR_TYPE_FRAME_END_INFO}; fe.displayTime = fs.predictedDisplayTime; fe.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
		CK(xrEndFrame(s, &fe));
		frames++;
	}
	printf("ran %d frames\n", frames);
	xrDestroySession(s); xrDestroyInstance(inst);
	return 0;
}
