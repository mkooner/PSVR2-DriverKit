// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//
// Eye tracking test: a headless OpenXR client (XR_MND_headless, no window or graphics) that asks the runtime for eye
// gaze (XR_EXT_eye_gaze_interaction) and prints where you look, 10 times a second, as yaw (+ right) and pitch (+ up) in
// degrees relative to the head, plus how many samples had a valid gaze at the end.
//
// With Monado (monado-service running, PS VR2 connected):
//   XR_RUNTIME_JSON=<build>/openxr_monado-dev.json ./xreyetest [seconds]
// Look left, right, up and down: the numbers should follow. "no gaze" means the runtime gave no valid eye pose.
//
// Build: gcc -O2 -o xreyetest xreyetest.c -lopenxr_loader -lm
#include <openxr/openxr.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CK(x)                                                                                                          \
	do {                                                                                                           \
		XrResult r_ = (x);                                                                                     \
		if (XR_FAILED(r_)) {                                                                                   \
			printf("%s failed: %d\n", #x, r_);                                                             \
			return 1;                                                                                      \
		}                                                                                                      \
	} while (0)

int
main(int argc, char **argv)
{
	const double seconds = argc > 1 ? atof(argv[1]) : 30.0;

	const char *ext[] = {"XR_MND_headless", "XR_EXT_eye_gaze_interaction"};
	XrInstanceCreateInfo ici = {XR_TYPE_INSTANCE_CREATE_INFO};
	strcpy(ici.applicationInfo.applicationName, "xreyetest");
	ici.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
	ici.enabledExtensionCount = 2;
	ici.enabledExtensionNames = ext;
	XrInstance inst;
	CK(xrCreateInstance(&ici, &inst));

	XrSystemGetInfo sgi = {XR_TYPE_SYSTEM_GET_INFO};
	sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
	XrSystemId sys;
	CK(xrGetSystem(inst, &sgi, &sys));

	XrSystemEyeGazeInteractionPropertiesEXT eyeProps = {XR_TYPE_SYSTEM_EYE_GAZE_INTERACTION_PROPERTIES_EXT};
	XrSystemProperties props = {XR_TYPE_SYSTEM_PROPERTIES};
	props.next = &eyeProps;
	CK(xrGetSystemProperties(inst, sys, &props));
	printf("system: %s, eye gaze supported: %s\n", props.systemName, eyeProps.supportsEyeGazeInteraction ? "yes" : "NO");
	if (!eyeProps.supportsEyeGazeInteraction) {
		return 1;
	}

	// The gaze pose action, bound to the eye gaze interaction profile.
	XrActionSetCreateInfo asci = {XR_TYPE_ACTION_SET_CREATE_INFO};
	strcpy(asci.actionSetName, "eyetest");
	strcpy(asci.localizedActionSetName, "Eye test");
	XrActionSet set;
	CK(xrCreateActionSet(inst, &asci, &set));
	XrActionCreateInfo aci = {XR_TYPE_ACTION_CREATE_INFO};
	aci.actionType = XR_ACTION_TYPE_POSE_INPUT;
	strcpy(aci.actionName, "gaze");
	strcpy(aci.localizedActionName, "Gaze");
	XrAction gaze;
	CK(xrCreateAction(set, &aci, &gaze));
	XrPath profile, gazePath;
	CK(xrStringToPath(inst, "/interaction_profiles/ext/eye_gaze_interaction", &profile));
	CK(xrStringToPath(inst, "/user/eyes_ext/input/gaze_ext/pose", &gazePath));
	XrActionSuggestedBinding binding = {gaze, gazePath};
	XrInteractionProfileSuggestedBinding sb = {XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
	sb.interactionProfile = profile;
	sb.countSuggestedBindings = 1;
	sb.suggestedBindings = &binding;
	CK(xrSuggestInteractionProfileBindings(inst, &sb));

	XrSessionCreateInfo sci = {XR_TYPE_SESSION_CREATE_INFO};
	sci.systemId = sys;
	XrSession s;
	CK(xrCreateSession(inst, &sci, &s));
	XrSessionActionSetsAttachInfo att = {XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
	att.countActionSets = 1;
	att.actionSets = &set;
	CK(xrAttachSessionActionSets(s, &att));

	XrActionSpaceCreateInfo gsi = {XR_TYPE_ACTION_SPACE_CREATE_INFO};
	gsi.action = gaze;
	gsi.poseInActionSpace.orientation.w = 1.0f;
	XrSpace gazeSpace, viewSpace;
	CK(xrCreateActionSpace(s, &gsi, &gazeSpace));
	XrReferenceSpaceCreateInfo rsi = {XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
	rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
	rsi.poseInReferenceSpace.orientation.w = 1.0f;
	CK(xrCreateReferenceSpace(s, &rsi, &viewSpace));

	int running = 0;
	long samples = 0, valid = 0;
	XrTime start = 0, lastPrint = 0;
	for (;;) {
		XrEventDataBuffer ev = {XR_TYPE_EVENT_DATA_BUFFER};
		while (xrPollEvent(inst, &ev) == XR_SUCCESS) {
			if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
				XrEventDataSessionStateChanged *c = (XrEventDataSessionStateChanged *)&ev;
				if (c->state == XR_SESSION_STATE_READY) {
					XrSessionBeginInfo bi = {XR_TYPE_SESSION_BEGIN_INFO};
					bi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
					CK(xrBeginSession(s, &bi));
					running = 1;
					printf("session running: look around (left, right, up, down)\n");
				}
			}
			ev.type = XR_TYPE_EVENT_DATA_BUFFER;
		}
		if (!running) {
			continue;
		}
		XrFrameState fs = {XR_TYPE_FRAME_STATE};
		CK(xrWaitFrame(s, NULL, &fs));
		CK(xrBeginFrame(s, NULL));
		const XrTime t = fs.predictedDisplayTime;
		if (start == 0) {
			start = t;
		}

		XrActiveActionSet active = {set, XR_NULL_PATH};
		XrActionsSyncInfo sync = {XR_TYPE_ACTIONS_SYNC_INFO};
		sync.countActiveActionSets = 1;
		sync.activeActionSets = &active;
		xrSyncActions(s, &sync);

		XrSpaceLocation loc = {XR_TYPE_SPACE_LOCATION};
		XrResult lr = xrLocateSpace(gazeSpace, viewSpace, t, &loc);
		const int ok = lr == XR_SUCCESS && (loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT);
		samples++;
		valid += ok;

		if (t - lastPrint >= 100000000LL) { // 10 Hz
			lastPrint = t;
			if (ok) {
				// The gaze looks along the pose's -Z; express it as yaw/pitch in the view (head) space.
				const XrQuaternionf q = loc.pose.orientation;
				const float x = -2.0f * (q.x * q.z + q.w * q.y);
				const float y = -2.0f * (q.y * q.z - q.w * q.x);
				const float z = -(1.0f - 2.0f * (q.x * q.x + q.y * q.y));
				const float yaw = atan2f(x, -z) * 57.2958f;
				const float pitch = atan2f(y, sqrtf(x * x + z * z)) * 57.2958f;
				printf("%6.1f s  gaze yaw %+6.1f deg  pitch %+6.1f deg\n", (t - start) / 1e9, yaw, pitch);
			} else {
				printf("%6.1f s  no gaze\n", (t - start) / 1e9);
			}
			fflush(stdout);
		}

		XrFrameEndInfo fe = {XR_TYPE_FRAME_END_INFO};
		fe.displayTime = t;
		fe.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
		CK(xrEndFrame(s, &fe));
		if ((t - start) / 1e9 >= seconds) {
			break;
		}
	}
	printf("valid gaze in %ld of %ld samples (%.0f%%)\n", valid, samples, samples ? 100.0 * valid / samples : 0.0);
	xrDestroySession(s);
	xrDestroyInstance(inst);
	return 0;
}
