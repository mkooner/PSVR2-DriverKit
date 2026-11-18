// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenXR API layer that records an app's frame timing as the runtime gives it, per frame, without changing anything:
 * every xrWaitFrame (predicted display time and period, shouldRender), xrLocateViews (the display time asked for)
 * and xrEndFrame (the display time submitted, layer count), each with CLOCK_MONOTONIC at the call.
 *
 * Works with any runtime (SteamVR, Monado), so the same game can be compared: on a steady 90 Hz the predicted display
 * times are one refresh apart and the period is one refresh; steps of 0 or 2 refreshes are frames the runtime slotted
 * one refresh off. analysis/xr_frame_log.py reads it (and lines it up with tools/vrframetiming).
 *
 * Enable for one app run:
 *   XR_API_LAYER_PATH=<dir with XR_APILAYER_psvr2_frame_log.json> XR_ENABLE_API_LAYERS=XR_APILAYER_psvr2_frame_log \
 *   XR_FRAME_LOG=/path/out.csv <app>
 */
#include <openxr/openxr.h>
#include <openxr/openxr_loader_negotiation.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static PFN_xrGetInstanceProcAddr next_gipa;
static PFN_xrWaitFrame next_wait;
static PFN_xrBeginFrame next_begin;
static PFN_xrEndFrame next_end;
static PFN_xrLocateViews next_locate;
static FILE *log_file;
static pthread_mutex_t log_lock = PTHREAD_MUTEX_INITIALIZER;
static unsigned long long rows;

static long long
mono_ns(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (long long)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void
close_log(void)
{
	pthread_mutex_lock(&log_lock);
	if (log_file != NULL) {
		fclose(log_file);
		log_file = NULL;
	}
	pthread_mutex_unlock(&log_lock);
}

static void
row(char kind, long long t_call, long long t_ret, long long xr_a, long long xr_b, int n)
{
	pthread_mutex_lock(&log_lock);
	if (log_file != NULL) {
		fprintf(log_file, "%c,%lld,%lld,%lld,%lld,%d\n", kind, t_call, t_ret, xr_a, xr_b, n);
		if (++rows % 2000 == 0) {
			fflush(log_file);
		}
	}
	pthread_mutex_unlock(&log_lock);
}

static XRAPI_ATTR XrResult XRAPI_CALL
layer_wait(XrSession s, const XrFrameWaitInfo *wi, XrFrameState *st)
{
	long long a = mono_ns();
	XrResult r = next_wait(s, wi, st);
	long long b = mono_ns();
	if (XR_SUCCEEDED(r) && st != NULL) {
		row('W', a, b, (long long)st->predictedDisplayTime, (long long)st->predictedDisplayPeriod, (int)st->shouldRender);
	}
	return r;
}

static XRAPI_ATTR XrResult XRAPI_CALL
layer_begin(XrSession s, const XrFrameBeginInfo *bi)
{
	long long a = mono_ns();
	XrResult r = next_begin(s, bi);
	row('B', a, mono_ns(), 0, 0, (int)r);
	return r;
}

static XRAPI_ATTR XrResult XRAPI_CALL
layer_end(XrSession s, const XrFrameEndInfo *ei)
{
	long long a = mono_ns();
	XrResult r = next_end(s, ei);
	row('E', a, mono_ns(), ei != NULL ? (long long)ei->displayTime : 0, (long long)r, ei != NULL ? (int)ei->layerCount : 0);
	return r;
}

static XRAPI_ATTR XrResult XRAPI_CALL
layer_locate(XrSession s, const XrViewLocateInfo *li, XrViewState *vs, uint32_t cap, uint32_t *count, XrView *views)
{
	long long a = mono_ns();
	XrResult r = next_locate(s, li, vs, cap, count, views);
	if (cap > 0 && li != NULL) {
		row('L', a, mono_ns(), (long long)li->displayTime, vs != NULL ? (long long)vs->viewStateFlags : 0, (int)cap);
	}
	return r;
}

static XRAPI_ATTR XrResult XRAPI_CALL
layer_gipa(XrInstance inst, const char *name, PFN_xrVoidFunction *fn)
{
	if (strcmp(name, "xrWaitFrame") == 0) { *fn = (PFN_xrVoidFunction)layer_wait; return XR_SUCCESS; }
	if (strcmp(name, "xrBeginFrame") == 0) { *fn = (PFN_xrVoidFunction)layer_begin; return XR_SUCCESS; }
	if (strcmp(name, "xrEndFrame") == 0) { *fn = (PFN_xrVoidFunction)layer_end; return XR_SUCCESS; }
	if (strcmp(name, "xrLocateViews") == 0) { *fn = (PFN_xrVoidFunction)layer_locate; return XR_SUCCESS; }
	if (strcmp(name, "xrGetInstanceProcAddr") == 0) { *fn = (PFN_xrVoidFunction)layer_gipa; return XR_SUCCESS; }
	return next_gipa(inst, name, fn);
}

static XRAPI_ATTR XrResult XRAPI_CALL
layer_create_instance(const XrInstanceCreateInfo *info, const XrApiLayerCreateInfo *layer_info, XrInstance *instance)
{
	if (layer_info == NULL || layer_info->nextInfo == NULL) {
		return XR_ERROR_INITIALIZATION_FAILED;
	}
	next_gipa = layer_info->nextInfo->nextGetInstanceProcAddr;
	XrApiLayerCreateInfo next = *layer_info;
	next.nextInfo = layer_info->nextInfo->next;
	XrResult r = layer_info->nextInfo->nextCreateApiLayerInstance(info, &next, instance);
	if (XR_FAILED(r)) {
		return r;
	}
	next_gipa(*instance, "xrWaitFrame", (PFN_xrVoidFunction *)&next_wait);
	next_gipa(*instance, "xrBeginFrame", (PFN_xrVoidFunction *)&next_begin);
	next_gipa(*instance, "xrEndFrame", (PFN_xrVoidFunction *)&next_end);
	next_gipa(*instance, "xrLocateViews", (PFN_xrVoidFunction *)&next_locate);

	const char *path = getenv("XR_FRAME_LOG");
	char def[512];
	if (path == NULL || path[0] == '\0') {
		snprintf(def, sizeof def, "%s/xr-frame-log-%d.csv", getenv("HOME") ? getenv("HOME") : "/tmp", (int)getpid());
		path = def;
	}
	pthread_mutex_lock(&log_lock);
	if (log_file == NULL) {
		log_file = fopen(path, "w");
		if (log_file != NULL) {
			setvbuf(log_file, NULL, _IOFBF, 1 << 20);
			fprintf(log_file, "# XR_APILAYER_psvr2_frame_log; runtime instance created at mono %lld ns\n", mono_ns());
			fprintf(log_file, "kind,call_mono_ns,return_mono_ns,xr_a,xr_b,n\n");
			fprintf(log_file, "# W: xr_a predictedDisplayTime, xr_b predictedDisplayPeriod, n shouldRender\n");
			fprintf(log_file, "# L: xr_a displayTime asked, xr_b viewStateFlags; E: xr_a displayTime submitted, xr_b result, n layers\n");
			atexit(close_log);
		}
	}
	pthread_mutex_unlock(&log_lock);
	return r;
}

XRAPI_ATTR XrResult XRAPI_CALL
xrNegotiateLoaderApiLayerInterface(const XrNegotiateLoaderInfo *li, const char *layer_name, XrNegotiateApiLayerRequest *req)
{
	if (li == NULL || req == NULL || li->structType != XR_LOADER_INTERFACE_STRUCT_LOADER_INFO ||
	    req->structType != XR_LOADER_INTERFACE_STRUCT_API_LAYER_REQUEST) {
		return XR_ERROR_INITIALIZATION_FAILED;
	}
	req->layerInterfaceVersion = XR_CURRENT_LOADER_API_LAYER_VERSION;
	req->layerApiVersion = XR_CURRENT_API_VERSION;
	req->getInstanceProcAddr = layer_gipa;
	req->createApiLayerInstance = layer_create_instance;
	return XR_SUCCESS;
}
