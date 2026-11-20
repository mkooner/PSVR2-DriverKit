// Offline check of the V (projection layer pose) rows: feeds xrEndFrame a two-view projection layer through the layer.
#include "xr_frame_log.c"
#include <assert.h>
#include <stdlib.h>
#include <unistd.h>
static XRAPI_ATTR XrResult XRAPI_CALL fake_end(XrSession s, const XrFrameEndInfo *ei) { (void)s; (void)ei; return XR_SUCCESS; }
int main(void)
{
	char path[] = "/tmp/pose_row_test-XXXXXX";
	int fd = mkstemp(path);
	assert(fd >= 0);
	log_file = fdopen(fd, "w");
	next_end = fake_end;
	XrCompositionLayerProjectionView v[2] = {{XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}, {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}};
	v[0].pose = (XrPosef){{0, 0.1305262f, 0, 0.9914449f}, {-0.032f, 1.6f, 0.0f}};
	v[1].pose = (XrPosef){{0, 0.1305262f, 0, 0.9914449f}, {0.032f, 1.6f, 0.0f}};
	v[0].subImage.swapchain = (XrSwapchain)(uintptr_t)0x42;
	v[0].subImage.imageRect = (XrRect2Di){{0, 0}, {3040, 3100}};
	XrCompositionLayerProjection proj = {XR_TYPE_COMPOSITION_LAYER_PROJECTION};
	proj.space = (XrSpace)(uintptr_t)0x7;
	proj.viewCount = 2;
	proj.views = v;
	XrCompositionLayerQuad quad = {XR_TYPE_COMPOSITION_LAYER_QUAD};
	const XrCompositionLayerBaseHeader *layers[2] = {(const XrCompositionLayerBaseHeader *)&proj, (const XrCompositionLayerBaseHeader *)&quad};
	XrFrameEndInfo ei = {XR_TYPE_FRAME_END_INFO};
	ei.displayTime = 123456789;
	ei.layerCount = 2;
	ei.layers = layers;
	layer_end(XR_NULL_HANDLE, &ei);
	fclose(log_file);
	log_file = NULL;
	FILE *f = fopen(path, "r");
	char line[1024];
	int v_rows = 0, e_rows = 0;
	while (fgets(line, sizeof(line), f)) {
		if (line[0] == 'V') {
			v_rows++;
			printf("%s", line);
			long long t, disp; unsigned li; unsigned long long sp, sc; int x, y, w, h; float p[14];
			int n = sscanf(line, "V,%lld,%lld,%u,%llu,%llu,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
			               &t, &disp, &li, &sp, &sc, &x, &y, &w, &h, &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6],
			               &p[7], &p[8], &p[9], &p[10], &p[11], &p[12], &p[13]);
			assert(n == 23 && disp == 123456789 && li == 0 && sp == 7 && sc == 0x42 && w == 3040 && h == 3100);
			assert(p[0] == -0.032f && p[7] == 0.032f && p[1] == 1.6f && p[13] > 0.99f);
		}
		e_rows += line[0] == 'E';
	}
	fclose(f);
	assert(v_rows == 1 && e_rows == 1); // the quad layer gets no V row
	printf("PASS: one V row for the projection layer (none for the quad), all fields read back\n");
	return 0;
}
