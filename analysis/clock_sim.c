// Simulates the PS Sense host<->controller clocks with the driver's own estimator (pssense_clock.h) on synthetic
// Bluetooth reports built from measured numbers, and measures the error of IMU sample host times:
//   old: IMU timestamps converted with the clock fitted on the device timestamp (input @49, 1ms granular)
//   new: IMU timestamps converted with a clock fitted on the IMU timestamp itself (input @29)
// for several offsets between the two counters (their relation is unknown on hardware, hence the sweep).
// Measured inputs: ~450 reports/s (PSVR2-Documentation 400-530 Hz), LED round trips 9-15ms with a ~10ms median
// (our logs), max one-way delays 10-20ms with occasional stalls (our logs), clock skew 7-14ppm (our logs).
// The one-way estimator's inherent bias is the minimum transport latency, which is reported separately: it is the
// same for both and is what the two-way echo estimate addresses.
#include "pssense_clock.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static double
urand(void)
{
	return (rand() + 0.5) / ((double)RAND_MAX + 1.0);
}

static int
cmp(const void *a, const void *b)
{
	double x = *(const double *)a, y = *(const double *)b;
	return x < y ? -1 : x > y;
}

int
main(void)
{
	const double rate = 450.0, skew = 10e-6, min_lat_ns = 3.0e6, jitter_mean_ns = 2.0e6;
	const double gaps_ms[] = {0.0, 0.5, 2.5, 1000.0};
	for (size_t g = 0; g < sizeof(gaps_ms) / sizeof(gaps_ms[0]); g++) {
		srand(1234);
		struct pssense_clock dev = {0}, imu = {0};
		const int n = (int)(rate * 120); // 2 minutes
		double *err_old = malloc(sizeof(double) * n), *err_new = malloc(sizeof(double) * n);
		int m = 0;
		for (int i = 0; i < n; i++) {
			double t_true = 1e9 + i * (1e9 / rate);                 // host time of the report's generation
			double imu_t = t_true * (1.0 + skew) + gaps_ms[g] * 1e6; // IMU counter
			double dev_t = floor((t_true * (1.0 + skew)) / 1e6) * 1e6; // device counter, 1ms granularity
			double lat = min_lat_ns + -log(urand()) * jitter_mean_ns;
			if (urand() < 0.002) {
				lat += urand() * 20e6; // occasional stalls, up to the measured ~20ms
			}
			timepoint_ns recv = (timepoint_ns)(t_true + lat);
			pssense_clock_push(&dev, recv, (timepoint_ns)dev_t);
			pssense_clock_push(&imu, recv, (timepoint_ns)imu_t);
			if (i > rate * 20 && pssense_clock_ready(&dev) && pssense_clock_ready(&imu)) { // after 20s
				timepoint_ns h_old, h_new;
				pssense_clock_to_local(&dev, (timepoint_ns)imu_t, &h_old);
				pssense_clock_to_local(&imu, (timepoint_ns)imu_t, &h_new);
				err_old[m] = fabs((double)h_old - t_true);
				err_new[m] = fabs((double)h_new - t_true - min_lat_ns); // minus the shared one-way bias
				m++;
			}
		}
		qsort(err_old, m, sizeof(double), cmp);
		qsort(err_new, m, sizeof(double), cmp);
		printf("counter gap %7.1f ms: IMU sample time error, old (device-timestamp clock) median %9.3f ms p95 %9.3f ms"
		       " | new (IMU-timestamp clock, beyond the shared %.1f ms one-way bias) median %.3f ms p95 %.3f ms\n",
		       gaps_ms[g], err_old[m / 2] / 1e6, err_old[m * 95 / 100] / 1e6, min_lat_ns / 1e6, err_new[m / 2] / 1e6,
		       err_new[m * 95 / 100] / 1e6);
		free(err_old);
		free(err_new);
	}
	return 0;
}
