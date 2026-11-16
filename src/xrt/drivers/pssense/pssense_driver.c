// Copyright 2023, Collabora, Ltd.
// Copyright 2023, Jarett Millard
// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PlayStation Sense controller prober and driver code.
 * @author Jarett Millard <jarett.millard@gmail.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_pssense
 */

#include "xrt/xrt_prober.h"

#include "hid/h_hid.h"
#include "os/os_threading.h"
#include "os/os_time.h"

#include "math/m_api.h"
#include "math/m_vec3.h"

#include "tracking/t_constellation.h"
#include "tracking/t_imu.h"

#include "constellation/t_constellation_tracker.h"
#include "constellation/t_led_sync_refinement.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_linux.h"
#include "util/u_logging.h"
#include "util/u_resampler.h"
#include "util/u_time.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"
#include "util/u_misc.h"

#include "math/m_clock_tracking.h"
#include "math/m_imu_3dof.h"
#include "math/m_relation_history.h"
#include "math/m_space.h"

#include "pssense_clock.h"
#include "pssense_interface.h"
#include "pssense_led_model.h"
#include "pssense_protocol.h"

#include <stdio.h>
#include <errno.h>


/*!
 * @addtogroup drv_pssense
 * @{
 */

#define PSSENSE_TRACE(p, ...) U_LOG_XDEV_IFL_T(&p->base, p->log_level, __VA_ARGS__)
#define PSSENSE_DEBUG(p, ...) U_LOG_XDEV_IFL_D(&p->base, p->log_level, __VA_ARGS__)
#define PSSENSE_DEBUG_HEX(p, data, data_size) U_LOG_XDEV_IFL_D_HEX(&p->base, p->log_level, data, data_size)
#define PSSENSE_WARN(p, ...) U_LOG_XDEV_IFL_W(&p->base, p->log_level, __VA_ARGS__)
#define PSSENSE_ERROR(p, ...) U_LOG_XDEV_IFL_E(&p->base, p->log_level, __VA_ARGS__)

DEBUG_GET_ONCE_LOG_OPTION(pssense_log, "PSSENSE_LOG", U_LOGGING_INFO)
DEBUG_GET_ONCE_BOOL_OPTION(pssense_pc_polling_rate, "PSSENSE_SET_PC_POLLING_RATE", true)
/*
 * Advanced LED sync (gray codes + BG brightness curve, LED sync owning the clock offset). Set to false for the basic
 * mode, which binary searches the LED visibility edges and locks to their centre on top of the driver's Bluetooth
 * clock tracking (like PSVR2Toolkit's custom-controller-sync); on hardware it found the edges but lost the lock.
 */
DEBUG_GET_ONCE_BOOL_OPTION(pssense_advanced_led_sync, "PSSENSE_ADVANCED_LED_SYNC", true)
/*
 * With advanced LED sync, find the LED offset by counting the blobs the headset sees instead of needing solved poses
 * (PSVR2Toolkit's custom-controller-sync calibration), then hold it. Set to false for the gray code + BG path.
 */
DEBUG_GET_ONCE_BOOL_OPTION(pssense_count_calibration, "PSSENSE_COUNT_CALIBRATION", true)

/*!
 * Longest time the constellation tracker's prediction may run without a fresh optical fix before its position is
 * held. Between fixes the tracker double-integrates the accelerometer, so any bias or misalignment makes the
 * controller run away quadratically ("flies off into the distance"). Orientation keeps coming from the tracker.
 */
DEBUG_GET_ONCE_NUM_OPTION(pssense_max_prediction_ms, "PSSENSE_MAX_PREDICTION_MS", 150)

/*!
 * Largest distance (cm) the tracker's fused pose may be from the latest fresh optical fix before it is considered
 * diverged and the optical pose is used instead. On hardware the fused output drifted to hundreds of metres with a
 * constant velocity while the optical fixes stayed at the hands, flagged as tracked throughout. 0 disables.
 */
DEBUG_GET_ONCE_NUM_OPTION(pssense_fusion_max_error_cm, "PSSENSE_FUSION_MAX_ERROR_CM", 20)

//! With nothing new to send, output reports still go out this often (~30Hz) as a keepalive.
/*
 * With nothing else to send. 33ms kept ~30 reports/s per controller going; with two controllers and LED sync on top
 * the Bluetooth link backed up (LED command round trips of 0.2-1.3s, input reports up to 0.7s late).
 */
#define PSSENSE_KEEPALIVE_NS (U_TIME_1MS_IN_NS * 100)

/*
 * Bluetooth output report rate cap, per controller, when no PCM haptics are playing. Each report is a full PS5 format
 * report, and sending them as fast as the loop runs (~94Hz, e.g. while an LED sync sample waits to be marked as
 * applied) overflowed bluetoothd's socket ("BT socket write error: Resource temporarily unavailable (11)"), right
 * before the controller dropped off Bluetooth. Sony's driver and PSVR2Toolkit throttle the same way, skipping sends
 * while earlier ones are still queued. Anything pending (LED sync sample, vibration, LED settings) goes out in the next
 * allowed report. 0 disables the cap.
 */
DEBUG_GET_ONCE_FLOAT_OPTION(pssense_imu_angle_deg, "PSSENSE_IMU_ANGLE_DEG", PSSENSE_IMU_ANGLE_DEG_DEFAULT)
DEBUG_GET_ONCE_FLOAT_OPTION(pssense_imu_ref_correction_deg,
                            "PSSENSE_IMU_REF_CORRECTION_DEG",
                            PSSENSE_IMU_REF_CORRECTION_DEG_DEFAULT)
DEBUG_GET_ONCE_NUM_OPTION(pssense_bt_min_send_interval_ms, "PSSENSE_BT_MIN_SEND_INTERVAL_MS", 33)
//! How long to stop sending after a failed output report, to let the Bluetooth link drain.
#define PSSENSE_SEND_FAIL_BACKOFF_NS (U_TIME_1MS_IN_NS * 100)

/*!
 * Indices where each input is in the input list.
 */
enum pssense_input_index
{
	PSSENSE_INDEX_PS_CLICK,
	PSSENSE_INDEX_SHARE_CLICK,
	PSSENSE_INDEX_OPTIONS_CLICK,
	PSSENSE_INDEX_SQUARE_CLICK,
	PSSENSE_INDEX_SQUARE_TOUCH,
	PSSENSE_INDEX_TRIANGLE_CLICK,
	PSSENSE_INDEX_TRIANGLE_TOUCH,
	PSSENSE_INDEX_CROSS_CLICK,
	PSSENSE_INDEX_CROSS_TOUCH,
	PSSENSE_INDEX_CIRCLE_CLICK,
	PSSENSE_INDEX_CIRCLE_TOUCH,
	PSSENSE_INDEX_SQUEEZE_CLICK,
	PSSENSE_INDEX_SQUEEZE_TOUCH,
	PSSENSE_INDEX_SQUEEZE_PROXIMITY,
	PSSENSE_INDEX_SQUEEZE_PROXIMITY_FLOAT,
	PSSENSE_INDEX_TRIGGER_CLICK,
	PSSENSE_INDEX_TRIGGER_TOUCH,
	PSSENSE_INDEX_TRIGGER_VALUE,
	PSSENSE_INDEX_TRIGGER_PROXIMITY,
	PSSENSE_INDEX_TRIGGER_PROXIMITY_FLOAT,
	PSSENSE_INDEX_THUMBSTICK,
	PSSENSE_INDEX_THUMBSTICK_CLICK,
	PSSENSE_INDEX_THUMBSTICK_TOUCH,
	PSSENSE_INDEX_GRIP_POSE,
	PSSENSE_INDEX_AIM_POSE,
	PSSENSE_INPUT_COUNT,
};

/*!
 * Parsed calibration data from the PlayStation Sense controller.
 */
struct pssense_parsed_calibration
{
	struct xrt_vec3 gyro_scale;
	struct xrt_vec3 accel_scale;

	struct xrt_vec3 accel_bias;
	struct xrt_vec3_i32 gyro_bias;
};

/*!
 * PlayStation Sense state parsed from a data packet.
 */
struct pssense_input_state
{
	uint64_t timestamp_ns;
	uint32_t seq_no;

	bool ps_click;
	bool share_click;
	bool options_click;
	bool square_click;
	bool square_touch;
	bool triangle_click;
	bool triangle_touch;
	bool cross_click;
	bool cross_touch;
	bool circle_click;
	bool circle_touch;
	bool squeeze_click;
	bool squeeze_touch;
	float squeeze_proximity;
	bool trigger_click;
	bool trigger_touch;
	float trigger_value;
	float trigger_proximity;
	bool thumbstick_click;
	bool thumbstick_touch;
	struct xrt_vec2 thumbstick;

	struct xrt_vec3_i32 gyro_raw;
	struct xrt_vec3_i32 accel_raw;

	bool battery_state_valid;
	bool battery_charging;
	//! Charge level from 0..1
	float battery_charge_percent;
};

/*!
 * A single PlayStation Sense Controller.
 *
 * @implements xrt_device
 * @implements xrt_frame_node
 * @implements t_timing_event_sink
 * @implements t_constellation_tracker_led_model
 * @implements t_constellation_tracker_device
 */
struct pssense_device
{
	struct xrt_device base;
	struct xrt_frame_node node;
	struct t_timing_event_sink timing_event_sink;
	struct t_constellation_tracker_led_model led_model;
	struct t_constellation_tracker_device constellation_device;

	bool usb;
	struct h_hid_device *hid;
	struct os_thread_helper controller_thread;
	//! The read thread stopped on an error (controller disconnected or turned off), protected by controller_thread.
	bool disconnected;

	struct
	{
		//! Host <-> controller clock, see pssense_clock.h, on the device timestamp (input @49); LED scheduling.
		struct pssense_clock clock;
		/*!
		 * Host <-> controller clock on the IMU timestamp (input @29), for IMU sample times. The two are separate
		 * counters (PSVR2-Documentation: @49 has 1ms granularity), and IMU times used to be converted with the
		 * device timestamp's clock, putting IMU samples off the camera frames by whatever separates them.
		 */
		struct pssense_clock imu_clock;
		//! Echo-based (two-way) offset estimate on the IMU timestamp, diagnostics only for now.
		struct
		{
			uint32_t last_echo_us;
			bool have_echo;
			time_duration_ns window_min_rtt_ns, window_offset_ns;
			timepoint_ns window_start_ns;
			time_duration_ns last_offset_ns, last_rtt_ns;
			bool have_window;
		} echo;

		timepoint_ns latest_imu_time_ns;

		uint32_t imu_ticks_last;
		uint64_t imu_ticks_total;

		timepoint_ns latest_device_time_ns;

		uint32_t device_ticks_last;
		uint64_t device_ticks_total;

		uint32_t last_sent_host_timestamp_us;
		//! When the report carrying the last LED sync sample went out, for the round trip stats.
		timepoint_ns last_led_sample_send_ns;
		struct
		{
			uint32_t count;
			time_duration_ns sum_ns, max_ns;
			timepoint_ns last_log_ns;
		} led_rtt;
	} timing;

	struct
	{
		struct m_relation_history *imu_relation_history;
		struct m_imu_3dof fusion;
		struct xrt_pose pose;

		uint32_t received_frames;
		uint32_t last_exposure_sequence_id;
		timepoint_ns last_exposure_local_timestamp_ns;
		time_duration_ns average_exposure_interval_ns;

		/*!
		 * Whether or not the sequence number is being incremented automatically, should never
		 * be set to true during BROAD/BG/STABLE.
		 */
		bool increment_sequence_num;
		//! Count calibration in use: PRESCAN positions are re-anchored to every exposure, as in basic sync.
		bool count_calibration;
		//! Set by pssense_handle_read when it got a report, so the thread can drain the queue.
		bool read_had_data;
		//! Last PRESCAN re-anchor (new sequence number) in count calibration mode.
		timepoint_ns last_reanchor_ns;
		uint8_t led_sequence_num;

		//! A debug value for testing sync issues.
		int32_t timing_fudge_100us;

		/*!
		 * Host -> controller clock offset for PRESCAN blink positions, latched and then slewed at most
		 * PSSENSE_LED_MAP_SLEW_NS per frame toward the Bluetooth clock tracking. On hardware the tracking's
		 * estimate wandered by milliseconds within seconds (the LED visibility window moved 3-5ms between
		 * count calibration attempts, and edges flipped within one), while the clocks themselves only drift by
		 * crystal skew. PSVR2Toolkit's calibration likewise measures once and keeps the mapping.
		 */
		bool led_map_valid;
		time_duration_ns led_map_offset_ns;
		//! Host time the offset was last advanced by the optical drift trim.
		timepoint_ns led_map_last_host_ns;
		//! Largest |tracking - latched| seen since the last log, and when that was.
		time_duration_ns led_map_max_dev_ns;
		timepoint_ns led_map_last_log_ns;

		//! Locked by controller_thread.
		struct pssense_led_settings led_settings;

		bool use_constellation;

		//! Last optically tracked position, held while the constellation tracker has no lock.
		bool have_last_optical_position;
		struct xrt_vec3 last_optical_position;

		//! Rate limit for the "OUT" debug log of what clients are given.
		timepoint_ns last_out_log_ns;

		//! Rate limit for the "FINAL" debug log of what clients actually get.
		timepoint_ns last_final_log_ns;

		//! Timestamp of the last sample the constellation tracker solved optically (0 until the first one).
		timepoint_ns last_optical_fix_ns;
		//! That sample's pose, in the same (tracker) space as the fused pose.
		struct xrt_pose last_optical_fix_pose;
		//! Whether we're currently overriding a diverged fused pose with the optical one, and how often.
		bool overriding_fusion;
		uint32_t fusion_overrides;
		//! Held position (tracker space) while the prediction is stale, and whether we're holding.
		bool holding_position;
		uint32_t holds_started;
		struct t_constellation_tracker *constellation_tracker;
		t_constellation_device_id_t constellation_device_id;
		struct xrt_imu_sink *constellation_imu_sink;

		struct t_led_sync_refinement led_sync_refinement;
		uint8_t period_id;
		enum pssense_led_sync_phase sync_phase;

		bool led_sync_sample_needs_marking;
		bool led_sync_sample_needs_sending;
		struct t_led_sync_sample latest_led_sync_sample;
		//! The latest sample's base-time delta, in IMU ticks.
		int32_t led_sync_delta_ticks;
		//! What the deltas didn't fit into whole IMU ticks, in thirds of a ns, carried into the next one.
		int64_t led_sync_delta_remainder_ns3;

		struct xrt_pose T_led_imu;
	} tracking;

	enum xrt_hand hand;

	enum u_logging_level log_level;

	struct os_precise_sleeper sleeper;

	struct pssense_parsed_calibration calibration;
	bool has_calibration;

	//! Input state parsed from most recent packet
	struct pssense_input_state state;
	//! Pending output state to send to device
	struct
	{
		uint8_t next_seq_no;
		uint8_t packet_counter;

		struct u_resampler *pcm_haptics_resampler;

		//! PCM haptics were written recently enough that the resampler may still hold samples.
		timepoint_ns pcm_active_until_ns;
		//! When the last output report went out, and the LED settings it carried.
		timepoint_ns last_send_ns;
		struct pssense_led_settings last_sent_led_settings;
		//! No sends before this, set after a failed send.
		timepoint_ns backoff_until_ns;
		//! Send stats, logged every few seconds: reports sent/failed, sends held by the rate cap, and why
		//! reports were sent.
		struct
		{
			uint32_t sent, failed, held, led_sample, marking, keepalive, haptics, led_changed;
			timepoint_ns last_log_ns;
		} stats;

		bool send_vibration;
		uint8_t vibration_amplitude;
		uint8_t vibration_mode;

		uint64_t vibration_end_timestamp_ns;
		//! The vibration the last report carried.
		uint8_t last_sent_vibration_amplitude;
		uint8_t last_sent_vibration_mode;

		bool send_trigger_feedback;
		enum pssense_adaptive_trigger_mode trigger_feedback_mode;
	} output;
	struct
	{
		bool button_states;
		bool timing;
		bool tracking;
	} gui;
};

static struct pssense_device *
from_device(struct xrt_device *xdev)
{
	return container_of(xdev, struct pssense_device, base);
}

static struct pssense_device *
from_node(struct xrt_frame_node *node)
{
	return container_of(node, struct pssense_device, node);
}

static struct pssense_device *
from_timing_event_sink(struct t_timing_event_sink *sink)
{
	return container_of(sink, struct pssense_device, timing_event_sink);
}

static struct pssense_device *
from_constellation_device(struct t_constellation_tracker_device *device)
{
	return container_of(device, struct pssense_device, constellation_device);
}

const uint32_t CRC_POLYNOMIAL = 0xedb88320;

/*
 *
 * Internal functions
 *
 */

static uint32_t
crc32_le(uint32_t crc, uint8_t const *p, size_t len)
{
	int i;
	crc ^= 0xffffffff;
	while (len--) {
		crc ^= *p++;
		for (i = 0; i < 8; i++)
			crc = (crc >> 1) ^ ((crc & 1) ? CRC_POLYNOMIAL : 0);
	}
	return crc ^ 0xffffffff;
}

static void
pssense_add_clock_offset_sample_locked(struct pssense_device *pssense, timepoint_ns local_ns, timepoint_ns remote_ns)
{
	struct pssense_clock *c = &pssense->timing.clock;
	pssense_clock_push(c, local_ns, remote_ns);

	time_duration_ns skew_ns;
	if (pssense_clock_get_offset(c, &skew_ns)) {
		t_led_sync_push_host_device_clock_offset(&pssense->tracking.led_sync_refinement, -skew_ns);

		if (local_ns - c->stats.last_log_ns >= U_TIME_1S_IN_NS * 5) {
			if (c->stats.last_log_ns != 0) {
				PSSENSE_DEBUG(pssense,
				              "Clock: offset moved %+" PRId64 "us in %.1fs, skew %+.1fppm, max Bluetooth delay "
				              "%" PRId64 "us, %u resets",
				              (int64_t)((skew_ns - c->stats.output_at_log) / U_TIME_1US_IN_NS),
				              (double)(local_ns - c->stats.last_log_ns) / U_TIME_1S_IN_NS, c->b / 1000.0,
				              (int64_t)(c->stats.max_delay_ns / U_TIME_1US_IN_NS), c->stats.resets);
			}
			c->stats.output_at_log = skew_ns;
			c->stats.max_delay_ns = 0;
			c->stats.resets = 0;
			c->stats.last_log_ns = local_ns;
		}
	}
}

/*!
 * Feeds the IMU-timestamp clock (one-way, the same estimator as the device timestamp's) and the two-way echo
 * estimate: the controller echoes the host time of the newest output report it received (input @45), so for the
 * first report carrying a new echo, offset = (T_send + T_receive) / 2 - device time, within +-RTT/2, and the minimum
 * round trip per second is kept (PSVR2-Documentation 11.2). Logged against the one-way estimate every 5s.
 */
static void
pssense_add_imu_clock_sample_locked(struct pssense_device *pssense, timepoint_ns recv_time_ns, uint32_t echo_us)
{
	struct pssense_clock *c = &pssense->timing.imu_clock;
	pssense_clock_push(c, recv_time_ns, pssense->timing.latest_imu_time_ns);

	__typeof__(pssense->timing.echo) *e = &pssense->timing.echo;
	if (e->have_echo && echo_us != e->last_echo_us && echo_us != 0) {
		// Unwrap the 32-bit microsecond echo against the receive time; it was sent within the last ~71 minutes.
		uint32_t now_us = (uint32_t)(recv_time_ns / U_TIME_1US_IN_NS);
		timepoint_ns sent_ns = recv_time_ns - (timepoint_ns)(uint32_t)(now_us - echo_us) * U_TIME_1US_IN_NS;
		time_duration_ns rtt_ns = recv_time_ns - sent_ns;
		if (rtt_ns > 0 && rtt_ns < U_TIME_1MS_IN_NS * 40) {
			time_duration_ns offset_ns = (sent_ns + rtt_ns / 2) - pssense->timing.latest_imu_time_ns;
			if (recv_time_ns - e->window_start_ns >= U_TIME_1S_IN_NS) {
				if (e->window_start_ns != 0 && e->window_min_rtt_ns > 0) {
					e->last_offset_ns = e->window_offset_ns;
					e->last_rtt_ns = e->window_min_rtt_ns;
					e->have_window = true;
				}
				e->window_start_ns = recv_time_ns;
				e->window_min_rtt_ns = 0;
			}
			if (e->window_min_rtt_ns == 0 || rtt_ns < e->window_min_rtt_ns) {
				e->window_min_rtt_ns = rtt_ns;
				e->window_offset_ns = offset_ns;
			}
		}
	}
	e->last_echo_us = echo_us;
	e->have_echo = true;

	time_duration_ns imu_off_ns, dev_off_ns;
	if (c->stats.last_log_ns == 0 || recv_time_ns - c->stats.last_log_ns >= U_TIME_1S_IN_NS * 5) {
		if (c->stats.last_log_ns != 0 && pssense_clock_get_offset(c, &imu_off_ns) &&
		    pssense_clock_get_offset(&pssense->timing.clock, &dev_off_ns)) {
			PSSENSE_DEBUG(pssense,
			              "IMU clock: IMU counter - device timestamp counter %+.3fms, IMU skew %+.1fppm, "
			              "two-way echo offset - one-way %s%+.3fms (min round trip %.1fms)",
			              (double)(imu_off_ns - dev_off_ns) / -1e6, c->b / 1000.0, e->have_window ? "" : "n/a ",
			              e->have_window ? (double)(e->last_offset_ns - imu_off_ns) / 1e6 : 0.0,
			              (double)e->last_rtt_ns / 1e6);
		}
		c->stats.last_log_ns = recv_time_ns;
	}
}

//! Host time of an IMU timestamp (input @29), through the IMU timestamp's own clock.
static bool
pssense_imu_ts_to_host(struct pssense_device *pssense, timepoint_ns imu_ns, timepoint_ns *out_host_ns)
{
	return pssense_clock_to_local(&pssense->timing.imu_clock, imu_ns, out_host_ns);
}

//! IMU timestamp (input @29) of a host time.
static bool
pssense_host_ts_to_imu(struct pssense_device *pssense, timepoint_ns host_ns, timepoint_ns *out_imu_ns)
{
	return pssense_clock_to_remote(&pssense->timing.imu_clock, host_ns, out_imu_ns);
}

static bool
pssense_host_ts_to_device(struct pssense_device *pssense,
                          timepoint_ns host_timestamp_ns,
                          timepoint_ns *out_device_timestamp_ns)
{
	switch (pssense->tracking.latest_led_sync_sample.timestamp_mode) {
	case T_LED_SYNC_SAMPLE_TIMESTAMP_MODE_INVALID:
	case T_LED_SYNC_SAMPLE_TIMESTAMP_MODE_DEVICE_HOST_LATENCY: {
		if (!pssense_clock_to_remote(&pssense->timing.clock, host_timestamp_ns,
		                                             out_device_timestamp_ns)) {
			return false;
		}
		// The refinement routine only reports the device->host transfer latency in this mode, the driver
		// has to apply it on top of its own clock tracking.
		*out_device_timestamp_ns += pssense->tracking.latest_led_sync_sample.timestamp.device_host_latency_ns;
		return true;
	}
	case T_LED_SYNC_SAMPLE_TIMESTAMP_MODE_HOST_DEVICE_CLOCK_OFFSET: {
		const struct t_led_sync_sample *s = &pssense->tracking.latest_led_sync_sample;
		*out_device_timestamp_ns = host_timestamp_ns + s->timestamp.host_device_clock_offset_ns +
		                           (time_duration_ns)(s->clock_skew * (host_timestamp_ns - s->clock_epoch_ns));
		return true;
	}
	}

	return false;
}

//! Converts the latest sample's base-time delta to whole IMU ticks, carrying the remainder into the next one.
static void
pssense_convert_led_sync_delta_locked(struct pssense_device *pssense)
{
	const struct t_led_sync_sample *sample = &pssense->tracking.latest_led_sync_sample;

	// An absolute position replaces everything the deltas built up.
	if (sample->pssense_mode == T_LED_SYNC_SAMPLE_PSSENSE_MODE_PRESCAN) {
		pssense->tracking.led_sync_delta_ticks = 0;
		pssense->tracking.led_sync_delta_remainder_ns3 = 0;
		return;
	}

	// One IMU tick is exactly 1000 thirds of a nanosecond.
	int64_t delta_ns3 = sample->pssense.base_time_offset_ns * 3 + pssense->tracking.led_sync_delta_remainder_ns3;
	pssense->tracking.led_sync_delta_ticks = (int32_t)(delta_ns3 / 1000);
	pssense->tracking.led_sync_delta_remainder_ns3 = delta_ns3 % 1000;
}

static bool
pssense_device_ts_to_host(struct pssense_device *pssense,
                          timepoint_ns device_timestamp_ns,
                          timepoint_ns *out_host_timestamp_ns)
{
	switch (pssense->tracking.latest_led_sync_sample.timestamp_mode) {
	case T_LED_SYNC_SAMPLE_TIMESTAMP_MODE_INVALID:
	case T_LED_SYNC_SAMPLE_TIMESTAMP_MODE_DEVICE_HOST_LATENCY: {
		if (!pssense_clock_to_local(&pssense->timing.clock, device_timestamp_ns,
		                                            out_host_timestamp_ns)) {
			return false;
		}
		*out_host_timestamp_ns -= pssense->tracking.latest_led_sync_sample.timestamp.device_host_latency_ns;
		return true;
	}
	case T_LED_SYNC_SAMPLE_TIMESTAMP_MODE_HOST_DEVICE_CLOCK_OFFSET: {
		const struct t_led_sync_sample *s = &pssense->tracking.latest_led_sync_sample;
		double skew = s->clock_skew;
		double epoch = (double)s->clock_epoch_ns;
		double offset = (double)s->timestamp.host_device_clock_offset_ns;
		double device = (double)device_timestamp_ns;

		// Exact inverse of:
		// device_ts = host_ts + offset + skew * (host_ts - epoch)
		//          = host_ts * (1 + skew) + offset - skew * epoch
		// host_ts  = (device_ts - offset + skew * epoch) / (1 + skew)
		*out_host_timestamp_ns = (timepoint_ns)((device - offset + (skew * epoch)) / (1.0 + skew));
		return true;
	}
	}

	return false;
}

/*!
 * Reads one packet from the device, wrapping no data as EAGAIN. Does not block.
 */
static int
pssense_read_packet_data(struct pssense_device *pssense, uint8_t *buffer, size_t size)
{
	// Poll, don't block. Outer thread needs to run quick
	int ret = h_hid_read(pssense->hid, buffer, size, 0);

	// No data yet
	if (ret == 0) {
		return -EAGAIN;
	}

	if (ret < 0) {
		PSSENSE_ERROR(pssense, "Failed to read device '%i'!", ret);
		return ret;
	}

	return ret;
}

static void
pssense_update_fusion(struct pssense_device *pssense)
{
	// We don't have calibration yet, so we can't do anything
	if (!pssense->has_calibration) {
		return;
	}

	struct xrt_vec3 gyro = {
	    .x = (pssense->state.gyro_raw.x - pssense->calibration.gyro_bias.x) * pssense->calibration.gyro_scale.x,
	    .y = (pssense->state.gyro_raw.y - pssense->calibration.gyro_bias.y) * pssense->calibration.gyro_scale.y,
	    .z = (pssense->state.gyro_raw.z - pssense->calibration.gyro_bias.z) * pssense->calibration.gyro_scale.z,
	};

	struct xrt_vec3 accel = {
	    .x = (pssense->state.accel_raw.x - pssense->calibration.accel_bias.x) * pssense->calibration.accel_scale.x,
	    .y = (pssense->state.accel_raw.y - pssense->calibration.accel_bias.y) * pssense->calibration.accel_scale.y,
	    .z = (pssense->state.accel_raw.z - pssense->calibration.accel_bias.z) * pssense->calibration.accel_scale.z,
	};

	m_imu_3dof_update(&pssense->tracking.fusion, pssense->timing.latest_imu_time_ns, &accel, &gyro);
	pssense->tracking.pose.orientation = pssense->tracking.fusion.rot;

	struct xrt_space_relation space_relation = {
	    .pose = pssense->tracking.pose,
	    .relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                      XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT,
	    .angular_velocity = gyro,
	};
	m_relation_history_push(pssense->tracking.imu_relation_history, &space_relation,
	                        pssense->timing.latest_imu_time_ns);

	if (pssense->tracking.constellation_imu_sink != NULL) {
		struct xrt_imu_sample sample = {
		    .accel_m_s2 =
		        {
		            .x = accel.x,
		            .y = accel.y,
		            .z = accel.z,
		        },
		    .gyro_rad_secs =
		        {
		            .x = gyro.x,
		            .y = gyro.y,
		            .z = gyro.z,
		        },
		};

		if (pssense_imu_ts_to_host(pssense, pssense->timing.latest_imu_time_ns, &sample.timestamp_ns)) {
			xrt_sink_push_imu(pssense->tracking.constellation_imu_sink, &sample);
		}
	}
}

static int
pssense_handle_packet(struct pssense_device *pssense,
                      timepoint_ns recv_time_ns,
                      const struct pssense_input_report_common *data)
{
	// Final input state
	struct pssense_input_state input = {
	    .timestamp_ns = recv_time_ns,
	};

#if 0 // IMU rate test
	static timepoint_ns last[2] = {0};
	static double rate[2] = {133, 133};
	rate[pssense->hand] =
	    (rate[pssense->hand] * 0.999) + ((1 / time_ns_to_s(recv_time_ns - last[pssense->hand])) * 0.001);
	printf("%d\trate: %lfhz\n", pssense->hand, rate[pssense->hand]);
	last[pssense->hand] = recv_time_ns;
#endif

	uint32_t seq_no = __le32_to_cpu(data->seq_no);
	if (input.seq_no != 0 && seq_no != input.seq_no + 1) {
		PSSENSE_WARN(pssense, "Missed seq no %u. Previous was %u", seq_no, input.seq_no);
	}
	input.seq_no = seq_no;

	// Update input state
	input.ps_click = (data->buttons[1] & 16) != 0;
	input.squeeze_touch = (data->buttons[2] & 8) != 0;
	input.squeeze_proximity = data->squeeze_proximity / 255.0f;
	input.trigger_touch = (data->buttons[1] & 128) != 0;
	input.trigger_value = data->trigger_value / 255.0f;
	input.trigger_proximity = data->trigger_proximity / 255.0f;
	input.thumbstick.x = (data->thumbstick_x - 128) / 128.0f;
	input.thumbstick.y = (data->thumbstick_y - 128) / -128.0f;
	input.thumbstick_touch = (data->buttons[2] & 4) != 0;

	if (pssense->hand == XRT_HAND_LEFT) {
		input.share_click = (data->buttons[1] & 1) != 0;
		input.square_click = (data->buttons[0] & 1) != 0;
		input.square_touch = (data->buttons[2] & 2) != 0;
		input.triangle_click = (data->buttons[0] & 8) != 0;
		input.triangle_touch = (data->buttons[2] & 1) != 0;
		input.squeeze_click = (data->buttons[0] & 16) != 0;
		input.trigger_click = (data->buttons[0] & 64) != 0;
		input.thumbstick_click = (data->buttons[1] & 4) != 0;
	} else if (pssense->hand == XRT_HAND_RIGHT) {
		input.options_click = (data->buttons[1] & 2) != 0;
		input.cross_click = (data->buttons[0] & 2) != 0;
		input.cross_touch = (data->buttons[2] & 2) != 0;
		input.circle_click = (data->buttons[0] & 4) != 0;
		input.circle_touch = (data->buttons[2] & 1) != 0;
		input.squeeze_click = (data->buttons[0] & 32) != 0;
		input.trigger_click = (data->buttons[0] & 128) != 0;
		input.thumbstick_click = (data->buttons[1] & 8) != 0;
	}

	// Update IMU data
	uint32_t imu_ticks = __le32_to_cpu(data->imu_ticks);
	int64_t imu_ticks_delta = imu_ticks - pssense->timing.imu_ticks_last;
	if (imu_ticks_delta >= 0) {
		pssense->timing.imu_ticks_total += imu_ticks_delta;
		pssense->timing.imu_ticks_last = imu_ticks;

		pssense->timing.latest_imu_time_ns = IMU_TICKS_TO_NS(pssense->timing.imu_ticks_total);

		input.gyro_raw.x = (int16_t)__le16_to_cpu(data->gyro[0]);
		input.gyro_raw.y = (int16_t)__le16_to_cpu(data->gyro[1]);
		input.gyro_raw.z = (int16_t)__le16_to_cpu(data->gyro[2]);

		input.accel_raw.x = (int16_t)__le16_to_cpu(data->accel[0]);
		input.accel_raw.y = (int16_t)__le16_to_cpu(data->accel[1]);
		input.accel_raw.z = (int16_t)__le16_to_cpu(data->accel[2]);
	} else {
		PSSENSE_WARN(pssense, "Time went backwards. Check your play area for black holes.");
	}

	uint32_t device_ticks = __le32_to_cpu(data->device_timestamp_ticks);
	int64_t device_ticks_delta = device_ticks - pssense->timing.device_ticks_last;
	if (device_ticks_delta >= 0) {
		pssense->timing.device_ticks_total += device_ticks_delta;
		pssense->timing.device_ticks_last = device_ticks;

		pssense->timing.latest_device_time_ns = IMU_TICKS_TO_NS(pssense->timing.device_ticks_total);
	} else {
		PSSENSE_WARN(pssense, "Device time went backwards. Check your play area for black holes.");
	}

	// Battery state is upper 4 bits
	uint8_t battery_state = data->battery_state >> 4;

	// Charge values go from 0..10, so add 5% and cap at 100% so we never show 0% charge
	float battery_percent = MIN(1.0f, (data->battery_state & 0xf) * .1f + .05);

	bool battery_state_valid, charging;
	if (battery_state == CHARGE_STATE_DISCHARGING) {
		battery_state_valid = true;
		charging = false;
	} else if (battery_state == CHARGE_STATE_CHARGING) {
		battery_state_valid = true;
		charging = true;
	} else if (battery_state == CHARGE_STATE_FULL) {
		battery_state_valid = true;
		charging = true;
		battery_percent = 1.0f;
	} else if (battery_state == CHARGE_STATE_ABNORMAL_VOLTAGE) {
		battery_state_valid = false;
		PSSENSE_WARN(pssense, "Unable to determine charge state: abnormal voltage");
	} else if (battery_state == CHARGE_STATE_ABNORMAL_TEMP) {
		battery_state_valid = false;
		PSSENSE_WARN(pssense, "Unable to determine charge state: abnormal temp");
	} else if (battery_state == CHARGE_STATE_CHARGING_ERROR) {
		battery_state_valid = false;
		PSSENSE_WARN(pssense, "Unable to determine charge state: charging error");
	} else {
		battery_state_valid = false;
		PSSENSE_WARN(pssense, "Unable to determine charge state: unknown reason");
	}

	input.battery_state_valid = battery_state_valid;
	if (battery_state_valid) {
		if (charging != input.battery_charging || battery_percent != input.battery_charge_percent) {
			PSSENSE_TRACE(pssense, "Battery at %.f%%, %s", battery_percent * 100,
			              charging ? "charging" : "discharging");
		}
		input.battery_charging = charging;
		input.battery_charge_percent = battery_percent;
	}

	os_thread_helper_lock(&pssense->controller_thread);

	// Mark the LED sync refinement sample as applied
	uint32_t latest_host_send_time = __le32_to_cpu(data->host_timestamp);
	/*
	 * The controller echoes the host timestamp of the newest output report it received. The sample is applied
	 * once that is the report carrying it or a later one. This used to accept any other value, including echoes
	 * of reports sent *before* the sample, so samples were marked applied right away, before the controller had
	 * them; with Bluetooth round trips of 100ms+ LED sync then measured the previous LED state (on hardware the
	 * count calibration saw more blobs with the LEDs off than on).
	 */
	if (pssense->tracking.led_sync_sample_needs_marking &&
	    (int32_t)(latest_host_send_time - pssense->timing.last_sent_host_timestamp_us) >= 0) {
		t_led_sync_mark_latest_sample_applied(&pssense->tracking.led_sync_refinement, recv_time_ns);
		pssense->tracking.led_sync_sample_needs_marking = false;

		time_duration_ns rtt_ns = recv_time_ns - pssense->timing.last_led_sample_send_ns;
		pssense->timing.led_rtt.count++;
		pssense->timing.led_rtt.sum_ns += rtt_ns;
		pssense->timing.led_rtt.max_ns = MAX(pssense->timing.led_rtt.max_ns, rtt_ns);
		if (recv_time_ns - pssense->timing.led_rtt.last_log_ns >= U_TIME_1S_IN_NS * 5) {
			PSSENSE_DEBUG(pssense, "LED command round trip: mean %" PRId64 "ms, max %" PRId64 "ms over %u",
			              (int64_t)(pssense->timing.led_rtt.sum_ns / pssense->timing.led_rtt.count /
			                        U_TIME_1MS_IN_NS),
			              (int64_t)(pssense->timing.led_rtt.max_ns / U_TIME_1MS_IN_NS),
			              pssense->timing.led_rtt.count);
			pssense->timing.led_rtt.count = 0;
			pssense->timing.led_rtt.sum_ns = 0;
			pssense->timing.led_rtt.max_ns = 0;
			pssense->timing.led_rtt.last_log_ns = recv_time_ns;
		}
	}

	// Update the clock offset
	pssense_add_clock_offset_sample_locked(pssense, recv_time_ns, pssense->timing.latest_device_time_ns);
	pssense_add_imu_clock_sample_locked(pssense, recv_time_ns, latest_host_send_time);

	pssense->state = input;
	pssense_update_fusion(pssense);

	os_thread_helper_unlock(&pssense->controller_thread);

	return 0;
}

static int
pssense_handle_read(struct pssense_device *pssense)
{
	int ret;

	// Report data
	uint8_t buf[INPUT_REPORT_BLUETOOTH_LENGTH] = {0};
	ret = pssense_read_packet_data(pssense, buf, sizeof(buf));

	// Get the receive time as close to the packet read as possible
	timepoint_ns recv_time_ns = os_monotonic_get_ns();

	if (ret == -EAGAIN) {
		// No data yet, not an error
		return 0;
	}

	if (ret < 0) {
		PSSENSE_ERROR(pssense, "Error reading from device: %d", ret);
		return ret;
	}
	pssense->tracking.read_had_data = true;

	switch (buf[0]) {
	case INPUT_REPORT_ID_USB: {
		struct pssense_usb_input_report data = {0};
		if (ret != sizeof(data)) {
			/*
			 * Over Bluetooth, the controller sends short "simple" reports with this same ID until it's
			 * switched to full 0x31 reports. Returning an error here ends the controller thread on the very
			 * first packet (no input, output, LEDs or LED sync after that), so just skip it.
			 */
			if (!pssense->usb) {
				PSSENSE_DEBUG(pssense, "Skipping %d byte Bluetooth simple report", ret);
			} else {
				PSSENSE_WARN(pssense, "Unexpected USB input report size %d (expected %zu)", ret,
				             sizeof(data));
			}
			return 0;
		}

		memcpy(&data, buf, sizeof(data));

		return pssense_handle_packet(pssense, recv_time_ns, &data.common);
	}
	case INPUT_REPORT_ID_BLUETOOTH: {
		struct pssense_bluetooth_input_report data = {0};
		if (ret != sizeof(data)) {
			PSSENSE_WARN(pssense, "Unexpected Bluetooth input report size %d (expected %zu), skipping", ret,
			             sizeof(data));
			return 0;
		}

		memcpy(&data, buf, sizeof(data));

		// Verify the CRC of the packet
		uint32_t expected_crc = __le32_to_cpu(data.crc);
		uint32_t crc = crc32_le(0, &INPUT_REPORT_CRC32_SEED, 1);
		crc = crc32_le(crc, (uint8_t *)&data, sizeof(struct pssense_bluetooth_input_report) - 4);
		if (crc != expected_crc) {
			// One corrupted Bluetooth packet shouldn't end the controller thread, skip it.
			PSSENSE_WARN(pssense, "CRC mismatch; skipping input. Expected %08X but got %08X", expected_crc,
			             crc);
			return 0;
		}

		return pssense_handle_packet(pssense, recv_time_ns, &data.common);
	}
	default: {
		PSSENSE_WARN(pssense, "Unhandled HID report id %u", buf[0]);
	}
	}

	return 0;
}

static void
pssense_set_output_report_settings_locked(struct pssense_device *pssense,
                                          struct pssense_output_settings *settings,
                                          bool do_vibration,
                                          uint64_t now_ns)
{
	if (now_ns >= pssense->output.vibration_end_timestamp_ns) {
		pssense->output.vibration_amplitude = 0;
	}

	if (pssense->output.send_vibration && do_vibration) {
		settings->flag1 |= OUTPUT_SETTINGS_ENABLE_VIBRATION_BITS | pssense->output.vibration_mode;
		settings->vibration_amplitude = pssense->output.vibration_amplitude;
		pssense->output.last_sent_vibration_amplitude = pssense->output.vibration_amplitude;
		pssense->output.last_sent_vibration_mode = pssense->output.vibration_mode;
		pssense->output.send_vibration = pssense->output.vibration_amplitude > 0;
	}

	if (pssense->output.send_trigger_feedback) {
		settings->flag1 |= PSSENSE_OUTPUT_SETTINGS_FLAG1_ADAPTIVE_TRIGGER_ENABLE;
		settings->trigger_settings.mode = pssense->output.trigger_feedback_mode;
		pssense->output.send_trigger_feedback = false;
	}

	// Give it some time to settle
	if (pssense->tracking.received_frames > 10) {
		settings->led_settings = pssense->tracking.led_settings;

#if 0
		PSSENSE_DEBUG(pssense, "Full LED settings: cycle length %uns, cycle position %luns, sequence number %u",
		              settings->led_settings.cycle_length / 3,
		              (timepoint_ns)IMU_TICKS_TO_NS(settings->led_settings.cycle_position),
		              pssense->tracking.led_sequence_num);

		PSSENSE_DEBUG_HEX(pssense, (uint8_t *)&settings->led_settings,
		                  sizeof(settings->led_settings));
#endif
	} else {
		settings->led_settings.phase = LED_SYNC_PHASE_LED_ALL_OFF;
	}

	pssense->output.next_seq_no = (pssense->output.next_seq_no + 1) % 16;

	uint32_t host_send_ts = os_monotonic_get_ns() / U_TIME_1US_IN_NS;

	// Set the host timestamp as *close* as possible to us sending the packet
	settings->host_timestamp_send_time_us = __cpu_to_le32(host_send_ts);

	if (pssense->tracking.led_sync_sample_needs_sending) {
		pssense->timing.last_sent_host_timestamp_us = host_send_ts;
		pssense->timing.last_led_sample_send_ns = os_monotonic_get_ns();
		pssense->tracking.led_sync_sample_needs_sending = false;
		pssense->tracking.led_sync_sample_needs_marking = true;
	}
}

static int
pssense_send_bluetooth_output_report_locked(struct pssense_device *pssense)
{
	uint64_t timestamp_ns = os_monotonic_get_ns();

	struct pssense_ps5_output_report report = {
	    .report_id = OUTPUT_REPORT_ID_BLUETOOTH,
	    // low bits are always zero, to indicate we are using the PS5 packet format
	    .seq_no_mode = (pssense->output.next_seq_no << 4) | (0x0),
	    .tag = OUTPUT_REPORT_TAG,
	    // Packet counter needs to increment with every packet, or PCM haptics won't work.
	    .counter = pssense->output.packet_counter++,
	};

	float pcm_buf[PCM_HAPTIC_BUF_SIZE] = {0};
	size_t read_pcm_samples = u_resampler_read(pssense->output.pcm_haptics_resampler, pcm_buf, ARRAY_SIZE(pcm_buf));

	if (read_pcm_samples > 0) {
		for (size_t i = 0; i < read_pcm_samples; i++) {
			// Convert from float [-1, 1] to int8 [-128, 127].
			report.haptics[i] = (int8_t)(CLAMP(((pcm_buf[i] + 1.0f) * 0.5f * 255) - 128, -128, 127));
		}
	}

	pssense_set_output_report_settings_locked(pssense, &report.settings, read_pcm_samples == 0, timestamp_ns);

	uint32_t crc = crc32_le(0, &OUTPUT_REPORT_CRC32_SEED, 1);
	crc = crc32_le(crc, (uint8_t *)&report, sizeof(struct pssense_ps5_output_report) - 4);
	report.crc = __cpu_to_le32(crc);

	PSSENSE_TRACE(pssense,
	              "Setting vibration amplitude: %u, mode: %02X, trigger feedback mode: %02X. Next seq no: %u. PCM "
	              "samples: %zu",
	              pssense->output.vibration_amplitude, pssense->output.vibration_mode,
	              pssense->output.trigger_feedback_mode, pssense->output.next_seq_no, read_pcm_samples);
	int ret = h_hid_write(pssense->hid, (uint8_t *)&report, sizeof(report));
	if (ret != sizeof(report)) {
		PSSENSE_WARN(pssense, "Failed to send output report: %d", ret);
		return ret < 0 ? ret : -EIO;
	}

#if 0
	PSSENSE_DEBUG_HEX(pssense, (uint8_t *)&report, sizeof(report));
#endif

	return 0;
}

static int
pssense_send_usb_report_locked(struct pssense_device *pssense)
{
	uint64_t timestamp_ns = os_monotonic_get_ns();

	struct pssense_usb_output_report report = {
	    .seq_no_mode = (pssense->output.next_seq_no << 4) | (0x2),
	};

	pssense_set_output_report_settings_locked(pssense, &report.settings, true, timestamp_ns);

	int ret = h_hid_write(pssense->hid, (uint8_t *)&report, sizeof(report));
	if (ret != sizeof(report)) {
		PSSENSE_WARN(pssense, "Failed to send output report: %d", ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int
pssense_send_output_report_locked(struct pssense_device *pssense)
{
	if (pssense->usb) {
		return pssense_send_usb_report_locked(pssense);
	} else {
		return pssense_send_bluetooth_output_report_locked(pssense);
	}

	assert(!"unreachable");
	return -EINVAL;
}

//! Sets the controller to use the lower 133hz polling rate
static bool
pssense_set_pc_polling_rate(struct pssense_device *pssense)
{
	struct pssense_set_polling_rate_feature_report report = {
	    .report_id = SET_POLLING_RATE_FEATURE_REPORT_ID,
	    .unk = 0x0E,
	    .rate_1 = __cpu_to_le16(0x000C),
	    .rate_2 = __cpu_to_le16(0x0002),
	};

	uint32_t crc = crc32_le(0, &SET_FEATURE_REPORT_CRC32_SEED, 1);
	crc = crc32_le(crc, (uint8_t *)&report, sizeof(struct pssense_set_polling_rate_feature_report) - 4);
	report.crc = __cpu_to_le32(crc);

	int ret = h_hid_set_feature(pssense->hid, (uint8_t *)&report, sizeof report);
	if (ret < 0) {
		PSSENSE_ERROR(pssense, "Failed to set PC polling rate, reason %d", ret);
		return false;
	}

	return true;
}

static void *
pssense_run_thread(void *ptr)
{
	U_TRACE_SET_THREAD_NAME("PS Sense");

	struct pssense_device *pssense = ptr;

#ifdef XRT_OS_LINUX
	u_linux_try_to_set_realtime_priority_on_thread(pssense->log_level, "PS Sense");
#endif

	os_thread_helper_lock(&pssense->controller_thread);

	// 32/3000hz (PCM haptic rate), this will *technically* run slightly fast, but like, that's fine.
	const time_duration_ns pcm_haptics_period_ns = 10666666;

	timepoint_ns next_output_ns = os_monotonic_get_ns();

	int result = 0;
	// Consecutive failed output reports. A dropped Bluetooth write is transient (both controllers were seen failing
	// at the same instant on a shared adapter), so it must not end the thread: that stops LED updates for good,
	// freezing LED sync. Only give up when sending keeps failing, which means the controller is gone.
	uint32_t failed_sends = 0;
	const uint32_t max_failed_sends = 200; // ~2s at the output rate
	while (os_thread_helper_is_running_locked(&pssense->controller_thread) && result >= 0) {
		os_thread_helper_unlock(&pssense->controller_thread);

		/*
		 * Drain what's queued rather than one report per loop: the loop sleeps up to 1ms between passes and
		 * reports arriving in a burst after a Bluetooth stall waited behind each other, adding latency.
		 */
		for (int reads = 0; reads < 16; reads++) {
			pssense->tracking.read_had_data = false;
			result = pssense_handle_read(pssense);
			if (result < 0 || !pssense->tracking.read_had_data) {
				break;
			}
		}

		if (result >= 0) {
			timepoint_ns now = os_monotonic_get_ns();

			if (now >= next_output_ns) {
				os_thread_helper_lock(&pssense->controller_thread);
				bool carried_led_sample = pssense->tracking.led_sync_sample_needs_sending;

				/*
				 * Only send when there is something to send: PCM haptics playing, vibration or trigger
				 * feedback changes, an LED sync sample, or changed LED settings; otherwise a slow keepalive.
				 * Sending a full report to every controller at the PCM rate regardless loaded the Bluetooth
				 * link enough to back up writes ("BT socket write error ... (11)") right before adapter resets.
				 */
				bool led_changed = memcmp(&pssense->output.last_sent_led_settings,
				                          &pssense->tracking.led_settings,
				                          sizeof(pssense->tracking.led_settings)) != 0;
				if (led_changed && pssense->tracking.count_calibration) {
					// The PRESCAN position is rebuilt every frame, but without a new sequence number
					// the controller ignores it, so that alone isn't worth a report.
					struct pssense_led_settings a = pssense->output.last_sent_led_settings;
					struct pssense_led_settings b = pssense->tracking.led_settings;
					a.cycle_position_absolute = 0;
					b.cycle_position_absolute = 0;
					// Follows the averaged exposure interval, so changes slightly every frame too.
					a.cycle_length = 0;
					b.cycle_length = 0;
					led_changed = memcmp(&a, &b, sizeof(a)) != 0;
				}
				/*
				 * A vibration only needs a report when it changes; send_vibration stays set for as long as
				 * it plays, which kept both controllers at the rate cap. Waiting to mark an LED sync sample
				 * applied needs any report (the keepalive does), not one every slot.
				 */
				bool vibration_changed =
				    pssense->output.send_vibration &&
				    (pssense->output.vibration_amplitude != pssense->output.last_sent_vibration_amplitude ||
				     pssense->output.vibration_mode != pssense->output.last_sent_vibration_mode ||
				     now >= (timepoint_ns)pssense->output.vibration_end_timestamp_ns);
				bool needed = pssense->usb || now < pssense->output.pcm_active_until_ns || vibration_changed ||
				              pssense->output.send_trigger_feedback ||
				              carried_led_sample ||
				              led_changed || (now - pssense->output.last_send_ns) >= PSSENSE_KEEPALIVE_NS;

				/*
				 * Rate cap for Bluetooth: PCM haptics need the full rate, everything else waits for the
				 * next allowed slot (the pending flags stay set, so nothing is lost, only merged).
				 */
				const time_duration_ns min_interval_ns =
				    (time_duration_ns)debug_get_num_option_pssense_bt_min_send_interval_ms() *
				    U_TIME_1MS_IN_NS;
				bool pcm_active = now < pssense->output.pcm_active_until_ns;
				bool rate_ok = pssense->usb ||
				               (now >= pssense->output.backoff_until_ns &&
				                (pcm_active || now - pssense->output.last_send_ns >= min_interval_ns));
				if (needed && !rate_ok) {
					pssense->output.stats.held++;
					needed = false;
				}
				if (needed) {
					pssense->output.stats.led_sample += carried_led_sample ? 1 : 0;
					pssense->output.stats.marking +=
					    pssense->tracking.led_sync_sample_needs_marking ? 1 : 0;
					pssense->output.stats.haptics += (pcm_active || pssense->output.send_vibration ||
					                                  pssense->output.send_trigger_feedback)
					                                     ? 1
					                                     : 0;
					pssense->output.stats.led_changed += led_changed ? 1 : 0;
					pssense->output.stats.keepalive +=
					    (now - pssense->output.last_send_ns) >= PSSENSE_KEEPALIVE_NS ? 1 : 0;
				}

				int send_result = 0;
				if (needed) {
					send_result = pssense_send_output_report_locked(pssense);
					if (send_result >= 0) {
						pssense->output.stats.sent++;
						pssense->output.last_send_ns = now;
						pssense->output.last_sent_led_settings = pssense->tracking.led_settings;
					} else {
						pssense->output.stats.failed++;
						pssense->output.backoff_until_ns = now + PSSENSE_SEND_FAIL_BACKOFF_NS;
					}
					if (send_result < 0 && carried_led_sample) {
						// The LED sync sample in this report never reached the controller, send it
						// again rather than letting it be marked as applied.
						pssense->tracking.led_sync_sample_needs_marking = false;
						pssense->tracking.led_sync_sample_needs_sending = true;
					}
				}
				os_thread_helper_unlock(&pssense->controller_thread);

				if (send_result < 0) {
					if (++failed_sends >= max_failed_sends) {
						PSSENSE_ERROR(pssense, "Output reports failed %u times in a row, stopping",
						              failed_sends);
						result = send_result;
					}
				} else {
					if (failed_sends > 0) {
						PSSENSE_DEBUG(pssense, "Output reports recovered after %u failures",
						              failed_sends);
					}
					failed_sends = 0;
				}

				if (now - pssense->output.stats.last_log_ns >= U_TIME_1S_IN_NS * 5) {
					double secs = (double)(now - pssense->output.stats.last_log_ns) / U_TIME_1S_IN_NS;
					if (pssense->output.stats.last_log_ns != 0) {
						PSSENSE_DEBUG(pssense,
						              "Send stats: %.1f reports/s, %u sent, %u failed, %u held by the rate "
						              "cap; reasons: LED sample %u, marking %u, keepalive %u, haptics %u, "
						              "LED settings %u",
						              pssense->output.stats.sent / secs, pssense->output.stats.sent,
						              pssense->output.stats.failed, pssense->output.stats.held,
						              pssense->output.stats.led_sample, pssense->output.stats.marking,
						              pssense->output.stats.keepalive, pssense->output.stats.haptics,
						              pssense->output.stats.led_changed);
					}
					memset(&pssense->output.stats, 0, sizeof(pssense->output.stats));
					pssense->output.stats.last_log_ns = now;
				}

				next_output_ns = next_output_ns + pcm_haptics_period_ns;
			}
		}

		if (result >= 0) {
			// @note we don't break the earlier `now` out into the outer scope accessible from here since
			//       sending may take some non-negligible amount of time.
			timepoint_ns to_next_output = next_output_ns - os_monotonic_get_ns();

			// Only sleep if it's an increment greater than 50us, Linux doesn't like sleeping that short and
			// often oversleeps a bit, timing matters with PCM haptics and LED sync!
			if (to_next_output > (U_TIME_1US_IN_NS * 50L)) {
				// Sleep 1ms, or half the time to the next output report, whichever is smaller. We want
				// to wake up frequently enough to read incoming packets, but not so much that we waste
				// CPU time waking too often.
				os_precise_sleeper_nanosleep(&pssense->sleeper,
				                             MIN(to_next_output / 2, U_TIME_1MS_IN_NS));
			}
		}

		os_thread_helper_lock(&pssense->controller_thread);
	}

	if (result < 0) {
		/*
		 * No more input will arrive, so the last pose can't be updated. Without this the stale fusion state
		 * kept being predicted forward with its last angular velocity, a controller spinning in place.
		 */
		pssense->disconnected = true;
		PSSENSE_ERROR(pssense, "Controller disconnected (read thread stopped: %d), reporting it untracked",
		              result);
	}

	os_thread_helper_unlock(&pssense->controller_thread);

	return NULL;
}

static void
pssense_get_imu_fusion_pose(struct pssense_device *pssense,
                            int64_t at_timestamp_ns,
                            struct xrt_space_relation *out_relation)
{
	// The history is indexed by IMU timestamp, so convert with the IMU timestamp's clock.
	timepoint_ns device_ts;
	if (!pssense_host_ts_to_imu(pssense, at_timestamp_ns, &device_ts)) {
		(*out_relation) = (struct xrt_space_relation){0};
		return;
	}

	/*
	 * Don't extrapolate the orientation far past the newest IMU sample: with the host -> controller clock off,
	 * or reports arriving late over Bluetooth, the history predicted the last angular velocity for hundreds of
	 * milliseconds, and the controller spun.
	 */
	int64_t latest_ns = 0;
	struct xrt_space_relation latest_rel;
	if (m_relation_history_get_latest(pssense->tracking.imu_relation_history, &latest_ns, &latest_rel)) {
		if (device_ts > latest_ns + (int64_t)U_TIME_1MS_IN_NS * 30) {
			device_ts = latest_ns + (int64_t)U_TIME_1MS_IN_NS * 30;
		}
		/*
		 * Nor look up an orientation older than the newest one: that only adds lag, and with the clock estimate
		 * off by hundreds of ms under Bluetooth congestion the controller followed the hand slowly.
		 */
		if (device_ts < latest_ns) {
			device_ts = latest_ns;
		}
	}

	m_relation_history_get(pssense->tracking.imu_relation_history, device_ts, out_relation);
}

static void
pssense_get_imu_fusion_pose(struct pssense_device *pssense,
                            int64_t at_timestamp_ns,
                            struct xrt_space_relation *out_relation);

//! Replace the orientation of @p rel with the controller's own IMU fusion (3DoF), if it has one.
static void
pssense_use_imu_orientation(struct pssense_device *pssense, int64_t at_timestamp_ns, struct xrt_space_relation *rel)
{
	struct xrt_space_relation imu_rel = {0};
	pssense_get_imu_fusion_pose(pssense, at_timestamp_ns, &imu_rel);
	if ((imu_rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		return;
	}
	rel->pose.orientation = imu_rel.pose.orientation;
	rel->angular_velocity = imu_rel.angular_velocity;
	rel->relation_flags = (enum xrt_space_relation_flags)(
	    rel->relation_flags | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
}

static void
pssense_get_constellation_pose(struct pssense_device *pssense,
                               int64_t at_timestamp_ns,
                               struct xrt_space_relation *out_relation)
{
	(*out_relation) = XRT_C11_COMPOUND(struct xrt_space_relation) XRT_SPACE_RELATION_ZERO;

	if (pssense->tracking.constellation_tracker &&
	    t_constellation_tracker_get_tracked_pose(pssense->tracking.constellation_tracker,
	                                             pssense->tracking.constellation_device_id, at_timestamp_ns,
	                                             out_relation) < 0) {
		(*out_relation) = XRT_C11_COMPOUND(struct xrt_space_relation) XRT_SPACE_RELATION_ZERO;
	}

	// What the tracker gives clients (fusion + IMU prediction), ~30Hz, to compare against the optical solves.
	if (pssense->log_level <= U_LOGGING_DEBUG && at_timestamp_ns - pssense->tracking.last_out_log_ns > 33000000) {
		pssense->tracking.last_out_log_ns = at_timestamp_ns;
		const struct xrt_space_relation *r = out_relation;
		PSSENSE_DEBUG(pssense, "OUT %" PRId64 " %d flags %x pos %.4f %.4f %.4f vel %.3f %.3f %.3f", at_timestamp_ns,
		              pssense->hand == XRT_HAND_LEFT ? 0 : 1, r->relation_flags, r->pose.position.x,
		              r->pose.position.y, r->pose.position.z, r->linear_velocity.x, r->linear_velocity.y,
		              r->linear_velocity.z);
	}

	if ((out_relation->relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0 &&
	    (out_relation->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
		const time_duration_ns max_prediction_ns =
		    (time_duration_ns)debug_get_num_option_pssense_max_prediction_ms() * U_TIME_1MS_IN_NS;
		const bool fresh = pssense->tracking.last_optical_fix_ns != 0 &&
		                   at_timestamp_ns - pssense->tracking.last_optical_fix_ns <= max_prediction_ns;

		const float max_error_m = (float)debug_get_num_option_pssense_fusion_max_error_cm() / 100.0f;
		if (fresh && max_error_m > 0.0f) {
			struct xrt_vec3 d = m_vec3_sub(out_relation->pose.position,
			                               pssense->tracking.last_optical_fix_pose.position);
			if (m_vec3_len(d) > max_error_m) {
				// The fusion has diverged from what the cameras see; trust the cameras.
				if (!pssense->tracking.overriding_fusion) {
					pssense->tracking.overriding_fusion = true;
					pssense->tracking.fusion_overrides++;
					PSSENSE_DEBUG(pssense,
					              "Fused pose %.2fm from the optical fix, using the optical pose "
					              "(override #%u)",
					              m_vec3_len(d), pssense->tracking.fusion_overrides);
				}
				out_relation->pose = pssense->tracking.last_optical_fix_pose;
				// The diverged fusion's orientation is as bad as its position, use the plain IMU one.
				pssense_use_imu_orientation(pssense, at_timestamp_ns, out_relation);
				out_relation->linear_velocity = (struct xrt_vec3)XRT_VEC3_ZERO;
				out_relation->relation_flags = (enum xrt_space_relation_flags)(
				    out_relation->relation_flags & ~XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT);
			} else if (pssense->tracking.overriding_fusion) {
				pssense->tracking.overriding_fusion = false;
				PSSENSE_DEBUG(pssense, "Fused pose agrees with the optical fix again");
			}
		}

		if (fresh || max_prediction_ns <= 0) {
			// Recent optical fix: the (fused or optical) pose is trustworthy, remember where it was.
			pssense->tracking.have_last_optical_position = true;
			pssense->tracking.last_optical_position = out_relation->pose.position;
			if (pssense->tracking.holding_position) {
				pssense->tracking.holding_position = false;
				PSSENSE_DEBUG(pssense, "Optical fix again, releasing held position");
			}
			return;
		}

		// Stale: keep the tracker's orientation, hold position where it was last backed by an optical fix.
		if (!pssense->tracking.holding_position) {
			pssense->tracking.holding_position = true;
			pssense->tracking.holds_started++;
			PSSENSE_DEBUG(pssense, "No optical fix for %" PRId64 "ms, holding position (hold #%u)",
			              (int64_t)((at_timestamp_ns - pssense->tracking.last_optical_fix_ns) / U_TIME_1MS_IN_NS),
			              pssense->tracking.holds_started);
		}
		if (pssense->tracking.have_last_optical_position) {
			out_relation->pose.position = pssense->tracking.last_optical_position;
		}
		/*
		 * Without fresh fixes the tracker keeps predicting its fused state, orientation included, and a diverged
		 * fusion froze or spun it: 3DoF stopped working once any optical fix had arrived. Use the plain IMU
		 * orientation, as when there's no optical lock at all.
		 */
		pssense_use_imu_orientation(pssense, at_timestamp_ns, out_relation);
		out_relation->linear_velocity = (struct xrt_vec3)XRT_VEC3_ZERO;
		out_relation->relation_flags = (enum xrt_space_relation_flags)(
		    (out_relation->relation_flags & ~(XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
		                                      XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT)) |
		    XRT_SPACE_RELATION_POSITION_VALID_BIT);
		return;
	}

	/*
	 * No optical lock (LED sync still searching, or out of view). Returning an identity pose with no flags put the
	 * controller at the tracking origin, which is the head's startup position, so it was never visible. Use the IMU
	 * orientation instead, at the last optical position, or in front of the startup head pose at chest height.
	 */
	struct xrt_space_relation imu_rel = {0};
	pssense_get_imu_fusion_pose(pssense, at_timestamp_ns, &imu_rel);
	if ((imu_rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		return;
	}

	out_relation->pose.orientation = imu_rel.pose.orientation;
	out_relation->angular_velocity = imu_rel.angular_velocity;
	if (pssense->tracking.have_last_optical_position) {
		out_relation->pose.position = pssense->tracking.last_optical_position;
	} else {
		float sx = pssense->hand == XRT_HAND_LEFT ? -1.0f : 1.0f;
		out_relation->pose.position = (struct xrt_vec3){.x = sx * 0.2f, .y = -0.4f, .z = -0.4f};
	}
	out_relation->linear_velocity = (struct xrt_vec3)XRT_VEC3_ZERO;
	out_relation->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
}

static void
parse_pssense_calibration(struct pssense_device *pssense,
                          const struct pssense_calibration_data *calibration_data,
                          struct pssense_parsed_calibration *out_parsed)
{
	int16_t gyro_speed_ref1 = (int16_t)__le16_to_cpu(calibration_data->gyro_speed_ref1);
	int16_t gyro_speed_ref2 = (int16_t)__le16_to_cpu(calibration_data->gyro_speed_ref2);
	float rad_per_sec_ref = ((float)gyro_speed_ref1 + (float)gyro_speed_ref2) * 0.5f * (float)(M_PI / 180.0);
	if (!(rad_per_sec_ref > 0.0f)) {
		PSSENSE_DEBUG(pssense, "Got invalid gyro speed reference of %f rad/s. Falling back to default.",
		              rad_per_sec_ref);
		rad_per_sec_ref = (float)(M_PI * 3.0);
	}

	PSSENSE_DEBUG(pssense, "Gyro speed reference: %f rad/s", rad_per_sec_ref);

	int16_t gyro_plus_x = __le16_to_cpu(calibration_data->gyro_plus_x);
	int16_t gyro_minus_x = __le16_to_cpu(calibration_data->gyro_minus_x);

	int16_t gyro_plus_y = __le16_to_cpu(calibration_data->gyro_plus_y);
	int16_t gyro_minus_y = __le16_to_cpu(calibration_data->gyro_minus_y);

	int16_t gyro_plus_z = __le16_to_cpu(calibration_data->gyro_plus_z);
	int16_t gyro_minus_z = __le16_to_cpu(calibration_data->gyro_minus_z);

	int16_t accel_plus_x = __le16_to_cpu(calibration_data->accel_plus_x);
	int16_t accel_minus_x = __le16_to_cpu(calibration_data->accel_minus_x);
	int16_t accel_plus_y = __le16_to_cpu(calibration_data->accel_plus_y);
	int16_t accel_minus_y = __le16_to_cpu(calibration_data->accel_minus_y);
	int16_t accel_plus_z = __le16_to_cpu(calibration_data->accel_plus_z);
	int16_t accel_minus_z = __le16_to_cpu(calibration_data->accel_minus_z);

	struct xrt_vec3_i32 gyro_bias = {
	    .x = (int16_t)__le16_to_cpu(calibration_data->gyro_bias_x),
	    .y = (int16_t)__le16_to_cpu(calibration_data->gyro_bias_y),
	    .z = (int16_t)__le16_to_cpu(calibration_data->gyro_bias_z),
	};

	float gyro_span_x =
	    (fabsf((float)gyro_plus_x - (float)gyro_bias.x) + fabsf((float)gyro_minus_x - (float)gyro_bias.x)) / 2.0f;
	float gyro_span_y =
	    (fabsf((float)gyro_plus_y - (float)gyro_bias.y) + fabsf((float)gyro_minus_y - (float)gyro_bias.y)) / 2.0f;
	float gyro_span_z =
	    (fabsf((float)gyro_plus_z - (float)gyro_bias.z) + fabsf((float)gyro_minus_z - (float)gyro_bias.z)) / 2.0f;

	struct xrt_vec3 gyro_scale = {
	    .x = rad_per_sec_ref / gyro_span_x,
	    .y = rad_per_sec_ref / gyro_span_y,
	    .z = rad_per_sec_ref / gyro_span_z,
	};

	struct xrt_vec3 accel_bias = {
	    .x = (accel_plus_x + accel_minus_x) / 2.0f,
	    .y = (accel_plus_y + accel_minus_y) / 2.0f,
	    .z = (accel_plus_z + accel_minus_z) / 2.0f,
	};

	struct xrt_vec3 accel_scale = {
	    .x = (1.0f / (float)(accel_plus_x - accel_bias.x)) * MATH_GRAVITY_M_S2,
	    .y = (1.0f / (float)(accel_plus_y - accel_bias.y)) * MATH_GRAVITY_M_S2,
	    .z = (1.0f / (float)(accel_plus_z - accel_bias.z)) * MATH_GRAVITY_M_S2,
	};

	PSSENSE_DEBUG(pssense, "Gyro scale: %f\t%f\t%f rad/s", gyro_scale.x, gyro_scale.y, gyro_scale.z);
	PSSENSE_DEBUG(pssense, "Gyro bias: %d\t%d\t%d", gyro_bias.x, gyro_bias.y, gyro_bias.z);
	PSSENSE_DEBUG(pssense, "Accel scale: %f\t%f\t%f m/s^2", accel_scale.x, accel_scale.y, accel_scale.z);
	PSSENSE_DEBUG(pssense, "Accel bias: %f\t%f\t%f", accel_bias.x, accel_bias.y, accel_bias.z);

	(*out_parsed) = XRT_C11_COMPOUND(struct pssense_parsed_calibration){
	    .gyro_scale = gyro_scale,
	    .accel_scale = accel_scale,

	    .accel_bias = accel_bias,
	    .gyro_bias = gyro_bias,
	};
}

/*!
 * Retrieving the calibration data report will switch the Sense controller from compat mode into full mode.
 */
static bool
pssense_get_calibration_data(struct pssense_device *pssense)
{
	struct pssense_calibration_data calibration_data = {0};

	bool invalid_crc;
	do {
		invalid_crc = false;

		// Calibration has to be read in two parts with two feature reads.
		for (int i = 0; i < 2; i++) {
			struct pssense_feature_report report_buffer = {.report_id = CALIBRATION_DATA_FEATURE_REPORT_ID};
			int ret = h_hid_get_feature(pssense->hid, report_buffer.report_id, (uint8_t *)&report_buffer,
			                            sizeof(report_buffer));

			if (ret < 0) {
				PSSENSE_ERROR(pssense, "Failed to retrieve calibration report: %d", ret);
				return false;
			}

			if (ret != sizeof(report_buffer)) {
				PSSENSE_ERROR(pssense, "Invalid byte count transferred, expected %zu got %d",
				              sizeof(report_buffer), ret);
				return false;
			}

			switch (report_buffer.part_id) {
			case CALIBRATION_DATA_PART_ID_1: {
				memcpy((uint8_t *)&calibration_data, report_buffer.data, sizeof(report_buffer.data));
				break;
			}
			case CALIBRATION_DATA_PART_ID_2: {
				memcpy(((uint8_t *)&calibration_data) + sizeof(report_buffer.data), report_buffer.data,
				       sizeof(report_buffer.data));
				break;
			}
			default: {
				PSSENSE_ERROR(pssense, "Unknown calibration data part ID %u", report_buffer.part_id);
				return false;
			}
			}

			uint32_t crc = crc32_le(0, &FEATURE_REPORT_CRC32_SEED, 1);
			crc = crc32_le(crc, (uint8_t *)&report_buffer, sizeof(report_buffer) - 4);
			uint32_t expected_crc = __le32_to_cpu(report_buffer.crc);

			// Only check CRC on Bluetooth, CRC is zeroed out on USB.
			if (crc != expected_crc && !pssense->usb) {
				PSSENSE_WARN(pssense, "Invalid feature report CRC. Expected 0x%08X, actual 0x%08X",
				             expected_crc, crc);
				invalid_crc = true;
			}
		}
	} while (invalid_crc);

	parse_pssense_calibration(pssense, &calibration_data, &pssense->calibration);
	pssense->has_calibration = true;

	PSSENSE_DEBUG(pssense, "Calibration data retrieved and parsed successfully");

	return true;
}

static uint64_t
saturating_add_uint64(uint64_t a, uint64_t b)
{
	if (UINT64_MAX - a < b) {
		return UINT64_MAX;
	} else {
		return a + b;
	}
}

/*
 *
 * Frame node implementations
 *
 */

static void
pssense_node_break_apart(struct xrt_frame_node *node)
{
	struct pssense_device *pssense = from_node(node);

	// Make sure the USB thread is stopped
	os_thread_helper_stop_and_wait(&pssense->controller_thread);
}

static void
pssense_node_destroy(struct xrt_frame_node *node)
{
	struct pssense_device *pssense = from_node(node);

	// Destroy the controller thread
	os_thread_helper_destroy(&pssense->controller_thread);

	// Deinit the precise sleeper
	os_precise_sleeper_deinit(&pssense->sleeper);

	if (pssense->output.pcm_haptics_resampler) {
		u_resampler_destroy(pssense->output.pcm_haptics_resampler);
		pssense->output.pcm_haptics_resampler = NULL;
	}

	m_imu_3dof_close(&pssense->tracking.fusion);

	if (pssense->hid != NULL) {
		h_hid_destroy(pssense->hid);
		pssense->hid = NULL;
	}

	// Relation histories are used from the frame context lifecycle in the constellation tracker device callbacks.
	m_relation_history_destroy(&pssense->tracking.imu_relation_history);

	// LED sync is used on the frame context lifecycle, so it needs to be destroyed in here.
	t_led_sync_refinement_destroy(&pssense->tracking.led_sync_refinement);

	// Remove the variable tracking.
	u_var_remove_root(pssense);

	if (pssense->led_model.leds != NULL) {
		free(pssense->led_model.leds);
		pssense->led_model.leds = NULL;
	}

	// Actually free the pointer
	free(pssense);
}

/*
 *
 * Timing event sink implementation
 *
 */

static void
pssense_timing_event_sink_push(struct t_timing_event_sink *sink, const struct t_timing_event *event)
{
	struct pssense_device *pssense = from_timing_event_sink(sink);

	// We only care about exposure start.
	if (event->type != T_TIMING_EVENT_TYPE_CAMERA_EXPOSURE_START) {
		return;
	}

	struct t_timing_event_camera_exposure_start camera_exposure = event->camera_exposure_start;

	PSSENSE_TRACE(pssense, "Received timing event: %d, seq id: %" PRIu64 ", timestamp: %" PRId64 "ns", event->type,
	              camera_exposure.sequence_id, camera_exposure.timestamp_ns);

	if (pssense->tracking.received_frames++ == 0) {
		pssense->tracking.last_exposure_sequence_id = camera_exposure.sequence_id;
		pssense->tracking.last_exposure_local_timestamp_ns = camera_exposure.timestamp_ns;
		return;
	}

	uint32_t sequence_id_delta = camera_exposure.sequence_id - pssense->tracking.last_exposure_sequence_id;

	time_duration_ns estimated_interval =
	    (camera_exposure.timestamp_ns - pssense->tracking.last_exposure_local_timestamp_ns) / (sequence_id_delta);

	// If available, use the frame period from the event
	if (camera_exposure.frame_period_ns > 0) {
		estimated_interval = camera_exposure.frame_period_ns;
	}

	// If this is the second frame we've received
	if (pssense->tracking.received_frames == 2) {
		pssense->tracking.average_exposure_interval_ns = estimated_interval;
		PSSENSE_TRACE(pssense, "Initial exposure interval: %" PRId64 "ns", estimated_interval);
	} else {
		// Iteratively average out the estimated interval to try to avoid noise
		pssense->tracking.average_exposure_interval_ns =
		    (estimated_interval * 0.1f) + (pssense->tracking.average_exposure_interval_ns * 0.9f);
		PSSENSE_TRACE(pssense, "Updated exposure interval: %" PRId64 "ns",
		              pssense->tracking.average_exposure_interval_ns);
	}

	pssense->tracking.last_exposure_sequence_id = camera_exposure.sequence_id;
	pssense->tracking.last_exposure_local_timestamp_ns = camera_exposure.timestamp_ns;

	if (pssense->tracking.average_exposure_interval_ns > 0) {
		// Update the frame period to the one we're using internally and push the timing event
		struct t_timing_event_camera_exposure_start led_sync_event = event->camera_exposure_start;
		led_sync_event.frame_period_ns = pssense->tracking.average_exposure_interval_ns;
		t_led_sync_push_timing_event(&pssense->tracking.led_sync_refinement, &led_sync_event);
	}

	os_thread_helper_lock(&pssense->controller_thread);

	// update the LED settings
	if (pssense->tracking.received_frames > 10) {
		// Update the sample from the LED sync routine, but only check if we aren't pending a sample.
		if (!pssense->tracking.led_sync_sample_needs_sending &&
		    !pssense->tracking.led_sync_sample_needs_marking &&
		    t_led_sync_get_sample(&pssense->tracking.led_sync_refinement,
		                          &pssense->tracking.latest_led_sync_sample)) {
			pssense->tracking.led_sync_sample_needs_sending = true;
			pssense->tracking.period_id =
			    DURATION_NS_TO_PERIOD_ID(pssense->tracking.latest_led_sync_sample.blink_duration_ns);
			pssense->tracking.led_sequence_num += 1;

			// Converted once here, so rebuilding the settings before they're sent can't lose it.
			pssense_convert_led_sync_delta_locked(pssense);

			switch (pssense->tracking.latest_led_sync_sample.pssense_mode) {
			case T_LED_SYNC_SAMPLE_PSSENSE_MODE_PRESCAN:
				pssense->tracking.sync_phase = LED_SYNC_PHASE_PRESCAN;
				break;
			case T_LED_SYNC_SAMPLE_PSSENSE_MODE_BROAD:
				pssense->tracking.sync_phase = LED_SYNC_PHASE_BROAD;
				break;
			case T_LED_SYNC_SAMPLE_PSSENSE_MODE_BG: pssense->tracking.sync_phase = LED_SYNC_PHASE_BG; break;
			case T_LED_SYNC_SAMPLE_PSSENSE_MODE_STABLE:
				pssense->tracking.sync_phase = LED_SYNC_PHASE_STABLE;
				break;
			case T_LED_SYNC_SAMPLE_PSSENSE_MODE_LED_ALL_OFF:
				pssense->tracking.sync_phase = LED_SYNC_PHASE_LED_ALL_OFF;
				break;
			case T_LED_SYNC_SAMPLE_PSSENSE_MODE_LED_ALL_ON:
				pssense->tracking.sync_phase = LED_SYNC_PHASE_LED_ALL_ON;
				break;
			}
			if (pssense->tracking.latest_led_sync_sample.reset_clock_map) {
				pssense->tracking.led_map_valid = false;
			}
		}

		uint8_t period_id = pssense->tracking.period_id;

		// Once the refinement trims the cycle for the clock skew it owns it, before that use our own
		// estimate.
		double cycle_period_ns = pssense->tracking.latest_led_sync_sample.cycle_period_ns;
		if (cycle_period_ns <= 0.0) {
			cycle_period_ns = (double)pssense->tracking.average_exposure_interval_ns;
		}

		// inside thirds of a nanosecond
		uint32_t cycle_length = (uint32_t)(cycle_period_ns * 3.0 + 0.5);

#if 0
		static int64_t jitter_integration = 0;

		PSSENSE_DEBUG(pssense, "(blink length %uus) drift %ldus",
		              (uint32_t)(IMU_TICKS_TO_NS(150LLU * 32 + 1250) / 1000), jitter_integration / 1000);
		jitter_integration += jitter;

		if (jitter_integration > U_TIME_1S_IN_NS) {
			jitter_integration = 0;
		}
#endif

		if (pssense->tracking.timing_fudge_100us != 0 &&
		    pssense->tracking.led_settings.phase != LED_SYNC_PHASE_PRESCAN) {
			pssense->tracking.led_sync_delta_ticks +=
			    NS_TO_IMU_TICKS((int64_t)pssense->tracking.timing_fudge_100us * 100 * U_TIME_1US_IN_NS);

			pssense->tracking.led_sequence_num += 1;

			// Reset fudge since it stacks
			pssense->tracking.timing_fudge_100us = 0;
		}

		pssense->tracking.led_settings = (struct pssense_led_settings){
		    .phase = pssense->tracking.sync_phase,
		    .cycle_length = __cpu_to_le32(cycle_length),
		    .sequence_number = pssense->tracking.led_sequence_num,
		    .led_blink = {0xFF, 0xFF, 0xFF, 0xFF},
		    .period_id = period_id,
		};

		if (pssense->tracking.led_settings.phase == LED_SYNC_PHASE_PRESCAN) {
			/*
			 * Convert the timestamp, latency offset will be applied within here. Must happen after fetching
			 * the sample above, so the latency and the fudge offset below come from the same sample.
			 */
			timepoint_ns next_blink_time = 0;
			if (!pssense_host_ts_to_device(pssense, pssense->tracking.last_exposure_local_timestamp_ns,
			                               &next_blink_time)) {
				os_thread_helper_unlock(&pssense->controller_thread);
				return;
			}

			if (pssense->tracking.count_calibration) {
				const timepoint_ns host_ns = pssense->tracking.last_exposure_local_timestamp_ns;
				time_duration_ns target_ns = next_blink_time - host_ns;
				time_duration_ns dev_ns = target_ns - pssense->tracking.led_map_offset_ns;
				/*
				 * The mapping is latched once and then only advanced by the drift rate LED sync measures
				 * optically (the sample's clock_skew), never pulled toward the Bluetooth clock estimate,
				 * whose errors (0-330us/s drift or ~0.5ms steps, depending on controller and run) used to
				 * move the LEDs out of the window. Only a jump over 50ms (reconnect) re-latches it.
				 */
				if (!pssense->tracking.led_map_valid || dev_ns > U_TIME_1MS_IN_NS * 50 ||
				    dev_ns < -U_TIME_1MS_IN_NS * 50) {
					// First use, or a reconnect / clock reset.
					PSSENSE_DEBUG(pssense,
					              "LED clock map latched at %" PRId64 "us (%s, %+" PRId64
					              "us from the previous)",
					              (int64_t)(target_ns / U_TIME_1US_IN_NS),
					              pssense->tracking.led_map_valid ? "jump over 50ms" : "first use",
					              (int64_t)((target_ns - pssense->tracking.led_map_offset_ns) /
					                        U_TIME_1US_IN_NS));
					pssense->tracking.led_map_offset_ns = target_ns;
					pssense->tracking.led_map_valid = true;
				} else {
					double skew = pssense->tracking.latest_led_sync_sample.clock_skew;
					pssense->tracking.led_map_offset_ns += (time_duration_ns)(
					    skew * (double)(host_ns - pssense->tracking.led_map_last_host_ns));
					time_duration_ns abs_dev = dev_ns < 0 ? -dev_ns : dev_ns;
					pssense->tracking.led_map_max_dev_ns = MAX(pssense->tracking.led_map_max_dev_ns, abs_dev);
				}
				pssense->tracking.led_map_last_host_ns = host_ns;
				if (host_ns - pssense->tracking.led_map_last_log_ns > U_TIME_1S_IN_NS * 5) {
					PSSENSE_DEBUG(pssense,
					              "LED clock map: Bluetooth tracking wandered up to %" PRId64
					              "us from the held mapping in the last 5s, mapping offset %" PRId64 "us",
					              (int64_t)(pssense->tracking.led_map_max_dev_ns / U_TIME_1US_IN_NS),
					              (int64_t)(pssense->tracking.led_map_offset_ns / U_TIME_1US_IN_NS));
					pssense->tracking.led_map_max_dev_ns = 0;
					pssense->tracking.led_map_last_log_ns = host_ns;
				}
				next_blink_time = host_ns + pssense->tracking.led_map_offset_ns;
			}

			next_blink_time += (int64_t)pssense->tracking.timing_fudge_100us * 100 * U_TIME_1US_IN_NS;
			// Apply the fudge offset, which will line up the blink center with exposure center
			next_blink_time += (int64_t)pssense->tracking.latest_led_sync_sample.fudge_offset_ns;

			// in IMU ticks
			uint32_t cycle_position = NS_TO_IMU_TICKS(next_blink_time);

			pssense->tracking.led_settings.cycle_position_absolute = __cpu_to_le32(cycle_position);
		} else {
			// Only the rebuilds before the sample is sent carry its delta, after that it would stack.
			int32_t delta_ticks = pssense->tracking.led_sync_delta_ticks;
			pssense->tracking.led_settings.cycle_position_delta = __cpu_to_le32((uint32_t)delta_ticks);
		}

		memcpy(pssense->tracking.led_settings.led_blink, pssense->tracking.latest_led_sync_sample.pssense.leds,
		       sizeof(pssense->tracking.led_settings.led_blink));

		for (size_t i = 0; i < ARRAY_SIZE(pssense->tracking.led_settings.led_blink); i++) {
			uint8_t *led = &pssense->tracking.led_settings.led_blink[i];
			if ((*led) == 0xFF) {
				continue;
			}

			// The wire LED IDs are 1-indexed
			(*led)++;
		}

		static_assert(sizeof(pssense->tracking.led_settings.led_blink) ==
		                  sizeof(pssense->tracking.latest_led_sync_sample.pssense.leds),
		              "Size of the sync LEDs doesn't equal wire LEDs.");

		/*
		 * The count calibration and its hold blink in PRESCAN, whose absolute position is rebuilt from every
		 * exposure; a new sequence number makes the controller take it, so it follows the clock tracking. Every
		 * 500ms is plenty now that the clock skew is tracked: doing it every frame kept both controllers sending
		 * at the Bluetooth rate cap.
		 */
		timepoint_ns reanchor_now_ns = os_monotonic_get_ns();
		bool count_reanchor = pssense->tracking.count_calibration &&
		                      pssense->tracking.sync_phase == LED_SYNC_PHASE_PRESCAN &&
		                      reanchor_now_ns - pssense->tracking.last_reanchor_ns >= U_TIME_1MS_IN_NS * 500;
		if (count_reanchor) {
			pssense->tracking.last_reanchor_ns = reanchor_now_ns;
		}
		if (pssense->tracking.increment_sequence_num || count_reanchor) {
			pssense->tracking.led_sequence_num += 1;
#if 0
			PSSENSE_DEBUG(pssense, "%lu\t%lu",
			              (pssense->tracking.last_exposure_local_timestamp_ns %
			               pssense->tracking.average_exposure_interval_ns) /
			                  1000,
			              (next_blink_time % pssense->tracking.average_exposure_interval_ns) / 1000);
#endif
		}
	}
	os_thread_helper_unlock(&pssense->controller_thread);
}

/*
 *
 * Constellation tracker device implementations
 *
 */

/*!
 * The controller's IMU-only orientation (IMU frame, which the LED model is moved into for the tracker) at a host
 * time, for the tracker's gravity and gyro gates. Looked up exactly in the IMU history, without the latest-sample
 * clamp used for display, since the camera exposure is usually 10-30ms older than the newest IMU sample.
 */
static bool
pssense_get_imu_orientation(struct t_constellation_tracker_device *device,
                            int64_t timestamp_ns,
                            struct xrt_quat *out_orientation)
{
	struct pssense_device *pssense = from_constellation_device(device);
	bool ok = false;
	os_thread_helper_lock(&pssense->controller_thread);
	timepoint_ns imu_ts;
	if (pssense_host_ts_to_imu(pssense, timestamp_ns, &imu_ts)) {
		struct xrt_space_relation rel = {0};
		m_relation_history_get(pssense->tracking.imu_relation_history, imu_ts, &rel);
		if ((rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
			// Into the LED model's IMU frame, see PSSENSE_IMU_REF_CORRECTION_DEG_DEFAULT.
			struct xrt_quat corr;
			math_quat_from_angle_vector(DEG_TO_RAD(debug_get_float_option_pssense_imu_ref_correction_deg()),
			                            &(struct xrt_vec3){1, 0, 0}, &corr);
			math_quat_rotate(&rel.pose.orientation, &corr, out_orientation);
			ok = true;
		}
	}
	os_thread_helper_unlock(&pssense->controller_thread);
	return ok;
}

static void
pssense_push_constellation_tracker_sample(struct t_constellation_tracker_device *device,
                                          struct t_constellation_tracker_sample *sample)
{
	struct pssense_device *pssense = from_constellation_device(device);

	// Forward to LED sync
	t_led_sync_push_constellation_sample(&pssense->tracking.led_sync_refinement, sample);

	if (sample->has_world_pose) {
		os_thread_helper_lock(&pssense->controller_thread);
		if (sample->timestamp_ns > pssense->tracking.last_optical_fix_ns) {
			pssense->tracking.last_optical_fix_ns = sample->timestamp_ns;
			pssense->tracking.last_optical_fix_pose = sample->world_pose;
		}
		os_thread_helper_unlock(&pssense->controller_thread);
	}

	/*
	 * For calibrating the headset camera mount offline: the optical pose next to the IMU orientation at the same
	 * time. With the right mount, both agree on which way is down.
	 */
	if (sample->has_world_pose && pssense->log_level <= U_LOGGING_DEBUG) {
		struct xrt_space_relation imu = {0};
		os_thread_helper_lock(&pssense->controller_thread);
		pssense_get_imu_fusion_pose(pssense, sample->timestamp_ns, &imu);
		os_thread_helper_unlock(&pssense->controller_thread);

		const struct xrt_pose *w = &sample->world_pose;
		PSSENSE_DEBUG(pssense,
		              "OPT %" PRId64 " %d %.5f %.5f %.5f %.6f %.6f %.6f %.6f IMU %d %.6f %.6f %.6f %.6f",
		              sample->timestamp_ns, pssense->hand == XRT_HAND_LEFT ? 0 : 1, w->position.x, w->position.y,
		              w->position.z, w->orientation.x, w->orientation.y, w->orientation.z, w->orientation.w,
		              (imu.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0, imu.pose.orientation.x,
		              imu.pose.orientation.y, imu.pose.orientation.z, imu.pose.orientation.w);
	}
}

/*
 *
 * Driver implementations
 *
 */

static void
pssense_device_destroy(struct xrt_device *xdev)
{
	struct pssense_device *pssense = from_device(xdev);

	// Stop the thread helper
	os_thread_helper_stop_and_wait(&pssense->controller_thread);

	// Don't free the pointer or destroy any resources,
	// since they may be needed after device destroy but before node destroy
}

static xrt_result_t
pssense_device_update_inputs(struct xrt_device *xdev)
{
	struct pssense_device *pssense = from_device(xdev);

	// Lock the data.
	os_thread_helper_lock(&pssense->controller_thread);

	timepoint_ns host_update_time_ns;
	if (!pssense_device_ts_to_host(pssense, pssense->timing.latest_device_time_ns, &host_update_time_ns)) {
		host_update_time_ns = pssense->state.timestamp_ns;
	}

	// Update all the inputs to the correct timestamp
	for (uint32_t i = 0; i < ((uint32_t)PSSENSE_INPUT_COUNT); i++) {
		pssense->base.inputs[i].timestamp = (int64_t)host_update_time_ns;
	}
	pssense->base.inputs[PSSENSE_INDEX_PS_CLICK].value.boolean = pssense->state.ps_click;
	pssense->base.inputs[PSSENSE_INDEX_SHARE_CLICK].value.boolean = pssense->state.share_click;
	pssense->base.inputs[PSSENSE_INDEX_OPTIONS_CLICK].value.boolean = pssense->state.options_click;
	pssense->base.inputs[PSSENSE_INDEX_SQUARE_CLICK].value.boolean = pssense->state.square_click;
	pssense->base.inputs[PSSENSE_INDEX_SQUARE_TOUCH].value.boolean = pssense->state.square_touch;
	pssense->base.inputs[PSSENSE_INDEX_TRIANGLE_CLICK].value.boolean = pssense->state.triangle_click;
	pssense->base.inputs[PSSENSE_INDEX_TRIANGLE_TOUCH].value.boolean = pssense->state.triangle_touch;
	pssense->base.inputs[PSSENSE_INDEX_CROSS_CLICK].value.boolean = pssense->state.cross_click;
	pssense->base.inputs[PSSENSE_INDEX_CROSS_TOUCH].value.boolean = pssense->state.cross_touch;
	pssense->base.inputs[PSSENSE_INDEX_CIRCLE_CLICK].value.boolean = pssense->state.circle_click;
	pssense->base.inputs[PSSENSE_INDEX_CIRCLE_TOUCH].value.boolean = pssense->state.circle_touch;
	pssense->base.inputs[PSSENSE_INDEX_SQUEEZE_CLICK].value.boolean = pssense->state.squeeze_click;
	pssense->base.inputs[PSSENSE_INDEX_SQUEEZE_TOUCH].value.boolean = pssense->state.squeeze_touch;
	pssense->base.inputs[PSSENSE_INDEX_SQUEEZE_PROXIMITY].value.boolean = pssense->state.squeeze_proximity > 0.7f;
	pssense->base.inputs[PSSENSE_INDEX_SQUEEZE_PROXIMITY_FLOAT].value.vec1.x = pssense->state.squeeze_proximity;
	pssense->base.inputs[PSSENSE_INDEX_TRIGGER_CLICK].value.boolean = pssense->state.trigger_click;
	pssense->base.inputs[PSSENSE_INDEX_TRIGGER_TOUCH].value.boolean = pssense->state.trigger_touch;
	pssense->base.inputs[PSSENSE_INDEX_TRIGGER_VALUE].value.vec1.x = pssense->state.trigger_value;
	pssense->base.inputs[PSSENSE_INDEX_TRIGGER_PROXIMITY].value.boolean = pssense->state.trigger_proximity > 0.7f;
	pssense->base.inputs[PSSENSE_INDEX_TRIGGER_PROXIMITY_FLOAT].value.vec1.x = pssense->state.trigger_proximity;
	pssense->base.inputs[PSSENSE_INDEX_THUMBSTICK].value.vec2 = pssense->state.thumbstick;
	pssense->base.inputs[PSSENSE_INDEX_THUMBSTICK_CLICK].value.boolean = pssense->state.thumbstick_click;
	pssense->base.inputs[PSSENSE_INDEX_THUMBSTICK_TOUCH].value.boolean = pssense->state.thumbstick_touch;

	// Done now.
	os_thread_helper_unlock(&pssense->controller_thread);

	return XRT_SUCCESS;
}

static xrt_result_t
set_vibration_output(struct pssense_device *pssense,
                     const struct xrt_output_value *value,
                     bool *send_vibration,
                     uint8_t *vibration_amplitude,
                     uint8_t *vibration_mode)
{
	switch (value->type) {
	case XRT_OUTPUT_VALUE_TYPE_VIBRATION: {
		*send_vibration = true;
		*vibration_amplitude = (uint8_t)(value->vibration.amplitude * 255.0f);
		*vibration_mode = OUTPUT_SETTINGS_VIBRATE_MODE_CLASSIC_RUMBLE;

		if (value->vibration.frequency != XRT_FREQUENCY_UNSPECIFIED) {
			if (value->vibration.frequency <= 70) {
				*vibration_mode = OUTPUT_SETTINGS_VIBRATE_MODE_LOW_60HZ;
			} else if (value->vibration.frequency >= 110) {
				*vibration_mode = OUTPUT_SETTINGS_VIBRATE_MODE_HIGH_120HZ;
			}
		}
		break;
	}
	case XRT_OUTPUT_VALUE_TYPE_PCM_VIBRATION: {
		os_thread_helper_lock(&pssense->controller_thread);
		// Reset the resampler if we're not appending.
		if (!value->pcm_vibration.append) {
			u_resampler_reset(pssense->output.pcm_haptics_resampler);
		}

		size_t samples_consumed =
		    u_resampler_write(pssense->output.pcm_haptics_resampler, value->pcm_vibration.buffer,
		                      value->pcm_vibration.buffer_size, value->pcm_vibration.sample_rate);
		if (value->pcm_vibration.sample_rate > 0.0f) {
			// Keep sending at the full rate while these samples (plus some slack) play out.
			timepoint_ns play_ns = (timepoint_ns)((double)value->pcm_vibration.buffer_size /
			                                      value->pcm_vibration.sample_rate * U_TIME_1S_IN_NS);
			pssense->output.pcm_active_until_ns = os_monotonic_get_ns() + play_ns + U_TIME_1MS_IN_NS * 100;
		}
		os_thread_helper_unlock(&pssense->controller_thread);

		*value->pcm_vibration.samples_consumed = samples_consumed;
		break;
	}
	default: {
		U_LOG_XDEV_UNSUPPORTED_OUTPUT(&pssense->base, pssense->log_level, XRT_OUTPUT_NAME_PSSENSE_VIBRATION);
		return XRT_ERROR_OUTPUT_UNSUPPORTED;
		break;
	}
	}

	return XRT_SUCCESS;
}

static xrt_result_t
pssense_set_output(struct xrt_device *xdev, enum xrt_output_name name, const struct xrt_output_value *value)
{
	struct pssense_device *pssense = from_device(xdev);

	bool send_vibration = false;
	uint8_t vibration_amplitude;
	uint8_t vibration_mode;

	bool send_trigger_feedback = false;
	enum pssense_adaptive_trigger_mode trigger_feedback_mode;

	switch (name) {
	case XRT_OUTPUT_NAME_PSSENSE_VIBRATION: {
		xrt_result_t result =
		    set_vibration_output(pssense, value, &send_vibration, &vibration_amplitude, &vibration_mode);
		if (result != XRT_SUCCESS) {
			return result;
		}

		break;
	}
	case XRT_OUTPUT_NAME_PSSENSE_TRIGGER_FEEDBACK: {
		for (uint64_t i = 0; i < value->force_feedback.force_feedback_location_count; i++) {
			if (value->force_feedback.force_feedback[i].location ==
			    XRT_FORCE_FEEDBACK_LOCATION_LEFT_INDEX) {
				send_trigger_feedback = true;
				if (value->force_feedback.force_feedback[i].value > 0) {
					trigger_feedback_mode = TRIGGER_FEEDBACK_MODE_SIMPLE_FEEDBACK;
				} else {
					trigger_feedback_mode = TRIGGER_FEEDBACK_MODE_OFF;
				}
			}
		}

		break;
	}
	default: {
		U_LOG_XDEV_UNSUPPORTED_OUTPUT(&pssense->base, pssense->log_level, name);
		return XRT_ERROR_OUTPUT_UNSUPPORTED;
	}
	}

	timepoint_ns now = os_monotonic_get_ns();

	os_thread_helper_lock(&pssense->controller_thread);
	if (send_vibration && (vibration_amplitude != pssense->output.vibration_amplitude ||
	                       vibration_mode != pssense->output.vibration_mode)) {
		pssense->output.send_vibration = true;
		pssense->output.vibration_amplitude = vibration_amplitude;
		pssense->output.vibration_mode = vibration_mode;
		// Some applications (hello_xr has been seen doing this) will set the duration to INT64_MAX, so when
		// adding directly, it overflows and doesn't work. This prevents that.
		pssense->output.vibration_end_timestamp_ns = saturating_add_uint64(now, value->vibration.duration_ns);
	}

	if (send_trigger_feedback && trigger_feedback_mode != pssense->output.trigger_feedback_mode) {
		pssense->output.send_trigger_feedback = true;
		pssense->output.trigger_feedback_mode = trigger_feedback_mode;
	}
	os_thread_helper_unlock(&pssense->controller_thread);

	return XRT_SUCCESS;
}

xrt_result_t
pssense_get_output_limits(struct xrt_device *xdev, struct xrt_output_limits *limits)
{
	(*limits) = XRT_C11_COMPOUND(struct xrt_output_limits){
	    // PCM data is played back at 3000hz
	    .haptic_pcm_sample_rate = PCM_SAMPLE_RATE,
	};

	return XRT_SUCCESS;
}

static struct xrt_quat
quat_from_x_rot(float x)
{
	return (struct xrt_quat){
	    .x = sinf(x * 0.5f),
	    .y = 0,
	    .z = 0,
	    .w = cosf(x * 0.5f),
	};
}

static xrt_result_t
pssense_get_tracked_pose(struct xrt_device *xdev,
                         enum xrt_input_name name,
                         int64_t at_timestamp_ns,
                         struct xrt_space_relation *out_relation)
{
	struct pssense_device *pssense = from_device(xdev);

	if (name != XRT_INPUT_PSSENSE_AIM_POSE && name != XRT_INPUT_PSSENSE_GRIP_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&pssense->base, pssense->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	struct xrt_relation_chain xrc = {0};

	float sx = pssense->hand == XRT_HAND_LEFT ? 1.0f : -1.0f;
	struct xrt_pose T_steamvrroot_model = {
	    .orientation = quat_from_x_rot(DEG_TO_RAD(-39)),
	    .position = {.x = -sx * .01432f, .y = .007713f, .z = .10399f},
	};
	struct xrt_pose T_steamvrroot_xrgrip = {
	    .orientation = quat_from_x_rot(DEG_TO_RAD(20.6)),
	    .position = {.x = sx * .007, .y = -.00182941, .z = .1019482},
	};
	struct xrt_pose T_steamvrroot_xraim = {
	    .orientation = quat_from_x_rot(DEG_TO_RAD(-39.4)),
	    .position = {.x = sx * .007, .y = -.03894766, .z = .00949694},
	};

	switch (name) {
	case XRT_INPUT_PSSENSE_AIM_POSE: {
		m_relation_chain_push_pose(&xrc, &T_steamvrroot_xraim);
		break;
	}
	case XRT_INPUT_PSSENSE_GRIP_POSE: {
		m_relation_chain_push_pose(&xrc, &T_steamvrroot_xrgrip);
		break;
	}
	default: assert(!"Unreachable");
	}
	m_relation_chain_push_inverted_pose_if_not_identity(&xrc, &T_steamvrroot_model);

#if 0 // Presently unused OpenVR poses, but might be useful if we need to expose more stuff for st/openvr
	struct xrt_pose T_steamvrroot_xrhandmodel = {
	    .orientation = quat_from_x_rot(DEG_TO_RAD(39.4)),
	    .position = {.x = -sx * 11.25, .y = -1.82941, .z = 101.9482},
	};
	struct xrt_pose T_steamvrroot_handgrip = {
	    .orientation = quat_from_x_rot(DEG_TO_RAD(5.037)),
	    .position = {.x = 0, .y = 3, .z = 97},
	};
	struct xrt_pose T_steamvrroot_tip = {
	    .orientation = quat_from_x_rot(DEG_TO_RAD(37.4)),
	    .position = {.x = sx * 16.694, .y = -25.22, .z = 24.687},
	};
#endif

	m_relation_chain_push_inverted_pose_if_not_identity(&xrc, &pssense->tracking.T_led_imu);

	struct xrt_space_relation *rel = m_relation_chain_reserve(&xrc);

	os_thread_helper_lock(&pssense->controller_thread);
	bool disconnected = pssense->disconnected;
	if (disconnected) {
		*rel = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
	} else if (pssense->tracking.use_constellation) {
		pssense_get_constellation_pose(pssense, at_timestamp_ns, rel);
	} else {
		pssense_get_imu_fusion_pose(pssense, at_timestamp_ns, rel);
	}
	os_thread_helper_unlock(&pssense->controller_thread);

	if (disconnected) {
		*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
		return XRT_SUCCESS;
	}

	m_relation_chain_resolve(&xrc, out_relation);

	// What clients actually get (after the fusion override, hold and fallback), ~30Hz.
	if (pssense->log_level <= U_LOGGING_DEBUG && name == XRT_INPUT_PSSENSE_GRIP_POSE &&
	    at_timestamp_ns - pssense->tracking.last_final_log_ns > 33000000) {
		pssense->tracking.last_final_log_ns = at_timestamp_ns;
		const struct xrt_space_relation *r = out_relation;
		PSSENSE_DEBUG(pssense, "FINAL %" PRId64 " %d flags %x pos %.4f %.4f %.4f rot %.4f %.4f %.4f %.4f",
		              at_timestamp_ns, pssense->hand == XRT_HAND_LEFT ? 0 : 1, r->relation_flags,
		              r->pose.position.x, r->pose.position.y, r->pose.position.z, r->pose.orientation.x,
		              r->pose.orientation.y, r->pose.orientation.z, r->pose.orientation.w);
	}

	return XRT_SUCCESS;
}

static xrt_result_t
pssense_get_battery_status(struct xrt_device *xdev, bool *out_present, bool *out_charging, float *out_charge)
{
	struct pssense_device *pssense = from_device(xdev);

	if (!pssense->state.battery_state_valid) {
		*out_present = false;
		return XRT_SUCCESS;
	}

	*out_present = true;
	*out_charging = pssense->state.battery_charging;
	*out_charge = pssense->state.battery_charge_percent;

	return XRT_SUCCESS;
}

/*
 *
 * Exported functions
 *
 */

#define SET_INPUT(NAME) (pssense->base.inputs[PSSENSE_INDEX_##NAME].name = XRT_INPUT_PSSENSE_##NAME)

struct xrt_device *
pssense_create(struct xrt_prober *xp,
               struct xrt_prober_device *xpdev,
               struct xrt_frame_context *xfctx,
               struct t_timing_event_sink **out_timing_sink)
{
	struct h_hid_device *hid = NULL;
	int ret;

	// On USB, we need to use interface 2, rather than 0 like on Bluetooth.
	int iface = xpdev->bus == XRT_BUS_TYPE_USB ? 2 : 0;
	ret = xrt_prober_open_hid_interface(xp, xpdev, iface, &hid);
	if (ret != 0) {
		U_LOG_E("Failed to open HID interface for PlayStation Sense controller!");
		return NULL;
	}

	unsigned char product_name[128];
	ret = xrt_prober_get_string_descriptor( //
	    xp,                                 //
	    xpdev,                              //
	    XRT_PROBER_STRING_PRODUCT,          //
	    product_name,                       //
	    sizeof(product_name));              //
	if (ret <= 0) {
		U_LOG_E("Failed to get product name from Bluetooth device!");
		return NULL;
	}

	enum u_device_alloc_flags flags = U_DEVICE_ALLOC_TRACKING_NONE;
	struct pssense_device *pssense = U_DEVICE_ALLOCATE(struct pssense_device, flags, PSSENSE_INPUT_COUNT, 2);
	PSSENSE_DEBUG(pssense, "PlayStation Sense controller found");

	pssense->node.break_apart = pssense_node_break_apart;
	pssense->node.destroy = pssense_node_destroy;

	pssense->timing_event_sink.push_timing_event = pssense_timing_event_sink_push;

	pssense->constellation_device.push_constellation_tracker_sample = pssense_push_constellation_tracker_sample;
	pssense->constellation_device.get_imu_orientation = pssense_get_imu_orientation;

	pssense->base.name = XRT_DEVICE_PSSENSE;
	snprintf(pssense->base.str, XRT_DEVICE_NAME_LEN, "%s", product_name);

	/*
	 * SteamVR (st/steamvr_drv) registers devices by serial and logs "attempted to add tracked device with no
	 * serial number" for an empty one, and the controllers don't show up. Use the serial string descriptor (the
	 * Bluetooth address over Bluetooth) when there is one, otherwise a stable per-side fallback.
	 */
	unsigned char serial[XRT_DEVICE_NAME_LEN] = {0};
	ret = xrt_prober_get_string_descriptor(xp, xpdev, XRT_PROBER_STRING_SERIAL_NUMBER, serial, sizeof(serial) - 1);
	if (ret > 0 && serial[0] != '\0') {
		snprintf(pssense->base.serial, XRT_DEVICE_NAME_LEN, "%s", (const char *)serial);
	} else {
		snprintf(pssense->base.serial, XRT_DEVICE_NAME_LEN, "PS Sense %s %04x",
		         xpdev->product_id == PSSENSE_PID_LEFT ? "Left" : "Right", xpdev->product_id);
	}

	pssense->base.update_inputs = pssense_device_update_inputs;
	pssense->base.set_output = pssense_set_output;
	pssense->base.get_output_limits = pssense_get_output_limits;
	pssense->base.get_tracked_pose = pssense_get_tracked_pose;
	pssense->base.get_battery_status = pssense_get_battery_status;
	pssense->base.destroy = pssense_device_destroy;

	pssense->base.supported.orientation_tracking = true;
	pssense->base.supported.battery_status = true;

	pssense->usb = xpdev->bus == XRT_BUS_TYPE_USB;

	m_imu_3dof_init(&pssense->tracking.fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);


	m_relation_history_create(&pssense->tracking.imu_relation_history);

	pssense->log_level = debug_get_log_option_pssense_log();
	pssense->hid = hid;

	// Initialize the IMU orientation to be correct
	const float imu_angle_deg = debug_get_float_option_pssense_imu_angle_deg();
	PSSENSE_DEBUG(pssense, "LED -> IMU rotation: %.2f deg about X", imu_angle_deg);
	struct xrt_quat imu_orientation_quat = quat_from_x_rot(DEG_TO_RAD(imu_angle_deg));

	const struct t_constellation_tracker_led *led_model = NULL;
	if (xpdev->product_id == PSSENSE_PID_LEFT) {
		pssense->base.device_type = XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER;
		pssense->hand = XRT_HAND_LEFT;

		pssense->led_model.match_parameters = DEFAULT_MATCH_PARAMETERS;
		led_model = pssense_left_leds;
		pssense->led_model.led_count = ARRAY_SIZE(pssense_left_leds);

		pssense->tracking.T_led_imu = (struct xrt_pose){
		    .orientation = imu_orientation_quat,
		    .position = T_led_imu_left,
		};
	} else if (xpdev->product_id == PSSENSE_PID_RIGHT) {
		pssense->base.device_type = XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;
		pssense->hand = XRT_HAND_RIGHT;

		pssense->led_model.match_parameters = DEFAULT_MATCH_PARAMETERS;
		led_model = pssense_right_leds;
		pssense->led_model.led_count = ARRAY_SIZE(pssense_right_leds);

		pssense->tracking.T_led_imu = (struct xrt_pose){
		    .orientation = imu_orientation_quat,
		    .position = T_led_imu_right,
		};
	} else {
		PSSENSE_ERROR(pssense, "Unable to determine controller type");
		pssense_device_destroy(&pssense->base);
		return NULL;
	}

	pssense->led_model.leds =
	    U_TYPED_ARRAY_CALLOC(struct t_constellation_tracker_led, pssense->led_model.led_count);
	if (pssense->led_model.leds == NULL) {
		PSSENSE_ERROR(pssense, "Failed to allocate memory for LED model.");
		pssense_device_destroy(&pssense->base);
		return NULL;
	}
	memcpy(pssense->led_model.leds, led_model,
	       sizeof(struct t_constellation_tracker_led) * pssense->led_model.led_count);

	// Prevent overflow
	assert(pssense->led_model.led_count == ARRAY_SIZE(pssense_led_emission_offsets));

	struct xrt_pose T_imu_led;
	math_pose_invert(&pssense->tracking.T_led_imu, &T_imu_led);
	for (size_t i = 0; i < pssense->led_model.led_count; i++) {
		struct xrt_vec3 led_position_in_led_space = pssense->led_model.leds[i].position;
		struct xrt_vec3 led_normal_in_led_space = pssense->led_model.leds[i].normal;

		struct xrt_vec3 led_position_in_imu_space;
		struct xrt_vec3 led_normal_in_imu_space;

		// Get the emission offset for this LED
		struct xrt_vec3 emission_offset = pssense_led_emission_offsets[i];
		// Swap for left hand
		if (pssense->hand == XRT_HAND_LEFT) {
			emission_offset.x = -emission_offset.x;
		}
		// Add the emission offset, fixing up an issue with Sony's original model.
		math_vec3_accum(&emission_offset, &led_position_in_led_space);

		math_pose_transform_point(&T_imu_led, &led_position_in_led_space, &led_position_in_imu_space);
		math_quat_rotate_vec3(&T_imu_led.orientation, &led_normal_in_led_space, &led_normal_in_imu_space);

		// Update the position/normal so that it's IMU-relative
		pssense->led_model.leds[i].position = led_position_in_imu_space;
		pssense->led_model.leds[i].normal = led_normal_in_imu_space;
	}

	SET_INPUT(PS_CLICK);
	SET_INPUT(SHARE_CLICK);
	SET_INPUT(OPTIONS_CLICK);
	SET_INPUT(SQUARE_CLICK);
	SET_INPUT(SQUARE_TOUCH);
	SET_INPUT(TRIANGLE_CLICK);
	SET_INPUT(TRIANGLE_TOUCH);
	SET_INPUT(CROSS_CLICK);
	SET_INPUT(CROSS_TOUCH);
	SET_INPUT(CIRCLE_CLICK);
	SET_INPUT(CIRCLE_TOUCH);
	SET_INPUT(SQUEEZE_CLICK);
	SET_INPUT(SQUEEZE_TOUCH);
	SET_INPUT(SQUEEZE_PROXIMITY);
	SET_INPUT(SQUEEZE_PROXIMITY_FLOAT);
	SET_INPUT(TRIGGER_CLICK);
	SET_INPUT(TRIGGER_TOUCH);
	SET_INPUT(TRIGGER_VALUE);
	SET_INPUT(TRIGGER_PROXIMITY);
	SET_INPUT(TRIGGER_PROXIMITY_FLOAT);
	SET_INPUT(THUMBSTICK);
	SET_INPUT(THUMBSTICK_CLICK);
	SET_INPUT(THUMBSTICK_TOUCH);
	SET_INPUT(GRIP_POSE);
	SET_INPUT(AIM_POSE);

	pssense->base.outputs[0].name = XRT_OUTPUT_NAME_PSSENSE_VIBRATION;
	pssense->base.outputs[1].name = XRT_OUTPUT_NAME_PSSENSE_TRIGGER_FEEDBACK;

	os_precise_sleeper_init(&pssense->sleeper);

	pssense->output.pcm_haptics_resampler = u_resampler_create(4000, PCM_SAMPLE_RATE);
	if (pssense->output.pcm_haptics_resampler == NULL) {
		PSSENSE_ERROR(pssense, "Failed to create PCM resampler");
		pssense_device_destroy(&pssense->base);
		return NULL;
	}

	// @todo Once LED blink refinement is fixed, enable that again
	enum t_led_sync_refinement_flags sync_refinement_flags = T_LED_SYNC_REFINEMENT_FLAGS_OPTICAL_DRIVEN_OFFSET | //
	                                                         T_LED_SYNC_REFINEMENT_FLAGS_HAS_LATENCY_CAP |       //
	                                                         T_LED_SYNC_REFINEMENT_FLAGS_PS_SENSE;
	if (!debug_get_bool_option_pssense_advanced_led_sync()) {
		sync_refinement_flags &= ~T_LED_SYNC_REFINEMENT_FLAGS_OPTICAL_DRIVEN_OFFSET;
		sync_refinement_flags &= ~T_LED_SYNC_REFINEMENT_FLAGS_PS_SENSE;

		// Automatically increment the sequence number if we're not doing advanced LED sync.
		pssense->tracking.increment_sequence_num = true;
	} else if (debug_get_bool_option_pssense_count_calibration()) {
		sync_refinement_flags |= T_LED_SYNC_REFINEMENT_FLAGS_COUNT_CALIBRATION;
		pssense->tracking.count_calibration = true;
	}

	// @note We don't do blink duration refinement right now because that needs to eventually adjust the latency
	//       offset as it goes and produces a worse result with the current implementation.
	struct t_led_sync_refinement_options led_sync_refinement_options = {
	    .flags = sync_refinement_flags,
	    .initial_blink_duration_ns = PERIOD_ID_TO_DURATION_NS(30),
	    .min_blink_duration_ns = PERIOD_ID_TO_DURATION_NS(1),
	    .max_blink_duration_ns = PERIOD_ID_TO_DURATION_NS(MAX_PERIOD_ID),
	    .time_to_resync_ns = T_LED_SYNC_DEFAULT_RESYNC_TIME,
	    // Frames after a sample is confirmed applied before measuring. 7 dated from when "applied" was marked
	    // before the controller had the sample; now it's confirmed by the controller's echo (~10ms round trips),
	    // and the count calibration's steps took ~0.35s each, giving the window time to drift.
	    .settle_frames = 3,
	    // 8ms latency cap, something a bit overboard for bluetooth but better than searching the whole range
	    .latency_cap_ns = U_TIME_1MS_IN_NS * 8LL,
	};
	ret = t_led_sync_refinement_init(&pssense->tracking.led_sync_refinement, &led_sync_refinement_options);
	if (ret != 0) {
		PSSENSE_ERROR(pssense, "Failed to init LED sync refinement!");
		pssense_device_destroy(&pssense->base);
		return NULL;
	}

	pssense->tracking.period_id = DURATION_NS_TO_PERIOD_ID(led_sync_refinement_options.initial_blink_duration_ns);
	pssense->tracking.sync_phase = LED_SYNC_PHASE_PRESCAN;

	ret = os_thread_helper_init(&pssense->controller_thread);
	if (ret != 0) {
		PSSENSE_ERROR(pssense, "Failed to init threading!");
		pssense_device_destroy(&pssense->base);
		return NULL;
	}

	ret = os_thread_helper_start(&pssense->controller_thread, pssense_run_thread, pssense);
	if (ret != 0) {
		PSSENSE_ERROR(pssense, "Failed to start thread!");
		pssense_device_destroy(&pssense->base);
		return NULL;
	}

	// Try to set the PC polling rate if the user requested it.
	if (debug_get_bool_option_pssense_pc_polling_rate() && //
	    !pssense_set_pc_polling_rate(pssense)) {
		PSSENSE_ERROR(pssense, "PC polling rate requested, but got error when attempting to apply.");
	}

	/*
	 * Over Bluetooth the feature read can fail transiently (-1 on hardware, for one controller, while both were
	 * connecting), which dropped that controller for the whole session. Retry for a few seconds first.
	 */
	bool got_calibration = false;
	for (int attempt = 0; attempt < 15 && !got_calibration; attempt++) {
		if (attempt > 0) {
			PSSENSE_WARN(pssense, "Retrying calibration read (attempt %d)", attempt + 1);
			os_nanosleep(U_TIME_1MS_IN_NS * 200);
		}
		got_calibration = pssense_get_calibration_data(pssense);
	}
	if (!got_calibration) {
		PSSENSE_ERROR(pssense, "Failed to retrieve calibration data");
		pssense_device_destroy(&pssense->base);
		return NULL;
	}

	u_var_add_root(pssense, pssense->base.str, false);
	u_var_add_log_level(pssense, &pssense->log_level, "Log level");

	u_var_add_gui_header(pssense, &pssense->gui.button_states, "Button States");
	u_var_add_bool(pssense, &pssense->state.ps_click, "PS Click");
	if (pssense->hand == XRT_HAND_LEFT) {
		u_var_add_bool(pssense, &pssense->state.share_click, "Share Click");
		u_var_add_bool(pssense, &pssense->state.square_click, "Square Click");
		u_var_add_bool(pssense, &pssense->state.square_touch, "Square Touch");
		u_var_add_bool(pssense, &pssense->state.triangle_click, "Triangle Click");
		u_var_add_bool(pssense, &pssense->state.triangle_touch, "Triangle Touch");
	} else if (pssense->hand == XRT_HAND_RIGHT) {
		u_var_add_bool(pssense, &pssense->state.options_click, "Options Click");
		u_var_add_bool(pssense, &pssense->state.cross_click, "Cross Click");
		u_var_add_bool(pssense, &pssense->state.cross_touch, "Cross Touch");
		u_var_add_bool(pssense, &pssense->state.circle_click, "Circle Click");
		u_var_add_bool(pssense, &pssense->state.circle_touch, "Circle Touch");
	}
	u_var_add_bool(pssense, &pssense->state.squeeze_click, "Squeeze Click");
	u_var_add_bool(pssense, &pssense->state.squeeze_touch, "Squeeze Touch");
	u_var_add_ro_f32(pssense, &pssense->state.squeeze_proximity, "Squeeze Proximity");
	u_var_add_bool(pssense, &pssense->state.trigger_click, "Trigger Click");
	u_var_add_bool(pssense, &pssense->state.trigger_touch, "Trigger Touch");
	u_var_add_ro_f32(pssense, &pssense->state.trigger_value, "Trigger");
	u_var_add_ro_f32(pssense, &pssense->state.trigger_proximity, "Trigger Proximity");
	u_var_add_ro_f32(pssense, &pssense->state.thumbstick.x, "Thumbstick X");
	u_var_add_ro_f32(pssense, &pssense->state.thumbstick.y, "Thumbstick Y");
	u_var_add_bool(pssense, &pssense->state.thumbstick_click, "Thumbstick Click");
	u_var_add_bool(pssense, &pssense->state.thumbstick_touch, "Thumbstick Touch");

	u_var_add_gui_header(pssense, &pssense->gui.timing, "Timing");
	u_var_add_ro_i64_ns(pssense, &pssense->timing.latest_imu_time_ns, "Latest IMU Time (ns)");
	u_var_add_ro_u64(pssense, &pssense->timing.imu_ticks_total, "Latest IMU Time (ticks)");
	u_var_add_ro_i64_ns(pssense, &pssense->timing.latest_device_time_ns, "Latest Device Time (ns)");
	u_var_add_ro_u64(pssense, &pssense->timing.device_ticks_total, "Latest Device Time (ticks)");

	u_var_add_gui_header(pssense, &pssense->gui.tracking, "Tracking");
	u_var_add_ro_vec3_i32(pssense, &pssense->state.gyro_raw, "Raw Gyro");
	u_var_add_ro_vec3_i32(pssense, &pssense->state.accel_raw, "Raw Accel");
	u_var_add_bool(pssense, &pssense->has_calibration, "Has Calibration");
	u_var_add_ro_vec3_i32(pssense, &pssense->calibration.gyro_bias, "Gyro Bias");
	u_var_add_ro_vec3_f32(pssense, &pssense->calibration.gyro_scale, "Gyro Scale");
	u_var_add_ro_vec3_f32(pssense, &pssense->calibration.accel_bias, "Accel Bias");
	u_var_add_ro_vec3_f32(pssense, &pssense->calibration.accel_scale, "Accel Scale");
	u_var_add_pose(pssense, &pssense->tracking.pose, "Pose");
	m_imu_3dof_add_vars(&pssense->tracking.fusion, pssense, "3dof Fusion");
	u_var_add_ro_u32(pssense, &pssense->tracking.received_frames, "Received Frames");
	u_var_add_ro_u32(pssense, &pssense->tracking.last_exposure_sequence_id, "Last Exposure Sequence ID");
	u_var_add_ro_i64_ns(pssense, &pssense->tracking.last_exposure_local_timestamp_ns,
	                    "Last Exposure Timestamp (ns)");
	u_var_add_ro_i64_ns(pssense, &pssense->tracking.average_exposure_interval_ns, "Average Exposure Interval (ns)");
	u_var_add_bool(pssense, &pssense->tracking.increment_sequence_num, "Increment LED Sequence Number");
	u_var_add_u8(pssense, &pssense->tracking.led_sequence_num, "LED Sequence Number");
	u_var_add_u8(pssense, &pssense->tracking.period_id, "LED Blink Period ID");
	u_var_add_i32(pssense, (int32_t *)&pssense->tracking.sync_phase, "LED sync phase");
	u_var_add_u8(pssense, &pssense->tracking.led_settings.led_blink[0], "LED Blink[0]");
	u_var_add_u8(pssense, &pssense->tracking.led_settings.led_blink[1], "LED Blink[1]");
	u_var_add_i32(pssense, &pssense->tracking.timing_fudge_100us, "Timing Fudge (100us)");
	u_var_add_bool(pssense, &pssense->tracking.use_constellation, "Use Constellation Tracking");

	xrt_frame_context_add(xfctx, &pssense->node);

	if (out_timing_sink != NULL) {
		*out_timing_sink = &pssense->timing_event_sink;
	}

	return &pssense->base;
}

int
pssense_add_to_constellation_tracker(struct xrt_device *xdev, struct t_constellation_tracker *tracker)
{
	struct pssense_device *pssense = from_device(xdev);

	struct t_constellation_tracker_device_params params = {
	    .led_model = pssense->led_model,
	    .imu_calibration = PSSENSE_IMU_CALIBRATION,
	};
	int ret = t_constellation_tracker_add_device(tracker, &params, &pssense->constellation_device,
	                                             &pssense->tracking.constellation_device_id);
	if (ret < 0) {
		PSSENSE_ERROR(pssense, "Failed to add device to constellation tracker: %d", ret);
		return -1;
	} else {
		pssense->tracking.use_constellation = true;
	}

	pssense->tracking.constellation_imu_sink = params.imu_sink;
	pssense->tracking.constellation_tracker = tracker;

	pssense->base.tracking_origin = t_constellation_tracker_get_tracking_origin(tracker);

	return 0;
}

/*!
 * @}
 */
