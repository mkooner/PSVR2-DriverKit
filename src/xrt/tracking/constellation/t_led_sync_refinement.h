// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  A routine to automatically refine latency offsets of LED blink times using constellation samples.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "util/u_time.h"
#include "util/u_logging.h"

#include "os/os_threading.h"

#include "tracking/t_tracking.h"
#include "tracking/t_time_sync.h"
#include "tracking/t_constellation.h"


//! 5 seconds after last visually seen sample to resync
#define T_LED_SYNC_DEFAULT_RESYNC_TIME (time_duration_ns)(U_TIME_1S_IN_NS * 5LL)

#define T_LED_SYNC_SELECTED_LEDS 2

//! How many elements are in the seen LEDs ring buffer
#define T_LED_SYNC_SEEN_LEDS_ACCUMULATION_BUFFER_SIZE 5
//! The amount of bits in the full gray code we're decoding
#define T_LED_SYNC_GRAY_CODE_SIZE_BITS 11
//! 20 frames per sample
#define T_LED_SYNC_BRIGHTNESS_CURVE_NUM_SAMPLES 20

// @todo Remove this once clang-format is updated in CI
// clang-format off
#define T_LED_SYNC_SEEN_LED_BUFFER_INDEX(sequence_id) ((sequence_id) % T_LED_SYNC_SEEN_LEDS_ACCUMULATION_BUFFER_SIZE)
#define T_LED_SYNC_GRAY_CODE_BUFFER_INDEX(camera_index, sequence_id, led_idx)                                          \
	(((((camera_index) * T_LED_SYNC_GRAY_CODE_SIZE_BITS) + ((sequence_id) % T_LED_SYNC_GRAY_CODE_SIZE_BITS)) *     \
	  T_LED_SYNC_SELECTED_LEDS) +                                                                                  \
	 (led_idx))
// clang-format on

//! Flags to control the behavior of the LED sync refinement routine.
enum t_led_sync_refinement_flags
{
	T_LED_SYNC_REFINEMENT_FLAGS_NONE = 0,
	//! Whether to try to optimize the blink duration after finding an offset, to lessen power usage.
	T_LED_SYNC_REFINEMENT_FLAGS_BLINK_DURATION = 1 << 0,
	/*!
	 * Whether to lock in a clock offset and have it be driven entirely by the sync refinement, utilizing the
	 * optical samples to measure clock skew/offset between the device and host, rather than the driver's normal
	 * remote clock synchronization.
	 *
	 * Use this flag when the driver's clock tracking is very noisy and is causing LED sync to be unreliable. But
	 * currently will cause LED sync to eventually drift as clock skew between the devices takes effect.
	 */
	T_LED_SYNC_REFINEMENT_FLAGS_OPTICAL_DRIVEN_OFFSET = 1 << 1,
	/*!
	 * Whether the device has specified a maximum latency cap. This prevents the sync refinement from searching
	 * parts of the exposure that aren't possible, which can cause refinement to sometimes take longer.
	 */
	T_LED_SYNC_REFINEMENT_FLAGS_HAS_LATENCY_CAP = 1 << 2,
	/*!
	 * Whether the device we're syncing is specifically a PS VR2 controller ("pssense"), and so has access to the
	 * extra Gray code (BROAD) and brightness curve integral (BG) syncing modes.
	 */
	T_LED_SYNC_REFINEMENT_FLAGS_PS_SENSE = 1 << 3,
	/*!
	 * PS Sense: find the offset by counting the blobs the cameras see (LEDs all off for a baseline, then binary
	 * search the visibility edges), like PSVR2Toolkit's custom-controller-sync calibration, instead of needing a
	 * solved pose for every step. Needs timing events with a blob count.
	 */
	T_LED_SYNC_REFINEMENT_FLAGS_COUNT_CALIBRATION = 1 << 4,
};

//! Options for the LED sync refinement.
struct t_led_sync_refinement_options
{
	//! The flags to use when refining.
	enum t_led_sync_refinement_flags flags;

	//! How long the LEDs should be initially blinking for, in nanoseconds.
	time_duration_ns initial_blink_duration_ns;

	//! The minimum duration to use for the LED blinks, if using reduction.
	time_duration_ns min_blink_duration_ns;
	//! The maximum duration to use for the LED blinks, if using reduction.
	time_duration_ns max_blink_duration_ns;

	//! Time to wait after the last visually seen sample before resyncing.
	time_duration_ns time_to_resync_ns;
	//! Frames to wait after the sample was applied to let the device settle.
	uint32_t settle_frames;

	//! The maximum latency offset to use, if using a latency cap.
	time_duration_ns latency_cap_ns;
};

enum t_led_sync_sample_timestamp_mode
{
	T_LED_SYNC_SAMPLE_TIMESTAMP_MODE_INVALID = 0,
	//! Used when the LED sync refinement is just computing a basic latency offset.
	T_LED_SYNC_SAMPLE_TIMESTAMP_MODE_DEVICE_HOST_LATENCY = 1,
	//! Used when the LED sync refinement is fully deriving the clock offset, latency included.
	T_LED_SYNC_SAMPLE_TIMESTAMP_MODE_HOST_DEVICE_CLOCK_OFFSET = 2,
};

enum t_led_sync_sample_pssense_mode
{
	//! "PRESCAN", used to scan for an initial offset to then refine.
	T_LED_SYNC_SAMPLE_PSSENSE_MODE_PRESCAN = 0,
	/*!
	 * "BROAD", blink certain LEDs in an 11-frame Gray code to refine where we are in the offset faster than a
	 * binary search can.
	 */
	T_LED_SYNC_SAMPLE_PSSENSE_MODE_BROAD,
	/*!
	 * "BG", blink certain LEDs with a brightness curve over a set of frames, then fit a curve to that measured
	 * data, allowing us to get sub-frame accuracy in terms of where we are.
	 */
	T_LED_SYNC_SAMPLE_PSSENSE_MODE_BG,
	//! "STABLE", all LEDs blink narrowly on the trimmed cycle, holding sync with nothing measured.
	T_LED_SYNC_SAMPLE_PSSENSE_MODE_STABLE,
	//! All LEDs off, for the blob count baseline of the count calibration.
	T_LED_SYNC_SAMPLE_PSSENSE_MODE_LED_ALL_OFF,
	//! All LEDs lit regardless of timing, for the count calibration's visibility check.
	T_LED_SYNC_SAMPLE_PSSENSE_MODE_LED_ALL_ON,
};

struct t_led_sync_sample_pssense_data
{
	//! The base time offset to send to the controller (in nanoseconds).
	time_duration_ns base_time_offset_ns;
	//! The LEDs to enable, or 0xff if we don't have a selection.
	uint8_t leds[4];
};

//! A sample read out from the driver, to pass to the device in question.
struct t_led_sync_sample
{
	union {
		//! The latency offset from the device timestamps to the host.
		time_duration_ns device_host_latency_ns;

		//! The clock offset from the host timestamp to the device.
		time_duration_ns host_device_clock_offset_ns;
	} timestamp;
	enum t_led_sync_sample_timestamp_mode timestamp_mode;

	/*!
	 * Clock skew (device vs host, in ns per ns) and its host-time epoch. Only used in
	 * HOST_DEVICE_CLOCK_OFFSET mode: device_ts = host_ts + offset + skew * (host_ts - epoch).
	 */
	double clock_skew;
	timepoint_ns clock_epoch_ns;

	//! The LED cycle period in device ns, already trimmed for the skew. 0 lets the driver use its own estimate.
	double cycle_period_ns;
	//! Count calibration: re-latch the driver's host -> controller LED mapping from its clock tracking.
	bool reset_clock_map;

	//! The latency offset to apply to fudge the blink to line up as best as possible with the exposure.
	time_duration_ns fudge_offset_ns;
	//! The duration to make the LEDs blink for, in nanoseconds.
	time_duration_ns blink_duration_ns;

	struct t_led_sync_sample_pssense_data pssense;
	enum t_led_sync_sample_pssense_mode pssense_mode;
};

enum t_led_sync_phase
{
	//! The initial phase, where we haven't made any adjustments yet.
	T_LED_SYNC_PHASE_INIT = 0,
	//! Trying to find some offset that gets us in sync at all.
	T_LED_SYNC_PHASE_FIND_INITIAL_OFFSET,
	//! Trying to align the rising edge of the LED blinks with the falling edge of the camera exposure.
	T_LED_SYNC_PHASE_FIND_RIGHT_EDGE,
	//! Trying to align the falling edge of the LED blinks with the rising edge of the camera exposure.
	T_LED_SYNC_PHASE_FIND_LEFT_EDGE,
	//! Trying to decode the Gray codes present in the LEDs we've selected.
	T_LED_SYNC_PHASE_DECODE_GRAY,
	//! Trying to optimize the brightness curve for the correct time offset.
	T_LED_SYNC_PHASE_OPTIMIZE_BRIGHTNESS_CURVE,
	//! We've found an offset, now we can optimize the blink duration to something that keeps it tracking.
	T_LED_SYNC_PHASE_REFINE_BLINK_DURATION,
	//! We've found an offset and optimized the center of the LED blink to the center of the exposure.
	T_LED_SYNC_PHASE_MAINTAIN_OFFSET,
	//! PS Sense only, BG has converged so the LEDs hold sync on the trimmed cycle until BG checks them again.
	T_LED_SYNC_PHASE_STABLE,
	//! PS Sense count calibration, see @ref T_LED_SYNC_REFINEMENT_FLAGS_COUNT_CALIBRATION.
	T_LED_SYNC_PHASE_COUNT_CALIBRATE,
	//! The count calibration found the visibility window, the LEDs blink centred in it every frame.
	T_LED_SYNC_PHASE_COUNT_HOLD,
};

struct t_led_sync_seen_led
{
	//! How many times it's been seen
	uint8_t accum_seen_count;
	//! How "facing" the camera it was
	float accum_facing_dot;
};

//! One camera's view of the selected LEDs in one frame, brightness is the blob's greysum, 0 if it wasn't seen.
struct t_led_sync_frame
{
	uint64_t sequence_id;
	float brightness[T_LED_SYNC_SELECTED_LEDS];
	float facing_dot[T_LED_SYNC_SELECTED_LEDS];
};

//! Frame period measured over a long run of frames, so it's steady enough to base the LED cycle on.
struct t_led_sync_period_estimate
{
	uint64_t anchor_sequence_id;
	timepoint_ns anchor_ns;
	//! The previous run's estimate, the next one has to agree with it to be trusted.
	double candidate_ns;
	//! The last trusted estimate, 0 until there is one.
	double period_ns;
};

//! Clock sync from BROAD on: the LED cycle, the clock model kept on it, and BG's offset servo and skew fit.
struct t_led_sync_delta_sync
{
	//! The camera frame the clock model last matched the LED cycle at, its drift is measured from here.
	uint64_t anchor_sequence_id;
	timepoint_ns anchor_ns;

	//! Host time the published offset is valid at, the skew extrapolates from here.
	timepoint_ns epoch_ns;
	//! Estimated clock skew, device vs host, in ns per ns.
	double skew;
	//! Sum of all servo corrections this BG session.
	time_duration_ns servo_integral_ns;
	//! Smoothed offset of the LEDs from the exposure, as the servo correction that would center them.
	double led_error_ns;
	bool has_led_error;

	//! The frame period the LED cycle is trimmed from, set when BROAD starts and moved onto each trusted estimate.
	double base_period_ns;
	//! The skew changed and the new cycle still has to be sent.
	bool trim_pending;
	//! Host time of the last accepted measurement, for the no-data watchdog.
	timepoint_ns last_measurement_ns;

	//! Skew windows fitted since BG was entered.
	uint32_t num_windows;
	//! When the fitted rate last started staying under the STABLE gate, 0 if it isn't.
	timepoint_ns converged_since_ns;
	//! Whether this sync has reached STABLE, BG comes back from it warm.
	bool been_stable;
	//! Host time STABLE was entered.
	timepoint_ns stable_entry_ns;

	//! Least-squares window of the phase deviation against time.
	uint32_t num_samples;
	timepoint_ns window_start_ns;
	double sum_t, sum_tt, sum_d, sum_td;
};

struct t_led_sync_refinement
{
	//! Whether the structure has been fully initialized.
	bool initialized;

	//! Which device this is for the log ("left", "right"), set by the driver after init; empty: unnamed.
	char name[16];

	enum u_logging_level log_level;

	//! The options to use for refinement
	struct t_led_sync_refinement_options options;

	struct os_mutex lock;

	/*
	 * Exposure Info
	 */

	/*!
	 * Whether we know the actual exposure time of the frames. We may not, so we have to work without it in some
	 * cases.
	 */
	bool has_exposure_time;
	//! The known time the frame was exposed for.
	time_duration_ns exposure_time_ns;
	//! The known interval between frames.
	time_duration_ns exposure_interval_ns;
	struct t_led_sync_period_estimate period_estimate;
	//! The latest timing event's frame and exposure time.
	uint64_t last_exposure_sequence_id;
	//! Smoothed exposure time of that frame, see @ref exposure_timeline.
	timepoint_ns last_exposure_ns;

	/*!
	 * Exposure timestamps smoothed over frames. The cameras run on a fixed hardware cadence, but the timestamps
	 * reaching us jitter (PS VR2: re-anchors measured +-170us on average over ~11 frames, up to 1.5ms, with random
	 * sign), and the delta sync re-anchor moved the LEDs by that jitter every 10 frames.
	 */
	struct
	{
		bool valid;
		uint64_t sequence_id;
		double exposure_ns;
		double period_ns;
		//! Residual stats (raw - predicted), logged every few seconds.
		double residual_sq_sum, residual_max_abs;
		uint32_t residual_count, resets;
		timepoint_ns last_log_ns;
	} exposure_timeline;

	//! Count calibration state, see @ref T_LED_SYNC_REFINEMENT_FLAGS_COUNT_CALIBRATION.
	struct
	{
		int state;
		bool owns_token;
		//! Latest blob count from a timing event, and its frame.
		bool has_count;
		uint32_t last_count;
		uint64_t count_sequence_id;
		//! Counts accumulated since the last LED change settled.
		uint32_t frames, sum;
		//! Sum of squared counts this phase, for the frame-to-frame noise.
		uint64_t sum_sq;
		//! Variance of the phase means' difference, from frames without an LED change (see COUNT_NOISE_SIGMAS).
		float noise_var;
		//! The on-phase's frame variance, until the off phase completes.
		float on_var;
		//! Frame-to-frame count variance (blobs^2), and the frames each phase counts because of it.
		float frame_var;
		uint32_t phase_frames;
		//! Repeats of the current step, and their summed on - off differences.
		uint32_t repeats;
		float repeat_sum;
		/*!
		 * The tracker's view of the step: the sequence IDs of the frames counted with the LEDs at the test offset,
		 * and the most of this controller's blobs the tracker matched in one of them (see COUNT_TRACKER_MIN_LEDS).
		 */
		uint64_t on_from_seq, on_to_seq;
		uint32_t trk_leds_max;
		uint64_t last_counted_sequence_id;
		float baseline, threshold;
		//! Each step measures the LEDs at the offset, then off, and compares the two.
		bool off_phase;
		float on_mean;
		//! Local re-searches since the last full restart.
		uint32_t local_retries;
		//! Gave the token to the other controller's hold probe mid calibration, resume this step after.
		bool paused;

		/*!
		 * Free-running LED cycle: the blink is placed once with an absolute (clock tracking based) position,
		 * then only moved by relative deltas, so the Bluetooth clock estimate no longer moves the LEDs. Its
		 * drift against the camera is measured optically and trimmed out of the cycle period.
		 */
		bool anchored;
		//! The offset the controller's cycle currently has, relative to the anchor.
		time_duration_ns applied_ns;
		//! Cycle period trim (fraction), kept across recalibrations.
		double skew;
		//! The previous left edge found, for measuring the drift between searches.
		bool have_prev_left;
		time_duration_ns prev_left_ns;
		timepoint_ns prev_left_time_ns;

		/*!
		 * Keeping the sweep away from the headset's own (SLAM) exposures, which on PS VR2 are timestamped ~5.4ms
		 * after the controller exposure: a full-frame sweep lights the LEDs across them, and head tracking got
		 * worse (more snaps) with repeated sweeps. Sweeps start just before the last known window and stop
		 * after one frame, and a sweep that found nothing waits a while with the LEDs off before the next.
		 */
		bool have_known_left;
		time_duration_ns known_left_ns;
		time_duration_ns swept_ns;
		timepoint_ns retry_after_ns;

		//! When this calibration started (from the token), for the time limit.
		timepoint_ns started_ns;
		//! Sweeps in a row that saw nothing.
		uint32_t empty_sweeps;
		//! The current edge search has read "off" at least once; the left one's result.
		bool edge_seen_off, left_seen_off;
		//! The right edge search read "on" at least once (else the controller was out of view during it).
		bool edge_seen_on;
		//! A failed confirmation is being read again before searching again.
		bool confirm_reread;
		//! The latest window with a plausible width, from any pass of this calibration.
		bool have_plausible;
		time_duration_ns plausible_left_ns, plausible_right_ns;
		//! The edges of this pass are plausible, a failed confirmation can accept them.
		bool pass_plausible;
		//! Largest on - off blob count difference read in this pass, for the quick acceptance.
		float pass_max_delta;
		//! This calibration follows a hold that lost the LEDs: finish on plausible edges without confirming.
		bool quick;

		//! Hold: the centre, measured window, probing state and drift correction.
		time_duration_ns hold_centre_ns, hold_window_ns;
		int hold_sub;
		float hold_left_mean, hold_centre_mean, hold_off_mean, hold_right_mean_tmp;
		/*!
		 * Drift estimate over the hold: the phase applied since it started is the probe corrections plus the
		 * integral of the cycle trim, and the rate is that over the elapsed time (see handle_count_hold).
		 */
		timepoint_ns hold_start_ns, hold_trim_last_ns;
		double hold_corrections_sum_ns, hold_trim_integral_ns;
		uint32_t hold_unseen_probes;
		//! The hold has seen this controller since its calibration (the window is confirmed by the tracker).
		bool hold_seen_since_calib;
		//! When the current run of unseen probes started.
		timepoint_ns hold_unseen_since_ns;
		//! Adaptive probe interval: lengthened while the probes keep finding the blink centred.
		time_duration_ns hold_interval_ns;
		uint32_t hold_centred_streak;
		//! The first LED timing found was logged as a startup milestone (u_startup_mark).
		bool startup_marked;
		/*!
		 * Bluetooth clock follow (see COUNT_HOLD_BT_*): the driver's host -> controller clock offset, smoothed;
		 * its value and the held centre at the last probe that found the blink centred; and a move made from it
		 * that the next probe has to confirm (the centre before it, to go back to). bt_trim_ref_ns: the drift trim
		 * applied up to the reference (hold_trim_integral_ns then), which already moved the LEDs with the clock.
		 */
		bool bt_valid, bt_have_ref, bt_jump_pending;
		double bt_smooth_ns;
		double bt_ref_ns;
		double bt_trim_ref_ns;
		time_duration_ns bt_centre_ref_ns, bt_pre_jump_centre_ns;
		//! The offset and the trim when the pending move was made: the reference once the next probe confirms it.
		double bt_jump_smooth_ns, bt_jump_trim_ns;
		uint32_t bt_jumps, bt_jumps_undone;
		//! A drift estimate over COUNT_HOLD_DRIFT_MAX and when it was made, which a later one has to agree with.
		double hold_fast_rate;
		timepoint_ns hold_fast_rate_ns;
		bool hold_have_fast_rate;
		/*!
		 * High-drift detection (see COUNT_HOLD_FAST_*): all centre moves over the hold (steps, Bluetooth moves,
		 * undos), the current 10 s stretch's start and its value then, the last stretch's rate, and whether high
		 * drift was seen (kept for the session: the clock causing it doesn't change with a recalibration).
		 */
		double hold_moves_sum_ns;
		timepoint_ns hold_seg_start_ns;
		double hold_seg_start_phase_ns;
		double hold_seg_prev_rate;
		bool hold_have_seg_rate;
		bool high_drift;
		//! Nudge check: brightness of this device's matched LEDs from the tracker, per side.
		bool hold_measuring;
		uint64_t hold_meas_from_seq;
		timepoint_ns hold_meas_start_ns;
		double hold_bright_sum;
		uint32_t hold_bright_n;
		float hold_minus_mean;
		uint32_t hold_minus_n;
		//! The last probe saw the controller, drift is only applied then.
		bool hold_in_view;
		timepoint_ns hold_last_probe_ns, hold_last_apply_ns;
		double hold_drift_ns_per_s;
		uint32_t hold_corrections;
		time_duration_ns full_window_ns, offset_ns, lower_ns, upper_ns, left_ns, right_ns;
		uint32_t attempts;
	} count_calib;

	/*
	 * Driver Sample
	 */

	//! Whether we have a sample ready to be sent to the driver/device.
	bool has_sample_for_driver;
	//! The sample to be sent to the driver/device on it's next convenience.
	struct t_led_sync_sample sample_for_driver;
	//! Whether the sample has been applied
	bool sample_applied;

	/*
	 * State
	 */

	//! The current search phase
	enum t_led_sync_phase phase;

	/*!
	 * The current estimated latency offset between the device and the host, in nanoseconds. This is what we are
	 * trying to refine.
	 */
	time_duration_ns current_clock_offset_ns;
	/*!
	 * The amount of fudge between the latency offset and the actual blink start time, in nanoseconds, so we can
	 * place the blink at the optimal time for the exposure.
	 */
	time_duration_ns current_blink_fudge_ns;

	//! How long the LEDs blink for, each frame.
	time_duration_ns current_blink_duration_ns;

	//! The amount of frames since the controller was last visually seen.
	uint32_t frames_since_last_visually_seen;

	//! The sequence ID of the latest timing event we processed.
	uint64_t current_sequence_id;

	//! The time the latest sample was applied to the driver/device
	timepoint_ns last_sample_apply_time_ns;

	//! The found left edge of the exposure, -1 if not found yet.
	time_duration_ns found_left_edge_ns;
	//! The found right edge of the exposure, -1 if not found yet.
	time_duration_ns found_right_edge_ns;

	struct
	{
		//! The current left bound of the binary search, if we're in a find edge phase.
		time_duration_ns left_bound_ns;
		//! The current right bound of the binary search, if we're in a find edge phase.
		time_duration_ns right_bound_ns;
	} binary_search_state;

	struct
	{
		//! The index into the seen LEDs ring buffer
		uint32_t cur_sequence_id;

		//! The time when we last selected the set of LEDs
		timepoint_ns last_selection_ns;

		uint32_t num_samples;

		/*!
		 * The seen LEDs ring buffer.
		 *
		 * Indexed via `(leds_seen_idx * XRT_CONSTELLATION_MAX_LEDS_PER_DEVICE) + led_id`
		 */
		struct t_led_sync_seen_led
		    buf[XRT_CONSTELLATION_MAX_LEDS_PER_DEVICE * T_LED_SYNC_SEEN_LEDS_ACCUMULATION_BUFFER_SIZE];
	} leds_seen;

	struct
	{
		//! Whether or not we have applied the final cycle position or not.
		bool applied_gray_start;

		/*!
		 * A ring buffer storing the last @ref T_LED_SYNC_GRAY_CODE_SIZE_BITS bits of the brightness sequence
		 * per camera.
		 *
		 * Indexed via @ref T_LED_SYNC_GRAY_CODE_BUFFER_INDEX
		 */
		float
		    led_brightnesses[T_LED_SYNC_GRAY_CODE_SIZE_BITS * XRT_TRACKING_MAX_CAMS * T_LED_SYNC_SELECTED_LEDS];
		/*!
		 * A ring buffer storing the last @ref T_LED_SYNC_GRAY_CODE_SIZE_BITS bits of the sequence, storing the
		 * facing directions of each sample, so that we can pick the value from the camera that saw the LED the
		 * best.
		 *
		 * Indexed via @ref T_LED_SYNC_GRAY_CODE_BUFFER_INDEX
		 */
		float
		    led_facing_dots[T_LED_SYNC_GRAY_CODE_SIZE_BITS * XRT_TRACKING_MAX_CAMS * T_LED_SYNC_SELECTED_LEDS];

		//! How many decodes we've had that match in a row, see @ref GRAY_CODE_MATCHES_NEEDED
		uint32_t matching_decodes;
		//! The gray code we're trying
		uint8_t decoded_gray_code;
		//! The last time we had an accepted gray code (or the start of this phase).
		timepoint_ns last_accepted_gray_code_ns;
		//! When the current decode attempt started, either entering the phase or the last recovery jump.
		timepoint_ns attempt_start_ns;

		//! An accepted jump that couldn't be queued yet (another LED command was still waiting), retried.
		bool has_pending_jump;
		time_duration_ns pending_jump_ns;
		//! A jump was queued, waiting for the driver to report it applied.
		bool awaiting_jump_apply;
		//! Don't decode until this sequence id, so the code buffer only holds frames from after the last jump.
		uint64_t settle_until_sequence_id;
		//! Brightness per bit (oldest first) and decoded value per selected LED, of the last decode, for logs.
		float last_brightnesses[T_LED_SYNC_SELECTED_LEDS][T_LED_SYNC_GRAY_CODE_SIZE_BITS];
		uint16_t last_decoded_values[T_LED_SYNC_SELECTED_LEDS];
		//! Jump bookkeeping, logged with each accepted code.
		uint32_t jumps_sent, jumps_delayed_ticks, jumps_applied;
	} decode_gray;

	struct
	{
		//! The LEDs @ref frames were recorded for.
		uint8_t leds[T_LED_SYNC_SELECTED_LEDS];
		//! The first frame the next measurement may use.
		uint64_t window_start_sequence_id;

		//! Why BG windows did or didn't produce a measurement, summarised every couple of seconds.
		struct
		{
			uint32_t samples, incomplete, no_cycle, no_measure, measured, corrections;
			float error_steps_sum;
			timepoint_ns last_log_ns;
		} stats;
		//! Ring buffer of the last @ref T_LED_SYNC_BRIGHTNESS_CURVE_NUM_SAMPLES frames per camera.
		struct t_led_sync_frame frames[XRT_TRACKING_MAX_CAMS][T_LED_SYNC_BRIGHTNESS_CURVE_NUM_SAMPLES];
	} optimize_brightness_curve;

	struct t_led_sync_delta_sync delta_sync;

	struct
	{
		//! The last blink duration that didn't cause the device to become unstable.
		time_duration_ns last_good_blink_duration_ns;

		//! Whether we're currently trying to back off a lower blink duration.
		bool backing_off;
	} blink_time_refinement_state;

	struct
	{
		//! The latest estimated host->device clock offset from the driver.
		time_duration_ns driver_host_device_clock_offset_ns;

		bool has_saved_offset;
		time_duration_ns saved_offset;
	} optical_driven_offset;
};

int
t_led_sync_refinement_init(struct t_led_sync_refinement *refinement,
                           const struct t_led_sync_refinement_options *options);

void
t_led_sync_refinement_destroy(struct t_led_sync_refinement *refinement);

/*!
 * Back to the starting state (as just after init), keeping the options and the mutex: for a controller that
 * disconnected or reconnected. Gives the count calibration turn back. Takes the refinement's lock.
 */
void
t_led_sync_refinement_restart(struct t_led_sync_refinement *refinement);

void
t_led_sync_push_timing_event(struct t_led_sync_refinement *refinement,
                             const struct t_timing_event_camera_exposure_start *event);

void
t_led_sync_push_constellation_sample(struct t_led_sync_refinement *refinement,
                                     const struct t_constellation_tracker_sample *sample);

bool
t_led_sync_get_sample(struct t_led_sync_refinement *refinement, struct t_led_sync_sample *out_sample);

/*!
 * True while the LED timing is locked (count hold, or STABLE in the other modes), false while it is being searched for,
 * when the cameras only catch the LEDs now and then as the timing sweeps across the exposure.
 */
bool
t_led_sync_is_locked(struct t_led_sync_refinement *refinement);

/*!
 * Whether the device's LEDs may be in the camera pictures now: locked, or calibrating with them lit at a test offset
 * (the tracker's view of those frames is part of the calibration, see COUNT_TRACKER_MIN_LEDS).
 */
bool
t_led_sync_leds_may_be_lit(struct t_led_sync_refinement *refinement);

void
t_led_sync_mark_latest_sample_applied(struct t_led_sync_refinement *refinement, timepoint_ns apply_time_ns);

/*!
 * Pushes a clock offset to the refinement routine, this is the offset *from* the host *to* the device.
 *
 * To convert a timestamp from host -> device domain, you would do:
 * `device_timestamp = host_timestamp + clock_offset_ns`
 */
void
t_led_sync_push_host_device_clock_offset(struct t_led_sync_refinement *refinement, time_duration_ns clock_offset_ns);
