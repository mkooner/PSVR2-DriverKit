# Calibrated finger gestures

Gestures are rules on Hand Landmarker's joints. Each rule names one or two joint features, with an "on" threshold and
a looser "off" threshold, separately for the left and right hand. The thresholds are fitted on labelled recordings
from the PS VR2 cameras. The driver reads the rules from a JSON config, so recalibrating or adding a gesture needs no
rebuild.

## Where things are

| What | Where |
|---|---|
| Joint features (one definition for the driver, the tests and the tools) | `trial/src/xrt/tracking/hand/mercury/hg_gesture_geometry.hpp`: `gesture::jointFeatures` |
| Features published per frame | `t_hand_gestures.joint_features` (Mercury `compute_hand_gestures`, from the camera with the highest hand presence) |
| Rule engine and state machine | `trial/src/xrt/state_trackers/steamvr_psvr2/ovrn_gesture_rules.hpp` |
| Built-in config (generated) | `ovrn_gesture_rules_default.hpp` |
| Your config (overrides the built-in one) | `$XDG_CONFIG_HOME/monado/psvr2-gestures.json`, or `~/.config/monado/psvr2-gestures.json` |
| Fit, validate, replay, export | `psvr2-test/hands/gesture_fit.py` |
| Spoken prompts for recordings | `psvr2-test/tools/gesture_prompts.py` (`GESTURE_PROMPTS_SET=pinch\|fist\|pointclaw`) |
| Extracted landmarks per recording | `psvr2-test/logs/landmarks/landmarks-<stamp>.log`, plus `replay_to_session_ns.txt` |
| Unit tests | `trial/tests/tests_ovrn_gestures.cpp` |

## Features

The 18 features are, for each finger (index, middle, ring, little):

- the angle at the middle joint (`*_pip`) and the end joint (`*_dip`), in degrees (180 is straight)
- the reach (`*_reach`): tip-to-wrist divided by knuckle-to-wrist (about 1.8 straight, under 1 curled)
- the thumb tip to that fingertip, divided by palm width (`thumb_*`)

plus the thumb's end-joint angle (`thumb_ip`) and the thumb tip to the index knuckle divided by palm width
(`thumb_out`, small when the thumb is tucked in).

## State machine

Each gesture moves through four states:

1. **Idle.**
2. **Candidate**, once every term holds at its "on" threshold.
3. **Active**, once that has lasted `activate_ms`.
4. **Releasing**, when a term fails its "off" threshold. It returns to idle after `release_ms`, or back to active if
   the terms hold again before then.

Other rules:

- Only one gesture is active at a time. A candidate with higher `priority` takes over when its hold time is up.
- When measurements are missing, or the hand presence is below `min_confidence`, every state is kept for `grace_ms`
  and then let go. A hand that is found again starts from idle, with fresh smoothing.
- Features are averaged over `smooth_frames` camera frames.
- `output` sets what an active gesture does: `grip` drives the grip, `block_pinch` stops a click, `none` does nothing.

The engine changes the inputs like this: an enabled `grip` rule decides the fist, an active `block_pinch` rule stops
the pinch, and an enabled `pinch` rule decides the pinch whenever the frame has the joint measurements (or while its
pinch is held through a short dropout). Frames without a measurement fall back to the image pinch detector if
`psvr2_pinch.onnx` is installed, and otherwise to the gesture mapping (`hg_custom_gesture_weights.h`). Every 10 s the
log says which source decided the pinch: `ovrn: left hand pinch source (10 s): formula 97%, image detector 0%,
gesture mapping 3% of ... frames`.

Each state change into or out of active is logged with the feature values and thresholds:
`ovrn: left gesture rule fist: candidate -> active (held long enough); index_reach 0.91 (on <= 1.075, off 1.217)`.
`OVRN_GESTURE_DEBUG=1` logs every state change.

Offline scores on prompted recordings didn't predict how the pinch felt in the headset, either way. Judge gesture
changes with a headset test, and treat the offline numbers as rough guidance.

## Calibrating, or adding a gesture

1. **Record** a labelled session with the prompts, adding a prompt set for a new gesture in `gesture_prompts.py`. The
   label's gesture name becomes the gesture id, for example `middle_pinch`.

   ```
   MONADO_BUILD=.../trial/build HAND_TRACKING=1 HANDLOG=1 RECORD_CAMERA=1 RECORD_CAMERA_HZ=0 GESTURE_PROMPTS=1 GESTURE_PROMPTS_SET=pinch ./launch-steamvr.sh
   ```

   Labels now start when the spoken prompt ends. Recordings made before 2026-11-25 20:30 need
   `retime_labels.py` first, and the result saved as `gesture-labels-<stamp>-retimed.csv`. For newer recordings, copy
   the label file to that name.
2. **Check the labels against the images.** Pull a few zoomed frame sequences across prompt windows with
   `pv2c_frames.py`, and confirm the hand shows what the label says.
3. **Extract the landmarks** with one replay:

   ```
   MERCURY_DETECTION_LOG=out.log hand_sim camera-<stamp>.pv2c <calibration.bin> vrhandlog-<stamp>.csv sim.csv
   grep '^E,' out.log > logs/landmarks/landmarks-<stamp>.log
   ```

   Add `<stamp> <REPLAY_TO_SESSION_NS>` (printed by `hand_sim`) to `logs/landmarks/replay_to_session_ns.txt`.
4. **Fit and validate:**

   ```
   python3 -I gesture_fit.py --write ~/.config/monado/psvr2-gestures.json
   ```

   - Each gesture is fitted on one session (or round of prompts) and scored on another. A gesture is only switched on
     if it reaches frame precision 0.85 and recall 0.7 on data it wasn't fitted on.
   - Every gesture's `status` says what it was validated on, or why it's off. A gesture that doesn't reach the bar is
     written with `enabled: false`.
   - The fitted config goes to your config folder. `--header` regenerates the built-in config instead, which needs a
     rebuild.
5. **Inspect** with the debugging replay:

   ```
   python3 -I gesture_fit.py --replay <stamp> --config ~/.config/monado/psvr2-gestures.json [--hand left] [--export frames.csv]
   ```

   It prints every state change, with the label at that moment and the reason. The export has one row per frame: the
   features, the active gesture and the changes.
6. **Test in the headset.** Restart SteamVR; the log lists every rule and whether it's on.

## What the 2026-11-25 recordings support

The recordings are the benchmark (`164622`) and the pinch session (`195136`), with labels re-timed and checked
against frames.

| Gesture | Left | Right | In the config |
|---|---|---|---|
| Fist | validated on the unseen round: precision 0.94–1.0, recall 0.87–0.89; 0 false activations on the unseen pinch session | precision 0.92–1.0, recall 0.84–0.93; 0 false on the pinch session | on (grip) |
| Point | below the bar | below the bar on one of the two held-out rounds | off |
| Claw | below the bar | below the bar on one of the two held-out rounds | off |
| Index pinch | formula: thumb tip to index tip ≤ 0.558 palm widths (off above 0.583) | ≤ 0.500 (off above 0.518) | **on by default.** The offline scores were poor (separation 0.45–0.63), but in the headset (2026-11-25) it worked best of the pinch options. The image detector "didn't work at all", so it isn't installed. The gesture mapping covers frames without a measurement. The thresholds are the midpoints of the pinch session's pinch and open medians (`configs/pinch-formula.json`, `configs/default.json`) |
| Middle, ring and little finger pinches; two-finger grip; thumbs-up | no recorded examples | no recorded examples | off: needs a recording |
| Open palm | not a rule: what's left when nothing else is active | the same | — |

Limits:

- Fist, point and claw were only recorded in the benchmark, so they're validated across its two rounds of prompts,
  not across sessions.
- One person and one room.
- The activation times are measured from the label's start, which is about 0.5 s after the spoken prompt ends, so
  they understate the true delay from the moment the hand closes.
