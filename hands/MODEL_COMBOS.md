# Hand model combinations: what has been tried

Benchmark: the 2026-11-25 00:26 session (`logs/camera-20261125-002624.pv2c`, full 60 fps, 58 spoken prompts,
`vrhandlog` of what SteamVR got). Tool: `hand_sim` (the real tracker + the real SteamVR plugin hand code) and
`score_prompts.py` (per prompt: pinch -> trigger, fist -> grip, dashboard gesture -> system button, false presses).
The simulator matches the real session per prompt (full model: identical on the left hand, within run-to-run noise on
the right), so a combination's score here is what it would score in the headset on the same movements.

Two choices make a combination:

- **Finger points for the 3D hand** (what Mercury's hand fitting uses): MediaPipe full, MediaPipe light, Mercury's own
  keypoint model (`grayscale_keypoint_new.onnx`), or one view MediaPipe and the other Mercury.
- **Gestures** (pinch, fist, open palm): MediaPipe measurements (pinch distance, Gesture Recognizer categories) from a
  MediaPipe model, or the joint rules on the fitted 3D hand (`ovrn_hand_gestures.hpp`, no MediaPipe).

| # | 3D hand from | Gestures from | Needs | Status |
|---|---|---|---|---|
| A | MediaPipe full | MediaPipe full | `PSVR2_HT_PROFILE=mediapipe` | tried (the default until 2026-11-25) |
| B | MediaPipe light | MediaPipe light | light model file | tried: rejected in the headset (left hand almost never found) |
| C | Mercury's model | joint rules | `MERCURY_MEDIAPIPE=0` | not yet |
| D | MediaPipe full | joint rules | `MERCURY_MEDIAPIPE_GESTURES=0` | not yet |
| E | MediaPipe light | joint rules | light model, `MERCURY_MEDIAPIPE_GESTURES=0` | not yet |
| F | Mercury's model | MediaPipe full | `MERCURY_MEDIAPIPE_FIT=0` (the "mercury" profile) | default since 2026-11-25; benchmark: tracking L 99% R 98% (hands in view), pinch 5/6 5/6, fist 2/2 2/2 |
| G | Mercury's model | MediaPipe light | code: as F | not yet |
| H | MediaPipe light | MediaPipe full | code: two landmark models | not yet |
| I | MediaPipe one view + Mercury other view | MediaPipe | code: per-view choice | not yet |
| J | older MediaPipe landmark (`hand_landmark_MEDIAPIPE.onnx`, 3 outputs) | MediaPipe | code: no world-landmark output | not yet |

Scores go in the table below as each one is run (same rules and thresholds for all; thresholds were tuned on A).

## Scores

| # | Pinch L | Pinch R | Fist L | Fist R | False trigger L / R (of 21) | False grip L / R (of 25) | Dashboard | ms per frame (idle / 8 busy cores) |
|---|---|---|---|---|---|---|---|---|
| real (A) | 6/6 | 6/6 | 2/2 | 2/2 | 11 / 11 | 3 / 6 | | |
| A | 6/6 | 5-6/6 | 2/2 | 2/2 | 10-11 / 7-12 | 3 / 4-7 | | 13.6 / 15.9 |
| B | 5/6 | 3/6 | 2/2 | 2/2 | 10 / 6 | 3 / 6 | | 9.7 / 10.8 |
| F (mercury profile) | 5/6 | 4-6/6 | 2/2 | 2/2 | 9-10 / 7-9 | 3 / 4-5 | 4/4 (new gesture, 3 runs) | |

Note: the 30 fps recordings (20:50, 21:47, 23:37) gave misleading tracking numbers in replay and were deleted;
only the full-rate benchmark is used.
