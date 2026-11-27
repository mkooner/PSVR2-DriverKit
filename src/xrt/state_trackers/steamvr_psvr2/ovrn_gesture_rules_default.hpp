// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The built-in gesture config (generated from psvr2-test hands/configs/default.json: hands/gesture_fit.py's
 *         fit, the formula pinch and the formula gestures' params; don't edit): used when
 *         $XDG_CONFIG_HOME/monado/psvr2-gestures.json doesn't exist.
 * @ingroup st_ovrn
 */
#pragma once

namespace ovrn::gest::rules {

inline const char *const kDefaultConfig = R"json({
 "version": 1,
 "source": "psvr2-test hands/gesture_fit.py fit (fist, point, claw, recordings 164622, 195136, labels re-timed) plus the formula pinch (hands/configs/pinch-formula.json), chosen in the headset",
 "smooth_frames": 3,
 "grace_ms": 100,
 "hands": {
  "left": {
   "fist": {
    "enabled": true,
    "output": "grip",
    "priority": 3,
    "min_confidence": 0.5,
    "activate_ms": 66,
    "release_ms": 100,
    "terms": [
     {
      "feature": "index_reach",
      "op": "<=",
      "on": 1.0752,
      "off": 1.2168
     }
    ],
    "validation": {
     "fit 164622 round 1 -> test 164622 round 2": {
      "precision": 0.94,
      "recall": 0.87,
      "prompts_caught": "1/1",
      "false_prompts": "1/6",
      "false_per_min": 5.3,
      "activate_ms": 11.0,
      "release_ms": null,
      "transitions_false": "2/31"
     },
     "fit 164622 round 2 -> test 164622 round 1": {
      "precision": 1.0,
      "recall": 0.89,
      "prompts_caught": "1/1",
      "false_prompts": "0/15",
      "false_per_min": 0.0,
      "activate_ms": 4.0,
      "release_ms": null,
      "transitions_false": "1/31"
     }
    },
    "status": "calibrated: passed precision >= 0.85 and recall >= 0.7 on the unseen round of prompts (only one session has it)"
   },
   "point": {
    "enabled": false,
    "output": "block_pinch",
    "priority": 2,
    "min_confidence": 0.5,
    "activate_ms": 66,
    "release_ms": 100,
    "terms": [
     {
      "feature": "little_dip",
      "op": "<=",
      "on": 139.5604,
      "off": 144.0841
     },
     {
      "feature": "thumb_little",
      "op": "<=",
      "on": 1.0834,
      "off": 1.248
     }
    ],
    "validation": {
     "fit 164622 round 1 -> test 164622 round 2": {
      "precision": 0.0,
      "recall": 0.0,
      "prompts_caught": "0/1",
      "false_prompts": "1/6",
      "false_per_min": 4.7,
      "activate_ms": null,
      "release_ms": null,
      "transitions_false": "1/31"
     },
     "fit 164622 round 2 -> test 164622 round 1": {
      "precision": 0.0,
      "recall": 0.0,
      "prompts_caught": "0/1",
      "false_prompts": "5/15",
      "false_per_min": 6.4,
      "activate_ms": null,
      "release_ms": null,
      "transitions_false": "3/31"
     }
    },
    "status": "unreliable: below precision 0.85 / recall 0.7 on the unseen round of prompts (only one session has it)"
   },
   "claw": {
    "enabled": false,
    "output": "block_pinch",
    "priority": 1,
    "min_confidence": 0.5,
    "activate_ms": 150,
    "release_ms": 100,
    "terms": [
     {
      "feature": "ring_pip",
      "op": ">=",
      "on": 155.3325,
      "off": 155.9238
     },
     {
      "feature": "little_dip",
      "op": "<=",
      "on": 151.4507,
      "off": 152.9649
     }
    ],
    "validation": {
     "fit 164622 round 1 -> test 164622 round 2": {
      "precision": 0.0,
      "recall": 0.0,
      "prompts_caught": "0/1",
      "false_prompts": "0/6",
      "false_per_min": 0.0,
      "activate_ms": null,
      "release_ms": null,
      "transitions_false": "0/31"
     },
     "fit 164622 round 2 -> test 164622 round 1": {
      "precision": 0.0,
      "recall": 0.0,
      "prompts_caught": "0/1",
      "false_prompts": "1/15",
      "false_per_min": 1.3,
      "activate_ms": null,
      "release_ms": null,
      "transitions_false": "3/31"
     }
    },
    "status": "unreliable: below precision 0.85 / recall 0.7 on the unseen round of prompts (only one session has it)"
   },
   "index_pinch": {
    "enabled": true,
    "output": "pinch",
    "priority": 1,
    "min_confidence": 0.5,
    "activate_ms": 33,
    "release_ms": 33,
    "terms": [
     {
      "feature": "thumb_index",
      "op": "<=",
      "on": 0.558,
      "off": 0.583
     }
    ],
    "validation": {
     "fit 164622 -> test 195136": {
      "precision": 0.1,
      "recall": 0.02,
      "prompts_caught": "2/14",
      "false_prompts": "2/20",
      "false_per_min": 5.4,
      "activate_ms": 525.0,
      "release_ms": null,
      "transitions_false": "3/8"
     },
     "fit 195136 -> test 164622": {
      "precision": 0.0,
      "recall": 0.01,
      "prompts_caught": "1/6",
      "false_prompts": "19/30",
      "false_per_min": 14.2,
      "activate_ms": 1668.0,
      "release_ms": 199.0,
      "transitions_false": "7/23"
     }
    },
    "status": "default since the 2026-11-25 headset tests (worked best of the pinch options): thumb tip to index tip <= 0.558 palm widths, off above 0.583 (midpoint of the pinch session's pinch and open medians); the gesture mapping covers frames without a measurement"
   },
   "open": {
    "enabled": false,
    "output": "none",
    "priority": 0,
    "min_confidence": 0.5,
    "activate_ms": 66,
    "release_ms": 66,
    "terms": [],
    "status": "not a rule: the open hand is what's left when no other gesture is active (as a rule it also fired on every other recorded state)"
   },
   "middle_pinch": {
    "enabled": false,
    "output": "none",
    "priority": 0,
    "min_confidence": 0.5,
    "activate_ms": 66,
    "release_ms": 66,
    "terms": [],
    "status": "no recorded examples: needs a recording (gesture_prompts.py)"
   },
   "ring_pinch": {
    "enabled": false,
    "output": "none",
    "priority": 0,
    "min_confidence": 0.5,
    "activate_ms": 66,
    "release_ms": 66,
    "terms": [],
    "status": "no recorded examples: needs a recording (gesture_prompts.py)"
   },
   "little_pinch": {
    "enabled": false,
    "output": "none",
    "priority": 0,
    "min_confidence": 0.5,
    "activate_ms": 66,
    "release_ms": 66,
    "terms": [],
    "status": "no recorded examples: needs a recording (gesture_prompts.py)"
   },
   "two_finger_grip": {
    "enabled": false,
    "output": "none",
    "priority": 0,
    "min_confidence": 0.5,
    "activate_ms": 66,
    "release_ms": 66,
    "terms": [],
    "status": "no recorded examples: needs a recording (gesture_prompts.py)"
   },
   "thumbs_up": {
    "enabled": false,
    "output": "none",
    "priority": 0,
    "min_confidence": 0.5,
    "activate_ms": 66,
    "release_ms": 66,
    "terms": [],
    "status": "no recorded examples: needs a recording (gesture_prompts.py)"
   }
  },
  "right": {
   "fist": {
    "enabled": true,
    "output": "grip",
    "priority": 3,
    "min_confidence": 0.5,
    "activate_ms": 150,
    "release_ms": 100,
    "terms": [
     {
      "feature": "little_reach",
      "op": "<=",
      "on": 0.8964,
      "off": 1.0751
     }
    ],
    "validation": {
     "fit 164622 round 1 -> test 164622 round 2": {
      "precision": 1.0,
      "recall": 0.84,
      "prompts_caught": "1/1",
      "false_prompts": "0/6",
      "false_per_min": 0.0,
      "activate_ms": 14.0,
      "release_ms": null,
      "transitions_false": "1/31"
     },
     "fit 164622 round 2 -> test 164622 round 1": {
      "precision": 0.92,
      "recall": 0.93,
      "prompts_caught": "1/1",
      "false_prompts": "2/15",
      "false_per_min": 2.7,
      "activate_ms": 8.0,
      "release_ms": null,
      "transitions_false": "1/31"
     }
    },
    "status": "calibrated: passed precision >= 0.85 and recall >= 0.7 on the unseen round of prompts (only one session has it)"
   },
   "point": {
    "enabled": false,
    "output": "block_pinch",
    "priority": 2,
    "min_confidence": 0.5,
    "activate_ms": 150,
    "release_ms": 66,
    "terms": [
     {
      "feature": "middle_reach",
      "op": "<=",
      "on": 1.2326,
      "off": 1.3356
     },
     {
      "feature": "thumb_index",
      "op": ">=",
      "on": 0.8788,
      "off": 0.842
     }
    ],
    "validation": {
     "fit 164622 round 1 -> test 164622 round 2": {
      "precision": 0.2,
      "recall": 0.14,
      "prompts_caught": "1/1",
      "false_prompts": "1/6",
      "false_per_min": 4.7,
      "activate_ms": 7.0,
      "release_ms": null,
      "transitions_false": "2/31"
     },
     "fit 164622 round 2 -> test 164622 round 1": {
      "precision": 1.0,
      "recall": 0.87,
      "prompts_caught": "1/1",
      "false_prompts": "0/15",
      "false_per_min": 0.0,
      "activate_ms": 17.0,
      "release_ms": null,
      "transitions_false": "1/31"
     }
    },
    "status": "unreliable: below precision 0.85 / recall 0.7 on the unseen round of prompts (only one session has it)"
   },
   "claw": {
    "enabled": false,
    "output": "block_pinch",
    "priority": 1,
    "min_confidence": 0.5,
    "activate_ms": 150,
    "release_ms": 0,
    "terms": [
     {
      "feature": "ring_dip",
      "op": ">=",
      "on": 169.1629,
      "off": 168.2536
     },
     {
      "feature": "thumb_ip",
      "op": "<=",
      "on": 152.5579,
      "off": 155.3696
     }
    ],
    "validation": {
     "fit 164622 round 1 -> test 164622 round 2": {
      "precision": 1.0,
      "recall": 1.0,
      "prompts_caught": "1/1",
      "false_prompts": "0/6",
      "false_per_min": 0.0,
      "activate_ms": 7.0,
      "release_ms": 2206.0,
      "transitions_false": "0/31"
     },
     "fit 164622 round 2 -> test 164622 round 1": {
      "precision": 0.03,
      "recall": 0.31,
      "prompts_caught": "1/1",
      "false_prompts": "7/15",
      "false_per_min": 31.0,
      "activate_ms": 15.0,
      "release_ms": null,
      "transitions_false": "6/31"
     }
    },
    "status": "unreliable: below precision 0.85 / recall 0.7 on the unseen round of prompts (only one session has it)"
   },
   "index_pinch": {
    "enabled": true,
    "output": "pinch",
    "priority": 1,
    "min_confidence": 0.5,
    "activate_ms": 33,
    "release_ms": 33,
    "terms": [
     {
      "feature": "thumb_index",
      "op": "<=",
      "on": 0.5,
      "off": 0.518
     }
    ],
    "validation": {
     "fit 164622 -> test 195136": {
      "precision": 0.2,
      "recall": 0.08,
      "prompts_caught": "5/14",
      "false_prompts": "7/20",
      "false_per_min": 12.1,
      "activate_ms": 1108.0,
      "release_ms": 1833.0,
      "transitions_false": "1/8"
     },
     "fit 195136 -> test 164622": {
      "precision": 0.13,
      "recall": 0.26,
      "prompts_caught": "4/6",
      "false_prompts": "18/30",
      "false_per_min": 15.4,
      "activate_ms": 12.0,
      "release_ms": 249.0,
      "transitions_false": "9/23"
     }
    },
    "status": "default since the 2026-11-25 headset tests (worked best of the pinch options): thumb tip to index tip <= 0.5 palm widths, off above 0.518 (midpoint of the pinch session's pinch and open medians); the gesture mapping covers frames without a measurement"
   },
   "open": {
    "enabled": false,
    "output": "none",
    "priority": 0,
    "min_confidence": 0.5,
    "activate_ms": 66,
    "release_ms": 66,
    "terms": [],
    "status": "not a rule: the open hand is what's left when no other gesture is active (as a rule it also fired on every other recorded state)"
   },
   "middle_pinch": {
    "enabled": false,
    "output": "none",
    "priority": 0,
    "min_confidence": 0.5,
    "activate_ms": 66,
    "release_ms": 66,
    "terms": [],
    "status": "no recorded examples: needs a recording (gesture_prompts.py)"
   },
   "ring_pinch": {
    "enabled": false,
    "output": "none",
    "priority": 0,
    "min_confidence": 0.5,
    "activate_ms": 66,
    "release_ms": 66,
    "terms": [],
    "status": "no recorded examples: needs a recording (gesture_prompts.py)"
   },
   "little_pinch": {
    "enabled": false,
    "output": "none",
    "priority": 0,
    "min_confidence": 0.5,
    "activate_ms": 66,
    "release_ms": 66,
    "terms": [],
    "status": "no recorded examples: needs a recording (gesture_prompts.py)"
   },
   "two_finger_grip": {
    "enabled": false,
    "output": "none",
    "priority": 0,
    "min_confidence": 0.5,
    "activate_ms": 66,
    "release_ms": 66,
    "terms": [],
    "status": "no recorded examples: needs a recording (gesture_prompts.py)"
   },
   "thumbs_up": {
    "enabled": false,
    "output": "none",
    "priority": 0,
    "min_confidence": 0.5,
    "activate_ms": 66,
    "release_ms": 66,
    "terms": [],
    "status": "no recorded examples: needs a recording (gesture_prompts.py)"
   }
  }
 },
 "params": {
  "left": {
   "palm_angle_deg": 40,
   "pinch_open_middle": 1.2,
   "pinch_open_ring": 1.35,
   "pinch_open_little": 1.6,
   "pinch_closed_middle": 0.4,
   "pinch_closed_ring": 0.45,
   "pinch_closed_little": 0.55,
   "swipe_thumbward_is_left": 0
  },
  "right": {
   "palm_angle_deg": 40,
   "pinch_open_middle": 1.0,
   "pinch_open_ring": 1.15,
   "pinch_open_little": 1.4,
   "pinch_closed_middle": 0.4,
   "pinch_closed_ring": 0.45,
   "pinch_closed_little": 0.55,
   "swipe_thumbward_is_left": 1
  }
 },
 "params_note": "pinch_open_*: the median thumb-to-fingertip distance / palm width of open hands in the 2026-11-25 recordings (164622, 195136) per hand; pinch_closed_*: estimates (no middle / ring / little pinch recorded); tap / swipe defaults (ovrn_microgestures.hpp) gave 0.06 false taps and 0.76 false swipes a minute on those recordings (hands/microgestures.py)"
})json";

} // namespace ovrn::gest::rules
