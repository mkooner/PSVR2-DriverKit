# Offline analysis tools

All read a test session's log (`logs/steamvr-*.log`, PSSENSE_LOG=debug) and recording (`logs/record-*.csv`).

- `harness/` — runs the driver's camera-chain code with Monado's own math on the calibration block recorded in a
  session (V row, gen-data type 17). Build with `harness/build.sh`; extract the block first (see `variants.py`).
  Arguments: `chain <block.bin> <mount_transpose> <relation_transpose> <invert_rule> <cv_flip>`.
  Validated: `0 1 0 0` reproduces the camera positions the driver logs.
- `variants.py <log> <record> <harness dir>` — re-places every hardware solve with each chain variant and reports
  cross-camera disagreement, where the controllers end up, and camera facing.
- `hold_sim.py` — simulates the count-calibration hold (old vs new) with measured numbers.
