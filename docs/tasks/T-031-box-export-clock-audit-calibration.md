# T-031: box stages 1 and 2: export every session, clock audit, calibration and extrinsics
Status: blocked (T-015, T-005, T-006, T-010, T-011, T-008, T-027, T-028, T-029, T-030)
Type: job
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-031-box-stage1-2
Depends on: T-005, T-006, T-008, T-010, T-011, T-015, T-017, T-027, and the recorded sessions T-025 to T-030
Design: "Offline analysis", "Calibration and time sync"; "Data model" (`dataset/`, `calib/`)

## Goal
Run the export and calibration stages on the real bags, check the rosbag2 adapter against real data, and produce
`calib/camera.yaml`, `calib/extrinsics.yaml` and `clock_audit.csv` for every session.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC1 (rates, sizes from real bags), AC4 (clock facts for time matching), AC5/AC8 inputs (K, height, tilt), open questions 8 and 9.

## Files expected to change (max 2 source files + their tests)
- None in the repo expected. Output: `docs/test-reports/camera-spike-stage1-2.md` (new). Datasets and calib go in the data path
  that T-023 cleared. A parser fix needed here is a separate code ticket.

## Approach (step by step)
1. Job rules: box, light-to-moderate CPU; ask the owner first; one job at a time; `nice`; not alongside ComfyUI or a gpuq job. No GPU.
   Read-only on the bags (copy checksummed on arrival; never modified).
2. For each session bag: `python3 -m camera_spike export` (T-015) into `dataset/<session>/`; verify the counts against `ros2 bag info`
   (if the box has it) and the manifest. A corrupt split is read with `Rosbag2Source`; the `Bad` count is reported.
3. Clock audit (T-005) per topic per session: header-stamp base offset (does `camera_node` stamp sensor time or `now()`?), jitter,
   gaps. If an offset is constant it may be corrected; otherwise those topics are excluded from timing claims (design).
4. Calibrate (T-011) from the S2 compressed frames; compare with the S0/S2 raw clip (JPEG effect); `prior_check` on fx; report RMS,
   subset spread, n.
5. Extrinsics: floor-plane fit (T-008) on the S2 floor board frames and later on VO points; pitch/roll against the 4.5 deg from
   `fit_camera_tilt.py`; height against the S0 ruler (disagreement reported, not averaged); yaw from S5 straight legs vs lidar/IMU
   heading (spread over 5 legs). Time offsets camera-IMU and camera-Cartographer with T-010 (peak lag and width; "not established"
   is a valid result).
6. Fit the stillness threshold on S3 (T-006 `fit_threshold`); write it to `calib/stillness.yaml` with the fitting data.
7. Report: every number with n, mean, spread. "Not verified".

## Tests required (and which gates must be seen to fail)
- Gate: the adapter's per-topic message counts equal the manifest/bag-info counts exactly, or the difference is explained.
- Gate: the export of S3 reproduced twice gives byte-identical `images.csv` (determinism).
- Failure is a result: a calibration that does not converge is reported as such.

## Follow-ups (found out of scope)
- If `Rosbag2Source` mishandles a real bag, raise a fix ticket against T-015.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
