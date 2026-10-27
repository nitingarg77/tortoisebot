# T-003: SyntheticBag and ScriptedVO fakes (with fake Cartographer pose)
Status: blocked (T-002)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-003-synthetic-bag-vo
Depends on: T-002
Design: "Interfaces and contracts", rows "Bag reading" and "Visual odometry"

## Goal
Provide the fakes that let every later module and the end-to-end desk run be tested with known answers and no robot:
a `BagSource` built from a scripted ground-truth path with injectable faults, and a `VisualOdometry` that returns the
ground truth distorted by a known scale, rotation, drift and lost intervals.

## Acceptance criteria satisfied (AC numbers from the requirements)
Enabling ticket for AC4, AC6, AC7, AC8 tests (known-answer inputs for matching, comparison, lost-interval reporting and
before/after-alignment reporting).

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/synthetic.py` (new)
- Test: `analysis/camera_spike/test/test_synthetic.py`

## Approach (step by step)
1. Read `camera_spike/spike_types.py` (T-002). Import only from it, numpy, cv2 (for `imencode`) and stdlib.
2. Ground truth: `class GroundTruth` built from segments. Constructor helpers (all return `GroundTruth`):
   `GroundTruth.stationary(duration_s, x=0.0, y=0.0, yaw=0.0)`, `.straight(length_m, speed_mps, x0=0, y0=0, yaw=0)`,
   `.turn_in_place(angle_rad, rate_rps)`, `.concat(*parts)`. Method `pose(t_s: float) -> (x, y, yaw)` and
   `stationary_intervals() -> list[tuple[float, float]]` (the exact truth the stillness test must recover).
3. `class SyntheticBag(BagSource)`:
   `__init__(self, gt, *, t0_ns=1_700_000_000_000_000_000, frame_hz=10, imu_hz=50, scan_hz=10,
   header_base_offset_ns=0, camera_time_offset_ns=0, gaps=(), duplicates=(), nan_beams=0, carto_jump=None, seed=0)`.
   - `gaps`: list of `(topic, t0_s, t1_s)`; messages in that span are not emitted.
   - `duplicates`: list of `(topic, t_s)`; that message is emitted twice.
   - `header_base_offset_ns`: added to every `header_ns` (e.g. 1.7e18 to mimic a different clock base); recv times unchanged.
   - `camera_time_offset_ns`: added to frame `header_ns` only (a camera/IMU lag).
   - `carto_jump=(t_s, dx, dy)`: `map->odom` steps by that at `t_s` while `odom->base_link` stays continuous.
   - Frames: `jpeg` is `cv2.imencode('.jpg', 8x8 uint8 image whose pixel [0,0] encodes the frame index)`; plus one frame
     per `corrupt_every_n` set to `b"not a jpeg"` if `corrupt_every_n` is given (the consumer must count it, T-015).
   - Scans: 360 beams, ranges 2.0 m plus noise sigma 0.003 m (seeded) while stationary; while moving the ranges change
     by `0.5 * speed` per second so stillness detection (T-006) has something to detect; `nan_beams` beams set to inf.
   - IMU: gz = true yaw rate (+ noise sigma 0.01 rad/s, seeded), others 0 / 9.81.
   - Yields `Bad(topic, recv_ns, reason)` for injected corruption, never raises.
4. `class ScriptedVO(VisualOdometry)`:
   `__init__(self, gt, *, scale=1.0, rot_deg=0.0, offset_xy=(0.0, 0.0), drift_per_m=0.0, lost=(), seed=0, t0_ns=...)`.
   `run(dataset_dir, calib, out_dir)` ignores the dataset and returns a `Trajectory` at 10 Hz equal to
   `scale * R(rot_deg) @ gt_xy + offset + random-walk drift` (drift sigma proportional to distance travelled), z = 0, with
   rows inside `lost` intervals ABSENT and a `TrackingLog` with `state='lost'` rows for them (and `ok` elsewhere).
   It must not interpolate.
5. Keep every random draw behind `np.random.default_rng(seed)` so tests are deterministic.

## Tests required (and which gates must be seen to fail)
- Frame / IMU / scan counts equal rate x duration (+/- 1) for a 60 s stationary path.
- A `gaps=[('imu', 10, 12)]` removes about 100 IMU messages and none from other topics.
- `header_base_offset_ns=1.7e18` shifts only `header_ns`.
- `carto_jump` produces a step in `map->odom` of the stated size and none in `odom->base_link`.
- `ScriptedVO` with `lost=[(5.0, 7.0)]` has no row with t in (5, 7) and `TrackingLog` marks the interval; with
  `scale=2.0` positions are doubled.
- Determinism: two constructions with the same seed give identical arrays.
- Gate seen to fail: make `ScriptedVO` interpolate across `lost` (add rows) and confirm the lost-interval test fails;
  restore.

## Follow-ups (found out of scope)
- A richer scan model (walls, doorways) is not needed; T-006 tests only need moving vs not moving.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
