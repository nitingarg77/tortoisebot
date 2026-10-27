# T-013: ground-plane visual odometry (feature tracking, integration, TUM output, tracking log)
Status: blocked (T-002, T-003, T-004, T-012)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-013-ground-plane-vo
Depends on: T-002, T-003, T-004, T-012
Design: "Interfaces and contracts" row "Visual odometry"; "Pure modules" `ground_plane_vo`

## Goal
A `VisualOdometry` implementation that reads a `dataset/<session>/` directory (images + images.csv), tracks floor features
between consecutive frames, integrates planar motion with T-012, and writes a TUM trajectory plus a tracking log.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC5 (names system and version: the module exposes `NAME` and `VERSION` strings for the report).
- AC7 (tracking loss reported, with where and for how long).
- AC10 B (metric drift per metre from this system).

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/ground_plane_vo.py` (new)
- Test: `analysis/camera_spike/test/test_ground_plane_vo.py`

## Approach (step by step)
1. Class `GroundPlaneVO(VisualOdometry)`; `__init__(self, mount: CamMount, min_inliers: int = 12, max_features: int = 300,
   roi_top_frac: float = 0.35)`; `NAME = "ground_plane_vo"`, `VERSION = "1"`.
2. `run(dataset_dir, calib, out_dir)`:
   - read `images.csv (recv_ns, header_ns, file)`; decode each with `cv2.imdecode`; a `None` result is counted and skipped
     (failure-handling row "JPEG decode failure"); undistort with `calib` (`cv2.undistortPoints`/`remap`).
   - per consecutive pair: `cv2.goodFeaturesToTrack` in the lower part of the image (floor), `cv2.calcOpticalFlowPyrLK`
     with forward-backward check, then `planar_motion` (T-012). If inliers < `min_inliers` or the call raises
     `InsufficientPairs`, the frame is `lost`: no trajectory row, a `TrackingLog` row with `state='lost'`, and integration
     resumes at the next good pair from the last pose (the lost interval's motion is unknown and not guessed).
   - A gap in frames longer than 2 periods is also `lost`.
   - Integrate SE(2): pose_k = pose_{k-1} composed with motion. z = 0. Quaternion from yaw.
3. Write `out_dir/traj.tum` (`t x y z qx qy qz qw`, t in seconds of `recv_ns`), `out_dir/tracking.csv (t, state, n_features)` and
   `out_dir/run.json` (command, version, parameters). Return the `Trajectory` and `TrackingLog`.
4. No file under `dataset_dir` is modified.

## Tests required (and which gates must be seen to fail)
- Using `render_floor` + `make_floor_texture`: render a 1.00 m straight move at 10 fps into a temp `dataset/`
  and the recovered end point is within a tolerance chosen by the implementer and RECORDED in the test (target: 3 cm);
  a 90 deg in-place turn recovered within a recorded tolerance (target: 2 deg).
- Gate seen to fail: construct `GroundPlaneVO` with `height_m` 10 percent wrong; the 1.00 m test must fail. Also with tilt wrong
  by 3 deg. Restore.
- Black frames for 1 s in the middle: those frames are `lost`, the trajectory has no rows in the hole, and the end pose
  after the hole is not interpolated (drift accepted, flagged as lost).
- A corrupt JPEG (`b"x"`) in `images.csv` is counted and skipped without raising.
- The output TUM file round-trips through a reader into an identical `Trajectory`.

## Follow-ups (found out of scope)
- Real floor texture adequacy is measured in the VO job (T-032), not here.
- Rolling shutter / blur effects are measured, not simulated (design R5).

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
