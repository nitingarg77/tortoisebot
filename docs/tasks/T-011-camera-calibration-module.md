# T-011: camera calibration module (checkerboard to CameraModel, subset spread, raw-vs-JPEG comparison)
Status: blocked (T-002, T-004)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-011-camera-calib
Depends on: T-002, T-004
Design: "Calibration and time sync" rows "Intrinsics and distortion" and "JPEG effect on calibration"

## Goal
Turn checkerboard frames into intrinsics with an honest error figure, plus a function that compares two calibrations (raw
clip vs compressed stream).

## Acceptance criteria satisfied (AC numbers from the requirements)
- Open question 8 (calibration unknown, established by the spike). Supports AC5 and AC8: VO needs K.
- Supports AC1 report content (JPEG effect on calibration).

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/camera_calib.py` (new)
- Test: `analysis/camera_spike/test/test_camera_calib.py`

## Approach (step by step)
1. Signatures (numpy, cv2):
   ```python
   @dataclass class CameraModel: K: np.ndarray; dist: np.ndarray; rms: float; n_frames: int; fx_subset: list[float]; fx_spread: float; size: tuple[int,int]
   def detect_corners(gray, pattern: tuple[int,int]) -> np.ndarray | None          # findChessboardCorners + cornerSubPix
   def select_spread_frames(corner_sets: list[np.ndarray], max_frames: int = 60) -> list[int]    # greedy farthest-point on corner-centroid / area / skew
   def calibrate(images: list[np.ndarray], pattern: tuple[int,int], square_m: float, prior_fx: float = 635.0) -> CameraModel
   def compare_models(a: CameraModel, b: CameraModel) -> dict[str, float]          # |dfx|, |dfy|, |dcx|, |dcy|, |dk1|
   def prior_check(m: CameraModel, prior_fx: float, rel_tol: float = 0.15) -> tuple[bool, str]
   ```
2. `calibrate`: detect on every image, count failures (reported, in `n_frames` the used ones), select spread frames, run
   `cv2.calibrateCamera`, then repeat on 3 disjoint subsets of frames to fill `fx_subset` and `fx_spread` (sample std).
   Raise `InsufficientPairs` if fewer than 10 usable frames.
3. `prior_check` flags (does not reject) fx more than 15 percent from the prior (about 635 px from the 53.5 deg HFOV).
   The design: "fx far from the prior is investigated, not accepted".
4. The pattern size and square size are inputs. The printed board is the owner's; its inner-corner count and square size are
   recorded in the S2 ticket (owner decision).

## Tests required (and which gates must be seen to fail)
- Render 40 board poses with `render_board` through known K (fx 635) and a small k1; `calibrate` recovers fx and fy
  within 1 percent, principal point within 3 px, RMS below 0.5 px.
- Subset spread is finite and small on clean data; larger when noise (sigma 0.5 px on corners via re-rendering with noise) is added.
- `compare_models` of a model with itself is all zeros; JPEG re-encode (quality 50) of the same renders changes parameters
  by a measured, reported amount (assert only that it is computed, not a bound).
- Fewer than 10 detections raises `InsufficientPairs`.
- Gate seen to fail: calibrate with the wrong `square_m` and confirm the translation-scale assertion fails; swap
  `pattern` axes and confirm detection count test fails. Restore.

## Follow-ups (found out of scope)
- Fisheye or rolling-shutter models are not attempted; the OV5647 binned mode is a pinhole + k1,k2 approximation.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
