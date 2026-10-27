# T-007: SE(2) / Sim(2) fits and start anchoring (spike_align, part 1)
Status: blocked (T-002)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-007-alignment-core
Depends on: T-002
Design: "Scale and alignment (AC8)", labels U0, U1, F-carto, F-tape-all

## Goal
The alignment maths: Umeyama similarity fit in 2D with and without scale, applying a fit, and anchoring a trajectory to
the start pose. Refuses with a typed error when it cannot be done honestly.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC8 (scale handling stated; error after fitting reported next to error before).
- AC6 (comparison of visual trajectory with Cartographer and with the tape needs these fits).

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/spike_align.py` (new)
- Test: `analysis/camera_spike/test/test_spike_align.py`

## Approach (step by step)
1. Signatures:
   ```python
   @dataclass(frozen=True) class Sim2: s: float; R: np.ndarray; t: np.ndarray
       def apply(self, xy: np.ndarray) -> np.ndarray      # (N,2) -> (N,2): s * xy @ R.T + t
   def umeyama_sim2(src: np.ndarray, dst: np.ndarray, with_scale: bool = True) -> Sim2    # proper rotation only (det = +1)
   def se2_fit(src, dst) -> Sim2                          # same with s fixed to 1.0
   def start_anchor(xy: np.ndarray, yaw: np.ndarray, t_ns: np.ndarray, window: Window) -> Sim2   # SE(2) mapping the window-median pose to (0,0,0)
   ```
2. Raise `InsufficientPairs` if `len(src) < 3` or shapes differ; raise `DegenerateGeometry` if the source points are
   collinear (rank of the centred matrix < 2) because rotation and scale are then not determined for Sim(2) and the
   report should say so. `start_anchor` needs at least one sample in the window; otherwise `NotAvailable`.
3. Use the closed-form Umeyama via SVD, fixing a reflection (det = -1) by flipping the last singular direction.
4. `start_anchor`: median x, y over the window and circular-mean yaw (`atan2(median sin, median cos)`), then rotate and
   translate so that pose becomes the origin with zero heading. No scale.

## Tests required (and which gates must be seen to fail)
- Recovers a known Sim(2) (s=1.3, 37 deg, t=(2,-1)) from 20 noise-free points to 1e-9; SE(2) recovers rotation and
  translation with `s == 1.0`.
- With Gaussian noise (sigma 0.01 m, seeded) the recovered parameters are within 3 sigma-derived tolerance.
- `with_scale=False` on data that really has scale 1.3 leaves a larger residual than `with_scale=True` (the "fit hides
  poor result" demonstration).
- Fewer than 3 pairs raises `InsufficientPairs`; collinear points raise `DegenerateGeometry`; a mirrored dst never yields
  det(R) = -1.
- `start_anchor` of a trajectory that starts at (3, 2, 90 deg) gives first-window pose (0, 0, 0) to 1e-9.
- Gate seen to fail: drop the det-correction line and confirm the mirrored-data test fails; restore.

## Follow-ups (found out of scope)
- Leave-one-out and the floor-plane fit are T-008, in the same file and a second module.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
