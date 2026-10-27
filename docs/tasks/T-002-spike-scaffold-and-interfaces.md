# T-002: camera-spike package scaffold, shared types and the BagSource / VisualOdometry interfaces
Status: ready
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-002-spike-scaffold
Depends on: none
Design: `docs/architecture/camera-assisted-localisation-spike.md` (D5: `analysis/camera_spike/` with `COLCON_IGNORE`)

## Goal
Create the home for every ROS-free spike module, wired into `./run_tests.sh` so no later module can rot (CLAUDE s3).
Define the dataclasses and the two interfaces every later ticket imports. This is the first ticket: interface first,
fake in T-003.

## Acceptance criteria satisfied (AC numbers from the requirements)
Enabling ticket. Supports AC5, AC6, AC7, AC8 (the interfaces they run through). No AC is met by this ticket alone.

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/spike_types.py` (new)
- `run_tests.sh` (add `analysis/camera_spike` to the `for pkg in ...` list)
- Scaffolding, not logic: `analysis/camera_spike/COLCON_IGNORE` (empty file), `analysis/camera_spike/camera_spike/__init__.py`,
  `analysis/camera_spike/test/conftest.py` (copy the pattern of `tortoisebot_firmware/test/conftest.py` so
  `import camera_spike` resolves when run from `analysis/camera_spike`).
- Test: `analysis/camera_spike/test/test_spike_types.py`

## Approach (step by step)
1. Read `tortoisebot_firmware/test/conftest.py` and `run_tests.sh`. The loop does `cd "$pkg" && python3 -m pytest test -q`,
   so `pkg=analysis/camera_spike` works unchanged. Add it to the list; do not touch anything else in the script.
2. Create `COLCON_IGNORE`, `__init__.py`, `conftest.py`.
3. In `spike_types.py` (imports numpy and stdlib only; no ROS, no cv2 at module scope) define:
   ```python
   @dataclass(frozen=True) class Frame:      recv_ns: int; header_ns: int; jpeg: bytes
   @dataclass(frozen=True) class ImuSample:  recv_ns: int; header_ns: int; gx: float; gy: float; gz: float; ax: float; ay: float; az: float
   @dataclass(frozen=True) class Scan:       recv_ns: int; header_ns: int; angle_min: float; angle_inc: float; ranges: np.ndarray  # float32, inf kept as inf
   @dataclass(frozen=True) class Tf:         recv_ns: int; parent: str; child: str; x: float; y: float; z: float; qx: float; qy: float; qz: float; qw: float
   @dataclass(frozen=True) class Bad:        topic: str; recv_ns: int; reason: str     # a corrupt message, counted not raised
   @dataclass(frozen=True) class Mark:       wall_ns: int; mono_ns: int; label: str; event: str; text: str   # event in {arrive,leave,start,stop,note}
   @dataclass(frozen=True) class Window:     start_ns: int; end_ns: int
   @dataclass class Trajectory:  t_ns: np.ndarray; pos: np.ndarray; quat_xyzw: np.ndarray   # shapes (N,), (N,3), (N,4); times in the bag receive clock
   @dataclass class TrackingLog: t_ns: np.ndarray; state: list[str]; n_features: np.ndarray  # state in {ok,lost,init}
   class SpikeError(Exception); class InsufficientPairs(SpikeError); class DegenerateGeometry(SpikeError); class NotAvailable(SpikeError)
   class BagSource(Protocol):  frames()->Iterator[Frame|Bad]; imu(); scans(); transforms(); static_transforms(); all time-ordered
   class VisualOdometry(Protocol): run(self, dataset_dir: Path, calib: "CameraModelLike", out_dir: Path) -> tuple[Trajectory, TrackingLog]
   ```
   `CameraModelLike` is a Protocol with `K: np.ndarray (3,3)` and `dist: np.ndarray`. Document in the module docstring:
   "lost frames are absent, never interpolated".
4. Tests: construct each dataclass; `Trajectory` shape check helper `validate()` raising `SpikeError` on mismatched lengths;
   the typed errors are `SpikeError` subclasses.
5. Run `env PATH=/usr/bin:/bin:$PATH PYTHONDONTWRITEBYTECODE=1 ./run_tests.sh`; the new line `analysis/camera_spike` must
   print a pass count above 0. Record the total (T-001 says baseline 245, CLAUDE.md says 228; report what you see).

## Tests required (and which gates must be seen to fail)
- `test_spike_types.py` as above.
- Gate seen to fail: temporarily rename the test function file to `check_spike_types.py` (no `test_` prefix) and confirm
  `run_tests.sh` reports "no tests ran" / exit 5 for that directory, not green; restore. This proves the new directory is
  really collected (CLAUDE s3, silent-zero-tests rule).
- Also seen to fail: `Trajectory.validate()` with `pos` length different from `t_ns`.

## Follow-ups (found out of scope)
- CI (`.github/`) is owner-only; it runs `run_tests.sh`, so no CI edit is needed, but the owner should confirm the new
  path is picked up on the next push.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
