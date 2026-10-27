# T-015: stage 1 export (bag to dataset) and the rosbag2 adapter
Status: blocked (T-002, T-003, T-005)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-015-bag-export
Depends on: T-002, T-003, T-005
Design: "Data model" (`dataset/<session>/`); "Interfaces" row "Bag reading"; ADR 5 (ROS confined to one adapter)

## Goal
Convert any `BagSource` into the `dataset/<session>/` directory every later stage reads, and provide the one module that
imports `rosbag2_py`. The export is desk-tested on `SyntheticBag`; the adapter is checked against a real bag later (T-031).

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC1 (per-topic rate and size are computed here from the bag), AC4 (clock audit written out), AC5 (input for visual SLAM).

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/bag_export.py` (new; no ROS import)
- `analysis/camera_spike/camera_spike/rosbag2_source.py` (new; the ONLY ROS importer in the package)
- Test: `analysis/camera_spike/test/test_bag_export.py`, `analysis/camera_spike/test/test_rosbag2_source.py`

## Approach (step by step)
1. `bag_export.py`: `export(source: BagSource, out_dir: Path, carto_frames=("map","odom","base_link")) -> ExportSummary`.
   Writes: `images/<recv_ns>.jpg` (the recorded bytes verbatim, not re-encoded), `images.csv (recv_ns, header_ns, file)`,
   `imu.csv`, `scans.npz` (arrays `t_ns`, `ranges` of shape N x B, `angle_min`, `angle_inc`), `carto.csv (t, map_base x y yaw,
   map_odom x y yaw)` composed from `/tf` (`map->odom` and `odom->base_link`), `clock_audit.csv` (one row per topic, T-005),
   `bad.csv (topic, recv_ns, reason)` for every `Bad`. Refuses to write into an existing non-empty directory
   (`FileExistsError`); the source is never modified.
   `ExportSummary` has per-topic count, bytes, mean rate, expected rate ratio, bad count, gaps (T-005 `find_gaps`).
2. A JPEG that `cv2.imdecode` cannot decode is still written (it is data) but counted in `bad.csv` with reason `decode`.
3. `rosbag2_source.py`: `Rosbag2Source(bag_dir: Path)` implementing `BagSource` with `rosbag2_py.SequentialReader`
   (sqlite3) and `rclpy.serialization.deserialize_message`. Message-to-dataclass functions are separate pure functions
   `frame_from_msg(recv_ns, msg)`, `imu_from_msg`, `scan_from_msg`, `tf_from_msg` taking duck-typed objects so they can be
   tested with `types.SimpleNamespace`. Corrupt or undeserialisable messages yield `Bad`, never raise. Output is time ordered.
   Imports of `rosbag2_py`/`rclpy` happen inside `__init__`, not at module scope, so `check_names.sh`/pytest import the
   module on a machine without ROS.
4. Tag in the module docstring: "desk tests cover the conversion functions only; reading a real .db3 is verified in T-031".

## Tests required (and which gates must be seen to fail)
- Round trip: `SyntheticBag` (with a gap, duplicate, corrupt JPEG and header offset) exported; `images.csv` rows equal frames
  emitted; the JPEG bytes on disk equal the originals byte for byte (gate: re-encode once and the test must fail);
  `carto.csv` shows the injected `carto_jump` in `map->odom` only; `bad.csv` has the corrupt one.
- Export into an existing non-empty directory raises.
- `frame_from_msg` etc. on `SimpleNamespace` fakes produce the right dataclasses; a message missing a field yields `Bad`.
- Importing `rosbag2_source` without ROS installed does not raise (the silent-skip trap: the test asserts the import works and
  that constructing `Rosbag2Source` without ROS raises a clear `SpikeError`, rather than being skipped).

## Follow-ups (found out of scope)
- Verification against a real bag and a real corrupt split happens in T-031.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
