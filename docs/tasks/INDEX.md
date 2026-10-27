# Task index

Status vocabulary: see `docs/WORKFLOW.md`. "blocked" means a dependency is not done.

## Rigid-frame SLAM (precision-5cm)

| ID | Title | Type | Depends on | Status |
|---|---|---|---|---|
| T-001 | rigid-frame SLAM variant for goal runs (no global loop closure) | job | none | ready |

## Camera-assisted localisation, Phase 1 spike (design `docs/architecture/camera-assisted-localisation-spike.md`)

Robot sessions T-025 to T-030 and the box runs all follow T-001's robot measurements. The recorder is not run during T-001's measurements.

### ROS-free pure modules and desk tests (`analysis/camera_spike/`)

| ID | Title | Type | Depends on | Status |
|---|---|---|---|---|
| T-002 | spike scaffold, shared types, BagSource / VisualOdometry interfaces, run_tests.sh entry | code | none | ready |
| T-003 | SyntheticBag and ScriptedVO fakes | code | T-002 | blocked |
| T-004 | render synthetic floor and checkerboard frames | code | T-002 | blocked |
| T-005 | clock audit and gap detection | code | T-002, T-003 | blocked |
| T-006 | stationary windows and mark-to-window association | code | T-002, T-003 | blocked |
| T-007 | SE(2)/Sim(2) fits and start anchoring | code | T-002 | blocked |
| T-008 | leave-one-out error and floor-plane fit | code | T-007 | blocked |
| T-009 | spike metrics | code | T-002 | blocked |
| T-010 | time offset by cross-correlation | code | T-002, T-003 | blocked |
| T-011 | camera calibration module | code | T-002, T-004 | blocked |
| T-012 | ground-plane geometry | code | T-002, T-004, T-007 | blocked |
| T-013 | ground-plane VO | code | T-002, T-003, T-004, T-012 | blocked |
| T-014 | report rendering and blank thresholds file | code | T-002, T-009 | blocked |
| T-015 | bag export and rosbag2 adapter | code | T-002, T-003, T-005 | blocked |
| T-016 | ORB-SLAM3 subprocess adapter | code | T-002, T-003, T-011 | blocked |
| T-017 | pass pipeline and end-to-end desk run | code | T-003, T-005, T-006, T-007, T-008, T-009, T-010, T-014, T-015 | blocked |

### Pi tools (`tortoisebot_firmware`)

| ID | Title | Type | Depends on | Status |
|---|---|---|---|---|
| T-018 | bring in record_capture.sh (ea199de) | code | none | ready |
| T-019 | recorder: spike topics, 1 GB splits, disk gate, --dry-run | code | T-018 | blocked |
| T-020 | mark_cross.py | code | none | ready |
| T-021 | res_sampler.py | code | none | ready |
| T-022 | session manifest writer | code | none | ready |

### Jobs (robot sessions and box runs; never inline, owner present or asked first)

| ID | Title | Type | Depends on | Status |
|---|---|---|---|---|
| T-023 | check bag/dataset paths are excluded from the Drive backup (D7) | job | none | ready |
| T-024 | build ORB-SLAM3 v1.0 on the box | job | T-016 | blocked |
| T-025 | session S0 preflight (robot, at rest) | job | T-001, T-019, T-020, T-021, T-022, T-023 | blocked |
| T-026 | session S1 Pi cost (robot, at rest) | job | T-025 | blocked |
| T-027 | session S2 calibration capture (robot, at rest) | job | T-025 | blocked |
| T-028 | session S3 10-minute rest (robot, at rest) | job | T-026 | blocked |
| T-029 | session S4 Set P passes, at least 3 (robot, hand-placed) | job | T-026, T-028, T-020 | blocked |
| T-030 | session S5 five guarded 1 m drives (robot, motion) | job | T-001, T-026, T-029 | blocked |
| T-031 | box stages 1-2: export, clock audit, calibration | job | T-005, T-006, T-008, T-010, T-011, T-015, T-017, T-027, T-025..T-030 | blocked |
| T-032 | box: ground-plane VO runs | job | T-013, T-031 | blocked |
| T-033 | box: ORB-SLAM3 runs | job | T-016, T-024, T-031 | blocked |
| T-034 | box stages 4-5: alignment, metrics, report | job | T-017, T-026, T-031, T-032, T-033 | blocked |
