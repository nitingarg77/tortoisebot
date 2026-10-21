# Test plan: run 4, every goal within 5 cm (ticket A-005)

Branch `feature/precision-5cm`. Owner decisions D-031 (5 cm applies to exploration goals; no encoders in
Phase 1; odometry stays lidar + IMU) and D-032 (tighter tolerance and better localisation). Background:
`docs/test-reports/auto-mapping-run3.md`.

Status: **not run.** Nothing here has touched the robot. Every change below is a hypothesis until run 4.
Floor markers (ground truth) and copying the run-3 robot logs are still undecided/pending; the plan works
with or without markers.

## 1. What changed, and what it should improve

| File | Param | Old -> new | Reason (one line) |
|---|---|---|---|
| nav2_params_robot.yaml | `general_goal_checker.xy_goal_tolerance` | 0.15 -> 0.05 | Old value declared success 14-20 cm short (BUILD_LOG 221-223, 304-305) |
| nav2_params_robot.yaml | `FollowPath.trans_stopped_velocity` | 0.25 -> 0.02 | 0.25 > `max_vel_x`, so RotateToGoal always saw "stopped" (run-3 report A3) |
| nav2_params_robot.yaml | `FollowPath.xy_goal_tolerance` | 0.05 (kept) | Must equal the checker's; wider would spin the robot in place short of the goal |
| nav2_params_robot.yaml | `FollowPath.max_vel_x`, `max_speed_xy` | 0.15 -> 0.10, 0.22 -> 0.10 | Slow final approach; DWB has no distance-based scaling, so the cap is the lever. Coast is unmeasured |
| nav2_params_robot.yaml | `velocity_smoother.max/min_velocity[0]` | +-0.15 -> +-0.10 | Mirror of `max_vel_x` (file comment) |
| explore_params_robot.yaml | `max_linear_speed_vmax` | 0.15 -> 0.10 | Mirror of `max_vel_x` (MRTSP estimate only) |
| nav2_params_robot.yaml | `progress_checker` | Simple -> `PoseProgressChecker`, radius 0.25 -> 0.10, `required_movement_angle` 0.5 (new), 20 s kept | In-place turns and the slow creep near walls no longer count as "no progress" (run 3: 9 aborts) |
| nav2_params_robot.yaml | `GridBased.tolerance` | 0.5 -> 0.1 | A plan must not end up to 0.5 m short of the goal |
| explore_params_robot.yaml | `occ_threshold` | 65 (default) -> 50 | Goals >= ~0.33 m from a wall: radius 0.10 + tolerance 0.05 + worst pose error 0.12 = 0.27 m needed. Default 65 gives ~0.24 m |
| slam_real_mapbased.lua | `linear_search_window` | 0.1 -> 0.3 | Same as `slam_real.lua`, whose comment records a 0.1 window losing the match on late scans (-559 m runaway, 2026-09-25) |

Deliberately not changed, with the reason:
- **Controller stays DWB.** Regulated Pure Pursuit has `approach_velocity_scaling_dist` but also
  `min_approach_linear_velocity`, and the robot's real average speed (0.04 m/s) is below that kind of floor;
  swapping controllers cannot be checked at a desk. Revisit if run 4 shows the speed cap is not enough.
- **`slam_real.lua`** (SLAM for exploration): it carried run 1 (25.5 m, 21 goals, no aborts). The 7-12 cm pose
  lag has no profile behind it; tuning Cartographer blind could undo that. Section 4 measures first.
- **Inflation 0.5/0.55, scale 3.0**: at these values the inscribed zone is only 0.1 m, and with
  `occ_threshold` 50 the goals are 0.33 m clear, so inflation is not what blocks a 5 cm goal.
- **AMCL** (`sigma_hit` 0.2 etc.): never measured on the real robot; candidate for a later A/B.
- **Yaw tolerance 0.25 rad**: unchanged. Note the checker is `stateful`: once within 5 cm the robot may
  rotate to satisfy yaw, and an open-loop spin can move it. Measure position at the moment of success AND at rest.

Expected effects, each a hypothesis: success is no longer declared 14-20 cm short; goals are not abandoned
for lack of travel; the robot is slower near goals, coasting less. The goal-pose error then becomes limited
by pose accuracy (7-12 cm seen), which no param change here fixes. If section 4's stationary drift is
above ~2 cm, 5 cm cannot be verified by the robot's own pose, and the pass criterion rests on the independent witness.

## 2. Goal sets

Frame: map origin at the start pose, x forward, y left, yaw 0 (the pose Cartographer / AMCL starts from).
Choose a start with at least 1.2 m of free floor ahead and 0.6 m to each side; check every goal below is at
least 0.35 m from any wall on the saved map, else shift it and record the shift.

**Set P (precise, 10 goals, the gate).** Scripted, so ground truth is known. Taught with
`nav_course.py --record` by placing the robot by hand on each marker/measured spot, then run with
`--places`. Goal yaw 0 for all (avoids a spin that could move the robot).

| # | x (m) | y (m) | # | x (m) | y (m) |
|---|---|---|---|---|---|
| 1 | 0.50 | 0.00 | 6 | 0.00 | -0.50 |
| 2 | 1.00 | 0.00 | 7 | 0.50 | -0.50 |
| 3 | 1.00 | 0.50 | 8 | 1.00 | -0.50 |
| 4 | 0.50 | 0.50 | 9 | 0.50 | 0.00 |
| 5 | 0.00 | 0.50 | 10 | 0.00 | 0.00 |

The set mixes 0.5 m forward legs (the nav6-nav8 protocol), 0.5 m lateral steps that force a turn, and a return
to the start (a closed loop exposes drift: goal 10 should land where it began).

**Set E (exploration, reported separately, decided by owner whether it is also a gate).** D-031 says the
5 cm target applies to exploration goals. A frontier goal is chosen by the explorer, so its pose is not
known in advance. Run the explorer for 10 min as in run 3 and judge every goal it sends by the logged goal pose
against the pose at success. Set P is the gate because it can be measured on the floor; Set E tells whether
the same tolerance is achievable on frontier goals.

Run order: (1) hand-place and record Set P spots and the golden scans (section 5); (2) stationary-drift
baseline (section 4); (3) Set P on a saved map with `navigation_mapbased` (AMCL) ; (4) Set P in SLAM
mode (`navigation_slam`, mapping on) ; (5) Set E exploration run. Steps 3 and 4 are the same 10 goals in
the two localisation modes and answer which one to use for goal runs.

## 3. What to record

Every run, before starting the stack:

```
ros2 bag record -o ~/logs/bags/run4_<step> /tf /tf_static /odom /scan /scan_fused /imu /plan /local_plan \
  /cmd_vel /cmd_vel_nav /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback \
  /local_costmap/costmap /global_costmap/costmap /map
```
(`/imu` is the Cartographer input topic; check names with `ros2 topic list` first and fix the list.) Disk on
the Pi: check `df -h` first; the costmaps are the large part, record them at the lower rate by running the
costmap topics in a second bag only for Set E.

Also keep, per run: the goal list with sent time, Nav2's result code and message per goal
(`~/logs/nav/run4.<step>.robot.log` and `.compute.log`), `vcgencmd measure_temp` and `get_throttled`
every 10 s (cause B6), and for Set E the final `map.pgm`.

**After each run, copy back to the box** (needs `ssh tb`, owner present):

```
rsync -a tb:~/logs/ ~/robot_logs/run4/      # run on the box; includes bags, nav logs, explore/run4/map.pgm
```
Also copy the still-pending run-3 files in the same command: `~/logs/nav/run3.robot.log`,
`run3.compute.log`, `~/logs/explore/run3/map.pgm`. Do not delete anything on the robot until the box copy is
listed and its sizes match.

Preflight (CLAUDE.md section 1: check the value landed): the robot checkout is still `690c45c` plus
hand-copied files. After deploying this branch, on the robot
`grep -E 'xy_goal_tolerance|trans_stopped_velocity|max_vel_x|required_movement' <installed>/nav2_params_robot.yaml`
and `ros2 param get /controller_server general_goal_checker.xy_goal_tolerance` must show 0.05; also
`ros2 param get /controller_server progress_checker.plugin`. Config files are installed by `colcon build`
(heavy job: ask the owner) unless the workspace is symlink-installed; do not assume.

Motion safety (CLAUDE.md section 2): goals go through `nav_course.py` / `one_goal.py`, which cancel the
goal on exit. Ctrl-C on the client does not stop a goal Nav2 already accepted: cancel the goal or deactivate
`controller_server`. Owner present at the robot throughout, finger on the stop.

## 4. Localisation baseline (before judging any goal)

The pose moved 0.12 m with no motion commanded (BUILD_LOG 314-318). Measure it, do not assume:
- 5 times, robot stationary, stack running: log `map -> base_link` for 60 s. Report n, mean and max excursion.
  Done in each mode (mapbased AMCL; SLAM). Target: excursion under 2 cm at rest. Above that, the robot's own
  pose cannot verify 5 cm and the golden-scan witness (section 5) is the only measurement.
- Run the 0.1 m vs 0.3 m `linear_search_window` of `slam_real_mapbased.lua` for the mapbased mode: report
  runaway events and CPU (`top` on the Pi). If 0.3 loads the Pi enough to delay scans, the change hurt: revert it.
- Measure coast: command a 0.5 m forward goal at 0.10 and at 0.15 m/s caps; record lidar ahead (median over several
  bearings, per CLAUDE.md) after zero `cmd_vel`, 5 times each. This decides whether the speed cap does anything.

## 5. How to measure the error

Define the reference point once: the origin of `base_link` (mid-axle) projected on the floor. Mark it on the
chassis (a pointer or a laser line) so it can be read against the floor.

**With floor markers.** Tape a cross at each Set P goal (positions from section 2, set out with a tape from a
fixed datum and a square). After Nav2 reports success, and again after 10 s at rest, measure the offset from
the marker to the chassis point with a ruler (x along the start heading, y across): error = hypot(dx, dy).
Ruler accuracy about 0.5 cm, so a 5 cm criterion has 10x margin. Photograph each.

**Without markers.** Two independent witnesses, both needed:
1. *Golden scan.* Hand-place the robot at each goal using a tape from two walls (the distances give x,y to
   about 1 cm), record 3 s of `/scan`. After the run, in the bag, take a scan at the moment of success and at rest
   and compute the rigid transform to the golden scan offline (ICP; or lidar median to walls at several
   bearings as `places.drive_verdict` does). Report the transform magnitude as the error. This does not use
   Cartographer's pose at all.
2. *Tape.* After success, measure the distance from the chassis reference point to the two nearest walls
   and compare with the hand-placed values.

Report for every goal: goal, map pose at success, map pose at rest, independent error (marker or golden scan),
Nav2 result and time, number of recoveries. Keep the map-pose error (Nav2's view) separate from the independent
one: they disagreed by more than 5 cm on 2026-10-01 (`drive_verdict` `pose_disagrees`).

## 6. Pass criterion

- **Pass:** at least 9 of the 10 Set P goals end within 0.05 m of the goal by the independent measurement,
  at success and at rest, in at least one localisation mode.
- Every goal is reported, including failures. Zero recoveries is not required but is reported.
- Fail on any contact with a wall, any runaway (`RunawayDetector` trip), or a goal reported SUCCEEDED while the
  independent error is above 0.05 m (counted, and reported as a false success).
- Set E: report goals sent, reached within 0.05 m by map pose at success, "Failed to make progress" count (run 3: 9),
  and total distance. Compared with run 3, not gated unless the owner says so.
- One run is not a measurement (CLAUDE.md section 1): repeat Set P at least 3 times and report n, mean and spread
  before calling a pass.

## 7. Rollback

Anything unsafe or a clear regression: cancel the goal, deactivate `controller_server`, stop.

Return to the previous behaviour with the `frontier-exploration` params:

```
git checkout frontier-exploration -- tortoisebot_navigation/config tortoisebot_slam/config
```
then redeploy and re-run the preflight grep (expect `xy_goal_tolerance: 0.15`). Single-knob rollbacks, if one
change is suspected: `max_vel_x` back to 0.15 in `nav2_params_robot.yaml`, `velocity_smoother` and
`explore_params_robot.yaml` together; delete `occ_threshold` from `explore_params_robot.yaml`;
`slam_real_mapbased.lua` `linear_search_window` back to 0.1; `progress_checker` back to
`nav2_controller::SimpleProgressChecker` with radius 0.25 (and no angle).

## 8. Desk checks done

`./run_tests.sh`: 245 passed (firmware 37, navigation 193, imu 15) plus the undefined-names check over 54
files. `test_precision_params.py` (17 tests) pins the values above and checks that every config yaml parses.
It cannot say whether the robot reaches 5 cm.
