# Test report: auto-mapping run 3 (2026-10-02)

Result: **FAIL** against the Phase 1 target (each goal reached within 5 cm). 9 frontier goals ended in
"Failed to make progress" near walls.
Command: frontier exploration via `autobringup.launch.py` + `/control_exploration`, under the scratch
supervisor `~/logs/explore_run.py` on the robot (not in this repo). 10 min run.
Collected / passed / failed / skipped: goals sent: **not recorded on the box** / reached: not recorded /
failed: 9 ("Failed to make progress") / skipped: n/a.

Written 2026-10-21 by Claude (CoS task A-004), read-only, from what is on the gpu-box. **The primary
evidence for this run is not on the box** (see "Evidence, and what is missing"). Every number below is
quoted from a source with a reference; nothing is reconstructed.

## Evidence, and what is missing

| Source | Where | What it holds |
|---|---|---|
| Session note written at the end of the run-3 session | box: `~/.claude/projects/-workspace-robotics-tb-ws/memory/tortoisebot-status-2026-10-02.md:11` | The only record of run 3 on the box: 10 min, 9.1 m driven, 9 goals "Failed to make progress", each near a wall, last 3.5 min facing the skirting, map ~20.6 m², doubled walls, camera tilt 4.75° vs 4.5°, CPU 82.7 °C |
| Run-3 robot and compute logs | robot only: `~/logs/nav/run3.robot.log`, `~/logs/nav/run3.compute.log` | Per-goal targets, poses, Nav2 messages. **Not read** (task tier forbids `ssh tb`) |
| Run-3 map | robot only: `~/logs/explore/run3/map.pgm` | Not read |
| Rosbag | none found | No `.db3`/`.mcap` anywhere under `/workspace/robotics` or `~/.ros` from 2026-10-01..03 |
| Git | `frontier-exploration` | No commit on 2026-10-02. Last run-related commit `5ac1b45` (2026-10-01 17:32) |
| BUILD_LOG | `BUILD_LOG.md` | Runs 1 and 2 (2026-10-01) and the 0.5 m forward-goal tests nav6–nav8; **no run-3 entry** |
| Box ROS logs | `~/.ros/log` | Newest files 2026-10-01 10:28 (RViz, scripts). Nothing from 2026-10-02 |
| Session transcript | not on the box or the Mac mini | The run-3 session was likely run from another machine; not found |

**Important correction to the brief.** The "stopped 14–20 cm short" figure is **not from run 3**. It is
from three 0.5 m forward goals (nav6–nav8) on 2026-10-01, which all reported SUCCEEDED
(`BUILD_LOG.md:220-223`, `BUILD_LOG.md:231-234`). Run 3's 9 goals did not succeed at all; they were
abandoned by the progress checker. These are two separate failure modes and are treated separately below.

## Setup

| Item | Value | Evidence |
|---|---|---|
| Code on the robot | `690c45c` with the `cf74b7b` files hand-copied in (content identical to origin); the pull + rebuild of 2026-10-02 never ran | status note `:15` |
| Code on the box | `5ac1b45`, clean, in sync with origin | status note `:9` |
| Nav2 / SLAM params in force | Same on both: `git log 690c45c..5ac1b45 -- tortoisebot_navigation/config tortoisebot_slam/config` is empty; last change to `nav2_params_robot.yaml` / `explore_params_robot.yaml` is `8a3e098` (2026-09-29), an ancestor of `690c45c` | git |
| Nav2 params | `tortoisebot_navigation/config/nav2_params_robot.yaml`, loaded by `navigation_slam.launch.py:38` | |
| SLAM | Cartographer `tortoisebot_slam/config/slam_real.lua`: `provide_odom_frame = true`, `use_odometry = false` (motors open-loop, no encoders), `use_imu_data = true` | `slam_real.lua:35,37,67,70` |
| Map | Fresh Cartographer map; ~20.6 m² free floor, two rooms, slightly doubled walls in the right-hand room; saved by hand to the robot | status note `:11` |
| Camera | Tilt −4.5° configured, edge threshold 80; re-measured mid-run at 4.75° down | `autobringup.launch.py:178,187`; status note `:11` |
| Goal generation | `frontier_exploration_ros2`, params `explore_params_robot.yaml` on top of the package defaults: `frontier_selection_min_distance` 0.5, `frontier_candidate_min_goal_distance_m` 0.5, `mrtsp_solver: greedy`, `goal_preemption_enabled: false`, `frontier_suppression_enabled: true`. `occ_threshold` not overridden, so the package default 65 applies | `explore_params_robot.yaml:25-26,40,48-49`; `frontier_exploration_ros2/config/params.yaml:180` |
| Supervisor | scratch `~/logs/explore_run.py` on the robot, with the SIGINT fix that exists only there; Ctrl-C stop worked twice | status note `:11,12` |

Run-over-run context: run 1 25.5 m, 21 goals, none aborted; run 2 5.6 m in 322 s, 10 goals, 7 "Failed to
make progress"; run 3 9.1 m in 10 min, 9 "Failed to make progress" (`BUILD_LOG.md:40-42`,
`BUILD_LOG.md:119-120`, status note `:11`). The failure rate rose after the camera was re-aimed down
(`cf74b7b`), which is itself a lead (cause H3).

## Goals

**The per-goal data is not available on the box.** Targets, final poses, end errors and wall distances
for the 9 goals are in `~/logs/nav/run3.robot.log` / `run3.compute.log` on the robot, which this task was
not allowed to read. No rosbag was recorded.

| # | Target (map) | Final pose (map) | Error (cm) | Outcome | Nearest wall / obstacle |
|---|---|---|---|---|---|
| 1–9 | not on box | not on box | not on box | "Failed to make progress" (all 9) | "close to a wall" (all 9); distance not recorded on box |
| last 3.5 min | — | facing the skirting | — | robot sat still | skirting board, distance not recorded |

Source for the outcome column: status note `:11`. Total goals sent in run 3 is not recorded on the box,
so the success count, if any, is unknown.

For comparison, the only goals on this robot with end errors on the box are the 2026-10-01 forward goals:

| Run | Nav2 | Left to goal (map) | Source |
|---|---|---|---|
| 0.30 m | SUCCEEDED 1.4 s | 0.149 / 0.152 m short | `BUILD_LOG.md:304` |
| 0.50 m | SUCCEEDED 2.8 s | 0.271 / 0.231 m short; lidar progress 0.34 m | `BUILD_LOG.md:305` |
| nav6 0.50 m | SUCCEEDED 2.3 s | 0.197 m | `BUILD_LOG.md:221` |
| nav7 0.50 m | SUCCEEDED 2.3 s | 0.142 m | `BUILD_LOG.md:222` |
| nav8 0.50 m | SUCCEEDED 2.3 s | 0.140 m | `BUILD_LOG.md:223` |

## Causes

### A. Why goals end 14–20 cm short (applies to every successful goal, not specific to run 3)

**A1. The goal checker accepts anything within 15 cm. Evidenced.**
`nav2_params_robot.yaml:130-134`: `general_goal_checker`, `SimpleGoalChecker`, `xy_goal_tolerance: 0.15`,
`stateful: True`. Nav2 declares success the moment the robot enters a 15 cm circle and stops there.
nav7/nav8 ended at 0.142/0.140 m, just inside it (`BUILD_LOG.md:222-223`). With this value the 5 cm target
cannot be met by design.

**A2. The success test runs on a pose that lags and jumps by more than 5 cm. Evidenced.**
On the 0.50 m goal "the map pose lagged by about 7 cm and 5°. A minute later, with no motion commanded, it
had moved forward 0.12 m" (`BUILD_LOG.md:314-318`). nav6 ended 0.197 m short by map pose, outside 0.15, and
still reported success (`BUILD_LOG.md:233-234`), so the checker's frame (`odom`) and the map pose disagreed
by ≥ 5 cm. Cartographer provides `odom` from lidar + IMU only, with `use_odometry = false` because the
motors have no encoders (`slam_real.lua:35-37,67`). A pose that drifts 7–12 cm cannot verify a 5 cm goal.

**A3. Tolerance mismatch inside the controller. Hypothesis.**
DWB's own `xy_goal_tolerance: 0.05` (`nav2_params_robot.yaml:164`) disagrees with the goal checker's 0.15
(`:133`), and `trans_stopped_velocity: 0.25` (`:165`) is above `max_vel_x: 0.15` (`:141`), so any check of
"is the robot stopped" by that value is always true. Which component reads which value in this Nav2 build
was not checked; the effect on stopping distance is unmeasured.

**A4. Open-loop coasting. Hypothesis.**
`decel_lim_x: -1.0` (`:150`) and velocity smoother `feedback: "OPEN_LOOP"` (`:401`) assume the wheels
follow commands; the motors are open-loop PWM with real speed "well below the commanded 0.15 m/s (0.04 m/s
average while exploring)" (`nav2_params_robot.yaml:123-125`). The BUILD_LOG itself lists "whatever the
robot coasts" as unmeasured (`BUILD_LOG.md:232-233`).

### B. Why run 3's 9 goals failed near walls

**B1. The progress checker abandons slow, turn-heavy approaches near walls. Mechanism evidenced earlier; not confirmed for run 3 (no logs).**
`progress_checker`: 0.25 m in 20 s (`nav2_params_robot.yaml:128-129`). The comment above it records that
"turning in place covers no distance at all, so the stricter check abandoned 11 of 12 frontier goals as
'Failed to make progress'" (`:123-127`, 2026-09-25). Near a wall, DWB must turn and creep; at a real
0.04 m/s, 0.25 m takes ~6 s of straight driving with no turning, so any manoeuvre near a wall with
repeated in-place rotations can exceed 20 s without moving 0.25 m. Run 3's message is exactly
"Failed to make progress" (status note `:11`).

**B2. The robot is driving inside inflated space by the time it is near a wall. Evidenced (params), effect a hypothesis.**
`robot_radius: 0.1` (`:199`, `:260`) with `inflation_radius` 0.5 local / 0.55 global and
`cost_scaling_factor: 3.0` (`:203-204`, `:319-320`). The explorer's own comment says the costmaps
"inflate to 0.5–0.55 m, so a frontier much closer than that is rarely reachable"
(`explore_params_robot.yaml:22-25`). With 0.55 m of inflation in rooms that hold ~20.6 m² of free floor,
much of the floor near furniture and skirting is high-cost; the global costmap also grades half-confirmed
Cartographer walls (cells at 51+) as high cost rather than free (`lethal_cost_threshold: 65`,
`trinary_costmap: false`, `nav2_params_robot.yaml:315-316` and comment `:301-314`). Whether the 9 goals
sat inside that band is unknown without the run-3 log.

**B3. Frontier goals may be placed in or behind inflated cells. Hypothesis.**
The explorer filters frontiers by `occ_threshold: 65` on the global costmap (package default,
`frontier_exploration_ros2/config/params.yaml:175-180`; not overridden in `explore_params_robot.yaml`). The
planner then accepts goals in unknown space and plans to within 0.5 m of an unreachable goal
(`planner_server` `tolerance: 0.5`, `allow_unknown: true`, `nav2_params_robot.yaml:349-351`). A frontier
on a wall line therefore yields a path that ends against the wall, which the controller cannot finish.
Frontier suppression (`explore_params_robot.yaml:40`, package `frontier_suppression_attempt_threshold: 2`,
`params.yaml:232`) retries a failing frontier before backing off.

**B4. The re-aimed camera adds close obstacles. Hypothesis, with a dated correlation.**
After `cf74b7b` (camera 4.5° down, threshold 80) "Failed to make progress" went from 0 of 21 goals (run 1)
to 7 of 10 (run 2) and 9 (run 3) (`BUILD_LOG.md:41-42`, `:119-120`; status note `:11`). The camera now sees
floor from 0.24 m (`BUILD_LOG.md:71`), and the threshold margin is thin: 80 gave 0 false hits
but 75 already gave 7, swept on one floor in one light (`BUILD_LOG.md:92`, `:108-109`). Skirting boards and plinths at 0.24–0.5 m now mark obstacles the lidar alone did
not, which pins the robot near walls. The tilt itself was ruled out (4.75° measured vs 4.5°, status note
`:11`); a false-positive rate on this floor during run 3 was not measured.

**B5. Localisation drift. Evidenced in the map, effect a hypothesis.**
"Slightly doubled walls in the right-hand room" (status note `:11`) means Cartographer's pose slipped
during the run. Doubled walls narrow the free space and add inflated cells on both copies.

**B6. CPU load. Hypothesis.**
CPU peaked at 82.7 °C (status note `:11`); earlier load averages were 11–16 with Cartographer ~1.5 cores
(`BUILD_LOG.md:46-47`), and the explorer at upstream defaults once "helped starve Nav2 into timing out
every goal" (`explore_params_robot.yaml:42-45`). Thermal throttling on the Pi would lower the
controller's real rate below `controller_frequency: 10.0` (`nav2_params_robot.yaml:113`). Not measured.

**Ruled out / not a cause for run 3:** TF freeze (tf2 0.25.24 overlay; 0 stale-TF lines in runs 1 and 2,
`BUILD_LOG.md:42-43,120`); camera tilt (status note `:11`).

**Config hygiene (not a cause):** the recovery block is named `recoveries_server` with
`nav2_recoveries/*` plugins (`nav2_params_robot.yaml:357-370`), a pre-Humble name; the running node is
`behavior_server` (`BUILD_LOG.md:133`), so these params are probably ignored and behaviors run on
defaults.

## Fixes, ranked by expected impact on the 5 cm target

| Rank | Fix | Exact change | How to verify next run |
|---|---|---|---|
| 0 (prerequisite) | Make the run analysable | Record a rosbag (`/tf`, `/tf_static`, `/odom`, `/scan`, `/scan_fused`, `/plan`, `/local_plan`, `/cmd_vel`, `navigate_to_pose/_action/status`, `/local_costmap/costmap`) and copy `~/logs/nav/run*.log` to the box after each run | Report can fill the goal table with real numbers |
| 1 | Tighten the goal checker | `nav2_params_robot.yaml:133` `xy_goal_tolerance: 0.15 → 0.05`; keep `stateful: True`; align DWB `:164` (already 0.05) | 10 × 0.5 m forward goals (nav6–8 protocol): end distance by lidar witness ≤ 0.05 m in ≥ 9 of 10; count oscillation/timeouts |
| 2 | An independent, sub-5 cm position witness | Tape-measure or floor-marker ground truth at each goal, plus the lidar-median witness already in `places.drive_verdict`. Longer term: wheel encoders, then `use_odometry = true` in `slam_real.lua:37` | Map pose vs ground truth within 2 cm at rest, and no drift > 2 cm over 60 s stationary (A2 saw 12 cm) |
| 3 | Slow the final approach | Lower speed near the goal: e.g. DWB `RotateToGoal.slowing_factor` / `max_vel_x` 0.15 → 0.08 for precision goals, or switch to Regulated Pure Pursuit with `approach_velocity_scaling_dist`; set `trans_stopped_velocity` `:165` 0.25 → ~0.02 | Overshoot/undershoot distribution over the same 10 goals; stop distance after zero `cmd_vel` |
| 4 | Stop the progress checker killing near-wall goals | `nav2_params_robot.yaml:128-129`: e.g. `required_movement_radius 0.25 → 0.10`, keep 20 s; or a rotation-aware checker (`nav2_controller::PoseProgressChecker` with `required_movement_angle`, if the installed Nav2 has it) | Re-run a 10 min exploration in the same two rooms: "Failed to make progress" count vs 9 |
| 5 | Keep frontier goals out of inflated space | Add `occ_threshold` lower than 65 (e.g. 40) to `explore_params_robot.yaml`; consider `frontier_candidate_min_goal_distance_m` ≥ 0.6; planner `tolerance: 0.5 → 0.2` (`nav2_params_robot.yaml:349`) | Log goal cost in the global costmap per goal; failures near walls should drop |
| 6 | Reduce inflation for a 0.1 m robot | `inflation_radius` 0.5/0.55 → ~0.35, `cost_scaling_factor` 3.0 → ~5.0 (`:203-204`, `:319-320`) | Same exploration run; distance travelled and failure count vs run 3; no contacts |
| 7 | Measure camera false positives near skirting | Re-sweep `edge_threshold` against skirting/plinths in these rooms; compare `/scan_fused` vs `/scan` hits under 0.5 m | Count of camera-only returns under 0.5 m per minute with no real obstacle |
| 8 | Thermal | Fan on the Pi (Claude suggested it on 2026-10-02); log `vcgencmd measure_temp` / `get_throttled` during runs | No throttle flags; controller loop rate ≥ 9 Hz |
| 9 | Config hygiene | Rename `recoveries_server` → `behavior_server` with `nav2_behaviors/*` plugins (`:357-370`) | `ros2 param get /behavior_server ...` returns the file's values |

Fixes 1–3 are what the 5 cm target needs; 4–7 are what run 3's failures need. Fix 1 alone, on today's
pose estimate, will likely trade "success at 15 cm" for timeouts and oscillation, because the pose it
checks against moves by more than 5 cm (A2).

## Gates seen to fail
None run. This is a write-up from records, not a test execution.

## Failures
See Causes. Expected: each goal within 0.05 m. Actual: 9 goals aborted near walls (run 3); successful goals
on 2026-10-01 stopped 0.14–0.27 m short.

## Not verified (and why)
- Every per-goal number for run 3: logs are on the robot only; `ssh tb` was out of scope.
- The run-3 map: on the robot only.
- No rosbag exists for run 3.
- How many goals run 3 sent in total, and whether any succeeded.
- Which Nav2 component reads DWB's `xy_goal_tolerance` and `trans_stopped_velocity` in the installed version.
- Whether the run-3 goals were inside the inflated band or behind walls (B2, B3).
- CPU throttling during run 3.

## Open questions for the owner
1. May someone copy `~/logs/nav/run3.robot.log`, `run3.compute.log` and `~/logs/explore/run3/map.pgm` from
   the robot to the box (needs `ssh tb`)? The goal table can then be filled in from real data.
2. Does the 5 cm target apply to frontier-exploration goals, or only to named-place / navigation goals?
   Frontier goals only need to get the sensor near the frontier; holding them to 5 cm will cost exploration
   coverage.
3. What is the ground truth for "within 5 cm"? Cartographer's pose has been seen 7–12 cm off; a 5 cm claim
   needs a tape, floor markers, or encoders.
4. Are wheel encoders (closed-loop odometry) on the table for Phase 1? Without them the pose is lidar/IMU only.
5. The robot checkout is still `690c45c` + hand-copied files; the sync was denied on 2026-10-02. OK to sync it
   before run 4?
6. The supervisor's SIGINT fix lives only in the robot's `~/logs/explore_run.py`. Bring it into the repo
   before the next run?
