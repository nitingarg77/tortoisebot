# Test report: auto-mapping run 3 (2026-10-02)

Result: **FAIL** against the Phase 1 target (each goal reached within 5 cm). **12 goals sent, 0 reached,
0 within 5 cm.** End distance to target 0.92–2.06 m (median ~1.3 m). The progress checker fired
"Failed to make progress" 9 times, spread over 8 of the 12 goals; the explorer cancelled 11 goals and the
last was aborted by Nav2, then the pose watchdog stopped the run.
Command: frontier exploration via `autobringup.launch.py` + `/control_exploration`, under the scratch
supervisor `~/logs/explore_run.py` on the robot (not in this repo). 10 min run.
Collected / passed / failed / skipped: goals sent: **12** (11 after the supervisor's start) / reached: **0** /
failed: 12 (9 "Failed to make progress" events on 8 goals, 3 cancelled or aborted without one) / skipped: n/a.

Written 2026-10-21 by Claude (CoS task A-004), read-only, from what is on the gpu-box. Updated the same day
(CoS task A-006) from the run-3 logs copied to the box (`/workspace/robotics/tb_ws/robot_logs/run3/`); the
goal table and the claims it tests are now from those logs. Every number below is quoted from a source with
a reference; nothing is reconstructed.

## Evidence, and what is missing

| Source | Where | What it holds |
|---|---|---|
| Session note written at the end of the run-3 session | box: `~/.claude/projects/-workspace-robotics-tb-ws/memory/tortoisebot-status-2026-10-02.md:11` | The only record of run 3 on the box: 10 min, 9.1 m driven, 9 goals "Failed to make progress", each near a wall, last 3.5 min facing the skirting, map ~20.6 m², doubled walls, camera tilt 4.75° vs 4.5°, CPU 82.7 °C |
| Run-3 robot and compute logs | copied to box: `robot_logs/run3/run3.compute.log` (24 942 lines), `run3.robot.log` (508 lines) | Nav2, explorer, Cartographer, watchdog messages (compute); sensor drivers (robot). Cited below as `compute.log:N` |
| Run-3 supervisor log | box: `robot_logs/run3/explore/run.log` (68 lines) | Map pose every ~10 s, distance, load average, stop reason. Cited as `run.log:N` |
| Run-3 map | box: `robot_logs/run3/explore/map.pgm` + `map.yaml` (0.05 m/px, origin −5.65, −3.05, trinary) | End-of-run map; the supervisor's own save failed (`run.log:67`) |
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
| Goal generation | `frontier_exploration_ros2`, params `explore_params_robot.yaml` on top of the package defaults: `frontier_selection_min_distance` 0.5, `frontier_candidate_min_goal_distance_m` 0.5, `mrtsp_solver: greedy`, `goal_preemption_enabled: false`, `frontier_suppression_enabled: true`. The running node reported `occ_threshold=50` and suppression `threshold=3` (`compute.log:337,342`), not the package defaults 65 / 2 assumed in A-004 | `explore_params_robot.yaml:25-26,40,48-49`; `frontier_exploration_ros2/config/params.yaml:180` |
| Supervisor | scratch `~/logs/explore_run.py` on the robot, with the SIGINT fix that exists only there; Ctrl-C stop worked twice | status note `:11,12` |

Run-over-run context: run 1 25.5 m, 21 goals, none aborted; run 2 5.6 m in 322 s, 10 goals, 7 "Failed to
make progress"; run 3 9.1 m in 10 min, 9 "Failed to make progress" (`BUILD_LOG.md:40-42`,
`BUILD_LOG.md:119-120`, status note `:11`). The failure rate rose after the camera was re-aimed down
(`cf74b7b`), which is itself a lead (cause H3).

## Goals

**How goals were counted.** Every Nav2 goal logs one `bt_navigator` "Begin navigating" line:
`grep -c 'Begin navigating' run3.compute.log` → **12**, and `grep -c 'Failed to make progress'` → **9**.
No line matches `succeeded|Reached|SUCCEEDED` for a goal. Goal 1 was sent at 1790938880, before the
supervisor's `start` (`run.log:1-2`, 1790939018); it belongs to the same launch and is included.

**How "final pose" was taken.** Nav2 does not log the robot pose at goal end. The final pose is the
"current location" in the *next* goal's "Begin navigating" line, logged 2–4 s after the cancel, with the
robot stopped ("Goal was canceled. Stopping the robot."). For goal 12 it is the supervisor's sample 0.6 s
after the abort (`run.log:40`). Error = straight-line distance from that pose to the dispatched target.
"Target wall dist." is the distance from the target to the nearest occupied cell (< 50) of the end-of-run
`map.pgm`; "Robot wall dist." the same for the final pose (map has doubled walls, so ± one wall thickness).

| # | Sent (`compute.log`) | Target (map, m) | Final pose (map, m) | Error (m) | Result and end (`compute.log`) | Target / robot wall dist. (m) | Cause (from the log) |
|---|---|---|---|---|---|---|---|
| 1 | `:3205` 1790938880.2, from (0.01, −0.01) | (1.46, 2.03) | not logged at end; (1.52, 1.99) 122 s later, `run.log:1` | n/a (0.07 at +122 s) | Cancelled by explorer `:3548` 896.8 after `follow_path` ack timeout `:3520` | 0.32 / 0.34 | Controller action server too slow to acknowledge (CPU). How the pose moved 2.5 m between 880 and 1018 is not in the logs |
| 2 | `:5024` 1790939020.4, from (1.51, 1.98) | (0.23, 2.28) | (1.51, 2.00) `:6055` | 1.31 | Failed to make progress `:5411`, again `:5975`; cancelled `:6016` 1062.1 | 0.12 (in unknown) / 0.34 | Turning in place at start: yaw 91→99→87° with 0.02 m moved (`run.log:3-7`) |
| 3 | `:6055` 1790939064.5 | (0.23, 2.28) | (1.51, 1.99) `:7424` | 1.31 | Spin `:6527`, Failed to make progress `:6604`, wait `:6800`; cancelled `:7304` 1107.1 | 0.12 / 0.34 | Same frontier retried; ack and costmap-clear timeouts `:6520-6524`, `:6795-6798` |
| 4 | `:7424` 1790939110.5 | (0.23, 2.28) | (1.52, 2.14) `:8805` | 1.30 | Failed to make progress `:8212`; cancelled `:8672` 1140.1 | 0.12 / not computed | Third try at the same frontier (suppression threshold 3, `:342`) |
| 5 | `:8805` 1790939142.4 | (2.93, −0.52) | (1.75, 0.14) `:10690` | 1.35 | Backup `:10607`; cancelled `:10643` 1190.1; no progress-checker failure | **0.03, target on an occupied cell** / 0.65 | Frontier dispatched onto a wall (B3) |
| 6 | `:10690` 1790939192.5 | (−1.15, −1.19) | (−1.14, −0.04) `:11365` | 1.15 | Cancelled `:11288` 1225.1; no progress-checker failure; planner ack timeout `:10817` | 0.39 (unknown) / 0.66 | Explorer gave up after 33 s; no Nav2 error logged |
| 7 | `:11365` 1790939228.6 | (−1.17, −1.16) | (−1.17, −0.10) `:12046` | 1.06 | Failed to make progress `:11955`; cancelled `:11994` 1250.1 | 0.36 / 0.60 | Robot in open floor, target in unknown inside inflation band |
| 8 | `:12046` 1790939252.5 | (−1.17, −1.16) | (−1.20, −0.24) `:12867` | 0.92 | Failed to make progress `:12744`; cancelled `:12827` 1274.1 | 0.36 / not computed | As 7; moved 0.14 m toward the target |
| 9 | `:12867` 1790939276.5 | (−2.47, −1.51) | (−2.45, −0.23) `:14132` | 1.28 | `follow_path` abort `:13382`, Failed to make progress `:13686`, spin `:13844`, "No valid trajectories out of 209" `:13885`; cancelled `:14077` 1333.1 | 0.25 / **0.07** | Robot pinned against a wall |
| 10 | `:14132` 1790939336.5 | (−3.97, −1.45) | (−2.44, −0.25) `:14612` | 1.94 | Failed to make progress `:14518`; cancelled `:14531` 1357.1; "No valid trajectories" `:14239` | 0.47 / **0.06** | Pinned against the same wall |
| 11 | `:14612` 1790939360.6 | (−3.97, −1.45) | (−2.44, −0.23) `:15133` | 1.96 | Failed to make progress `:15019`; cancelled `:15036` 1381.1 | 0.47 / 0.06 | As 10 |
| 12 | `:15133` 1790939384.5 | (−3.97, −1.45) | (−2.34, −0.19) `run.log:40` | 2.06 | Nav2 "Goal failed" `:15424`, explorer STATUS_ABORTED `:15426`; backup "Collision Ahead" `:15435`; pose watchdog stops run `:15455` 1403.7 | 0.47 / 0.16 | Spin, backup and wait all launched within 0.1 s amid timeouts (`:15400-15437`); then a 0.65 m/s pose jump |
| after 12 | — | — | (−2.94, 0.15), `run.log:66` | 1.90 to target 12 | Explorer stopped by watchdog; robot idle 220 s to the 600 s limit (`run.log:41-63`) | — / 0.23 | Not "stuck at the skirting": the watchdog had stopped exploration |

Summary: 12 goals, 6 distinct frontiers (three were retried 2–3 times). 0 within 5 cm, 0 reached; errors
0.92–2.06 m, because no goal was driven to completion. 9 progress-checker failures on 8 goals (2, 3, 4, 7, 8,
9, 10, 11). Goals 1, 5, 6 and 12 ended without one.

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

### B. Why run 3's goals failed (12 sent, 9 progress-checker failures on 8 of them)

**B1. The progress checker abandons slow, turn-heavy approaches. Evidenced in run 3; "near walls" only partly.**
`progress_checker`: 0.25 m in 20 s (`nav2_params_robot.yaml:128-129`). The comment above it records that
"turning in place covers no distance at all, so the stricter check abandoned 11 of 12 frontier goals as
'Failed to make progress'" (`:123-127`, 2026-09-25). Near a wall, DWB must turn and creep; at a real
0.04 m/s, 0.25 m takes ~6 s of straight driving with no turning, so any manoeuvre near a wall with
repeated in-place rotations can exceed 20 s without moving 0.25 m. Run 3's message is exactly
"Failed to make progress" (status note `:11`).
*Run-3 logs:* 9 "Failed to make progress" lines (`compute.log:5411…15019`) on goals 2, 3, 4, 7, 8, 9, 10,
11. On goals 2–4 the robot turned in place at its start point (yaw 81–105°, 0.40 m moved in 120 s,
`run.log:3-14`), 0.34 m from the nearest mapped wall; on 7–8 it was 0.60 m from any wall. Only goals 9–11
failed with the robot against a wall (0.06–0.07 m, with "No valid trajectories" `compute.log:13885,14239`).
So the mechanism holds, but "each near a wall" is true for 3 of the 8.

**B2. The robot is driving inside inflated space by the time it is near a wall. Evidenced (params), effect a hypothesis.**
`robot_radius: 0.1` (`:199`, `:260`) with `inflation_radius` 0.5 local / 0.55 global and
`cost_scaling_factor: 3.0` (`:203-204`, `:319-320`). The explorer's own comment says the costmaps
"inflate to 0.5–0.55 m, so a frontier much closer than that is rarely reachable"
(`explore_params_robot.yaml:22-25`). With 0.55 m of inflation in rooms that hold ~20.6 m² of free floor,
much of the floor near furniture and skirting is high-cost; the global costmap also grades half-confirmed
Cartographer walls (cells at 51+) as high cost rather than free (`lethal_cost_threshold: 65`,
`trinary_costmap: false`, `nav2_params_robot.yaml:315-316` and comment `:301-314`). Whether the 9 goals
sat inside that band is unknown without the run-3 log.
*Run-3 logs:* every dispatched target lies 0.03–0.47 m from an occupied cell of the end-of-run map, i.e.
inside the 0.5–0.55 m inflation band, and all but goal 1 are in unknown cells (goal table). Upgraded to
evidenced for the target placement; the cost value at each target was not logged.

**B3. Frontier goals are placed in or on inflated cells. Evidenced in run 3.**
The explorer filters frontiers by `occ_threshold` on the global costmap. A-004 assumed the package default
65; the running node reported **50** (`compute.log:337`), so the source of that value is unconfirmed. The
planner then accepts goals in unknown space and plans to within 0.5 m of an unreachable goal
(`planner_server` `tolerance: 0.5`, `allow_unknown: true`, `nav2_params_robot.yaml:349-351`). A frontier
on a wall line therefore yields a path that ends against the wall, which the controller cannot finish.
Frontier suppression (`explore_params_robot.yaml:40`, package `frontier_suppression_attempt_threshold: 2`,
`params.yaml:232`) retries a failing frontier before backing off.
*Run-3 logs:* goal 5's target (2.93, −0.52) sits on an occupied cell (0.03 m), and the explorer cancelled it
after a backup (`compute.log:8805-10643`). Suppression ran at `threshold=3` (`compute.log:342`): frontiers
(0.23, 2.28) and (−3.97, −1.45) were each sent 3 times and (−1.17, −1.16) twice, so 7 of 12 goals were
repeats of a frontier that had already failed.

**B4. The re-aimed camera adds close obstacles. Hypothesis, with a dated correlation.**
After `cf74b7b` (camera 4.5° down, threshold 80) "Failed to make progress" went from 0 of 21 goals (run 1)
to 7 of 10 (run 2) and 9 failures on 8 of 12 (run 3) (`BUILD_LOG.md:41-42`, `:119-120`; status note `:11`). The camera now sees
floor from 0.24 m (`BUILD_LOG.md:71`), and the threshold margin is thin: 80 gave 0 false hits
but 75 already gave 7, swept on one floor in one light (`BUILD_LOG.md:92`, `:108-109`). Skirting boards and plinths at 0.24–0.5 m now mark obstacles the lidar alone did
not, which pins the robot near walls. The tilt itself was ruled out (4.75° measured vs 4.5°, status note
`:11`); a false-positive rate on this floor during run 3 was not measured. *Run-3 logs:* fusion reported the
camera used in 62–97% of scans across 41 summaries (`robot.log:128` low, `:411` high), but no per-obstacle source is logged; still a hypothesis.

**B5. Localisation drift. Evidenced, and it ended the run.**
"Slightly doubled walls in the right-hand room" (status note `:11`) means Cartographer's pose slipped
during the run. Doubled walls narrow the free space and add inflated cells on both copies.
*Run-3 logs:* right after goal 12's recoveries the pose watchdog tripped on "pose estimate moving at
0.65 m/s (limit 0.50)" and stopped the explorer (`compute.log:15455`); the map pose moved 0.68 m in 10 s
(`run.log:40-41`), 4× the 0.15 m/s speed cap. The "last 3.5 min facing the skirting" in the status note is
this watchdog stop (220 s idle, `run.log:41-63`), not a stuck robot. The supervisor's 9.1 m includes that
jump. Goal 1's pose also moved 2.5 m between 880 and 1018 with no logged cause.

**B6. CPU load. Evidenced in run 3 (was a hypothesis).**
CPU peaked at 82.7 °C (status note `:11`); earlier load averages were 11–16 with Cartographer ~1.5 cores
(`BUILD_LOG.md:46-47`), and the explorer at upstream defaults once "helped starve Nav2 into timing out
every goal" (`explore_params_robot.yaml:42-45`). Thermal throttling on the Pi would lower the
controller's real rate below `controller_frequency: 10.0` (`nav2_params_robot.yaml:113`).
*Run-3 logs:* load average 11.8–18.1 on the 4-core Pi throughout (`run.log:3-62`); 64 "Control loop missed
its desired rate of 10Hz", 110 "Behavior Tree tick rate 50 was exceeded", 5 planner-loop misses (one at
2.6 Hz against 20 Hz, `compute.log:15431`), and 14 "Timed out while waiting for action server to
acknowledge goal" plus 7 costmap-clear service timeouts. These timeouts broke the recovery tree: goal 1
ended on one (`:3520`), and goal 12 launched spin, backup and wait within 0.1 s (`:15400-15437`). Throttling
flags were not logged, so heat vs plain load is still open.

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
| 4 | CPU load, then thermal (was 8) | Cut load before tuning Nav2: e.g. Cartographer `num_threads` to the cores available (7 requested, more than available: `compute.log:172`), explorer `map_processing_rate_hz` stays 0.5, camera fusion rate; fan on the Pi; log `vcgencmd measure_temp` / `get_throttled` | Zero action-server ack timeouts and < 5 controller-rate misses per 10 min (run 3: 14 and 64); load average < 8 |
| 5 | Stop the progress checker killing near-wall goals | `nav2_params_robot.yaml:128-129`: e.g. `required_movement_radius 0.25 → 0.10`, keep 20 s; or a rotation-aware checker (`nav2_controller::PoseProgressChecker` with `required_movement_angle`, if the installed Nav2 has it) | Re-run a 10 min exploration in the same two rooms: "Failed to make progress" count vs 9 on 8 goals |
| 6 | Keep frontier goals out of inflated space | Set `occ_threshold` explicitly, lower than the 50 in force (e.g. 40) to `explore_params_robot.yaml`; consider `frontier_candidate_min_goal_distance_m` ≥ 0.6; planner `tolerance: 0.5 → 0.2` (`nav2_params_robot.yaml:349`) | Log goal cost in the global costmap per goal; failures near walls should drop |
| 7 | Reduce inflation for a 0.1 m robot | `inflation_radius` 0.5/0.55 → ~0.35, `cost_scaling_factor` 3.0 → ~5.0 (`:203-204`, `:319-320`) | Same exploration run; distance travelled and failure count vs run 3; no contacts |
| 8 | Measure camera false positives near skirting | Re-sweep `edge_threshold` against skirting/plinths in these rooms; compare `/scan_fused` vs `/scan` hits under 0.5 m | Count of camera-only returns under 0.5 m per minute with no real obstacle |
| 9 | Config hygiene | Rename `recoveries_server` → `behavior_server` with `nav2_behaviors/*` plugins (`:357-370`) | `ros2 param get /behavior_server ...` returns the file's values |

Fixes 1–3 are what the 5 cm target needs; 4–8 are what run 3's failures need.

**Ranking after the run-3 logs (A-006).** Fixes 1–3 are unchanged: run 3 reached no goal, so it neither
confirms nor contradicts A1 (15 cm tolerance) or A2 (~7 cm drift); the 14–20 cm figure stays a 10-01 nav6–8
result. The pose watchdog's 0.65 m/s jump (B5) strengthens fix 2. Among run-3 fixes, CPU load moved from 8
to 4: the logs contradict "not measured" (B6) and show timeouts breaking the recovery tree on goals 1 and 12.
The progress checker (now 5) stays high, since it fired on 8 of 12 goals, but its "near walls" framing holds
for only 3 of them. Fix 6 now starts from `occ_threshold` 50, not 65. Fix 1 alone, on today's
pose estimate, will likely trade "success at 15 cm" for timeouts and oscillation, because the pose it
checks against moves by more than 5 cm (A2).

## Gates seen to fail
None run. This is a write-up from records, not a test execution.

## Failures
See Causes. Expected: each goal within 0.05 m. Actual (run 3): 12 goals sent, 0 reached, end errors 0.92–2.06 m; 9 progress-checker failures on 8 goals,
then a pose-watchdog stop. Successful goals
on 2026-10-01 stopped 0.14–0.27 m short.

## Not verified (and why)
- Robot pose at each goal's exact end (taken from the next goal's start, 2–4 s later; see Goals).
- Global-costmap cost at each target (not logged); wall distances are from the end-of-run map.
- No rosbag exists for run 3.
- Which Nav2 component reads DWB's `xy_goal_tolerance` and `trans_stopped_velocity` in the installed version.
- Why goal 1's pose moved 2.5 m between 1790938880 and 1790939018 (nothing logged).
- CPU throttling during run 3 (load is logged, throttle flags are not).

## Open questions for the owner
1. ~~Copy the run-3 logs to the box~~ Done (copied to `robot_logs/run3/`, used by A-006). Keep doing it after
   every run.
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
