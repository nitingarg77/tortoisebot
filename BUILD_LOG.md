# Build log

Dated entries of what was built, what broke, what was measured and what is
still open. Newest first. Numbers are as measured on the day; commit hashes
are on branch `frontier-exploration` unless noted.

---

## 2026-09-29 — working standards from the 09-28 failures, the audit against them, CI pushed over SSH, the camera fused into the lidar scan, and a silent /imu outage the refactor left behind

### Added

- **CLAUDE.md** (`1e3c1f7`): working standards for this repository —
  evidence before assertion, three mandatory guards on anything that commands
  `cmd_vel` (a clearance guard, a did-it-actually-move check, a stop on every
  exit path), testing without the robot, concurrency and determinism,
  resource handling, when a design phase is warranted, calibration belonging
  to this chassis, and what a commit message must state. Each rule is tied to
  a specific 09-28 failure: the CPU-load fix committed against an unmeasured
  cause (`176a4d1`), the left-motor trim reported as working before the
  parameter had reached the motors (`bc38cd7`), a rotation test whose
  evidence was two identical lidar readings from a robot that had not moved,
  and a deadband test with no clearance guard that drove 6.8 m when 3 m was
  asked for. Bug fixes are deliberately exempted from the design-first
  requirement — removing the duplicated voxel layer was four lines and took
  the stack from 0/8 goals to 2/0 (`681766f`, 09-28) — while new subsystems
  get design, then mocked interfaces, then tests.

### Faults found in the audit against `1e3c1f7`, and what was done

An audit of the repository against that standards file failed three of its
eight sections, including one where the standards file was itself wrong
(`379d6eb`).

- **Section 3, testability: nothing here could be imported without a
  robot.** `floor_scan.py`, `differential.py`, `nav_course.py` and
  `pose_watchdog.py` all imported `rclpy` at module scope, and
  `differential.py` imported `RPi.GPIO` too; there were no tests beyond the
  default ament linters. The duty-cycle change in `3356eb2` and the numpy
  rewrite in `176a4d1` had both been checked against logic pasted into
  throwaway scripts rather than the code that ships. The pure logic now
  lives in `tortoisebot_firmware/motor_math.py` and
  `tortoisebot_navigation/floor_geometry.py`, importing nothing but the
  standard library, numpy and cv2; `differential.py` and `floor_scan.py` are
  now thin shells that import those modules, and `floor_scan.py` loses 38
  lines of duplicated geometry. 98 tests run at a desk in under half a
  second, wired into each package via `ament_add_pytest_test` (run with
  `colcon test --packages-select tortoisebot_firmware
  tortoisebot_navigation`). Both rewrites are pinned against the
  implementations they replaced: `old_duty` for the duty cycle,
  `loop_reference` across 40 random frames for the boundary search.
- **CLAUDE.md section 3 was itself wrong.** It claimed `range_from_row` was
  checkable at a desk with no ROS; it was not, since the file it lived in
  imports `rclpy`. Corrected, with the reason kept rather than quietly
  reworded.
- **Section 2, motion guards.** `1ms.py` commanded 1.0 m/s — five times this
  robot's maximum — with no lidar, no clearance check and no stop path, and
  was broken in a way that hid this: `rate.sleep()` with no spinning executor
  blocks before it ever publishes. Deleted; nothing referenced it.
  `wheel_balance.py` (09-28) had no `finally`, so an exception mid-burst left
  the robot driving until `differential.py`'s 1 s `cmd_vel` timeout caught
  it; it now stops on every exit path and checks clearance during the burst
  rather than only before it. `pose_watchdog.py` exited without reasserting
  its stop, releasing a robot that had been halted for a reason; it now
  leaves one last stop behind if it was tripped.
- **`__pycache__` was being committed.** Six `.pyc` files were tracked,
  including one for `floor_scan` that had turned up in an unrelated diff.
  Now covered by `.gitignore`.

### The audit finished: four more modules tested (`2944d9c`)

`379d6eb` (above) had cleared two of eight packages and left the
safety-relevant logic untested; `2944d9c` finishes that audit. Four more
modules, each pulled out of a node that could only be reached with a robot
attached:

- **`runaway.py`**, the pose-estimate runaway detector — the only thing that
  stops a robot whose localisation has diverged, and it had no test at all.
  Its judgement is a balance between two real events, both now pinned: the
  0.33 m scan-matching snap of 09-25 must not trip it, and a 2 m in 2 s
  runaway must. That is why speed is averaged over a window rather than taken
  between samples.
- **`scan_geometry.py`**, reading a scan by bearing. `cone()` is the function
  whose misuse drove the robot 6.8 m on a request for 3 m (the deadband-test
  failure CLAUDE.md section 2 is built on): pointed through a doorway while
  the robot curved, it reported negative progress and the tool built on it
  concluded the motors were dead. The tests encode that run; `is_usable_as_odometer()`
  is the check that was missing.
- **`sample_checks.py`**, the BNO055 gates that decide what reaches
  Cartographer. They fire constantly, so both directions are tested: the
  318 m/s² reading from the logs is rejected, and a real 1.2 rad/s spin is
  not.
- **`places.py`**, the course file. The built-in places are simulator
  coordinates, so a real course is always a file a human typed and a typo
  becomes a goal somewhere unintended; now rejects `.inf` and `.nan`, which
  parse as floats and previously survived.

`pose_watchdog.py`, `imu_node.py`, `nav_course.py`, `calibrate_drive.py` and
`wheel_balance.py` now import these rather than carrying their own copies, so
the tests cover the code that actually runs; the two drive tools keep a thin
`LaserScan` adapter at the boundary, which is what lets the shared module need
no ROS.

`run_tests.sh` runs all of it — 167 tests — locally in about a second. The
matching GitHub Actions workflow (`.github/workflows/tests.yml`) is written
but was **not part of `2944d9c`**: pushing to `.github/workflows/` needs a
token with `workflow` scope, which the one used for this commit does not
have, so the file sits locally, untracked, pending a push with the right
credentials. Until then the 167 tests only run when someone remembers to run
`run_tests.sh`.

**Resolved the same day** (`475a86b`): the gh CLI token carries only `gist`,
`read:org` and `repo`, not `workflow`, so the workflow file was pushed over
SSH instead, which is not an OAuth app and is not subject to that
restriction. The remote itself is left on HTTPS; only this one push used SSH.
The workflow runs the same three suites — `tortoisebot_firmware`,
`tortoisebot_navigation`, `tortoisebot_imu` — on every push and pull request,
and deliberately does not build the ROS packages, since that needs a Humble
container and would turn a one-second check into a slow one nobody waits for;
build breakage stays a robot-side check, logic regressions are now caught
here instead of only when someone remembers `run_tests.sh`.

`MODULE.md`'s "no CI and no tests beyond the default ament linters" is
replaced with what the 167 tests actually cover and, more usefully, what they
do not: pure modules, not running nodes. Nothing here exercises a live graph,
so a node that hangs a costmap the way `floor_scan` hung `controller_server`
(09-28, `5ca83df`) is still only found by driving the robot. CLAUDE.md section
3 gains the same caveat, plus the rule that a new module goes into
`run_tests.sh`/CI or it will rot.

**Not verified on the robot.** It went offline mid-deploy and both SSH paths
timed out, so the rewired nodes have not been built or run on hardware.
`differential.py`, `floor_scan.py`, `pose_watchdog.py`, `imu_node.py` and
`nav_course.py` all now import package modules, and with `--symlink-install` a
pull without a rebuild puts the new sources live against modules that are not
installed — both workspaces need a build on the robot before anything moves.

### The camera-into-lidar fusion left open on 09-28 was written (`8a3e098`)

The 09-28 entry's open item — fuse the camera's ranges into the lidar scan so
each costmap keeps one observation source instead of two — is written, not
yet run. `scan_fusion_node.py` merges `/floor_scan` into `/scan` and publishes
`/scan_fused`, which now replaces `scan` and `floor_scan` as the observation
source on both the local and global costmap in `nav2_params_robot.yaml`. The
decision logic lives in `tortoisebot_navigation/scan_fusion.py`, which imports
no ROS; 43 tests cover it, taking the suite from 167 to 210, still run by
`./run_tests.sh` and CI.

The fusion rule is not `min(lidar, camera)`. `ydlidar.yaml` sets
`invalid_range_is_inf: false`, so this driver encodes a no-return as 0.0; a
plain minimum would return 0.0 at exactly the bearings where the lidar saw
nothing and the camera is the only sensor that can see — the white panels —
and 0.0 is under `range_min`, so `laser_geometry` drops the beam and nothing
is marked at all. The rule is the nearest *valid* return per beam, and a beam
neither sensor saw keeps the lidar's own encoding untouched. Because the
camera can only pull a reading closer and never push one away, `/scan_fused`
is safe to let clear, unlike `/floor_scan`, so a camera false positive now
heals on the next scan instead of being marked permanently. The 33 mm lidar
offset is projected per beam rather than ignored (1.8° of bearing and 3 cm of
range at 0.45 m and 26.75°), and rotation between the camera and lidar stamps
is corrected from the IMU's yaw — 200 ms at 1.2 rad/s smears the fan 13.8°;
translation over the same interval costs 2.6 cm, under one costmap cell, and
is left alone. Unknown yaw is not treated as zero: the fan is then trusted
only inside 50 ms.

Degradation, since this node is now the costmaps' only source where the raw
lidar driver used to be: a dead camera, a stale fan, a fan that reads as a
wall at arm's length, and a fusion exception all fall back to publishing the
raw lidar scan; a dead lidar publishes nothing, since a fabricated scan would
read as clear space. The node logs what fraction of scans the camera
contributed to every 30 s, and it launches on the robot side unconditionally
(not gated on `use_camera`), since gating it there would delete the costmaps'
only source when the camera is off.

**Not yet run on the robot** — it was offline. Whether it cures the
`controller_server` hang is unverified; that needs 20 minutes of exploration
on `/scan_fused` with `controller_server`'s `Transform time` watched. Sim is
untouched: `nav2_params_simulation.yaml` keeps `/scan` and the node does not
launch there, and Cartographer keeps the raw `/scan` either way.

### A way to measure whether the fusion is doing anything was added (`49c68c4`)

The open item above from `8a3e098` — whether `/scan_fused` cures the
`controller_server` hang — still needs the robot, but three narrower
questions under it did not: whether `/scan_fused` keeps up with `/scan`'s
rate, whether the camera ever actually changes a beam rather than the node
being a passthrough, and whether any beam is ever pushed *further* away,
which is the safety property the whole fusion rule rests on — fusion is
meant to only pull a reading nearer, and if it doesn't the costmap could be
clearing real obstacles instead of adding them. `check_fusion.py` subscribes
to `/scan` and `/scan_fused`, matches them by stamp, and asserts those three.
It commands no motion, so it is safe to run while exploring. It cannot
arrange the hang itself — that still needs Nav2 up, a goal in flight, and
`controller_server`'s `Transform time` watched by eye — and is reported as a
manual step rather than asserted. `MODULE.md` now points at it instead of
just describing that manual step.

### /imu had published nothing since 379d6eb, and the check that would have caught it (`4658c53`)

Found the moment the robot came back online, in the bringup log:
`[imu_publisher]: Error: name 'norm' is not defined` — not a warning. The
error is on the quaternion normalisation line, which runs before the
publish, so every sample died there and `/imu` published nothing since
379d6eb. Cartographer lost the gravity direction it aligns its 2D frame to,
and `scan_fusion` (`8a3e098`, above, committed the same day as the break)
lost the yaw it de-rotates the camera fan with, which would have silently
degraded fusion to the 50 ms unknown-yaw window.

`norm` was a local computed by the BNO055 gates; splitting those into
`sample_checks.py` took the local with them and left `imu_node.py` referring
to a name that no longer existed. `sample_checks` already exports `norm`, so
the fix is to import it.

The class of bug is the wider point: `imu_node.py`, like the other node
scripts, imports `rclpy`, `board`, `RPi.GPIO` and `cv2` at module scope, so it
cannot be imported off-robot and the pytest suites never touch it — the gap
left open above as "not verified on the robot." pyflakes parses rather than
imports, so it sees undefined names without a robot or ROS. `check_names.sh`
runs it over all 48 files in `tortoisebot_*` and is wired into both
`run_tests.sh` and `.github/workflows/tests.yml`. It checks only undefined
names, not unused imports: the vendored launch files are full of unused
imports, and a check that shouts about cosmetics gets ignored, and then it
catches nothing. Run against the tree before this fix, it reports exactly one
line: `imu_node.py:162`.

### `/scan_fused` measured on the robot, and the hang question still open (`3b6ebe0`)

`check_fusion.py` (`49c68c4`, above) was run on the robot: 30 s with the
drivers up and nothing navigating. `/scan_fused` kept up with `/scan` — 11.7 Hz
against 11.6 Hz, no scans dropped. The camera changed a beam in 306 of 315
scans (97%), 11,615 beams in total, and 5,907 of those were beams the lidar
saw nothing on at all — the node's own report agrees independently at 96-97%.
No beam was ever pushed further away, the safety property the fusion rule
rests on. The only degradation seen was "no yaw for a 0.08 s gap", 5 times in
~700 scans: the IMU history did not span the interval, and the fan was
skipped rather than de-rotated wrongly, which is the designed behaviour.

This answers the three narrower questions `49c68c4` posed, without yet
needing Nav2. **Still open**, unchanged from `8a3e098`: whether `/scan_fused`
cures the `controller_server` hang, which needs Nav2 up and a goal in
flight — this run deliberately had neither. `MODULE.md` now records the fix
as "written and half-verified on the robot" rather than "not yet run".

### Deployed to the robot, and two things found doing it

The robot had been unreachable all through the 09-29 work, so everything
above was written and tested at a desk. It came back on the same LAN as the
development box (robot `192.168.1.3` / `192.168.1.23`, box `192.168.1.100`,
same gateway), which is what ROS 2 discovery needs; tailscale `ssh tb` works
for a shell but the two are not interchangeable for DDS.

It was 13 commits behind at `d755d0c`. Pulled, then
`colcon build --symlink-install` over `tortoisebot_navigation`,
`tortoisebot_firmware`, `tortoisebot_imu` and `tortoisebot_bringup` — the
build is not optional, because six nodes now import package modules and
`--symlink-install` puts pulled sources live against modules that are not yet
installed. Verified against the *install* space rather than the source: all
seven Python modules import, the entry points are symlinked, and the installed
`nav2_params_robot.yaml` and `autobringup.launch.py` diff clean. 210 tests
pass on the Pi.

**`pkill` on the launch parent orphans every node it started.** Restarting
bringup that way left two complete node sets running, including **two
`differential.py` both driving the motor GPIO**. It also produced a false
measurement that was nearly believed: `/scan_fused` read 23.4 Hz against
`/scan`'s 11.6, exactly double, and `/floor_scan` read 10.1 Hz against a
configured 5 — two of each node, while only one lidar driver could hold
`/dev/ttyUSB0`. The numbers recorded under `3b6ebe0` are from a clean
single-instance restart. Kill by PID over `pgrep -f "install/(tortoisebot|ydlidar)"`
plus `camera_ros/camera_node`, which lives under `/opt/ros` and is missed by
an `install/` pattern; and bracket every pattern (`[a]utobringup`) or `pkill`
matches the ssh command string and kills its own shell mid-loop, which it did
three times.

**`cam_ws` on the robot was 4 commits stale**, missing `station/` entirely and
carrying an install space dated 09-22 against changed `counter_node.py` and
`overlay.py`. Nothing pointed at it; it turned up only on an explicit
box-versus-robot comparison of both repositories. Pulled and rebuilt. Box and
robot now agree at `9470f1c` / `e9a5ca3`, both clean.

### Navigation run: the hang is NOT cured by fusing to one source (`c106c62`)

Nav2 and Cartographer were brought up with `/scan_fused` as the **only**
observation source on both costmaps — confirmed in the log, `Subscribed to
Topics: scan_fused`, once per costmap, with every lifecycle node active and no
errors. One `NavigateToPose` goal 0.80 m ahead was sent. Over the 90 s it was
live:

| | |
|---|---|
| `Transform data too old`, `map` → `odom` | **895 lines** |
| Distinct `Transform time` values | **1** — `1790688350.982`, unchanging |
| `Data time` in the same lines | advanced normally, `…357` → `…447` |
| `Failed to make progress` | 5, each followed by `Aborting handle` |
| `Resulting plan has 0 poses` | **0** — the 09-28 voxel fix held |

One `Transform time` across 90 s is a frozen buffer by this repository's own
definition. So the "second `tf2_ros::MessageFilter`" hypothesis that
`8a3e098` was built on is **wrong, or not the whole cause**. The A/B behind it
was one run each and was recorded as indicated rather than proven; it should
now be read as refuted. The fusion node still does what it was separately
measured to do — 5,907 lidar-blind beams recovered — but it does not fix this.

What it cost in practice: the robot drove while `controller_server` reported
`Failed to make progress`, because Nav2 was steering against a pose frozen 7 s
before the goal was even sent. It overshot the 0.80 m goal and curved off
heading, which is what a frozen `map` → `odom` looks like from outside — it
does not stop the robot, it blinds Nav2 to the fact that the robot is moving.

**Two process errors of mine, both worth keeping.** The goal ran for its full
`TIMEOUT = 90` when a 0.80 m goal at DWB's speed needs about 6 s; a timeout
should be derived from the distance, not a flat constant. And interrupting the
`ssh` that launched it did **not** stop the remote process — `ssh` without a
TTY does not reliably signal it — so the robot kept driving after the command
was cancelled locally. The script's `finally` did cancel correctly at exactly
+90 s, so the guard worked; the leash was too long and the kill path was an
illusion.

**Not yet distinguished:** (a) Cartographer stopped publishing `map` → `odom`,
or (b) only `controller_server`'s listener stopped being served. Cartographer
was alive throughout (7,594 log lines, clean exit), which rules out a crash but
not a stall. `scripts/tf_watch.py` (`c106c62`) was added for exactly this: a
third process with its own tf2 buffer and its own raw `/tf` counter, so it can
say whether the wire kept delivering while a buffer stood still. It commands no
motion, and the freeze happened while idle, so it needs no goal.

Recorded but **not** claimed as cause: the buffer's last update fell within
about a second of the test script starting, which added a TF listener and two
sensor-QoS subscriptions to a Pi already at load 11.6.

### Cartographer is ruled out; the freeze is inside `controller_server` (`c57046f`)

`tf_watch.py` (`c106c62`, above) was run for 100 s with Cartographer up, Pi at
load 11.15: 11,823 `map → odom` messages on the wire, 118 Hz, and the
independent buffer in `tf_watch.py`'s own process never froze — longest gap
0.0 s. So the transform is not missing and SLAM is not stalling: **the freeze
is inside `controller_server`'s process**, explanation (b) from `c106c62`,
now with direct evidence rather than inference, and it happened with a single
observation source, so the `MessageFilter` count is not the trigger.

**Caveat.** Nav2 had failed to activate during this run, so `controller_server`
was not querying `map → odom` and could not be watched in the same window. The
wire measurement stands; the simultaneous observation — this buffer healthy
while `controller_server`'s is frozen, at the same time — does not exist yet.

Nav2 failed to activate because of another orphan: two `planner_server`
processes, one 1,356 s old from the previous launch, stayed alive because the
cleanup pattern never matched the launch parent's real command line and the
survivor was never force-killed — the second time in this session that
orphaned nodes produced a misleading reading (the first was the doubled
`differential.py`/`/scan_fused` instances found deploying, above).

**Next, not yet run down:** 118 Hz is itself higher than the 66 Hz measured on
09-28. Cartographer's `pose_publish_period_sec` sets the rate, and a listener
that cannot keep up with that flood is a candidate cause of the starvation
rather than a victim of it.

### Open

- **Why `controller_server`'s buffer freezes is still unknown.** `c57046f`
  confirms the freeze is inside that process (ruling out Cartographer) but not
  why. Still needed: the simultaneous observation (this process's buffer
  healthy while `controller_server`'s is frozen, in the same run), and a look
  at whether 118 Hz `map → odom` — faster than the 66 Hz seen on 09-28 — is
  outrunning the listener.
- **The orphan-cleanup pattern misses `planner_server` from a previous
  launch.** A 1,356 s old process from an earlier run kept Nav2 from
  activating during the `c57046f` test; the pattern needs to match the launch
  parent's real command line, or the survivor needs to be force-killed.
- **The camera's contribution has only been measured stationary.** 97% of
  scans and 5,907 lidar-blind beams were recorded on a robot that was not
  moving. Driving changes the exposure, the floor texture and the yaw-rate
  term all at once.
- **CI does not run on the default branch.** The repository's default is
  `ros2-humble`; the workflow exists only on `frontier-exploration`, so
  `gh run list` reports nothing and the checks are invisible from the
  repository front page. Merging is a branch decision, not made here.
- **Tests still cover the logic, not the nodes.** `check_names.sh` closes the
  undefined-name hole that hid the `/imu` outage, but nothing exercises a
  running graph, so a node that hangs a costmap is still only found by
  driving.

## 2026-09-28 — floor_scan: a second obstacle source for what the lidar can't see, why it hangs Nav2, a global costmap that discarded Cartographer's walls, and a drivetrain that curves under a straight command

### Faults found, and what was done

**Commanded straight, the robot curves, and a stopped wheel still ran at 15%
duty** (`3356eb2`). Measured with the IMU over a 2.66 m run: 55° of heading
change, about 20 deg/m, roughly 6% more speed on one wheel than the other; it
curves left going forward and right in reverse, one fault seen from both
directions. That is enough to walk the robot into whatever is on its left
over a few metres, and is the likely reason it kept clipping wall panels,
wedged itself in a doorway, and failed nearly every Nav2 goal with "Failed to
make progress": the controller computes a path the robot cannot drive. There
are no encoders, so nothing notices. `left_trim` and `right_trim` are now the
only correction available, both defaulting to 1.0 so this commit alone
changes nothing until they are set; the calibration procedure is in the code
comment. Separately, `min_pwm_val` was applied to every wheel including one
asked for zero, so a stopped wheel still ran at 15% duty and the robot could
not hold a wheel still or drive a gentle arc — a floor on the duty cycle only
means anything once the wheel is meant to turn. The new `duty_for()` returns
0.0 for a wheel commanded to exactly 0.0 and is otherwise identical to the old
formula, checked at every speed from 0.01 to 0.30 m/s. `min_pwm` is now a
parameter too, left at 15: the IMU says 0.6 rad/s (25% duty) does not move the
robot at all while 1.0 rad/s (42%) gives 1.19 rad/s actual, so the real
stiction threshold sits between 25% and 42% and 15 never reached it — raising
it changes low-speed behaviour and wants its own measurement.

**White fluted wall panels are invisible to the lidar** (`6703944`). Measured
on the robot facing a panel at 1.2 m: the beams straight ahead returned
nothing in 55-80% of scans, because the ribs reflect the beam away. Nav2
never saw the wall, and the robot drove into it during exploration.

**floor_scan subscribed to the wrong image topic in bringup** (`ce7aadf`).
`camera_ros` publishes under its node name; bringup names it `camera_node`,
so images are on `/camera_node/image_raw` while floor_scan defaulted to
`/camera/image_raw`. It started and ran but subscribed to nothing and
published nothing, leaving the costmaps waiting on it. `autobringup.launch.py`
now passes `image_topic: /camera_node/image_raw` explicitly.

**Freezing exposure to keep the tuned threshold valid instead froze the
camera black** (`0c17b6b`). `61d8a21` turned auto exposure and auto white
balance off so the view floor_scan tuned against would not change level; with
both off the camera never exposes at all, every frame came out at mean
brightness 1 of 255, and floor_scan read the whole fan as clear. Auto
exposure and auto white balance are on again. Instead, floor_scan scales each
frame to a fixed median brightness before measuring edges, so the tuned
threshold means the same thing whatever the exposure does.

**floor_scan and an unthrottled camera starved Nav2's transform listener,
freezing `map → odom` mid-run** (`176a4d1`). Exploration stalled: the robot
sat at (0,0) while bt_navigator accepted goal after goal 3 m away and
controller_server answered "Reached the goal!" within 100 ms of each one. Its
log showed "Transform data too old", against a transform time that never
advanced, even though Cartographer was still publishing `map → odom` at 17 Hz
with current stamps — controller_server's own transform listener had stopped
being served 107 s into the run and never caught up, so the goal checker
thought the frozen pose was already at each goal. The Pi had load average 17,
3% idle: the camera free-ran at 18-30 fps and floor_scan deserialised every
900 kB frame although it only uses 5 of them a second, and `_boundary_rows`
walked 60 columns x ~200 rows in Python every tick. The camera is now capped
to 10 fps with `FrameDurationLimits` (measured 9.97 Hz on the live node), and
`_boundary_rows` is now numpy, checked against the old loop on 400 random
frames across run lengths 1-5 and every horizon, identical every time.

**A second costmap observation source hangs `controller_server`** (`5ca83df`).
Adding `floor_scan` beside `scan` in `observation_sources` stopped the robot
dead in a way that read as a navigation bug rather than a crash: Nav2
accepted each frontier goal and reported it reached within 100 ms, three
metres away, without moving. Two runs an hour apart, same stack and load,
differing only in `camera:=True/False`, separate cause from symptom — with
`floor_scan` the robot covered 0 m in 20 minutes and the lifecycle manager
eventually tore Nav2 down on a missed bond heartbeat; without it the robot
drove 3.7 m in the first two minutes and kept exploring. It is not CPU and
not the network: `/tf` held 66 Hz and under 0.1 s old for five minutes,
checked against an independent tf2 buffer in another process that never
froze — only `controller_server`'s own buffer stopped advancing. Capping the
camera to 10 fps and rewriting `_boundary_rows` in numpy (`176a4d1`, above)
cut load average from 17 to 5.3 and changed nothing. What turns the hang into
a silent false success is that Nav2 Humble's `ControllerServer::isGoalReached()`
discards the return value of `transformPose`: once `map → odom` can't be
resolved, the goal silently becomes the origin of `odom`, and the same frozen
buffer pins the robot's own reported pose near that origin too, so both fall
inside `xy_goal_tolerance`. Written up in `MODULE.md` as indicated, not
proven: the explanation offered there — a second observation source doubles
the `tf2_ros::MessageFilter` count on `Costmap2DROS`, from two filters to four
because the global costmap lists both sources under two layers — is a
hypothesis, and the A/B is one run each.

**The global costmap's voxel layer duplicated `obstacle_layer` and could not
keep up on the real robot** (`681766f`). It listed `obstacle_layer` and
`voxel_layer` with identical observation sources, ranges and clearing
settings; a single-plane lidar gives a voxel layer no vertical information,
so it marked the same cells the obstacle layer already had, across 16
z-levels, and published a voxel grid nothing subscribes to. The local
costmap never had it. On the real robot the global costmap could not keep
up: planner_server ran at 1.6-5 Hz against its 20 Hz target, so the
behaviour tree's compute_path_to_pose call timed out and FollowPath was
handed an empty path — "Resulting plan has 0 poses in it", 763 times in one
run. Every goal aborted, recovery cleared the whole costmap, and the next
rebuild was slower still; eight goals in a row failed and none succeeded
while the robot drifted on recovery spins. The voxel layer's discarded-below
`origin_z = -0.2` carries over as `obstacle_layer`'s `min_obstacle_height`,
kept for the reason it was there before: nothing should arrive below zero on
a planar scan, but the margin is what stops a change to Cartographer's
tracking_frame from silently dropping scans. Map from the run this replaces
is saved on the robot as `~/maps/site_map_2035`.

**The global costmap's `static_layer` discarded 99% of Cartographer's
walls** (`2f03153`). Cartographer publishes an occupancy probability, not a
yes/no: of 3150 wall cells measured on the live map, 55.7% sit at 51-64,
15.1% at 90-98, and only 21 cells — 0.7% — reach 100. `static_layer` was
left on Nav2's defaults, `trinary_costmap true` with
`lethal_cost_threshold 100`, so it kept those 21 cells and turned every
other wall into free space; checked against the running costmap, 61.8% of
Cartographer's walls were marked free there, and only 14.4% lethal, and
those came from the obstacle layer marking them live rather than from the
map. The planner's whole knowledge of walls was therefore the obstacle
layer's 2.5 m reach — anything further off it routed straight through, and
the controller then could not follow the path it was given. Separate from
the voxel layer removed in `681766f`, and predates it. `lethal_cost_threshold`
is now 65, which keeps 44.3% of wall cells lethal outright, and
`trinary_costmap` is now false, so the rest is graded instead of discarded:
a cell at 51 costs 199 of 254 — avoided, not impassable — which matters
while mapping, when most walls are only half-confirmed. Not yet verified on
the robot: the run in progress at the time was still on the old setting.

**`ros2 param set left_trim` changed the parameter store but not the running
motors** (`bc38cd7`). `left_trim`, `right_trim` and `min_pwm` were read once
in `__init__`, so a live `ros2 param set` had no effect until the node was
restarted — found while attempting the calibration `3356eb2` left open: the
drift on the next run came out six times smaller, and was briefly taken for
the trim working, when it was run-to-run variation and the trim had never
been applied. Calibration is drive, measure, adjust, drive again, which
restarting the node between attempts would make unusable, so `differential.py`
now takes `left_trim`, `right_trim`, `min_pwm` and `cmd_vel_timeout` from an
`add_on_set_parameters_callback` and logs the values whenever they change.

**Per-wheel duty differs enough to explain the curve on its own** (`d755d0c`).
Pivoting on one wheel at a time and reading the IMU proved repeatable where
whole-run drift measurement was not: the right wheel managed 0.140 and
0.122 m/s across two tries, the left 0.116 and 0.107, a ratio of 0.83-0.88.
Untrimmed out-and-back runs gave +32.6 deg/m forward and -42.4 deg/m reverse
with spreads of ±36 and ±28 — too erratic for a single run to mean anything,
which is how an early measurement of +7.0 deg/m and a later one of +1.1 had
looked like a fix when nothing had changed. `left_trim` is now set to 1.28 in
`autobringup.launch.py`, picked by measurement rather than arithmetic: 1.25
gave -2.9 deg/m forward and 1.31 gave -3.0, against -1.5 at 1.28. At 1.28 the
runs read -1.5 deg/m forward and +0.3 reverse with a spread of ±0.5 — the
spread collapsing matters more than the mean, and the leftover 1-3 deg/m is
attributed to floor, slip and battery sag rather than the wheels. The value is
specific to these motors and will not carry to another chassis.

### Added

- **floor_scan** (`6703944`, `tortoisebot_navigation/scripts/floor_scan.py`):
  finds the row where the floor ends in the camera image and converts it to a
  distance using the camera's fixed height and tilt, publishing a LaserScan
  that Nav2 can take as a second obstacle source alongside the lidar.
  Calibrated against the lidar on the real robot: camera tilt measured 4.3°
  up (the URDF says level, the bracket is not), so the useful range runs from
  about 0.45 m to 2.5 m. The geometry reproduces the measured case, junction
  row 360 of 480 at 1.19 m, to within 5 cm. Colour segmentation was tried
  first and rejected: the panel is the same white as the glossy floor, which
  mirrors it, so the mask was unusable; a vertical brightness gradient finds
  the junction regardless of colour.
- **Tuned against a sweep on the real floor** (`0f6504a`): `edge_threshold`
  raised from 18 to 45. 18 fired on the marble floor's veining (61 false
  detections in 180 beams), 30 still fired 10 times, 45 fired none while
  keeping 177/180 real detections, and 60 started missing the junction
  itself. Columns that disagree with their neighbours by more than 12 rows
  (`neighbour_tolerance_rows`) are now dropped as outliers. The same sweep
  refined the camera tilt from 4.3° to 5.0°, moving the junction row's
  mapped distance from 1.09 m to the lidar's measured 1.19 m.
- **Fed into the costmaps and started with the camera** (`61d8a21`):
  `/floor_scan` joins `/scan` as an observation source in all three costmap
  layers (local obstacle, global obstacle, global voxel), marking only
  (`clearing: False`) — it is a 53° fan, blind closer than ~0.45 m and
  unreliable past 2.5 m, so it must not be trusted to declare space free; the
  lidar keeps that job. floor_scan now starts alongside the camera in
  bringup. The camera drops to 640x480; auto exposure and auto white balance
  were turned off here so floor_scan's thresholds, tuned against the real
  floor's brightness, would not drift as the view changed, but that turned
  out to blank the image instead and was reverted the same day (`0c17b6b`,
  above). Measured against the lidar after tuning: 1.15 m vs 1.18 m on the
  fluted panel the lidar can barely see, 53 of 60 beams reading, no false
  near returns.
- **Drive calibration tools** (`d755d0c`,
  `tortoisebot_firmware/scripts/wheel_balance.py` and `calibrate_drive.py`):
  `wheel_balance.py` pivots on one wheel at a time and reads the IMU's yaw
  rate as a direct measure of that wheel's speed — usable now that a wheel
  commanded to zero actually gets zero duty (`3356eb2`). `calibrate_drive.py`
  drives out-and-back legs and reports each run's drift in deg/m rather than
  an average alone, guarded against driving into anything by the lidar.
  Installed alongside the driver rather than left ad hoc, since the trim
  value is specific to these motors and recalibration will be needed again
  after a motor change.

### Open

- **The camera observation source stops the robot from exploring at all**,
  per the `controller_server` hang above (`5ca83df`). Proposed fix, untried:
  fuse the camera's ranges into the lidar scan and publish one topic, nearer
  of the two per bearing, so the costmap keeps a single observation source
  instead of two. The lidar sits at (-0.033, 0, 0.167) from `base_link` with
  zero rotation, so bearings map straight across.
- **The global costmap's new `lethal_cost_threshold`/`trinary_costmap`
  setting (`2f03153`) is not yet verified on the robot** — the run in
  progress when it was made was still on the old setting.
- **`min_pwm`'s real stiction threshold (`3356eb2`) is unmeasured** — it sits
  somewhere between 25% duty (0.6 rad/s commanded, no rotation) and 42%
  (1.0 rad/s commanded, 1.19 rad/s actual); `min_pwm` is left at the
  inherited 15 pending that measurement.

## 2026-09-25 — second machine, frontier exploration, and a Pi that cannot keep up

### Machines

| Name | Hardware | Role | Address |
|---|---|---|---|
| `raspberry-pi` | Raspberry Pi 4 B, 8 GB, Ubuntu 22.04, kernel 5.15 raspi | on the robot: drivers | 192.168.1.6 (also .23), Wi-Fi |
| `nitin-linux2` | desktop, 12 cores, RTX 3060 | SLAM, Nav2, explorer, RViz | 192.168.1.100, wired |
| `linux-mac` | MacBook Air 2015 (MacBookAir7,1), i5-5250U, 3.7 GB, Ubuntu 22.04 | operator station | 192.168.1.18, Wi-Fi; Tailscale 100.99.197.70 |

All three run ROS 2 Humble with the defaults: domain 0, Fast DDS. Nodes
discover each other across machines with no extra configuration (checked
with talker/listener both ways, then `robot_description`, `tf_static` and
`joint_states` from the robot).

Clock offset between the Pi and the desktop is under 5 ms (IMU messages
arrive with a median age of −1 ms). Lidar messages arrive ~113 ms after their
stamp, with Wi-Fi spikes up to 0.7 s.

### Setup notes

- **`tb_ws` on the laptop is built there, not copied.** `--symlink-install`
  bakes absolute `/workspace/robotics/tb_ws/...` paths into `install/`, and
  the laptop has no `/workspace`. `src/` is rsynced and built in place
  (51 s, `--parallel-workers 2`). `ros-humble-robot-localization` had to be
  installed there.
- **The Pi's workspace is built without `--symlink-install`, and must stay
  that way.** Switching modes fails in `ament_cmake_python` ("existing path
  cannot be removed: Is a directory") and leaves the mode recorded in each
  package's `CMakeCache.txt`. Recovering means deleting that package's
  `build/` and `install/` directories. This machine hit the same thing
  for `tortoisebot_firmware`, which was first built in copy mode on 09-16.
- **Latched topics need matching QoS on the command line.**
  `robot_description` and `tf_static` are published once, transient-local:
  `ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /tf_static`.
  Without those flags, before the topic name, the echo prints nothing and looks
  like a network fault.
- **RViz on the laptop** runs, but the map/costmap shader fails to link on Mesa
  ("active samplers with a different type refer to the same texture image
  unit"), so costmaps render as rainbow stripes. Software rendering does the
  same. RViz now runs on the desktop's RTX 3060, where it renders correctly.

### Faults found, and what was done

**Pi Wi-Fi dropped out.** `brcmfmac: CMD53 sg block write failed -84` at
11:48; the Pi kept running ROS with no network until power-cycled. Wi-Fi power
saving was on. It is now turned off at every boot by a unit installed on the Pi
only (not in this repo), `/etc/systemd/system/wifi-powersave-off.service`:

```ini
[Unit]
Description=Disable Wi-Fi power saving on wlan0 (brcmfmac drops out with it on)
After=sys-subsystem-net-devices-wlan0.device netplan-wpa-wlan0.service
BindsTo=sys-subsystem-net-devices-wlan0.device

[Service]
Type=oneshot
ExecStart=/usr/sbin/iw dev wlan0 set power_save off
RemainAfterExit=yes

[Install]
WantedBy=sys-subsystem-net-devices-wlan0.device
```

Verified across a reboot (`Power save: off`). No dropout since.

**IMU spikes moved the map 240 m** (`b615fab`). The BNO055 intermittently
returns ~318 m/s² (32 g, beyond its own 4 g range) on a stationary robot,
always about the same value, which points at the I2C bus or wiring rather
than noise. These samples passed the quaternion check, and Cartographer, with
no wheel odometry, integrated them into a 240 m jump in `odom → base_link`.
`imu_node.py` now drops samples with |accel| > 19.6 m/s² or |gyro| > 8.7 rad/s.
Afterwards, a stationary robot held within 3 cm for 3 minutes, and 15+ spikes
were rejected in that time.

**Motors kept the last command forever** (`f1c3f8b`). `differential.py` is
open-loop PWM and held the last duty cycle until a new `cmd_vel` arrived. A
Wi-Fi drop, a crashed controller or a closed teleop left the robot driving.
It now stops after 1 s without `cmd_vel` (`cmd_vel_timeout`, 0 disables).
Tested on the robot: PWM 15 on command, 0 at 1.2 s. On shutdown it zeroes the
duty cycle and calls `GPIO.cleanup()`, because the software-PWM thread could
otherwise leave an enable pin latched high.
**Keyboard teleop now needs the key held**; a single tap moves the robot for 1 s.

**The Pi 4 cannot run SLAM and Nav2 together.** With Cartographer, Nav2,
explorer and camera: load average ~13 on 4 cores, 67–70 °C, soft temperature
limit reached (`get_throttled 0x80000`), and 231 behaviour-tree overruns. Nav2
servers timed out acknowledging each other and every goal failed. Once the
robot moved, Cartographer's pose ran away (−138 → −431 → −559 m). Measures:

| Change | Commit | Effect |
|---|---|---|
| `camera:=False` launch argument | `dcf3dfa` | camera cost ~21% CPU; unused by exploration |
| Explorer: 0.5 Hz, greedy MRTSP, no preemption | `dcf3dfa` | was ~28% CPU |
| Cartographer: search window 0.1 → 0.3 m, motion filter 1° / 5 cm / 1 s, optimise every 90 nodes | `244f9f6` | hand-driven 6.5 min, ~40 m loop: no runaway, loop almost closed, 16 submaps instead of 35 |
| Nav2: DWB 10×20 samples (was 20×100), local costmap 5/2 Hz, BT loop 20 ms, trajectory debug and Groot off | `88c7917` | see tests B below |
| `stack:=robot` / `stack:=compute` | `dcf3dfa` | SLAM and Nav2 on the desktop instead |

Even with all of the above, the full stack on the Pi (test B) overloaded it
until SSH stopped answering within a minute. **Running SLAM and Nav2 on the
Pi 4 is not viable; the split setup is the working configuration.**

**Nav2 abandoned goals for lack of progress** (`88c7917`). With open-loop
motors the robot averaged 0.04 m/s while exploring, and turning in place
covers no distance, so the 0.5 m in 10 s progress check abandoned 11 of 12
goals. Now 0.25 m in 20 s.

**Glass is invisible to the lidar.** In test A the robot pushed against a
glass wall for about a minute ("Failed to make progress") until it was lifted
and moved by hand. The explorer also sees open space through glass, so the
frontier beyond it stays attractive.

### Added

- **Frontier exploration** (`dcf3dfa`): [mertgulerx/frontier_exploration_ros2](https://github.com/mertgulerx/frontier_exploration_ros2),
  cloned into `tb_ws/src/frontier_exploration_ros2` at `ec530d2` on each
  machine. **That clone is not tracked by this repo.** `autobringup` starts
  it 30 s in, with Nav2, when `exploration:=True`. It idles until started
  unless `auto_explore:=True`. Tuning is in
  `tortoisebot_navigation/config/explore_params_robot.yaml` (base frame
  `base_link`, 0.15 m/s, 0.5 m minimum goal distance, settle after each goal,
  suppression of repeatedly failing frontiers).
  Start or stop it with `ros2 run frontier_exploration_ros2 frontier_exploration_ctl start|stop`,
  or from the RViz panel `frontier_exploration_ros2_rviz/ExplorationControlPanel`.
  The panel has to be built separately with
  `--base-paths src/frontier_exploration_ros2/plugin/frontier_exploration_ros2_rviz`.
- **Pose watchdog** (`dcf3dfa`, `tortoisebot_navigation/scripts/pose_watchdog.py`).
  The wheels cap the robot at 0.20 m/s (65 mm, 60 rpm). If `odom → base_link`
  averages more than 0.5 m/s over 2 s, it stops the explorer, cancels all
  NavigateToPose goals and publishes zero `cmd_vel`. It stays tripped until
  `~/reset`. A 0.5 s window tripped on a one-off 0.33 m scan-matching
  correction, which is why it averages over 2 s. Checked offline against
  synthetic TF: a 0.33 m step does not trip it; a 1 m/s runaway trips it
  within ~1 s. In test A it tripped when the robot was carried by hand.
- **Launch arguments** on `autobringup.launch.py`: `camera`, `stack`
  (`all` | `robot` | `compute`), `rviz`, `auto_explore`.

### Exploration tests

| Run | Setup | Result |
|---|---|---|
| 1 | all on Pi, camera on | stopped: pose jumped to (142, −190) m before the explorer started (IMU spikes) |
| 2 | all on Pi, IMU filter | no goal reached, every one failed; Nav2 timeouts, load 13 |
| 3 | all on Pi, no camera, lighter explorer | robot drove a few metres, then pose ran away to −559 m; stopped by shutting down bringup |
| 4 | hand-driven, `slam_only`, tuned Cartographer | ~40 m in 6.5 min, no runaway, loop almost closed |
| 5 | split (drivers on Pi, rest on desktop) | 1 of 7 goals reached, 11 "Failed to make progress", no runaway; Pi load 1.4, 52 °C |
| A | split + lighter Nav2 + relaxed progress check | 1 goal reached, 3 no-progress, stuck on glass, watchdog tripped when carried |
| B | all on Pi + lighter Nav2 | not run: Pi stopped answering SSH within a minute of bringup |

### How to run it now

On the Pi:

```bash
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False stack:=robot camera:=False
```

On the desktop:

```bash
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False stack:=compute rviz:=False exploration:=True
```

Then run RViz on the desktop with a copy of `nav2.rviz` that adds the
Exploration panel and the `/explore/frontiers` markers (`~/tb_explore.rviz`
on the desktop, not in this repo).

To stop everything on the robot: `ssh nitin@192.168.1.6 bash ~/stop_bringup.sh`.
That script (Pi only) interrupts the launch and then any orphaned nodes.
`ros2 launch` sometimes exits on SIGINT and leaves children running.

### Open

- **Glass.** Put a strip of paper or matte tape on the glass at lidar height,
  or add a Nav2 keepout mask for mapped rooms.
- **IMU wiring.** The 318 m/s² spikes and non-unit quaternions (~50 in
  5 min) are filtered, but they point at the I2C connection.
- **Pi cooling.** It reaches 67–70 °C under load; add a heatsink or fan.
- **RViz dies when Cartographer restarts** (desktop and laptop, several
  times). Close RViz before restarting SLAM and reopen it afterwards.
- **`nav2.rviz` Image panel** subscribes to `/camera/image_raw`, but the camera
  publishes `/camera_node/image_raw`.
- **Nav2 changes are only validated in exploration so far.** The 10-goal
  course (MODULE.md §5) ran on 09-24, headless in simulation, before these
  changes.
  Rerun it after `88c7917`.
- **Pi's `.git`** was not yet synced to `frontier-exploration` (it was off
  charging); its working tree already matches.

## 2026-09-24 — 10-goal navigation course, and the TF remap question settled

All runs in simulation: headless Ignition + Nav2 on an isolated
`ROS_DOMAIN_ID`, because a second, unrelated sim was running on the machine.

### Added

- **`tortoisebot_navigation/scripts/nav_course.py`** (`39b7b81`): ten
  NavigateToPose goals through the five named places, reporting each goal's
  status, duration and distance from target. A course rather than one goal,
  because the 09-21 TF fault passed a single-goal test. Place coordinates and
  the `amcl_pose` QoS come from `spike_needle_nl` (09-23).
- **`--namespace`** (`8bcf45d`): rclpy does not honour `ROS_NAMESPACE`, so the
  node is constructed in the namespace instead.
- **`--record FILE` / `--places FILE`** (`067b63e`). The built-in places are
  simulator coordinates. `--record` drives nothing: push or drive the robot to
  a spot, name it, and its `/amcl_pose` goes into a YAML file. `--places` runs
  the course against those coordinates, in file order. Checked on a live
  headless stack: one place recorded, a two-goal course from the file passed
  2/2, and a malformed file reports which entry is wrong instead of raising.

### Results

| Configuration | Commit | Goals | Worst error | TF staleness |
|---|---|---|---|---|
| default, no namespace (remaps conditional, as in `dc8b7cc`) | `39b7b81` | 10/10 | 0.12 m | none |
| `use_namespace:=True` (remaps applied) | `8bcf45d` | 10/10 | 0.28 m | none |
| remaps unconditional as in `ddac963`, no namespace, single `/clock` publisher | `72aa2f4` | 10/10 | 0.13 m | none |

Thirty goals, three configurations, no reproduction of the 09-21 failure.
**The `/tf` remaps are not the cause.** The two-`/clock`-publishers artifact
found on 09-23 explains the original 1–4/10 runs better. `dc8b7cc` stays,
because without a namespace it is a no-op, but MODULE.md §5 no longer presents
it as a fix for a known defect.

### How to run

```bash
python3 nav_course.py --laps 2 --timeout 120
python3 nav_course.py --namespace robot1        # with use_namespace:=True
python3 nav_course.py --record my_room.yaml     # name each spot in turn
python3 nav_course.py --places my_room.yaml --laps 1
```

Check `ros2 topic info /clock` shows `Publisher count: 1` first.

## 2026-09-23 — natural-language console spike (branch `spike/needle-nl`)

`d2d5b1f`, in `spike_needle_nl/`, outside `src/`, so colcon ignores it. Nothing
in the robot packages was changed.

### What was built

An offline text console over the Nav2 stack using
[Needle 3](https://github.com/cactus-compute/needle). Ten tools over the real
interfaces: `navigate_to_pose`, `/cmd_vel`, `/amcl_pose`, `/scan`, and
controller_server's FollowPath speed limits. Each tool is tagged
`read` / `tune` / `act`. `console.py` decodes a call without executing it,
applies the risk tier, and runs the function itself only after the gate
passes; `act` tools always ask for confirmation. `run_eval.py` scores 32
spoken commands by exact match.

### Results

| Measure | Result |
|---|---|
| Exact match, 32 commands, live headless Ignition + Nav2 | 21–22/32 (66–69%) |
| Requests with no matching tool that it refused | 3–4 of 7 |
| Named places reached | 5/5, worst position error 0.15 m |
| Moves attempted without confirmation that reached ROS | 0 of 3 |

`Needle.complete()` is not stateless: a prior unrelated query changes the next
answer ("head over to the living room" decodes correctly at 0.9968 cold, and
picks a different tool at 0.6025 after four unrelated queries). Calling
`Needle.reset()` per utterance took the eval from 34% to 69%. Confidence does
not separate right from wrong ("what's the weather in Lagos" decodes to a
real tool call at 1.00), so the gate carries the safety, not the model.
Verdict in the commit: promising enough to fine-tune, not deployable yet.

### Faults found, and what was done

- **`/scan` QoS.** The bridge publishes best effort; a default reliable
  subscriber receives nothing, and rclpy only warns. Fixed in the backend.
- **`/amcl_pose` is transient-local** and published only when the estimate
  moves, so a volatile late subscriber gets nothing on a parked robot. Fixed.
- **`navigation_status`** reported cancelled and aborted goals as still
  navigating. It now maps `GoalStatus` and clears the handle.
- **Three of five waypoints were unnavigable**, measured against
  `explored_map.pgm`: bedroom was inside an obstacle (0.00 m clearance),
  kitchen had 0.25 m and stalled 0.45 m short with nothing logged by Nav2.
  The places now have 1.6–2.0 m clearance.
- **Two sim stacks running at once** gave two `/clock` publishers, so
  simulated time ran backwards and every TF buffer was cleared. An earlier
  round of this work had blamed a localisation bug in the workspace; that was
  retracted in `RESULTS.md`. The EKF is fine.

### How to run

```sh
python3 -m venv nv && ./nv/bin/pip install cactus-needle
export NEEDLE_TELEMETRY=0 DO_NOT_TRACK=1
./nv/bin/python run_eval.py --verbose
./nv/bin/python console.py            # mock backend
./nv/bin/python console.py --live     # real robot; act tools move it
```

### Open

- Fine-tune the model; refusals of out-of-scope requests are the weak point.

## 2026-09-21 — first builds and bringup on the Pi, and a Nav2 TF regression

### Faults found, and what was done

**Nav2 goals failed partway through a run** (`dc8b7cc`). `ddac963` (08-14)
remapped `/tf` → `tf` on every Nav2 node unconditionally. Partway through a
run the controller's `map → odom` went stale ("Transform data too old") while
AMCL was still publishing it at 10 Hz, and every later goal failed. A single
goal still succeeded, which is how it passed the 08-14 test. On the 10-goal
simulated course:

| Build | Goals | Stale-transform errors |
|---|---|---|
| `7addf58` (before namespacing) | 10/10 | 0 |
| `ros2-humble` + IMU relabel | 1–4/10 | 1505–2708 |
| same, Nav2 TF remaps removed | 10/10 | 0 |
| `dc8b7cc`, default launch | 10/10 | 0 |
| `dc8b7cc`, `namespace:=robot1` | one goal succeeded; TF on `/robot1/tf` only | — |

The remaps are now applied with `SetRemap` only when `use_namespace` is set.
The root cause was not understood on the day (see 09-23 and 09-24).

**The lidar driver did not build** (`8e85600`, `a27eae9`). README and
MODULE.md said `ydlidar_ros2_driver` needs the YDLidar SDK installed
system-wide. It does not: the SDK is vendored in `YDLidar-SDK/` and builds as
the colcon package `ydlidar_sdk`, but the driver's `package.xml` does not
declare it, so colcon neither builds it first nor puts it on the driver's
search path. The build steps now build `ydlidar_sdk` first, then source
`install/setup.bash`, then build the rest. With that, all 8 robot packages
build on the Pi.

**First real-robot bringup aborted before any node started** (`ec651bf`).
`tortoisebotreal.xacro` included two Gazebo files via
`$(find tortoisebot_gazebo)`, which the robot build does not install. Both
contain only `<gazebo>` blocks, so they were dropped; the expanded URDF keeps
the same 8 links and 7 joints with identical origins.

### Changed

- **IMU relabel moved onto `ros2-humble`**: `7a764e1` and `28e625a` are the
  08-13/08-14 commits `829433a` and `b75fd81` from branch
  `imu-frame-base-link`, reapplied on top of `dc8b7cc`.
- **Docs follow the relabel** (`9ac9590`). The lidar is back at 0.167 m in the
  map frame, so the voxel floor at −0.2 now gives about 0.37 m of headroom
  rather than 0.26 m; it stays at −0.2 so reverting the relabel cannot start
  dropping scans. MODULE.md records the IMU axes as measured on the robot:
  x forward, y left, z up.
- **README clones this fork** (`nitingarg77/tortoisebot`) instead of
  `rigbetellabs/tortoisebot` (`8e85600`).
- **`/scan` rate documented as measured on this robot's YDLidar X2: 11.5 Hz**,
  not 10 Hz (`8e85600`).
- **`differential.py` wire-colour comments** now match this robot's wiring,
  taken from the vendor SD card. Pin numbers unchanged (`8e85600`).

## 2026-08-14 — dependencies, MODULE.md, namespaces, voxel floor

### Added

- **Package dependencies** (`3422d18`). All eight first-party manifests were
  the `ros2 pkg create` template, so `rosdep install` succeeded and installed
  nothing. rosdep keys per package, before → after:

  | Package | Keys | Package | Keys |
  |---|---|---|---|
  | bringup | 3 → 10 | gazebo | 6 → 15 |
  | control | 4 → 6 | imu | 5 → 5 |
  | description | 3 → 11 | firmware | 6 → 8 |
  | navigation | 3 → 17 | slam | 3 → 7 |

  Robot and simulation drivers are REP 149 conditional dependencies on
  `TORTOISEBOT_TARGET` (unset = simulation, `robot` = firmware, imu, lidar and
  camera). `navigation.launch.py` no longer looks up bringup's share directory
  (unused, and it made the two packages depend on each other).
  `autobringup.launch.py` resolves `tortoisebot_gazebo` through
  `FindPackageShare`, so the real robot can launch without the simulator
  installed.
- **MODULE.md** (`7addf58`): the interface contract (scan, odom, imu,
  `robot_description` in; `cmd_vel` and the TF chain out), why covariance is
  part of it, a per-package reuse classification, a porting checklist, and
  known gaps. It notes that `wheel_separation` 0.17186 is duplicated across
  three files with nothing keeping them in sync. README §2.2 now builds with
  `--packages-up-to`: six packages with `TORTOISEBOT_TARGET` unset, eight with
  `robot`.
- **README: IMU dependencies and localisation pipeline** (`b565348`):
  `adafruit-circuitpython-bno055`, `Adafruit-Blinka`, `RPi.GPIO`, and the
  `i2cdetect` check for the sensor at 0x28. The robot has no wheel encoders,
  so `/odom` exists only in simulation; `odom → base_link` comes from the EKF
  in simulation and from Cartographer on hardware. Rates measured: scan 10 Hz,
  imu and odom 50 Hz, `cmd_vel_nav` 10 Hz, `cmd_vel` 20 Hz.
- **Optional namespace** (`ddac963`): `namespace:=robot1 use_namespace:=True`,
  both off by default. Frame names are not prefixed; `/tf` and `/tf_static`
  are remapped to relative names so each robot gets its own TF tree. Costmap
  scan topics use a `<robot_namespace>/scan` placeholder (nav2 1.1.20 has no
  `joinWithParentNamespace`). `/clock` is not namespaced. Adds
  `rviz/nav2_namespaced.rviz`; `rviz.launch.py` now honours `use_sim_time`
  instead of hardcoding True. Two robots in one Ignition world are not
  covered. This also corrected MODULE.md's claim that 26+ TF frame names would
  need prefixing.

### Faults found, and what was done

- **Namespaced delayed nodes landed outside the namespace** (`ddac963`).
  `PushRosNamespace` does not survive into a `TimerAction`, so the EKF, both
  lifecycle managers and the spawn node started un-namespaced; the spawn node
  waited on `/robot_description` and the robot never appeared. Each delayed
  node now carries its own group.
- **Voxel floor too close to the scan** (`7302c7e`). With
  `tracking_frame = imu_link`, Cartographer put z = −0.110 into the published
  transform, placing the lidar at 0.057 m in the map frame. The global
  costmap's voxel layer discards points below `origin_z: 0.0`; a 2° pitch over
  the castor drops returns at 2 m by 0.070 m, below the floor. `origin_z` is
  now −0.2 (about 0.26 m of margin). Hardware config only; checked to parse,
  not run.
- **Stale reason for not fusing linear acceleration** (`0f8d26f`). The EKF
  comment blamed Ignition's zero covariance, which the relay now fixes. It now
  gives the real reason: wheel odometry already measures linear velocity, and
  `two_d_mode` pins z, roll and pitch.
- **Warning next to `imu0_config`** (`b75fd81`, branch `imu-frame-base-link`;
  reapplied on 09-21 as `28e625a`): relabelled IMU data is not exact for
  linear acceleration under pitch or roll, with the steps to revert the
  relabel and raise the voxel floor.

### Measurements

After `ddac963`, in simulation:

| Path | Result |
|---|---|
| default | scan 10.0 Hz, imu/odom/relays/filtered 50 Hz, one `odom → base_link` publisher at 50.0 Hz, goal succeeded, 0 recoveries |
| `namespace:=robot1` | every node under `/robot1`, bare `/tf` silent, both costmaps on `/robot1/scan`, goal on `/robot1/navigate_to_pose` succeeded, 0 recoveries |

The default path was tested with a single goal only; the 10-goal course on
09-21 showed that was not enough.

## 2026-08-13 — IMU into localisation, RViz scan QoS, velocity smoother

All verified in simulation unless noted.

### Faults found, and what was done

**RViz showed no scan** (`c48266f`). The LaserScan display in `nav2.rviz`
and `simulation.rviz` asked for reliable QoS; `ros_gz_bridge` publishes
`/scan` best effort. Cartographer and the costmaps were unaffected.

**Nothing was acceleration-limited** (`c48266f`). `controller_server`
published straight to `/cmd_vel`, while `velocity_smoother` listens on
`cmd_vel_nav`, which had no publishers. Both navigation launches now remap it
there; `behavior_server` still publishes to `/cmd_vel`. Peak commanded
acceleration fell from +7.61 to +0.25 m/s². `nav2_params_robot.yaml` had no
`velocity_smoother` section (Nav2 defaults 0.50 m/s, 2.5 m/s²); added,
mirroring `controller_server`'s limits.

**The EKF ignored the IMU in simulation** (`9b6a4a6`). Ignition's IMU and
odometry messages have no covariance fields, so the bridge emitted zeros, and
the EKF rejected 100% of IMU messages at the Mahalanobis gate (331.99 against
0.64) while copying wheel odometry through. New
`tortoisebot_gazebo/scripts/sim_covariance_relay.py` republishes both with
realistic covariances on `/imu_with_covariance` and `/odom_with_covariance`,
which `ekf_mapbased` now reads; the bridge's own `/imu` and `/odom` are left
for Nav2, Cartographer and the smoother. IMU rejection threshold 0.8 → 5.0.

| At rest after a 10-goal course | Yaw error vs ground truth |
|---|---|
| wheel odometry | +23.45° |
| EKF, before (identical to wheel odom) | +18.1° |
| EKF, after | +0.047° |

10/10 goals, zero recoveries.

**Nine silent IMU defects on hardware** (`9b6a4a6`):

| Defect | Fix |
|---|---|
| published `/imu/data`; Cartographer reads `/imu` | now publishes `imu` |
| `frame_id` `imu` is not a TF frame | now `imu_link` (parameter) |
| covariances all `[-1.0] * 9` ("not measured") | real values, as parameters |
| `use_imu_data` false in `slam_real.lua` and `slam_real_mapbased.lua` | enabled |
| node commented out of `autobringup` | enabled |
| raw BNO055 offset LSBs subtracted from scaled readings (59 m/s² phantom accel on X; magnetometer offsets unused) | written to the sensor's offset registers |
| `math.radians()` on gyro already in rad/s | removed |
| `linear_acceleration` (gravity removed) | `acceleration` |
| 10 Hz vs 50 Hz in sim | 50 Hz (parameter) |

Checked against a stub sensor only.

**Cartographer's IMU offset leaked into the map** (`829433a`, branch
`imu-frame-base-link`; reapplied on 09-21 as `7a764e1`). Cartographer requires
the IMU frame to coincide with `tracking_frame`. Setting
`tracking_frame = imu_link` put z = −0.110 into `map → odom` and
`map → base_link`, placing the lidar at 0.057 m instead of 0.167 m. The IMU
data is now labelled `base_link` while the sensor stays on `imu_link` in the
URDF and in Ignition. This is exact for orientation and angular velocity on
this robot (`imu_joint` is a pure z offset). Verified: `map → odom` z =
0.0000, lidar at 0.1670 m, Cartographer takes the IMU at 50.00 Hz, 10/10
goals, EKF yaw 0.047° from ground truth against wheel odometry's 23.49°.

### Open

- IMU on the robot: I2C at 50 Hz and the chip's axis orientation relative to
  `base_link` still to be checked (axes confirmed on 09-21).

## Before 2026-08-13 — upstream starting point

This repo is a fork of
[rigbetellabs/tortoisebot](https://github.com/rigbetellabs/tortoisebot)
(remote `upstream`; the fork is `nitingarg77/tortoisebot`, remote `origin`).
Branch `ros2-humble` carries 192 upstream commits from 2021-02-08 to
2026-08-05, and the work above starts after `eae9f74` (2026-08-05, merge of
upstream PR #46, a README network-setup update). The most recent upstream
work on that branch migrated the simulation to Ignition Gazebo for Humble
(`0be5a8f`, 2026-04-28), fixed the autobringup SLAM/Nav logic and renamed the
Cartographer configs (`1b97142`), and added a camera node to autobringup
(`766364e`). Upstream's other branches (melodic, noetic, galactic, jazzy and
others) are fetched but unchanged here.
