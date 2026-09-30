# TortoiseBot as a reusable autonomy module

This document describes the seam between the parts of this workspace that are
generic autonomous-mobile-robot machinery and the parts that only make sense on
this particular chassis. It exists so that porting the stack to a different
robot is a matter of working through a checklist rather than reading every file.

It reflects the code as it stands. Where something is not yet modular, it says
so rather than describing an intention.

---

## 1. The interface contract

The autonomy layer consumes sensors and produces velocity commands. It has no
knowledge of motors, GPIO, or any specific sensor part number.

### What a chassis must provide

| Interface | Type | Rate | Notes |
|---|---|---|---|
| `scan` | `sensor_msgs/LaserScan` | 10 Hz | planar, mounted level |
| `odom` | `nav_msgs/Odometry` | 50 Hz | **must carry real covariances** |
| `imu` | `sensor_msgs/Imu` | 50 Hz | optional; gravity **included** in `linear_acceleration` |
| `robot_description` | URDF | latched | `base_link` plus a frame per sensor |
| `/clock` | `rosgraph_msgs/Clock` | sim only | when `use_sim_time:=True` |

### What the module produces

| Interface | Type | Notes |
|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | the only actuation output |
| `map` → `odom` → `base_link` | TF | single publisher per link, see §4 |
| `map` | `nav_msgs/OccupancyGrid` | from Cartographer |
| NavigateToPose | action | Nav2 entry point |

The contract is empirically validated: the same interface drives both Ignition
Fortress and the physical robot, with completely different code underneath.
That is the strongest evidence that this boundary is real.

### Covariance is part of the contract, not a detail

`ignition.msgs.IMU` and `ignition.msgs.Odometry` have no covariance fields, so
bridged messages arrive all-zero and `robot_localization` reads zero back as
`1e-9` — a claim of near-perfect certainty. The filter then rejected 100% of IMU
messages and copied wheel odometry through untouched.

`tortoisebot_gazebo/scripts/sim_covariance_relay.py` exists solely to repair
this, republishing onto `imu_with_covariance` and `odom_with_covariance`. Any
new chassis feeding this module must publish honest covariances or add an
equivalent shim. Measured effect after a 10-goal course: wheel odometry drifted
+23.45° from ground truth while the fused estimate held +0.047°.

---

## 2. Layer map

| Package | Classification | Reusable as-is? |
|---|---|---|
| `tortoisebot_control` | generic | yes — it is upstream `teleop_twist_keyboard` |
| `tortoisebot_navigation` | generic logic, tuned values | yes, after retuning §3 |
| `tortoisebot_slam` | generic logic, tuned values | yes, after retuning §3 |
| `tortoisebot_bringup` | generic composition pattern | pattern yes, contents no |
| `tortoisebot_gazebo` | mixed | worlds yes, bridge/plugins no |
| `tortoisebot_description` | **chassis-specific** | no — URDF, meshes, geometry |
| `tortoisebot_firmware` | **chassis-specific** | no — GPIO pinout, open loop |
| `tortoisebot_imu` | part-specific | only for another BNO055 |

Despite its name, `tortoisebot_control` contains **no controllers**. It is a
single teleop node. There is no `ros2_control` layer anywhere in this
workspace; see §5.

---

## 3. Porting checklist

Every value below is chassis-specific and must be revisited for new hardware.
Line numbers are indicative, not load-bearing.

### 3.1 Drive geometry — duplicated in three files

`wheel_separation = 0.17186` appears in all three, and nothing keeps them in
sync. Changing one and not the others produces a robot whose simulation and
reality disagree in a way that looks like a tuning problem.

- `tortoisebot_gazebo/gazebo/tortoisebot_ignition.gazebo:31` (also `wheel_radius 0.0325`)
- `tortoisebot_gazebo/gazebo/tortoisebot_plugins.gazebo:16`
- `tortoisebot_description/models/urdf/tortoisebot_simple.gazebo:62`

### 3.2 Footprint and inflation

- `robot_radius: 0.1` — `nav2_params_robot.yaml:182` and `:224`
- `inflation_radius: 0.5` (local) / `0.55` (global)

### 3.3 Velocity and acceleration limits — three places that must agree

- `controller_server`: `max_vel_x 0.15`, `max_vel_theta 1.5`, `acc_lim_x 1.0`, `acc_lim_theta 1.5`, `max_speed_xy 0.22`
- `velocity_smoother`: `max_velocity [0.15, 0.0, 1.5]`, `min_velocity [-0.15, 0.0, -1.5]`
- simulation plugin: `max_linear_velocity 0.23`, `max_angular_velocity 1.5`

The smoother limits deliberately mirror the controller's. If they diverge, the
smoother silently clips commands the planner believes were issued.

### 3.4 Sensor mounting

- `imu_link` and `lidar` origins in the xacro files.
- Cartographer **asserts** the IMU frame is colocated with `tracking_frame`
  within `1e-5` m. This stack satisfies it by labelling IMU data `base_link`
  (`frame_id` in `imu_node.py`, `ignition_frame_id` in the sim) while the chip
  physically sits at `imu_link`, 0.11 m up, so `tracking_frame` stays
  `base_link`. That is exact only because `imu_joint` is a pure vertical offset
  with no rotation and the chip's axes match `base_link` — checked on the robot
  on 2026-09-21: x forward, y left, z up. **On a new chassis, check both.** A
  horizontal offset or a rotated chip breaks the relabel; move `tracking_frame`
  to the IMU frame instead, and expect the offset to appear in `map` → `odom`.
- Linear acceleration must stay unfused under the relabel; the warning and the
  revert steps are next to `imu0_config` in `ekf_mapbased.yaml`.
- `voxel_layer.origin_z: -0.2` in `nav2_params_robot.yaml` is a discard floor.
  The lidar sits at `0.167` m, so it gives `0.367` m of headroom, roughly 10° at
  2 m range. It is deliberately left low enough to survive reverting the
  relabel, which drops the lidar to `0.057` m.

### 3.5 Sensor rates

`ydlidar.yaml frequency: 10.0` (this robot's YDLidar X2 measured 11.5 Hz on
`/scan`, 2026-09-21); `controller_frequency: 10.0`;
`smoothing_frequency: 20.0`; IMU `rate_hz: 50.0` (a node parameter, so it can
be lowered on a slower I2C bus without rebuilding).

---

## 4. Build targets

The robot and simulation driver stacks are selected separately: the lidar, IMU,
camera and motor drivers are only useful on the Pi, and Ignition is pointless
there. `ydlidar_ros2_driver` does build on a workstation, but only after
`ydlidar_sdk` (the vendored `YDLidar-SDK/`), which its `package.xml` does not
declare. colcon therefore neither orders the SDK first, nor includes it in
`--packages-up-to`, nor adds its prefix to the driver's `CMAKE_PREFIX_PATH`, so
it has to be built explicitly and the workspace sourced before the main build.

`tortoisebot_bringup/package.xml` expresses this with REP-149 conditional
dependencies keyed on `TORTOISEBOT_TARGET`:

```bash
# Simulation workstation (default when the variable is unset)
colcon build --packages-up-to tortoisebot_bringup tortoisebot_control

# Physical robot
export TORTOISEBOT_TARGET=robot
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ydlidar_sdk
source install/setup.bash
colcon build --packages-up-to tortoisebot_bringup tortoisebot_control
```

**Forgetting the export on the robot is a silent failure**: rosdep will not
install the lidar, IMU, camera or motor packages, the build will succeed, and
the launch will fail at runtime on a missing package.

### `odom → base_link` has a different publisher per mode

- **Simulation** — `ekf_filter_node` fuses `odom_with_covariance` and
  `imu_with_covariance`. Ignition's DiffDrive TF is deliberately **not** bridged;
  Fortress ignores `<publish_tf>false</publish_tf>`, so bridging it made it
  compete with the EKF for the same transform.
- **Robot** — Cartographer, via `provide_odom_frame = true`. The EKF node is
  commented out in `autobringup.launch.py` because **the robot has no wheel
  encoders**, so there is no odometry to fuse. `differential.py` is open-loop
  GPIO PWM and publishes no `odom` at all.

---

## 5. Running under a namespace

The whole stack can be pushed into a namespace, so several robots can share one
DDS domain:

```bash
ros2 launch tortoisebot_bringup autobringup.launch.py \
    namespace:=robot1 use_namespace:=True
```

Both arguments default to off (`namespace:=''`, `use_namespace:=False`), and
that path is behaviourally identical to having no namespace support at all.

> [!WARNING]
> **The namespaced path is verified for a single goal only.** Remapping `/tf` →
> `tf` on the Nav2 nodes made the controller's `map` → `odom` go stale partway
> through a run ("Transform data too old") while AMCL was still publishing it at
> 10 Hz. This happened even with no namespace, where `/tf` and `tf` name the same
> topic, and it cut a 10-goal simulated course from 10/10 to 1–4/10; one goal
> still succeeds, which is how it first passed testing. The navigation launch
> files therefore apply these remaps with `SetRemap` only when
> `use_namespace:=True`, which restores the default path.
>
> **Both paths passed a 10-goal course on 2026-09-24**, run with
> `tortoisebot_navigation/scripts/nav_course.py --laps 2` against a headless
> Ignition + Nav2 stack on an isolated `ROS_DOMAIN_ID`:
>
> | Path | Result | Worst error | TF staleness in the log |
> |---|---|---|---|
> | default (no namespace, no remaps) | 10/10 | 0.12 m | none |
> | `use_namespace:=True` (remaps applied) | 10/10 | 0.28 m | none |
>
> The decisive experiment was then run: the remaps were made **unconditional
> again**, exactly as in `ddac963`, with no namespace and a verified single
> `/clock` publisher. That course also passed **10/10, worst error 0.13 m, no
> TF staleness**.
>
> **The remaps are therefore not the cause.** The failure could not be
> reproduced in any of the three configurations. The likeliest explanation of
> the original 1-4/10 runs is the two-`/clock`-publishers artifact described in
> `spike_needle_nl/RESULTS.md`: a leftover second simulator makes simulated
> time run backwards and clears every TF buffer, which produces exactly this
> symptom from an unrelated cause. That the same workspace was being used to
> debug two sim stacks at the time fits.
>
> `dc8b7cc` stays, because without a namespace `/tf` and `tf` name the same
> topic and the conditional form is a no-op there, so it costs nothing. But it
> should not be read as a fix for a known defect: on this evidence there was no
> defect in the remaps. Three runs of ten goals each, one per configuration,
> all on 2026-09-24.

### Frame names are deliberately *not* prefixed

This is the part that surprises people, and an earlier version of this document
got it wrong. There is no `frame_prefix` anywhere and no `map` → `robot1/map`
rewriting. Instead every node remaps `/tf` → `tf` and `/tf_static` →
`tf_static`:

```python
TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
```

`/tf` is an absolute name, so it normally escapes any namespace. Making it
relative lets `PushRosNamespace` move it to `/robot1/tf`, which gives each robot
its **own TF tree** (in the Nav2 launches the remaps are applied by a
conditional `SetRemap` rather than per node; see the warning above). Two robots
can then both use `map`, `odom` and `base_link` without colliding, because the
trees are separate. This is what `nav2_bringup`
does, and it is why the Cartographer `.lua` files and every `global_frame` /
`robot_base_frame` setting are untouched.

**The consequence to remember:** a node that misses these two remappings keeps
publishing to the global `/tf` and silently corrupts the tree. Any new node
added to this stack needs them.

### The two things that could not simply follow the namespace

- **Costmap observation topics.** Costmap layers run on a child node, so a
  relative `scan` resolves to `/robot1/local_costmap/scan`, not `/robot1/scan`.
  Newer nav2 has `joinWithParentNamespace` to fix this; **1.1.20 here does
  not**. The params therefore carry a `<robot_namespace>/scan` placeholder that
  `ReplaceString` fills in at launch. An empty namespace yields `/scan`.
- **The `ros_gz_bridge` arguments**, which name the Ignition topic and the ROS
  topic with a single string. This version (0.244.25) has no `config_file`
  option to separate them, so the arguments stay absolute to match the SDF and
  the ROS side is moved with remappings. `/clock` is deliberately excluded — it
  is global, and namespacing it would leave every `use_sim_time` node waiting
  for a clock nobody publishes. The bridge's `qos_overrides` parameter names
  embed the fully qualified topic name, so they are built in an
  `OpaqueFunction` once the namespace is known.

RViz configs store absolute topic names and cannot be pushed into a namespace,
so there are two variants: `rviz/nav2.rviz` and `rviz/nav2_namespaced.rviz`,
the latter using the same `<robot_namespace>` placeholder. This mirrors
upstream's `nav2_namespaced_view.rviz`.

### What this does not give you

**Two robots in a single Ignition world.** The gz-side topics (`/scan`, `/imu`,
`/cmd_vel`) are baked into `tortoisebot_ignition.gazebo`, as is the model name
inside `tf_topic`. Namespacing is ROS-side only; a second robot in the same
world would need those templated per robot.

## 6. Known gaps

These are real limits of the current code, listed so nobody rediscovers them.

**No hardware abstraction.** There is no `ros2_control` layer. `differential.py`
talks directly to GPIO pins. Swapping motor hardware means rewriting that node
rather than selecting a plugin. Closing this properly requires wheel encoders,
which this chassis does not have — without them there is no velocity feedback to
close a loop around, and no `odom` on the real robot.

**Third-party drivers vendored in-tree.** `v4l2_camera`, `ydlidar_ros2_driver`
and `YDLidar-SDK` are pinned copies rather than vcs-managed dependencies.
Changing lidar means editing this repository. Note also that
`autobringup.launch.py` launches `camera_ros`, which is a *different* package
from the vendored `v4l2_camera` and is not installed on the development machine.

**`controller_server`'s `map` -> `odom` buffer freezes: an ABBA deadlock in tf2
0.25.23, fixed upstream but not yet in apt.** Found 2026-09-30 by backtrace;
see "The cause" at the end of this entry. The history below is kept because
every step of it was a reasonable reading of the evidence at the time.
Previously headed "the cause is not yet known". The 09-28 measurement below attributes it to a second observation
source; the 09-29 measurement further down shows the hang persisting with a
single source, so read both before acting on either. Adding
`floor_scan` beside `scan` in `observation_sources` (nav2_params_robot.yaml)
stops the robot dead, in a way that reads as a navigation bug rather than a
crash. Measured on the real robot, 2026-09-28:

| | `scan` + `floor_scan` | `scan` alone |
|---|---|---|
| Distance driven in 20 min | 0 m, stuck at the origin | 3.7 m in the first 2 min, then exploring |
| `controller_server`'s TF buffer | frozen at one timestamp, permanently | transiently stale, recovers |
| Nav2 stack | torn down after `behavior_server` missed its 4 s bond heartbeat | healthy |

It is not CPU and not the network. `/tf` on the wire stayed at 66 Hz and under
0.1 s old throughout, verified over five minutes with an independent tf2
buffer in another process that never froze; no transform was ever stamped in
the future, so the buffer was not poisoned by the data. Only the buffer inside
`controller_server` stopped advancing. Capping the camera to 10 fps and
rewriting `floor_scan`'s inner loop in numpy (176a4d1) cut the load average
from 17 to 5.3 and changed nothing.

What turns the hang into silent wrong behaviour is Nav2 Humble itself:
`ControllerServer::isGoalReached()` **discards the return value** of
`nav_2d_utils::transformPose`. When `map` -> `odom` cannot be resolved, the
goal quietly becomes the origin of `odom`, and the same frozen buffer pins the
robot's own pose near that origin too. Both are then within
`xy_goal_tolerance`, so the controller logs `Reached the goal!` about 100 ms
after accepting a goal three metres away, `bt_navigator` reports success, and
the explorer moves on. The only hint is one line per cycle from a logger that
is not a node:

    [tf_help]: Transform data too old when converting from map to odom
    Data time: 1790606201.884, Transform time: 1790605942.520   <- never changes

A frozen `Transform time` is the signature. A moving one is ordinary lag.

Untested hypothesis: a second source means a second `tf2_ros::MessageFilter`
on the same `Costmap2DROS`, and the global costmap made it worse by listing
both sources under *two* layers (`obstacle_layer` and `voxel_layer`), so
`floor_scan` took the filter count from two to four. The A/B above is one run
each, so treat the cause as indicated rather than proven.

**Fix written and half-verified on the robot** (2026-09-29). Rather than
two sources, `scan_fusion_node.py` merges the camera into the lidar scan and
publishes `/scan_fused`, which is now the only source on both costmaps. The
decision logic is in `tortoisebot_navigation/scan_fusion.py`, which imports no
ROS and has 43 tests. Three things about it are worth knowing before touching
it:

- **Not `min(lidar, camera)`.** `ydlidar.yaml` sets `invalid_range_is_inf:
  false`, so this driver publishes a no-return as **0.0**. A plain minimum
  returns 0.0 at exactly the bearings where the lidar saw nothing and the
  camera is the only sensor that can see — the white panels — and 0.0 is under
  `range_min`, so `laser_geometry` drops the beam and nothing is marked at all.
  The rule is the nearest *valid* return, and a beam neither sensor saw keeps
  the lidar's own encoding untouched.
- **The fused topic may clear, where `/floor_scan` could not.** Taking the
  nearer of the two per beam means the camera can only pull a reading closer,
  never push it away, so it cannot declare free anything the lidar sees. The
  gain is that a camera false positive heals on the next scan instead of being
  marked permanently.
- **The 33 mm lidar offset is not negligible.** At 0.45 m and 26.75° it moves
  the bearing 1.8° and the range 3 cm, so each camera beam is projected rather
  than copied across. Rotation between the two stamps is corrected from the
  IMU's yaw — 200 ms at 1.2 rad/s smears the fan 13.8°. Translation over the
  same interval costs 2.6 cm, under one costmap cell, and is left uncorrected.

Measured on the robot, 30 s with the drivers up and nothing navigating
(`ros2 run tortoisebot_navigation check_fusion.py --seconds 30`):

| | |
|---|---|
| `/scan_fused` rate | 11.7 Hz, against `/scan`'s 11.6 Hz — no scans dropped |
| Scans where the camera changed a beam | 306 of 315 (97%) |
| Beams changed | 11,615, of which **5,907 were beams the lidar saw nothing on** |
| Beams pushed *further* away | 0 |

The 5,907 figure is the point of the whole node: those are returns only the
camera produced, on a stationary robot in this building. The zero is the safety
property — fusion only ever pulls a reading nearer. The node's own 30 s report
agrees independently at 96-97%.

The only degradation seen was `no yaw for a 0.08 s gap`, 5 times in ~700 scans
(0.7%), which is the IMU history not spanning the interval and the fan being
skipped rather than de-rotated wrongly. Working as designed.

### It does not cure the hang (2026-09-29, measured)

Nav2 was brought up on the robot with `/scan_fused` as the **only** observation
source on both costmaps, confirmed in the log (`Subscribed to Topics:
scan_fused`, once per costmap). One `NavigateToPose` goal 0.80 m ahead was
sent. Over the 90 s it was live:

| | |
|---|---|
| `Transform data too old`, `map` -> `odom` | **895 lines** |
| Distinct `Transform time` values in them | **1** — `1790688350.982`, unchanging |
| `Data time` in the same lines | advanced normally, `...357` -> `...447` |
| `Failed to make progress` | 5, each followed by `Aborting handle` |
| `Resulting plan has 0 poses` | **0** — the 09-28 voxel fix held |

One distinct `Transform time` across 90 seconds is the frozen buffer, by this
file's own definition. **The single-source change did not fix it**, so the
"second `tf2_ros::MessageFilter`" hypothesis above is wrong, or is not the
whole cause. The A/B it rests on was one run each, which is why it was recorded
as indicated rather than proven; treat it now as refuted unless something
re-establishes it.

What this cost in practice: the robot drove while `controller_server` reported
`Failed to make progress`, because Nav2 was steering against a pose frozen 7 s
before the goal was sent. It overshot the 0.80 m goal and curved off heading.
A frozen `map` -> `odom` does not stop the robot; it makes Nav2 blind to the
fact that it is moving.

**(a) is ruled out — but only as of 2026-09-30, by the simultaneous runs
below.** The 09-29 run that first claimed it (reproduced next) had no freeze
*anywhere*, so it showed Cartographer healthy in a quiet window, not innocent
during a freeze. That claim was premature and is kept here as the reasoning
that did not hold. `scripts/tf_watch.py` holds its own tf2 buffer in a third
process and separately watches raw `/tf` off the wire. Run for 100 s with
Cartographer up, on a Pi at load 11.15:

| | |
|---|---|
| `map` -> `odom` **on the wire** | 11,823 messages in 100 s = **118 Hz** |
| Longest freeze of an ordinary buffer | **0.0 s** |

Cartographer publishes it 118 times a second and an ordinary listener tracks it
without difficulty. So the transform is not missing and SLAM is not stalling:
**the freeze is inside `controller_server`'s process**, which is explanation
(b) and matches what 09-28 concluded — except that it now happens with a single
observation source, so the `MessageFilter` count is not the trigger.

Caveat on that run: Nav2 had failed to activate at the time (see below), so
`controller_server` was not querying and its buffer could not be watched in the
same window. The wire measurement stands regardless, but the simultaneous
observation — this buffer healthy *while* `controller_server`'s is frozen — has
not been made yet.

**118 Hz is itself worth a look.** 09-28 measured `/tf` at 66 Hz. Cartographer's
`pose_publish_period_sec` decides this, and a listener that cannot keep up with
the flood is a candidate cause of the starvation rather than a victim of it.

### What the Humble source says (read 2026-09-29, Nav2 1.1.20, geometry2 humble)

Read because the obvious theory -- "`controller_server`'s executor is starved"
-- turned out not to fit the code at all.

**The buffer's listener is not on `controller_server`'s executor.**
`Costmap2DROS::on_configure` builds it as
`TransformListener(*tf_buffer_)`, the single-argument constructor, which makes
its own node (`transform_listener_impl_<addr>` -- the six seen in the graph)
and spins it on **its own dedicated thread** with its own
`SingleThreadedExecutor`. `controller_server` reads everything through
`costmap_ros_->getTfBuffer()`, so this listener is the one feeding the buffer
that freezes.

**But costmap work runs on that thread. Verified link by link:**

1. The listener's subscription callback calls `buffer_.setTransform()`.
2. `BufferCore::setTransformImpl` ends by calling `testTransformableRequests()`,
   which invokes every newly-satisfiable request's callback **on the calling
   thread**, after dropping its locks (`buffer_core.cpp`).
3. `tf2_ros::MessageFilter` registers exactly such a callback through
   `Buffer::waitForTransform` whenever a message arrives before its transform.
4. In Humble, callback queues are disabled (`TODO(clalancette)`), so
   `messageReady` calls `signalMessage` directly.
5. `signalMessage` is `ObstacleLayer::laserScanCallback`: laser projection, a
   lock on the `ObservationBuffer`, and `bufferCloud`.

So whenever a scan beats its transform, the obstacle layer's entire scan
callback runs **on the thread that is supposed to be feeding the buffer**, and
no `/tf` is consumed while it does. On a fast machine TF usually arrives first
and this path is rare; on a loaded Pi it would not be. That is load-dependent
in the way the fault is, and it explains why an ordinary listener with no
`MessageFilter` attached -- `tf_watch.py` -- never froze.

**The deadlock this suggests does not happen, and here is why.**
`bufferCloud` calls `tf2_buffer_.transform(..., tf_tolerance_)`, and a timed
lookup in `tf2_ros::Buffer::canTransform` is a polling loop with
`sleep_for(10ms)` -- which, on the listener thread, would wait for a transform
only that thread can deliver. But the `MessageFilter` targets `global_frame_`,
`sensor_frame` is unset in `nav2_params_robot.yaml`, and `bufferCloud` asks for
`lidar -> global_frame_` at the scan's own stamp: exactly what the filter
already verified before it fired. The lookup resolves on the first check and
never sleeps. **Link 6 breaks, so this is a coupling, not a deadlock.**

What remains unverified on this thread: the `ObservationBuffer` mutex is taken
both here (in `laserScanCallback`) and by the costmap update thread in
`getMarkingObservations`. Contention there would stall the listener too. Not
measured.

**The experiment staged from this.** `slam_real.lua`
`pose_publish_period_sec` 5e-3 -> 2e-2, 200 Hz -> 50 Hz, which cuts
`setTransform` + `testTransformableRequests` work on that thread by more than
half. It cuts both ways: a slower TF rate makes scans beat their transform
more often, which sends *more* scan callbacks down the listener-thread path.
20 ms between samples keeps that under a quarter of the 85 ms scan period.
Whether the trade wins is untested. Verify with `tf_watch.py` running
**alongside** a live goal from `one_goal.py`, before and after; revert to 5e-3
if the hang gets worse.

Correlation recorded, **not** claimed as cause: the buffer's last update fell
within about a second of the test script starting, which added a TF listener
and two sensor-QoS subscriptions to a Pi already at load 11.6.

### The TF-rate A/B: not a fix (2026-09-30, measured)

Four launches, alternating 200 Hz / 50 Hz, identical protocol
(`scripts/tf_ab_launch.sh`, analysed by `scripts/tf_ab_analyse.py`; raw logs in
`~/logs/ab/` on the robot). Each: Nav2 up with `/scan_fused` as the single
source, 45 s settle, 450 s of `tf_watch.py` tracing an independent buffer, four
in-place turns — two early, two about 9 minutes after launch.

| launch | rate | froze? | onset | `tf_watch` during the freeze |
|---|---|---|---|---|
| before1 | 200 Hz | **yes**, 437.8 s, never recovered | 52 s after active | ≤ 1.0 s stale |
| after1 | 50 Hz | no — 0 stale lines in 7.5 min | — | — |
| before2 | 200 Hz | **yes** | ~9 min after launch | not covered (see below) |
| after2 | 50 Hz | **yes**, 457.5 s, never recovered | 87 s after active | ≤ 1.05 s stale |

**2 of 2 froze at 200 Hz, 1 of 2 at 50 Hz. At n=2 that difference is noise, and
the rate change does not prevent the freeze.** Load was indistinguishable too
(end-of-run load average 13.6–14.7 at both rates), so with no measured benefit
the config was **reverted to 5e-3** — every earlier observation of the freeze
was made at 200 Hz, and the next test should be as well. Actual map->odom on the wire was
59–88 Hz when 200 was asked for and 32–34 Hz for 50: Cartographer never met
either request on this Pi.

**What the runs did establish — explanation (b), observed directly, twice.** In
before1 both buffers received the same transform, stamped `1790751630.346`;
`tf_watch`'s kept advancing and was never more than 1.0 s stale, while
`controller_server`'s held that one stamp until teardown. after2 shows the same
(independent buffer ≤ 1.05 s stale throughout a 457 s controller freeze). So
Cartographer keeps publishing and an ordinary listener keeps receiving:
**`controller_server`'s own listener stops being fed.** Cartographer (a) is now
properly ruled out, for those two freezes.

**The freeze does not need a goal and does not need `one_goal.py`.** In before1
it began 13 s before the first goal was sent.

**Every freeze observed so far followed a `/tf` subscriber joining or leaving
the graph** — 09-29 (`one_goal.py` joining, ~1 s), before1 (`tf_watch`'s
subscription coming up, ~2 s), before2 (a probe's `one_goal.py` shutting down,
~1.3 s), after2 (a probe's `one_goal.py` starting, 2–8 s). Four of four, but not
deterministic: each launch has about ten such events and most trigger nothing,
and after1 had all of them and never froze. It points at DDS discovery or
transport rather than at Nav2's code, and it is the next thing to test
directly — join and leave `/tf` subscribers on purpose, with no goals at all.
It is a correlation, **not** a cause. Fast DDS 2.6.12 / `rmw_fastrtps_cpp`
6.2.10; no shared-memory transport errors in any of the four launches, with
`/dev/shm` cleared before the drivers started.

**A frozen controller does not fail safe, and not always the same way.** before1:
turns reported "Reached the goal!" 32 ms after acceptance with no rotation —
the documented false success. before2 and after2: turns ran on until "Failed to
make progress" or the leash, and in after2 the robot rotated 135–150° for a 90°
request while the controller steered against a pose it could no longer update.
The leash and the cancel-all stopped every one.

**Two flaws in the method, for whoever repeats it.**

- `tf_watch` runs 450 s from settle and the late probes start just as it ends,
  so it has no samples for the late window. before2's freeze fell there. Give
  it at least 520 s.
- **The IMU is not a reliable ground truth at this load.** During the runs it
  reported turns of the wrong sign (−23.9° against TF's +68°; −61° against
  +66.6°), with `incomplete BNO055 read` and `non-unit quaternion (norm ~2.2)`
  in the driver log. `one_goal.py --turn` judges false success by the IMU, so
  its `rc=2` flags this session are not evidence; the analysis used
  `controller_server`'s own stale-TF lines, which do not depend on it. The same
  IMU feeds `scan_fusion`'s yaw correction and Cartographer.

### The join/leave test, first result (2026-09-30)

`scripts/tf_join_launch.sh` + `tf_join_probe.py`: Nav2 up with motion locked
out (velocity_smoother max/min_velocity set to zero and read back, and
`/cmd_vel_nav` confirmed to have no other reader), then `controller_server`
probed every 10 s through its own FollowPath action — the only detector, since
a frozen controller logs nothing while idle — first quietly, then with
processes joining and leaving the graph on a schedule.

- **join1** stopped at the motion lock, as designed: `max_velocity` read back
  unchanged and no probe was sent. The set had never reached the node — with
  the CLI daemon stopped, `ros2 param` reported `Node not found` and
  `Wait for service timed out` on first attempts. Now retried with read-back.
- **join2** was frozen at its first probe, before the quiet phase began. The
  frozen Transform time, `…055.741`, falls inside the retried `ros2 param
  set/get` calls of the motion lock (~`…041`–`…057`). That is a fifth freeze
  within seconds of a participant joining or leaving — and this participant,
  a `ros2 param` CLI node, has **no `/tf` subscription at all**. One point
  against "/tf matching specifically", for participant discovery in general.
  Still a correlation.

**Thread snapshot of the frozen `controller_server`** (`/proc`, 16 threads,
two samples 7 s apart; aarch64 syscall 98 = futex, 207 = recvfrom): one thread
had used 3.2 s of CPU and then used **none**, parked in `futex_wait`, while the
two Fast DDS receive threads stayed active in `recvfrom` — packets were still
reaching the process. A healthy TF listener wakes on every message at 60+ Hz,
so a thread that worked and then went fully idle mid-stream is the candidate
stalled listener. What `/proc` cannot say is why: an executor idly waiting for
data also sits in `futex_wait`, so "blocked on a lock" and "never handed
another sample" look the same from here. A backtrace would settle it; no
debugger is installed on the robot.

**Design flaw for the next run:** the motion lock's own CLI calls are
join/leave events, and the freeze can happen during them, so the quiet phase
is never reached. The caps should be set and verified from inside the one
long-lived probe process instead.

**Working hypothesis, not established:** `/tf` is RELIABLE, KEEP_LAST.
`controller_server`'s listener is a slow reader — the Humble source puts the
obstacle layer's scan callback on that same thread — and a burst of discovery
traffic when a participant joins or leaves could make it slower still. If the
writer's history overwrites samples that reader has not acknowledged and the
reliable stream then fails to recover past the gap, delivery to that one
reader would stop permanently while fast readers carry on. That would fit
everything seen so far: load-dependent, near join/leave events, one reader
only, never recovers. It is a guess about Fast DDS internals and needs either
a backtrace or a change of DDS implementation to test.

**That guess was wrong.** The backtrace below replaces it.

### The cause: an ABBA deadlock in tf2 0.25.23 (2026-09-30, backtrace)

join3 (gdb installed on the robot for this) froze in its stimulus phase, 2.6 s
after a plain participant joined, and was backtraced twice, 5 s apart. Against
a genuine healthy baseline taken 6 minutes earlier, exactly two threads that
had been busy went to zero CPU, and both were blocked **acquiring mutexes** —
`futex_wait(expected=2)` under `pthread_mutex_lock`, not the condition-variable
wait an idle executor shows:

| thread | what it is | where it is stuck |
|---|---|---|
| LWP 36805 | the TF listener's dedicated thread | `subscription_callback` -> `setTransform` -> `testTransformableRequests` -> Buffer's callback -> `pthread_mutex_lock` |
| LWP 36806 | the costmap's executor thread | obstacle layer `LaserScan` -> `Buffer::waitForTransform` -> `addTransformableRequest` -> `pthread_mutex_lock` |

In the tf2 source **at the installed version** (0.25.23):

- `BufferCore::testTransformableRequests` takes `transformable_requests_mutex_`
  and still holds it when it invokes each ready request's callback. That
  callback is `tf2_ros::Buffer`'s, and it takes `timer_to_request_map_mutex_`.
- `tf2_ros::Buffer::waitForTransform` takes `timer_to_request_map_mutex_` and,
  while holding it, calls `addTransformableRequest`, which takes
  `transformable_requests_mutex_`.

Opposite lock order on two threads: each holds what the other wants, forever.
The listener thread never inserts another transform, so the buffer freezes at
whatever it last received; nothing throws, so nothing is logged.

**Fixed upstream:** geometry2 `9997e9695` on `humble`, 2026-09-10, "Fix ABBA
deadlock between `waitForTransform` and `testTransformableRequests`"
(backport of #982, PR #990), released as tf2 0.25.24. It changes only
`tf2/src/buffer_core.cpp`. The robot and the box both run 0.25.23, built
09-07/09-08; apt's candidate is still 0.25.23 as of 09-30.

**How it explains everything recorded above:**

- Permanent, silent, one process only: a deadlock does not recover, and only a
  buffer with a `MessageFilter` calling `waitForTransform` can take lock B
  first. `tf_watch.py` has none, which is why it never froze.
- A race, so load-dependent and not every launch: the costmap thread must
  register a request (a scan arriving before its transform) in the window
  where the listener is running a ready callback.
- **09-28's two-source A/B was not wrong, just not the whole story**: a second
  observation source doubles the `waitForTransform` registrations, which
  raises the odds of hitting the window. One source lowers the rate; it does
  not remove the race.
- **The join/leave correlation (5 of 5) is timing, not DDS**: a participant
  joining causes a burst of discovery work on an already-loaded Pi, which
  delays TF behind scans and pushes more scans down the `waitForTransform`
  path at the moment TF inserts resume.
- The TF-rate A/B was noise because the rate changes the odds, not the lock
  order.

**Where the reasoning went wrong, so it is not repeated.** On 09-29 the Humble
source was read to look for exactly this kind of lock inversion — and found
none, because the branch head already contained the fix. The code read was not
the code running. The version check (`dpkg -l`, then the source at that
release) took one command and would have ended this on 09-29.

**34 files hardcode the name `tortoisebot`** in frames, topics, package names
and model names.

**Tests cover the logic, not the nodes.** 210 tests run without a robot,
ROS or hardware (`./run_tests.sh`, and CI on every push), but they cover the
pure modules -- `motor_math`, `floor_geometry`, `scan_geometry`, `runaway`,
`places`, `scan_fusion`, `sample_checks` -- rather than the nodes that use
them. Nothing
exercises a running graph: no launch tests, no costmap under a synthetic scan,
nothing that would have caught `floor_scan` hanging `controller_server`. That
class of bug is still found by driving the robot into something.
