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
> `use_namespace:=True`, which restores the default path. The root cause is not
> yet understood, so expect the namespaced path to fail the same way on longer
> runs until it is.

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

**34 files hardcode the name `tortoisebot`** in frames, topics, package names
and model names.

**No CI and no tests** beyond the default ament linters.
