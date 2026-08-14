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
- Cartographer's `tracking_frame = "imu_link"` **asserts** the IMU frame is
  colocated with the tracking frame within `1e-5` m. Moving the IMU without
  updating this aborts Cartographer at startup with a colocation assert.
- `voxel_layer.origin_z: -0.2` in `nav2_params_robot.yaml` is a discard floor.
  It was lowered from `0.0` because the lidar sits `0.057` m above `base_link`
  under the current frame choice, leaving only `0.057` m of headroom — a 2°
  pitch would sink a 2 m return below the floor and silently drop it. The
  current value gives `0.257` m, roughly 7°.

### 3.5 Sensor rates

`ydlidar.yaml frequency: 10.0`; `controller_frequency: 10.0`;
`smoothing_frequency: 20.0`; IMU `rate_hz: 50.0` (a node parameter, so it can
be lowered on a slower I2C bus without rebuilding).

---

## 4. Build targets

The robot and simulation driver stacks are mutually exclusive. `ydlidar_ros2_driver`
requires the YDLidar SDK installed system-wide and does not build on a normal
workstation; Ignition is pointless on the robot.

`tortoisebot_bringup/package.xml` expresses this with REP-149 conditional
dependencies keyed on `TORTOISEBOT_TARGET`:

```bash
# Simulation workstation (default when the variable is unset)
colcon build --packages-up-to tortoisebot_bringup tortoisebot_control

# Physical robot
export TORTOISEBOT_TARGET=robot
rosdep install --from-paths src --ignore-src -r -y
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

## 5. Known gaps

These are real limits of the current code, listed so nobody rediscovers them.

**No hardware abstraction.** There is no `ros2_control` layer. `differential.py`
talks directly to GPIO pins. Swapping motor hardware means rewriting that node
rather than selecting a plugin. Closing this properly requires wheel encoders,
which this chassis does not have — without them there is no velocity feedback to
close a loop around, and no `odom` on the real robot.

**Not namespace-safe — cannot yet run two robots on one DDS domain.** No launch
file uses `PushRosNamespace`. Topic namespacing alone would not be sufficient:
there are 26+ hardcoded TF frame names (`map`, `odom`, `base_link`, `imu_link`)
spread across six config files in `tortoisebot_navigation/config` and
`tortoisebot_slam/config`. Two robots would publish colliding `map` and `odom`
frames into one TF tree. A real fix needs frame prefixing everywhere
(`robot_state_publisher`'s `frame_prefix`, Cartographer's four frame settings,
the EKF's four, and every Nav2 `global_frame`/`robot_base_frame`), and the
`ros_gz_bridge` arguments name the gz and ROS topics with a single string, so
the bridge needs converting to its YAML config form to namespace only the ROS
side. Interim workaround for isolating robots: separate `ROS_DOMAIN_ID`.

**Third-party drivers vendored in-tree.** `v4l2_camera`, `ydlidar_ros2_driver`
and `YDLidar-SDK` are pinned copies rather than vcs-managed dependencies.
Changing lidar means editing this repository. Note also that
`autobringup.launch.py` launches `camera_ros`, which is a *different* package
from the vendored `v4l2_camera` and is not installed on the development machine.

**34 files hardcode the name `tortoisebot`** in frames, topics, package names
and model names.

**No CI and no tests** beyond the default ament linters.
