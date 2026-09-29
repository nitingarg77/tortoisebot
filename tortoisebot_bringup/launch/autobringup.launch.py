#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    desc_pkg    = get_package_share_directory('tortoisebot_description')
    slam_pkg    = get_package_share_directory('tortoisebot_slam')
    nav_pkg     = get_package_share_directory('tortoisebot_navigation')

    # Resolved lazily, via substitution rather than get_package_share_directory,
    # so that neither package has to be installed unless it is actually used.
    # ydlidar_ros2_driver is robot only and tortoisebot_gazebo is simulation
    # only; an eager lookup here would make every robot install the simulator
    # and every simulator install the Raspberry Pi lidar driver.
    lidar_pkg   = FindPackageShare('ydlidar_ros2_driver')
    gazebo_pkg  = FindPackageShare('tortoisebot_gazebo')

    default_map     = os.path.join(nav_pkg,   'maps',   'explored_map.yaml')
    sim_rviz_config = os.path.join(desc_pkg,  'rviz',   'simulation.rviz')
    nav_rviz_config = os.path.join(desc_pkg,  'rviz',   'nav2.rviz')
    lidar_params    = PathJoinSubstitution([lidar_pkg, 'params', 'ydlidar.yaml'])
    real_urdf       = os.path.join(desc_pkg, 'models', 'urdf', 'tortoisebotreal.xacro')
    ekf_slam_params = os.path.join(slam_pkg, 'config', 'ekf.yaml')
    explore_params  = os.path.join(nav_pkg,  'config', 'explore_params_robot.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration  = LaunchConfiguration('exploration')
    slam_only    = LaunchConfiguration('slam_only')
    auto_explore = LaunchConfiguration('auto_explore')
    map_file     = LaunchConfiguration('map_file')
    camera_port  = LaunchConfiguration('camera_port')
    use_camera   = LaunchConfiguration('camera')
    stack        = LaunchConfiguration('stack')
    use_rviz     = LaunchConfiguration('rviz')
    namespace     = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    # Forwarded verbatim to every included launch file. Each one declares the
    # same pair and pushes its own nodes, rather than this file wrapping the
    # includes in one group, because the Gazebo process and the gz side of the
    # bridge must stay outside any namespace.
    ns_args = {
        'namespace':     namespace,
        'use_namespace': use_namespace,
    }

    world_file = PathJoinSubstitution(
        [gazebo_pkg, 'worlds', 'nav2_test_world.sdf'])

    ignition_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_pkg, 'launch', 'ignition_sim.launch.py'])),
        launch_arguments={
            'world':   world_file,
            'spawn_x': '0.0',
            'spawn_y': '0.0',
            **ns_args,
        }.items(),
        condition=IfCondition(use_sim_time)
    )

    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'False', **ns_args}.items(),
        condition=UnlessCondition(use_sim_time)
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([lidar_pkg, 'launch', 'ydlidar_launch.py'])),
        launch_arguments={'params_file': lidar_params}.items(),
        condition=UnlessCondition(use_sim_time)
    )

    # BNO055 IMU node. Real robot only -- in sim the Ignition imu_sensor feeds
    # /imu through ros_gz_bridge instead. Publishes /imu, which is what
    # cartographer.launch.py remaps imu to.
    imu = Node(
        package='tortoisebot_imu',
        executable='imu_node.py',
        name='imu_publisher',
        output='screen',
        condition=UnlessCondition(use_sim_time)
    )


    motors = Node(
        package='tortoisebot_firmware',
        executable='differential.py',
        name='differential',
        output='screen',
        parameters=[{
            # This robot's left motor is the weaker of the two. Pivoting on one
            # wheel at a time and reading the IMU: the right wheel managed 0.140
            # and 0.122 m/s on two tries, the left 0.116 and 0.107, a ratio of
            # 0.83-0.88. Untrimmed it curved 30-60 deg/m and wandered into
            # whatever was on its left, which is how it kept clipping wall
            # panels and wedged itself in a doorway.
            #
            # 1.28 was picked by measurement, not arithmetic, and it is a local
            # best: 1.25 and 1.31 both came out worse (-2.9 and -3.0 deg/m
            # forward, against -1.5 at 1.28). Out-and-back runs at 1.28 give
            # -1.5 deg/m forward and +0.3 reverse with a spread of +/-0.5,
            # against +/-36 before trimming. The leftover 1-3 deg/m is floor,
            # slip and battery sag rather than the wheels, so tuning further
            # chases noise.
            #
            # It is specific to this robot's motors and will not carry to
            # another chassis. Recalibrate with scripts/calibrate_drive.py.
            'left_trim': 1.28,
        }],
        condition=UnlessCondition(use_sim_time)
    )

    camera = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[
            {'camera': camera_port},
            {'format': 'RGB888'},
            {'width': 640},
            {'height': 480},
            # 10 fps, not the 18-30 the sensor free-runs at. floor_scan only
            # ticks at 5 Hz, and every extra frame is a 900 kB message that
            # camera_node serialises and floor_scan deserialises for nothing.
            # That traffic, on top of Cartographer, is what starved
            # controller_server's transform listener: its map->odom froze and
            # it reported every goal reached without the robot moving.
            {'FrameDurationLimits': [100000, 100000]},
        ],
        # Real robot only, and optional: at 800x600 RGB888 the camera costs a
        # Pi 4 about 20% CPU that exploration does not use (camera:=False).
        condition=IfCondition(PythonExpression([
            "'true' if ('", use_sim_time, "' == 'false' or '", use_sim_time, "' == 'False') and ('", use_camera, "' == 'true' or '", use_camera, "' == 'True') else 'false'"
        ]))
    )

    # Camera obstacles, for the walls the lidar cannot see. Runs beside the
    # camera, so it goes with the robot-side group; costs little because it
    # works on a 320-wide copy at 5 Hz, from a camera limited to 10 fps.
    floor_scan = Node(
        package='tortoisebot_navigation',
        executable='floor_scan.py',
        name='floor_scan',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # camera_ros publishes under its node name, so with name
            # 'camera_node' the images are on /camera_node/image_raw, not the
            # /camera/image_raw the node defaults to.
            'image_topic': '/camera_node/image_raw',
        }],
        condition=IfCondition(PythonExpression([
            "'true' if ('", use_sim_time, "' == 'false' or '", use_sim_time,
            "' == 'False') and ('", use_camera, "' == 'true' or '", use_camera,
            "' == 'True') else 'false'"
        ]))
    )

    # Merges /floor_scan into /scan and publishes /scan_fused, which is the
    # only observation source both costmaps take. It replaces listing `scan`
    # and `floor_scan` separately, which gave each costmap layer a second
    # tf2_ros::MessageFilter and hung controller_server -- see MODULE.md.
    #
    # Deliberately NOT gated on use_camera. With the camera off it passes the
    # lidar scan straight through, which is what keeps Nav2 seeing anything at
    # all; gating it here would delete the costmaps' only source. It runs on
    # the robot side because that is where all three inputs are, so one topic
    # crosses to the compute side instead of two.
    scan_fusion = Node(
        package='tortoisebot_navigation',
        executable='scan_fusion_node.py',
        name='scan_fusion',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_sim_time))

    # EKF node — disabled (not used in any pipeline)
    # ekf = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_slam_params, {'use_sim_time': False}],
    #     condition=IfCondition(PythonExpression([
    #         "'true' if ('", use_sim_time, "' == 'false' or '", use_sim_time, "' == 'False') and ('", exploration, "' == 'false' or '", exploration, "' == 'False') else 'false'"
    #     ]))
    # )

    cartographer = TimerAction(
        period=6.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg, 'launch', 'cartographer.launch.py')),
            condition=IfCondition(PythonExpression([
                "'true' if ('", exploration, "' == 'true' or '", exploration, "' == 'True') or ('", use_sim_time, "' == 'false' or '", use_sim_time, "' == 'False') else 'false'"
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'is_odom_only': PythonExpression(["'false' if '", exploration, "' == 'true' or '", exploration, "' == 'True' else 'true'"]),
                **ns_args,
            }.items()
        )]
    )

    navigation = TimerAction(
        period=14.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_pkg, 'launch', 'navigation_mapbased.launch.py')),
            condition=UnlessCondition(exploration),
            launch_arguments={
                'map':         map_file,
                'use_sim_time': use_sim_time,
                **ns_args,
            }.items()
        )]
    )

 
    navigation_slam = TimerAction(
        period=20.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_pkg, 'launch', 'navigation_slam.launch.py')),
            condition=IfCondition(PythonExpression([
                "'true' if ('", exploration, "' == 'true' or '", exploration, "' == 'True') and ('", slam_only, "' == 'false' or '", slam_only, "' == 'False') else 'false'"
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                **ns_args,
            }.items()
        )]
    )

    # Frontier explorer, alongside Nav2 in exploration mode. It always starts,
    # but only drives off on its own with auto_explore:=True; otherwise it
    # idles until started from its RViz panel or control_exploration service.
    # Launched after navigation_slam (20 s) so Nav2's action server is up.
    explorer = TimerAction(
        period=30.0,
        actions=[GroupAction([
            PushRosNamespace(namespace=namespace,
                             condition=IfCondition(use_namespace)),
            Node(
                package='frontier_exploration_ros2',
                executable='frontier_explorer',
                name='frontier_explorer',
                output='screen',
                parameters=[explore_params, {
                    'use_sim_time': use_sim_time,
                    'autostart':    auto_explore,
                }],
            ),
        ])],
        condition=IfCondition(PythonExpression([
            "'true' if ('", exploration, "' == 'true' or '", exploration, "' == 'True') and ('", slam_only, "' == 'false' or '", slam_only, "' == 'False') else 'false'"
        ]))
    )

    # Stops the explorer and cancels Nav2 goals if the SLAM pose runs away
    # (see pose_watchdog.py). Same condition as Nav2: nothing to cancel without
    # it, and in slam_only mode its zero cmd_vel would only fight teleop.
    pose_watchdog = GroupAction([
        PushRosNamespace(namespace=namespace,
                         condition=IfCondition(use_namespace)),
        Node(
            package='tortoisebot_navigation',
            executable='pose_watchdog.py',
            name='pose_watchdog',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ], condition=IfCondition(PythonExpression([
        "'true' if ('", exploration, "' == 'true' or '", exploration, "' == 'True') and ('", slam_only, "' == 'false' or '", slam_only, "' == 'False') else 'false'"
    ])))

    rviz = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'rviz.launch.py')),
            launch_arguments={
                'rvizconfig': nav_rviz_config,
                'use_sim_time': use_sim_time,
                **ns_args,
            }.items(),
            condition=IfCondition(PythonExpression([
                "'true' if ('", exploration, "' == 'true' or '", exploration, "' == 'True') and ('", slam_only, "' == 'false' or '", slam_only, "' == 'False') else 'false'"
            ]))
        )]
    )

    rviz_slam = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'rviz.launch.py')),
            launch_arguments={
                'rvizconfig': nav_rviz_config,
                'use_sim_time': use_sim_time,
                **ns_args,
            }.items(),
            condition=IfCondition(PythonExpression([
                "'true' if ('", exploration, "' == 'true' or '", exploration, "' == 'True') and ('", slam_only, "' == 'true' or '", slam_only, "' == 'True') else 'false'"
            ]))
        )]
    )

    rviz_map = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'rviz.launch.py')),
            launch_arguments={
                'rvizconfig': sim_rviz_config,
                'use_sim_time': use_sim_time,
                **ns_args,
            }.items(),
            condition=UnlessCondition(exploration)
        )]
    )

    return LaunchDescription([

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('use_sim_time', default_value='True',
                              description='True=Ignition Sim, False=Real Robot'),
        DeclareLaunchArgument('exploration',  default_value='True',
                              description='True=SLAM mapping, False=Map-based Nav'),
        DeclareLaunchArgument('slam_only',     default_value='False',
                              description='True=SLAM-only mapping (Cartographer, no Nav2), False=Standard mapping (SLAM + Nav2)'),
        DeclareLaunchArgument('auto_explore', default_value='False',
                              description='True=frontier explorer drives off as soon as it starts, False=it waits for the RViz panel (exploration mode only)'),
        DeclareLaunchArgument('map_file',     default_value=default_map,
                              description='Path to saved map yaml (used when exploration=False)'),
        DeclareLaunchArgument('stack',        default_value='all',
                              choices=['all', 'robot', 'compute'],
                              description='all=everything here, robot=drivers only, compute=SLAM, Nav2, explorer and watchdog only'),
        DeclareLaunchArgument('rviz',         default_value='True',
                              description='Start RViz (compute side only)'),
        DeclareLaunchArgument('camera',       default_value='True',
                              description='Start the camera on the real robot (False saves ~20% CPU on the Pi)'),
        DeclareLaunchArgument('camera_port',  default_value='0',
                              description='Camera port (e.g. 0 for /dev/video0, or /base/soc/...)'),
        DeclareLaunchArgument('namespace',     default_value='',
                              description='Robot namespace. Empty means no namespace.'),
        DeclareLaunchArgument('use_namespace', default_value='False',
                              description='Whether to push the whole stack into "namespace"'),


        # stack:=robot runs only what needs the hardware; stack:=compute runs
        # SLAM, Nav2, the explorer and the watchdog, e.g. on a desktop on the
        # same network when the Pi 4 cannot keep up. stack:=all (default) runs
        # everything in one place, as before.
        GroupAction([
            ignition_sim,
            state_publisher,

            # The robot-only drivers are grouped rather than sent namespace
            # arguments, because ydlidar_launch.py is third-party and does not
            # declare them; passing an undeclared argument to an included launch
            # file is an error. PushRosNamespace applies to included files too, so
            # the group achieves the same thing without touching vendor code.
            GroupAction([
                PushRosNamespace(namespace=namespace,
                                 condition=IfCondition(use_namespace)),
                lidar,
                imu,
                motors,
                camera,
                floor_scan,
                scan_fusion,
            ]),
        ], condition=IfCondition(PythonExpression(["'", stack, "' in ('all', 'robot')"]))),

        GroupAction([
            cartographer,
            navigation,
            navigation_slam,
            explorer,
            pose_watchdog,
        ], condition=IfCondition(PythonExpression(["'", stack, "' in ('all', 'compute')"]))),

        GroupAction([
            rviz,
            rviz_slam,
            rviz_map,
        ], condition=IfCondition(PythonExpression([
            "'", use_rviz, "'.lower() == 'true' and '", stack, "' in ('all', 'compute')"]))),
    ])
