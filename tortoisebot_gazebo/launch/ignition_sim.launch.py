#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, DeclareLaunchArgument,
                             SetEnvironmentVariable, TimerAction, GroupAction,
                             OpaqueFunction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


# Relative tf names let PushRosNamespace move TF onto <ns>/tf, giving each
# robot its own tree, which is why no frame name anywhere needs a prefix.
TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

# The Ignition side of every bridged topic. The parameter_bridge argument
# syntax names the gz topic and the ROS topic with a single string, and this
# ros_gz_bridge (0.244.25) has no config_file option to separate them, so the
# arguments stay absolute to match the SDF and the ROS side is moved into the
# namespace by remapping instead.
#
# /clock is deliberately NOT in this list. It is a global concept: namespacing
# it would leave every node with use_sim_time waiting for a clock that nobody
# publishes.
BRIDGED_TOPICS = [
    '/cmd_vel',
    '/odom',
    '/scan',
    '/imu',
    '/camera/image_raw',
    '/camera/camera_info',
]


def bridge_setup(context, *args, **kwargs):
    """Build the main bridge node.

    This needs an OpaqueFunction because the qos_overrides parameter names
    embed the fully qualified topic name, which is only known once the
    namespace has been resolved. A static dict would keep pointing at /scan
    after the ROS side had moved to /robot1/scan, silently losing the
    best_effort override that RViz needs to display the scan.
    """
    ns = context.perform_substitution(LaunchConfiguration('namespace'))
    use_ns = context.perform_substitution(
        LaunchConfiguration('use_namespace')).lower() in ('true', '1')
    prefix = f'/{ns}' if (use_ns and ns) else ''

    return [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # NOTE: DiffDrive's odom->base_link TF is intentionally NOT bridged.
            # Fortress's DiffDrive ignores <publish_tf>false</publish_tf>, so
            # bridging it here made it compete with ekf_filter_node for the same
            # transform. The EKF (odom + IMU) is the single authority.
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        # Absolute source, relative target: the gz topic stays where the SDF
        # put it while the ROS topic follows the enclosing namespace.
        remappings=[(t, t.lstrip('/')) for t in BRIDGED_TOPICS],
        parameters=[{
            'use_sim_time': True,
            f'qos_overrides.{prefix}/tf_static.publisher.durability': 'transient_local',
            f'qos_overrides.{prefix}/scan.publisher.reliability': 'best_effort',
            f'qos_overrides.{prefix}/odom.publisher.reliability': 'reliable',
        }],
        output='screen'
    )]


def generate_launch_description():

    desc_pkg   = FindPackageShare('tortoisebot_description').find(
                     'tortoisebot_description')
    gazebo_pkg = FindPackageShare('tortoisebot_gazebo').find(
                     'tortoisebot_gazebo')
    nav_pkg    = FindPackageShare('tortoisebot_navigation').find(
                     'tortoisebot_navigation')
    ekf_config    = os.path.join(nav_pkg, 'config', 'ekf_mapbased.yaml')
    default_world = os.path.join(gazebo_pkg, 'worlds', 'nav2_test_world.sdf')
    xacro_file    = os.path.join(
        desc_pkg, 'models', 'urdf', 'tortoisebot_sim.xacro')

    world   = LaunchConfiguration('world')
    gui     = LaunchConfiguration('gui')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    namespace     = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    # Nests the EKF parameters under the namespace so the top level key still
    # matches the node name once it has been pushed into one.
    ekf_params = RewrittenYaml(
        source_file=ekf_config,
        root_key=namespace,
        param_rewrites={},
        convert_types=True,
    )

    return LaunchDescription([

        DeclareLaunchArgument('gui',     default_value='true',
                              description='Launch Gazebo with GUI (false = headless server)'),
        DeclareLaunchArgument('world',   default_value=default_world,
                              description='Path to world SDF file'),
        DeclareLaunchArgument('spawn_x', default_value='0.0',
                              description='Robot spawn X position'),
        DeclareLaunchArgument('spawn_y', default_value='0.0',
                              description='Robot spawn Y position'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot namespace. Empty means no namespace.'),
        DeclareLaunchArgument('use_namespace', default_value='False',
                              description='Whether to push the ROS side into "namespace"'),

        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[
                os.path.join(desc_pkg, '..'), ':',
                os.path.join(gazebo_pkg, '..'), ':',
                os.path.join('/opt/ros/humble', 'share'),
            ]
        ),

        # Gazebo itself is not a ROS node and stays outside the namespace group.
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world],
            output='screen',
            condition=IfCondition(gui)
        ),

        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', '-s', world],
            output='screen',
            condition=UnlessCondition(gui)
        ),

        GroupAction([
            PushRosNamespace(namespace=namespace,
                             condition=IfCondition(use_namespace)),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': robot_description,
                    'publish_frequency': 50.0,
                }],
                remappings=TF_REMAPPINGS,
            ),

            OpaqueFunction(function=bridge_setup),

            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='joint_state_bridge',
                arguments=[
                    '/world/default/model/tortoisebot/joint_state'
                    '@sensor_msgs/msg/JointState[ignition.msgs.Model'
                ],
                remappings=[(
                    '/world/default/model/tortoisebot/joint_state',
                    'joint_states'
                )],
                parameters=[{'use_sim_time': True}],
                output='screen'
            ),

            # PushRosNamespace does not survive into a TimerAction: the group's
            # context scope is popped before the timer fires, so a deferred node
            # lands outside the namespace. That left this node subscribing to
            # /robot_description while robot_state_publisher published on
            # /robot1/robot_description, and the robot never spawned. Each
            # delayed node therefore gets its own group.
            TimerAction(period=2.0, actions=[GroupAction([
                PushRosNamespace(namespace=namespace,
                                 condition=IfCondition(use_namespace)),
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    output='screen',
                    arguments=[
                        '-name', 'tortoisebot',
                        '-topic', 'robot_description',
                        '-x', spawn_x,
                        '-y', spawn_y,
                        '-z', '0.01',
                    ],
                )
            ])]),

            # Fills in the covariances ignition.msgs.IMU and ignition.msgs.Odometry
            # have no fields for, republishing onto imu_with_covariance and
            # odom_with_covariance for the EKF. The bridge's own imu and odom are
            # left alone -- nav2, cartographer and the velocity smoother read those
            # and ignore covariance entirely. All of its topic names are already
            # relative, so it follows the namespace without further work.
            Node(
                package='tortoisebot_gazebo',
                executable='sim_covariance_relay.py',
                name='sim_covariance_relay',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),

            TimerAction(period=4.0, actions=[GroupAction([
                PushRosNamespace(namespace=namespace,
                                 condition=IfCondition(use_namespace)),
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[ekf_params],
                    remappings=TF_REMAPPINGS + [
                        ('odometry/filtered', 'odometry/filtered'),
                    ],
                )
            ])]),
        ]),
    ])
