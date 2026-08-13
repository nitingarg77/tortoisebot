#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, DeclareLaunchArgument,
                             SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    gz_scan_topic = '/scan'

    return LaunchDescription([

        DeclareLaunchArgument('gui',     default_value='true',
                              description='Launch Gazebo with GUI (false = headless server)'),
        DeclareLaunchArgument('world',   default_value=default_world,
                              description='Path to world SDF file'),
        DeclareLaunchArgument('spawn_x', default_value='0.0',
                              description='Robot spawn X position'),
        DeclareLaunchArgument('spawn_y', default_value='0.0',
                              description='Robot spawn Y position'),

        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[
                os.path.join(desc_pkg, '..'), ':',
                os.path.join(gazebo_pkg, '..'), ':',
                os.path.join('/opt/ros/humble', 'share'),
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description,
                'publish_frequency': 50.0,
            }]
        ),

        #Main ros_gz_bridge
        Node(
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
                gz_scan_topic + '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            ],
            remappings=[
                (gz_scan_topic, '/scan'),
            ],
            parameters=[{
                'use_sim_time': True,
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
                'qos_overrides./scan.publisher.reliability': 'best_effort',
                'qos_overrides./odom.publisher.reliability': 'reliable',
            }],
            output='screen'
        ),

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
                '/joint_states'
            )],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

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

        TimerAction(period=2.0, actions=[
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
        ]),

        # Fills in the covariances ignition.msgs.IMU and ignition.msgs.Odometry
        # have no fields for, republishing onto /imu_with_covariance and
        # /odom_with_covariance for the EKF. The bridge's own /imu and /odom are
        # left alone -- nav2, cartographer and the velocity smoother read those
        # and ignore covariance entirely.
        Node(
            package='tortoisebot_gazebo',
            executable='sim_covariance_relay.py',
            name='sim_covariance_relay',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        TimerAction(period=4.0, actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_config],
                remappings=[
                    ('odometry/filtered', '/odometry/filtered'),
                ],
            )
        ]),
    ])
