#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            OpaqueFunction, GroupAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


# Turns the absolute /tf and /tf_static into node relative names so that
# PushRosNamespace can move them under the robot's namespace. This is what
# gives each robot its own TF tree, and it is why none of the frame names in
# the .lua files need a prefix. Every node that touches TF needs this; missing
# it on one node silently corrupts the tree.
TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def lua_for(is_odom_only, use_sim, rigid_frame):
    """Pick the Cartographer config. No ROS needed, so it is unit tested.

    rigid_frame only applies to the real robot's SLAM config: it selects
    slam_real_rigid.lua, which switches off global loop closure (T-001).
    """
    if is_odom_only:
        return 'slam_real_mapbased.lua'
    if use_sim:
        return 'slam_sim.lua'
    return 'slam_real_rigid.lua' if rigid_frame else 'slam_real.lua'


def launch_setup(context, *args, **kwargs):
    slam_pkg = get_package_share_directory('tortoisebot_slam')
    config_dir = os.path.join(slam_pkg, 'config')

    use_sim_time   = LaunchConfiguration('use_sim_time')
    resolution     = LaunchConfiguration('resolution')
    publish_period = LaunchConfiguration('publish_period_sec')
    namespace      = LaunchConfiguration('namespace')
    use_namespace  = LaunchConfiguration('use_namespace')

    use_sim_str = context.perform_substitution(use_sim_time).lower()
    is_odom_only_str = context.perform_substitution(LaunchConfiguration('is_odom_only')).lower()
    
    rigid_str = context.perform_substitution(LaunchConfiguration('rigid_frame')).lower()
    lua_basename = lua_for(is_odom_only_str == 'true', use_sim_str == 'true',
                           rigid_str == 'true')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', lua_basename,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=TF_REMAPPINGS + [
            ('scan', 'scan'),
            ('odom', 'odom'),
            ('imu',  'imu'),
        ],
        output='screen'
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=TF_REMAPPINGS,
        output='screen'
    )

    nodes = [cartographer_node]
    if is_odom_only_str != 'true':
        nodes.append(occupancy_grid_node)

    return [GroupAction([
        PushRosNamespace(namespace=namespace,
                         condition=IfCondition(use_namespace)),
    ] + nodes)]


def generate_launch_description():

    return LaunchDescription([

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace. Empty means no namespace.'
        ),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='False',
            description='Whether to push the nodes into "namespace"'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='True = simulation (slam_sim.lua), False = real robot (slam_real.lua / slam_real_mapbased.lua)'
        ),
        DeclareLaunchArgument(
            'is_odom_only',
            default_value='false',
            description='If true, runs in odometry-only mode (does not publish map frame)'
        ),
        DeclareLaunchArgument(
            'rigid_frame',
            default_value='false',
            description='If true, real-robot SLAM uses slam_real_rigid.lua (no global loop closure, T-001)'
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Occupancy grid resolution in metres per cell'
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='How often the occupancy grid is published'
        ),

        OpaqueFunction(function=launch_setup),
    ])
