#!/usr/bin/env python3
import launch
import os
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
import launch_ros
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import ReplaceString

# RViz subscribes to TF like any other node, so it needs the same relative
# names to see a namespaced robot's tree.
TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='tortoisebot_description'
    ).find('tortoisebot_description')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/simulation.rviz')
    namespaced_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_namespaced.rviz')

    rvizconfig    = LaunchConfiguration('rvizconfig')
    namespace     = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time  = LaunchConfiguration('use_sim_time')

    # An RViz config stores absolute topic names, so unlike a node it cannot
    # simply be pushed into a namespace. Two variants are kept instead, which
    # is what nav2_bringup does with nav2_namespaced_view.rviz. The namespaced
    # one carries <robot_namespace> placeholders that are filled in here.
    namespaced_config = ReplaceString(
        source_file=namespaced_rviz_config_path,
        replacements={'<robot_namespace>': ('/', namespace)},
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file (ignored when use_namespace is True)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='namespace',
            default_value='',
            description='Robot namespace. Empty means no namespace.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_namespace',
            default_value='False',
            description='Whether to start the namespaced RViz variant'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Flag to enable use_sim_time'
        ),

        Node(
            condition=UnlessCondition(use_namespace),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rvizconfig],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        launch.actions.GroupAction(
            condition=IfCondition(use_namespace),
            actions=[
                PushRosNamespace(namespace=namespace),
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', namespaced_config],
                    parameters=[{'use_sim_time': use_sim_time}],
                    remappings=TF_REMAPPINGS,
                ),
            ]
        ),
    ])
