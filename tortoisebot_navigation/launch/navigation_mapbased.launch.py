#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import ReplaceString, RewrittenYaml


# Relative tf names let PushRosNamespace move TF onto <ns>/tf, giving each
# robot its own tree. That is why no frame name in the yaml needs a prefix.
# Every node that publishes or listens to TF needs these; missing them on a
# single node silently corrupts the tree.
TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():

    nav_pkg     = get_package_share_directory('tortoisebot_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml     = LaunchConfiguration('map')
    namespace     = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    set_initial_pose = LaunchConfiguration('set_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    raw_params = PythonExpression([
        "'", os.path.join(nav_pkg, 'config', 'nav2_params_simulation.yaml'), "' if '",
        use_sim_time, "' == 'true' or '", use_sim_time, "' == 'True' else '",
        os.path.join(nav_pkg, 'config', 'nav2_params_robot.yaml'), "'"
    ])

    # '' when there is no namespace, '/robot1' when there is. See
    # navigation_slam.launch.py for why the costmap topics need this literal
    # substitution rather than a relative name.
    ns_prefix = PythonExpression(
        ["'/' + '", namespace, "' if '", namespace, "' else ''"])

    replaced_params = ReplaceString(
        source_file=raw_params,
        replacements={'<robot_namespace>': ns_prefix},
    )

    params_file = RewrittenYaml(
        source_file=replaced_params,
        root_key=namespace,
        param_rewrites={},
        convert_types=True,
    )

    default_map = os.path.join(nav_pkg, 'maps', 'explored_map.yaml')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file'
    )
    declare_set_initial_pose = DeclareLaunchArgument(
        'set_initial_pose',
        default_value='True',
        description='Whether AMCL should be seeded automatically at startup'
    )
    declare_initial_pose_x = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='AMCL initial pose X in map frame'
    )
    declare_initial_pose_y = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='AMCL initial pose Y in map frame'
    )
    declare_initial_pose_yaw = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='AMCL initial yaw in map frame'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }],
        remappings=TF_REMAPPINGS
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'set_initial_pose': set_initial_pose,
            'initial_pose.x': initial_pose_x,
            'initial_pose.y': initial_pose_y,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': initial_pose_yaw,
            'initial_pose.cov_x': 0.5,
            'initial_pose.cov_y': 0.5,
            'initial_pose.cov_yaw': 0.5,
        }],
        remappings=TF_REMAPPINGS + [('scan', 'scan')]
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=TF_REMAPPINGS
    )

    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=TF_REMAPPINGS
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        # Must be cmd_vel_nav, not cmd_vel: velocity_smoother listens on
        # cmd_vel_nav and republishes to cmd_vel. Publishing straight to
        # cmd_vel left the smoother with zero publishers, so nothing was
        # acceleration-limited. behavior_server deliberately keeps publishing
        # to cmd_vel directly, matching upstream nav2.
        remappings=TF_REMAPPINGS + [('cmd_vel', 'cmd_vel_nav')]
    )

    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=TF_REMAPPINGS
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=TF_REMAPPINGS
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=TF_REMAPPINGS
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=TF_REMAPPINGS + [
            ('cmd_vel',          'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': [
                'map_server',
                'amcl',
            ]
        }]
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': [
                'planner_server',
                'smoother_server',
                'controller_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ]
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace. Empty means no namespace.'
        ),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='False',
            description='Whether to push the nav2 stack into "namespace"'
        ),
        declare_sim_time,
        declare_map,
        declare_set_initial_pose,
        declare_initial_pose_x,
        declare_initial_pose_y,
        declare_initial_pose_yaw,

        GroupAction([
            PushRosNamespace(namespace=namespace,
                             condition=IfCondition(use_namespace)),
            map_server,
            amcl,
            planner,
            smoother,
            controller,
            behavior,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            # PushRosNamespace does not survive into a TimerAction: the group's
            # context scope is popped before the timer fires, so a deferred node
            # lands outside the namespace. Each delayed node therefore gets its
            # own group.
            TimerAction(period=3.0, actions=[GroupAction([
                PushRosNamespace(namespace=namespace,
                                 condition=IfCondition(use_namespace)),
                lifecycle_manager_localization,
            ])]),
            TimerAction(period=8.0, actions=[GroupAction([
                PushRosNamespace(namespace=namespace,
                                 condition=IfCondition(use_namespace)),
                lifecycle_manager_navigation,
            ])]),
        ]),
    ])
