#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from nav2_common.launch import ReplaceString, RewrittenYaml


# Relative tf names let PushRosNamespace move TF onto <ns>/tf, giving each
# robot its own tree. That is why no frame name in the yaml needs a prefix.
#
# They are applied with SetRemap only when use_namespace is true. Applied
# unconditionally, as a per-node remapping, they broke the default path even
# though '/tf' and 'tf' resolve to the same topic without a namespace: the
# controller's map->odom went stale partway through a run ("Transform data too
# old") while AMCL was still publishing it at 10 Hz. A single goal still
# succeeded, which is how it got past testing; a 10-goal course reached 1-4/10.
# The root cause is not yet understood, so the namespaced path is unverified
# beyond one goal. See MODULE.md section 5.
TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():

    nav_pkg = get_package_share_directory('tortoisebot_navigation')

    use_sim_time  = LaunchConfiguration('use_sim_time')
    namespace     = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    raw_params = PythonExpression([
        "'", os.path.join(nav_pkg, 'config', 'nav2_params_simulation.yaml'), "' if '",
        use_sim_time, "' == 'true' or '", use_sim_time, "' == 'True' else '",
        os.path.join(nav_pkg, 'config', 'nav2_params_robot.yaml'), "'"
    ])

    # '' when there is no namespace, '/robot1' when there is. Substituting this
    # rather than conditioning ReplaceString off means one params file serves
    # both paths, instead of upstream nav2's duplicated default/multirobot pair.
    ns_prefix = PythonExpression(
        ["'/' + '", namespace, "' if '", namespace, "' else ''"])

    # The costmap observation-source topics cannot be relative, because costmap
    # layers live on a child node and this nav2 build has no
    # joinWithParentNamespace to reroute them. They carry a <robot_namespace>
    # placeholder which is substituted literally here.
    replaced_params = ReplaceString(
        source_file=raw_params,
        replacements={'<robot_namespace>': ns_prefix},
    )

    # Nests the whole file under the namespace so the top level keys still match
    # the node names once they have been pushed into it.
    params_file = RewrittenYaml(
        source_file=replaced_params,
        root_key=namespace,
        param_rewrites={},
        convert_types=True,
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
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
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel',          'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )

    # node_names stay unqualified on purpose: the lifecycle manager resolves
    # them against its own namespace, so they follow the group automatically.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),

        GroupAction([
            PushRosNamespace(namespace=namespace,
                             condition=IfCondition(use_namespace)),
            *[SetRemap(src=src, dst=dst, condition=IfCondition(use_namespace))
              for src, dst in TF_REMAPPINGS],
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
            TimerAction(period=2.0, actions=[GroupAction([
                PushRosNamespace(namespace=namespace,
                                 condition=IfCondition(use_namespace)),
                lifecycle_manager,
            ])]),
        ]),
    ])
