from ament_index_python import get_package_share_directory
import launch
import os
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression
import launch_ros
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterValue

# See tortoisebot_slam/launch/cartographer.launch.py for why this exists:
# relative tf names let PushRosNamespace give each robot its own TF tree.
TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    default_model_path = os.path.join(get_package_share_directory('tortoisebot_description'), 'models/urdf/tortoisebot_simple.xacro')
    real_robot_model_path = os.path.join(get_package_share_directory('tortoisebot_description'), 'models/urdf/tortoisebotreal.xacro')
    
    robot_state_publisher_sim = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)
        }],
        remappings=TF_REMAPPINGS,
        condition=IfCondition(use_sim_time)
    )
    
    robot_state_publisher_real = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(Command(['xacro ', real_robot_model_path]), value_type=str)
        }],
        remappings=TF_REMAPPINGS,
        condition=UnlessCondition(use_sim_time)
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='namespace',
            default_value='',
            description='Robot namespace. Empty means no namespace.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_namespace',
            default_value='False',
            description='Whether to push the nodes into "namespace"'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='False',
            description='Flag to enable use_sim_time'
        ),
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file (used only when use_sim_time is True)'
        ),
        launch.actions.GroupAction([
            PushRosNamespace(namespace=namespace,
                             condition=IfCondition(use_namespace)),
            robot_state_publisher_sim,
            robot_state_publisher_real,
            joint_state_publisher_node,
        ]),
    ])
