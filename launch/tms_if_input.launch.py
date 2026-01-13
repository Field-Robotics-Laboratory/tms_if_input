import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.conditions import IfCondition

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    tms_if_input_dir = get_package_share_directory("tms_if_input")
    json_file_path = os.path.join(tms_if_input_dir, "json_samples", "ver1.json")

    # ---- Launch Arguments ----
    prefix_arg = DeclareLaunchArgument('prefix', default_value='tms_if_input')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')


    prefix = LaunchConfiguration('prefix')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        prefix_arg,
        use_namespace_arg,
        use_sim_time_arg,

        GroupAction([

            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=prefix,
            ),

            Node(
                package='tms_if_input',
                executable='loader_from_path',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'json_path': json_file_path,
                }],
            ),

            Node(
                package='tms_if_input',
                executable='scenario_segmentor',
                parameters=[{'use_sim_time': use_sim_time}],
            ),

            Node(
                package='tms_if_input',
                executable='taskset_compiler',
                parameters=[{'use_sim_time': use_sim_time}],
            ),

        ]),
    ])