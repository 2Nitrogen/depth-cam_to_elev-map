"""Local fast-mesh baseline launch.

Mirrors realsense_surfel_mapper / realsense_elevation_mapper launch
patterns: params_file argument defaulting to share/.../config/params.yaml,
plus use_sim_time injected as a runtime parameter dict.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('realsense_fast_mesh_baseline')
    default_params = PathJoinSubstitution([pkg, 'config', 'params.yaml'])

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to params yaml',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock (e.g. when replaying a rosbag with --clock).',
    )

    node = Node(
        package='realsense_fast_mesh_baseline',
        executable='local_fast_mesh_node',
        name='local_fast_mesh_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([params_file_arg, use_sim_time_arg, node])
