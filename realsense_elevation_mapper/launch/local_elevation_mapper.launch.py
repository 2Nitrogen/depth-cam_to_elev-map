from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('realsense_elevation_mapper')
    default_params = PathJoinSubstitution([pkg, 'config', 'params.yaml'])

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to params yaml',
    )

    node = Node(
        package='realsense_elevation_mapper',
        executable='local_elevation_mapper_node',
        name='local_elevation_mapper_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_file_arg, node])
