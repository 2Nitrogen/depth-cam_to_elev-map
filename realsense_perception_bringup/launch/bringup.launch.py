"""Bringup: state estimator + elevation mapper (+ optional RViz).

This launch assumes the RealSense ROS2 wrapper (realsense2_camera) is
started separately, since the camera launch is environment-specific.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    mapper_pkg = FindPackageShare('realsense_elevation_mapper')
    estimator_pkg = FindPackageShare('realsense_state_estimator')
    bringup_pkg = FindPackageShare('realsense_perception_bringup')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz with the default perception config.',
    )

    estimator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [estimator_pkg, 'launch', 'state_estimator.launch.py']
        )),
    )

    mapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [mapper_pkg, 'launch', 'local_elevation_mapper.launch.py']
        )),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([bringup_pkg, 'rviz', 'default.rviz']),
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        rviz_arg,
        estimator_launch,
        mapper_launch,
        rviz_node,
    ])
