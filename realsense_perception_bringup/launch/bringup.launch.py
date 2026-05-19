"""Bringup: state estimator + elevation mapper (+ optional RViz).

This launch assumes the RealSense ROS2 wrapper (realsense2_camera) is
started separately, since the camera launch is environment-specific.

When source:=rosbag, a static TF (parent->child as configured in
camera_mount.json) is spawned to connect the state_estimator's
odom->base_link subtree to the bag's camera_link->camera_*_optical_frame
subtree. use_sim_time is also forced to true in this mode, since rosbag
playback only makes sense with --clock + sim time.
"""
import json

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    source = LaunchConfiguration('source').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    rviz_str = LaunchConfiguration('rviz').perform(context)
    camera_mount_config = LaunchConfiguration('camera_mount_config').perform(context)

    # `choices=` on the LaunchArgument already validates, but be explicit
    # for the few code paths below that branch on the value.
    effective_use_sim_time = 'true' if source == 'rosbag' else use_sim_time_str
    use_sim_time_bool = effective_use_sim_time.lower() == 'true'

    mapper_pkg = FindPackageShare('realsense_elevation_mapper').perform(context)
    estimator_pkg = FindPackageShare('realsense_state_estimator').perform(context)
    bringup_pkg = FindPackageShare('realsense_perception_bringup').perform(context)

    actions = []

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{estimator_pkg}/launch/state_estimator.launch.py'
        ),
        launch_arguments={'use_sim_time': effective_use_sim_time}.items(),
    ))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{mapper_pkg}/launch/local_elevation_mapper.launch.py'
        ),
        launch_arguments={'use_sim_time': effective_use_sim_time}.items(),
    ))

    if rviz_str.lower() == 'true':
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', f'{bringup_pkg}/rviz/default.rviz'],
            parameters=[{'use_sim_time': use_sim_time_bool}],
            output='screen',
        ))

    if source == 'rosbag':
        with open(camera_mount_config, 'r') as f:
            mount = json.load(f)
        t = mount['translation_m']
        r = mount['rotation_rpy_rad']
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_mount_static_tf',
            arguments=[
                '--x', str(t['x']),
                '--y', str(t['y']),
                '--z', str(t['z']),
                '--roll', str(r['roll']),
                '--pitch', str(r['pitch']),
                '--yaw', str(r['yaw']),
                '--frame-id', str(mount['parent_frame']),
                '--child-frame-id', str(mount['child_frame']),
            ],
            parameters=[{'use_sim_time': use_sim_time_bool}],
        ))

    return actions


def generate_launch_description() -> LaunchDescription:
    bringup_pkg = FindPackageShare('realsense_perception_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'source',
            default_value='live',
            choices=['live', 'rosbag'],
            description=(
                'Data source: "live" for a real RealSense camera, "rosbag" '
                'for replayed data. When "rosbag", use_sim_time is forced '
                'to true and a static TF (parent/child from camera_mount_config) '
                'is published to connect base_link with camera_link.'
            ),
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz with the default perception config.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description=(
                'Use /clock instead of system time. Ignored (forced true) '
                'when source=rosbag.'
            ),
        ),
        DeclareLaunchArgument(
            'camera_mount_config',
            default_value=PathJoinSubstitution(
                [bringup_pkg, 'config', 'camera_mount.json']
            ),
            description=(
                'Path to JSON describing the base_link->camera_link mount '
                'transform. Used only when source=rosbag (live mode is '
                'expected to get this TF from URDF / robot_state_publisher).'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
