"""Bringup: state estimator + elevation mapper (+ optional RViz).

This launch assumes the RealSense ROS2 wrapper (realsense2_camera) is
started separately, since the camera launch is environment-specific.

By default the launch publishes a static `base_link -> camera_link`
transform from `config/camera_mount.json` so a desktop test (no URDF,
just a D435i on a table) has a complete TF chain out of the box. If a
URDF / robot_state_publisher is already publishing that edge, pass
`publish_camera_mount:=false` to avoid a conflict.

When source:=rosbag, use_sim_time is forced to true since rosbag
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    source = LaunchConfiguration('source').perform(context)
    rviz_str = LaunchConfiguration('rviz').perform(context)
    imu_filter_str = LaunchConfiguration('imu_filter').perform(context)
    imu_combine_str = LaunchConfiguration('imu_combine').perform(context)
    publish_camera_mount_str = LaunchConfiguration('publish_camera_mount').perform(context)
    mode = LaunchConfiguration('mode').perform(context)

    use_sim_time_bool = (source == 'rosbag')
    use_sim_time_str = 'true' if use_sim_time_bool else 'false'

    imu_combine_bool = imu_combine_str.lower() == 'true'

    estimator_pkg = FindPackageShare('realsense_state_estimator').perform(context)
    bringup_pkg = FindPackageShare('realsense_perception_bringup').perform(context)

    actions = []

    # imu_combiner_node: merges split /accel/sample + /gyro/sample into
    # /camera/camera/imu (mirroring realsense2_camera's unite_imu_method
    # behavior). The node is adaptive — it introspects the output topic
    # at runtime and goes passive whenever another publisher is present,
    # so it's safe to always launch regardless of whether the wrapper or
    # bag already provides a combined topic.
    if imu_combine_bool:
        actions.append(Node(
            package='realsense_state_estimator',
            executable='imu_combiner_node',
            name='imu_combiner_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_bool}],
        ))

    # imu_filter_madgwick: consumes raw accel+gyro on /camera/camera/imu and
    # publishes /imu/data with .orientation populated. state_estimator's
    # gravity_from_imu estimator depends on this. Disable only if you are
    # supplying /imu/data from elsewhere.
    if imu_filter_str.lower() == 'true':
        actions.append(Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False,
                'use_sim_time': use_sim_time_bool,
            }],
            remappings=[
                ('imu/data_raw', '/camera/camera/imu'),
                ('imu/data', '/imu/data'),
            ],
        ))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{estimator_pkg}/launch/state_estimator.launch.py'
        ),
        launch_arguments={'use_sim_time': use_sim_time_str}.items(),
    ))

    # Mapper selection. elev_map and fast_mesh share the same upstream
    # pipeline (state_estimator, IMU, camera mount static TF). They are
    # mutually exclusive — switching at launch time avoids running
    # multiple mappers on the same input cloud, which would be wasteful.
    if mode == 'elev_map':
        mapper_pkg = FindPackageShare('realsense_elevation_mapper').perform(context)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{mapper_pkg}/launch/local_elevation_mapper.launch.py'
            ),
            launch_arguments={'use_sim_time': use_sim_time_str}.items(),
        ))
    elif mode == 'fast_mesh':
        mapper_pkg = FindPackageShare('realsense_fast_mesh_baseline').perform(context)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{mapper_pkg}/launch/fast_mesh_baseline.launch.py'
            ),
            launch_arguments={'use_sim_time': use_sim_time_str}.items(),
        ))
    else:
        # DeclareLaunchArgument(choices=...) already validates, but defend
        # against bypass via explicit value mismatch.
        raise RuntimeError(
            f"mode must be 'elev_map' or 'fast_mesh', got '{mode}'."
        )

    if rviz_str.lower() == 'true':
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', f'{bringup_pkg}/rviz/default.rviz'],
            parameters=[{'use_sim_time': use_sim_time_bool}],
            output='screen',
        ))

    # Publish the static base_link -> camera_link transform from
    # camera_mount.json. Required for the TF chain in both live (when no
    # URDF is providing it) and rosbag (no URDF at all) modes. Turn off
    # via publish_camera_mount:=false if your URDF / robot_state_publisher
    # already publishes that edge.
    if publish_camera_mount_str.lower() == 'true':
        with open(f'{bringup_pkg}/config/camera_mount.json', 'r') as f:
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
    return LaunchDescription([
        DeclareLaunchArgument(
            'source',
            default_value='live',
            choices=['live', 'rosbag'],
            description=(
                'Data source: "live" for a real RealSense camera, "rosbag" '
                'for replayed data. "rosbag" forces use_sim_time:=true so '
                'every node aligns with /clock.'
            ),
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz with the default perception config.',
        ),
        DeclareLaunchArgument(
            'imu_filter', default_value='true',
            description=(
                'Spawn imu_filter_madgwick to derive /imu/data (with '
                'orientation) from /camera/camera/imu. Set false if an '
                'external node already publishes /imu/data.'
            ),
        ),
        DeclareLaunchArgument(
            'imu_combine', default_value='true',
            choices=['true', 'false'],
            description=(
                'Spawn imu_combiner_node to merge split /accel/sample + '
                '/gyro/sample into /camera/camera/imu. The node is adaptive '
                '(goes passive when another publisher already provides the '
                'combined topic), so leaving this on is safe for both '
                'unite_imu_method=2 and split-mode IMU sources, live or '
                'replayed. Set false to skip launching it entirely.'
            ),
        ),
        DeclareLaunchArgument(
            'publish_camera_mount', default_value='true',
            description=(
                'Publish a static base_link -> camera_link TF from '
                'config/camera_mount.json. Default true so a desktop D435i '
                'test works without a URDF. Set false if your URDF / '
                'robot_state_publisher already publishes that edge.'
            ),
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='elev_map',
            choices=['elev_map', 'fast_mesh'],
            description=(
                'Which mapper to start: "elev_map" (Kalman-fused 2D '
                'elevation grid, default) or "fast_mesh" (single-frame '
                'PCL OrganizedFastMesh baseline with per-vertex slope '
                'coloring). Both share the state_estimator + IMU + '
                'camera_mount upstream pipeline; choose one per launch.'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
