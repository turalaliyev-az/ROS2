import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rover_bringup_share = get_package_share_directory('rover_bringup')
    xacro_path = os.path.join(rover_bringup_share, 'urdf', 'rover.urdf.xacro')

    use_lidar = LaunchConfiguration('use_lidar')
    use_camera = LaunchConfiguration('use_camera')

    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar', default_value='true')
    declare_use_camera = DeclareLaunchArgument(
        'use_camera', default_value='true')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    motor_bridge_node = Node(
        package='motor_bridge',
        executable='motor_bridge_node',
        name='motor_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baud_rate': 115200,
            'wheel_diameter_m': 0.09022,
            'wheel_separation_m': 0.44,
            'ticks_per_rev_left': 74,
            'ticks_per_rev_right': 73,
            'max_pwm': 500,
            'max_wheel_speed_mps': 0.5,
            # ekf_node (see below) now owns the odom->base_footprint TF
            # and publishes the fused result as /odom; this node's own
            # wheel-only estimate is renamed out of the way so it feeds
            # the filter as an input instead of colliding with it.
            'publish_tf': False,
        }],
        remappings=[('odom', 'wheel_odom')],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(rover_bringup_share, 'config', 'ekf.yaml')],
        remappings=[('odometry/filtered', 'odom')],
    )

    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ldlidar_node',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'topic_name': 'scan',
            'frame_id': 'laser_frame',
            'port_name': '/dev/ttyUSB0',
            'port_baudrate': 230400,
            'laser_scan_dir': True,
            # Robot's own mount/bracket sits in the lidar's scan plane at a
            # fixed bearing -- confirmed via 59 scans (6s), 42/59 close hits
            # all landing at 170-180 deg, ~0.46m, essentially zero variance.
            # No real obstacle: with nothing physically near the robot the
            # reading stayed identical, so it's self-detection, not clutter.
            # Cropped narrowly (165-185, only 20 deg) rather than reusing
            # the old 90 deg placeholder -- that width isn't needed here and
            # would blind a much wider rear arc than the mount actually
            # occupies.
            'enable_angle_crop_func': True,
            'angle_crop_min': 165.0,
            'angle_crop_max': 185.0,
        }],
        condition=IfCondition(use_lidar),
    )

    # Turns the RealSense depth image into a second LaserScan so obstacles
    # at the camera's height (e.g. tabletops, chair backs) that sit outside
    # the lidar's fixed scan plane still show up for the local costmap.
    # scan_height samples a vertical band of pixel rows (not just the
    # center row) so thin objects like chair/table legs are more likely to
    # be caught even if they only occupy part of that band.
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_scan',
        output='screen',
        parameters=[{
            'scan_height': 40,
            'range_min': 0.3,
            'range_max': 8.0,
            'output_frame': 'camera_depth_optical_frame',
        }],
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/camera/depth/camera_info'),
            ('scan', '/camera_scan'),
        ],
        condition=IfCondition(use_camera),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_lidar)
    ld.add_action(declare_use_camera)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(motor_bridge_node)
    ld.add_action(ekf_node)
    ld.add_action(ldlidar_node)
    ld.add_action(depth_to_scan_node)

    try:
        realsense_share = get_package_share_directory('realsense2_camera')
    except PackageNotFoundError:
        realsense_share = None

    if realsense_share is not None:
        realsense_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_share, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'camera_name': 'camera',
                'camera_namespace': 'camera',
                'enable_color': 'true',
                'enable_depth': 'true',
                'pointcloud.enable': 'true',
                # Full-res depth produced a ~137k-point cloud per frame,
                # pegging the nav2 container at 100% CPU once both
                # costmaps raytraced it (observed 2026-09-21). Decimation
                # magnitude 3 cuts that by roughly 9x -- still dense
                # enough to catch chair/table legs, light enough to keep
                # goals responsive.
                'decimation_filter.enable': 'true',
                'decimation_filter.filter_magnitude': '3',
            }.items(),
            condition=IfCondition(use_camera),
        )
        ld.add_action(realsense_launch)

    return ld
