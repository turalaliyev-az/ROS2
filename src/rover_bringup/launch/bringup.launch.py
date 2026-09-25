import glob
import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_port(udev_link, by_id_glob, fallback):
    """Prefer a device name that survives re-plugging over the kernel's ttyACMn/ttyUSBn."""
    if os.path.exists(udev_link):
        return udev_link
    matches = sorted(glob.glob(os.path.join('/dev/serial/by-id', by_id_glob)))
    return matches[0] if matches else fallback


def generate_launch_description():
    rover_bringup_share = get_package_share_directory('rover_bringup')
    xacro_path = os.path.join(rover_bringup_share, 'urdf', 'rover.urdf.xacro')

    use_lidar = LaunchConfiguration('use_lidar')
    use_camera = LaunchConfiguration('use_camera')
    esp32_port = LaunchConfiguration('esp32_port')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_rear_crop = LaunchConfiguration('lidar_rear_crop')

    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar', default_value='true')
    declare_use_camera = DeclareLaunchArgument(
        'use_camera', default_value='true')
    # /dev/serial/by-id names come from the default udev rules (no sudo
    # needed) and stay the same across reconnects, unlike ttyACM0/ttyACM1.
    declare_esp32_port = DeclareLaunchArgument(
        'esp32_port',
        default_value=_resolve_port(
            '/dev/rover_esp32', '*USB_Single_Serial_5C4E000865*', '/dev/ttyACM0'))
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value=_resolve_port('/dev/rover_lidar', '*CP2102*', '/dev/ttyUSB0'))
    declare_lidar_rear_crop = DeclareLaunchArgument(
        'lidar_rear_crop', default_value='true',
        description='Mask the 20 deg behind the lidar (only needed while something sits there)')

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
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            'serial_port': esp32_port,
            'baud_rate': 115200,
            'wheel_diameter_m': 0.09022,
            'wheel_separation_m': 0.44,
            'ticks_per_rev_left': 74,
            'ticks_per_rev_right': 73,
            # Hard cap per wheel, above anything Nav2 asks for (0.2 m/s +
            # 0.5 rad/s turn = 0.31 m/s on the outer wheel); mainly limits
            # teleop, whose default speed is 0.5 m/s.
            'max_wheel_speed_mps': 0.35,
            'max_linear_accel_mps2': 0.3,
            'max_linear_decel_mps2': 1.0,
            'max_angular_accel_rps2': 1.0,
            'max_angular_decel_rps2': 2.0,
            # ekf_node owns odom->base_footprint and publishes the fused /odom;
            # this wheel-only estimate feeds it as an input.
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
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'topic_name': 'scan',
            'frame_id': 'laser_frame',
            'port_name': lidar_port,
            'port_baudrate': 230400,
            'laser_scan_dir': True,
            # Something carried on the robot (probably the laptop) showed up at
            # a fixed ~0.46 m directly behind (published /scan 170-180 deg).
            # Turn this off when nothing sits there, so the rear isn't blind.
            # The crop compares the lidar's native clockwise angle, and
            # laser_scan_dir mirrors it (published = 360 - native), so native
            # 175-195 is what masks published 165-185.
            'enable_angle_crop_func': lidar_rear_crop,
            'angle_crop_min': 175.0,
            'angle_crop_max': 195.0,
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
            # A LaserScan needs a z-up frame; the optical frame (z forward)
            # turns the scan plane vertical.
            'output_frame': 'camera_depth_frame',
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
    ld.add_action(declare_esp32_port)
    ld.add_action(declare_lidar_port)
    ld.add_action(declare_lidar_rear_crop)
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
