import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """SLAM plus Nav2's navigation servers (no AMCL/map_server): drive or explore while mapping."""
    rover_bringup_share = get_package_share_directory('rover_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    params_file = LaunchConfiguration('params_file')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(rover_bringup_share, 'config', 'nav2_params.yaml'))

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_bringup_share, 'launch', 'slam.launch.py')))

    # All Nav2 servers in one process, as nav2.launch.py does: with a process
    # per server the lifecycle manager sometimes asked a server for its state
    # before discovery had found it, and aborted the whole bring-up.
    container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        parameters=[params_file, {'autostart': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )

    # slam_toolbox provides map->odom and /map; the global costmap's static
    # layer and the planner work on that live map.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'True',
            'container_name': 'nav2_container',
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_params)
    ld.add_action(slam)
    ld.add_action(container)
    ld.add_action(navigation)
    return ld
