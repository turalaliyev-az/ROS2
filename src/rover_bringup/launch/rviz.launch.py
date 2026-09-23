import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    rover_bringup_share = get_package_share_directory('rover_bringup')

    declare_config = DeclareLaunchArgument(
        'config', default_value='mapping',
        description="Which rviz config to load: 'mapping' or 'nav'")

    rviz_config = PathJoinSubstitution(
        [rover_bringup_share, 'rviz', [LaunchConfiguration('config'), '.rviz']])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    ld = LaunchDescription()
    ld.add_action(declare_config)
    ld.add_action(rviz_node)
    return ld
