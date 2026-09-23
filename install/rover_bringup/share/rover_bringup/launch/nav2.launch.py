import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rover_bringup_share = get_package_share_directory('rover_bringup')
    default_map = os.path.join(rover_bringup_share, 'maps', 'restaurant_map_v2.yaml')
    default_params = os.path.join(rover_bringup_share, 'config', 'nav2_params.yaml')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_map = DeclareLaunchArgument('map', default_value=default_map)
    declare_params = DeclareLaunchArgument('params_file', default_value=default_params)

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': 'false',
            'slam': 'False',
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_map)
    ld.add_action(declare_params)
    ld.add_action(nav2_launch)
    return ld
