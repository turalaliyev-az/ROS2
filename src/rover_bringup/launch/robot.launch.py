import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Whole robot, as started at boot: sensors/motors plus the web control server.

    The web server then starts navigation or mapping itself, depending on what
    the operator picks in the browser.
    """
    rover_bringup_share = get_package_share_directory('rover_bringup')
    lidar_rear_crop = LaunchConfiguration('lidar_rear_crop')
    web_port = LaunchConfiguration('web_port')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_bringup_share, 'launch', 'bringup.launch.py')),
        launch_arguments={'lidar_rear_crop': lidar_rear_crop}.items(),
    )

    web = Node(
        package='rover_web',
        executable='rover_web_server',
        name='rover_web_server',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        # On shutdown it first stops the Nav2/mapping processes it started
        # (up to ~10 s); don't escalate to SIGTERM before that is done.
        sigterm_timeout='15',
        parameters=[{'port': web_port}],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('lidar_rear_crop', default_value='true'))
    ld.add_action(DeclareLaunchArgument('web_port', default_value='8080'))
    ld.add_action(bringup)
    ld.add_action(web)
    return ld
