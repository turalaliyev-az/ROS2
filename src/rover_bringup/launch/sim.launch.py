import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    rover_bringup_share = get_package_share_directory('rover_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    loopback_sim_share = get_package_share_directory('nav2_loopback_sim')

    xacro_path = os.path.join(rover_bringup_share, 'urdf', 'rover.urdf.xacro')
    default_map = os.path.join(rover_bringup_share, 'maps', 'restaurant_map.yaml')
    default_params = os.path.join(rover_bringup_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(rover_bringup_share, 'rviz', 'nav.rviz')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    declare_map = DeclareLaunchArgument('map', default_value=default_map)
    declare_params = DeclareLaunchArgument('params_file', default_value=default_params)
    declare_autostart = DeclareLaunchArgument('autostart', default_value='true')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    # No AMCL here: nav2_loopback_sim publishes ground-truth map->odom and
    # fakes odom->base_footprint straight from cmd_vel, so there is no
    # localization uncertainty to fight. This isolates whether our own
    # nav2_params.yaml (footprint, costmap, planner, controller) can
    # actually drive the robot to a clicked goal, independent of the real
    # robot's AMCL-convergence and sensor-noise problems.
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'True',
            'params_file': params_file,
            'autostart': autostart,
            'use_localization': 'False',
        }.items(),
    )

    loopback_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loopback_sim_share, 'loopback_simulation.launch.py')),
        launch_arguments={
            'params_file': params_file,
            # loopback_simulation.launch.py applies this as its own
            # override parameter AFTER loading params_file, so setting
            # scan_frame_id inside nav2_params.yaml alone has no effect --
            # it must be passed here too, matching our URDF's lidar frame.
            'scan_frame_id': 'laser_frame',
        }.items(),
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    start_map_server = GroupAction(
        actions=[
            SetParameter('use_sim_time', True),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[configured_params, {'yaml_filename': map_yaml}],
            ),
            # map_server is a standalone process here (not composed into
            # nav2_container), so without a head start the lifecycle
            # manager's configure call can reach it before its lifecycle
            # service server is even registered -- observed as an
            # instant (sub-millisecond) "Failed to change state" followed
            # by "Aborting bringup", with map_server only getting around
            # to configuring ~0.5s later, too late to matter.
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_map_server',
                        output='screen',
                        parameters=[
                            configured_params,
                            {'autostart': autostart}, {'node_names': ['map_server']}],
                    ),
                ],
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_map)
    ld.add_action(declare_params)
    ld.add_action(declare_autostart)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(start_map_server)
    ld.add_action(loopback_sim_cmd)
    ld.add_action(rviz_node)
    ld.add_action(bringup_cmd)
    return ld
