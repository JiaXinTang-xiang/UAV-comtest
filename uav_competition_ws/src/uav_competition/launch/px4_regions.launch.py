"""PX4 real sensors and route mission; test injection is off by default."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('uav_competition')
    cfg = PathJoinSubstitution([share, 'config', 'mission.yaml'])
    world = PathJoinSubstitution([share, 'worlds', 'competition_course.world'])
    tests = ParameterValue(LaunchConfiguration('enable_tests'), value_type=bool)
    route = LaunchConfiguration('route')
    logs = LaunchConfiguration('log_dir')
    return LaunchDescription([
        DeclareLaunchArgument('route', default_value='right', choices=['right', 'left']),
        DeclareLaunchArgument('enable_tests', default_value='false'),
        DeclareLaunchArgument('log_dir', default_value=str(Path.home() / 'uav_logs')),
        Node(package='uav_competition', executable='px4_offboard', output='screen', parameters=[{
            'config_file': cfg, 'route': route, 'enable_tests': tests,
            'hover_test': False, 'require_sensor_preflight': True, 'log_dir': logs}]),
        Node(package='uav_competition', executable='px4_sensor_adapter', output='screen', parameters=[{'enable_tests': tests}]),
        Node(package='uav_competition', executable='preflight_check', output='screen'),
        Node(package='uav_competition', executable='px4_region_mission', output='screen', parameters=[{
            'config_file': cfg, 'world_file': world, 'log_dir': logs, 'route': route, 'enable_tests': tests}]),
    ])
