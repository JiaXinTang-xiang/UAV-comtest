"""PX4 real sensors and route mission; test injection is off by default."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import yaml


def nodes(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    interfaces = yaml.safe_load(Path(value('interface_file')).read_text()) or {}
    remappings = list(interfaces.get('remappings', {}).items())
    common = {'config_file': value('config_file'), 'world_file': value('world_file'), 'route': value('route'),
              'enable_tests': value('enable_tests').lower() == 'true', 'log_dir': value('log_dir')}
    return [
        Node(package='uav_competition', executable='px4_offboard', output='screen',
             parameters=[dict(common, hover_test=False, require_sensor_preflight=True)], remappings=remappings),
        Node(package='uav_competition', executable='px4_sensor_adapter', output='screen',
             parameters=[{'enable_tests': common['enable_tests']}], remappings=remappings),
        Node(package='uav_competition', executable='preflight_check', output='screen', remappings=remappings),
        Node(package='uav_competition', executable='px4_region_mission', output='screen',
             parameters=[dict(common, world_file=value('world_file'))], remappings=remappings),
    ]


def generate_launch_description():
    share = FindPackageShare('uav_competition')
    cfg = PathJoinSubstitution([share, 'config', 'mission.yaml'])
    world = PathJoinSubstitution([share, 'worlds', 'competition_course.world'])
    return LaunchDescription([
        DeclareLaunchArgument('route', default_value='right', description='Key in config_file route_profiles'),
        DeclareLaunchArgument('enable_tests', default_value='false'),
        DeclareLaunchArgument('log_dir', default_value=str(Path.home() / 'uav_logs')),
        DeclareLaunchArgument('config_file', default_value=cfg),
        DeclareLaunchArgument('world_file', default_value=world),
        DeclareLaunchArgument('interface_file', default_value=PathJoinSubstitution([share, 'config', 'interfaces.yaml'])),
        OpaqueFunction(function=nodes),
    ])
