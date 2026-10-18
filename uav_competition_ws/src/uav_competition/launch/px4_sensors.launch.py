"""Real PX4 sensors and guarded hover; start flight by service call."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('uav_competition'), 'config', 'mission.yaml'])
    return LaunchDescription([
        Node(package='uav_competition', executable='px4_offboard', output='screen', parameters=[{
            'config_file': config, 'hover_test': True, 'require_sensor_preflight': True,
            'log_dir': '/home/jiaxintang/桌面/bao/logs/flight',
        }]),
        Node(package='uav_competition', executable='px4_sensor_adapter', output='screen'),
        Node(package='uav_competition', executable='preflight_check', output='screen'),
    ])
