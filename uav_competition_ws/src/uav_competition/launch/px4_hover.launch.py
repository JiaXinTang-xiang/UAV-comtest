"""Connect to an existing PX4/Gazebo instance; start flight via a service."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('uav_competition'), 'config', 'mission.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('log_dir', default_value='/home/jiaxintang/桌面/bao/logs/flight'),
        Node(package='uav_competition', executable='px4_offboard', output='screen', parameters=[{
            'config_file': config,
            'hover_test': True,
            'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool),
            'log_dir': LaunchConfiguration('log_dir'),
        }]),
    ])
