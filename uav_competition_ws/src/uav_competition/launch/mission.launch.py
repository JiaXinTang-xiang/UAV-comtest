from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    cfg=PathJoinSubstitution([FindPackageShare('uav_competition'),'config','mission.yaml'])
    return LaunchDescription([DeclareLaunchArgument('route',default_value='right'),Node(package='uav_competition',executable='preflight_check',parameters=[{'mock_mode':True}],output='screen'),Node(package='uav_competition',executable='sensor_adapter',output='screen'),Node(package='uav_competition',executable='safety_manager',parameters=[{'config_file':cfg}],output='screen'),Node(package='uav_competition',executable='mission_controller',parameters=[{'config_file':cfg,'route':LaunchConfiguration('route')}],output='screen')])
