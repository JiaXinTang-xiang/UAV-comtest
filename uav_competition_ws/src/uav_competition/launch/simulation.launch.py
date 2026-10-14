from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    world=PathJoinSubstitution([FindPackageShare('uav_competition'),'worlds','competition_course.world'])
    cfg=PathJoinSubstitution([FindPackageShare('uav_competition'),'config','mission.yaml'])
    use_mock=LaunchConfiguration('use_mock')
    gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('gazebo_ros'),'launch','gazebo.launch.py'])), launch_arguments={'world':world,'verbose':'true'}.items())
    nodes=[Node(package='uav_competition',executable='sensor_adapter',name='sensor_adapter',output='screen'),Node(package='uav_competition',executable='preflight_check',name='preflight_check',output='screen'),Node(package='uav_competition',executable='safety_manager',name='safety_manager',parameters=[{'config_file':cfg}],output='screen'),Node(package='uav_competition',executable='mission_controller',name='mission_controller',parameters=[{'config_file':cfg}],output='screen'),Node(package='uav_competition',executable='mock_flight',name='mock_flight',condition=IfCondition(use_mock),output='screen')]
    return LaunchDescription([DeclareLaunchArgument('use_mock',default_value='true',description='Use kinematic mock until PX4 SITL is installed.'),gazebo,*nodes])
