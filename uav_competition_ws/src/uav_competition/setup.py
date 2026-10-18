from setuptools import setup
from glob import glob
import os

package_name = 'uav_competition'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        *[(os.path.join('share', package_name, base),
           [os.path.join(base, f) for f in files])
          for base, dirs, files in os.walk('models', followlinks=True) if files],
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'preflight_check = uav_competition.preflight_check:main',
        'sensor_adapter = uav_competition.sensor_adapter:main',
        'mission_controller = uav_competition.mission_controller:main',
        'safety_manager = uav_competition.safety_manager:main',
        'mock_flight = uav_competition.mock_flight:main',
        'px4_offboard = uav_competition.px4_offboard_node:main',
        'px4_region_mission = uav_competition.px4_region_mission:main',
        'px4_sensor_adapter = uav_competition.px4_sensor_adapter:main',
    ]},
)
