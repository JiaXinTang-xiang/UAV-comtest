"""VM generation must preserve the approved course and physical vehicle."""
from pathlib import Path
import runpy
import xml.etree.ElementTree as ET

import pytest
import yaml

from uav_competition.path_planner import CoursePlanner

PACKAGE = Path(__file__).parents[1]
ROOT = Path(__file__).parents[4]
GENERATE = runpy.run_path(str(ROOT / 'scripts/prepare_vm_scene.py'))['generate']


@pytest.fixture
def vm_world(tmp_path):
    return GENERATE(tmp_path / 'vm.world', tmp_path / 'gui.so')


def test_vm_keeps_course_collisions_and_both_routes(vm_world):
    original = CoursePlanner(PACKAGE / 'worlds/competition_course.world')
    lightweight = CoursePlanner(vm_world)
    assert original.boxes == lightweight.boxes
    cfg = yaml.safe_load((PACKAGE / 'config/mission.yaml').read_text())
    for profile in cfg['route_profiles'].values():
        planner = CoursePlanner(vm_world, profile['clearance'])
        goals = [cfg['mission']['takeoff'], *profile['targets'].values(),
                 cfg['mission']['landing_target']]
        for start, goal in zip(goals, goals[1:]):
            path = planner.plan(start, goal)
            assert all(planner.visible(a, b) for a, b in zip(path, path[1:]))


def test_vm_keeps_dynamics_and_px4_plugins(vm_world):
    original = ET.parse(PACKAGE / 'models/iris_training/iris.sdf').getroot().find('model')
    vm = ET.parse(vm_world).getroot().find("world/model[@name='iris']")

    def structure(element):
        return (element.tag, element.attrib, (element.text or '').strip(),
                [structure(child) for child in element])

    for link in original.findall('link'):
        counterpart = vm.find(f"link[@name='{link.get('name')}']")
        for tag in ('pose', 'inertial', 'collision'):
            assert [structure(e) for e in link.findall(tag)] == [
                structure(e) for e in counterpart.findall(tag)]
    for tag in ('joint', 'plugin'):
        for element in original.findall(tag):
            if element.get('name') == 'training_ground_truth':
                continue
            assert structure(element) == structure(vm.find(f"{tag}[@name='{element.get('name')}']"))
    assert not vm.findall('.//mesh')


def test_vm_has_real_sensors_and_valid_lockstep_settings(vm_world):
    world = ET.parse(vm_world).getroot().find('world')
    step = float(world.findtext('physics/max_step_size'))
    rate = int(world.findtext('physics/real_time_update_rate'))
    assert rate == 250 and rate % 250 == 0 and step == 1 / rate
    assert world.findtext('physics/real_time_factor') == '1'
    assert world.findtext('scene/shadows') == 'false'
    vm = world.find("model[@name='iris']")
    flow = vm.find("link/sensor[@name='training_flow_camera']")
    assert flow.findtext('camera/image/width') == '64'
    assert flow.findtext('camera/image/height') == '64'
    assert flow.findtext('update_rate') == '20'
    assert flow.find("plugin[@filename='libgazebo_opticalflow_plugin.so']") is not None
    assert flow.find("plugin[@name='flow_image_ros']") is None
    lidar = vm.find("link/sensor[@name='training_scan']")
    assert lidar.findtext('update_rate') == '10'
    assert lidar.findtext('ray/scan/horizontal/samples') == '180'
    assert lidar.findtext('ray/range/max') == '15'
    assert vm.find("model[@name='lidar']/link/sensor/plugin[@filename='libgazebo_lidar_plugin.so']") is not None
