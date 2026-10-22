#!/usr/bin/env python3
"""Generate a lighter scene from the approved course, preserving collisions."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'uav_competition_ws/src/uav_competition'


def put(parent, tag, value):
    child = parent.find(tag)
    if child is None:
        child = ET.SubElement(parent, tag)
    child.text = str(value)
    return child


def box_visual(link, name, size, pose, color):
    visual = ET.SubElement(link, 'visual', name=name)
    put(visual, 'pose', pose)
    box = ET.SubElement(ET.SubElement(visual, 'geometry'), 'box')
    put(box, 'size', size)
    material = ET.SubElement(visual, 'material')
    put(material, 'ambient', color)
    put(material, 'diffuse', color)
    put(visual, 'cast_shadows', 'false')


def generate(output, gui_plugin=None):
    tree = ET.parse(PACKAGE / 'worlds/competition_course.world')
    world = tree.getroot().find('world')
    # PX4 Classic's lockstep interface explicitly supports 250 Hz / 4 ms.
    # Do not change real-time factor: mission deadlines use wall-clock time.
    physics = world.find('physics')
    put(physics, 'max_step_size', '0.004')
    put(physics, 'real_time_update_rate', '250')
    put(physics, 'real_time_factor', '1')
    scene = world.find('scene')
    put(scene, 'shadows', 'false')
    put(scene, 'grid', 'false')
    put(scene, 'origin_visual', 'false')
    # A directional light replaces the external sun, with no sky or shadows.
    for include in list(world.findall('include')):
        if include.findtext('uri') == 'model://sun':
            world.remove(include)
    light = ET.SubElement(world, 'light', name='vm_light', type='directional')
    put(light, 'cast_shadows', 'false')
    put(light, 'diffuse', '0.8 0.8 0.8 1')
    put(light, 'specular', '0 0 0 1')
    put(light, 'direction', '-0.5 0.1 -0.9')
    if gui_plugin:
        plugin = ET.SubElement(world.find('gui'), 'plugin',
                               name='vm_render_limit', filename=str(gui_plugin.resolve()))
        put(plugin, 'fps', '15')

    vehicle = ET.parse(PACKAGE / 'models/iris_training/iris.sdf').getroot().find('model')
    include = next(i for i in world.findall('include') if i.findtext('uri') == 'model://iris')
    put(vehicle, 'pose', include.findtext('pose'))
    vehicle.set('name', include.findtext('name', 'iris'))
    world.remove(include)
    world.append(vehicle)
    # Only visual meshes are replaced. Every inertia, collision, rotor joint,
    # motor and flight sensor plugin remains in the model.
    base = vehicle.find("link[@name='base_link']")
    for visual in list(base.findall('visual')):
        base.remove(visual)
    box_visual(base, 'vm_body', '0.24 0.18 0.10', '0 0 0 0 0 0', '0.12 0.15 0.18 1')
    box_visual(base, 'vm_arm_a', '0.52 0.025 0.025', '0 0 0.02 0 0 1.02', '0.25 0.25 0.25 1')
    box_visual(base, 'vm_arm_b', '0.52 0.025 0.025', '0 0 0.02 0 0 -1.02', '0.25 0.25 0.25 1')
    box_visual(base, 'vm_front', '0.05 0.07 0.025', '0.12 0 0.06 0 0 0', '0.95 0.25 0.10 1')
    for index in range(4):
        rotor = vehicle.find(f"link[@name='rotor_{index}']")
        for visual in list(rotor.findall('visual')):
            rotor.remove(visual)
        color = '0.15 0.45 0.95 1' if index < 2 else '0.85 0.85 0.85 1'
        box_visual(rotor, f'vm_prop_{index}', '0.256 0.018 0.005', '0 0 0 0 0 0', color)
    camera = vehicle.find("link/sensor[@name='training_flow_camera']")
    put(camera, 'update_rate', '20')
    put(camera.find("plugin[@name='training_optical_flow']"), 'outputRate', '10')
    # The actual optical flow plugin still consumes rendered 64x64 images.
    # Remove the extra ROS camera publisher used only for manual image viewing.
    image_pub = camera.find("plugin[@name='flow_image_ros']")
    if image_pub is not None:
        camera.remove(image_pub)
    scan = vehicle.find("link/sensor[@name='training_scan']")
    put(scan, 'update_rate', '10')
    put(scan.find('ray/scan/horizontal'), 'samples', '180')
    put(vehicle.find("plugin[@name='training_ground_truth']"), 'update_rate', '10')
    # Keep real downward range, but disable its decorative ray visualization.
    lidar_include = next(i for i in vehicle.findall('include') if i.findtext('uri') == 'model://lidar')
    lidar = ET.parse(ROOT / 'PX4-Autopilot/Tools/simulation/gazebo-classic/'
                     'sitl_gazebo-classic/models/lidar/model.sdf').getroot().find('model')
    put(lidar, 'pose', lidar_include.findtext('pose'))
    put(lidar.find('link/sensor'), 'visualize', 'false')
    vehicle.remove(lidar_include)
    vehicle.append(lidar)
    for visual in world.findall('.//visual'):
        put(visual, 'cast_shadows', 'false')
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(output, encoding='utf-8', xml_declaration=True)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'logs/simulation/vm_course.world')
    parser.add_argument('--gui-plugin', type=Path)
    args = parser.parse_args()
    print(generate(args.output, args.gui_plugin))
