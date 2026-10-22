"""Load planar static collision footprints, including local SDF model includes.

Support boxes, cylinders and spheres. Reject unresolved includes, relative_to
frames and unsupported geometry instead of silently planning through them.
"""
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from .frames import MapFrame, rotate


def pose(element):
    node = element.find('pose')
    if node is not None and node.get('relative_to'):
        raise ValueError('SDF relative_to requires a resolved world export')
    values = list(map(float, element.findtext('pose', '0 0 0 0 0 0').split()))
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('Invalid SDF pose')
    if abs(values[3])+abs(values[4]) > 1e-6:
        raise ValueError('Tilted static collision requires a resolved planar map')
    return values[0], values[1], values[2], values[5]


def compose(a, b):
    x, y = rotate(b[0], b[1], a[3])
    return a[0]+x, a[1]+y, a[2]+b[2], a[3]+b[3]


class Footprint:
    def __init__(self, vertices):
        self.vertices = vertices
        xs, ys = zip(*vertices)
        self.bounds = min(xs), max(xs), min(ys), max(ys)

    def distance(self, point):
        x, y = point
        inside, distance = False, float('inf')
        for a, b in zip(self.vertices, self.vertices[1:]+self.vertices[:1]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            t = max(0, min(1, ((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy))) if dx or dy else 0
            distance = min(distance, math.hypot(x-a[0]-t*dx, y-a[1]-t*dy))
            if (a[1] > y) != (b[1] > y) and x < a[0]+(y-a[1])*dx/dy:
                inside = not inside
        return 0.0 if inside else distance


def load_footprints(world_file, altitude, frame=None, model_paths=()):
    frame = frame or MapFrame()
    world_file = Path(world_file).resolve()
    paths = [Path(p) for p in model_paths if p]
    paths += [Path(p) for p in os.environ.get('GAZEBO_MODEL_PATH', '').split(':') if p]
    paths += [Path(__file__).resolve().parents[1]/'models', world_file.parent.parent/'models', Path.home()/'.gazebo/models',
              Path('/usr/share/gazebo-11/models')]
    # Bundled PX4 sensor models when running from this project checkout.
    for parent in (*world_file.parents, *Path(__file__).resolve().parents):
        models = parent/'PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'
        if models.is_dir():
            paths.append(models)
            break
    result = []

    def resolve(uri, directory):
        if uri.startswith('model://'):
            candidates = [p/uri[8:] for p in paths]
        elif uri.startswith('file://'):
            location = Path(uri[7:])
            candidates = [location if location.is_absolute() else directory/location]
        else:
            candidates = [directory/uri]
        for candidate in candidates:
            if candidate.is_dir():
                cfg = candidate/'model.config'
                sdf = ET.parse(cfg).getroot().findtext('sdf', 'model.sdf') if cfg.exists() else 'model.sdf'
                candidate = candidate/sdf
            if candidate.is_file():
                return candidate.resolve()
        raise ValueError(f'Cannot resolve local model: {uri}; configure model_paths')

    def walk(parent, transform, static, directory, stack):
        for child in parent:
            if child.tag == 'model':
                is_static = static or child.findtext('static', 'false').lower() in ('true', '1')
                # Moving aircraft do not belong in the static planning map.
                if not is_static:
                    continue
                walk(child, compose(transform, pose(child)), True, directory, stack)
            elif child.tag == 'include':
                uri = child.findtext('uri', '')
                path = resolve(uri, directory)
                if path in stack:
                    raise ValueError(f'Cyclic model include: {path}')
                root = ET.parse(path).getroot()
                override = child.findtext('static')
                # Include pose replaces the root model pose in SDF.
                wrapper = ET.Element('sdf')
                for model in root.findall('model'):
                    if override is not None:
                        node = model.find('static')
                        if node is None:
                            node = ET.SubElement(model, 'static')
                        node.text = override
                    old_pose = model.find('pose')
                    if old_pose is not None:
                        model.remove(old_pose)
                    wrapper.append(model)
                walk(wrapper, compose(transform, pose(child)), static, path.parent, stack+(path,))
            elif child.tag == 'link' and static:
                base = compose(transform, pose(child))
                for collision in child.findall('collision'):
                    tf = compose(base, pose(collision))
                    geometry = collision.find('geometry')
                    if geometry is None:
                        raise ValueError('Collision has no geometry')
                    if geometry.find('plane') is not None:
                        normal = list(map(float, geometry.findtext('plane/normal', '0 0 1').split()))
                        if len(normal) != 3 or abs(normal[0])+abs(normal[1]) > 1e-6:
                            raise ValueError('Non-horizontal static plane needs a finite collision map')
                        if abs(tf[2]-altitude) < 0.01:
                            raise ValueError('Planning altitude intersects a static plane')
                        continue
                    box = geometry.find('box')
                    if box is not None:
                        sx, sy, sz = map(float, box.findtext('size').split())
                        if not all(math.isfinite(v) and v > 0 for v in (sx, sy, sz)):
                            raise ValueError('Non-positive collision dimensions')
                        points = [(-sx/2,-sy/2),(sx/2,-sy/2),(sx/2,sy/2),(-sx/2,sy/2)]
                    elif geometry.find('cylinder') is not None or geometry.find('sphere') is not None:
                        shape = geometry.find('cylinder')
                        if shape is None:
                            shape = geometry.find('sphere')
                        radius = float(shape.findtext('radius'))
                        if not math.isfinite(radius) or radius <= 0:
                            raise ValueError('Non-positive collision radius')
                        sz = float(shape.findtext('length', str(radius*2)))
                        # Circumscribed polygon: never under-estimate the circle.
                        r = radius/math.cos(math.pi/32)
                        points = [(r*math.cos(i*math.tau/32),r*math.sin(i*math.tau/32)) for i in range(32)]
                    else:
                        raise ValueError(f'Unsupported static geometry: {ET.tostring(geometry, encoding="unicode")[:160]}')
                    if not math.isfinite(sz) or sz <= 0 or any(not all(math.isfinite(v) for v in p) for p in points):
                        raise ValueError('Non-positive collision dimensions')
                    if abs(tf[2]-altitude) > sz/2:
                        continue
                    vertices = []
                    for x, y in points:
                        dx, dy = rotate(x, y, tf[3])
                        vertices.append(frame.to_map(tf[0]+dx, tf[1]+dy))
                    result.append(Footprint(vertices))

    root = ET.parse(world_file).getroot()
    world = root.find('world')
    if world is None:
        raise ValueError('Expected SDF world')
    walk(world, (0, 0, 0, 0), False, world_file.parent, (world_file,))
    return result
