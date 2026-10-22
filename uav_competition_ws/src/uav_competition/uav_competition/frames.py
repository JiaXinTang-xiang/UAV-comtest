"""Explicit planar map/world and local-ENU transforms; angles are radians."""
import math


def rotate(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c*x-s*y, s*x+c*y


class MapFrame:
    def __init__(self, config=None):
        config = config or {}
        self.origin = tuple(config.get('map_origin_world', [0.0, 0.0, 0.0]))
        self.yaw = float(config.get('map_yaw_world', 0.0))
        self.local_yaw = float(config.get('px4_enu_yaw_world', 0.0)) - self.yaw
        if len(self.origin) != 3 or not all(math.isfinite(v) for v in (*self.origin, self.yaw, self.local_yaw)):
            raise ValueError('Invalid map/world frame configuration')

    def to_world(self, x, y):
        dx, dy = rotate(x, y, self.yaw)
        return self.origin[0]+dx, self.origin[1]+dy

    def to_map(self, x, y):
        return rotate(x-self.origin[0], y-self.origin[1], -self.yaw)

    def local_to_map(self, east, north):
        return rotate(east, north, self.local_yaw)

    def map_to_local(self, x, y):
        return rotate(x, y, -self.local_yaw)
