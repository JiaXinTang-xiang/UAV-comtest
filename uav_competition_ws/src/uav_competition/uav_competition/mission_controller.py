#!/usr/bin/env python3
import math, time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

class MissionController(Node):
    """Route-aware mission state machine shared by mock and PX4 backends."""
    def __init__(self):
        super().__init__('mission_controller')
        self.declare_parameter('config_file', '')
        self.declare_parameter('route', '')
        cfg = self._load(self.get_parameter('config_file').value)
        m = cfg.get('mission', {})
        self.alt = float(m.get('cruise_altitude', 2.5)); self.alt_tol = float(m.get('altitude_tolerance', 0.1)); self.pos_tol = float(m.get('position_tolerance', 0.35)); self.timeout = float(m.get('max_mission_seconds', 180.0)); self.frame = m.get('frame_id', 'map')
        self.takeoff = tuple(m.get('takeoff', [5.0, 9.0])); self.areas = {k: tuple(v) for k, v in m.get('areas', {}).items()}; self.target = tuple(m.get('landing_target', [3.0, 1.0]))
        route_name = self.get_parameter('route').value or m.get('selected_route', 'right'); self.route_name = route_name; self.route = [tuple(p) for p in m.get('route', {}).get(route_name, [])]
        if not self.route or self.route[0] != self.takeoff: self.route.insert(0, self.takeoff)
        if self.route[-1] != self.target: self.route.append(self.target)
        self.route_i = 0; self.state = 'INIT'; self.mission_t = None; self.pose = None; self.preflight = False; self.violation = False
        self.target_pub = self.create_publisher(PoseStamped, '/uav/command/position', 10); self.state_pub = self.create_publisher(String, '/uav/mission/state', 10); self.event_pub = self.create_publisher(String, '/uav/mission/event', 10)
        self.create_subscription(PoseStamped, '/uav/pose', lambda m: setattr(self, 'pose', m.pose), 10); self.create_subscription(Bool, '/uav/preflight/ready', lambda m: setattr(self, 'preflight', m.data), 10); self.create_subscription(Bool, '/uav/safety/violation', lambda m: setattr(self, 'violation', m.data), 10)
        self.create_timer(1.0 / float(m.get('control_rate_hz', 20.0)), self.tick); self.get_logger().info(f'route={self.route_name}, {len(self.route)} waypoints')
    def _load(self, path): return yaml.safe_load(Path(path).read_text()) if path and Path(path).exists() else {}
    def set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'mission {self.state} -> {new_state}'); self.state = new_state; event = String(); event.data = new_state; self.event_pub.publish(event)
    def goal(self, x, y, z):
        msg = PoseStamped(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = self.frame; msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z); msg.pose.orientation.w = 1.0; self.target_pub.publish(msg)
    def reached(self, point, z=None):
        if self.pose is None: return False
        d = math.hypot(self.pose.position.x - point[0], self.pose.position.y - point[1]); dz = 0.0 if z is None else abs(self.pose.position.z - z); return d <= self.pos_tol and dz <= self.alt_tol
    def tick(self):
        now = time.monotonic()
        if self.violation and self.state not in ('FAILSAFE', 'COMPLETE'): self.set_state('FAILSAFE')
        if self.mission_t and now - self.mission_t > self.timeout and self.state not in ('COMPLETE', 'FAILSAFE'): self.set_state('FAILSAFE')
        if self.state == 'INIT': self.set_state('PREFLIGHT')
        elif self.state == 'PREFLIGHT':
            self.goal(*self.takeoff, self.alt)
            if self.preflight: self.mission_t = now; self.set_state('ARM')
        elif self.state == 'ARM': self.goal(*self.takeoff, self.alt); self.set_state('TAKEOFF')
        elif self.state == 'TAKEOFF':
            self.goal(*self.takeoff, self.alt)
            if self.reached(self.takeoff, self.alt): self.route_i = 1; self.set_state('HOVER')
        elif self.state == 'HOVER': self.goal(*self.takeoff, self.alt); self.set_state('AREA_2')
        elif self.state in ('AREA_2', 'AREA_3', 'AREA_4', 'AREA_5'):
            area_num = int(self.state.split('_')[1]); target_area = self.areas.get(f'area_{area_num}', self.takeoff)
            if self.route_i < len(self.route):
                waypoint = self.route[self.route_i]; self.goal(*waypoint, self.alt)
                if self.reached(waypoint, self.alt): self.route_i += 1
            else: self.goal(*target_area, self.alt)
            if self.reached(target_area, self.alt): self.set_state('LAND_APPROACH' if area_num == 5 else f'AREA_{area_num + 1}')
        elif self.state == 'LAND_APPROACH':
            self.goal(*self.target, self.alt)
            if self.reached(self.target, self.alt): self.set_state('LAND')
        elif self.state == 'LAND':
            self.goal(*self.target, 0.15)
            if self.reached(self.target, 0.15): self.set_state('COMPLETE')
        elif self.state == 'FAILSAFE' and self.pose: self.goal(self.pose.position.x, self.pose.position.y, max(0.15, self.pose.position.z))
        msg = String(); msg.data = self.state; self.state_pub.publish(msg)
def main(args=None):
    rclpy.init(args=args); node = MissionController(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
