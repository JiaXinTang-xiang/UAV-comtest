"""Region mission using real PX4 state, clearance planning and live lidar.

Start is explicit via /uav/mission/start. Ground truth is logged only.
"""
import json
import math
import time
from concurrent.futures import ThreadPoolExecutor
import copy
from pathlib import Path
from datetime import datetime
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from nav_msgs.msg import Path as RosPath
from .path_planner import CoursePlanner


class RegionMission(Node):
    def __init__(self):
        super().__init__('px4_region_mission')
        cfg = yaml.safe_load(Path(self.declare_parameter('config_file', '').value).read_text())
        m, p = cfg['mission'], cfg['planner']
        world = self.declare_parameter('world_file', '').value
        self.route = self.declare_parameter('route', 'right').value
        profile = cfg['route_profiles'][self.route]
        self.planner = CoursePlanner(world, profile['clearance'], p['resolution'], m['cruise_altitude'])
        self.executor_pool = ThreadPoolExecutor(max_workers=1)
        self.planning = None
        self.replans = []
        self.observations = []
        self.last_memory_update = 0.0
        self.hold = None
        self.replan_started = None
        self.plan_attempts = 0
        self.test_enabled = bool(self.declare_parameter('enable_tests', False).value)
        self.corridor_test = False
        self.test_events = []
        self.route_error = ''
        self.takeoff, self.landing = m['takeoff'], m['landing_target']
        self.targets = list(profile['targets'].items())
        self.targets += [('LAND_APPROACH', self.landing)]
        self.alt, self.alt_tol = m['cruise_altitude'], m['altitude_tolerance']
        self.tol, self.stop = p['waypoint_tolerance'], p['lidar_stop_distance']
        self.emergency_distance = float(p.get('emergency_landing_distance', 0.20))
        self.close_since = None
        # A side wall can be close while the planned corridor ahead is clear.
        # Emergency stopping therefore uses only a forward sector; the full
        # scan remains available to the dynamic planner in update_obstacles().
        self.emergency_half_angle = math.radians(float(p.get('emergency_sector_half_angle_deg', 45.0)))
        self.corridor = profile['corridor_limit']
        self.timeout = m['max_mission_seconds']
        self.state, self.flight = 'WAITING', ''
        self.pose = None
        self.pose_t = self.scan_t = self.flight_t = 0.0
        self.scan = None
        self.min_range = 0.0
        self.ahead_range = float('inf')
        self.ahead_angle = 0.0
        self.index, self.path_i = 0, 1
        self.path, self.visited = [], []
        self.started = self.blocked = self.stable = None
        self.request = None
        self.failure = ''
        self.result_written = False
        self.logdir = Path(self.declare_parameter('log_dir', '').value)
        self.logdir.mkdir(parents=True, exist_ok=True)
        self.log = self.logdir / datetime.now().strftime('region_%Y%m%d_%H%M%S.json')
        self.pub = self.create_publisher(PoseStamped, '/uav/command/position', 10)
        self.path_pub = self.create_publisher(RosPath, '/uav/mission/path', 10)
        self.hold_pub = self.create_publisher(Bool, '/uav/command/hold', 10)
        self.state_pub = self.create_publisher(String, '/uav/mission/state', 10)
        self.create_subscription(PoseStamped, '/uav/pose', self.pose_cb, 10)
        self.create_subscription(String, '/uav/flight/state', self.flight_cb, 10)
        self.create_subscription(LaserScan, '/uav/sensors/lidar', self.scan_cb, 10)
        self.start_client = self.create_client(Trigger, '/uav/flight/start')
        self.land_client = self.create_client(Trigger, '/uav/flight/land')
        self.create_service(Trigger, '/uav/mission/start', self.start_cb)
        self.create_service(Trigger, '/uav/test/corridor', self.corridor_test_cb)
        self.create_service(Trigger, '/uav/test/timeout', self.timeout_test_cb)
        self.create_timer(.05, self.tick)
        # Reject an impossible course before arming; preserve the approved map.
        previous = self.takeoff
        self.planned = []
        try:
            for name, target in self.targets:
                path = self.planner.plan(previous, target)
                self.planned.append({'region': name, 'path': path})
                previous = target
        except ValueError as exc:
            self.route_error = str(exc)
            self.get_logger().error(f'ROUTE_UNAVAILABLE route={self.route}: {exc}')
        if not self.route_error:
            self.get_logger().info(f'route={self.route}; ready for /uav/mission/start; all paths have clearance')
        if self.test_enabled:
            self.get_logger().warn('TEST MODE: fault injection services enabled')

    def corridor_test_cb(self, req, res):
        res.success = self.test_enabled and self.flight == 'ACTIVE' and self.planning is None
        if res.success:
            self.corridor_test = True
            self.test_events.append('monitor_corridor_offset=1.0m')
        res.message = 'TEST: offset corridor monitor by 1 m; real pose/control unchanged' if res.success else 'Requires enable_tests:=true and ACTIVE flight without planning'
        return res

    def timeout_test_cb(self, req, res):
        res.success = self.test_enabled and self.flight == 'ACTIVE'
        if res.success:
            self.timeout = time.monotonic()-self.started+2.0
            self.test_events.append('mission_deadline_in_2s')
        res.message = 'TEST: actual mission deadline shortened to 2 s from now' if res.success else 'Requires enable_tests:=true and ACTIVE flight'
        return res

    def pose_cb(self, msg):
        self.pose, self.pose_t = msg.pose, time.monotonic()

    def flight_cb(self, msg):
        self.flight, self.flight_t = msg.data, time.monotonic()

    def scan_cb(self, msg):
        self.scan, self.scan_t = msg, time.monotonic()
        ranges = [v for v in msg.ranges if math.isfinite(v) and msg.range_min <= v <= msg.range_max]
        self.min_range = min(ranges) if ranges else 0.0

    def forward_range(self, target):
        """Return the nearest valid return in the direction of the next goal.

        LaserScan angles are relative to scan_link, which is yaw-aligned with
        base_link in the training model.  Side returns are still consumed by
        the planner, but they must not trigger an emergency landing merely
        because the aircraft is flying through a narrow corridor.
        """
        if self.scan is None or self.pose is None:
            return float('inf'), 0.0
        q = self.pose.orientation
        yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                         1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        bearing = math.atan2(target[1] - self.pose.position.y,
                             target[0] - self.pose.position.x)
        desired = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
        candidates = []
        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r) or not self.scan.range_min <= r <= self.scan.range_max:
                continue
            a = self.scan.angle_min + i * self.scan.angle_increment
            da = math.atan2(math.sin(a - desired), math.cos(a - desired))
            if abs(da) <= self.emergency_half_angle:
                candidates.append((r, a))
        return min(candidates) if candidates else (float('inf'), desired)

    def transition(self, state):
        if self.state != state:
            self.get_logger().info(f'mission {self.state} -> {state}')
            self.state = state
            self.stable = None

    def start_cb(self, req, res):
        if self.route_error:
            res.success, res.message = False, f'Route unavailable: {self.route_error}'
            return res
        if self.state != 'WAITING' or self.flight != 'READY' or self.pose is None:
            res.success, res.message = False, f'Not ready: mission={self.state}, flight={self.flight}'
        elif not self.start_client.service_is_ready():
            res.success, res.message = False, 'Flight service unavailable'
        else:
            self.started = time.monotonic()
            self.transition('STARTING')
            res.success, res.message = True, 'Mission requested; waiting for flight self-check and arm'
        return res

    def goal(self, point, altitude=None):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y = map(float, point)
        msg.pose.position.z = float(self.alt if altitude is None else altitude)
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)

    def fail(self, reason):
        if self.state in ('FAILED', 'COMPLETE', 'ABORTING'):
            return
        self.hold_pub.publish(Bool(data=True))
        self.failure = reason
        self.get_logger().error(reason)
        self.transition('ABORTING')
        if self.land_client.service_is_ready():
            self.land_client.call_async(Trigger.Request())

    def publish_path(self):
        msg = RosPath()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path[self.path_i-1:]:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.position.z, pose.pose.orientation.w = float(self.alt), 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def update_obstacles(self, now):
        if now-self.last_memory_update < .3 or self.scan is None:
            return
        self.last_memory_update = now
        q, pos = self.pose.orientation, self.pose.position
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        memory = {(round(pt[0]/.10), round(pt[1]/.10)): (t, pt)
                  for t, pt in self.observations if now-t < 12.0}
        # Ignore returns inside the vehicle/sensor envelope.  These returns
        # are not usable obstacles and otherwise make a replan reject its
        # own current start point as lacking dynamic clearance.
        own_radius = 0.45
        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r) or not self.scan.range_min < r < 4.0:
                continue
            angle = yaw+self.scan.angle_min+i*self.scan.angle_increment
            pt = (pos.x+r*math.cos(angle), pos.y+r*math.sin(angle))
            if math.dist((pos.x, pos.y), pt) < own_radius:
                continue
            # Known walls are already represented by exact collision geometry.
            wall_distance = min(math.hypot(max(x0-pt[0], 0, pt[0]-x1),
                                          max(y0-pt[1], 0, pt[1]-y1))
                                for x0, x1, y0, y1 in self.planner.boxes)
            if wall_distance < .25:
                continue
            memory[(round(pt[0]/.10), round(pt[1]/.10))] = (now, pt)
        self.observations = list(memory.values())
        self.planner.dynamic = [pt for _, pt in self.observations]

    def request_plan(self, reason):
        if self.planning is not None:
            return
        if self.replan_started is None:
            self.replan_started = time.monotonic()
            self.plan_attempts = 0
        self.plan_attempts += 1
        if self.plan_attempts > 5 or time.monotonic()-self.replan_started > 8.0:
            self.fail('replanning_unstable_or_timeout')
            return
        pos = self.pose.position
        self.hold = (pos.x, pos.y)
        self.hold_pub.publish(Bool(data=True))
        planner = copy.deepcopy(self.planner)
        self.planning = self.executor_pool.submit(planner.plan, self.hold, self.targets[self.index][1])
        self.replans.append({'reason': reason, 'from': self.hold,
                             'region': self.targets[self.index][0], 'obstacle_points': len(planner.dynamic)})
        self.get_logger().info(f'PLAN_REQUEST {reason}; obstacle_points={len(planner.dynamic)}; holding')

    def begin_leg(self):
        self.transition(self.targets[self.index][0])
        self.request_plan('region_transition')

    def corridor_error(self, xy):
        a, b = self.path[self.path_i-1], self.path[self.path_i]
        dx, dy = b[0]-a[0], b[1]-a[1]
        u = max(0, min(1, ((xy[0]-a[0])*dx+(xy[1]-a[1])*dy)/(dx*dx+dy*dy))) if dx or dy else 0
        return math.dist(xy, (a[0]+u*dx, a[1]+u*dy))

    def tick(self):
        now = time.monotonic()
        self.state_pub.publish(String(data=self.state))
        if self.state == 'WAITING':
            self.goal(self.takeoff)
            return
        if self.state in ('COMPLETE', 'FAILED'):
            if not self.result_written:
                self.log.write_text(json.dumps({'state': self.state, 'failure': self.failure,
                    'route': self.route, 'test_mode': self.test_enabled, 'test_events': self.test_events,
                    'visited': self.visited, 'planned': self.planned, 'replans': self.replans,
                    'flight_state': self.flight, 'elapsed_s': now-self.started}, indent=2))
                self.result_written = True
            return
        if self.flight == 'FAILED':
            self.failure = self.failure or 'flight_failed'
            self.transition('FAILED')
            return
        if self.flight == 'COMPLETE':
            self.transition('COMPLETE' if self.state == 'LANDING' and not self.failure else 'FAILED')
            return
        if self.state == 'ABORTING':
            if self.flight == 'READY':
                self.transition('FAILED')
            return
        if now-self.started > self.timeout:
            self.fail('mission_timeout')
            return
        if self.pose is None or now-self.pose_t > .5 or now-self.flight_t > 1.0:
            self.fail('stale_pose_or_flight_state')
            return
        if self.state == 'STARTING':
            self.goal(self.takeoff)
            if self.request is None:
                self.request = self.start_client.call_async(Trigger.Request())
            elif self.request.done():
                response = self.request.result()
                if response.success:
                    self.transition('TAKEOFF')
                else:
                    self.fail(response.message)
            return
        if self.state == 'TAKEOFF':
            self.goal(self.takeoff)
            if self.flight == 'ACTIVE':
                self.begin_leg()
            return
        if self.state == 'LANDING':
            self.hold_pub.publish(Bool(data=False))
            self.goal(self.landing, .15)
            return
        if self.flight == 'LANDING':
            self.fail('unexpected_flight_landing')
            return
        if now-self.scan_t > .5:
            self.fail('lidar_timeout')
            return
        pos = self.pose.position
        xy = (pos.x, pos.y)
        self.update_obstacles(now)
        if self.planning is not None:
            self.hold_pub.publish(Bool(data=True))
            self.goal(self.hold)
            if self.planning.done():
                try:
                    candidate = self.planning.result()
                except Exception as exc:
                    self.replans[-1]['error'] = str(exc)
                    self.planning = None
                    # A transient scan can block a narrow route. Hold and
                    # retry a bounded number of times before landing; do not
                    # abort immediately on the first dynamic-map failure.
                    if self.plan_attempts < 5:
                        self.get_logger().warn(f'no_safe_path; retrying plan: {exc}')
                        self.request_plan('retry_after_no_safe_path')
                    else:
                        self.fail(f'no_safe_path: {exc}')
                    return
                self.planning = None
                if not all(self.planner.visible(a, b) for a, b in zip(candidate, candidate[1:])):
                    self.request_plan('new_observation_during_planning')
                    return
                self.replan_started = None
                self.path, self.path_i = candidate, 1
                self.stable = None
                self.replans[-1]['path'] = candidate
                self.get_logger().info(f'PLAN_READY waypoints={len(candidate)}')
                self.hold_pub.publish(Bool(data=False))
                self.publish_path()
            return
        self.hold_pub.publish(Bool(data=False))
        error = self.corridor_error(xy) + (1.0 if self.corridor_test else 0.0)
        if error > self.corridor:
            self.fail(f'outside_planned_corridor: error={error:.3f},limit={self.corridor:.3f}')
            return
        self.publish_path()
        # Check ahead before entering an obstacle's clearance envelope.
        target = self.path[self.path_i]
        d = math.dist(xy, target)
        f = min(1.0, 2.0/d) if d else 1.0
        lookahead = (pos.x+(target[0]-pos.x)*f, pos.y+(target[1]-pos.y)*f)
        if not self.planner.visible(xy, lookahead):
            self.request_plan('lidar_path_blocked')
            return
        target = self.path[self.path_i]
        self.ahead_range, self.ahead_angle = self.forward_range(target)
        if self.ahead_range < self.stop:
            # A single ray can hit a nearby side wall while the collision-box
            # planner still confirms that the commanded segment is clear.
            # Treat this as a warning. Dynamic returns are already inserted
            # into planner.dynamic by update_obstacles() and will request a
            # replan through the visibility check above.
            if self.close_since is None:
                self.close_since = now
            if self.ahead_range < self.emergency_distance and now - self.close_since >= 0.4:
                self.fail(f'obstacle_inside_emergency_distance: {self.ahead_range:.3f} '
                          f'angle={math.degrees(self.ahead_angle):.1f}deg')
                return
            self.get_logger().warn(
                f'near obstacle ignored while planned segment is clear: '
                f'{self.ahead_range:.3f}m angle={math.degrees(self.ahead_angle):.1f}deg',
                throttle_duration_sec=2.0)
        else:
            self.close_since = None
        self.goal(target)
        if math.dist(xy, target) <= self.tol and abs(pos.z-self.alt) <= self.alt_tol:
            if self.stable is None:
                self.stable = now
            if now-self.stable < .3:
                return
            if self.path_i < len(self.path)-1:
                self.path_i += 1
                self.stable = None
            else:
                self.visited.append({'region': self.targets[self.index][0], 'x': pos.x, 'y': pos.y, 'z': pos.z})
                self.index += 1
                if self.index == len(self.targets):
                    self.transition('LANDING')
                else:
                    self.begin_leg()
        else:
            self.stable = None


def main(args=None):
    rclpy.init(args=args)
    node = RegionMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.executor_pool.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
