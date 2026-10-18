#!/usr/bin/env python3
"""Validate measured values, source timestamps, quality and sensor freshness.

Real PX4 mode consumes raw optical flow (radians), never a velocity proxy.
Mock mode must be explicitly selected by the mock launch.
"""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, LaserScan, BatteryState
from std_msgs.msg import Bool, String
from px4_msgs.msg import SensorOpticalFlow, DistanceSensor


def finite(*values):
    return all(math.isfinite(v) for v in values)


class PreflightCheck(Node):
    def __init__(self):
        super().__init__('preflight_check')
        self.timeout = float(self.declare_parameter('sensor_timeout', 1.0).value)
        self.mock = bool(self.declare_parameter('mock_mode', False).value)
        self.require_flow = bool(self.declare_parameter('require_optical_flow', True).value)
        self.quality = int(self.declare_parameter('minimum_flow_quality', 1).value)
        self.last = {k: None for k in ('pose', 'velocity', 'imu', 'lidar', 'flow', 'battery', 'range')}
        self.source_stamps = {}
        self.reasons = {}
        self.received = {}
        self.previous_ready = None
        self.last_log = 0.0
        self.ok_pub = self.create_publisher(Bool, '/uav/preflight/ready', 10)
        self.report_pub = self.create_publisher(String, '/uav/preflight/report', 10)
        self.create_subscription(PoseStamped, '/uav/pose', self.pose_cb, 10)
        self.create_subscription(TwistStamped, '/uav/velocity', self.velocity_cb, 10)
        self.create_subscription(Imu, '/uav/sensors/imu', self.imu_cb, 10)
        self.create_subscription(LaserScan, '/uav/sensors/lidar', self.scan_cb, 10)
        self.create_subscription(BatteryState, '/uav/battery', self.battery_cb, 10)
        if self.mock:
            self.create_subscription(TwistStamped, '/uav/sensors/optical_flow', lambda m: self.record('flow', m, True), 10)
        else:
            self.create_subscription(SensorOpticalFlow, '/uav/sensors/optical_flow_raw', self.flow_cb, 10)
            self.create_subscription(DistanceSensor, '/uav/sensors/downward_range', self.range_cb, 10)
        self.timer = self.create_timer(0.2, self.check)

    def record(self, key, msg, valid, reason='invalid_value'):
        self.received[key] = time.monotonic()
        if hasattr(msg, 'header'):
            stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        else:
            stamp = msg.timestamp
        if not valid or stamp <= 0:
            # A single bad frame is rejected; recent valid samples age out.
            self.reasons[key] = reason
            return
        # A repeated source timestamp must not keep an old sample alive.
        if self.source_stamps.get(key) == stamp:
            return
        self.source_stamps[key] = stamp
        self.last[key] = time.monotonic()
        self.reasons.pop(key, None)

    def pose_cb(self, m):
        p = m.pose.position
        self.record('pose', m, finite(p.x, p.y, p.z))

    def velocity_cb(self, m):
        p = m.twist.linear
        self.record('velocity', m, finite(p.x, p.y, p.z))

    def imu_cb(self, m):
        a, g = m.linear_acceleration, m.angular_velocity
        valid = finite(a.x, a.y, a.z, g.x, g.y, g.z) and 1.0 < math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z) < 40.0
        self.record('imu', m, valid)

    def scan_cb(self, m):
        valid = (len(m.ranges) >= 30 and finite(m.range_min, m.range_max, m.angle_increment) and
                 0.0 <= m.range_min < m.range_max and m.angle_increment > 0.0 and
                 any(math.isfinite(v) and m.range_min <= v <= m.range_max for v in m.ranges) and
                 not any(math.isnan(v) or v < 0 for v in m.ranges))
        self.record('lidar', m, valid,
                    f'beams={len(m.ranges)},limits={m.range_min:.3f}..{m.range_max:.3f},'
                    f'nan={sum(math.isnan(v) for v in m.ranges)},'
                    f'in_range={sum(math.isfinite(v) and m.range_min <= v <= m.range_max for v in m.ranges)}')

    def battery_cb(self, m):
        self.record('battery', m, m.present and finite(m.voltage, m.percentage) and m.voltage > 0 and 0.15 <= m.percentage <= 1.0,
                    'disconnected_low_or_unknown')

    def flow_cb(self, m):
        valid = (finite(*m.pixel_flow) and 0 < m.integration_timespan_us <= 500000 and
                 m.quality >= self.quality)
        self.record('flow', m, valid, f'quality={m.quality},integration_us={m.integration_timespan_us}')

    def range_cb(self, m):
        self.record('range', m, m.orientation == DistanceSensor.ROTATION_DOWNWARD_FACING and
                    finite(m.current_distance, m.min_distance, m.max_distance) and
                    m.min_distance - 0.011 <= m.current_distance <= m.max_distance + 0.011 and m.signal_quality != 0,
                    f'distance={m.current_distance:.3f},limits={m.min_distance:.3f}..{m.max_distance:.3f},'
                    f'orientation={m.orientation},quality={m.signal_quality}')

    def check(self):
        required = ['pose', 'velocity', 'imu', 'lidar', 'battery']
        if self.require_flow:
            required.append('flow')
        if not self.mock:
            required.append('range')
        now = time.monotonic()
        missing = [k for k in required if self.last[k] is None or now-self.last[k] > (2.5 if k == 'battery' else self.timeout)]
        msg = Bool()
        msg.data = not missing
        self.ok_pub.publish(msg)
        report = String()
        details = []
        for k in missing:
            valid_age = f'{now-self.last[k]:.3f}s' if self.last[k] is not None else 'never'
            received_age = f'{now-self.received[k]:.3f}s' if k in self.received else 'never'
            details.append(f'{k}:{self.reasons.get(k, "missing_or_stale")} '
                           f'(valid_age={valid_age},received_age={received_age})')
        report.data = ('MOCK_' if self.mock else '') + ('READY' if not missing else 'WAITING:' + ','.join(details))
        self.report_pub.publish(report)
        # A brief failure must be visible even between periodic READY messages.
        if msg.data != self.previous_ready or now-self.last_log >= 3.0:
            if msg.data:
                self.get_logger().info(report.data)
            else:
                self.get_logger().warn(report.data)
            self.last_log = now
        self.previous_ready = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = PreflightCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
