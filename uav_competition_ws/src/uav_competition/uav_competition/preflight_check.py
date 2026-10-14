#!/usr/bin/env python3
import time
from typing import Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, LaserScan, BatteryState
from std_msgs.msg import Bool, String

class PreflightCheck(Node):
    def __init__(self):
        super().__init__('preflight_check')
        self.timeout = float(self.declare_parameter('sensor_timeout', 0.5).value)
        self.require_flow = bool(self.declare_parameter('require_optical_flow', True).value)
        self.last = {k: None for k in ('pose','velocity','imu','lidar','flow','battery')}
        self.ok_pub = self.create_publisher(Bool, '/uav/preflight/ready', 10)
        self.report_pub = self.create_publisher(String, '/uav/preflight/report', 10)
        self.create_subscription(PoseStamped, '/uav/pose', lambda m: self.mark('pose'), 10)
        self.create_subscription(TwistStamped, '/uav/velocity', lambda m: self.mark('velocity'), 10)
        self.create_subscription(Imu, '/uav/sensors/imu', lambda m: self.mark('imu'), 10)
        self.create_subscription(LaserScan, '/uav/sensors/lidar', lambda m: self.mark('lidar'), 10)
        self.create_subscription(TwistStamped, '/uav/sensors/optical_flow', lambda m: self.mark('flow'), 10)
        self.create_subscription(BatteryState, '/uav/battery', lambda m: self.mark('battery'), 10)
        self.timer = self.create_timer(0.2, self.check)
    def mark(self, key): self.last[key] = time.monotonic()
    def check(self):
        required = ['pose','velocity','imu','lidar','battery'] + (['flow'] if self.require_flow else [])
        now = time.monotonic()
        missing = [k for k in required if self.last[k] is None or now-self.last[k] > self.timeout]
        msg = Bool(); msg.data = not missing; self.ok_pub.publish(msg)
        report = String(); report.data = 'READY' if not missing else 'WAITING:' + ','.join(missing); self.report_pub.publish(report)
        if missing: self.get_logger().info(report.data, throttle_duration_sec=2.0)
        else: self.get_logger().info('preflight sensors ready', throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args); node=PreflightCheck(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
