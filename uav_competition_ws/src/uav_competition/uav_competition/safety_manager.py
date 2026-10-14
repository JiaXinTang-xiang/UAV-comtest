#!/usr/bin/env python3
import math, time, yaml
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
class SafetyManager(Node):
    def __init__(self):
        super().__init__('safety_manager')
        self.declare_parameter('config_file','')
        cfg=self._load(self.get_parameter('config_file').value)
        s=cfg.get('safety',{}); self.bounds=s.get('bounds',[.25,9.75,.25,9.75]); self.min_z=s.get('min_altitude',.15); self.max_z=s.get('max_altitude',3.2); self.stop=s.get('obstacle_stop_distance',.65); self.timeout=s.get('stale_sensor_seconds',.5)
        self.last_pose=None; self.last_pose_t=0; self.min_range=float('inf'); self.violation=False
        self.pub=self.create_publisher(Bool,'/uav/safety/violation',10); self.event_pub=self.create_publisher(String,'/uav/safety/event',10)
        self.create_subscription(PoseStamped,'/uav/pose',self.pose_cb,10); self.create_subscription(LaserScan,'/uav/sensors/lidar',self.scan_cb,10); self.create_timer(.1,self.check)
    def _load(self,p):
        if p and Path(p).exists(): return yaml.safe_load(Path(p).read_text()) or {}
        return {}
    def pose_cb(self,m): self.last_pose=m.pose; self.last_pose_t=time.monotonic()
    def scan_cb(self,m):
        vals=[x for x in m.ranges if math.isfinite(x) and x>m.range_min]; self.min_range=min(vals) if vals else float('inf')
    def check(self):
        reason=''; p=self.last_pose
        if p is None or time.monotonic()-self.last_pose_t>self.timeout: reason='stale_pose'
        elif not(self.bounds[0]<=p.position.x<=self.bounds[1] and self.bounds[2]<=p.position.y<=self.bounds[3]): reason='geofence'
        elif not(self.min_z<=p.position.z<=self.max_z): reason='altitude'
        elif self.min_range<self.stop: reason='obstacle'
        self.violation=bool(reason); b=Bool(); b.data=self.violation; self.pub.publish(b)
        if reason:
            e=String(); e.data='FAILSAFE:'+reason; self.event_pub.publish(e); self.get_logger().warn(e.data,throttle_duration_sec=1.0)
def main(args=None):
    rclpy.init(args=args); n=SafetyManager(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
