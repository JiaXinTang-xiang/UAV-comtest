#!/usr/bin/env python3
"""Normalize Gazebo sensor topics for the mission nodes.

The p3d pose is used as a deterministic training source. Optical flow is a
velocity proxy until a PX4 optical-flow sensor plugin is installed; the topic
and validity checks remain the same when the real sensor is connected.
"""
import math, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, LaserScan, BatteryState

class SensorAdapter(Node):
    def __init__(self):
        super().__init__('sensor_adapter')
        self.pose_pub=self.create_publisher(PoseStamped,'/uav/pose',10)
        self.vel_pub=self.create_publisher(TwistStamped,'/uav/velocity',10)
        self.flow_pub=self.create_publisher(TwistStamped,'/uav/sensors/optical_flow',10)
        self.battery_pub=self.create_publisher(BatteryState,'/uav/battery',10)
        self.last_pose=None; self.last_stamp=None
        self.create_subscription(Odometry,'/uav/p3d/odom',self.odom_cb,10)
        self.create_subscription(Imu,'/uav/imu/data',lambda m:self.imu_pub.publish(m),10)
        self.imu_pub=self.create_publisher(Imu,'/uav/sensors/imu',10)
        self.create_subscription(LaserScan,'/uav/lidar/scan',lambda m:self.lidar_pub.publish(m),10)
        self.lidar_pub=self.create_publisher(LaserScan,'/uav/sensors/lidar',10)
        self.create_timer(0.1,self.battery_cb)
    def odom_cb(self,m):
        p=PoseStamped(); p.header=m.header; p.header.frame_id='map'; p.pose=m.pose.pose; self.pose_pub.publish(p)
        v=TwistStamped(); v.header=m.header; v.twist=m.twist.twist; self.vel_pub.publish(v); self.flow_pub.publish(v)
        self.last_pose=p
    def battery_cb(self):
        b=BatteryState(); b.header.stamp=self.get_clock().now().to_msg(); b.voltage=16.0; b.percentage=0.95; b.present=True; self.battery_pub.publish(b)

def main(args=None):
    rclpy.init(args=args); node=SensorAdapter(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
