#!/usr/bin/env python3
"""Kinematic fallback for testing mission logic before PX4 is installed."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
class MockFlight(Node):
    def __init__(self):
        super().__init__('mock_flight'); self.p=[5.,9.,0.5]; self.v=[0.,0.,0.]; self.goal=None
        self.create_subscription(PoseStamped,'/uav/command/position',lambda m:setattr(self,'goal',(m.pose.position.x,m.pose.position.y,m.pose.position.z)),10)
        self.pub=self.create_publisher(Odometry,'/uav/p3d/odom',10); self.imu_pub=self.create_publisher(Imu,'/uav/imu/data',10); self.scan_pub=self.create_publisher(LaserScan,'/uav/lidar/scan',10); self.create_timer(.02,self.tick)
    def tick(self):
        if self.goal:
            dt=.02; k=1.5; old=self.p[:]
            for i in range(3):
                e=self.goal[i]-self.p[i]; step=max(-1.0*dt,min(1.0*dt,k*e*dt)); self.p[i]+=step; self.v[i]=step/dt
        m=Odometry(); m.header.stamp=self.get_clock().now().to_msg(); m.header.frame_id='map'; m.child_frame_id='base_link'; m.pose.pose.position.x,self_p_y,self_p_z=self.p; m.pose.pose.position.y=self_p_y; m.pose.pose.position.z=self_p_z; m.pose.pose.orientation.w=1.; m.twist.twist.linear.x,self_v_y,self_v_z=self.v; m.twist.twist.linear.y=self_v_y; m.twist.twist.linear.z=self_v_z; self.pub.publish(m)
        imu=Imu(); imu.header=m.header; imu.orientation.w=1.0; imu.linear_acceleration.z=9.81; self.imu_pub.publish(imu)
        scan=LaserScan(); scan.header=m.header; scan.angle_min=-3.14; scan.angle_max=3.14; scan.angle_increment=0.1; scan.range_min=0.05; scan.range_max=20.0; scan.ranges=[10.0]*63; self.scan_pub.publish(scan)
def main(args=None):
    rclpy.init(args=args); n=MockFlight(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
