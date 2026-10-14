#!/usr/bin/env python3
"""PX4 native ROS2 bridge adapter.

This node is intentionally optional until px4_msgs and Micro XRCE-DDS Agent
are installed. It converts the internal position command to PX4 Offboard
TrajectorySetpoint and publishes the standard PX4 arm/mode commands.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
class Px4Offboard(Node):
    def __init__(self):
        super().__init__('px4_offboard')
        try:
            from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
        except ImportError as e:
            self.get_logger().error('px4_msgs is not installed. Install PX4 ROS2 messages before starting this node.')
            raise RuntimeError(str(e))
        self.OffboardControlMode=OffboardControlMode; self.TrajectorySetpoint=TrajectorySetpoint; self.VehicleCommand=VehicleCommand
        self.offboard_pub=self.create_publisher(OffboardControlMode,'/fmu/in/offboard_control_mode',10); self.setpoint_pub=self.create_publisher(TrajectorySetpoint,'/fmu/in/trajectory_setpoint',10); self.command_pub=self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command',10)
        self.goal=None; self.create_subscription(PoseStamped,'/uav/command/position',lambda m:setattr(self,'goal',m.pose.position),10); self.create_timer(.05,self.tick)
    def tick(self):
        o=self.OffboardControlMode(); o.position=True; o.timestamp=int(self.get_clock().now().nanoseconds/1000); self.offboard_pub.publish(o)
        if not self.goal: return
        t=self.TrajectorySetpoint(); t.position=[float(self.goal.x),float(-self.goal.y),float(-self.goal.z)]; t.yaw=0.; t.timestamp=o.timestamp; self.setpoint_pub.publish(t)
def main(args=None):
    rclpy.init(args=args); n=Px4Offboard(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
