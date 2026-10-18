"""Real sensor transport only: no velocity-derived flow or fixed battery."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import SensorOpticalFlow, DistanceSensor
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


class Px4SensorAdapter(Node):
    def __init__(self):
        super().__init__('px4_sensor_adapter')
        self.test_enabled = bool(self.declare_parameter('enable_tests', False).value)
        self.drop = False
        self.create_service(SetBool, '/uav/test/sensor_dropout', self.dropout_cb)
        self.flow = self.create_publisher(SensorOpticalFlow, '/uav/sensors/optical_flow_raw', 10)
        self.range = self.create_publisher(DistanceSensor, '/uav/sensors/downward_range', 10)
        self.scan = self.create_publisher(LaserScan, '/uav/sensors/lidar', 10)
        self.create_subscription(SensorOpticalFlow, '/fmu/out/sensor_optical_flow', lambda m: self.forward(self.flow, m), qos_profile_sensor_data)
        self.create_subscription(DistanceSensor, '/fmu/out/distance_sensor', lambda m: self.forward(self.range, m), qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/uav/lidar/scan', lambda m: self.forward(self.scan, m), qos_profile_sensor_data)

    def forward(self, publisher, msg):
        if not self.drop:
            publisher.publish(msg)

    def dropout_cb(self, req, res):
        res.success = self.test_enabled
        if self.test_enabled:
            self.drop = req.data
            res.message = f'TEST: sensor forwarding paused={self.drop}; raw Gazebo/PX4 sources untouched'
            self.get_logger().warn(res.message)
        else:
            res.message = 'Requires enable_tests:=true'
        return res


def main(args=None):
    rclpy.init(args=args)
    n = Px4SensorAdapter()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
