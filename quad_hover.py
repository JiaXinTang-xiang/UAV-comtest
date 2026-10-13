import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyJointEffort
import time

class DroneHover(Node):
    def __init__(self):
        super().__init__('drone_hover')
        self.client = self.create_client(ApplyJointEffort, '/gazebo/apply_joint_effort')
        self.timer = self.create_timer(0.02, self.timer_cb)

    def timer_cb(self):
        if not self.client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn("等待Gazebo服务...")
            return
        
        # 关节名和转向完全匹配你的SDF
        joint_list = [
            ("iris::rotor0_joint", 0.035),
            ("iris::rotor1_joint", -0.035),
            ("iris::rotor2_joint", 0.035),
            ("iris::rotor3_joint", -0.035),
        ]
        for j_name, effort in joint_list:
            req = ApplyJointEffort.Request()
            req.joint_name = j_name
            req.effort = effort
            req.start_time.sec = 0
            req.start_time.nanosec = 0
            req.duration.sec = 0
            req.duration.nanosec = 0
            self.client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = DroneHover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

