#!/usr/bin/env python3
"""Insert a real Gazebo collision box ahead of the current planned segment.

Run with the ROS2 and competition workspace sourced. This tool never edits
world files and never informs the planner directly; detection must use lidar.
"""
import argparse
import math
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity, DeleteEntity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['add', 'remove'])
    parser.add_argument('--ahead', type=float, default=1.8)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('training_obstacle_tool')
    try:
        if args.action == 'remove':
            client = node.create_client(DeleteEntity, '/delete_entity')
            request = DeleteEntity.Request()
            request.name = 'training_dynamic_obstacle'
        else:
            latest = {}
            for key, topic, msg_type in [
                ('pose', '/uav/pose', PoseStamped), ('path', '/uav/mission/path', Path),
                ('flight', '/uav/flight/state', String), ('mission', '/uav/mission/state', String),
            ]:
                node.create_subscription(msg_type, topic, lambda m, k=key: latest.update({k: (time.monotonic(), m)}), 10)
            deadline = time.monotonic()+5.0
            while time.monotonic() < deadline and len(latest) < 4:
                rclpy.spin_once(node, timeout_sec=.1)
            if len(latest) < 4 or any(time.monotonic()-t > .5 for t, _ in latest.values()):
                raise RuntimeError('No fresh flight/path telemetry. Start a region flight first.')
            if latest['flight'][1].data != 'ACTIVE' or not latest['mission'][1].data.startswith('AREA_'):
                raise RuntimeError('Requires ACTIVE region flight; insert at start of AREA_4 long segment.')
            pos = latest['pose'][1].pose.position
            path = latest['path'][1].poses
            if len(path) < 2:
                raise RuntimeError('Path has no next segment')
            goal = path[1].pose.position
            length = math.hypot(goal.x-pos.x, goal.y-pos.y)
            if not 1.5 <= args.ahead <= 2.0 or length < args.ahead+.6:
                raise RuntimeError('Segment too short. Use start of AREA_4; need at least ahead+0.6 m remaining.')
            x = pos.x+(goal.x-pos.x)*args.ahead/length
            y = pos.y+(goal.y-pos.y)*args.ahead/length
            client = node.create_client(SpawnEntity, '/spawn_entity')
            request = SpawnEntity.Request()
            request.name = 'training_dynamic_obstacle'
            request.xml = '''<sdf version="1.6"><model name="training_dynamic_obstacle">
<static>true</static><link name="box"><collision name="collision"><geometry>
<box><size>0.4 0.4 3</size></box></geometry></collision><visual name="visual">
<geometry><box><size>0.4 0.4 3</size></box></geometry><material>
<ambient>1 0.4 0 1</ambient><diffuse>1 0.4 0 1</diffuse></material>
</visual></link></model></sdf>'''
            request.initial_pose.position.x = x
            request.initial_pose.position.y = y
            request.initial_pose.position.z = 1.5
            request.initial_pose.orientation.w = 1.0
            request.reference_frame = 'world'
            print(f'Insert real 0.4 x 0.4 x 3 m obstacle at ({x:.3f}, {y:.3f}), ahead={args.ahead} m', flush=True)
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Gazebo factory service unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
        if not future.done():
            raise RuntimeError('Gazebo service timed out; inspect simulator before retry')
        result = future.result()
        print(f'success={result.success}, message={result.status_message}')
        if not result.success:
            raise RuntimeError(result.status_message)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
