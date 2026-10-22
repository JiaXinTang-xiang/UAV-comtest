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
import json
import yaml
from pathlib import Path as FilePath


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['add', 'remove'])
    parser.add_argument('--ahead', type=float, default=1.8)
    parser.add_argument('--config', type=FilePath, default=FilePath(__file__).resolve().parents[1]/
                        'uav_competition_ws/src/uav_competition/config/mission.yaml')
    parser.add_argument('--interfaces', type=FilePath)
    parser.add_argument('--record', type=FilePath,
                        default=FilePath(__file__).resolve().parents[1]/'logs/obstacle_injection.json')
    args = parser.parse_args()
    from uav_competition.frames import MapFrame
    frames = MapFrame((yaml.safe_load(args.config.read_text()) or {}).get('frames'))
    remappings = (yaml.safe_load(args.interfaces.read_text()) or {}).get('remappings', {}) if args.interfaces else {}
    rclpy.init(args=['--ros-args', *[a for src,dst in remappings.items() for a in ('-r', f'{src}:={dst}')]])
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
            points = [(p.pose.position.x, p.pose.position.y) for p in path]
            # Curved paths contain short samples. Walk their arc length instead
            # of treating path[1] as a long, straight segment.
            from uav_competition.trajectory import project
            _, nearest = project((pos.x,pos.y),points[0],points[1])
            points = [nearest,*points[1:]]
            length = sum(math.dist(a,b) for a,b in zip(points,points[1:]))
            if not 1.5 <= args.ahead <= 2.0 or length < args.ahead+.6:
                raise RuntimeError('Segment too short. Use start of AREA_4; need at least ahead+0.6 m remaining.')
            remaining = args.ahead
            for a,b in zip(points,points[1:]):
                segment = math.dist(a,b)
                if segment >= remaining:
                    x,y = [a[i]+(b[i]-a[i])*remaining/segment for i in range(2)]
                    break
                remaining -= segment
            if math.hypot(x-pos.x,y-pos.y) < 1.4:
                raise RuntimeError('Curve folds back too close to the aircraft; wait for a clearer section')
            client = node.create_client(SpawnEntity, '/spawn_entity')
            request = SpawnEntity.Request()
            request.name = 'training_dynamic_obstacle'
            request.xml = '''<sdf version="1.6"><model name="training_dynamic_obstacle">
<static>true</static><link name="box"><collision name="collision"><geometry>
<box><size>0.4 0.4 3</size></box></geometry></collision><visual name="visual">
<geometry><box><size>0.4 0.4 3</size></box></geometry><material>
<ambient>1 0.4 0 1</ambient><diffuse>1 0.4 0 1</diffuse></material>
</visual></link></model></sdf>'''
            wx, wy = frames.to_world(x,y)
            wz = frames.origin[2]+1.5
            request.initial_pose.position.x = wx
            request.initial_pose.position.y = wy
            request.initial_pose.position.z = wz
            request.initial_pose.orientation.z = math.sin(frames.yaw/2)
            request.initial_pose.orientation.w = math.cos(frames.yaw/2)
            request.reference_frame = 'world'
            print(f'Insert real 0.4 x 0.4 x 3 m obstacle: map=({x:.3f}, {y:.3f}), world=({wx:.3f}, {wy:.3f}), ahead={args.ahead} m', flush=True)
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Gazebo factory service unavailable')
        requested_unix = time.time()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
        if not future.done():
            raise RuntimeError('Gazebo service timed out; inspect simulator before retry')
        result = future.result()
        print(f'success={result.success}, message={result.status_message}')
        if not result.success:
            raise RuntimeError(result.status_message)
        args.record.parent.mkdir(parents=True,exist_ok=True)
        record = {'action':args.action,'unix_time':requested_unix,'completed_unix':time.time(),'success':result.success,
                  'status_message':result.status_message}
        if args.action == 'add':
            record.update(map_position=[x,y],world_position=[wx,wy,wz],size=[.4,.4,3],
                          mission=latest['mission'][1].data,
                          aircraft_map_position=[pos.x,pos.y],ahead_m=args.ahead)
        output = args.record if args.action == 'add' else args.record.with_name(args.record.stem+'_removed.json')
        output.write_text(json.dumps(record,indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
