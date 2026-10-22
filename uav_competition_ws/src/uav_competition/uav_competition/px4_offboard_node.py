#!/usr/bin/env python3
"""PX4 position adapter and an isolated SITL takeoff/hover/land check.

Map axes follow the Gazebo world (east, north, up). PX4 local axes are
north, east, down. A disarmed calibration ties local EKF coordinates to the
configured takeoff point. No fake flow, lidar or battery data is published.
PX4 retains its own position/attitude PID controllers; this node ramps
position references and supervises mode, arming, telemetry and landing.
"""
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    BatteryStatus, OffboardControlMode, SensorCombined, TrajectorySetpoint,
    VehicleCommand, VehicleCommandAck, VehicleLandDetected,
    VehicleLocalPosition, VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, String, Float32
from .frames import MapFrame
from .trajectory import limit_velocity
from .path_planner import CoursePlanner
from std_srvs.srv import Trigger


class Px4Offboard(Node):
    def __init__(self):
        super().__init__('px4_offboard')
        config = self.declare_parameter('config_file', '').value
        cfg = yaml.safe_load(Path(config).read_text()) if config else {}
        mission = cfg.get('mission', {})
        self.frame = MapFrame(cfg.get('frames'))
        self.acceleration = float(cfg.get('trajectory', {}).get('max_xy_acceleration', .35))
        self.reference_velocity = [0.0, 0.0]
        self.speed_request = None
        self.speed_request_t = 0.0
        safety = cfg.get('safety', {})
        self.takeoff = [float(v) for v in mission.get('takeoff', [5.0, 9.0])]
        self.altitude = float(mission.get('cruise_altitude', 2.5))
        self.tolerance = float(mission.get('altitude_tolerance', 0.1))
        self.bounds = safety.get('bounds', [0.25, 9.75, 0.25, 9.75])
        self.max_altitude = float(safety.get('max_altitude', 3.2))
        self.hover_test = bool(self.declare_parameter('hover_test', True).value)
        self.auto_start = bool(self.declare_parameter('auto_start', False).value)
        self.require_sensors = bool(self.declare_parameter('require_sensor_preflight', False).value)
        self.preflight_ready = False
        self.preflight_t = 0.0
        self.preflight_report = 'No report received'
        self.preflight_report_t = 0.0
        self.flight_timeout = float(mission.get('max_mission_seconds', 180.0))
        route = self.declare_parameter('route', 'right').value
        self.xy_speed = float(cfg.get('route_profiles', {}).get(route, {}).get('max_xy_speed', mission.get('max_xy_speed', 0.5)))
        world = self.declare_parameter('world_file', '').value
        self.reference_map = CoursePlanner(world,
            cfg['route_profiles'][route]['clearance'], cfg['planner']['resolution'], self.altitude,
            cfg.get('map', {}).get('bounds', [0,10,0,10]), cfg.get('frames'),
            [str((Path(config).parent/Path(p).expanduser()).resolve())
             for p in cfg.get('map', {}).get('model_paths', [])]) if world else None
        if not math.isfinite(self.acceleration) or self.acceleration <= 0:
            raise ValueError('max_xy_acceleration must be positive')
        self.test_enabled = bool(self.declare_parameter('enable_tests', False).value)
        self.test_geofence_offset = 0.0
        self.test_events = []
        self.hold_requested = False
        self.hold_t = 0.0
        self.last_source = {}
        self.hover_seconds = float(self.declare_parameter('hover_seconds', 5.0).value)
        self.climb_rate = float(self.declare_parameter('climb_rate', 0.5).value)
        self.command_timeout = float(self.declare_parameter('command_timeout', 0.5).value)
        if not (0.0 < self.altitude < self.max_altitude and self.climb_rate > 0.0):
            raise ValueError('Invalid cruise altitude or climb rate')
        log_dir = Path(self.declare_parameter('log_dir', str(Path.home() / 'uav_logs')).value)
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_base = log_dir / datetime.now().strftime('px4_hover_%Y%m%d_%H%M%S_%f')
        self.log_file = self.log_base.with_suffix('.csv').open('w', newline='')
        self.writer = csv.writer(self.log_file)
        self.writer.writerow(['elapsed_s', 'px4_timestamp_us', 'state', 'map_x', 'map_y',
                              'height_above_start_m', 'vx', 'vy', 'vz', 'nav_state',
                              'armed', 'target_height_m', 'gazebo_world_z_m'])
        self.created = time.monotonic()
        self.state = 'WAITING'
        self.state_since = self.created
        self.position = self.status = self.imu = None
        self.position_t = self.status_t = self.imu_t = 0.0
        self.origin = None
        self.samples = []
        self.reset_counters = None
        self.yaw = 0.0
        self.goal = None
        self.goal_t = 0.0
        self.desired = [*self.takeoff, 0.0]
        self.reference = self.desired.copy()
        self.last_tick = self.created
        self.last_command = 0.0
        self.stable_since = None
        self.start_time = None
        self.ever_armed = False
        self.failure = ''
        self.hover_errors = []
        self.truth = None
        self.truth_start = None
        self.truth_t = 0.0
        self.truth_hover_errors = []
        self.landed = None
        self.landed_t = 0.0
        self.landing_confirmed = False
        self.auto_requested = False
        self.result_written = False
        self.latest_ack = None
        self.position_pub = self.create_publisher(PoseStamped, '/uav/pose', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '/uav/velocity', 10)
        self.imu_pub = self.create_publisher(Imu, '/uav/sensors/imu', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/uav/battery', 10)
        self.state_pub = self.create_publisher(String, '/uav/flight/state', 10)
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        qos = qos_profile_sensor_data
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_cb, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos)
        self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.imu_cb, qos)
        # These additional outputs need dds_topics.yaml entries and a PX4 rebuild.
        self.create_subscription(BatteryStatus, '/fmu/out/battery_status', self.battery_cb, qos)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.ack_cb, qos)
        self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.landed_cb, qos)
        self.create_subscription(PoseStamped, '/uav/command/position', self.goal_cb, 10)
        self.create_subscription(Float32, '/uav/command/speed', self.speed_cb, 10)
        self.create_subscription(Bool, '/uav/safety/violation', self.safety_cb, 10)
        self.create_subscription(Bool, '/uav/command/hold', self.hold_cb, 10)
        self.create_service(Trigger, '/uav/test/geofence', self.geofence_test_cb)
        self.create_subscription(Bool, '/uav/preflight/ready', self.preflight_cb, 10)
        self.create_subscription(String, '/uav/preflight/report', self.preflight_report_cb, 10)
        self.create_subscription(Odometry, '/uav/ground_truth/odom', self.truth_cb, qos)
        self.create_service(Trigger, '/uav/flight/start', self.start_cb)
        self.create_service(Trigger, '/uav/flight/land', self.land_cb)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'Waiting for PX4; hover_test={self.hover_test}; log={self.log_base}.csv')

    def armed(self):
        return self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def hold_cb(self, msg):
        if msg.data and not self.hold_requested and self.position is not None and self.origin is not None:
            self.reference = self.map_position()
            self.desired = self.reference.copy()
            self.reference_velocity = [0.0, 0.0]
        self.hold_requested, self.hold_t = msg.data, time.monotonic()

    def speed_cb(self, msg):
        if math.isfinite(msg.data) and msg.data > 0:
            self.speed_request = min(self.xy_speed, float(msg.data))
            self.speed_request_t = time.monotonic()

    def geofence_test_cb(self, req, res):
        res.success = self.test_enabled and self.state == 'ACTIVE'
        if res.success:
            self.test_geofence_offset = 20.0
            self.test_events.append('geofence_monitor_x_offset=20m')
            self.get_logger().warn('TEST: geofence monitor x offset 20 m; actual position/setpoints unchanged')
        res.message = 'Geofence monitor fault injected' if res.success else 'Requires enable_tests:=true and ACTIVE flight'
        return res

    def preflight_cb(self, msg):
        self.preflight_ready, self.preflight_t = msg.data, time.monotonic()

    def preflight_report_cb(self, msg):
        changed = msg.data != self.preflight_report
        self.preflight_report = msg.data
        self.preflight_report_t = time.monotonic()
        if changed and 'WAITING:' in msg.data:
            self.get_logger().warn(f'Sensor report: {msg.data}')

    def truth_cb(self, msg):
        # Independent scoring only. Ground truth is never sent to PX4 control.
        self.truth = msg.pose.pose.position.z
        self.truth_t = time.monotonic()

    def advancing(self, name, msg):
        if msg.timestamp <= 0 or self.last_source.get(name) == msg.timestamp:
            return False
        self.last_source[name] = msg.timestamp
        return True

    def fresh(self, now):
        return (self.position is not None and self.status is not None and
                now - self.position_t < 0.5 and now - self.status_t < 1.5 and
                now - self.imu_t < 0.5 and self.position.xy_valid and
                self.position.z_valid and self.position.v_xy_valid and self.position.v_z_valid)

    def status_cb(self, msg):
        if self.advancing('status', msg):
            self.status, self.status_t = msg, time.monotonic()

    def position_cb(self, msg):
        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.heading)):
            return
        if not self.advancing('position', msg):
            return
        self.position, self.position_t = msg, time.monotonic()
        counters = (msg.xy_reset_counter, msg.z_reset_counter)
        if self.origin is not None and self.reset_counters is not None:
            if counters[0] != self.reset_counters[0]:
                self.origin[0] += msg.delta_xy[0]
                self.origin[1] += msg.delta_xy[1]
            if counters[1] != self.reset_counters[1]:
                self.origin[2] += msg.delta_z
        self.reset_counters = counters
        if self.origin is None:
            return
        x, y, z = self.map_position()
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        yaw_enu = math.pi / 2.0 - msg.heading + self.frame.local_yaw
        pose.pose.orientation.z = math.sin(yaw_enu / 2.0)
        pose.pose.orientation.w = math.cos(yaw_enu / 2.0)
        self.position_pub.publish(pose)
        vel = TwistStamped()
        vel.header = pose.header
        vx, vy = self.frame.local_to_map(msg.vy, msg.vx)
        vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z = float(vx), float(vy), float(-msg.vz)
        self.velocity_pub.publish(vel)

    def map_position(self):
        p = self.position
        # ENU east = NED east (y); ENU north = NED north (x).
        x, y = self.frame.local_to_map(p.y-self.origin[1], p.x-self.origin[0])
        return [self.takeoff[0]+x, self.takeoff[1]+y, self.origin[2]-p.z]

    def imu_cb(self, msg):
        values = [*msg.gyro_rad, *msg.accelerometer_m_s2]
        if (not all(math.isfinite(v) for v in values) or
                msg.accelerometer_timestamp_relative == SensorCombined.RELATIVE_TIMESTAMP_INVALID):
            return
        if not self.advancing('imu', msg):
            return
        self.imu, self.imu_t = msg, time.monotonic()
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'
        # Body FRD -> FLU; attitude is not supplied by SensorCombined.
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.x = float(msg.gyro_rad[0])
        imu.angular_velocity.y = float(-msg.gyro_rad[1])
        imu.angular_velocity.z = float(-msg.gyro_rad[2])
        imu.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        imu.linear_acceleration.y = float(-msg.accelerometer_m_s2[1])
        imu.linear_acceleration.z = float(-msg.accelerometer_m_s2[2])
        self.imu_pub.publish(imu)

    def battery_cb(self, msg):
        if not self.advancing('battery', msg):
            return
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.voltage = float(msg.voltage_v)
        battery.current = float(-msg.current_a) if msg.current_a >= 0.0 else float('nan')
        battery.percentage = float(msg.remaining) if 0.0 <= msg.remaining <= 1.0 else float('nan')
        battery.temperature = float(msg.temperature)
        battery.present = msg.connected
        self.battery_pub.publish(battery)

    def ack_cb(self, msg):
        self.latest_ack = {'command': msg.command, 'result': msg.result}
        self.get_logger().info(f'PX4 command={msg.command}, result={msg.result} (0=accepted)')

    def landed_cb(self, msg):
        self.landed, self.landed_t = msg.landed, time.monotonic()

    def goal_cb(self, msg):
        if self.hover_test:
            return
        p = msg.pose.position
        if msg.header.frame_id != 'map' or not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return
        self.goal = [p.x, p.y, p.z]
        self.goal_t = time.monotonic()

    def safety_cb(self, msg):
        if msg.data and self.state in ('PRIMING', 'SET_MODE', 'ARM', 'TAKEOFF', 'HOVER', 'ACTIVE'):
            self.abort('external_safety_violation')

    def transition(self, state):
        self.get_logger().info(f'flight {self.state} -> {state}')
        self.state = state
        self.state_since = time.monotonic()
        self.last_command = 0.0
        self.stable_since = None

    def start_cb(self, request, response):
        now = time.monotonic()
        if self.require_sensors and (not self.preflight_ready or now - self.preflight_t > 1.0):
            response.success, response.message = False, 'Sensor preflight not ready; see /uav/preflight/report'
            return response
        if self.state != 'READY' or not self.fresh(now) or self.armed() or not self.status.pre_flight_checks_pass:
            response.success = False
            response.message = f'Not ready: state={self.state}; require fresh telemetry, disarmed, PX4 preflight pass'
            return response
        if not self.hover_test and (self.goal is None or now - self.goal_t > self.command_timeout):
            response.success = False
            response.message = 'No fresh map-frame mission target'
            return response
        # Refresh the on-ground EKF origin immediately before arming.
        self.origin = [self.position.x, self.position.y, self.position.z]
        self.truth_start = self.truth if now-self.truth_t < .5 else None
        self.start_time = now
        self.yaw = float(self.position.heading)
        self.reference = self.map_position()
        self.desired = [*self.takeoff, self.altitude]
        self.transition('PRIMING')
        response.success, response.message = True, 'Starting PX4 Offboard flight'
        return response

    def land_cb(self, request, response):
        if self.armed():
            self.transition('LANDING')
            response.success, response.message = True, 'PX4 landing requested'
        else:
            response.success, response.message = False, 'Vehicle is disarmed'
        return response

    def command(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.target_system = self.status.system_id if self.status else 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 191
        msg.from_external = True
        for key, value in params.items():
            setattr(msg, key, float(value))
        self.command_pub.publish(msg)
        self.last_command = time.monotonic()

    def abort(self, reason):
        if self.state in ('COMPLETE', 'FAILED', 'LANDING'):
            return
        self.failure = reason
        self.get_logger().error(f'Protection: {reason}')
        self.transition('LANDING' if self.armed() else 'FAILED')

    def stream_setpoint(self, dt):
        # Freeze the on-ground reference until PX4 confirms Offboard AND armed.
        if self.state in ('TAKEOFF', 'HOVER', 'ACTIVE'):
            delta = [b - a for a, b in zip(self.reference, self.desired)]
            distance = math.hypot(delta[0], delta[1])
            maximum = self.xy_speed
            if self.speed_request is not None and time.monotonic()-self.speed_request_t < .5:
                maximum = min(maximum, self.speed_request)
            speed = min(maximum, math.sqrt(2*self.acceleration*distance))
            desired_velocity = [delta[i]*speed/distance if distance else 0.0 for i in range(2)]
            self.reference_velocity = limit_velocity(self.reference_velocity, desired_velocity, self.acceleration, dt)
            next_xy = [self.reference[i]+self.reference_velocity[i]*dt for i in range(2)]
            if self.reference_map and not self.reference_map.visible(self.reference[:2], next_xy):
                self.abort('reference_path_collision')
                return
            self.reference[:2] = next_xy
            self.reference[2] += max(-self.climb_rate * dt, min(self.climb_rate * dt, delta[2]))
        stamp = self.get_clock().now().nanoseconds // 1000
        mode = OffboardControlMode()
        mode.timestamp, mode.position = stamp, True
        self.offboard_pub.publish(mode)
        target = TrajectorySetpoint()
        target.timestamp = stamp
        east, north = self.frame.map_to_local(self.reference[0]-self.takeoff[0], self.reference[1]-self.takeoff[1])
        target.position = [float(self.origin[0] + north),
                           float(self.origin[1] + east),
                           float(self.origin[2] - self.reference[2])]
        # NaN disables unused feed-forward terms; default zeros constrain them.
        target.velocity = [float('nan')] * 3
        target.acceleration = [float('nan')] * 3
        target.jerk = [float('nan')] * 3
        target.yaw = self.yaw
        target.yawspeed = float('nan')
        self.setpoint_pub.publish(target)

    def write_result(self):
        if self.result_written:
            return
        errors = self.hover_errors
        result = {'state': self.state, 'failure': self.failure,
                  'hover_test': self.hover_test, 'cruise_height_above_start_m': self.altitude,
                  'hover_samples': len(errors),
                  'hover_max_abs_error_m': max(errors) if errors else None,
                  'hover_mean_abs_error_m': sum(errors) / len(errors) if errors else None,
                  'hover_within_tolerance': (max(errors) <= self.tolerance) if errors else None,
                  'gazebo_height_reference': 'rise above base_link at start',
                  'gazebo_start_z_m': self.truth_start,
                  'gazebo_hover_samples': len(self.truth_hover_errors),
                  'gazebo_hover_max_abs_error_m': max(self.truth_hover_errors) if self.truth_hover_errors else None,
                  'gazebo_hover_within_tolerance': (max(self.truth_hover_errors) <= self.tolerance) if self.truth_hover_errors else None,
                  'final_map_position': self.map_position() if self.origin is not None else None,
                  'armed': self.armed(), 'latest_command_ack': self.latest_ack,
                  'scope': 'PX4 telemetry; independent Gazebo height when available; sensor preflight required=' + str(self.require_sensors)}
        result['test_mode'] = self.test_enabled
        result['test_events'] = self.test_events
        result['sensor_preflight_failure'] = getattr(self, 'preflight_failure', None)
        self.log_base.with_suffix('.json').write_text(json.dumps(result, indent=2))
        self.log_file.flush()
        self.result_written = True
        self.get_logger().info(f'Result: {self.log_base}.json')

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = min(0.2, now - self.last_tick), now
        elapsed = now - self.state_since
        if self.state == 'WAITING' and self.fresh(now) and not self.armed():
            # Calibrate only on the ground while stationary for two seconds.
            p = self.position
            if math.sqrt(p.vx ** 2 + p.vy ** 2 + p.vz ** 2) < 0.15:
                self.samples.append((now, p.x, p.y, p.z))
                if now - self.samples[0][0] >= 2.0:
                    self.origin = [sum(s[i] for s in self.samples) / len(self.samples) for i in (1, 2, 3)]
                    self.transition('READY')
            else:
                self.samples.clear()
        elif self.state == 'WAITING':
            self.samples.clear()
        if self.state == 'READY' and self.auto_start and not self.auto_requested:
            self.auto_requested = True
            self.start_cb(Trigger.Request(), Trigger.Response())
        active = self.state in ('PRIMING', 'SET_MODE', 'ARM', 'TAKEOFF', 'HOVER', 'ACTIVE')
        if active:
            if not self.fresh(now):
                self.abort('stale_or_invalid_telemetry')
            elif self.require_sensors and (not self.preflight_ready or now - self.preflight_t > 1.0):
                self.preflight_failure = {
                    'ready': self.preflight_ready,
                    'heartbeat_age_s': now-self.preflight_t,
                    'latest_report': self.preflight_report,
                    'report_age_s': now-self.preflight_report_t,
                }
                self.get_logger().error(f'Sensor guard detail: {self.preflight_failure}')
                self.abort('sensor_preflight_lost')
            elif self.start_time is not None and now - self.start_time > self.flight_timeout:
                self.abort('flight_timeout')
            elif self.status.failsafe:
                self.abort('px4_failsafe')
            elif self.origin is not None:
                x, y, z = self.map_position()
                x += self.test_geofence_offset
                if not (self.bounds[0] <= x <= self.bounds[1] and self.bounds[2] <= y <= self.bounds[3]):
                    self.abort('geofence')
                elif z > self.max_altitude:
                    self.abort('altitude_limit')
        if self.state == 'PRIMING' and elapsed >= 1.5:
            self.transition('SET_MODE')
        elif self.state == 'SET_MODE':
            if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.transition('ARM')
            elif elapsed > 12.0:
                self.abort('offboard_mode_timeout')
            elif now - self.last_command >= 1.0:
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1, param2=6)
        elif self.state == 'ARM':
            if self.armed():
                self.ever_armed = True
                self.transition('TAKEOFF')
            elif elapsed > 12.0:
                self.abort('arming_timeout')
            elif self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.abort('offboard_lost_before_arming')
            elif now - self.last_command >= 1.0:
                self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1)
        elif self.state in ('TAKEOFF', 'HOVER', 'ACTIVE'):
            if not self.armed():
                self.failure = 'unexpected_disarm'
                self.transition('FAILED')
            elif self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.abort('offboard_mode_lost')
            elif self.state == 'TAKEOFF':
                x, y, z = self.map_position()
                if abs(z - self.altitude) <= self.tolerance and math.hypot(x - self.takeoff[0], y - self.takeoff[1]) <= 0.35:
                    if self.stable_since is None:
                        self.stable_since = now
                    elif now - self.stable_since >= 1.0:
                        self.transition('HOVER' if self.hover_test else 'ACTIVE')
                else:
                    self.stable_since = None
                if elapsed > 35.0:
                    self.abort('takeoff_timeout')
            elif self.state == 'HOVER':
                self.hover_errors.append(abs(self.map_position()[2] - self.altitude))
                if self.truth is not None and now - self.truth_t < 0.5:
                    self.truth_hover_errors.append(abs(self.truth - (self.truth_start or 0.0) - self.altitude))
                if elapsed >= self.hover_seconds:
                    if max(self.hover_errors) > self.tolerance:
                        self.failure = 'hover_outside_altitude_tolerance'
                    self.transition('LANDING')
            else:
                if now - self.goal_t > self.command_timeout:
                    self.abort('stale_mission_command')
                elif self.goal[2] <= 0.15:
                    self.transition('LANDING')
                elif self.hold_requested and now-self.hold_t < 0.5:
                    self.desired = self.reference.copy()
                else:
                    self.desired = self.goal.copy()
        elif self.state == 'LANDING':
            if self.status is not None and now - self.status_t < 1.5:
                if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                    self.landing_confirmed = True
                if not self.armed():
                    ground = (self.origin is not None and now - self.position_t < 0.5 and
                              self.map_position()[2] < 0.3 and abs(self.position.vz) < 0.2)
                    if self.landed is not None and now - self.landed_t < 1.0:
                        ground = self.landed
                    passed = self.ever_armed and self.landing_confirmed and ground and not self.failure
                    self.transition('COMPLETE' if passed else 'FAILED')
                elif (not self.landing_confirmed or self.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND) and now - self.last_command >= 1.0:
                    # PX4 handles descent, touchdown detection and automatic disarm.
                    self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND, param4=float('nan'),
                                 param5=float('nan'), param6=float('nan'), param7=float('nan'))
            if elapsed > 45.0 and self.state == 'LANDING':
                self.failure = self.failure or 'landing_timeout'
                self.get_logger().error('Landing timeout: continuing LAND requests; vehicle not confirmed disarmed', throttle_duration_sec=5.0)
        if self.state in ('PRIMING', 'SET_MODE', 'ARM', 'TAKEOFF', 'HOVER', 'ACTIVE'):
            self.stream_setpoint(dt)
        if self.state in ('COMPLETE', 'FAILED'):
            self.write_result()
        state = String()
        state.data = self.state
        self.state_pub.publish(state)
        if self.origin is not None:
            p = self.position
            self.writer.writerow([round(now - self.created, 3), p.timestamp, self.state,
                                  *self.map_position(), p.vy, p.vx, -p.vz,
                                  self.status.nav_state, self.armed(), self.reference[2],
                                  self.truth if now - self.truth_t < 0.5 else ''])
            if int((now - self.created) * 20) % 20 == 0:
                self.log_file.flush()

    def destroy_node(self):
        self.log_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Px4Offboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Loss of Offboard heartbeat invokes PX4's own configured failsafe.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
