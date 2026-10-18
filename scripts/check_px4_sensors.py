#!/usr/bin/env python3
"""Record real sensor evidence; run with the sourced ROS2 and px4_msgs overlays.
Usage: check_px4_sensors.py OUTPUT.json SECONDS
"""
import rclpy,time,json,math,sys
from rclpy.qos import qos_profile_sensor_data as Q
from px4_msgs.msg import SensorOpticalFlow,DistanceSensor,BatteryStatus
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool,String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
rclpy.init();n=rclpy.create_node('sensor_evidence');start=time.monotonic();data={};records=[];state='';phase='';latest={}
def cb(k,m):
 global state,phase
 v=data.setdefault(k,{'samples':0,'valid':0,'first':None,'last':None});v['samples']+=1
 if k=='flow':a={'quality':m.quality,'integration_us':m.integration_timespan_us,'flow':[float(x) for x in m.pixel_flow]};good=m.quality>0 and m.integration_timespan_us>0
 elif k=='range':a={'distance':m.current_distance,'quality':m.signal_quality};good=m.signal_quality>0 and m.min_distance<=m.current_distance<=m.max_distance
 elif k=='battery':a={'voltage':m.voltage_v,'remaining':m.remaining,'connected':m.connected};good=m.connected and 0<m.remaining<=1
 elif k=='lidar':a={'beams':len(m.ranges),'nearest':min(m.ranges)};good=len(m.ranges)==360 and not any(math.isnan(x) for x in m.ranges)
 elif k=='preflight':a=m.data;good=m.data
 elif k=='flight':state=m.data;a=m.data;good=True
 elif k=='mission':phase=m.data;a=m.data;good=True
 else:
  p=m.pose.pose.position if k=='truth' else m.pose.position;a=[p.x,p.y,p.z];good=True
 v['valid']+=int(good);v['first']=a if v['first'] is None else v['first'];v['last']=a;latest[k]=a
for k,t,T in [('flow','/fmu/out/sensor_optical_flow',SensorOpticalFlow),('range','/fmu/out/distance_sensor',DistanceSensor),('battery','/fmu/out/battery_status',BatteryStatus),('lidar','/uav/lidar/scan',LaserScan),('preflight','/uav/preflight/ready',Bool),('flight','/uav/flight/state',String),('mission','/uav/mission/state',String),('truth','/uav/ground_truth/odom',Odometry),('pose','/uav/pose',PoseStamped)]:n.create_subscription(T,t,lambda m,k=k:cb(k,m),Q)
last=0;active=False
while time.monotonic()-start<float(sys.argv[2]):
 rclpy.spin_once(n,timeout_sec=.05);now=time.monotonic()-start
 if now-last>.1:records.append({'t':now,'state':state,'mission':phase,**latest});last=now
 if state in ('TAKEOFF','ACTIVE','HOVER'):active=True
 if active and state in ('COMPLETE','FAILED') and (not phase or phase in ('COMPLETE','FAILED')):break
result={'duration_s':time.monotonic()-start,'topics':data,'records':records}
open(sys.argv[1],'w').write(json.dumps(result,indent=2));print(json.dumps(data,indent=2));n.destroy_node();rclpy.shutdown()
