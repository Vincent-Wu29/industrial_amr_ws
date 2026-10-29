#!/usr/bin/env python3
"""Read-only ROS diagnostic snapshot for a running system."""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from amr_interfaces.msg import TaskSnapshot
from rosidl_runtime_py.convert import message_to_ordereddict

def main():
    p=argparse.ArgumentParser(); p.add_argument('--seconds',type=float,default=3); args=p.parse_args()
    rclpy.init(); n=Node('amr_probe'); out={'sensors':{},'diagnostics':{}}
    def diagnostic(m):
        for s in m.status: out['diagnostics'][s.name]=s.message
    n.create_subscription(DiagnosticArray,'/diagnostics',diagnostic,10)
    for topic in ['/system/ready','/ground_truth/valid','/motion_guard/enabled']:
        n.create_subscription(Bool,topic,lambda m,t=topic:out.__setitem__(t,m.data),10)
    def sensor(m,topic):
        value={'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'frame':m.header.frame_id}
        if topic=='/scan': value.update(beams=len(m.ranges),nearest=min(m.ranges))
        if topic=='/odom': value.update(x=m.pose.pose.position.x,y=m.pose.pose.position.y,v=m.twist.twist.linear.x,w=m.twist.twist.angular.z)
        out['sensors'][topic]=value
    for topic,kind in [('/scan',LaserScan),('/imu',Imu),('/odom',Odometry)]:
        n.create_subscription(kind,topic,lambda m,t=topic:sensor(m,t),qos_profile_sensor_data)
    n.create_subscription(TaskSnapshot,'/mission/state',lambda m:out.__setitem__('mission',message_to_ordereddict(m)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    end=time.monotonic()+args.seconds
    while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.1)
    print(json.dumps(out,indent=2)); n.destroy_node(); rclpy.try_shutdown()
if __name__=='__main__': main()
