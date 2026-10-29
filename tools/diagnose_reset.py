#!/usr/bin/env python3
"""Record raw Gazebo + ROS evidence around a clock reset, with owned cleanup."""
import json
import os
from pathlib import Path
import subprocess
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from amr_interfaces.msg import TaskSnapshot
from amr_interfaces.action import ExecuteTransport
from run_navigation_validation import stop

def main():
    os.environ['ROS_DOMAIN_ID']='157'; os.environ['GZ_PARTITION']='amr_diag_'+uuid.uuid4().hex
    out=Path('artifacts/reset_diagnostic_'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())).resolve(); out.mkdir(parents=True)
    rclpy.init(); node=Node('reset_diagnostic'); values={}; counts={}
    def remember(key,value): values[key]=value; counts[key]=counts.get(key,0)+1
    node.create_subscription(Bool,'/system/ready',lambda m:remember('ready',m.data),10)
    node.create_subscription(Clock,'/clock',lambda m:remember('clock',m.clock.sec+m.clock.nanosec*1e-9),10)
    node.create_subscription(TFMessage,'/ground_truth/poses',lambda m:remember('truth',[(t.child_frame_id,t.header.stamp.sec+t.header.stamp.nanosec*1e-9) for t in m.transforms]),10)
    node.create_subscription(Odometry,'/odom',lambda m:remember('odom',m.header.stamp.sec+m.header.stamp.nanosec*1e-9),qos_profile_sensor_data)
    node.create_subscription(TaskSnapshot,'/mission/state',lambda m:remember('mission',(m.state,m.load_state)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    log=(out/'system.log').open('w')
    process=subprocess.Popen(['ros2','launch','amr_bringup','system.launch.py','mode:=mission','gui:=false','real_time_factor:=3',f'artifacts_dir:={out}'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def until(predicate,timeout):
        end=time.monotonic()+timeout
        while not predicate():
            if process.poll() is not None: raise RuntimeError('system exited')
            if time.monotonic()>end: raise TimeoutError('condition')
            rclpy.spin_once(node,timeout_sec=.05)
    def command(args,name):
        try: result=subprocess.run(args,capture_output=True,text=True,timeout=5)
        except subprocess.TimeoutExpired: print(name,'TIMEOUT',flush=True); return ''
        (out/(name+'.txt')).write_text(result.stdout+result.stderr)
        return result.stdout
    try:
        until(lambda:values.get('ready') and values.get('mission')==(0,0),120)
        client=ActionClient(node,ExecuteTransport,'/mission/execute_transport')
        future=client.send_goal_async(ExecuteTransport.Goal(task_id='DIAG-RESET',source_station='A01',target_station='B02'))
        until(future.done,10); assert future.result().accepted
        until(lambda:values.get('mission',(0,))[0]==2,10)
        end=time.monotonic()+.5
        while time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.05)
        print('BEFORE',values,counts,flush=True)
        command(['gz','topic','-i','-t','/amr/evaluation/poses'],'publisher_before')
        command(['gz','service','-s','/world/warehouse/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','reset: {time_only: true}'],'reset_response')
        start=time.monotonic()
        for checkpoint in [1,5,12,20]:
            end=start+checkpoint
            while time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.05)
            print('AFTER',checkpoint,values,counts,flush=True)
            if checkpoint in (1,12):
                command(['gz','topic','-i','-t','/amr/evaluation/poses'],f'publisher_after_{checkpoint}')
                scene=command(['gz','service','-s','/world/warehouse/scene/info','--reqtype','gz.msgs.Empty','--reptype','gz.msgs.Scene','--timeout','3000','--req',''],f'scene_after_{checkpoint}')
                print('SCENE',checkpoint,'amr' in scene,len(scene),flush=True)
        (out/'snapshot.json').write_text(json.dumps({'values':values,'counts':counts},indent=2))
    finally:
        stop(process); log.close(); node.destroy_node(); rclpy.try_shutdown(); print('Artifacts:',out,flush=True)
if __name__=='__main__': main()
