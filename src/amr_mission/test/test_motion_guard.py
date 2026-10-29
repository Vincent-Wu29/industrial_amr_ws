import os
import signal
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.clock import Clock as RosClock, ClockType
from std_msgs.msg import Empty, Bool
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from rosgraph_msgs.msg import Clock

def test_guard_preserves_stamps_and_disarms_on_stale_input(tmp_path):
    domain=224+os.getpid()%3; env=os.environ.copy(); env['ROS_DOMAIN_ID']=str(domain)
    rclpy.init(domain_id=domain); n=Node('guard_fixture',parameter_overrides=[Parameter('use_sim_time',value=True)])
    heart=[True]; scan_on=[True]; statuses=[]; nonzero=[]; began=time.monotonic(); last_sim=[100.]
    clock_pub=n.create_publisher(Clock,'/clock',10); steady=RosClock(clock_type=ClockType.STEADY_TIME)
    heartbeat=n.create_publisher(Empty,'/mission/heartbeat',10)
    scan=n.create_publisher(LaserScan,'/scan',10); odom=n.create_publisher(Odometry,'/odom',10)
    command=n.create_publisher(TwistStamped,'/cmd_vel_nav',1)
    n.create_subscription(Bool,'/motion_guard/enabled',lambda m:statuses.append(m.data),10)
    n.create_subscription(TwistStamped,'/diff_drive_controller/cmd_vel',lambda m:nonzero.append(m) if m.twist.linear.x else None,10)
    client=n.create_client(SetBool,'/motion_guard/set_enabled')
    def pump():
        last_sim[0]=max(last_sim[0],100.+time.monotonic()-began)
        clock=Clock(); clock.clock=rclpy.time.Time(seconds=last_sim[0]).to_msg(); clock_pub.publish(clock)
        if heart[0]: heartbeat.publish(Empty())
        s=LaserScan(); s.header.stamp=clock.clock
        if scan_on[0]: scan.publish(s)
        o=Odometry(); o.header.stamp=s.header.stamp; odom.publish(o)
    n.create_timer(.05,pump,clock=steady); executor=SingleThreadedExecutor(); executor.add_node(n)
    thread=threading.Thread(target=executor.spin,daemon=True); thread.start()
    log=(tmp_path/'guard.log').open('w')
    process=subprocess.Popen(['ros2','run','amr_mission','motion_guard','--ros-args','-p','use_sim_time:=true'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def wait(predicate,seconds=5):
        end=time.monotonic()+seconds
        while not predicate() and time.monotonic()<end: time.sleep(.01)
        assert predicate()
    def enable():
        f=client.call_async(SetBool.Request(data=True)); wait(f.done); assert f.result().success
        wait(lambda:statuses and statuses[-1])
    try:
        assert client.wait_for_service(timeout_sec=10); time.sleep(.3); enable()
        m=TwistStamped(); m.header.stamp=(n.get_clock().now()-rclpy.duration.Duration(seconds=.1)).to_msg(); m.twist.linear.x=.1
        command.publish(m); wait(lambda:bool(nonzero))
        assert nonzero[-1].header.stamp==m.header.stamp
        heart[0]=False; wait(lambda:statuses and not statuses[-1])
        heart[0]=True; time.sleep(.2); enable()
        stale=TwistStamped(); stale.header.stamp=(n.get_clock().now()-rclpy.duration.Duration(seconds=2)).to_msg(); stale.twist.linear.x=.1
        before=len(nonzero); command.publish(stale); wait(lambda:statuses and not statuses[-1]); assert len(nonzero)==before
        enable()
        command.publish(TwistStamped())  # Nav2 may emit an unstamped explicit zero before recovery Wait.
        time.sleep(.7)
        assert statuses[-1], 'explicit zero during healthy recovery must remain stopped and armed'
        fresh=TwistStamped(); fresh.header.stamp=n.get_clock().now().to_msg(); fresh.twist.linear.x=.1
        command.publish(fresh); wait(lambda:len(nonzero)>before)
        assert nonzero[-1].header.stamp==fresh.header.stamp
        wait(lambda:statuses and not statuses[-1])  # last nonzero cannot be replayed after expiry
        enable(); invalid=TwistStamped(); invalid.header.stamp=n.get_clock().now().to_msg()
        invalid.twist.linear.x=.1; invalid.twist.linear.y=float('nan'); before=len(nonzero)
        command.publish(invalid); wait(lambda:statuses and not statuses[-1]); assert len(nonzero)==before
        enable(); command.publish(TwistStamped()); scan_on[0]=False
        wait(lambda:statuses and not statuses[-1])  # zero-command Wait cannot bypass scan freshness
    finally:
        os.killpg(process.pid,signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL); process.wait()
        executor.shutdown(); thread.join(timeout=2); n.destroy_node(); rclpy.try_shutdown(); log.close()
