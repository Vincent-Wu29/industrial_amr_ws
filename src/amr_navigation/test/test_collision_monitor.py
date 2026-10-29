"""Exercise the installed native monitor, including stale-scan fail-closed behavior."""
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point32, PolygonStamped, TwistStamped
from lifecycle_msgs.srv import ChangeState
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


def test_raw_scan_stop_approach_stamp_and_source_timeout(tmp_path):
    domain=227+os.getpid()%3; env=os.environ.copy(); env['ROS_DOMAIN_ID']=str(domain)
    rclpy.init(domain_id=domain)
    node=Node('collision_monitor_fixture',parameter_overrides=[Parameter('use_sim_time',value=True)])
    began=time.monotonic(); last_sim=[100.]; distance=[3.]; scan_on=[True]; outputs=[]; sent=set()
    clock_pub=node.create_publisher(Clock,'/clock',10)
    scans=node.create_publisher(LaserScan,'/scan',10)
    footprint=node.create_publisher(PolygonStamped,'/local_costmap/published_footprint',10)
    commands=node.create_publisher(TwistStamped,'/cmd_vel_plan',10)
    node.create_subscription(TwistStamped,'/cmd_vel_nav',lambda m:outputs.append(m),10)
    client=node.create_client(ChangeState,'/collision_monitor/change_state')
    def pump():
        last_sim[0]=max(last_sim[0],100.+time.monotonic()-began)
        stamp=rclpy.time.Time(seconds=last_sim[0]).to_msg()
        clock=Clock(); clock.clock=stamp; clock_pub.publish(clock)
        scan=LaserScan(); scan.header.frame_id='base_footprint'; scan.header.stamp=stamp
        scan.angle_min=-.1; scan.angle_max=.1; scan.angle_increment=.01
        scan.range_min=.05; scan.range_max=10.; scan.ranges=[float(distance[0])]*21
        if scan_on[0]: scans.publish(scan)
        poly=PolygonStamped(); poly.header=scan.header
        poly.polygon.points=[Point32(x=x,y=y) for x,y in [(.32,.27),(.32,-.27),(-.32,-.27),(-.32,.27)]]
        footprint.publish(poly)
        cmd=TwistStamped(); cmd.header=scan.header; cmd.twist.linear.x=.35
        sent.add((stamp.sec,stamp.nanosec)); commands.publish(cmd)
    node.create_timer(.05,pump,clock=RosClock(clock_type=ClockType.STEADY_TIME))
    executor=SingleThreadedExecutor(); executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True); thread.start()
    log=(tmp_path/'monitor.log').open('w')
    params=Path(get_package_share_directory('amr_navigation'))/'config/nav2.yaml'
    process=subprocess.Popen(['ros2','run','nav2_collision_monitor','collision_monitor','--ros-args',
        '--params-file',str(params),'-p','base_shift_correction:=false','-p','bond_heartbeat_period:=0.0'],
        env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def wait(predicate,seconds=8):
        end=time.monotonic()+seconds
        while not predicate() and time.monotonic()<end:
            assert process.poll() is None,(tmp_path/'monitor.log').read_text(); time.sleep(.02)
        assert predicate(),(tmp_path/'monitor.log').read_text()
    try:
        assert client.wait_for_service(timeout_sec=10)
        for transition in (1,3):
            request=ChangeState.Request(); request.transition.id=transition
            future=client.call_async(request); wait(future.done); assert future.result().success
        wait(lambda:outputs and math.isclose(outputs[-1].twist.linear.x,.35,abs_tol=1e-6))
        assert (outputs[-1].header.stamp.sec,outputs[-1].header.stamp.nanosec) in sent
        distance[0]=.6
        wait(lambda:outputs and 0<outputs[-1].twist.linear.x<.3)
        distance[0]=.36
        wait(lambda:outputs and outputs[-1].twist.linear.x==0.)
        distance[0]=3.
        wait(lambda:outputs and outputs[-1].twist.linear.x>.34)
        scan_on[0]=False
        wait(lambda:outputs and outputs[-1].twist.linear.x==0.)
        time.sleep(.2)
        assert outputs[-1].twist.linear.x==0., 'fresh commands cannot bypass expired scan data'
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL); process.wait()
        executor.shutdown(timeout_sec=3); thread.join(timeout=3)
        node.destroy_node(); rclpy.try_shutdown(); log.close()
