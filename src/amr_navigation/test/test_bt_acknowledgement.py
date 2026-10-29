"""Real Nav2 BT: delayed child acknowledgement reproduces the old RPC failure."""
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
import pytest
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from rclpy.parameter import Parameter
from rclpy.clock import Clock as RosClock, ClockType
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose, Wait
from nav2_msgs.srv import ClearEntireCostmap
from lifecycle_msgs.srv import ChangeState
from tf2_ros import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory as share


class Children(Node):
    def __init__(self):
        super().__init__('delayed_bt_children',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.clock_start=time.monotonic(); self.clock_pub=self.create_publisher(Clock,'/clock',10)
        self.last_sim=100.; self.timer_group=MutuallyExclusiveCallbackGroup()
        self.steady_clock=RosClock(clock_type=ClockType.STEADY_TIME)
        self.cb=ReentrantCallbackGroup(); self.pending=[]; self.plans=0; self.follows=0
        self.servers=[
            ActionServer(self,ComputePathToPose,'compute_path_to_pose',self.plan,goal_callback=self.delay,callback_group=self.cb),
            ActionServer(self,FollowPath,'follow_path',self.follow,goal_callback=self.delay,callback_group=self.cb),
            ActionServer(self,Wait,'wait',self.wait,callback_group=self.cb)]
        for name in ['local_costmap/clear_entirely_local_costmap','global_costmap/clear_entirely_global_costmap']:
            self.create_service(ClearEntireCostmap,name,lambda request,response:response,callback_group=self.cb)
        self.tf=StaticTransformBroadcaster(self)
        transform=TransformStamped(); transform.header.frame_id='map'; transform.child_frame_id='base_footprint'
        transform.transform.rotation.w=1.; self.tf.sendTransform(transform)
        self.odom=self.create_publisher(Odometry,'odom',10)
        self.create_timer(.01,self.tick,callback_group=self.timer_group,clock=self.steady_clock)

    async def delay(self,goal):
        future=Future(); self.pending.append((time.monotonic()+.12,future))
        await future
        return GoalResponse.ACCEPT

    def tick(self):
        self.last_sim=max(self.last_sim,100.+time.monotonic()-self.clock_start)
        clock=Clock(); clock.clock=rclpy.time.Time(seconds=self.last_sim).to_msg()
        self.clock_pub.publish(clock)
        remaining=[]
        for deadline,future in self.pending:
            if time.monotonic()>=deadline:
                if not future.done(): future.set_result(None)
            else: remaining.append((deadline,future))
        self.pending=remaining
        message=Odometry(); message.header.stamp=clock.clock; self.odom.publish(message)

    def plan(self,handle):
        self.plans+=1; result=ComputePathToPose.Result(); result.path.header=handle.request.goal.header
        start=PoseStamped(); start.header=handle.request.goal.header; start.pose.orientation.w=1.
        result.path.poses=[start,handle.request.goal]; handle.succeed(); return result

    def follow(self,handle):
        self.follows+=1; handle.succeed(); return FollowPath.Result()

    def wait(self,handle):
        # Recovery duration is irrelevant here; only child RPC acknowledgement is exercised.
        handle.succeed(); return Wait.Result()


@pytest.mark.parametrize('budget,expected',[(20,6),('configured',4)])
def test_real_bt_acknowledgement_budget(tmp_path,budget,expected):
    domain=227 if budget==20 else 228; env=os.environ.copy(); env['ROS_DOMAIN_ID']=str(domain)
    navigation=Path(share('amr_navigation')); config=navigation/'config/nav2.yaml'
    timeout=yaml.safe_load(config.read_text())['bt_navigator']['ros__parameters']['default_server_timeout'] if budget=='configured' else budget
    rclpy.init(domain_id=domain); node=Children(); executor=MultiThreadedExecutor(num_threads=6); executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True); thread.start()
    log_path=tmp_path/'bt.log'; log=log_path.open('w')
    process=subprocess.Popen(['ros2','run','nav2_bt_navigator','bt_navigator','--ros-args','--params-file',str(config),
        '-p','use_sim_time:=true','-p',f'default_server_timeout:={timeout}','-p','bond_heartbeat_period:=0.0',
        '-p',f'default_nav_to_pose_bt_xml:={navigation}/behavior_trees/navigate.xml'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def wait(future,seconds=8):
        end=time.monotonic()+seconds
        while not future.done() and time.monotonic()<end:
            assert process.poll() is None,log_path.read_text(); time.sleep(.01)
        assert future.done(),log_path.read_text()
        return future.result()
    try:
        transition=node.create_client(ChangeState,'/bt_navigator/change_state',callback_group=node.cb)
        assert transition.wait_for_service(timeout_sec=10),log_path.read_text()
        for number in [1,3]:
            request=ChangeState.Request(); request.transition.id=number
            assert wait(transition.call_async(request)).success,log_path.read_text()
        client=ActionClient(node,NavigateToPose,'navigate_to_pose',callback_group=node.cb)
        assert client.wait_for_server(timeout_sec=5)
        goal=NavigateToPose.Goal(); goal.pose.header.frame_id='map'; goal.pose.pose.position.x=1.; goal.pose.pose.orientation.w=1.
        handle=wait(client.send_goal_async(goal)); assert handle.accepted
        outcome=wait(handle.get_result_async()); assert outcome.status==expected,log_path.read_text()
        assert outcome.result.error_code==0
        if expected==4: assert node.plans==1 and node.follows==1
        else: assert 'Timed out while waiting for action server to acknowledge' in log_path.read_text()
    finally:
        if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL); process.wait()
        time.sleep(.2)
        executor.shutdown(timeout_sec=3); thread.join(timeout=3); node.destroy_node(); rclpy.try_shutdown(); log.close()
