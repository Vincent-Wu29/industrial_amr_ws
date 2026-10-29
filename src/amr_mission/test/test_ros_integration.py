"""L2: real DDS/action/service adapter with controllable fake Nav2 (no Gazebo)."""
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
import pytest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from rclpy.parameter import Parameter
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Bool
from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool
from amr_interfaces.action import ExecuteTransport
from amr_interfaces.msg import TaskSnapshot
from amr_interfaces.srv import ConfirmOperation, GetTransport, ReconcileLoad
from ament_index_python.packages import get_package_share_directory as share

def await_future(future,timeout=15):
    until=time.monotonic()+timeout
    while not future.done() and time.monotonic()<until: time.sleep(.01)
    assert future.done(), 'future deadline exceeded'
    return future.result()

class FakeSystem(Node):
    def __init__(self):
        super().__init__('fake_system',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.clock_start=time.monotonic(); self.clock_pub=self.create_publisher(Clock,'/clock',10)
        self.last_sim=100.; self.timer_group=MutuallyExclusiveCallbackGroup()
        self.steady_clock=RosClock(clock_type=ClockType.STEADY_TIME)
        self.guard=False; self.target=(2.,7.5,0.,1.); self.goals=0; self.delay_accept=0.; self.execute_delay=.25
        self.state=None; self.reject_goal=False; self.reject_cancel=False; self.cancels=0; self.odom_on=True; self.shutdown_event=threading.Event()
        self.waiters=[]; self.running_goals=0
        self.abort_unclassified=False
        self.cb=ReentrantCallbackGroup()
        self.ready=self.create_publisher(Bool,'/system/ready',10)
        self.guard_pub=self.create_publisher(Bool,'/motion_guard/enabled',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.odom=self.create_publisher(Odometry,'/odom',10); self.tf=TransformBroadcaster(self)
        self.create_service(SetBool,'/motion_guard/set_enabled',self.set_guard,callback_group=self.cb)
        self.create_subscription(TaskSnapshot,'/mission/state',lambda m:setattr(self,'state',m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.server=ActionServer(self,NavigateToPose,'/navigate_to_pose',execute_callback=self.execute,
            goal_callback=self.admit,cancel_callback=self.cancel_nav,callback_group=self.cb)
        self.create_timer(.05,self.tick,callback_group=self.timer_group,clock=self.steady_clock)
        self.client=ActionClient(self,ExecuteTransport,'/mission/execute_transport',callback_group=self.cb)
        self.confirm=self.create_client(ConfirmOperation,'/mission/confirm_operation',callback_group=self.cb)
        self.query=self.create_client(GetTransport,'/mission/get_transport',callback_group=self.cb)
    def set_guard(self,req,res): self.guard=req.data; res.success=True; return res
    def admit(self,goal):
        time.sleep(self.delay_accept); self.goals+=1; return GoalResponse.REJECT if self.reject_goal else GoalResponse.ACCEPT
    def cancel_nav(self,handle):
        self.cancels+=1
        return CancelResponse.REJECT if self.reject_cancel else CancelResponse.ACCEPT
    async def execute(self,handle):
        self.running_goals+=1
        try:
            p=handle.request.pose.pose; self.target=(p.position.x,p.position.y,p.orientation.z,p.orientation.w)
            deadline=time.monotonic()+self.execute_delay
            while time.monotonic()<deadline:
                if self.shutdown_event.is_set(): handle.abort(); return NavigateToPose.Result()
                if handle.is_cancel_requested: handle.canceled(); return NavigateToPose.Result()
                waiter=Future(); self.waiters.append(waiter)
                await waiter
            if self.abort_unclassified: handle.abort()
            elif handle.is_cancel_requested: handle.canceled()
            else: handle.succeed()
            return NavigateToPose.Result()
        finally: self.running_goals-=1
    def tick(self):
        self.last_sim=max(self.last_sim,100.+time.monotonic()-self.clock_start)
        clock=Clock(); clock.clock=rclpy.time.Time(seconds=self.last_sim).to_msg()
        self.clock_pub.publish(clock)
        waiters,self.waiters=self.waiters,[]
        for waiter in waiters:
            if not waiter.done(): waiter.set_result(None)
        self.ready.publish(Bool(data=True)); self.guard_pub.publish(Bool(data=self.guard))
        o=Odometry(); o.header.stamp=clock.clock
        if self.odom_on: self.odom.publish(o)
        tf=TransformStamped(); tf.header.stamp=o.header.stamp; tf.header.frame_id='map'; tf.child_frame_id='base_footprint'
        tf.transform.translation.x,self_target_y,self_z,self_w=self.target
        tf.transform.translation.y=self_target_y; tf.transform.rotation.z=self_z; tf.transform.rotation.w=self_w
        self.tf.sendTransform(tf)
    def send(self,task='TASK-1'):
        assert self.client.wait_for_server(timeout_sec=10)
        return await_future(self.client.send_goal_async(ExecuteTransport.Goal(task_id=task,source_station='A01',target_station='B02')))
    def wait_state(self,state,timeout=15):
        until=time.monotonic()+timeout
        while (not self.state or self.state.state!=state) and time.monotonic()<until: time.sleep(.02)
        assert self.state and self.state.state==state, f'expected state {state}, got {self.state}'
        return self.state
    def confirm_current(self,op):
        m=self.state
        req=ConfirmOperation.Request(run_id=m.run_id,task_id=m.task_id,station_id=m.current_station,interaction_id=m.interaction_id,operation=op,outcome=1)
        return await_future(self.confirm.call_async(req))

@pytest.fixture
def system(tmp_path):
    # Fresh process domain for every test; never joins a user's running simulation.
    domain=215+(os.getpid()+int(time.monotonic()*1000))%5
    os.environ['ROS_DOMAIN_ID']=str(domain)
    rclpy.init(domain_id=domain); node=FakeSystem(); executor=MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True); thread.start()
    exe=Path(share('amr_mission')).parents[1]/'lib/amr_mission/mission_manager'
    if not exe.exists(): exe=Path(os.environ['AMENT_PREFIX_PATH'].split(':')[0])/'lib/amr_mission/mission_manager'
    command=['ros2','run','amr_mission','mission_manager','--ros-args','-p','use_sim_time:=true','-p','reset_world_empty:=true',
             '-p',f"stations_file:={share('amr_mission')}/config/stations.yaml",'-p',f"map_file:={share('amr_navigation')}/maps/warehouse_v1.yaml",'-p',f'artifacts_dir:={tmp_path}']
    log=(tmp_path/'process.log').open('w')
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    node.mission_process=process
    node.process_artifacts=tmp_path
    try:
        deadline=time.monotonic()+15
        while (not node.state or node.state.load_state!=0) and time.monotonic()<deadline:
            assert process.poll() is None, (tmp_path/'process.log').read_text(); time.sleep(.05)
        assert node.state and node.state.load_state==0, (tmp_path/'process.log').read_text()
        yield node
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL); process.wait()
        node.shutdown_event.set()
        deadline=time.monotonic()+2
        while node.running_goals and time.monotonic()<deadline: time.sleep(.02)
        executor.shutdown(timeout_sec=3); thread.join(timeout=3); node.destroy_node(); rclpy.try_shutdown(); log.close()

def test_happy_path_dedup_and_query(system):
    n=system; handle=n.send(); assert handle.accepted
    n.wait_state(4); assert n.goals==1
    assert not n.send().accepted
    assert n.confirm_current(1).accepted
    n.wait_state(7); assert n.goals==2
    assert n.confirm_current(2).accepted
    result=await_future(handle.get_result_async()).result
    assert result.final_state==9 and result.load_state==0
    assert not n.send().accepted
    query=await_future(n.query.call_async(GetTransport.Request(task_id='TASK-1')))
    assert query.found and query.snapshot.state==9 and n.goals==2

def test_late_nav_accept_after_cancel(system):
    n=system; n.delay_accept=1.; n.execute_delay=2.
    handle=n.send(); assert handle.accepted
    n.wait_state(2); response=await_future(handle.cancel_goal_async()); assert response.goals_canceling
    result=await_future(handle.get_result_async()).result
    assert result.final_state==11 and result.error_code==500
    assert n.goals<=1 and not n.guard

def test_cancel_in_load_window_leaves_unknown(system):
    n=system; handle=n.send(); n.wait_state(4)
    assert await_future(handle.cancel_goal_async()).goals_canceling
    result=await_future(handle.get_result_async()).result
    assert result.final_state==11 and result.load_state==2 and result.requires_operator
    assert not n.send('SECOND').accepted

def test_nav_rejection_is_reported(system):
    n=system; n.reject_goal=True; handle=n.send(); assert handle.accepted
    result=await_future(handle.get_result_async()).result
    assert result.final_state==10 and result.error_code==202 and n.goals==1

def test_cancel_rejected_without_terminal_fails_hold(system):
    n=system; n.reject_cancel=True; n.execute_delay=15.
    handle=n.send(); n.wait_state(2)
    time.sleep(.3)
    assert await_future(handle.cancel_goal_async()).goals_canceling
    result=await_future(handle.get_result_async(),8).result
    assert result.final_state==10 and result.error_code==400 and result.requires_operator
    assert not n.send('SECOND').accepted

def test_stale_odometry_never_claims_canceled(system):
    n=system; n.execute_delay=15.; handle=n.send(); n.wait_state(2)
    n.odom_on=False; time.sleep(.7)
    assert await_future(handle.cancel_goal_async()).goals_canceling
    result=await_future(handle.get_result_async(),8).result
    assert result.final_state==10 and result.error_code==401

def test_acceptance_after_business_failure_is_still_canceled(system):
    n=system; n.delay_accept=11.; n.execute_delay=15.
    handle=n.send(); assert handle.accepted
    result=await_future(handle.get_result_async(),13).result
    assert result.final_state==10 and result.error_code==400 and result.requires_operator
    until=time.monotonic()+4
    while not n.cancels and time.monotonic()<until: time.sleep(.02)
    assert n.goals==1 and n.cancels==1, (n.process_artifacts/'process.log').read_text()
    assert not n.send('SECOND').accepted

def test_public_cli_happy_path(system):
    n=system
    process=subprocess.Popen(['ros2','run','amr_mission','task_cli','submit','--task-id','CLI-1'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
    try:
        n.wait_state(4); assert n.confirm_current(1).accepted
        n.wait_state(7); assert n.confirm_current(2).accepted
        stdout,stderr=process.communicate(timeout=10)
        assert process.returncode==0, stderr
        assert '"final_state": 9' in stdout
    finally:
        if process.poll() is None: os.killpg(process.pid,signal.SIGKILL); process.wait()

def test_unclassified_abort_quarantines_empty_robot_and_exits(system):
    n=system; n.abort_unclassified=True; handle=n.send(); assert handle.accepted
    result=await_future(handle.get_result_async()).result
    assert result.final_state==10 and result.error_code==203 and result.load_state==0 and result.requires_operator
    assert not n.send('SECOND').accepted
    client=n.create_client(ReconcileLoad,'/mission/reconcile_load',callback_group=n.cb)
    response=await_future(client.call_async(ReconcileLoad.Request(run_id=result.run_id,load_state=0,operator_note='checked empty')),1)
    assert not response.accepted
    assert n.mission_process.wait(timeout=3)==1
    events=''.join(path.read_text() for path in n.process_artifacts.rglob('events.jsonl'))
    assert 'nav_context_invalidated' in events

def test_unexpected_guard_loss_hard_latches_empty_robot(system):
    n=system; n.execute_delay=10.; handle=n.send(); n.wait_state(2)
    until=time.monotonic()+3
    while not (n.guard and n.goals) and time.monotonic()<until: time.sleep(.02)
    assert n.guard and n.goals; n.guard=False
    result=await_future(handle.get_result_async()).result
    assert result.final_state==10 and result.error_code==200 and result.load_state==0 and result.requires_operator
    assert not n.send('SECOND').accepted
    client=n.create_client(ReconcileLoad,'/mission/reconcile_load',callback_group=n.cb)
    response=await_future(client.call_async(ReconcileLoad.Request(run_id=result.run_id,load_state=0,operator_note='checked empty')))
    assert not response.accepted and n.mission_process.poll() is None

def test_public_cli_bounds_server_loss(system):
    n=system; n.execute_delay=15.
    process=subprocess.Popen(['ros2','run','amr_mission','task_cli','submit','--task-id','CLI-LOST'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
    try:
        n.wait_state(2); time.sleep(.3)
        os.killpg(n.mission_process.pid,signal.SIGKILL); n.mission_process.wait(timeout=5)
        stdout,stderr=process.communicate(timeout=8)
        assert process.returncode==2 and 'UNKNOWN' in stderr
    finally:
        if process.poll() is None: os.killpg(process.pid,signal.SIGKILL); process.wait()

def test_public_cli_bounds_missing_state_channel(system):
    n=system; n.execute_delay=15.
    process=subprocess.Popen(['ros2','run','amr_mission','task_cli','submit','--task-id','CLI-NO-STATE',
        '--ros-args','-r','/mission/state:=/deliberately_missing_state'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
    try:
        stdout,stderr=process.communicate(timeout=10)
        assert process.returncode==2 and 'heartbeat unavailable' in stderr and 'UNKNOWN' in stderr
    finally:
        if process.poll() is None: os.killpg(process.pid,signal.SIGKILL); process.wait()
