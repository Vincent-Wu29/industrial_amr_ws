import os
import signal
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from lifecycle_msgs.srv import GetState, ChangeState

def test_delayed_transition_response_is_verified_without_reconfigure(tmp_path):
    domain=220+os.getpid()%4; env=os.environ.copy(); env['ROS_DOMAIN_ID']=str(domain)
    rclpy.init(domain_id=domain); node=Node('lifecycle_fixture'); cb=ReentrantCallbackGroup()
    states={'fake0':1,'fake1':1}; counts={name:{1:0,3:0} for name in states}; ready=[]
    pub=node.create_publisher(Bool,'/system/physical_ready',10)
    node.create_timer(.05,lambda:pub.publish(Bool(data=True)),callback_group=cb)
    node.create_subscription(Bool,'/system/lifecycle_ready',lambda m:ready.append(m.data),10,callback_group=cb)
    def get(name):
        def callback(request,response): response.current_state.id=states[name]; return response
        return callback
    def change(name):
        def callback(request,response):
            transition=request.transition.id; counts[name][transition]+=1
            states[name]=2 if transition==1 else 3
            if transition==1: time.sleep(.6)  # applied, but response misses client deadline
            response.success=True; return response
        return callback
    for name in states:
        node.create_service(GetState,f'/{name}/get_state',get(name),callback_group=cb)
        node.create_service(ChangeState,f'/{name}/change_state',change(name),callback_group=cb)
    executor=MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True); thread.start()
    log=(tmp_path/'coordinator.log').open('w')
    process=subprocess.Popen(['ros2','run','amr_bringup','lifecycle_coordinator','--ros-args','-p','node_names:=[fake0, fake1]',
        '-p','response_timeout_s:=0.15','-p','startup_timeout_s:=8.0'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        end=time.monotonic()+10
        while not any(ready) and time.monotonic()<end:
            assert process.poll() is None,(tmp_path/'coordinator.log').read_text(); time.sleep(.02)
        assert any(ready),(tmp_path/'coordinator.log').read_text()
        assert counts=={'fake0':{1:1,3:1},'fake1':{1:1,3:1}}
        assert states=={'fake0':3,'fake1':3}
    finally:
        if process.poll() is None: os.killpg(process.pid,signal.SIGINT); process.wait(timeout=5)
        executor.shutdown(timeout_sec=3); thread.join(timeout=3); node.destroy_node(); rclpy.try_shutdown(); log.close()

def test_startup_without_dependencies_has_wall_deadline():
    env=os.environ.copy(); env['ROS_DOMAIN_ID']='232'
    result=subprocess.run(['ros2','run','amr_bringup','lifecycle_coordinator','--ros-args','-p','node_names:=[missing_node]',
        '-p','startup_timeout_s:=0.8'],env=env,capture_output=True,text=True,timeout=5)
    assert result.returncode==1 and 'startup deadline' in result.stderr+result.stdout
