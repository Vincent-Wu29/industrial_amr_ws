"""Complete single-vehicle simulation. One launch owns one fresh world."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, EmitEvent, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
import os
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory as share
from ament_index_python.packages import get_package_prefix
import subprocess
import shlex
import math
import xml.etree.ElementTree as ET
import yaml
import xacro

def setup(context):
    arg=lambda n: context.launch_configurations[n]
    mode=arg('mode'); world=arg('world')
    if mode not in ('navigation','mission','mapping'): raise ValueError('mode must be navigation, mission or mapping')
    if world not in ('mini_warehouse','warehouse','narrow_aisle'): raise ValueError('unknown world')
    sim=arg('use_sim_time').lower()=='true'
    if not sim: raise ValueError('Gazebo backend requires use_sim_time:=true')
    desc=Path(share('amr_description')); simulation=Path(share('amr_simulation'))
    navigation=Path(share('amr_navigation')); mission=Path(share('amr_mission'))
    mapping=mode=='mapping'
    map_file=arg('map') or str(navigation/'maps'/('narrow_aisle.yaml' if world=='narrow_aisle' else 'warehouse_v1.yaml'))
    stations=arg('stations') or str(mission/'config'/('stations_narrow.yaml' if world=='narrow_aisle' else 'stations.yaml'))
    params=arg('params_file') or str(navigation/'config/nav2.yaml')
    bt=str(navigation/'behavior_trees/navigate.xml')
    padding=.12
    if not mapping:
        navigation_config=yaml.safe_load(Path(params).read_text())
        padding=float(navigation_config['local_costmap']['local_costmap']['ros__parameters']['footprint_padding'])
        global_padding=float(navigation_config['global_costmap']['global_costmap']['ros__parameters']['footprint_padding'])
        if abs(padding-global_padding)>1e-9: raise ValueError('local/global footprint padding mismatch')
        validator=Path(get_package_prefix('amr_mission'))/'lib/amr_mission/validate_stations'
        checked=subprocess.run([str(validator),stations,map_file,str(padding)],capture_output=True,text=True,timeout=10)
        if checked.returncode: raise ValueError(checked.stderr)
    robot=xacro.process_file(str(desc/'urdf/amr.urdf.xacro'),mappings={'controllers_file':str(simulation/'config/controllers.yaml')}).toxml()
    critical=[]; actions=[]
    actions.append(SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',[
        str(Path(get_package_prefix('amr_simulation'))/'lib'),os.pathsep,
        EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',default_value='')]))
    def node(package,executable,**kwargs):
        n=Node(package=package,executable=executable,output='screen',**kwargs)
        actions.append(n); critical.append(n); return n
    seed=int(arg('seed'))
    factor=float(arg('real_time_factor'))
    if not math.isfinite(factor) or not 0<factor<=10: raise ValueError('real_time_factor must be in (0,10]')
    world_path=simulation/'worlds'/f'{world}.sdf'
    if factor!=1.0:
        tree=ET.parse(world_path); tree.find('.//physics/real_time_factor').text=str(factor)
        directory=Path(arg('artifacts_dir')).resolve(); directory.mkdir(parents=True,exist_ok=True)
        world_path=directory/'effective_world.sdf'; tree.write(world_path,encoding='unicode')
    gz_args=f'-r --seed {seed} '+shlex.quote(str(world_path))
    if arg('gui').lower()!='true': gz_args='-s --headless-rendering '+gz_args
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('ros_gz_sim'))/'launch/gz_sim.launch.py')),
                   launch_arguments={'gz_args':gz_args,'on_exit_shutdown':'true'}.items()))
    node('robot_state_publisher','robot_state_publisher',parameters=[{'robot_description':robot,'use_sim_time':True}])
    node('ros_gz_bridge','parameter_bridge',parameters=[{'config_file':str(simulation/'config/bridge.yaml'),'use_sim_time':True}])
    spawn=Node(package='ros_gz_sim',executable='create',arguments=['-world','warehouse','-name','amr','-topic','robot_description','-x','2.0','-y','7.5','-z','0.02'],output='screen')
    controllers=Node(package='controller_manager',executable='spawner',arguments=[
        'joint_state_broadcaster','diff_drive_controller','--activate-as-group','--controller-manager-timeout','60'],output='screen')
    def after_spawn(event,ctx):
        if event.returncode!=0: return [EmitEvent(event=Shutdown(reason='robot spawn failed'))]
        return [controllers]
    actions += [RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=after_spawn)),spawn]
    actions.append(RegisterEventHandler(OnProcessExit(target_action=controllers,on_exit=lambda event,ctx:
        [EmitEvent(event=Shutdown(reason='controller activation failed'))] if event.returncode else [])))
    # All autonomous navigation passes the raw-scan monitor. Mission additionally
    # requires the independent authorization/freshness guard at the final output.
    velocity='/cmd_vel_plan'
    if mapping:
        node('slam_toolbox','async_slam_toolbox_node',name='slam_toolbox',parameters=[str(navigation/'config/slam.yaml'),{'use_lifecycle_manager':False}])
    else:
        node('nav2_map_server','map_server',parameters=[params,{'yaml_filename':map_file,'use_sim_time':True,'bond_heartbeat_period':0.0}])
        node('nav2_amcl','amcl',parameters=[params,{'use_sim_time':True,'bond_heartbeat_period':0.0}])
        names=[]
        for package,executable in [('nav2_controller','controller_server'),('nav2_planner','planner_server'),('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator')]:
            names.append(executable)
            node(package,executable,parameters=[params,{'use_sim_time':True,'default_nav_to_pose_bt_xml':bt,'bond_heartbeat_period':0.0}],remappings=[('cmd_vel',velocity)])
        node('nav2_collision_monitor','collision_monitor',parameters=[params,{
            'bond_heartbeat_period':0.0,'cmd_vel_out_topic':'/cmd_vel_nav' if mode=='mission' else '/diff_drive_controller/cmd_vel'}])
    node('amr_bringup','readiness_monitor',parameters=[{'use_sim_time':True,'mapping':mapping}])
    node('amr_bringup','lifecycle_coordinator',parameters=[{'use_sim_time':True,'mode':'mapping' if mapping else 'navigation'}])
    node('amr_simulation','gz_truth_bridge',parameters=[{'use_sim_time':True}])
    node('amr_simulation','truth_monitor',parameters=[{'use_sim_time':True,'world':world,'artifacts_dir':arg('artifacts_dir')}])
    if mode=='mission':
        node('amr_mission','motion_guard',parameters=[str(mission/'config/mission.yaml')])
        node('amr_mission','mission_manager',parameters=[str(mission/'config/mission.yaml'),
             {'stations_file':stations,'map_file':map_file,'behavior_tree':bt,'footprint_padding':padding,'reset_world_empty':True,'artifacts_dir':arg('artifacts_dir')}])
        node('amr_mission','simulated_station',parameters=[{'use_sim_time':True,'mode':arg('device_mode'),'delay_s':1.0}])
    if arg('rviz').lower()=='true':
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(navigation/'config/amr.rviz')],parameters=[{'use_sim_time':True}],output='screen'))
    for process in critical:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,on_exit=[EmitEvent(event=Shutdown(reason='critical process exited'))])))
    return actions

def generate_launch_description():
    defaults={'mode':'mission','world':'warehouse','use_sim_time':'true','gui':'false','rviz':'false',
              'map':'','stations':'','params_file':'','device_mode':'manual','artifacts_dir':'artifacts','seed':'42','real_time_factor':'1.0'}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
