#!/usr/bin/env python3
"""Repository bootstrap utility; package manifests remain regular checked-in files."""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
PACKAGES={
 'amr_description':['xacro','robot_state_publisher','launch_ros'],
 'amr_simulation':['ros_gz_sim','ros_gz_bridge','gz_ros2_control','diff_drive_controller','joint_state_broadcaster','controller_manager','rclpy','rclcpp','gz_transport_vendor','gz_msgs_vendor','gz_sim_vendor','gz_plugin_vendor','ros_gz_interfaces','geometry_msgs','sensor_msgs','tf2_ros','tf2_msgs','nav_msgs','std_msgs'],
 'amr_navigation':['nav2_bringup','nav2_navfn_planner','nav2_dwb_controller','nav2_map_server','nav2_amcl','slam_toolbox','rclpy','nav2_msgs','amr_interfaces','tf2_ros','python3-yaml','rcl_interfaces'],
 'amr_bringup':['amr_description','amr_simulation','amr_navigation','amr_mission','amr_interfaces','nav2_msgs','rclpy','launch_ros','xacro','python3-yaml','lifecycle_msgs','controller_manager_msgs','sensor_msgs','nav_msgs','tf2_ros','rosgraph_msgs','std_msgs','std_srvs','diagnostic_msgs'],
 'amr_mission':['rclcpp','rclcpp_action','amr_interfaces','nav2_msgs','nav_msgs','sensor_msgs','geometry_msgs','std_msgs','std_srvs','diagnostic_msgs','tf2','tf2_ros','ament_index_cpp','yaml_cpp_vendor','rclpy','action_msgs','rosidl_runtime_py','python3-yaml'],
 'amr_interfaces':['rosidl_default_generators','builtin_interfaces','unique_identifier_msgs','geometry_msgs']}
def main():
 for name,deps in PACKAGES.items():
    folder=ROOT/'src'/name; folder.mkdir(parents=True,exist_ok=True)
    build_deps=['<buildtool_depend>ament_cmake</buildtool_depend>']
    extra=''
    if name=='amr_interfaces':
        extra='<exec_depend>rosidl_default_runtime</exec_depend>\n  <member_of_group>rosidl_interface_packages</member_of_group>'
    xml=f'''<?xml version="1.0"?>
<package format="3">
  <name>{name}</name><version>0.1.0</version>
  <description>Industrial AMR {name.removeprefix('amr_')} subsystem</description>
  <maintainer email="maintainer@example.com">AMR contributors</maintainer>
  <license>MIT</license>
  {''.join(build_deps)}
  {''.join(f'<depend>{d}</depend>' for d in deps)}
  {extra}
  <test_depend>ament_cmake_pytest</test_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
'''
    (folder/'package.xml').write_text(xml,encoding='utf-8')
    if name in ('amr_mission','amr_simulation'): continue
    cmake=f'cmake_minimum_required(VERSION 3.16)\nproject({name})\nfind_package(ament_cmake REQUIRED)\n'
    if name=='amr_interfaces':
        cmake+='find_package(rosidl_default_generators REQUIRED)\nfind_package(builtin_interfaces REQUIRED)\nfind_package(unique_identifier_msgs REQUIRED)\nfind_package(geometry_msgs REQUIRED)\n'
        files=sorted(str(p.relative_to(folder)).replace('\\','/') for p in folder.rglob('*') if p.suffix in ['.action','.msg','.srv'])
        cmake+='rosidl_generate_interfaces(${PROJECT_NAME}\n'+''.join(f'  "{f}"\n' for f in files)+'  DEPENDENCIES builtin_interfaces unique_identifier_msgs geometry_msgs)\nament_export_dependencies(rosidl_default_runtime)\n'
    else:
        for sub in ['launch','config','scripts','test']:
            (folder/sub).mkdir(exist_ok=True)
        cmake+='foreach(resource launch config maps worlds urdf behavior_trees)\n  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${resource}")\n    install(DIRECTORY ${resource} DESTINATION share/${PROJECT_NAME})\n  endif()\nendforeach()\nfile(GLOB scripts LIST_DIRECTORIES false CONFIGURE_DEPENDS scripts/*)\nif(scripts)\n  install(PROGRAMS ${scripts} DESTINATION lib/${PROJECT_NAME})\nendif()\n'
        cmake+='if(BUILD_TESTING)\n  find_package(ament_cmake_pytest REQUIRED)\n  file(GLOB tests CONFIGURE_DEPENDS test/test_*.py)\n  foreach(test ${tests})\n    get_filename_component(name ${test} NAME_WE)\n    ament_add_pytest_test(${name} ${test} TIMEOUT 120)\n  endforeach()\nendif()\n'
    cmake+='ament_package()\n'
    (folder/'CMakeLists.txt').write_text(cmake,encoding='utf-8')
 print('Six package manifests generated.')
if __name__=='__main__': main()
