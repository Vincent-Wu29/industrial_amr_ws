cmake_minimum_required(VERSION 3.16)
project(amr_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteTransport.action"
  "msg/MissionEvent.msg"
  "msg/TaskSnapshot.msg"
  "srv/ConfirmOperation.srv"
  "srv/GetStation.srv"
  "srv/GetTransport.srv"
  "srv/ReconcileLoad.srv"
  DEPENDENCIES builtin_interfaces unique_identifier_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
