cmake_minimum_required(VERSION 3.0.2)
project(ros_basics_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  RPY.msg
  SimplePose.msg
  SimplePoseStamped.msg
  SimpleVelocities.msg
  ProximitySensors.msg
)

add_service_files(
  FILES 
  AddWaypoint.srv
  RemoveWaypoint.srv
  CurrentWaypoint.srv
  GetWaypoints.srv
  SetWaypoints.srv
  CheckWaypointReached.srv
)

generate_messages(DEPENDENCIES
  std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

