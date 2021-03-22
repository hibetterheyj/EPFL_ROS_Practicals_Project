cmake_minimum_required(VERSION 3.0.2)
project(ros_basics_exercise)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  ros_basics_msgs
  ros_basics_control
  dynamic_reconfigure
)

# yujie 0315
generate_dynamic_reconfigure_options(
  cfg/ThymioController.cfg
)

catkin_package(
  CATKIN_DEPENDS ros_basics_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRS}
)

add_executable(thymio_control_ex_node src/thymio_control_node.cpp)
target_link_libraries(thymio_control_ex_node ros_basics_control_lib ${catkin_LIBRARIES})
add_dependencies(thymio_control_ex_node ${catkin_EXPORTED_TARGETS} ros_basics_msgs_generate_messages_cpp)

