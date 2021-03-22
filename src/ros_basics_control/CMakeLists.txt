cmake_minimum_required(VERSION 3.0.2)
project(ros_basics_control)

add_compile_options(-std=c++11)
add_definitions("-Wall -O3 -march=native")  

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  ros_basics_msgs
)

catkin_package(
  CATKIN_DEPENDS ros_basics_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ros_basics_control/include
)

add_library(ros_basics_control_lib src/thymio_interface_base.cpp src/thymio_simu_interface.cpp src/gazebo_robot_tracker.cpp src/robot_tracker.cpp)

add_executable(thymio_ctrl_node src/thymio_ctrl_node.cpp)
target_link_libraries(thymio_ctrl_node ros_basics_control_lib ${catkin_LIBRARIES})
add_dependencies(thymio_ctrl_node ${catkin_EXPORTED_TARGETS} ros_basics_msgs_generate_messages_cpp)

catkin_package(CATKIN_DEPENDS std_msgs
               INCLUDE_DIRS include
               LIBRARIES ros_basics_control_lib)

install(TARGETS ros_basics_control_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})