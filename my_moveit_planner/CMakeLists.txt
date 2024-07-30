cmake_minimum_required(VERSION 3.0.2)
project(my_moveit_planner)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  message_generation
  tf2_ros
  actionlib
  actionlib_msgs
  geometry_msgs
  rosserial_python
  std_srvs
)

## Uncomment and modify the following lines if you have custom messages
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Uncomment and modify the following lines if you have custom services
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Add action files
add_action_files(
  FILES 
  Pick.action
  Final.action
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS moveit_core moveit_ros_planning_interface roscpp rospy std_msgs message_runtime tf2_ros actionlib actionlib_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/action_server.py    
  scripts/action_client.py
  scripts/task_server.py  
  scripts/point_stampd.py
  scripts/plan_to_object.py 
  scripts/pick_place.py
  scripts/ex_ptstmpd.py
  scripts/Final_action.py
  scripts/rosservice_wrapper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
