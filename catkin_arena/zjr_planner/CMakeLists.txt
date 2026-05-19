cmake_minimum_required(VERSION 3.0.2)
project(zjr_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  tf2
  tf2_geometry_msgs
  tf2_ros
  message_generation
)

catkin_python_setup()


generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

