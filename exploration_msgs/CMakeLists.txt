cmake_minimum_required(VERSION 2.8.3)
project(exploration_msgs)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++14;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
)

add_action_files(
  FILES
  Explore.action
)

add_service_files(
  FILES
  SetPolygon.srv
)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime
)
