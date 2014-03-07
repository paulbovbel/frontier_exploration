cmake_minimum_required(VERSION 2.8.3)
project(robot_explore)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  roscpp
  std_msgs
  tf
  actionlib
  actionlib_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  UpdateBoundaryPolygon.srv
  GetNextFrontier.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  ExploreTask.action
)

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   std_msgs  actionlib_msgs nav_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES explore_costmap
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure geometry_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(robot_explore src/robot_explore.cpp)

target_link_libraries(
  robot_explore
  ${catkin_LIBRARIES}
)

add_library(explore_costmap plugins/bounded_explore_layer.cpp)
target_link_libraries(explore_costmap ${catkin_LIBRARIES})


