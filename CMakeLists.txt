cmake_minimum_required(VERSION 2.8.3)
project(frontier_exploration)

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

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

add_service_files(
  FILES
  UpdateBoundaryPolygon.srv
  GetNextFrontier.srv
)

add_action_files(
  FILES
  ExploreTask.action
)

 generate_messages(
   DEPENDENCIES
   geometry_msgs   std_msgs  actionlib_msgs nav_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES explore_costmap
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    roscpp
    std_msgs
    tf
    actionlib
    actionlib_msgs
    message_generation
  DEPENDS
    PCL
    Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(explore_server src/explore_server.cpp)

target_link_libraries(
  robot_explore
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES
  ${Boost_LIBRARIES}
)

add_library(explore_costmap plugins/bounded_explore_layer.cpp)
target_link_libraries(explore_costmap ${catkin_LIBRARIES})


