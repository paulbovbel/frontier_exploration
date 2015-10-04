cmake_minimum_required(VERSION 2.8.3)
project(frontier_exploration)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  move_base_msgs
  geometry_msgs
  roscpp
  std_msgs
  tf
  actionlib
  actionlib_msgs
  visualization_msgs
  message_generation
)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

add_message_files(
  FILES
  Frontier.msg
)

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
  geometry_msgs std_msgs  actionlib_msgs move_base_msgs visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES explore_costmap
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    roscpp
    tf
    actionlib
  DEPENDS
    PCL
    Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(explore_costmap plugins/bounded_explore_layer.cpp src/frontier_search.cpp)
target_link_libraries(explore_costmap ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(explore_costmap ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(explore_server src/explore_server.cpp)
target_link_libraries(explore_server ${catkin_LIBRARIES})
add_dependencies(explore_server ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(explore_client src/explore_client.cpp)
target_link_libraries(explore_client ${catkin_LIBRARIES})
add_dependencies(explore_client ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  # Find package test dependencies
  find_package(rostest REQUIRED)
  find_package(gtest)


  # Add the test folder to the include directories
  include_directories(src/test)

  include_directories(${GTEST_INCLUDE_DIRS})
  link_directories(${GTEST_LIBRARY_DIRS})

  catkin_add_gtest(frontier_search_tests src/test/frontier_search_tests.cpp)
  target_link_libraries(frontier_search_tests explore_costmap)

  catkin_add_gtest(geometry_tools_tests src/test/geometry_tools_tests.cpp)
  target_link_libraries(geometry_tools_tests explore_costmap)

  catkin_add_gtest(costmap_tools_tests src/test/costmap_tools_tests.cpp)
  target_link_libraries(costmap_tools_tests explore_costmap)

endif()

install(TARGETS explore_server explore_client
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS explore_costmap
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
