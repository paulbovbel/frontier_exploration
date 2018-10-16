cmake_minimum_required(VERSION 2.8.3)
project(frontier_exploration)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++14;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  exploration_server
  dynamic_reconfigure
  move_base_msgs
  geometry_msgs
  roscpp
  std_msgs
  std_srvs
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
  LIBRARIES frontier_plugin
  CATKIN_DEPENDS
    costmap_2d
    exploration_server
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

add_library(frontier_plugin src/frontier_plugin.cpp src/frontier_search.cpp)
target_link_libraries(frontier_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(frontier_plugin ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  # Find package test dependencies
  find_package(rostest REQUIRED)
  find_package(gtest)


  # Add the test folder to the include directories
  include_directories(src/test)

  include_directories(${GTEST_INCLUDE_DIRS})
  link_directories(${GTEST_LIBRARY_DIRS})

  catkin_add_gtest(frontier_search_tests src/test/frontier_search_tests.cpp)
  target_link_libraries(frontier_search_tests frontier_plugin)

  catkin_add_gtest(geometry_tools_tests src/test/geometry_tools_tests.cpp)
  target_link_libraries(geometry_tools_tests frontier_plugin)

  catkin_add_gtest(costmap_tools_tests src/test/costmap_tools_tests.cpp)
  target_link_libraries(costmap_tools_tests frontier_plugin)

endif()

install(TARGETS frontier_plugin
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)

install(FILES frontier_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

find_package(roslint REQUIRED)
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()
