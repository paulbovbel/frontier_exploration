cmake_minimum_required(VERSION 2.8.3)
project(exploration_server)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++14;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS actionlib costmap_2d exploration_msgs roscpp)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    actionlib
    costmap_2d
    exploration_msgs
    roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(exploration_server src/exploration_server.cpp)
target_link_libraries(exploration_server ${catkin_LIBRARIES})

add_executable(exploration_server_node src/exploration_server_node.cpp)
target_link_libraries(exploration_server_node
  exploration_server ${catkin_LIBRARIES}
)

install(TARGETS exploration_server exploration_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
