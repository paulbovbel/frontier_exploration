cmake_minimum_required(VERSION 2.8.3)
project(exploration_server)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++14;-Wall;-Werror")


find_package(catkin REQUIRED COMPONENTS actionlib costmap_2d exploration_msgs roscpp move_base_msgs tf)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    actionlib
    costmap_2d
    exploration_msgs
    roscpp
    tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(exploration_server src/exploration_server.cpp)
add_library(example_plugin src/example_plugin.cpp)
target_link_libraries(exploration_server ${catkin_LIBRARIES})

add_executable(exploration_server_node src/exploration_server_node.cpp)
target_link_libraries(exploration_server_node
  exploration_server ${catkin_LIBRARIES}
)

add_executable(plugin_client src/plugin_client.cpp)
target_link_libraries(plugin_client
  exploration_server ${catkin_LIBRARIES}
)

install(TARGETS exploration_server exploration_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


find_package(roslint REQUIRED)
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()
