cmake_minimum_required(VERSION 2.8.3)
project(polygon_layer)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++14;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  exploration_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES polygon_layer
  CATKIN_DEPENDS costmap_2d geometry_msgs
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(polygon_layer
  src/polygon_layer.cpp
)
target_link_libraries(polygon_layer
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS polygon_layer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

find_package(roslint REQUIRED)
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()
