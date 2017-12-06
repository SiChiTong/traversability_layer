cmake_minimum_required(VERSION 2.8.3)
project(traversability_layer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  grid_map_msgs
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    grid_map_msgs
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(traversability_layer src/traversability_layer.cpp)

add_dependencies(traversability_layer grid_map_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS traversability_layer
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)