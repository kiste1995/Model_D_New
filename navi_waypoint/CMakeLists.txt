cmake_minimum_required(VERSION 3.0.2)
project(navi_waypoint)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  geometry_msgs
  roscpp
  roslib
  zetabank_msgs
  std_msgs
  tf
)

find_package(PkgConfig)
pkg_check_modules(YAML_CPP yaml-cpp)
if(${YAML_CPP_VERSION} VERSION_GREATER 0.5)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()
include_directories(${YAML_CPP_INCLUDE_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES navi_Waypoint
  CATKIN_DEPENDS actionlib move_base_msgs geometry_msgs roscpp roslib std_msgs zetabank_msgs tf
  DEPENDS yaml-cpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS}
)

add_executable(navigation_waypoints_node src/navigation_waypoints_node.cpp)

target_link_libraries(navigation_waypoints_node   ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} )


