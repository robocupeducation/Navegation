cmake_minimum_required(VERSION 2.8.3)
project(navigation_robocup_edu)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  actionlib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES csuro_mapping
#  CATKIN_DEPENDS geometry_msgs roscpp rospy tf
#  DEPENDS system_lib
)


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(visit_my_home_node src/visit_my_home.cpp)
add_dependencies(visit_my_home_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries(visit_my_home_node ${catkin_LIBRARIES} )
