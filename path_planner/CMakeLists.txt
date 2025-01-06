cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_planner
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

add_executable(coverage_path_node src/coverage_path_node.cpp )

target_link_libraries(coverage_path_node
    ${catkin_LIBRARIES} 
)
