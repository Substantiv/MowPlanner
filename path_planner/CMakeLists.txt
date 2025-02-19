cmake_minimum_required(VERSION 3.0.2)
project(path_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${Eigen3_INCLUDE_DIRS}
  /usr/include/eigen3
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_planner
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs
)

add_library(trajectory_optimization src/alm_traj_opt.cpp)

target_link_libraries(trajectory_optimization
  ${catkin_LIBRARIES}
)

add_executable(planner_node src/planner_node.cpp)

target_link_libraries(planner_node
  trajectory_optimization
  ${catkin_LIBRARIES}
)
