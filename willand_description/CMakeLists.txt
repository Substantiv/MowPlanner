cmake_minimum_required(VERSION 3.0.2)
project(willand_description)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  gazebo_ros
  nav_msgs
)

find_package(Gazebo REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

add_library(robot_wrench src/robot_wrench.cpp)

target_link_libraries(robot_wrench
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_executable(robot_wrench_node src/robot_wrench_node.cpp)

target_link_libraries(robot_wrench_node
  robot_wrench
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)