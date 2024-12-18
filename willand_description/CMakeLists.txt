cmake_minimum_required(VERSION 3.0.2)
project(willand_description)


find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  gazebo_ros
  nav_msgs
)

find_package(Gazebo REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(contact_read_node src/contactRead.cpp)

target_link_libraries(contact_read_node
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
