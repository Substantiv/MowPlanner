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
find_package(Protobuf REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS lib/mu_values.proto)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROTOBUF_INCLUDE_DIRS}
)

catkin_package()

add_executable(robot_wrench_node src/robot_wrench_node.cpp ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(robot_wrench_node
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)