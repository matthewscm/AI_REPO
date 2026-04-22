cmake_minimum_required(VERSION 3.8)
project(ur3e_motion)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(move_to_pose src/move_to_pose.cpp)

ament_target_dependencies(move_to_pose
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
  moveit_msgs
  geometry_msgs
  shape_msgs
  std_msgs
)

install(TARGETS move_to_pose
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
