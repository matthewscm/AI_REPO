cmake_minimum_required(VERSION 3.8)
project(simulation_cpp)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED) # Added for geometric_shapes mesh conversion

# 2. Executable: static_environment_node
add_executable(static_environment_node src/static_environment_node.cpp)

# Use ament_target_dependencies for standard ROS 2 packages
ament_target_dependencies(static_environment_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  geometry_msgs
  geometric_shapes
)

# Use target_link_libraries for raw CMake packages (Eigen, Boost)
target_link_libraries(static_environment_node
  Eigen3::Eigen
)

target_include_directories(static_environment_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)

# 3. Executable: bottle_spawner_node (Similar logic)
add_executable(bottle_spawner_node src/bottleplacement.cpp)
ament_target_dependencies(bottle_spawner_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  shape_msgs
  geometric_shapes
)

# 4. Install Logic
install(TARGETS
  bottle_spawner_node
  static_environment_node
  DESTINATION lib/${PROJECT_NAME})

# Ensure meshes are installed so "package://simulation_cpp/meshes/..." works
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install other directories
install(DIRECTORY launch config scenes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()