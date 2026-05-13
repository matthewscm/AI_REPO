cmake_minimum_required(VERSION 3.8)
project(simulation_cpp)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------
# 1. Dependencies
# ---------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system) 

# ---------------------------------------------------------
# 2. Executable: static_environment_node
# ---------------------------------------------------------
add_executable(static_environment_node src/static_environment_node.cpp)
ament_target_dependencies(static_environment_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  geometry_msgs
  geometric_shapes
)
target_link_libraries(static_environment_node Eigen3::Eigen)

# ---------------------------------------------------------
# 3. Executable: bottle_spawner_node
# ---------------------------------------------------------
add_executable(bottle_spawner_node src/bottleplacement.cpp)
ament_target_dependencies(bottle_spawner_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  geometric_shapes
)
target_link_libraries(bottle_spawner_node Eigen3::Eigen)

# # ---------------------------------------------------------
# # 4. Executable: bottle_picker_monitor
# # ---------------------------------------------------------
# add_executable(bottle_picker_monitor src/bottle_picker_monitor.cpp)
# ament_target_dependencies(bottle_picker_monitor
#   rclcpp
#   moveit_ros_planning_interface
#   moveit_msgs
#   geometry_msgs
#   tf2
#   tf2_geometry_msgs
# )

# ---------------------------------------------------------
# 5. Executable: bottle_spawn_node (NEW)
# ---------------------------------------------------------
add_executable(bottle_spawn_node src/bottle_spawn_node.cpp)
ament_target_dependencies(bottle_spawn_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  geometric_shapes
)
target_link_libraries(bottle_spawn_node Eigen3::Eigen)

# ---------------------------------------------------------
# 6. Install Logic
# ---------------------------------------------------------
install(TARGETS
  static_environment_node
  bottle_spawner_node
  # bottle_picker_monitor
  bottle_spawn_node
  DESTINATION lib/${PROJECT_NAME})

# Ensure resources are installed to the share directory
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(DIRECTORY launch config scenes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()