cmake_minimum_required(VERSION 3.8)
project(simulation_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)

# 2. Executable Logic
# Points to src/bottleplacement.cpp relative to this CMakeLists file
add_executable(bottle_spawner_node src/bottleplacement.cpp)

ament_target_dependencies(bottle_spawner_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  shape_msgs
  geometric_shapes
)

# Add to your executable
target_link_libraries(bottle_spawner_node
  geometric_shapes::geometric_shapes
)

# 3. Install
install(TARGETS
  bottle_spawner_node
  DESTINATION lib/${PROJECT_NAME})

# Install mesh resources
install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()