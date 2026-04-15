cmake_minimum_required(VERSION 3.8)
project(simulation_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(jsoncpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rviz_rendering REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)


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



# add_executable(object_spawner_node src/object_spawner_node.cpp)
# ament_target_dependencies(object_spawner_node
#     rclcpp
#     moveit_ros_planning_interface
# )


# add_executable(camera_node src/camera_node.cpp)
# ament_target_dependencies(camera_node
#   rclcpp
#   sensor_msgs
#   cv_bridge
#   image_transport
#   OpenCV
# )


# add_executable(camera_subscriber_node src/camera_subscriber_node.cpp)
# ament_target_dependencies(camera_subscriber_node rclcpp sensor_msgs)

# add_executable(scene_loader_node src/scene_loader_node.cpp)
# ament_target_dependencies(scene_loader_node
#   rclcpp
#   moveit_ros_planning
#   moveit_core
# )

# target_link_libraries(scene_loader_node
#   jsoncpp_lib      # required for parsing .scene JSON
# )


# # 3. Install
# install(TARGETS
#   camera_node
#   DESTINATION lib/${PROJECT_NAME})

# 3. Install
install(TARGETS
  bottle_spawner_node
  # object_spawner_node
  # scene_loader_node
  DESTINATION lib/${PROJECT_NAME})

# Install mesh resources
install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install scene files
install(
  DIRECTORY scenes/
  DESTINATION share/${PROJECT_NAME}/scenes
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)



ament_package()