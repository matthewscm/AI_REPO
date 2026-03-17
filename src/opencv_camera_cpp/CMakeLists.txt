cmake_minimum_required(VERSION 3.8)
project(opencv_camera_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add your executable
add_executable(camera_node src/camera_node.cpp)
add_executable(bottle_publisher src/bottle_publisher.cpp)


# Link dependencies
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

ament_target_dependencies(bottle_publisher
  rclcpp
  std_msgs
)

# Install the executable
install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS bottle_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Testing / linting (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()