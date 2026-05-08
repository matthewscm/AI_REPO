cmake_minimum_required(VERSION 3.8)
project(bottle_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS dnn imgproc)
find_package(ament_index_cpp REQUIRED)

add_executable(detector_test_node src/detector_test_node.cpp)
ament_target_dependencies(detector_test_node rclcpp OpenCV ament_index_cpp)

# This tells ROS2 where to put the executable
install(TARGETS detector_test_node
  DESTINATION lib/${PROJECT_NAME})

# This tells ROS2 where to put your model so the code can find it
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
