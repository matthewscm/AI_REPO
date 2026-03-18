cmake_minimum_required(VERSION 3.8)
project(rs2_image_processing_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(bottle_classifier_node src/bottle_classifier_node.cpp)

target_include_directories(bottle_classifier_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(bottle_classifier_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(bottle_classifier_node ${OpenCV_LIBS})

install(TARGETS
  bottle_classifier_node
  DESTINATION lib/${PROJECT_NAME}
)

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
