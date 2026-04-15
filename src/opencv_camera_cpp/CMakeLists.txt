cmake_minimum_required(VERSION 3.8)
project(opencv_camera_cpp)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add your executable
add_executable(camera_node src/camera_node.cpp)
add_executable(bottle_publisher src/bottle_publisher.cpp)
add_executable(bottle_test_publisher src/bottle_test_publisher.cpp)
add_executable(mission_control_gui src/mission_control_gui.cpp)


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

ament_target_dependencies(bottle_test_publisher
  rclcpp
  std_msgs
  geometry_msgs
)

ament_target_dependencies(mission_control_gui
  rclcpp
  std_msgs
)

target_link_libraries(mission_control_gui
  Qt5::Widgets
)

# Install the executable
install(TARGETS
  camera_node
  bottle_publisher
  bottle_test_publisher
  mission_control_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
)
  
# Testing / linting (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()