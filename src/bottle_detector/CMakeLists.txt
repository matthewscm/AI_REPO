cmake_minimum_required(VERSION 3.8)
project(bottle_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------
# Find Dependencies
# ---------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_index_cpp REQUIRED)

# OpenCV setup - including ml for SVM and dnn for YOLO
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn ml highgui)

# ---------------------------------------------------------
# Executables
# ---------------------------------------------------------

# 1. The Main Detection Node
add_executable(detector_test_node src/detector_test_node.cpp)
target_link_libraries(detector_test_node ${OpenCV_LIBRARIES})
ament_target_dependencies(detector_test_node 
  rclcpp 
  cv_bridge 
  std_msgs
  sensor_msgs 
  image_transport 
  ament_index_cpp
)

# 2. The SVM Training Executable
add_executable(svm_trainer src/svm_trainer.cpp)
target_link_libraries(svm_trainer ${OpenCV_LIBRARIES})
ament_target_dependencies(svm_trainer 
  rclcpp 
  ament_index_cpp
)

# 3. The SVM Model Evaulator:
add_executable(svm_test_accuracy src/svm_test_accuracy.cpp)
target_link_libraries(svm_test_accuracy ${OpenCV_LIBRARIES})
ament_target_dependencies(svm_test_accuracy rclcpp ament_index_cpp)


# ---------------------------------------------------------
# Install Rules
# ---------------------------------------------------------

# Install executables to lib/package_name
install(TARGETS 
  detector_test_node 
  svm_trainer
  svm_test_accuracy
  DESTINATION lib/${PROJECT_NAME}
)

# Install models and data directories to share/package_name
install(DIRECTORY models data images
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------
# Testing & Linting
# ---------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()