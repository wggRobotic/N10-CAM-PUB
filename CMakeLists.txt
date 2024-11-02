cmake_minimum_required(VERSION 3.8)
project(n10_cam_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# add the cam_pub node
add_executable(cam_pub src/cam_pub.cpp)
ament_target_dependencies(cam_pub rclcpp std_msgs sensor_msgs OpenCV cv_bridge image_transport)

# add the qr_code_scanner node
add_executable(qr_code_scanner src/qr_code_scanner.cpp)
ament_target_dependencies(qr_code_scanner rclcpp OpenCV cv_bridge image_transport)

# Include directories
target_include_directories(cam_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(qr_code_scanner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Set C++ standard
target_compile_features(cam_pub PUBLIC c_std_99 cxx_std_17)
target_compile_features(qr_code_scanner PUBLIC c_std_99 cxx_std_17)

# Install targets
install(TARGETS cam_pub qr_code_scanner
  DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
