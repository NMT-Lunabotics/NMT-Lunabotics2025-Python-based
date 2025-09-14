cmake_minimum_required(VERSION 3.5)
project(usb_camera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable
add_executable(usb_camera_node src/usb_camera_node.cpp)
ament_target_dependencies(usb_camera_node rclcpp sensor_msgs cv_bridge OpenCV)

# Install executable
install(TARGETS usb_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()