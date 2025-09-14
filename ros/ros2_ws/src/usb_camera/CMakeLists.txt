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

# Include launch directory
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

# Install executables
install(TARGETS
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()