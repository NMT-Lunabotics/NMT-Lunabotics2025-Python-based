cmake_minimum_required(VERSION 3.5)
project(usb_camera)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
