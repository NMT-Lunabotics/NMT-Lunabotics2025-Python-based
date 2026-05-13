cmake_minimum_required(VERSION 3.8)
project(serial_commands)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  serial_commands/serial_talker_node.py
  serial_commands/serial_writer_node.py
  serial_commands/serial_listener_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()