cmake_minimum_required(VERSION 3.8)
project(point_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  point_navigation/imu_tf_frame_mapper.py
  point_navigation/point_navigation.py
  point_navigation/waypoint.py
  point_navigation/crop_camera_node.py
  point_navigation/image_compressor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()