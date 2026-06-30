cmake_minimum_required(VERSION 3.5)
project(steelhead_object_recognition)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python module package
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  steelhead_object_recognition/yolo_detector.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_export_dependencies(ament_cmake_python)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Package configuration
ament_package()
