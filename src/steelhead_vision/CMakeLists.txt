cmake_minimum_required(VERSION 3.10)
project(steelhead_vision)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(steelhead_vision SHARED
  src/object_detector.cpp
  src/contour_features.cpp
)

target_link_libraries(steelhead_vision PUBLIC ${OpenCV_LIBRARIES})

target_include_directories(steelhead_vision
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_targets(export_steelhead_vision HAS_LIBRARY_TARGET)
ament_export_dependencies("OpenCV")

# Install Python module package
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  steelhead_vision/yolo_detector.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_export_dependencies(ament_cmake_python)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS steelhead_vision
  EXPORT export_steelhead_vision
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Package configuration
ament_package()
