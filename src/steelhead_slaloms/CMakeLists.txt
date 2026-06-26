cmake_minimum_required(VERSION 3.5)
project(steelhead_slaloms)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(steelhead_vision_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(steelhead_interfaces REQUIRED)

include_directories(include)

add_library(steelhead_slaloms SHARED
  src/slalom_detector.cpp
)

rclcpp_components_register_nodes(steelhead_slaloms "steelhead_slaloms::SlalomDetector")

target_compile_definitions(steelhead_slaloms
  PRIVATE "steelhead_slaloms_CPP_BUILDING_DLL"
)

target_compile_options(steelhead_slaloms
  PRIVATE -Wno-vla
)

ament_target_dependencies(steelhead_slaloms
  "rclcpp"
  "rclcpp_components"
  "cv_bridge"
  "OpenCV"
  "steelhead_vision_utils"
  "sensor_msgs"
  "image_transport"
  "steelhead_interfaces"
)

install(TARGETS
  steelhead_slaloms
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
endif()

ament_package()