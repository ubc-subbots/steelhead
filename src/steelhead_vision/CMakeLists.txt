cmake_minimum_required(VERSION 3.10)
project(steelhead_vision)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(steelhead_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_library(steelhead_vision SHARED 
  src/gate_detector.cpp
  src/object_detector.cpp
  src/contour_features.cpp
)
target_include_directories(steelhead_vision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(steelhead_vision PRIVATE "STEELHEAD_VISION_CPP_BUILDING_DLL")

target_link_libraries(steelhead_vision PUBLIC 
  rclcpp::rclcpp 
  rclcpp_components::component 
  std_msgs::std_msgs
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  steelhead_interfaces::steelhead_interfaces
  cv_bridge::cv_bridge
  image_transport::image_transport
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(steelhead_vision PLUGIN "steelhead_vision::GateDetector" EXECUTABLE gate_detector_node)

install(TARGETS steelhead_vision
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  steelhead_vision/yolo_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
