cmake_minimum_required(VERSION 3.10)
project(steelhead_sensors)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(steelhead_sensors SHARED src/serial_subscriber.cpp)
target_include_directories(steelhead_sensors PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(steelhead_sensors PRIVATE "STEELHEAD_SENSORS_CPP_BUILDING_DLL")
target_link_libraries(steelhead_sensors PUBLIC rclcpp::rclcpp rclcpp_components::component std_msgs::std_msgs)
rclcpp_components_register_node(steelhead_sensors PLUGIN "steelhead_sensors::SerialSubscriber" EXECUTABLE serial_subscriber_node)

install(TARGETS steelhead_sensors
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config arduino DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  steelhead_sensors/bno085_imu_publisher.py
  steelhead_sensors/ms5837_depth_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
