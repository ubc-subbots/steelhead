cmake_minimum_required(VERSION 3.10)
project(steelhead_controls)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(steelhead_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(thrust_allocator 
  src/thrust_allocator.cpp
)
target_include_directories(thrust_allocator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(waypoint_marker
  src/waypoint_marker.cpp
)
target_include_directories(waypoint_marker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(trajectory_generator
  src/trajectory_generator.cpp
)
target_include_directories(trajectory_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(actuators_command
  src/actuators_command.cpp
)
target_include_directories(actuators_command PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(hover_at_depth
  src/hover_at_depth.cpp
)
target_include_directories(hover_at_depth PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(thrust_allocator PUBLIC rclcpp::rclcpp rclcpp_components::component Eigen3::Eigen steelhead_interfaces::steelhead_interfaces geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs ${OpenCV_LIBRARIES})
install(TARGETS thrust_allocator
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(waypoint_marker PUBLIC rclcpp::rclcpp rclcpp_components::component tf2::tf2 tf2_geometry_msgs::tf2_geometry_msgs steelhead_interfaces::steelhead_interfaces geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs nav_msgs::nav_msgs)
install(TARGETS waypoint_marker
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(trajectory_generator PUBLIC rclcpp::rclcpp rclcpp_components::component tf2::tf2 tf2_geometry_msgs::tf2_geometry_msgs steelhead_interfaces::steelhead_interfaces geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs nav_msgs::nav_msgs)
install(TARGETS trajectory_generator
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(actuators_command PUBLIC rclcpp::rclcpp rclcpp_components::component steelhead_interfaces::steelhead_interfaces)
install(TARGETS actuators_command
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(hover_at_depth PUBLIC rclcpp::rclcpp rclcpp_components::component steelhead_interfaces::steelhead_interfaces geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2::tf2 tf2_geometry_msgs::tf2_geometry_msgs)
install(TARGETS hover_at_depth
  DESTINATION lib/${PROJECT_NAME}
)


add_library(steelhead_controls SHARED
  src/thrust_allocator.cpp
  # src/waypoint_marker.cpp
  
)

rclcpp_components_register_nodes(steelhead_controls "steelhead_controls::ThrustAllocator")
rclcpp_components_register_nodes(steelhead_controls "steelhead_controls::WaypointMarker")

rclcpp_components_register_nodes(steelhead_controls "steelhead_controls::TrajectoryGenerator")

target_compile_definitions(steelhead_controls
  PRIVATE "steelhead_controls_CPP_BUILDING_DLL"
)

target_link_libraries(steelhead_controls PUBLIC rclcpp::rclcpp rclcpp_components::component Eigen3::Eigen steelhead_interfaces::steelhead_interfaces geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs ${OpenCV_LIBRARIES} tf2::tf2 tf2_geometry_msgs::tf2_geometry_msgs nav_msgs::nav_msgs)

install(TARGETS
  steelhead_controls
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

 
install(DIRECTORY
  test/config
  DESTINATION share/${PROJECT_NAME}/test
)



# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  steelhead_controls/auv_transform_publisher.py
  steelhead_controls/waypoint_marker_tester.py
  steelhead_controls/predetermined_route.py
  steelhead_controls/predetermined_thrust.py
  
  
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_thrust_allocator_launch.py
  )
endif()


add_executable(pid_controller src/pid_controller.cpp)
target_include_directories(pid_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pid_controller PUBLIC 
  rclcpp::rclcpp 
  rclcpp_components::component 
  sensor_msgs::sensor_msgs 
  std_msgs::std_msgs 
  steelhead_interfaces::steelhead_interfaces 
  tf2::tf2 
  tf2_geometry_msgs::tf2_geometry_msgs 
  ament_index_cpp::ament_index_cpp)
install(TARGETS pid_controller DESTINATION lib/${PROJECT_NAME})

ament_package()
