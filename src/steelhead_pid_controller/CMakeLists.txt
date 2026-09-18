cmake_minimum_required(VERSION 3.10)
project(steelhead_pid_controller)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(steelhead_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
#target_link_libraries(steelhead_pid_controller ${YAML_CPP_LIBRARIES})
add_executable(steelhead_pid_controller src/steelhead_pid_controller.cpp)

target_link_libraries(steelhead_pid_controller ${YAML_CPP_LIBRARIES})
target_include_directories(steelhead_pid_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS steelhead_pid_controller
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(steelhead_pid_controller PUBLIC rclcpp::rclcpp rclcpp_components::component sensor_msgs::sensor_msgs std_msgs::std_msgs steelhead_interfaces::steelhead_interfaces tf2::tf2 tf2_geometry_msgs::tf2_geometry_msgs ament_index_cpp::ament_index_cpp)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
