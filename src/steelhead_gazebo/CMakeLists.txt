cmake_minimum_required(VERSION 3.5)
project(steelhead_gazebo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ros_gz_sim REQUIRED)
set(CMAKE_PREFIX_PATH "/opt/ros/lyrical/opt/gz_msgs_vendor;/opt/ros/lyrical/opt/gz_transport_vendor;/opt/ros/lyrical/opt/gz_math_vendor;/opt/ros/lyrical/opt/gz_sim_vendor;/opt/ros/lyrical/opt/gz_common_vendor;/opt/ros/lyrical/opt/gz_plugin_vendor;/opt/ros/lyrical/opt/gz_utils_vendor;${CMAKE_PREFIX_PATH}")
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(steelhead_interfaces REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(underwater_camera 
  src/underwater_camera.cpp
)
target_include_directories(underwater_camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(underwater_camera PUBLIC image_transport::image_transport sensor_msgs::sensor_msgs message_filters::message_filters rclcpp::rclcpp rclcpp_components::component ${OpenCV_LIBRARIES} cv_bridge::cv_bridge steelhead_interfaces::steelhead_interfaces)

install(TARGETS 
  underwater_camera
  DESTINATION lib/${PROJECT_NAME}
)

add_library(steelhead_gazebo SHARED
  src/underwater_camera.cpp
)

rclcpp_components_register_nodes(steelhead_gazebo "steelhead_gazebo::UnderwaterCamera")

target_compile_definitions(steelhead_gazebo
  PRIVATE "steelhead_gazebo_CPP_BUILDING_DLL"
)

target_link_libraries(steelhead_gazebo PUBLIC rclcpp::rclcpp rclcpp_components::component image_transport::image_transport sensor_msgs::sensor_msgs message_filters::message_filters steelhead_interfaces::steelhead_interfaces geometry_msgs::geometry_msgs cv_bridge::cv_bridge ${OpenCV_LIBRARIES})

add_executable(actuators_command_simulation
  src/actuators_command_simulation.cpp
)
target_include_directories(actuators_command_simulation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(actuators_command_simulation PUBLIC rclcpp::rclcpp rclcpp_components::component steelhead_interfaces::steelhead_interfaces ros_gz_interfaces::ros_gz_interfaces ament_index_cpp::ament_index_cpp)

install(TARGETS actuators_command_simulation
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  steelhead_gazebo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/gazebo
)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/gazebo
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  steelhead_gazebo/bounding_box_image_saver.py
  steelhead_gazebo/train_yolo.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_export_dependencies(ament_cmake_python)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    test/test_gazebo_launch.py
    TARGET test_gazebo_launch_default_world
    ARGS "headless:=true"
  )
    
  add_launch_test(
    test/test_gazebo_launch.py
    TARGET test_gazebo_launch_cube_world
    ARGS "headless:=true" "world:=cube.world"
  )

  add_launch_test(
    test/test_underwater_camera_launch.py
    TARGET test_underwater_camera_launch
  )

endif()

ament_package()
