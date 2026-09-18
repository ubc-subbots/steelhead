cmake_minimum_required(VERSION 3.10)
project(steelhead_gazebo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3 -DNDEBUG")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gz-sim REQUIRED)
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
find_package(std_msgs REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

# ground_truth_sensor
# add_library(ground_truth_sensor SHARED src/ground_truth_sensor.cpp)
# target_include_directories(ground_truth_sensor PUBLIC include)
# target_link_libraries(ground_truth_sensor geometry_msgs::geometry_msgs)
# ament_export_libraries(ground_truth_sensor)
# install(TARGETS ground_truth_sensor
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# # pressure_sensor
# add_library(pressure_sensor SHARED src/pressure_sensor.cpp)
# target_include_directories(pressure_sensor PUBLIC include)
# target_link_libraries(pressure_sensor steelhead_interfaces::steelhead_interfaces geometry_msgs::geometry_msgs)
# ament_export_libraries(pressure_sensor)
# install(TARGETS pressure_sensor
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# thruster_driver_plugin
add_library(thruster_driver_plugin SHARED src/thruster_driver_plugin.cpp)
target_include_directories(thruster_driver_plugin PUBLIC 
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)
target_link_libraries(thruster_driver_plugin
  gz-sim
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)
ament_export_libraries(thruster_driver_plugin)
install(TARGETS thruster_driver_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# # underwater_camera
# add_executable(underwater_camera src/underwater_camera.cpp)
# target_include_directories(underwater_camera PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# target_link_libraries(underwater_camera
#   "image_transport" "sensor_msgs" "message_filters" "rclcpp"
#   "OpenCV" "cv_bridge" "steelhead_interfaces"
# )
# install(TARGETS underwater_camera DESTINATION lib/${PROJECT_NAME})

# steelhead_gazebo
add_library(steelhead_gazebo SHARED src/underwater_camera.cpp)
target_compile_definitions(steelhead_gazebo PRIVATE "steelhead_gazebo_CPP_BUILDING_DLL")
target_link_libraries(steelhead_gazebo ${OpenCV_LIBRARIES} cv_bridge::cv_bridge image_transport::image_transport sensor_msgs::sensor_msgs message_filters::message_filters steelhead_interfaces::steelhead_interfaces geometry_msgs::geometry_msgs)
rclcpp_components_register_nodes(steelhead_gazebo "steelhead_gazebo::UnderwaterCamera")
install(TARGETS steelhead_gazebo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# # torpedo_plugin
# add_library(torpedo_plugin SHARED src/torpedo_plugin.cpp)
# target_link_libraries(torpedo_plugin rclcpp::rclcpp)
# install(TARGETS torpedo_plugin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# # hydrodynamics_plugin
# add_library(hydrodynamics_plugin SHARED
#   src/hydrodynamics_plugin.cpp
#   gazebo_utils/src/gazebo_utils.cpp
#   math_utils/src/math_utils.cpp
# )
# target_include_directories(hydrodynamics_plugin PUBLIC include gazebo_utils math_utils)
# ament_export_libraries(hydrodynamics_plugin)
# install(TARGETS hydrodynamics_plugin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# # actuators_command_simulation
# add_executable(actuators_command_simulation src/actuators_command_simulation.cpp)
# target_include_directories(actuators_command_simulation PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# target_link_libraries(actuators_command_simulation
#   "rclcpp" "rclcpp_components" "steelhead_interfaces"
# )
# install(TARGETS actuators_command_simulation DESTINATION lib/${PROJECT_NAME})

# smooth_camera_plugin - deprecated since classic
# add_library(smooth_camera_plugin SHARED src/smooth_camera_plugin.cpp)
# target_include_directories(smooth_camera_plugin PUBLIC include)
# target_link_libraries(smooth_camera_plugin steelhead_interfaces::steelhead_interfaces)
# ament_export_libraries(smooth_camera_plugin)
# install(TARGETS smooth_camera_plugin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# camera_orbit_plugin - deprecated since classic
# add_library(camera_orbit_plugin SHARED src/camera_orbit_plugin.cpp)
# target_include_directories(camera_orbit_plugin PUBLIC include)
# target_link_libraries(camera_orbit_plugin steelhead_interfaces::steelhead_interfaces)
# ament_export_libraries(camera_orbit_plugin)
# install(TARGETS camera_orbit_plugin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# random_camera_plugin - deprecated since classic
# add_library(random_camera_plugin SHARED src/random_camera_plugin.cpp)
# target_include_directories(random_camera_plugin PUBLIC include)
# target_link_libraries(random_camera_plugin steelhead_interfaces::steelhead_interfaces)
# ament_export_libraries(random_camera_plugin)
# install(TARGETS random_camera_plugin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# bounding_box_plugin - deprecated since classic
# add_library(bounding_box_plugin SHARED src/bounding_box_plugin.cpp)
# target_include_directories(bounding_box_plugin PUBLIC include)
# target_link_libraries(bounding_box_plugin steelhead_interfaces::steelhead_interfaces)
# ament_export_libraries(bounding_box_plugin)
# install(TARGETS bounding_box_plugin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

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
