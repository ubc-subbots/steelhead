cmake_minimum_required(VERSION 3.5)
project(steelhead_gazebo)

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3 -DNDEBUG")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(steelhead_interfaces REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(ground_truth_sensor SHARED
  src/ground_truth_sensor.cpp
)
target_include_directories(ground_truth_sensor PUBLIC
  include
)
ament_target_dependencies(ground_truth_sensor
  "gazebo_dev"
  "gazebo_ros"
)
ament_export_libraries(ground_truth_sensor)
target_link_libraries(
  ground_truth_sensor
  ${geometry_msgs_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
)

install(TARGETS
  ground_truth_sensor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(thruster_driver_plugin SHARED
  src/thruster_driver_plugin.cpp
)
target_include_directories(thruster_driver_plugin PUBLIC
  include
)
ament_target_dependencies(thruster_driver_plugin
  "gazebo_dev"
  "gazebo_ros"
)
ament_export_libraries(thruster_driver_plugin)
target_link_libraries(
  thruster_driver_plugin 
  ${GAZEBO_LIBRARIES}
)

install(TARGETS
  thruster_driver_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(underwater_camera 
  src/underwater_camera.cpp
)
target_include_directories(underwater_camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(underwater_camera
  "image_transport"
  "sensor_msgs"
  "message_filters"
  "rclcpp"
  "OpenCV"
  "cv_bridge"
  "steelhead_interfaces"
)

install(TARGETS 
  underwater_camera
  DESTINATION lib/${PROJECT_NAME}
)

add_library(steelhead_gazebo SHARED
  src/underwater_camera.cpp
)

add_library(bounding_box_plugin SHARED src/bounding_box_plugin.cpp)
target_include_directories(bounding_box_plugin PUBLIC
  include
)
target_link_libraries(bounding_box_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(bounding_box_plugin
  "steelhead_interfaces"
)
ament_export_libraries(bounding_box_plugin)
target_link_libraries(bounding_box_plugin ${GAZEBO_LIBRARIES})


add_library(camera_orbit_plugin SHARED src/camera_orbit_plugin.cpp)
target_include_directories(camera_orbit_plugin PUBLIC
  include
)
target_link_libraries(camera_orbit_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(camera_orbit_plugin
  "steelhead_interfaces"
)
ament_export_libraries(camera_orbit_plugin)
target_link_libraries(camera_orbit_plugin ${GAZEBO_LIBRARIES})

add_library(random_camera_plugin SHARED src/random_camera_plugin.cpp)
target_include_directories(random_camera_plugin PUBLIC
  include
)
target_link_libraries(random_camera_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(random_camera_plugin
  "steelhead_interfaces"
)
ament_export_libraries(random_camera_plugin)
target_link_libraries(random_camera_plugin ${GAZEBO_LIBRARIES})

add_library(smooth_camera_plugin SHARED src/smooth_camera_plugin.cpp)
target_include_directories(smooth_camera_plugin PUBLIC
  include
)
target_link_libraries(smooth_camera_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(smooth_camera_plugin
  "steelhead_interfaces"
)
ament_export_libraries(smooth_camera_plugin)
target_link_libraries(smooth_camera_plugin ${GAZEBO_LIBRARIES})

rclcpp_components_register_nodes(steelhead_gazebo "steelhead_gazebo::UnderwaterCamera")

target_compile_definitions(steelhead_gazebo
  PRIVATE "steelhead_gazebo_CPP_BUILDING_DLL"
)

ament_target_dependencies(steelhead_gazebo
  "rclcpp"
  "rclcpp_components"
  "image_transport"
  "sensor_msgs"
  "message_filters"
  "steelhead_interfaces"
  "geometry_msgs"
)

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(hydrodynamics_plugin SHARED
  src/hydrodynamics_plugin.cpp
  gazebo_utils/src/gazebo_utils.cpp
  math_utils/src/math_utils.cpp
)
target_include_directories(hydrodynamics_plugin PUBLIC
  include
  gazebo_utils
  math_utils
)
ament_target_dependencies(hydrodynamics_plugin
  "gazebo_dev"
  "gazebo_ros"
)
ament_export_libraries(hydrodynamics_plugin)
target_link_libraries(hydrodynamics_plugin ${GAZEBO_LIBRARIES})

add_executable(actuators_command_simulation
  src/actuators_command_simulation.cpp
)
target_include_directories(actuators_command_simulation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  actuators_command_simulation
  "rclcpp"
  "rclcpp_components"
  "steelhead_interfaces"
  "gazebo_msgs"
)

install(TARGETS actuators_command_simulation
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  hydrodynamics_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  steelhead_gazebo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  bounding_box_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  camera_orbit_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  random_camera_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(TARGETS
  smooth_camera_plugin
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
