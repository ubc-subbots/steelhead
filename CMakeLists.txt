cmake_minimum_required(VERSION 3.8)
project(gazebo)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# Install Python scripts
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/spawn_steelhead_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF and meshes
install(DIRECTORY steelhead_urdf steelhead_urdf/meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

