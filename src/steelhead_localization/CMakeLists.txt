cmake_minimum_required(VERSION 3.5)
project(steelhead_localization)

# ---------------------------------------------------------
# 1. COMPILER & GLOBAL SETTINGS
# ---------------------------------------------------------
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-Wextra -Wpedantic)

# Ensure CMake can find the custom FindEigen.cmake if needed
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_model/cmake")

# ---------------------------------------------------------
# 2. FIND DEPENDENCIES
# ---------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV 4 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

# ---------------------------------------------------------
# 3. INCLUDE DIRECTORIES
# ---------------------------------------------------------
include_directories(
    include
    src/camera_model/include
    src/feature_tracker/src
    src/vins_estimator/src
    ${CERES_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# ---------------------------------------------------------
# 4. TARGET: camera_model (Shared Library)
# ---------------------------------------------------------
set(CAMERA_MODEL_SRC
    src/camera_model/src/chessboard/Chessboard.cc
    src/camera_model/src/calib/CameraCalibration.cc
    src/camera_model/src/camera_models/Camera.cc
    src/camera_model/src/camera_models/CameraFactory.cc
    src/camera_model/src/camera_models/CostFunctionFactory.cc
    src/camera_model/src/camera_models/PinholeCamera.cc
    src/camera_model/src/camera_models/CataCamera.cc
    src/camera_model/src/camera_models/EquidistantCamera.cc
    src/camera_model/src/camera_models/ScaramuzzaCamera.cc
    src/camera_model/src/sparse_graph/Transform.cc
    src/camera_model/src/gpl/gpl.cc
    src/camera_model/src/gpl/EigenQuaternionParameterization.cc
)

add_library(camera_model SHARED ${CAMERA_MODEL_SRC})
target_link_libraries(camera_model 
    ${Boost_LIBRARIES} 
    ${OpenCV_LIBS} 
    ${CERES_LIBRARIES}
)

# ---------------------------------------------------------
# 5. TARGET: Calibration (Executable)
# ---------------------------------------------------------
add_executable(Calibration 
    src/camera_model/src/intrinsic_calib.cc
)
target_link_libraries(Calibration camera_model)

# ---------------------------------------------------------
# 6. TARGET: feature_tracker (Executable/Node)
# ---------------------------------------------------------
add_executable(feature_tracker
    src/feature_tracker/src/feature_tracker_node.cpp
    src/feature_tracker/src/parameters.cpp
    src/feature_tracker/src/feature_tracker.cpp
)
ament_target_dependencies(feature_tracker 
    rclcpp rcpputils std_msgs sensor_msgs geometry_msgs cv_bridge image_transport
)
target_link_libraries(feature_tracker camera_model ${OpenCV_LIBS})

# ---------------------------------------------------------
# 7. TARGET: vins_estimator (Executable/Node)
# ---------------------------------------------------------
add_executable(vins_estimator
    src/vins_estimator/src/estimator_node.cpp
    src/vins_estimator/src/parameters.cpp
    src/vins_estimator/src/estimator.cpp
    src/vins_estimator/src/feature_manager.cpp
    src/vins_estimator/src/factor/pose_local_parameterization.cpp
    src/vins_estimator/src/factor/projection_factor.cpp
    src/vins_estimator/src/factor/projection_td_factor.cpp
    src/vins_estimator/src/factor/marginalization_factor.cpp
    src/vins_estimator/src/utility/utility.cpp
    src/vins_estimator/src/utility/visualization.cpp
    src/vins_estimator/src/utility/CameraPoseVisualization.cpp
    src/vins_estimator/src/initial/solve_5pts.cpp
    src/vins_estimator/src/initial/initial_aligment.cpp
    src/vins_estimator/src/initial/initial_sfm.cpp
    src/vins_estimator/src/initial/initial_ex_rotation.cpp
)
ament_target_dependencies(vins_estimator 
    rclcpp rcpputils std_msgs sensor_msgs visualization_msgs 
    geometry_msgs nav_msgs tf2 tf2_ros cv_bridge image_transport
)
target_link_libraries(vins_estimator camera_model ${OpenCV_LIBS} ${CERES_LIBRARIES})

# ---------------------------------------------------------
# 8. INSTALLATION & EXPORTS
# ---------------------------------------------------------

# Export info for downstream packages
ament_export_include_directories(include src/camera_model/include)
ament_export_libraries(camera_model)
ament_export_dependencies(std_msgs rclcpp OpenCV Ceres)

# Install targets
install(TARGETS camera_model Calibration feature_tracker vins_estimator
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install specific directories
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include/
)

ament_package()