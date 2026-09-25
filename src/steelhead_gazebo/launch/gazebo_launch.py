import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    world_arg = DeclareLaunchArgument(
        "world", default_value="cube.world", description="Gazebo world file"
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Set to 'true' to run gazebo headless",
    )

    # We need to add the models and worlds directories to env so gazebo can find them
    steelhead_gazebo_dir = get_package_share_directory("steelhead_gazebo")

    steelhead_gazebo_prefix = get_package_prefix("steelhead_gazebo")
    add_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
            os.pathsep + os.path.join(steelhead_gazebo_prefix, "lib"),
        ],
    )

    grp = "GZ_SIM_RESOURCE_PATH"
    add_resource_path = SetEnvironmentVariable(
        name=grp,
        value=[
            EnvironmentVariable(grp, default_value=""),
            os.pathsep + os.path.join(steelhead_gazebo_dir, "gazebo"),
            os.pathsep + os.path.join(steelhead_gazebo_dir, "gazebo", "models"),
        ],
    )

    gz_args = PythonExpression(
        [
            '" -r -v 1 "',
            ' + ("-s " if "',
            LaunchConfiguration("headless"),
            '" == "true" else "")',
            ' + " worlds/" + "',
            LaunchConfiguration("world"),
            '"',
        ]
    )

    gazebo_sim = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/base_link/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/front_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/front_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/bottom_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/bottom_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/world/competition/create@ros_gz_interfaces/srv/SpawnEntity",
        ],
        remappings=[
            ("/model/base_link/odometry", "/steelhead/state"),
            ("/imu", "/steelhead/drivers/imu/out"),
            ("/front_camera/image", "/steelhead/drivers/front_camera/image_raw"),
            (
                "/front_camera/depth_image",
                "/steelhead/drivers/front_camera/depth/image_raw",
            ),
            (
                "/front_camera/camera_info",
                "/steelhead/drivers/front_camera/camera_info",
            ),
            ("/front_camera/points", "/steelhead/drivers/front_camera/points"),
            ("/bottom_camera/image", "/steelhead/drivers/bottom_camera/image_raw"),
            (
                "/bottom_camera/camera_info",
                "/steelhead/drivers/bottom_camera/camera_info",
            ),
        ],
        output="screen",
        # ros_arguments=["--log-level", "WARN"],
    )

    ld.add_action(world_arg)
    ld.add_action(headless_arg)
    ld.add_action(add_resource_path)
    ld.add_action(add_plugin_path)
    ld.add_action(gazebo_sim)
    ld.add_action(bridge)
    return ld
