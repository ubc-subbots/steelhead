import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = LaunchDescription()

    serial = ComposableNodeContainer(
        name="serial_subscriber_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="steelhead_sensors",
                plugin="steelhead_sensors::SerialSubscriber",
                name="serial_subscriber",
            ),
        ],
        output="both",
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_sensors"),
                "launch",
                "imu_publisher_launch.py",
            )
        )
    )

    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_controls"),
                "launch",
                "pid_controller_launch.py",
            )
        )
    )

    thrust_allocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_controls"),
                "launch",
                "thrust_allocator_launch.py",
            )
        )
    )

    pressure_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_sensors"),
                "launch",
                "depth_sensor_publisher_launch.py",
            )
        )
    )

    hover = Node(
        package="steelhead_controls",
        executable="hover_at_depth",
        parameters=[{"depth": 1.0, "hold_yaw": True}],
        namespace="steelhead",
    )

    delay_hover = TimerAction(period=3.0, actions=[hover])

    mission_runner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_mission"),
                "launch",
                "mission_runner_launch.py",
            )
        ),
        launch_arguments={
            "mission": "competition_hardcode.xml",
            "use_sim_time": "false",
        }.items(),
    )

    delayed_mission = TimerAction(period=15.0, actions=[mission_runner])

    bag_record = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_bringup"),
                "launch",
                "bag_record_launch.py",
            )
        )
    )

    ld.add_action(bag_record)
    ld.add_action(serial)
    ld.add_action(imu)
    ld.add_action(pid_controller)
    ld.add_action(thrust_allocator)
    ld.add_action(pressure_sensor)
    # delay starting the hoverscript to let the imu calibrate. During this time, should move around the robot in figure 8s
    ld.add_action(delay_hover)
    # let the hover script reach depth before starting the sequence
    ld.add_action(delayed_mission)

    return ld
