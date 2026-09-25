import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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

    ta_config = os.path.join(
        get_package_share_directory("steelhead_controls"),
        "config",
        "thruster_config.yaml",
    )

    thrust_allocator = Node(
        name="thrust_allocator",
        namespace="/steelhead/controls",
        package="steelhead_controls",
        executable="thrust_allocator",
        output="screen",
        parameters=[ta_config],
        remappings=[("/steelhead/controls/signals", "/motor_control")],
    )

    hardcoded_thrusters = Node(
        name="predetermined_thrust",
        namespace="/steelhead/controls",
        package="steelhead_controls",
        executable="predetermined_thrust.py",
    )

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
    ld.add_action(thrust_allocator)
    ld.add_action(hardcoded_thrusters)

    return ld
