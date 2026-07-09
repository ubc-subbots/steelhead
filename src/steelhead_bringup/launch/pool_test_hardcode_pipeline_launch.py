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
                package="steelhead_controls",
                plugin="steelhead_controls::SerialSubscriber",
                name="serial_subscriber",
            ),
        ],
        output="both",
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_controls"),
                "launch",
                "imu_publisher_launch.py",
            )
        )
    )

    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_pid_controller"),
                "launch",
                "steelhead_pid_controller_launch.py",
            )
        )
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

    pressure_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_controls"),
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

    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_pipeline"),
                "launch",
                "pipeline_launch.py",
            )
        ),
        launch_arguments={"sequence": "competition_hardcode_sequence.yaml"}.items(),
    )

    delayed_pipeline = TimerAction(period=15.0, actions=[pipeline])

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
    ld.add_action(
        delay_hover
    )  # delay starting the hoverscript to let the imu calibrate. During this time, should move around the robot in figure 8s
    ld.add_action(
        delayed_pipeline
    )  # let the hover script reach depth before starting the sequence

    return ld
