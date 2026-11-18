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

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_sensors"),
                "launch",
                "cameras_publisher_launch.py",
            )
        )
    )

    gate_detector = ComposableNode(
        name="detector",
        namespace="/steelhead/gate",
        package="steelhead_vision",
        plugin="steelhead_vision::GateDetector",
        parameters=[{"debug": False}],
    )

    gate_container = ComposableNodeContainer(
        name="gate_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[gate_detector],
        output="screen",
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

    waypoint_marker = Node(
        package="steelhead_controls",
        executable="waypoint_marker",
        output="screen",
        # parameters=[{'use_sim_time': True}]
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

    trajectory_generator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("steelhead_controls")
            + "/launch/trajectory_generator_launch.py"
        )
    )

    actuators_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("steelhead_controls")
            + "/launch/actuators_command_server_launch.py"
        )
    )

    depth_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("steelhead_sensors")
            + "/launch/depth_sensor_publisher_launch.py"
        )
    )

    hover_at_depth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("steelhead_controls")
            + "/launch/hover_at_depth_launch.py"
        )
    )

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

    delayed_mission = TimerAction(period=5.0, actions=[mission_runner])

    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_controls"),
                "launch",
                "state_publisher_launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
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
    ld.add_action(state_publisher)
    ld.add_action(serial)
    # ld.add_action(micro_ros_agent)
    # ld.add_action(state_estimator)
    # ld.add_action(imu_tf)
    ld.add_action(pid_controller)
    # ld.add_action(waypoint_marker)
    ld.add_action(thrust_allocator)
    # ld.add_action(gate_container)
    # ld.add_action(trajectory_generator)
    # ld.add_action(keyboard_teleop)
    ld.add_action(actuators_server)
    ld.add_action(depth_sensor)
    ld.add_action(hover_at_depth)
    # ld.add_action(delayed_mission)

    return ld
