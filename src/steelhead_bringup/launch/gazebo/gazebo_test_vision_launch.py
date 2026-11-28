import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_gazebo"),
                "launch",
                "gazebo_launch.py",
            )
        ),
        launch_arguments={"world": "competition.world"}.items(),
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

    rviz_config_file = os.path.join(
        get_package_share_directory("steelhead_bringup"), "config", "gazebo_test.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
    )

    config = os.path.join(
        get_package_share_directory("steelhead_localization"),
        "config",
        "state_estimator_config.yaml",
    )

    state_estimator = Node(
        name="state_estimator",
        namespace="/steelhead/controls/ukf",
        package="robot_localization",
        executable="ukf_node",
        output="screen",
        parameters=[config, {"use_sim_time": True}],
    )

    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_controls"),
                "launch",
                "state_publisher_launch.py",
            )
        )
    )

    gate_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_vision"),
                "launch",
                "gate_detector_launch.py",
            )
        )
    )

    underwater_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("steelhead_gazebo")
            + "/launch/underwater_camera_launch.py"
        )
    )

    yolo_detector = Node(
        package="steelhead_vision",
        executable="yolo_detector.py",
        name="yolo_detector",
        parameters=[
            {
                "weights_path": os.path.join(
                    get_package_share_directory("steelhead_vision"),
                    "config",
                    "competition.pt",
                )
            },
            {"confidence_threshold": 0.4},
            {"inference_interval": 1.0},
        ],
        output="screen",
    )

    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_controls"),
                "launch",
                "pid_controller_launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    keyboard_pid_teleop = Node(
        name="keyboard_pid_teleop",
        namespace="/steelhead/teleop",
        package="steelhead_teleop",
        executable="keyboard_pid_teleop",
        output="screen",
    )

    hover_at_depth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_controls"),
                "launch",
                "hover_at_depth_launch.py",
            )
        )
    )

    ld.add_action(gazebo)
    ld.add_action(rviz)
    ld.add_action(thrust_allocator)
    ld.add_action(keyboard_pid_teleop)
    ld.add_action(hover_at_depth)
    ld.add_action(gate_detector)
    ld.add_action(state_publisher)
    # ld.add_action(underwater_camera) # the underwater camera simulator isn't that accurate and is very taxing on performance, so i'm disabling it for now
    ld.add_action(state_estimator)
    ld.add_action(yolo_detector)
    ld.add_action(pid_controller)

    return ld
