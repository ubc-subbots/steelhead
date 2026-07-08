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

    keyboard_teleop = Node(
        name="keyboard_pid_teleop",
        namespace="/steelhead/teleop",
        package="steelhead_teleop",
        executable="keyboard_pid_teleop",
        output="screen",
    )

    gate_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_gate"),
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
        package="steelhead_object_recognition",
        executable="yolo_detector.py",
        name="yolo_detector",
        parameters=[
            {
                "weights_path": os.path.join(
                    get_package_share_directory("steelhead_object_recognition"),
                    "config",
                    "competition.pt",
                )
            },
            {"confidence_threshold": 0.4},
            {"inference_interval": 1.0},
        ],
        output="screen",
    )

    hover_script = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_controls"),
                "launch",
                "hover_at_depth_launch.py",
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
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # there's some small mis matches with our physical model and simulation which results in a bunch of errors
    # in the terminal despite it working as expected. this a bandaid fix that does nothing but stops the false errors
    base_link_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_tf_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_link::base_link"],
        parameters=[{"use_sim_time": True}],
    )

    ld.add_action(gazebo)
    ld.add_action(rviz)
    ld.add_action(thrust_allocator)
    ld.add_action(keyboard_teleop)
    ld.add_action(gate_detector)
    ld.add_action(state_publisher)
    # ld.add_action(underwater_camera) # the underwater camera simulator isn't that good and is very taxing on performance, so i'm disabling it for now
    ld.add_action(state_estimator)
    ld.add_action(yolo_detector)
    ld.add_action(hover_script)
    ld.add_action(pid_controller)
    ld.add_action(base_link_tf_publisher)

    return ld
