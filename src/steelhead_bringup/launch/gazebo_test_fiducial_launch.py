import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    world_arg = DeclareLaunchArgument(
        "world", default_value="competition_task_gate.world"
    )
    # Must match the world, e.g. world:=competition_task_torpedo.world symbols:=torpedo
    symbols_arg = DeclareLaunchArgument("symbols", default_value="gate")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_gazebo"),
                "launch",
                "gazebo_launch.py",
            )
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
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

    # hold_yaw is off so the robot can be turned with ssh_keyboard_pid_teleop
    hover = Node(
        package="steelhead_controls",
        executable="hover_at_depth",
        parameters=[{"depth": 0.75, "hold_yaw": False}],
        namespace="steelhead",
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

    yolo_detector_front = Node(
        package="steelhead_vision",
        executable="yolo_detector.py",
        name="yolo_detector_front",
        parameters=[
            {
                "weights_path": os.path.join(
                    get_package_share_directory("steelhead_vision"),
                    "config",
                    "ffc_rs_26.pt",
                )
            },
            {"inference_interval": 0.2},
            {"input_image_topic": "/steelhead/drivers/front_camera/image_raw"},
            {"annotated_image_topic": "/steelhead/yolo_detector/front/annotated_image"},
            {"detections_topic": "/steelhead/yolo_detector/front/detections"},
        ],
        output="screen",
    )

    fiducial_localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_localization"),
                "launch",
                "fiducial_localizer_launch.py",
            )
        ),
        launch_arguments={
            "symbols": LaunchConfiguration("symbols"),
            "use_sim_time": "true",
        }.items(),
    )

    ld.add_action(world_arg)
    ld.add_action(symbols_arg)
    ld.add_action(gazebo)
    ld.add_action(thrust_allocator)
    ld.add_action(hover)
    ld.add_action(pid_controller)
    ld.add_action(yolo_detector_front)
    ld.add_action(fiducial_localizer)

    return ld
