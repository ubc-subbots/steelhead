import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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

    hover_script = Node(
        package="steelhead_controls",
        executable="hover_at_depth",
        parameters=[{"depth": 1.0, "hold_yaw": True}],
        namespace="steelhead",
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

    delayed_pipeline = TimerAction(period=5.0, actions=[pipeline])  # let gazebo spin up

    ld.add_action(gazebo)
    ld.add_action(thrust_allocator)
    ld.add_action(hover_script)
    ld.add_action(pid_controller)
    ld.add_action(delayed_pipeline)

    return ld
