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

    keyboard_teleop = Node(
        name="keyboard_pid_teleop",
        namespace="/steelhead/teleop",
        package="steelhead_teleop",
        executable="keyboard_pid_teleop",
        output="screen",
    )

    hover = Node(
        package="steelhead_controls",
        executable="hover_at_depth",
        parameters=[{"depth": 0.75, "hold_yaw": True}],
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

    rtab_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_localization"),
                "launch",
                "rtab_map_launch.py",
            )
        )
    )

    ld.add_action(gazebo)
    ld.add_action(thrust_allocator)
    ld.add_action(keyboard_teleop)
    ld.add_action(hover)
    ld.add_action(pid_controller)
    ld.add_action(rtab_slam)

    return ld
