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
        launch_arguments={"world": "competition_task_slaloms.world"}.items(),
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

    nav_node = Node(
        package="steelhead_tasks",
        executable="automated_competition_slaloms.py",
        name="automated_competition_slaloms",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    hover = Node(
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

    rviz_config_file = os.path.join(
        get_package_share_directory("steelhead_bringup"),
        "config",
        "gazebo_nav_test.rviz",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        # Silence log spam
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "WARN"],
        parameters=[{"use_sim_time": True}],
    )

    ld.add_action(gazebo)
    ld.add_action(thrust_allocator)
    ld.add_action(nav_node)
    ld.add_action(hover)
    ld.add_action(pid_controller)
    ld.add_action(rviz)

    return ld
