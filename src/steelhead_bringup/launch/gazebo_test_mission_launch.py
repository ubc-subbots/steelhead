import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    mission_arg = DeclareLaunchArgument(
        "mission",
        default_value="gate_only.xml",
        description="Behavior tree XML file in steelhead_mission/missions or absolute path",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_gazebo"),
                "launch",
                "gazebo_launch.py",
            )
        ),
        launch_arguments={"world": "competition_task_gate.world"}.items(),
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
                get_package_share_directory("steelhead_controls"),
                "launch",
                "pid_controller_launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("steelhead_bringup"),
        "config",
        "gazebo_cv_test.rviz",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "WARN"],
        parameters=[{"use_sim_time": True}],
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
            "mission": LaunchConfiguration("mission"),
            "use_sim_time": "true",
        }.items(),
    )

    yolo_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("steelhead_vision"),
                "launch",
                "yolo_detector_launch.py",
            )
        )
    )

    ld.add_action(mission_arg)
    ld.add_action(gazebo)
    ld.add_action(thrust_allocator)
    ld.add_action(hover_script)
    ld.add_action(pid_controller)
    ld.add_action(rviz)
    ld.add_action(mission_runner)
    ld.add_action(yolo_detector)

    return ld
