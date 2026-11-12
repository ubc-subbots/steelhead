import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    config_arg = DeclareLaunchArgument(
        "config_file", default_value="steelhead_pid_config.yaml"
    )

    config = PathJoinSubstitution(
        [
            FindPackageShare("steelhead_controls"),
            "config",
            LaunchConfiguration("config_file"),
        ]
    )

    steelhead_pid_controller = Node(
        package="steelhead_controls",
        namespace="/steelhead/controls",
        executable="pid_controller",
        name="pid_controller",
        output="screen",
        parameters=[
            config,
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )
            },
        ],
    )

    ld.add_action(config_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(steelhead_pid_controller)

    return ld
