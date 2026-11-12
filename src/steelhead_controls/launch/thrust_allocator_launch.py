from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    config_arg = DeclareLaunchArgument(
        "config_file", default_value="steelhead_thruster_config.yaml"
    )

    config = PathJoinSubstitution(
        [
            FindPackageShare("steelhead_controls"),
            "config",
            LaunchConfiguration("config_file"),
        ]
    )

    thrust_allocator = Node(
        name="thrust_allocator",
        namespace="/steelhead/controls",
        package="steelhead_controls",
        executable="thrust_allocator",
        output="screen",
        parameters=[config],
        remappings=[("/steelhead/controls/signals", "/motor_control")],
    )

    ld.add_action(config_arg)
    ld.add_action(thrust_allocator)

    return ld
