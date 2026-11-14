from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission_arg = DeclareLaunchArgument(
        "mission",
        default_value="gate_only.xml",
        description="Behavior tree XML file in steelhead_mission/missions or absolute path",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    groot_port_arg = DeclareLaunchArgument(
        "groot_port",
        default_value="1667",
        description="TCP port for Groot2 live monitoring (0 to disable)",
    )

    mission_runner = Node(
        package="steelhead_mission",
        executable="mission_runner",
        name="mission_runner",
        namespace="/steelhead",
        output="screen",
        parameters=[
            {
                "mission_file": LaunchConfiguration("mission"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "groot_port": LaunchConfiguration("groot_port"),
            }
        ],
    )

    return LaunchDescription(
        [
            mission_arg,
            use_sim_time_arg,
            groot_port_arg,
            mission_runner,
        ]
    )
