from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Starts the fiducial localizer with the chosen symbol file
def generate_launch_description():

    ld = LaunchDescription()

    symbols_arg = DeclareLaunchArgument("symbols", default_value="gate")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    symbols_file = PathJoinSubstitution(
        [
            FindPackageShare("steelhead_localization"),
            "config",
            ["fiducial_symbols_", LaunchConfiguration("symbols"), ".yaml"],
        ]
    )

    fiducial_localizer = Node(
        name="fiducial_localizer",
        namespace="/steelhead/localization",
        package="steelhead_localization",
        executable="fiducial_localizer.py",
        output="screen",
        parameters=[
            {
                "symbols_file": symbols_file,
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }
        ],
    )

    ld.add_action(symbols_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(fiducial_localizer)

    return ld
