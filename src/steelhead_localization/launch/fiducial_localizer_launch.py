from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # Picks config/fiducial_symbols_<symbols>.yaml, e.g. symbols:=torpedo
    symbols_arg = DeclareLaunchArgument("symbols", default_value="gate")
    # Sim launches must pass use_sim_time:=true; defaults to false for the real vehicle
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
