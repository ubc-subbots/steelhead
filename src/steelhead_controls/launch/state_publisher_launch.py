from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="steelhead_auv",
        description="URDF model name (without .urdf) within steelhead_controls/models",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true",
    )

    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("steelhead_controls"),
            "models",
            [LaunchConfiguration("model"), ".urdf"],
        ]
    )

    robot_desc = ParameterValue(Command(["cat ", urdf_path]), value_type=str)

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )
            },
        ],
    )

    ld.add_action(model_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(state_publisher)

    return ld
