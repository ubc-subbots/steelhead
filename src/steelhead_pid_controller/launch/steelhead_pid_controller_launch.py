from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ld = LaunchDescription()

    # Sim launches must pass use_sim_time:=true; defaults to false for the real vehicle
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    config = os.path.join(
        get_package_share_directory('steelhead_pid_controller'),
        'config',
        'pid.yaml'
    )

    pid_controller = Node(
        package='steelhead_pid_controller',
        namespace='/steelhead/controls',
        executable='steelhead_pid_controller',
        name='steelhead_pid_controller',
        output='screen',
        parameters=[
            config,
            {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
        ]
    )

    ld.add_action(use_sim_time_arg)
    ld.add_action(pid_controller)

    return ld
