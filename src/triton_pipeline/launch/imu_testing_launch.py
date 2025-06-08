from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ld = LaunchDescription()

    trajectory_generator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('triton_controls') + '/launch/trajectory_generator_launch.py'
        )
    )

    state_estimator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_controls'), 'launch', 'state_estimator_launch.py')
        )
    )

    imu_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       # arguments = "0 0 0 0 1.57079 0 imu_link base_link".split(" "))
                       arguments = "0 0 0 0 0 0 imu_link base_link".split(" "))


    ld.add_action(state_estimator)
    ld.add_action(trajectory_generator) 
    ld.add_action(imu_tf)


    return ld
