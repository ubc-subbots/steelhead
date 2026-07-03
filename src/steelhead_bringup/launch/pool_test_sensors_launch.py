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

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'imu_publisher_launch.py')
        )
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'cameras_publisher_launch.py')
        )
    )

    actuators_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('steelhead_controls') + '/launch/actuators_command_server_launch.py'
        )
    )

    depth_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('steelhead_controls') + '/launch/depth_sensor_publisher_launch.py'
        )
    )

    ld.add_action(imu)
    ld.add_action(cameras)
    ld.add_action(actuators_server)
    ld.add_action(depth_sensor)
    
    return ld
