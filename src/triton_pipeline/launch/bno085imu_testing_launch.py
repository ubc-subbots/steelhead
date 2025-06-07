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
            os.path.join(get_package_share_directory('triton_pid_controller'), 'launch', 'triton_pid_controller_launch.py')
        )
    )

    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_controls'), 'triton_controls', 'bno085_imu_publisher.py')
        )
    )

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


    # state and transform publisher?

    #ld.add_action(pipeline_manager)
    #ld.add_action(pipeline_sequence_manager)
    #ld.add_action(pipeline_container)
    # ld.add_action(micro_ros_agent)
    ld.add_action(imu)
    ld.add_action(state_estimator)
    ld.add_action(pid_controller)
    ld.add_action(trajectory_generator) # we use key publisher instead
    #ld.add_action(record)


    return ld
