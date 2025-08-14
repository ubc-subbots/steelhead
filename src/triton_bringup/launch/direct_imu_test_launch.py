from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # Use the full simulation launch that includes robot spawning
    full_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_gazebo'), 'launch', 'straight_line_navigator_sim_launch.py')
        ),
        launch_arguments={
            'headless': 'false'
        }.items()
    )

    # Our direct IMU controller (will replace the normal straight line navigator)
    direct_controller = Node(
        package='triton_controls',
        executable='direct_imu_controller',
        name='direct_imu_controller_override',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ld.add_action(full_sim)
    # Start our controller after everything else is up (20 seconds)
    ld.add_action(TimerAction(period=20.0, actions=[direct_controller]))

    return ld