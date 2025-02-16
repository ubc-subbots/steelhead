import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('triton_gazebo')
    sdf_file =  os.path.join(pkg_share, 'gazebo', 'models', 'triton_auv', 'model.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_gazebo'), 'launch', 'gazebo_launch.py')
        ),
        launch_arguments={'world': 'competition.world'}.items()
    )

    thrust_allocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_controls'), 'launch', 'thrust_allocator_launch.py')
        )
    )

    keyboard_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_teleop'), 'launch', 'keyboard_teleop_launch.py')
        )
    )

    # underwater_camera = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         get_package_share_directory('triton_gazebo') + '/launch/underwater_camera_launch.py'
    #     )
    # )

    ld.add_action(gazebo)
    ld.add_action(thrust_allocator)
    ld.add_action(keyboard_teleop)
    # ld.add_action(underwater_camera)

    return ld