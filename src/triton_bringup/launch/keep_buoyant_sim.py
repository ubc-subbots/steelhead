# this stuff probably doesn't work so can remove after comp
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Simple launch file that includes the keep buoyant navigator simulation
    """
    ld = LaunchDescription()

    # Include the full simulation to keep buyant launch from triton_gazebo
    straight_line_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_gazebo'), 'launch', 'keep_buoyant_launch.py')
        )
    )

    ld.add_action(straight_line_sim)

    return ld