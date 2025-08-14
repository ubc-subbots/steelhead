from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Simple launch file that includes the full straight line navigator simulation
    """
    ld = LaunchDescription()

    # Include the full simulation launch from triton_gazebo
    straight_line_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_gazebo'), 'launch', 'straight_line_navigator_sim_launch.py')
        )
    )

    ld.add_action(straight_line_sim)

    return ld