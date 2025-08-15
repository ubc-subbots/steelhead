from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch file for hover underwater pool test
    """
    ld = LaunchDescription()

    # Include the hover underwater launch from triton_controls
    hover_underwater = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_controls'), 'launch', 'hover_underwater_launch.py')
        )
    )

    ld.add_action(hover_underwater)

    return ld