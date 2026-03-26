
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('steelhead_localization'),
        'config',
<<<<<<< HEAD:src/steelhead_controls/launch/state_estimator_launch.py
        'state_estimator_config_GT_only.yaml'
=======
        'state_estimator_config.yaml'
>>>>>>> main:src/steelhead_localization/launch/state_estimator_launch.py
    )

    state_estimator = Node(
        name='state_estimator',
        namespace='/steelhead/controls/ukf',
        package='robot_localization',
        executable='ukf_node',
        output='screen',
        parameters=[config, {'use_sim_time': False}]
    )

    ld.add_action(state_estimator)

    return ld