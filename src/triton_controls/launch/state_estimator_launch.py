
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('triton_controls'),
        'config',
        'state_estimator_config_IMU_only.yaml'
    )

    state_estimator = Node(
        name='state_estimator',
        namespace='/triton/controls/ukf',
        package='robot_localization',
        executable='ukf_node',
        output='screen',
        parameters=[config, {'use_sim_time': True}]
    )

    imu_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       # arguments = "0 0 0 0 1.57079 0 imu_link base_link".split(" "))
                       arguments = "0 0 0 0 0 0 imu_link base_link".split(" "))

    ld.add_action(state_estimator)
    ld.add_action(imu_tf)

    return ld