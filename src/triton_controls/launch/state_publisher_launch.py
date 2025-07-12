
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    pkg_share = get_package_share_directory('triton_controls')
    sdf_file =  os.path.join(pkg_share, 'models', 'triton_auv', 'model.urdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    state_publisher = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        output='screen', 
        parameters=[rsp_params, {'use_sim_time': True}]
    )

    transform_publisher = Node(
        package='triton_controls',
        executable='auv_transform_publisher.py',
        name='auv_transform_publisher',
        output='screen', 
        parameters=[rsp_params, {'use_sim_time': True}]
    )

    ld.add_action(state_publisher)
    ld.add_action(transform_publisher)

    return ld