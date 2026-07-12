from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='steelhead_gazebo',
            executable='underwater_camera_effects',
            name='underwater_camera_effects',
            output='screen',
            parameters=[
                {'water_color_r': 0.06},
                {'water_color_g': 0.18},
                {'water_color_b': 0.32},
                {'attenuation_coefficient': 0.45},
                {'blur_strength': 2.2},
                {'max_depth': 8.0},
            ]
        )
    ])
